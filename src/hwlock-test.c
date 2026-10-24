/*
 * hwlock-test.c - take the D1 hardware spinlock from the HiFi4 while Linux
 *                 holds and releases the same lock.
 *
 * TEST SCAFFOLDING. Not for submission. Pairs with drivers/misc/d1-hwlock-test.c
 * in the kernel tree; the shared structure layout is duplicated in both files
 * and must be kept in sync.
 *
 * The spinlock is a "read to take, write 0 to free" block at 0x03005000. The
 * HiFi4 has no atomic instruction (XCHAL_HAVE_S32C1I = 0, no exclusive loads),
 * so this block is the only way it can synchronise with the C906 at all.
 *
 * The shared block lives in DDR at 0x483ff000, inside the same reserved-memory
 * carveout as the firmware image. _cache_config() maps 0x40000000+ as
 * CA_BYPASS, so it is uncached and a lost update is a real locking failure
 * rather than a coherency artefact.
 *
 * Local DRAM1 (0x00440000) was tried first and does not work: once
 * DSP_BOOT_RAMMAP bit 0 is cleared the SRAM belongs to the DSP and the
 * system-side alias is gone, so the C906 hangs when it touches it.
 */

#include <stdint.h>
#include <stdio.h>

#include <xtensa/hal.h>

#include "FreeRTOS.h"
#include "task.h"

#define HWLOCK_BASE        0x03005000u
#define HWLOCK_LOCK0       (HWLOCK_BASE + 0x100u)

#define SHARED_BASE        0x483ff000u
#define SHARED_MAGIC       0x5a5a1234u
#define SHARED_VERSION     2u

/* Bump on every firmware change; printed at startup so the console identifies it */
#define FW_REV             6

/* Must match struct shared in drivers/misc/d1-hwlock-test.c */
struct shared {
    volatile uint32_t magic;
    volatile uint32_t version;
    volatile uint32_t go;
    volatile uint32_t stop;
    volatile uint32_t target;
    volatile uint32_t counter;
    volatile uint32_t inside;
    volatile uint32_t dsp_violations;
    volatile uint32_t dsp_iters;
    volatile uint32_t dsp_done;
    volatile uint32_t linux_violations;
    volatile uint32_t dsp_ping;      /* bumped while polling, proves liveness */
    volatile uint32_t dsp_bench_req; /* 1 = Linux asks for a DSP-side benchmark */
    volatile uint32_t dsp_bench_done;
};

static inline uint32_t rd(volatile uint32_t *p) { return *p; }
static inline void wr(volatile uint32_t *p, uint32_t v) { *p = v; }

/* Order accesses to the shared block against the lock operations */
static inline void hw_barrier(void) { __asm__ __volatile__("memw" ::: "memory"); }

/*
 * The whole struct is 56 bytes and SHARED_BASE is 4 KiB aligned, so it lives in
 * a single 64-byte D-cache line.
 *
 * _cache_config() asks for XCHAL_CA_BYPASS on 0x40000000+, but evidently that
 * does not take effect for this region (the DSP never saw Linux's writes). So
 * do the maintenance explicitly - it is a no-op if the region really is
 * uncached, and correct if it is not.
 */
#define SHARED_BYTES 64

static inline void sh_sync_in(void) {
    xthal_dcache_region_invalidate((void *) SHARED_BASE, SHARED_BYTES);
    hw_barrier();
}

static inline void sh_sync_out(void) {
    hw_barrier();
    xthal_dcache_region_writeback((void *) SHARED_BASE, SHARED_BYTES);
}

static inline void lock_take(void) {
    volatile uint32_t *reg = (volatile uint32_t *) HWLOCK_LOCK0;

    /* Reading a free lock takes it and returns 0; a taken one returns 1 */
    while (rd(reg) != 0)
        ;
}

static inline void lock_free(void) {
    volatile uint32_t *reg = (volatile uint32_t *) HWLOCK_LOCK0;

    wr(reg, 0);
}

static int run_round(volatile struct shared *sh) {
    uint32_t max, i, trips = 0;

    sh_sync_in();
    max = rd(&sh->target);
    printf("hwlock: go, max=%u\n", (unsigned) max);

    /*
     * Run until Linux says stop, bounded by the iteration cap. Linux's burst
     * length therefore sets the duration of the whole test.
     */
    for (i = 0; i < max; i++) {
        uint32_t cur;

        lock_take();

        /* Pull Linux's latest view while we own the lock */
        sh_sync_in();

        if (rd(&sh->inside))
            trips++;

        wr(&sh->inside, 1);

        /* Deliberately not atomic; the gap is what makes a race visible */
        cur = rd(&sh->counter);
        cur = rd(&sh->counter) + 1;
        wr(&sh->counter, cur);

        wr(&sh->inside, 0);

        /* Push our updates before dropping the lock */
        sh_sync_out();
        lock_free();

        if (!(i & 0x3ff)) {
            wr(&sh->dsp_iters, i + 1);
            sh_sync_out();

            sh_sync_in();
            if (rd(&sh->stop))
                break;
        }
    }

    wr(&sh->dsp_iters, i);
    wr(&sh->dsp_violations, trips);
    wr(&sh->dsp_done, 1);
    sh_sync_out();

    printf("hwlock: done, iters=%u trips=%u\n", (unsigned) i, (unsigned) trips);
    return 0;
}

/*
 * DSP-side benchmark: take and free the lock in a tight loop with Linux idle.
 * The iteration count arrives in 'target'. It is timed from the Linux side in
 * wall clock, so nothing has to be assumed about the CCOUNT tick rate.
 *
 * Deliberately silent: the DSP's UART is the same ttyS0 Linux prints on, and a
 * printf here lands in the middle of the kernel's own result lines.
 */
static void run_bench(volatile struct shared *sh) {
    uint32_t max = rd(&sh->target);
    uint32_t i;

    for (i = 0; i < max; i++) {
        lock_take();
        lock_free();
    }

    wr(&sh->dsp_bench_done, 1);
    sh_sync_out();
}

void hwlock_test_run(void) {
    volatile struct shared *sh = (volatile struct shared *) SHARED_BASE;
    char stack_top_marker;   /* its address proves where the stack lives */
    uint32_t ping = 0;

    /* Revision stamp: makes it obvious from the console which build is running */
    printf("hwlock: fw rev %d, shared @ %p, stack top %p\n", FW_REV,
           (void *) SHARED_BASE, (void *) &stack_top_marker);

    /*
     * Each module load re-arms the block: shared_reset() clears magic, go and
     * dsp_done before setting magic again, so dsp_done == 0 distinguishes a
     * fresh round from the one that just finished.
     *
     * dsp_ping is bumped on every poll so Linux can tell whether this loop is
     * actually running - without relying on the (shared, garbled) UART.
     */
    for (;;) {
        while (1) {
            sh_sync_in();
            if (rd(&sh->magic) == SHARED_MAGIC &&
                rd(&sh->dsp_bench_req) && !rd(&sh->dsp_bench_done))
                break;

            if (rd(&sh->magic) == SHARED_MAGIC && rd(&sh->go) &&
                !rd(&sh->dsp_done))
                break;

            wr(&sh->dsp_ping, ++ping);
            sh_sync_out();
            vTaskDelay(10);
        }

        sh_sync_in();
        if (rd(&sh->dsp_bench_req) && !rd(&sh->dsp_bench_done))
            run_bench(sh);
        else
            run_round(sh);
    }
}
