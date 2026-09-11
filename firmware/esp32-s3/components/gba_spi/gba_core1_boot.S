/* gba_core1_boot.S — Xtensa asm trampoline: the APP-CPU (core 1) boot entry for our bare-metal loop.
 *
 * The ROM app-CPU boot path (triggered by ets_set_appcpu_boot_addr + unstall in gba_core1_start) sets up
 * PS / window registers / a temp stack before jumping here — exactly as it does for ESP-IDF's own
 * call_start_cpu1 (which is likewise a plain function set as the boot address). So we inherit a valid
 * windowed CPU state and only need to: switch SP to our reserved stack, terminate the backtrace, and
 * window-call into the C bare loop (which never returns). Lives in IRAM (no flash/cache dependency).
 */
    .section .iram1, "ax"
    .align  4
    .global gba_core1_boot
    .type   gba_core1_boot, @function
gba_core1_boot:
    movi    a1, core1_stack + 8192      /* SP = top of core 1's reserved stack */
    movi    a0, 0                        /* terminate call-stack backtrace */
    call8   gba_core1_main               /* windowed call into the C bare loop (noreturn) */
.Lhang:
    j       .Lhang                       /* never reached */
    .size   gba_core1_boot, . - gba_core1_boot
