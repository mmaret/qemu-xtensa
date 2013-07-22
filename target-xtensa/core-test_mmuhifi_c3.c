#include "cpu.h"
#include "exec/exec-all.h"
#include "exec/gdbstub.h"
#include "qemu/host-utils.h"

#include "core-test_mmuhifi_c3/core-isa.h"
#include "overlay_tool.h"

static const XtensaConfig test_mmuhifi_c3 = {
    .name = "test_mmuhifi_c3",
    .options = XTENSA_OPTIONS,
    .gdb_regmap = {
        .num_regs = 107,
        .num_core_regs = 62,
        .reg = {
#include "core-test_mmuhifi_c3/gdb-config.c"
        }
    },
    .nareg = XCHAL_NUM_AREGS,
    .ndepc = 1,
    EXCEPTIONS_SECTION,
    INTERRUPTS_SECTION,
    TLB_SECTION,
    .clock_freq_khz = 50000,
};

REGISTER_CORE(test_mmuhifi_c3)
