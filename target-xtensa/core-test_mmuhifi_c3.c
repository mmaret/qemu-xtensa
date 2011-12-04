#include "cpu.h"
#include "exec/exec-all.h"
#include "exec/gdbstub.h"
#include "qemu/host-utils.h"

#include "core-test_mmuhifi_c3/core-isa.h"
#include "overlay_tool.h"

static XtensaConfig test_mmuhifi_c3 __attribute__((unused)) = {
    .name = "test_mmuhifi_c3",
    .options = XTENSA_OPTIONS,
    .gdb_regmap = {
        .num_regs = 107,
        .num_core_regs = 62,
        .reg = {
#include "core-test_mmuhifi_c3/gdb-config.c"
        }
    },
    .clock_freq_khz = 10000,
    DEFAULT_SECTIONS
};

REGISTER_CORE(test_mmuhifi_c3)
