#include "cpu.h"
#include "exec/exec-all.h"
#include "exec/gdbstub.h"
#include "qemu/host-utils.h"

#include "core-fsf_2014/core-isa.h"
#include "overlay_tool.h"

static XtensaConfig fsf_2014 __attribute__((unused)) = {
    .name = "fsf_2014",
    .gdb_regmap = {
        .reg = {
#include "core-fsf_2014/gdb-config.c"
        }
    },
    .clock_freq_khz = 40000,
    DEFAULT_SECTIONS
};

REGISTER_CORE(fsf_2014)
