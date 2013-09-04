/*
 * Copyright (c) 2011, Max Filippov, Open Source and Linux Lab.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Open Source and Linux Lab nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "hw/hw.h"
#include "qemu/log.h"
#include "qemu/timer.h"

void check_interrupts(CPUXtensaState *env)
{
    CPUState *cs = CPU(xtensa_env_get_cpu(env));
    int minlevel = xtensa_get_cintlevel(env);
    uint32_t int_set_enabled = env->sregs[INTSET] & env->sregs[INTENABLE];
    int level;

    for (level = env->config->nlevel; level > minlevel; --level) {
        if (env->config->level_mask[level] & int_set_enabled) {
            env->pending_irq_level = level;
            cpu_interrupt(cs, CPU_INTERRUPT_HARD);
            qemu_log_mask(CPU_LOG_INT,
                    "%s level = %d, cintlevel = %d, "
                    "pc = %08x, a0 = %08x, ps = %08x, "
                    "intset = %08x, intenable = %08x, "
                    "ccount = %08x\n",
                    __func__, level, xtensa_get_cintlevel(env),
                    env->pc, env->regs[0], env->sregs[PS],
                    env->sregs[INTSET], env->sregs[INTENABLE],
                    env->sregs[CCOUNT]);
            return;
        }
    }
    env->pending_irq_level = 0;
    cpu_reset_interrupt(cs, CPU_INTERRUPT_HARD);
}

static void xtensa_set_irq(void *opaque, int irq, int active)
{
    CPUXtensaState *env = opaque;

    if (irq >= env->config->ninterrupt) {
        qemu_log("%s: bad IRQ %d\n", __func__, irq);
    } else {
        uint32_t irq_bit = 1 << irq;

        if (active) {
            env->sregs[INTSET] |= irq_bit;
        } else if (env->config->interrupt[irq].inttype == INTTYPE_LEVEL) {
            env->sregs[INTSET] &= ~irq_bit;
        }

        check_interrupts(env);
    }
}

void xtensa_timer_irq(CPUXtensaState *env, uint32_t id, uint32_t active)
{
    qemu_set_irq(env->irq_inputs[env->config->timerint[id]], active);
}

static void xtensa_ccompare_cb(void *opaque)
{
    XtensaCcompareTimer *ccompare = opaque;
    CPUXtensaState *env = ccompare->env;
    unsigned i = ccompare - env->ccompare;

    env->sregs[CCOUNT] = env->sregs[CCOMPARE + i];
    xtensa_timer_irq(env, i, 1);
}

static void *xtensa_env_get_extint(void *opaque, unsigned extint)
{
    CPUXtensaState *env = opaque;

    if (extint < env->config->nextint) {
        unsigned irq = env->config->extint[extint];
        return env->irq_inputs[irq];
    } else {
        qemu_log("%s: trying to acquire invalid external interrupt %d\n",
                __func__, extint);
        return NULL;
    }
}

void xtensa_irq_init(CPUXtensaState *env)
{
    env->irq_inputs = (void **)qemu_allocate_irqs(
            xtensa_set_irq, env, env->config->ninterrupt);
    if (xtensa_option_enabled(env->config, XTENSA_OPTION_TIMER_INTERRUPT)) {
        unsigned i;

        for (i = 0; i < env->config->nccompare; ++i) {
            env->ccompare[i].env = env;
            env->ccompare[i].timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                    xtensa_ccompare_cb, env->ccompare + i);
        }
    }
    env->irq_controller.opaque = env;
    env->irq_controller.get_irq = xtensa_env_get_extint;
}

#define MX_MAX_CPU 32
#define MX_MAX_IRQ 32

struct XtensaMx {
    unsigned n_cpu;
    unsigned n_irq;

    uint32_t ext_irq_state;
    uint32_t mieng;
    uint32_t miasg;
    uint32_t mirout[MX_MAX_IRQ];
    uint32_t mipipart;
    uint32_t runstall;

    QEMUTimer *timer;
    void **irq_inputs;
    XtensaIRQController irq_controller;
    struct XtensaMxCpu {
        XtensaMx *mx;
        CPUXtensaState *env;
        uint32_t mipicause;
        uint32_t mirout_cache;
        uint32_t irq_state_cache;
    } cpu[MX_MAX_CPU];
};

static uint32_t xtensa_mx_ext_reg_read(void *opaque, uint32_t offset)
{
    struct XtensaMx *mx = opaque;

    if (offset < MX_MAX_IRQ) {
        return mx->mirout[offset];
    } else if (offset >= 0x100 && offset < 0x100 + MX_MAX_CPU) {
        return mx->cpu[offset - 0x100].mipicause;
    } else {
        switch (offset) {
        case 0x180: /*MIENG*/
            return mx->mieng;

        case 0x188: /*MIASG*/
            return mx->miasg;

        case 0x190: /*MIPIPART*/
            return mx->mipipart;

        default:
            qemu_log("unknown RER in MX range: 0x%08x\n", offset);
            return 0;
        }
    }
}

static uint32_t xtensa_mx_get_ipi_for_cpu(const XtensaMx *mx, unsigned cpu)
{
    uint32_t mipicause = mx->cpu[cpu].mipicause;
    uint32_t mipipart = mx->mipipart;

    return ((mipicause & 1) << (mipipart & 3)) |
        ((mipicause & 0x000e) != 0) << ((mipipart >> 2) & 3) |
        ((mipicause & 0x00f0) != 0) << ((mipipart >> 4) & 3) |
        ((mipicause & 0xff00) != 0) << ((mipipart >> 6) & 3);
}

static uint32_t xtensa_mx_get_ext_irq_for_cpu(const XtensaMx *mx, unsigned cpu)
{
    return ((((mx->ext_irq_state & mx->mieng) | mx->miasg) &
                mx->cpu[cpu].mirout_cache) << 2) |
        xtensa_mx_get_ipi_for_cpu(mx, cpu);
}

static void xtensa_mx_update_cpu(XtensaMx *mx, unsigned cpu)
{
    uint32_t irq = xtensa_mx_get_ext_irq_for_cpu(mx, cpu);
    uint32_t changed_irq = mx->cpu[cpu].irq_state_cache ^ irq;
    XtensaIRQController *irq_controller =
        xtensa_env_get_irq_controller(mx->cpu[cpu].env);
    unsigned i;

    qemu_log_mask(CPU_LOG_INT, "%s: CPU %d, irq: %08x, changed_irq: %08x\n",
            __func__, cpu, irq, changed_irq);
    mx->cpu[cpu].irq_state_cache = irq;
    for (i = 0; changed_irq; ++i) {
        uint32_t mask = 1 << i;

        if (changed_irq & mask) {
            void *irq_line = xtensa_get_extint(irq_controller, i);

            changed_irq ^= mask;
            qemu_set_irq(irq_line, irq & mask);
        }
    }
}

static void xtensa_mx_update_all(XtensaMx *mx)
{
    unsigned i;
    for (i = 0; i < mx->n_cpu; ++i) {
        xtensa_mx_update_cpu(mx, i);
    }
}

static void xtensa_mx_ext_reg_write(void *opaque, uint32_t offset, uint32_t v)
{
    struct XtensaMx *mx = opaque;
    unsigned cpu;

    qemu_log("%s %08x: %08x\n", __func__, offset, v);
    if (offset < mx->n_irq) { /*MIROUT*/
        mx->mirout[offset] = v;
        for (cpu = 0; cpu < mx->n_cpu; ++cpu) {
            if (!(mx->cpu[cpu].mirout_cache & (1u << offset)) !=
                    !(v & (1u << cpu))) {
                mx->cpu[cpu].mirout_cache ^= 1u << offset;
                xtensa_mx_update_cpu(mx, cpu);
            }
        }
    } else if (offset >= 0x100 && offset < 0x100 + mx->n_cpu) { /*MIPICAUSE*/
        cpu = offset - 0x100;
        mx->cpu[cpu].mipicause &= ~v;
        xtensa_mx_update_cpu(mx, cpu);
    } else if (offset >= 0x140 && offset < 0x150) { /*MIPISET*/
        for (cpu = 0; cpu < mx->n_cpu; ++cpu) {
            if (v & (1u << cpu)) {
                mx->cpu[cpu].mipicause |= 1u << (offset - 0x140);
                xtensa_mx_update_cpu(mx, cpu);
            }
        }
    } else {
        uint32_t change = 0;

        switch (offset) {
        case 0x180: /*MIENG*/
            change = mx->mieng & ~v;
            mx->mieng &= ~v;
            break;

        case 0x184: /*MIENGSET*/
            change = ~mx->mieng & v;
            mx->mieng |= v;
            break;

        case 0x188: /*MIASG*/
            change = mx->miasg & ~v;
            mx->miasg &= ~v;
            break;

        case 0x18c: /*MIASGSET*/
            change = ~mx->miasg & v;
            mx->miasg |= v;
            break;

        case 0x190: /*MIPIPART*/
            change = mx->mipipart ^ v;
            mx->mipipart = v;
            break;

        default:
            qemu_log("unknown WER in MX range: 0x%08x = 0x%08x\n", offset, v);
            break;
        }
        if (change) {
            xtensa_mx_update_all(mx);
        }
    }
}

static uint32_t xtensa_mx_cpu_ext_reg_read(void *opaque, uint32_t offset)
{
    struct XtensaMxCpu *cpu = opaque;
    struct XtensaMx *mx = cpu->mx;

    switch (offset) {
    case 0x1a0: /*SYSCONFIGID*/
        return ((mx->n_cpu - 1) << 18) | (cpu - mx->cpu);

    case 0x200: /*MPSCORE*/
        return mx->runstall;

    default:
        return 0;
    }
}

static void xtensa_mx_cpu_ext_reg_write(void *opaque, uint32_t offset,
        uint32_t v)
{
    struct XtensaMxCpu *cpu = opaque;
    struct XtensaMx *mx = cpu->mx;
    uint32_t change;
    unsigned i;

    switch (offset) {
    case 0x200: /*MPSCORE*/
        change = mx->runstall ^ v;
        qemu_log_mask(CPU_LOG_INT,
                "%s: RUNSTALL changed by CPU %d: %08x -> %08x\n",
                __func__, (int)(cpu - mx->cpu), mx->runstall, v);
        mx->runstall = v;
        for (i = 0; i < mx->n_cpu; ++i, change >>= 1, v >>= 1) {
            if (change & 1) {
                xtensa_stall(mx->cpu[i].env, v & 1);
            }
        }
        cpu_exit(CPU(xtensa_env_get_cpu(cpu->env)));
        break;
    }
}

void xtensa_mx_register_env(XtensaMx *mx, CPUXtensaState *env)
{
    XtensaExtRegRange reg0 = {
        .base = 0,
        .sz = 0x1a0,
        .opaque = mx,
        .read = xtensa_mx_ext_reg_read,
        .write = xtensa_mx_ext_reg_write,
    };
    XtensaExtRegRange reg1 = {
        .base = 0x1a0,
        .sz = 0x90,
        .offset = 0x1a0,
        .opaque = mx->cpu + mx->n_cpu,
        .read = xtensa_mx_cpu_ext_reg_read,
        .write = xtensa_mx_cpu_ext_reg_write,
    };
    mx->cpu[mx->n_cpu].mx = mx;
    mx->cpu[mx->n_cpu].env = env;
    ++mx->n_cpu;
    xtensa_add_ext_reg(env, &reg0);
    xtensa_add_ext_reg(env, &reg1);
}

static void xtensa_mx_set_irq(void *opaque, int irq, int active)
{
    XtensaMx *mx = opaque;

    if (irq < mx->n_irq) {
        uint32_t old_irq_state = mx->ext_irq_state;

        if (active) {
            mx->ext_irq_state |= 1u << irq;
        } else {
            mx->ext_irq_state &= ~(1u << irq);
        }
        qemu_log_mask(CPU_LOG_INT,
                "%s: IRQ %d, active: %d, ext_irq_state: %08x -> %08x\n",
                __func__, irq, active, old_irq_state, mx->ext_irq_state);
        xtensa_mx_update_all(mx);
    } else {
        qemu_log("%s: IRQ %d out of range\n", __func__, irq);
    }
}

static void *xtensa_mx_get_irq(void *opaque, unsigned irq)
{
    XtensaMx *mx = opaque;

    assert(irq < mx->n_irq);
    return mx->irq_inputs[irq + 1];
}

static void xtensa_mx_timer_cb(void *opaque)
{
    XtensaMx *mx = opaque;
    timer_mod(mx->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              get_ticks_per_sec() / 1000);
}

XtensaMx *xtensa_mx_init(unsigned n_irq)
{
    XtensaMx *mx = calloc(1, sizeof(XtensaMx));
    mx->n_irq = n_irq + 1;
    mx->irq_inputs = (void **)qemu_allocate_irqs(
            xtensa_mx_set_irq, mx, n_irq + 1);
    mx->irq_controller.opaque = mx;
    mx->irq_controller.get_irq = xtensa_mx_get_irq;
    mx->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                             xtensa_mx_timer_cb, mx);
    return mx;
}

void xtensa_mx_reset(void *opaque)
{
    XtensaMx *mx = opaque;
    unsigned i;

    mx->mieng = mx->n_irq < 32 ? (1u << mx->n_irq) - 1 : ~0;
    mx->miasg = 0;
    mx->mipipart = 0;
    for (i = 0; i < mx->n_irq; ++i) {
        mx->mirout[i] = 1;
    }
    for (i = 0; i < mx->n_cpu; ++i) {
        mx->cpu[i].mipicause = 0;
        mx->cpu[i].mirout_cache = i ? 0 : mx->mieng;
    }
    mx->runstall = (1u << mx->n_cpu) - 2;
    for (i = 0; i < mx->n_cpu; ++i) {
        xtensa_stall(mx->cpu[i].env, i > 0);
    }
    timer_mod(mx->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              get_ticks_per_sec() / 1000);
}

XtensaIRQController *xtensa_mx_get_irq_controller(XtensaMx *mx)
{
    return &mx->irq_controller;
}
