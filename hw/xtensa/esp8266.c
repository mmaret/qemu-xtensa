/*
 * Copyright (c) 2014, Max Filippov, Open Source and Linux Lab.
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

#include "sysemu/sysemu.h"
#include "hw/boards.h"
#include "hw/loader.h"
#include "elf.h"
#include "exec/memory.h"
#include "exec/address-spaces.h"
#include "hw/char/serial.h"
#include "net/net.h"
#include "hw/sysbus.h"
#include "hw/block/flash.h"
#include "sysemu/block-backend.h"
#include "sysemu/char.h"
#include "sysemu/device_tree.h"
#include "qemu/error-report.h"
#include "bootparam.h"

/* Serial */

typedef struct Esp8266SerialState {
    MemoryRegion iomem;
    CharDriverState *chr;
} Esp8266SerialState;

static uint64_t esp8266_serial_read(void *opaque, hwaddr addr,
                                    unsigned size)
{
    //Esp8266SerialState *s = opaque;

    switch (addr) {
    case 0x1c: /*busy*/
        return 0;

    }
    return 0;
}

static void esp8266_serial_write(void *opaque, hwaddr addr,
                                 uint64_t val, unsigned size)
{
    Esp8266SerialState *s = opaque;

    switch (addr) {
    case 0x0: /*TX char*/
        if (s->chr) {
            uint8_t buf[1] = { val };
            qemu_chr_fe_write(s->chr, buf, 1);
        }
        break;
    }
}

static const MemoryRegionOps esp8266_serial_ops = {
    .read = esp8266_serial_read,
    .write = esp8266_serial_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static Esp8266SerialState *esp8266_serial_init(MemoryRegion *address_space,
                                               hwaddr base, qemu_irq irq,
                                               CharDriverState *chr)
{
    Esp8266SerialState *s = g_malloc(sizeof(Esp8266SerialState));

    s->chr = chr;
    memory_region_init_io(&s->iomem, NULL, &esp8266_serial_ops, s,
                          "esp8266.serial", 0x300);
    memory_region_add_subregion(address_space, base, &s->iomem);
    return s;
}

/* GPIO */

#define ESP8266_GPIO_STRAP_SD_START     (0x4 << 16)
#define ESP8266_GPIO_STRAP_FLASH_START  (0x3 << 16)
#define ESP8266_GPIO_STRAP_UART_START   (0x2 << 16)

typedef struct Esp8266GpioState {
    MemoryRegion iomem;
    uint32_t in;
} Esp8266GpioState;

static uint32_t user_entry;

static uint64_t esp8266_gpio_read(void *opaque, hwaddr addr,
                                  unsigned size)
{
    Esp8266GpioState *s = opaque;

    switch (addr) {
    case 0x18: /*in*/
        return s->in;

    case 0x80: /*cheat: user entry*/
        return user_entry;

    default:
        fprintf(stderr, "%s, %x\n", __func__, (uint32_t)addr);
        break;
    }
    return 0;
}

static void esp8266_gpio_write(void *opaque, hwaddr addr,
                               uint64_t val, unsigned size)
{
    //Esp8266GpioState *s = opaque;

    switch (addr) {
    }
}

static const MemoryRegionOps esp8266_gpio_ops = {
    .read = esp8266_gpio_read,
    .write = esp8266_gpio_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void esp8266_gpio_reset(void *opaque)
{
    Esp8266GpioState *s = opaque;

    s->in = ESP8266_GPIO_STRAP_FLASH_START | (0x7 << 29);
}

static Esp8266GpioState *esp8266_gpio_init(MemoryRegion *address_space,
                                           hwaddr base)
{
    Esp8266GpioState *s = g_malloc(sizeof(Esp8266GpioState));

    memory_region_init_io(&s->iomem, NULL, &esp8266_gpio_ops, s,
                          "esp8266.gpio", 0x100);
    memory_region_add_subregion(address_space, base, &s->iomem);
    qemu_register_reset(esp8266_gpio_reset, s);
    return s;

}

/* RTC */

typedef struct Esp8266RtcState {
    MemoryRegion iomem;
} Esp8266RtcState;

static uint64_t esp8266_rtc_read(void *opaque, hwaddr addr,
                                  unsigned size)
{
    //Esp8266RtcState *s = opaque;

    switch (addr) {
    case 0x08: /*?*/
        return 0;

    case 0x14: /*Reset reason: bits 0..3*/
        return 4;

    case 0x18: /*?*/
        return 5;

    default:
        fprintf(stderr, "%s, %x\n", __func__, (uint32_t)addr);
        break;
    }
    return 0;
}

static void esp8266_rtc_write(void *opaque, hwaddr addr,
                               uint64_t val, unsigned size)
{
    //Esp8266RtcState *s = opaque;

    switch (addr) {
    }
}

static const MemoryRegionOps esp8266_rtc_ops = {
    .read = esp8266_rtc_read,
    .write = esp8266_rtc_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void esp8266_rtc_reset(void *opaque)
{
    //Esp8266RtcState *s = opaque;
}

static Esp8266RtcState *esp8266_rtc_init(MemoryRegion *address_space,
                                           hwaddr base)
{
    Esp8266RtcState *s = g_malloc(sizeof(Esp8266RtcState));

    memory_region_init_io(&s->iomem, NULL, &esp8266_rtc_ops, s,
                          "esp8266.rtc", 0x100);
    memory_region_add_subregion(address_space, base, &s->iomem);
    qemu_register_reset(esp8266_rtc_reset, s);
    return s;

}

static uint64_t translate_phys_addr(void *opaque, uint64_t addr)
{
    XtensaCPU *cpu = opaque;

    return cpu_get_phys_page_debug(CPU(cpu), addr);
}


static void esp8266_reset(void *opaque)
{
    XtensaCPU *cpu = opaque;

    cpu_reset(CPU(cpu));
}

static uint64_t esp8266_io_read(void *opaque, hwaddr addr,
                                unsigned size)
{
    return 0;
}

static void esp8266_io_write(void *opaque, hwaddr addr,
                             uint64_t val, unsigned size)
{
}

static const MemoryRegionOps esp8266_io_ops = {
    .read = esp8266_io_read,
    .write = esp8266_io_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void xtensa_esp8266_init(MachineState *machine)
{
#ifdef TARGET_WORDS_BIGENDIAN
    int be = 1;
#else
    int be = 0;
#endif
    const char *rom_filename = "esp8266.rom";
    MemoryRegion *system_memory = get_system_memory();
    XtensaCPU *cpu = NULL;
    CPUXtensaState *env = NULL;
    MemoryRegion *ram, *system_io;
    QemuOpts *machine_opts = qemu_get_machine_opts();
    const char *cpu_model = machine->cpu_model;
    const char *kernel_filename = qemu_opt_get(machine_opts, "kernel");
    int n;

    if (!cpu_model) {
        cpu_model = "lx106";
    }

    for (n = 0; n < smp_cpus; n++) {
        cpu = cpu_xtensa_init(cpu_model);
        if (cpu == NULL) {
            error_report("unable to find CPU definition '%s'\n",
                         cpu_model);
            exit(EXIT_FAILURE);
        }
        env = &cpu->env;

        env->sregs[PRID] = n;
        qemu_register_reset(esp8266_reset, cpu);
        xtensa_select_static_vectors(env, 1);
        /* Need MMU initialized prior to ELF loading,
         * so that ELF gets loaded into virtual addresses
         */
        cpu_reset(CPU(cpu));
    }

    ram = g_malloc(sizeof(*ram));
    memory_region_init_ram(ram, NULL, "esp8266.dram", 0x00400000,
                           &error_abort);
    vmstate_register_ram_global(ram);
    memory_region_add_subregion(system_memory, 0x3ff00000, ram);

    system_io = g_malloc(sizeof(*system_io));
    memory_region_init_io(system_io, NULL, &esp8266_io_ops, NULL, "esp8266.io",
                          256 * 1024 * 1024);

    memory_region_add_subregion(system_memory, 0x60000000, system_io);

    if (!serial_hds[0]) {
        serial_hds[0] = qemu_chr_new("serial0", "null", NULL);
    }
    esp8266_serial_init(system_io, 0x00000000, xtensa_get_extint(env, 5),
                        serial_hds[0]);
    esp8266_gpio_init(system_io, 0x00000300);
    esp8266_rtc_init(system_io, 0x00000700);


    /* Use presence of kernel file name as 'boot from SRAM' switch. */
    if (kernel_filename) {
        uint64_t elf_entry;
        uint64_t elf_lowaddr;
        int success = load_elf(kernel_filename, translate_phys_addr, cpu,
                &elf_entry, &elf_lowaddr, NULL, be, ELF_MACHINE, 0);
        if (success > 0) {
            user_entry = elf_entry;
        } else {
            error_report("could not load kernel '%s'\n",
                         kernel_filename);
            exit(EXIT_FAILURE);
        }
        rom_filename = "esp8266-call-user.rom";
    }

    rom_filename = qemu_find_file(QEMU_FILE_TYPE_BIOS, rom_filename);
    if (!rom_filename ||
        load_image_targphys(rom_filename, 0x40000000, 65536) < 0) {
        error_report("unable to load ROM image '%s'\n", rom_filename);
        exit(EXIT_FAILURE);
    }
}

static QEMUMachine xtensa_esp8266_machine = {
    .name = "esp8266",
    .desc = "ESP8266 (" XTENSA_DEFAULT_CPU_MODEL ")",
    .init = xtensa_esp8266_init,
    .max_cpus = 1,
};

static void xtensa_lx_machines_init(void)
{
    qemu_register_machine(&xtensa_esp8266_machine);
}

machine_init(xtensa_lx_machines_init);
