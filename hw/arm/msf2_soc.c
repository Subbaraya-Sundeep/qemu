/*
 * Microsemi Smartfusion2 dev-kit emulation
 *
 * Copyright (c) 2017 Subbaraya Sundeep <sundeep.lkml@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu-common.h"
#include "hw/arm/arm.h"
#include "exec/address-spaces.h"
#include "hw/sysbus.h"
#include "hw/char/serial.h"
#include "hw/boards.h"
#include "sysemu/block-backend.h"
#include "hw/ssi/ssi.h"

#define MSF2_NUM_USARTS      1
#define MSF2_NUM_TIMERS      2

#define ENVM_BASE_ADDRESS     0x60000000
#define ENVM_SIZE             (128 * 1024)

#define DDR_BASE_ADDRESS      0xA0000000
#define DDR_SIZE             (1024 * 1024 * 1024)

#define SRAM_BASE_ADDRESS    0x20000000
#define SRAM_SIZE           (64 * 1024)

#define MSF2_TIMER_BASE      0x40004000
#define MSF2_SYSREG_BASE     0x40038000
#define MSF2_DDRC_BASE       0x40020000
#define MSF2_SPI0_BASE       0x40001000

static const uint32_t usart_addr[MSF2_NUM_USARTS] = { 0x40000000 };

static const int timer_irq[MSF2_NUM_TIMERS] = {14, 15};
static const int usart_irq[MSF2_NUM_USARTS] = {10};

static void msf2_init(MachineState *machine)
{
    const char *kernel_filename = NULL;
    DeviceState *dev, *nvic;
    int i;
    MemoryRegion *system_memory = get_system_memory();
    MemoryRegion *nvm = g_new(MemoryRegion, 1);
    MemoryRegion *nvm_alias = g_new(MemoryRegion, 1);
    MemoryRegion *sram = g_new(MemoryRegion, 1);
    MemoryRegion *ddr = g_new(MemoryRegion, 1);
    QemuOpts *machine_opts = qemu_get_machine_opts();
    SysBusDevice *busdev;
    DriveInfo *dinfo = drive_get_next(IF_MTD);
    qemu_irq cs_line;
    SSIBus *spi;

    kernel_filename = qemu_opt_get(machine_opts, "kernel");

    memory_region_init_ram(nvm, NULL, "MSF2.envm", ENVM_SIZE,
                           &error_fatal);
    memory_region_init_alias(nvm_alias, NULL, "STM32F205.flash.alias",
                             nvm, 0, ENVM_SIZE);
    vmstate_register_ram_global(nvm);

    memory_region_set_readonly(nvm, true);
    memory_region_set_readonly(nvm_alias, true);

    memory_region_add_subregion(system_memory, ENVM_BASE_ADDRESS, nvm);
    memory_region_add_subregion(system_memory, 0, nvm_alias);

    memory_region_init_ram(ddr, NULL, "MSF2.ddr", DDR_SIZE,
                           &error_fatal);
    vmstate_register_ram_global(ddr);
    memory_region_add_subregion(system_memory, DDR_BASE_ADDRESS, ddr);

    memory_region_init_ram(sram, NULL, "MSF2.sram", SRAM_SIZE,
                           &error_fatal);
    vmstate_register_ram_global(sram);
    memory_region_add_subregion(system_memory, SRAM_BASE_ADDRESS, sram);

    nvic = armv7m_init(system_memory, ENVM_SIZE, 96,
                       kernel_filename, "cortex-m3");

    for (i = 0; i < MSF2_NUM_USARTS; i++) {
	if (serial_hds[i])
                serial_mm_init(get_system_memory(), usart_addr[i], 2,
                       qdev_get_gpio_in(nvic, usart_irq[i]),
                       115200, serial_hds[i], DEVICE_NATIVE_ENDIAN);
    }

    dev = qdev_create(NULL, "msf2-timer");
    qdev_prop_set_uint32(dev, "clock-frequency", 83 * 1000000);
    qdev_init_nofail(dev);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, MSF2_TIMER_BASE);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0,
                           qdev_get_gpio_in(nvic, timer_irq[0]));
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 1,
                           qdev_get_gpio_in(nvic, timer_irq[1]));

    dev = qdev_create(NULL, "msf2-sysreg");
    qdev_init_nofail(dev);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, MSF2_SYSREG_BASE);

    dev = qdev_create(NULL, "msf2-ddrc");
    qdev_init_nofail(dev);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, MSF2_DDRC_BASE);

    dev = qdev_create(NULL, "msf2-spi");
    qdev_prop_set_uint32(dev, "num-ss-bits", 1);
    qdev_init_nofail(dev);
    busdev = SYS_BUS_DEVICE(dev);
    sysbus_mmio_map(busdev, 0, MSF2_SPI0_BASE);
    sysbus_connect_irq(busdev, 0, qdev_get_gpio_in(nvic, 2));

    spi = (SSIBus *)qdev_get_child_bus(dev, "spi");
    dev = ssi_create_slave_no_init(spi, "s25sl064p");
    if (dinfo)
		qdev_prop_set_drive(dev, "drive", blk_by_legacy_dinfo(dinfo),
                                    &error_fatal);
    qdev_init_nofail(dev);
    cs_line = qdev_get_gpio_in_named(dev, SSI_GPIO_CS, 0);
    sysbus_connect_irq(busdev, 1, cs_line);
}

static void msf2_machine_init(MachineClass *mc)
{
    mc->desc = "Microsemi Smart Fusion2 Development board";
    mc->init = msf2_init;
}

DEFINE_MACHINE("smartfusion2", msf2_machine_init)
