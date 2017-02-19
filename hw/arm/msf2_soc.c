/*
 * MSF2 SoC
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
#include "hw/arm/msf2_soc.h"

static const uint32_t timer_addr[MSF2_NUM_TIMERS] = { 0x40004000 };
static const uint32_t usart_addr[MSF2_NUM_USARTS] = { 0x40000000 };

static const int timer_irq[MSF2_NUM_TIMERS] = {14};
static const int usart_irq[MSF2_NUM_USARTS] = {10};

static void msf2_soc_initfn(Object *obj)
{
    MSF2State *s = MSF2_SOC(obj);
    int i;

    for (i = 0; i < MSF2_NUM_USARTS; i++) {
        object_initialize(&s->usart[i], sizeof(s->usart[i]),
                          TYPE_MSF2_USART);
        qdev_set_parent_bus(DEVICE(&s->usart[i]), sysbus_get_default());
    }

    for (i = 0; i < MSF2_NUM_TIMERS; i++) {
        object_initialize(&s->timer[i], sizeof(s->timer[i]),
                          TYPE_MSF2_TIMER);
        qdev_set_parent_bus(DEVICE(&s->timer[i]), sysbus_get_default());
    }
}

static void msf2_soc_realize(DeviceState *dev_soc, Error **errp)
{
    MSF2State *s = MSF2_SOC(dev_soc);
    DeviceState *usartdev, *timerdev, *nvic;
    SysBusDevice *usartbusdev, *timerbusdev;
    Error *err = NULL;
    int i;

    MemoryRegion *system_memory = get_system_memory();
    MemoryRegion *sram = g_new(MemoryRegion, 1);
    MemoryRegion *ddr = g_new(MemoryRegion, 1);

    memory_region_init_ram(ddr, NULL, "MSF2.ddr", DDR_SIZE,
                           &error_fatal);
    vmstate_register_ram_global(ddr);
    memory_region_add_subregion(system_memory, DDR_BASE_ADDRESS, ddr);

    memory_region_init_ram(sram, NULL, "MSF2.sram", SRAM_SIZE,
                           &error_fatal);
    vmstate_register_ram_global(sram);
    memory_region_add_subregion(system_memory, SRAM_BASE_ADDRESS, sram);

    nvic = armv7m_init(get_system_memory(), DDR_SIZE, 96,
                       s->kernel_filename, s->cpu_model);

    for (i = 0; i < MSF2_NUM_USARTS; i++) {
        serial_mm_init(get_system_memory(), usart_addr[i], 2,
                           qdev_get_gpio_in(nvic, usart_irq[i]),
                           115200, serial_hds[i],
                           DEVICE_NATIVE_ENDIAN);
    }

    for (i = 0; i < MSF2_NUM_TIMERS; i++) {
        timerdev = DEVICE(&(s->timer[i]));
        qdev_prop_set_uint64(timerdev, "clock-frequency", 83000000);
        object_property_set_bool(OBJECT(&s->timer[i]), true, "realized", &err);
        if (err != NULL) {
            error_propagate(errp, err);
            return;
        }
        timerbusdev = SYS_BUS_DEVICE(timerdev);
        sysbus_mmio_map(timerbusdev, 0, timer_addr[i]);
        sysbus_connect_irq(timerbusdev, 0,
                           qdev_get_gpio_in(nvic, timer_irq[i]));
    }
}

static Property msf2_soc_properties[] = {
    DEFINE_PROP_STRING("kernel-filename", MSF2State, kernel_filename),
    DEFINE_PROP_STRING("cpu-model", MSF2State, cpu_model),
    DEFINE_PROP_END_OF_LIST(),
};

static void msf2_soc_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = msf2_soc_realize;
    dc->props = msf2_soc_properties;
}

static const TypeInfo msf2_soc_info = {
    .name          = TYPE_MSF2_SOC,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(MSF2State),
    .instance_init = msf2_soc_initfn,
    .class_init    = msf2_soc_class_init,
};

static void msf2_soc_types(void)
{
    type_register_static(&msf2_soc_info);
}

type_init(msf2_soc_types)
