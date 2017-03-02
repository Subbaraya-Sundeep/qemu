/*
 * System Register block model of SmartFusion2
 *
 * Copyright (c) 2017 Subbaraya Sundeep <sundeep.lkml@gmail.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, see <http://www.gnu.org/licenses/>.
 */

#include "qemu/osdep.h"
#include "hw/hw.h"
#include "qemu/timer.h"
#include "hw/sysbus.h"
#include "sysemu/sysemu.h"
#include "qemu/log.h"

#ifndef MSF2_SYSREG_ERR_DEBUG
#define MSF2_SYSREG_ERR_DEBUG 0
#endif

#define DB_PRINT(...) do { \
        if (MSF2_SYSREG_ERR_DEBUG) { \
            fprintf(stderr,  ": %s: ", __func__); \
            fprintf(stderr, ## __VA_ARGS__); \
        } \
    } while (0);

#define R_PSS_RST_CTRL_SOFT_RST 0x1

enum {
    ESRAM_CR        = 0x00 / 4,
    ESRAM_MAX_LAT,
    DDR_CR,
    ENVM_CR,
    ENVM_REMAP_BASE_CR,
    ENVM_REMAP_FAB_CR,
    CC_CR,
    CC_REGION_CR,
    CC_LOCK_BASE_ADDR_CR,
    CC_FLUSH_INDX_CR,
    DDRB_BUF_TIMER_CR,
    DDRB_NB_ADDR_CR,
    DDRB_NB_SIZE_CR,
    DDRB_CR,

    SOFT_RESET_CR  = 0x48 / 4,
    M3_CR,

    GPIO_SYSRESET_SEL_CR = 0x58 / 4,

    MDDR_CR = 0x60 / 4,

    MSSDDR_PLL_STATUS_LOW_CR = 0x90 / 4,
    MSSDDR_PLL_STATUS_HIGH_CR,
    MSSDDR_FACC1_CR,
    MSSDDR_FACC2_CR,

    MSSDDR_PLL_STATUS = 0x150 / 4,

};

#define MSF2_SYSREG_MMIO_SIZE     0x300
#define MSF2_SYSREG_NUM_REGS      (MSF2_SYSREG_MMIO_SIZE / 4)

#define TYPE_MSF2_SYSREG "msf2-sysreg"
#define MSF2_SYSREG(obj) OBJECT_CHECK(Sf2SysregState, (obj), TYPE_MSF2_SYSREG)

#define MSF2_DDRC_MMIO_SIZE       0x150
#define MSF2_DDRC_NUM_REGS      (MSF2_DDRC_MMIO_SIZE / 4)

#define TYPE_MSF2_DDRC "msf2-ddrc"
#define MSF2_DDRC(obj) OBJECT_CHECK(DDRcState, (obj), TYPE_MSF2_DDRC)

typedef struct Sf2SysregState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;

    uint32_t regs[MSF2_SYSREG_NUM_REGS];
} Sf2SysregState;

typedef struct DDRcState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;

    uint32_t regs[MSF2_DDRC_NUM_REGS];
} DDRcState;

static void msf2_sysreg_reset(DeviceState *d)
{
    Sf2SysregState *s = MSF2_SYSREG(d);

    DB_PRINT("RESET\n");

    s->regs[MSSDDR_PLL_STATUS_LOW_CR] = 0x02420041;
    s->regs[MSSDDR_FACC1_CR] = 0x0A482124;
    s->regs[MSSDDR_PLL_STATUS] = 0x3;
}

static uint64_t msf2_sysreg_read(void *opaque, hwaddr offset,
    unsigned size)
{
    Sf2SysregState *s = opaque;
    offset /= 4;
    uint32_t ret = s->regs[offset];

    DB_PRINT("addr: %08" HWADDR_PRIx " data: %08" PRIx32 "\n", offset * 4, ret);
 
   return ret;
}

static void msf2_sysreg_write(void *opaque, hwaddr offset,
                          uint64_t val, unsigned size)
{
    Sf2SysregState *s = (Sf2SysregState *)opaque;
    offset /= 4;

    DB_PRINT("addr: %08" HWADDR_PRIx " data: %08" PRIx64 "\n", offset * 4, val);

    switch (offset) {
    case MSSDDR_PLL_STATUS:
        break;

    default:
        s->regs[offset] = val;
        break;
    }
}

static const MemoryRegionOps sysreg_ops = {
    .read = msf2_sysreg_read,
    .write = msf2_sysreg_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static uint64_t msf2_ddrc_read(void *opaque, hwaddr offset,
    unsigned size)
{
    DDRcState *s = opaque;
    offset /= 4;
    uint32_t ret = s->regs[offset];

    DB_PRINT("addr: %08" HWADDR_PRIx " data: %08" PRIx32 "\n", offset * 4, ret);
 
    return ret;
}

static void msf2_ddrc_write(void *opaque, hwaddr offset,
                          uint64_t val, unsigned size)
{
    DDRcState *s = opaque;
    offset /= 4;

    s->regs[offset] = val;

    DB_PRINT("addr: %08" HWADDR_PRIx " data: %08" PRIx64 "\n", offset * 4, val);
}

static const MemoryRegionOps ddrc_ops = {
    .read = msf2_ddrc_read,
    .write = msf2_ddrc_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void msf2_sysreg_init(Object *obj)
{
    Sf2SysregState *s = MSF2_SYSREG(obj);

    memory_region_init_io(&s->iomem, obj, &sysreg_ops, s, "sysreg",
                          MSF2_SYSREG_MMIO_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
}

static const VMStateDescription vmstate_msf2_sysreg = {
    .name = "msf2_sysreg",
    .version_id = 2,
    .minimum_version_id = 2,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, Sf2SysregState, MSF2_SYSREG_NUM_REGS),
        VMSTATE_END_OF_LIST()
    }
};

static void msf2_sysreg_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->vmsd = &vmstate_msf2_sysreg;
    dc->reset = msf2_sysreg_reset;
}

static void msf2_ddrc_init(Object *obj)
{
    DDRcState *s = MSF2_DDRC(obj);

    memory_region_init_io(&s->iomem, obj, &ddrc_ops, s, "ddrc",
                          MSF2_DDRC_MMIO_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
}

static const VMStateDescription vmstate_msf2_ddrc = {
    .name = "msf2_ddrc",
    .version_id = 2,
    .minimum_version_id = 2,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, DDRcState, MSF2_DDRC_NUM_REGS),
        VMSTATE_END_OF_LIST()
    }
};

static void msf2_ddrc_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->vmsd = &vmstate_msf2_ddrc;
}

static const TypeInfo msf2_sysreg_info = {
    .class_init = msf2_sysreg_class_init,
    .name  = TYPE_MSF2_SYSREG,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(Sf2SysregState),
    .instance_init = msf2_sysreg_init,
};

static const TypeInfo msf2_ddrc_info = {
    .class_init = msf2_ddrc_class_init,
    .name  = TYPE_MSF2_DDRC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(DDRcState),
    .instance_init = msf2_ddrc_init,
};

static void msf2_sysreg_register_types(void)
{
    type_register_static(&msf2_sysreg_info);
}

static void msf2_ddrc_register_types(void)
{
    type_register_static(&msf2_ddrc_info);
}

type_init(msf2_sysreg_register_types)
type_init(msf2_ddrc_register_types)
