/*
 * QEMU model of the Microsemi Smartfusion2 SPI Controller
 *
 * Copyright (C) 2017 Subbaraya Sundeep <sundeep.lkml@gmail.com>
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
#include "hw/sysbus.h"
#include "sysemu/sysemu.h"
#include "qemu/log.h"
#include "qemu/fifo32.h"

#include "hw/ssi/ssi.h"

#ifdef MSF2_SPI_ERR_DEBUG
#define DB_PRINT(...) do { \
    fprintf(stderr,  ": %s: ", __func__); \
    fprintf(stderr, ## __VA_ARGS__); \
    } while (0);
#else
    #define DB_PRINT(...)
#endif

#define FIFO_CAPACITY  32 
#define FIFO_CAPACITY  32 

#define R_CONTROL         0
#define R_DFSIZE          1
#define R_STATUS          2
#define R_INTCLR          3
#define R_RX              4
#define R_TX              5
#define R_CLKGEN          6
#define R_SS              7
#define R_MIS             8
#define R_RIS             9 
#define R_CONTROL2        10
#define R_COMMAND         11
#define R_PKTSIZE         12
#define R_CMDSIZE         13
#define R_HWSTATUS        14
#define R_STAT8           15
#define R_MAX             16

#define S_TXDATSENT       (1 << 0)
#define S_RXDATRCED       (1 << 1)
#define S_RXOVERFLOW      (1 << 2)
#define S_TXUNDERRUN      (1 << 3)
#define S_RXFIFOFUL       (1 << 4)
#define S_RXFIFOFULNXT    (1 << 5)
#define S_RXFIFOEMP       (1 << 6)
#define S_RXFIFOEMPNXT    (1 << 7)
#define S_TXFIFOFUL       (1 << 8)
#define S_TXFIFOFULNXT    (1 << 9)
#define S_TXFIFOEMP       (1 << 10)
#define S_TXFIFOEMPNXT    (1 << 11)
#define S_FRAMESTART      (1 << 12)
#define S_SSEL            (1 << 13)
#define S_ACTIVE          (1 << 14)

#define C_ENABLE          (1 << 0)
#define C_MODE            (1 << 1)
#define C_INTRXDATA       (1 << 1)
#define C_INTTXDATA       (1 << 1)
#define C_INTRXOVRFLO     (1 << 1)
#define C_INTTXTURUN      (1 << 1)
#define C_BIGFIFO         (1 << 1)
#define C_OENOFF          (1 << 1)
#define C_RESET           (1 << 1)

#define TXDONE            (1 << 0)
#define RXRDY             (1 << 1)
#define RXCHOVRF          (1 << 2)
#define TXCHUNDR          (1 << 3)
#define CMDINT            (1 << 4)
#define SSEND             (1 << 5)

#define TYPE_MSF2_SPI   "msf2-spi"
#define MSF2_SPI(obj)   OBJECT_CHECK(Msf2SPI, (obj), TYPE_MSF2_SPI)

typedef struct Msf2SPI {
    SysBusDevice parent_obj;

    MemoryRegion mmio;

    qemu_irq irq;
    int irqline;

    uint8_t num_cs;
    qemu_irq *cs_lines;

    SSIBus *spi;

    Fifo32 rx_fifo;
    Fifo32 tx_fifo;

    uint32_t regs[R_MAX];
} Msf2SPI;

static void txfifo_reset(Msf2SPI *s)
{
    fifo32_reset(&s->tx_fifo);

    s->regs[R_STATUS] &= ~S_TXFIFOFUL;
    s->regs[R_STATUS] |= S_TXFIFOEMP;
}

static void rxfifo_reset(Msf2SPI *s)
{
    fifo32_reset(&s->rx_fifo);

    s->regs[R_STATUS] &= ~S_RXFIFOFUL;
    s->regs[R_STATUS] |= S_RXFIFOEMP;
}

static void msf2_spi_do_reset(Msf2SPI *s)
{
    memset(s->regs, 0, sizeof s->regs);
	s->regs[R_CONTROL] = 0x80000102;
	s->regs[R_DFSIZE]  = 0x4;
	s->regs[R_STATUS]  = 0x2440;
	s->regs[R_CLKGEN] = 0x7;
	s->regs[R_STAT8]   = 0x7;

    rxfifo_reset(s);
    txfifo_reset(s);
}

static void msf2_spi_reset(DeviceState *d)
{
    msf2_spi_do_reset(MSF2_SPI(d));
}

static uint64_t
spi_read(void *opaque, hwaddr addr, unsigned int size)
{
    Msf2SPI *s = opaque;
    uint32_t r = 0;

    addr >>= 2;
    switch (addr) {

    default:
        if (addr < ARRAY_SIZE(s->regs)) {
            r = s->regs[addr];
        }
        break;
    }

    DB_PRINT("addr=" TARGET_FMT_plx " = %x\n", addr * 4, r);
    return r;
}

static void
spi_write(void *opaque, hwaddr addr,
            uint64_t val64, unsigned int size)
{
    Msf2SPI *s = opaque;
    uint32_t value = val64;
	int i;

    DB_PRINT("addr=" TARGET_FMT_plx " = %x\n", addr, value);
    addr >>= 2;
    switch (addr) {
	case R_TX:
    	s->regs[R_RX] = ssi_transfer(s->spi, value);
		s->regs[R_STATUS] &= ~S_RXFIFOEMP;
		break;

	case R_SS:
        s->regs[R_SS] = value;
    	for (i = 0; i < s->num_cs; ++i)
        	qemu_set_irq(s->cs_lines[i], !(s->regs[R_SS] & 1 << i));
		break;

    default:
        if (addr < ARRAY_SIZE(s->regs)) {
            s->regs[addr] = value;
        }
        break;
    }
}

static const MemoryRegionOps spi_ops = {
    .read = spi_read,
    .write = spi_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4
    }
};

static int msf2_spi_init(SysBusDevice *sbd)
{
    DeviceState *dev = DEVICE(sbd);
    Msf2SPI *s = MSF2_SPI(dev);
    int i;

    DB_PRINT("\n");

    s->spi = ssi_create_bus(dev, "spi");

    sysbus_init_irq(sbd, &s->irq);
    s->cs_lines = g_new0(qemu_irq, s->num_cs);
    ssi_auto_connect_slaves(dev, s->cs_lines, s->spi);
    for (i = 0; i < s->num_cs; ++i) {
        sysbus_init_irq(sbd, &s->cs_lines[i]);
    }

    memory_region_init_io(&s->mmio, OBJECT(s), &spi_ops, s,
                          "msf2-spi", R_MAX * 4);
    sysbus_init_mmio(sbd, &s->mmio);

    s->irqline = -1;

    return 0;
}

static const VMStateDescription vmstate_msf2_spi = {
    .name = "msf2_spi",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_FIFO32(tx_fifo, Msf2SPI),
        VMSTATE_FIFO32(rx_fifo, Msf2SPI),
        VMSTATE_UINT32_ARRAY(regs, Msf2SPI, R_MAX),
        VMSTATE_END_OF_LIST()
    }
};

static Property msf2_spi_properties[] = {
    DEFINE_PROP_UINT8("num-ss-bits", Msf2SPI, num_cs, 1),
    DEFINE_PROP_END_OF_LIST(),
};

static void msf2_spi_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);

    k->init = msf2_spi_init;
    dc->reset = msf2_spi_reset;
    dc->props = msf2_spi_properties;
    dc->vmsd = &vmstate_msf2_spi;
}

static const TypeInfo msf2_spi_info = {
    .name           = TYPE_MSF2_SPI,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(Msf2SPI),
    .class_init     = msf2_spi_class_init,
};

static void msf2_spi_register_types(void)
{
    type_register_static(&msf2_spi_info);
}

type_init(msf2_spi_register_types)
