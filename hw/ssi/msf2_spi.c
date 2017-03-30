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

#define FIFO_CAPACITY     32 
#define FIFO_CAPACITY     32 

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

#define FRAMESZ_MASK	  0x1F		

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

    qemu_irq cs_line;

    SSIBus *spi;

    Fifo32 rx_fifo;
    Fifo32 tx_fifo;

    int fifo_depth;
	bool enabled;

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

static void set_fifodepth(Msf2SPI *s)
{
	int size = s->regs[R_DFSIZE] & FRAMESZ_MASK;

	if (0 <= size && size <= 8)
		s->fifo_depth = 32;
	if (9 <= size && size <= 16)
		s->fifo_depth = 16;
	if (17 <= size && size <= 32)
		s->fifo_depth = 8;
}

static void msf2_spi_do_reset(Msf2SPI *s)
{
    memset(s->regs, 0, sizeof s->regs);
    s->regs[R_CONTROL] = 0x80000102;
    s->regs[R_DFSIZE] = 0x4;
    s->regs[R_STATUS] = 0x2440;
    s->regs[R_CLKGEN] = 0x7;
    s->regs[R_STAT8] = 0x7;

	s->fifo_depth = 4;
	s->enabled = false;

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
    case R_RX:
        s->regs[R_STATUS] &= ~S_RXFIFOFUL;
        r = fifo32_pop(&s->rx_fifo);
        if (fifo32_is_empty(&s->rx_fifo)) {
        	s->regs[R_STATUS] |= S_RXFIFOEMP;
        }
        break;

    default:
        if (addr < ARRAY_SIZE(s->regs)) {
            r = s->regs[addr];
        }
        break;
    }

	DB_PRINT("addr=" TARGET_FMT_plx " = %x\n", addr * 4, r);
    return r;
}

static void spi_flush_txfifo(Msf2SPI *s)
{
    uint32_t tx;
    uint32_t rx;

    while (!fifo32_is_empty(&s->tx_fifo)) {
        tx = fifo32_pop(&s->tx_fifo);
        DB_PRINT("data tx:%x\n", tx);
        rx = ssi_transfer(s->spi, tx);
        DB_PRINT("data rx:%x\n", rx);

        if (fifo32_num_used(&s->rx_fifo) == s->fifo_depth) {
            s->regs[R_STATUS] |= S_RXOVERFLOW;
        } else {
            fifo32_push(&s->rx_fifo, rx);
        	s->regs[R_STATUS] &= ~S_RXFIFOEMP;
        	if (fifo32_num_used(&s->rx_fifo) == (s->fifo_depth - 1)) {
                s->regs[R_STATUS] |= S_RXFIFOFULNXT;
            }
        	if (fifo32_num_used(&s->rx_fifo) == s->fifo_depth) {
                s->regs[R_STATUS] |= S_RXFIFOFUL;
            }
        }
    }
}

static void spi_write(void *opaque, hwaddr addr,
            uint64_t val64, unsigned int size)
{
    Msf2SPI *s = opaque;
    uint32_t value = val64;

    DB_PRINT("addr=" TARGET_FMT_plx " = %x\n", addr, value);
    addr >>= 2;

    switch (addr) {
    case R_TX:
        s->regs[R_STATUS] &= ~S_TXFIFOEMP;
        fifo32_push(&s->tx_fifo, value);
        if (fifo32_num_used(&s->tx_fifo) == (s->fifo_depth - 1)) {
            s->regs[R_STATUS] |= S_TXFIFOFULNXT;
        }
        if (fifo32_num_used(&s->tx_fifo) == s->fifo_depth) {
            s->regs[R_STATUS] |= S_TXFIFOFUL;
        }
		if (s->enabled)
        	spi_flush_txfifo(s);
        break;
 
    case R_SS:
        s->regs[R_SS] = value;
//		printf("cs:%d\n", !(s->regs[R_SS] & 1 << 0));
        qemu_set_irq(s->cs_line, !(s->regs[R_SS] & 1));
        break;

    case R_CONTROL:
		s->regs[R_CONTROL] = value;
		if (value & C_BIGFIFO) {
			set_fifodepth(s);
        } else {
			s->fifo_depth = 4;
		}
		if (value & C_ENABLE) {
			s->enabled = true;
		} else {
			s->enabled = false;
		}
		break;

    case R_DFSIZE:
		if (s->enabled)
			break;
		s->regs[R_DFSIZE] = value;
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

    DB_PRINT("\n");

    s->spi = ssi_create_bus(dev, "spi");

    sysbus_init_irq(sbd, &s->irq);
    ssi_auto_connect_slaves(dev, &s->cs_line, s->spi);
    sysbus_init_irq(sbd, &s->cs_line);

    memory_region_init_io(&s->mmio, OBJECT(s), &spi_ops, s,
                          "msf2-spi", R_MAX * 4);
    sysbus_init_mmio(sbd, &s->mmio);

    fifo32_create(&s->tx_fifo, FIFO_CAPACITY);
    fifo32_create(&s->rx_fifo, FIFO_CAPACITY);

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

static void msf2_spi_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);

    k->init = msf2_spi_init;
    dc->reset = msf2_spi_reset;
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
