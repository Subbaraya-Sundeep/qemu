/*
 * QEMU model of the SmartFusion2 timer.
 *
 * Copyright (c) 2017 Subbaraya Sundeep <sundeep.babi@gmail.com>.
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
#include "hw/ptimer.h"
#include "qemu/log.h"
#include "qemu/main-loop.h"

#define D(x)

#define NUM_TIMERS     2

#define R_VAL          0
#define R_LOADVAL      1
#define R_BGLOADVAL    2
#define R_CTRL         3  
#define R_RIS          4  
#define R_MIS          5  
#define R_MAX          6

#define TIMER_CTRL_ENBL     (1<<0)
#define TIMER_CTRL_ONESHOT  (1<<1)
#define TIMER_CTRL_INTR     (1<<2)
#define TIMER_RIS_ACK       (1<<0)
#define TIMER_RST_CLR       (1<<6)

struct msf2_timer
{
    QEMUBH *bh;
    ptimer_state *ptimer;
    void *parent;
    int nr; /* for debug.  */

    unsigned long timer_div;

    uint32_t regs[R_MAX];
    qemu_irq irq;
};

#define TYPE_MSF2_TIMER "msf2-timer"
#define MSF2_TIMER(obj) \
    OBJECT_CHECK(struct timerblock, (obj), TYPE_MSF2_TIMER)

struct timerblock
{
    SysBusDevice parent_obj;

    MemoryRegion mmio;
    uint32_t freq_hz;
    struct msf2_timer *timers;
};

static inline unsigned int timer_from_addr(hwaddr addr)
{
    /* Timers get a 6x32bit control reg area each.  */
    return addr / R_MAX;
}

static void timer_update_irq(struct msf2_timer *st)
{
    int isr;
    int ier;

    isr = !!(st->regs[R_RIS] & TIMER_RIS_ACK);
    ier = !!(st->regs[R_CTRL] & TIMER_CTRL_INTR);

    qemu_set_irq(st->irq, (ier && isr));
}

static uint64_t
timer_read(void *opaque, hwaddr addr, unsigned int size)
{
    struct timerblock *t = opaque;
    struct msf2_timer *st;
    uint32_t r = 0;
    unsigned int timer;
    int isr;
    int ier;

    addr >>= 2;
    timer = timer_from_addr(addr);
    st = &t->timers[timer];

    switch (addr)
    {
        case R_VAL:
             r = ptimer_get_count(st->ptimer);
             D(qemu_log("msf2_timer t=%d read counter=%x\n", timer, r));
             break;

        case R_MIS:
             isr = !!(st->regs[R_RIS] & TIMER_RIS_ACK);
             ier = !!(st->regs[R_CTRL] & TIMER_CTRL_INTR);
             r = ier && isr;
             break;

        default:
            if (addr < ARRAY_SIZE(st->regs))
                r = st->regs[addr];
            break;
    }
    D(fprintf(stderr, "%s timer=%d %x=%x\n", __func__, timer, addr * 4, r));
    return r;
}

static void timer_update(struct msf2_timer *st)
{
    uint64_t count;

    D(fprintf(stderr, "%s timer=%d\n", __func__, st->nr));

    if (!(st->regs[R_CTRL] & TIMER_CTRL_ENBL)) {
        ptimer_stop(st->ptimer);
		return;
	}

    count = st->regs[R_LOADVAL];
    ptimer_set_limit(st->ptimer, count, 1);
    ptimer_run(st->ptimer, 1);
}

static void
timer_write(void *opaque, hwaddr addr,
            uint64_t val64, unsigned int size)
{
    struct timerblock *t = opaque;
    struct msf2_timer *st;
    unsigned int timer;
    uint32_t value = val64;

    addr >>= 2;
    timer = timer_from_addr(addr);
    st = &t->timers[timer];
    D(fprintf(stderr, "%s addr=%x val=%x (timer=%d off=%d)\n",
             __func__, addr * 4, value, timer, addr & 3));

    switch (addr) 
    {
        case R_CTRL:
            st->regs[R_CTRL] = value;
            timer_update(st);
            break;
 
        case R_RIS:
            if (value & TIMER_RIS_ACK)
                st->regs[R_RIS] &= ~TIMER_RIS_ACK;
            break;

        case R_LOADVAL:
            st->regs[R_LOADVAL] = value;
            if (st->regs[R_CTRL] & TIMER_CTRL_ENBL)
                timer_update(st);
            break;

        case R_BGLOADVAL:
            st->regs[R_BGLOADVAL] = value;
            st->regs[R_LOADVAL] = value;
			break;

        case R_VAL:
        case R_MIS:
            break;

        default:
            if (addr < ARRAY_SIZE(st->regs))
                st->regs[addr] = value;
            break;
    }
    timer_update_irq(st);
}

static const MemoryRegionOps timer_ops = {
    .read = timer_read,
    .write = timer_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4
    }
};

static void timer_hit(void *opaque)
{
    struct msf2_timer *st = opaque;
    D(fprintf(stderr, "%s %d\n", __func__, st->nr));
    st->regs[R_RIS] |= TIMER_RIS_ACK;

    if (!(st->regs[R_CTRL] & TIMER_CTRL_ONESHOT))
        timer_update(st);
    timer_update_irq(st);
}

static void msf2_timer_realize(DeviceState *dev, Error **errp)
{
    struct timerblock *t = MSF2_TIMER(dev);
    unsigned int i;

    /* Init all the ptimers.  */
    t->timers = g_malloc0((sizeof t->timers[0]) * NUM_TIMERS);
    for (i = 0; i < NUM_TIMERS; i++) {
        struct msf2_timer *st = &t->timers[i];

        st->parent = t;
        st->nr = i;
        st->bh = qemu_bh_new(timer_hit, st);
        st->ptimer = ptimer_init(st->bh);
        ptimer_set_freq(st->ptimer, t->freq_hz);
        sysbus_init_irq(SYS_BUS_DEVICE(dev), &st->irq);
    }

    memory_region_init_io(&t->mmio, OBJECT(t), &timer_ops, t, "msf2-timer",
                          R_MAX * 4 * NUM_TIMERS);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &t->mmio);
}

static Property msf2_timer_properties[] = {
    DEFINE_PROP_UINT32("clock-frequency", struct timerblock, freq_hz,
                                                                83 * 1000000),
    DEFINE_PROP_END_OF_LIST(),
};

static void msf2_timer_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = msf2_timer_realize;
    dc->props = msf2_timer_properties;
}

static const TypeInfo msf2_timer_info = {
    .name          = TYPE_MSF2_TIMER,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(struct timerblock),
    .class_init    = msf2_timer_class_init,
};

static void msf2_timer_register_types(void)
{
    type_register_static(&msf2_timer_info);
}

type_init(msf2_timer_register_types)
