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

#ifndef HW_ARM_MSF2_SOC_H
#define HW_ARM_MSF2_SOC_H

#include "hw/timer/msf2_timer.h"
#include "hw/char/msf2_usart.h"

#define TYPE_MSF2_SOC "msf2-soc"
#define MSF2_SOC(obj) \
    OBJECT_CHECK(MSF2State, (obj), TYPE_MSF2_SOC)

#define MSF2_NUM_USARTS 1
#define MSF2_NUM_TIMERS 1

#define DDR_BASE_ADDRESS     0xA000000
#define DDR_SIZE             (1024 * 1024 * 1024)
#define SRAM_BASE_ADDRESS    0x2000000
#define SRAM_SIZE           (64 * 1024)

typedef struct MSF2State {
    /*< private >*/
    SysBusDevice parent_obj;
    /*< public >*/

    char *kernel_filename;
    char *cpu_model;

    MSF2UsartState usart[MSF2_NUM_USARTS];
    MSF2TimerState timer[MSF2_NUM_TIMERS];
} MSF2State;

#endif
