/****************************************************************************
 * arch/risc-v/src/esp32c3-legacy/hardware/esp32c3_systimer.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __ARCH_RISCV_SRC_ESP32C3_LEGACY_HARDWARE_ESP32C3_SYSTIMER_H
#define __ARCH_RISCV_SRC_ESP32C3_LEGACY_HARDWARE_ESP32C3_SYSTIMER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "esp32c3_soc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SYS_TIMER_SYSTIMER_CONF_REG          (DR_REG_SYS_TIMER_BASE + 0x0000)
/* SYS_TIMER_CLK_EN : R/W [31] : 1'b0 ; */

/* Description: register file clk gating */

#define SYS_TIMER_CLK_EN  (BIT(31))
#define SYS_TIMER_CLK_EN_M  (BIT(31))
#define SYS_TIMER_CLK_EN_V  0x1
#define SYS_TIMER_CLK_EN_S  31

/* SYS_TIMER_TIMER_UNIT0_WORK_EN : R/W [30] : 1'b1 ; */

/* Description: timer unit0 work enable */

#define SYS_TIMER_TIMER_UNIT0_WORK_EN  (BIT(30))
#define SYS_TIMER_TIMER_UNIT0_WORK_EN_M  (BIT(30))
#define SYS_TIMER_TIMER_UNIT0_WORK_EN_V  0x1
#define SYS_TIMER_TIMER_UNIT0_WORK_EN_S  30

/* SYS_TIMER_TIMER_UNIT1_WORK_EN : R/W [29] : 1'b0 ; */

/* Description: timer unit1 work enable */

#define SYS_TIMER_TIMER_UNIT1_WORK_EN  (BIT(29))
#define SYS_TIMER_TIMER_UNIT1_WORK_EN_M  (BIT(29))
#define SYS_TIMER_TIMER_UNIT1_WORK_EN_V  0x1
#define SYS_TIMER_TIMER_UNIT1_WORK_EN_S  29

/* SYS_TIMER_TIMER_UNIT0_CORE0_STALL_EN : R/W [28] : 1'b0 ; */

/* Description: If timer unit0 is stalled when core0 stalled */

#define SYS_TIMER_TIMER_UNIT0_CORE0_STALL_EN  (BIT(28))
#define SYS_TIMER_TIMER_UNIT0_CORE0_STALL_EN_M  (BIT(28))
#define SYS_TIMER_TIMER_UNIT0_CORE0_STALL_EN_V  0x1
#define SYS_TIMER_TIMER_UNIT0_CORE0_STALL_EN_S  28

/* SYS_TIMER_TIMER_UNIT0_CORE1_STALL_EN : R/W [27] : 1'b0 ; */

/* Description: If timer unit0 is stalled when core1 stalled */

#define SYS_TIMER_TIMER_UNIT0_CORE1_STALL_EN  (BIT(27))
#define SYS_TIMER_TIMER_UNIT0_CORE1_STALL_EN_M  (BIT(27))
#define SYS_TIMER_TIMER_UNIT0_CORE1_STALL_EN_V  0x1
#define SYS_TIMER_TIMER_UNIT0_CORE1_STALL_EN_S  27

/* SYS_TIMER_TIMER_UNIT1_CORE0_STALL_EN : R/W [26] : 1'b1 ; */

/* Description: If timer unit1 is stalled when core0 stalled */

#define SYS_TIMER_TIMER_UNIT1_CORE0_STALL_EN  (BIT(26))
#define SYS_TIMER_TIMER_UNIT1_CORE0_STALL_EN_M  (BIT(26))
#define SYS_TIMER_TIMER_UNIT1_CORE0_STALL_EN_V  0x1
#define SYS_TIMER_TIMER_UNIT1_CORE0_STALL_EN_S  26

/* SYS_TIMER_TIMER_UNIT1_CORE1_STALL_EN : R/W [25] : 1'b1 ; */

/* Description: If timer unit1 is stalled when core1 stalled */

#define SYS_TIMER_TIMER_UNIT1_CORE1_STALL_EN  (BIT(25))
#define SYS_TIMER_TIMER_UNIT1_CORE1_STALL_EN_M  (BIT(25))
#define SYS_TIMER_TIMER_UNIT1_CORE1_STALL_EN_V  0x1
#define SYS_TIMER_TIMER_UNIT1_CORE1_STALL_EN_S  25

/* SYS_TIMER_TARGET0_WORK_EN : R/W [24] : 1'b0 ; */

/* Description: target0 work enable */

#define SYS_TIMER_TARGET0_WORK_EN  (BIT(24))
#define SYS_TIMER_TARGET0_WORK_EN_M  (BIT(24))
#define SYS_TIMER_TARGET0_WORK_EN_V  0x1
#define SYS_TIMER_TARGET0_WORK_EN_S  24

/* SYS_TIMER_TARGET1_WORK_EN : R/W [23] : 1'b0 ; */

/* Description: target1 work enable */

#define SYS_TIMER_TARGET1_WORK_EN  (BIT(23))
#define SYS_TIMER_TARGET1_WORK_EN_M  (BIT(23))
#define SYS_TIMER_TARGET1_WORK_EN_V  0x1
#define SYS_TIMER_TARGET1_WORK_EN_S  23

/* SYS_TIMER_TARGET2_WORK_EN : R/W [22] : 1'b0 ; */

/* Description: target2 work enable */

#define SYS_TIMER_TARGET2_WORK_EN  (BIT(22))
#define SYS_TIMER_TARGET2_WORK_EN_M  (BIT(22))
#define SYS_TIMER_TARGET2_WORK_EN_V  0x1
#define SYS_TIMER_TARGET2_WORK_EN_S  22

/* SYS_TIMER_SYSTIMER_CLK_FO : R/W [0] : 1'b0 ; */

/* Description: systimer clock force on */

#define SYS_TIMER_SYSTIMER_CLK_FO  (BIT(0))
#define SYS_TIMER_SYSTIMER_CLK_FO_M  (BIT(0))
#define SYS_TIMER_SYSTIMER_CLK_FO_V  0x1
#define SYS_TIMER_SYSTIMER_CLK_FO_S  0

#define SYS_TIMER_SYSTIMER_UNIT0_OP_REG          (DR_REG_SYS_TIMER_BASE + 0x0004)

/* SYS_TIMER_TIMER_UNIT0_UPDATE : WT [30] : 1'b0 ; */

/* Description: update timer_unit0 */

#define SYS_TIMER_TIMER_UNIT0_UPDATE  (BIT(30))
#define SYS_TIMER_TIMER_UNIT0_UPDATE_M  (BIT(30))
#define SYS_TIMER_TIMER_UNIT0_UPDATE_V  0x1
#define SYS_TIMER_TIMER_UNIT0_UPDATE_S  30

/* SYS_TIMER_TIMER_UNIT0_VALUE_VALID : R/SS/WTC [29] : 1'b0 ; */

#define SYS_TIMER_TIMER_UNIT0_VALUE_VALID  (BIT(29))
#define SYS_TIMER_TIMER_UNIT0_VALUE_VALID_M  (BIT(29))
#define SYS_TIMER_TIMER_UNIT0_VALUE_VALID_V  0x1
#define SYS_TIMER_TIMER_UNIT0_VALUE_VALID_S  29

#define SYS_TIMER_SYSTIMER_UNIT1_OP_REG          (DR_REG_SYS_TIMER_BASE + 0x0008)

/* SYS_TIMER_TIMER_UNIT1_UPDATE : WT [30] : 1'b0 ; */

/* Description: update timer unit1 */

#define SYS_TIMER_TIMER_UNIT1_UPDATE  (BIT(30))
#define SYS_TIMER_TIMER_UNIT1_UPDATE_M  (BIT(30))
#define SYS_TIMER_TIMER_UNIT1_UPDATE_V  0x1
#define SYS_TIMER_TIMER_UNIT1_UPDATE_S  30

/* SYS_TIMER_TIMER_UNIT1_VALUE_VALID : R/SS/WTC [29] : 1'b0 ; */

/* Description: timer value is sync and valid */

#define SYS_TIMER_TIMER_UNIT1_VALUE_VALID  (BIT(29))
#define SYS_TIMER_TIMER_UNIT1_VALUE_VALID_M  (BIT(29))
#define SYS_TIMER_TIMER_UNIT1_VALUE_VALID_V  0x1
#define SYS_TIMER_TIMER_UNIT1_VALUE_VALID_S  29

#define SYS_TIMER_SYSTIMER_UNIT0_LOAD_HI_REG          (DR_REG_SYS_TIMER_BASE + 0x000c)

/* SYS_TIMER_TIMER_UNIT0_LOAD_HI : R/W [19:0] : 20'd0 ; */

/* Description: timer unit0 load high 32 bit */

#define SYS_TIMER_TIMER_UNIT0_LOAD_HI  0x000fffff
#define SYS_TIMER_TIMER_UNIT0_LOAD_HI_M  ((SYS_TIMER_TIMER_UNIT0_LOAD_HI_V)<<(SYS_TIMER_TIMER_UNIT0_LOAD_HI_S))
#define SYS_TIMER_TIMER_UNIT0_LOAD_HI_V  0xfffff
#define SYS_TIMER_TIMER_UNIT0_LOAD_HI_S  0

#define SYS_TIMER_SYSTIMER_UNIT0_LOAD_LO_REG          (DR_REG_SYS_TIMER_BASE + 0x0010)

/* SYS_TIMER_TIMER_UNIT0_LOAD_LO : R/W [31:0] : 32'd0 ; */

/* Description: timer unit0 load low 32 bit */

#define SYS_TIMER_TIMER_UNIT0_LOAD_LO  0xffffffff
#define SYS_TIMER_TIMER_UNIT0_LOAD_LO_M  ((SYS_TIMER_TIMER_UNIT0_LOAD_LO_V)<<(SYS_TIMER_TIMER_UNIT0_LOAD_LO_S))
#define SYS_TIMER_TIMER_UNIT0_LOAD_LO_V  0xffffffff
#define SYS_TIMER_TIMER_UNIT0_LOAD_LO_S  0

#define SYS_TIMER_SYSTIMER_UNIT1_LOAD_HI_REG          (DR_REG_SYS_TIMER_BASE + 0x0014)

/* SYS_TIMER_TIMER_UNIT1_LOAD_HI : R/W [19:0] : 20'd0 ; */

/* Description: timer unit1 load high 32 bit */

#define SYS_TIMER_TIMER_UNIT1_LOAD_HI  0x000fffff
#define SYS_TIMER_TIMER_UNIT1_LOAD_HI_M  ((SYS_TIMER_TIMER_UNIT1_LOAD_HI_V)<<(SYS_TIMER_TIMER_UNIT1_LOAD_HI_S))
#define SYS_TIMER_TIMER_UNIT1_LOAD_HI_V  0xfffff
#define SYS_TIMER_TIMER_UNIT1_LOAD_HI_S  0

#define SYS_TIMER_SYSTIMER_UNIT1_LOAD_LO_REG          (DR_REG_SYS_TIMER_BASE + 0x0018)

/* SYS_TIMER_TIMER_UNIT1_LOAD_LO : R/W [31:0] : 32'd0 ; */

/* Description: timer unit1 load low 32 bit */

#define SYS_TIMER_TIMER_UNIT1_LOAD_LO  0xffffffff
#define SYS_TIMER_TIMER_UNIT1_LOAD_LO_M  ((SYS_TIMER_TIMER_UNIT1_LOAD_LO_V)<<(SYS_TIMER_TIMER_UNIT1_LOAD_LO_S))
#define SYS_TIMER_TIMER_UNIT1_LOAD_LO_V  0xffffffff
#define SYS_TIMER_TIMER_UNIT1_LOAD_LO_S  0

#define SYS_TIMER_SYSTIMER_TARGET0_HI_REG          (DR_REG_SYS_TIMER_BASE + 0x001c)

/* SYS_TIMER_TIMER_TARGET0_HI : R/W [19:0] : 20'd0 ; */

/* Description: timer taget0 high 32 bit */

#define SYS_TIMER_TIMER_TARGET0_HI  0x000fffff
#define SYS_TIMER_TIMER_TARGET0_HI_M  ((SYS_TIMER_TIMER_TARGET0_HI_V)<<(SYS_TIMER_TIMER_TARGET0_HI_S))
#define SYS_TIMER_TIMER_TARGET0_HI_V  0xfffff
#define SYS_TIMER_TIMER_TARGET0_HI_S  0

#define SYS_TIMER_SYSTIMER_TARGET0_LO_REG          (DR_REG_SYS_TIMER_BASE + 0x0020)

/* SYS_TIMER_TIMER_TARGET0_LO : R/W [31:0] : 32'd0 ; */

/* Description: timer taget0 low 32 bit */

#define SYS_TIMER_TIMER_TARGET0_LO  0xffffffff
#define SYS_TIMER_TIMER_TARGET0_LO_M  ((SYS_TIMER_TIMER_TARGET0_LO_V)<<(SYS_TIMER_TIMER_TARGET0_LO_S))
#define SYS_TIMER_TIMER_TARGET0_LO_V  0xffffffff
#define SYS_TIMER_TIMER_TARGET0_LO_S  0

#define SYS_TIMER_SYSTIMER_TARGET1_HI_REG          (DR_REG_SYS_TIMER_BASE + 0x0024)

/* SYS_TIMER_TIMER_TARGET1_HI : R/W [19:0] : 20'd0 ; */

/* Description: timer taget1 high 32 bit */

#define SYS_TIMER_TIMER_TARGET1_HI  0x000fffff
#define SYS_TIMER_TIMER_TARGET1_HI_M  ((SYS_TIMER_TIMER_TARGET1_HI_V)<<(SYS_TIMER_TIMER_TARGET1_HI_S))
#define SYS_TIMER_TIMER_TARGET1_HI_V  0xfffff
#define SYS_TIMER_TIMER_TARGET1_HI_S  0

#define SYS_TIMER_SYSTIMER_TARGET1_LO_REG          (DR_REG_SYS_TIMER_BASE + 0x0028)

/* SYS_TIMER_TIMER_TARGET1_LO : R/W [31:0] : 32'd0 ; */

/* Description: timer taget1 low 32 bit */

#define SYS_TIMER_TIMER_TARGET1_LO  0xffffffff
#define SYS_TIMER_TIMER_TARGET1_LO_M  ((SYS_TIMER_TIMER_TARGET1_LO_V)<<(SYS_TIMER_TIMER_TARGET1_LO_S))
#define SYS_TIMER_TIMER_TARGET1_LO_V  0xffffffff
#define SYS_TIMER_TIMER_TARGET1_LO_S  0

#define SYS_TIMER_SYSTIMER_TARGET2_HI_REG          (DR_REG_SYS_TIMER_BASE + 0x002c)

/* SYS_TIMER_TIMER_TARGET2_HI : R/W [19:0] : 20'd0 ; */

/* Description: timer taget2 high 32 bit */

#define SYS_TIMER_TIMER_TARGET2_HI  0x000fffff
#define SYS_TIMER_TIMER_TARGET2_HI_M  ((SYS_TIMER_TIMER_TARGET2_HI_V)<<(SYS_TIMER_TIMER_TARGET2_HI_S))
#define SYS_TIMER_TIMER_TARGET2_HI_V  0xfffff
#define SYS_TIMER_TIMER_TARGET2_HI_S  0

#define SYS_TIMER_SYSTIMER_TARGET2_LO_REG          (DR_REG_SYS_TIMER_BASE + 0x0030)

/* SYS_TIMER_TIMER_TARGET2_LO : R/W [31:0] : 32'd0 ; */

/* Description: timer taget2 low 32 bit */

#define SYS_TIMER_TIMER_TARGET2_LO  0xffffffff
#define SYS_TIMER_TIMER_TARGET2_LO_M  ((SYS_TIMER_TIMER_TARGET2_LO_V)<<(SYS_TIMER_TIMER_TARGET2_LO_S))
#define SYS_TIMER_TIMER_TARGET2_LO_V  0xffffffff
#define SYS_TIMER_TIMER_TARGET2_LO_S  0

#define SYS_TIMER_SYSTIMER_TARGET0_CONF_REG          (DR_REG_SYS_TIMER_BASE + 0x0034)

/* SYS_TIMER_TARGET0_TIMER_UNIT_SEL : R/W [31] : 1'b0 ; */

/* Description: select which unit to compare */

#define SYS_TIMER_TARGET0_TIMER_UNIT_SEL  (BIT(31))
#define SYS_TIMER_TARGET0_TIMER_UNIT_SEL_M  (BIT(31))
#define SYS_TIMER_TARGET0_TIMER_UNIT_SEL_V  0x1
#define SYS_TIMER_TARGET0_TIMER_UNIT_SEL_S  31

/* SYS_TIMER_TARGET0_PERIOD_MODE : R/W [30] : 1'b0 ; */

/* Description: Set target0 to period mode */

#define SYS_TIMER_TARGET0_PERIOD_MODE  (BIT(30))
#define SYS_TIMER_TARGET0_PERIOD_MODE_M  (BIT(30))
#define SYS_TIMER_TARGET0_PERIOD_MODE_V  0x1
#define SYS_TIMER_TARGET0_PERIOD_MODE_S  30

/* SYS_TIMER_TARGET0_PERIOD : R/W [25:0] : 26'h0 ; */

/* Description: target0 period */

#define SYS_TIMER_TARGET0_PERIOD  0x03ffffff
#define SYS_TIMER_TARGET0_PERIOD_M  ((SYS_TIMER_TARGET0_PERIOD_V)<<(SYS_TIMER_TARGET0_PERIOD_S))
#define SYS_TIMER_TARGET0_PERIOD_V  0x3ffffff
#define SYS_TIMER_TARGET0_PERIOD_S  0

#define SYS_TIMER_SYSTIMER_TARGET1_CONF_REG          (DR_REG_SYS_TIMER_BASE + 0x0038)

/* SYS_TIMER_TARGET1_TIMER_UNIT_SEL : R/W [31] : 1'b0 ; */

/* Description: select which unit to compare */

#define SYS_TIMER_TARGET1_TIMER_UNIT_SEL  (BIT(31))
#define SYS_TIMER_TARGET1_TIMER_UNIT_SEL_M  (BIT(31))
#define SYS_TIMER_TARGET1_TIMER_UNIT_SEL_V  0x1
#define SYS_TIMER_TARGET1_TIMER_UNIT_SEL_S  31

/* SYS_TIMER_TARGET1_PERIOD_MODE : R/W [30] : 1'b0 ; */

/* Description: Set target1 to period mode */

#define SYS_TIMER_TARGET1_PERIOD_MODE  (BIT(30))
#define SYS_TIMER_TARGET1_PERIOD_MODE_M  (BIT(30))
#define SYS_TIMER_TARGET1_PERIOD_MODE_V  0x1
#define SYS_TIMER_TARGET1_PERIOD_MODE_S  30

/* SYS_TIMER_TARGET1_PERIOD : R/W [25:0] : 26'h0 ; */

/* Description: target1 period */

#define SYS_TIMER_TARGET1_PERIOD  0x03ffffff
#define SYS_TIMER_TARGET1_PERIOD_M  ((SYS_TIMER_TARGET1_PERIOD_V)<<(SYS_TIMER_TARGET1_PERIOD_S))
#define SYS_TIMER_TARGET1_PERIOD_V  0x3ffffff
#define SYS_TIMER_TARGET1_PERIOD_S  0

#define SYS_TIMER_SYSTIMER_TARGET2_CONF_REG          (DR_REG_SYS_TIMER_BASE + 0x003c)

/* SYS_TIMER_TARGET2_TIMER_UNIT_SEL : R/W [31] : 1'b0 ; */

/* Description: select which unit to compare */

#define SYS_TIMER_TARGET2_TIMER_UNIT_SEL  (BIT(31))
#define SYS_TIMER_TARGET2_TIMER_UNIT_SEL_M  (BIT(31))
#define SYS_TIMER_TARGET2_TIMER_UNIT_SEL_V  0x1
#define SYS_TIMER_TARGET2_TIMER_UNIT_SEL_S  31

/* SYS_TIMER_TARGET2_PERIOD_MODE : R/W [30] : 1'b0 ; */

/* Description: Set target2 to period mode */

#define SYS_TIMER_TARGET2_PERIOD_MODE  (BIT(30))
#define SYS_TIMER_TARGET2_PERIOD_MODE_M  (BIT(30))
#define SYS_TIMER_TARGET2_PERIOD_MODE_V  0x1
#define SYS_TIMER_TARGET2_PERIOD_MODE_S  30

/* SYS_TIMER_TARGET2_PERIOD : R/W [25:0] : 26'h0 ; */

/* Description: target2 period */

#define SYS_TIMER_TARGET2_PERIOD  0x03ffffff
#define SYS_TIMER_TARGET2_PERIOD_M  ((SYS_TIMER_TARGET2_PERIOD_V)<<(SYS_TIMER_TARGET2_PERIOD_S))
#define SYS_TIMER_TARGET2_PERIOD_V  0x3ffffff
#define SYS_TIMER_TARGET2_PERIOD_S  0

#define SYS_TIMER_SYSTIMER_UNIT0_VALUE_HI_REG          (DR_REG_SYS_TIMER_BASE + 0x0040)

/* SYS_TIMER_TIMER_UNIT0_VALUE_HI : RO [19:0] : 20'd0 ; */

/* Description: timer read value high 32bit */

#define SYS_TIMER_TIMER_UNIT0_VALUE_HI  0x000fffff
#define SYS_TIMER_TIMER_UNIT0_VALUE_HI_M  ((SYS_TIMER_TIMER_UNIT0_VALUE_HI_V)<<(SYS_TIMER_TIMER_UNIT0_VALUE_HI_S))
#define SYS_TIMER_TIMER_UNIT0_VALUE_HI_V  0xfffff
#define SYS_TIMER_TIMER_UNIT0_VALUE_HI_S  0

#define SYS_TIMER_SYSTIMER_UNIT0_VALUE_LO_REG          (DR_REG_SYS_TIMER_BASE + 0x0044)

/* SYS_TIMER_TIMER_UNIT0_VALUE_LO : RO [31:0] : 32'd0 ; */

/* Description: timer read value low 32bit */

#define SYS_TIMER_TIMER_UNIT0_VALUE_LO  0xffffffff
#define SYS_TIMER_TIMER_UNIT0_VALUE_LO_M  ((SYS_TIMER_TIMER_UNIT0_VALUE_LO_V)<<(SYS_TIMER_TIMER_UNIT0_VALUE_LO_S))
#define SYS_TIMER_TIMER_UNIT0_VALUE_LO_V  0xffffffff
#define SYS_TIMER_TIMER_UNIT0_VALUE_LO_S  0

#define SYS_TIMER_SYSTIMER_UNIT1_VALUE_HI_REG          (DR_REG_SYS_TIMER_BASE + 0x0048)

/* SYS_TIMER_TIMER_UNIT1_VALUE_HI : RO [19:0] : 20'd0 ; */

/* Description: timer read value high 32bit */

#define SYS_TIMER_TIMER_UNIT1_VALUE_HI  0x000fffff
#define SYS_TIMER_TIMER_UNIT1_VALUE_HI_M  ((SYS_TIMER_TIMER_UNIT1_VALUE_HI_V)<<(SYS_TIMER_TIMER_UNIT1_VALUE_HI_S))
#define SYS_TIMER_TIMER_UNIT1_VALUE_HI_V  0xfffff
#define SYS_TIMER_TIMER_UNIT1_VALUE_HI_S  0

#define SYS_TIMER_SYSTIMER_UNIT1_VALUE_LO_REG          (DR_REG_SYS_TIMER_BASE + 0x004c)

/* SYS_TIMER_TIMER_UNIT1_VALUE_LO : RO [31:0] : 32'd0 ; */

/* Description: timer read value low 32bit */

#define SYS_TIMER_TIMER_UNIT1_VALUE_LO  0xffffffff
#define SYS_TIMER_TIMER_UNIT1_VALUE_LO_M  ((SYS_TIMER_TIMER_UNIT1_VALUE_LO_V)<<(SYS_TIMER_TIMER_UNIT1_VALUE_LO_S))
#define SYS_TIMER_TIMER_UNIT1_VALUE_LO_V  0xffffffff
#define SYS_TIMER_TIMER_UNIT1_VALUE_LO_S  0

#define SYS_TIMER_SYSTIMER_COMP0_LOAD_REG          (DR_REG_SYS_TIMER_BASE + 0x0050)

/* SYS_TIMER_TIMER_COMP0_LOAD : WT [0] : 1'b0 ; */

/* Description: timer comp0 load value */

#define SYS_TIMER_TIMER_COMP0_LOAD  (BIT(0))
#define SYS_TIMER_TIMER_COMP0_LOAD_M  (BIT(0))
#define SYS_TIMER_TIMER_COMP0_LOAD_V  0x1
#define SYS_TIMER_TIMER_COMP0_LOAD_S  0

#define SYS_TIMER_SYSTIMER_COMP1_LOAD_REG          (DR_REG_SYS_TIMER_BASE + 0x0054)

/* SYS_TIMER_TIMER_COMP1_LOAD : WT [0] : 1'b0 ; */

/* Description: timer comp1 load value */

#define SYS_TIMER_TIMER_COMP1_LOAD  (BIT(0))
#define SYS_TIMER_TIMER_COMP1_LOAD_M  (BIT(0))
#define SYS_TIMER_TIMER_COMP1_LOAD_V  0x1
#define SYS_TIMER_TIMER_COMP1_LOAD_S  0

#define SYS_TIMER_SYSTIMER_COMP2_LOAD_REG          (DR_REG_SYS_TIMER_BASE + 0x0058)

/* SYS_TIMER_TIMER_COMP2_LOAD : WT [0] : 1'b0 ; */

/* Description: timer comp2 load value */

#define SYS_TIMER_TIMER_COMP2_LOAD  (BIT(0))
#define SYS_TIMER_TIMER_COMP2_LOAD_M  (BIT(0))
#define SYS_TIMER_TIMER_COMP2_LOAD_V  0x1
#define SYS_TIMER_TIMER_COMP2_LOAD_S  0

#define SYS_TIMER_SYSTIMER_UNIT0_LOAD_REG          (DR_REG_SYS_TIMER_BASE + 0x005c)

/* SYS_TIMER_TIMER_UNIT0_LOAD : WT [0] : 1'b0 ; */

/* Description: timer unit0 load value */

#define SYS_TIMER_TIMER_UNIT0_LOAD  (BIT(0))
#define SYS_TIMER_TIMER_UNIT0_LOAD_M  (BIT(0))
#define SYS_TIMER_TIMER_UNIT0_LOAD_V  0x1
#define SYS_TIMER_TIMER_UNIT0_LOAD_S  0

#define SYS_TIMER_SYSTIMER_UNIT1_LOAD_REG          (DR_REG_SYS_TIMER_BASE + 0x0060)

/* SYS_TIMER_TIMER_UNIT1_LOAD : WT [0] : 1'b0 ; */

/* Description: timer unit1 load value */

#define SYS_TIMER_TIMER_UNIT1_LOAD  (BIT(0))
#define SYS_TIMER_TIMER_UNIT1_LOAD_M  (BIT(0))
#define SYS_TIMER_TIMER_UNIT1_LOAD_V  0x1
#define SYS_TIMER_TIMER_UNIT1_LOAD_S  0

#define SYS_TIMER_SYSTIMER_INT_ENA_REG          (DR_REG_SYS_TIMER_BASE + 0x0064)

/* SYS_TIMER_TARGET2_INT_ENA : R/W [2] : 1'b0 ; */

/* Description: interupt2 enable */

#define SYS_TIMER_TARGET2_INT_ENA  (BIT(2))
#define SYS_TIMER_TARGET2_INT_ENA_M  (BIT(2))
#define SYS_TIMER_TARGET2_INT_ENA_V  0x1
#define SYS_TIMER_TARGET2_INT_ENA_S  2

/* SYS_TIMER_TARGET1_INT_ENA : R/W [1] : 1'b0 ; */

/* Description: interupt1 enable */

#define SYS_TIMER_TARGET1_INT_ENA  (BIT(1))
#define SYS_TIMER_TARGET1_INT_ENA_M  (BIT(1))
#define SYS_TIMER_TARGET1_INT_ENA_V  0x1
#define SYS_TIMER_TARGET1_INT_ENA_S  1

/* SYS_TIMER_TARGET0_INT_ENA : R/W [0] : 1'b0 ; */

/* Description: interupt0 enable */

#define SYS_TIMER_TARGET0_INT_ENA  (BIT(0))
#define SYS_TIMER_TARGET0_INT_ENA_M  (BIT(0))
#define SYS_TIMER_TARGET0_INT_ENA_V  0x1
#define SYS_TIMER_TARGET0_INT_ENA_S  0

#define SYS_TIMER_SYSTIMER_INT_RAW_REG          (DR_REG_SYS_TIMER_BASE + 0x0068)

/* SYS_TIMER_TARGET2_INT_RAW : R/WTC/SS [2] : 1'b0 ; */

/* Description: interupt2 raw */

#define SYS_TIMER_TARGET2_INT_RAW  (BIT(2))
#define SYS_TIMER_TARGET2_INT_RAW_M  (BIT(2))
#define SYS_TIMER_TARGET2_INT_RAW_V  0x1
#define SYS_TIMER_TARGET2_INT_RAW_S  2

/* SYS_TIMER_TARGET1_INT_RAW : R/WTC/SS [1] : 1'b0 ; */

/* Description: interupt1 raw */

#define SYS_TIMER_TARGET1_INT_RAW  (BIT(1))
#define SYS_TIMER_TARGET1_INT_RAW_M  (BIT(1))
#define SYS_TIMER_TARGET1_INT_RAW_V  0x1
#define SYS_TIMER_TARGET1_INT_RAW_S  1

/* SYS_TIMER_TARGET0_INT_RAW : R/WTC/SS [0] : 1'b0 ; */

/* Description: interupt0 raw */

#define SYS_TIMER_TARGET0_INT_RAW  (BIT(0))
#define SYS_TIMER_TARGET0_INT_RAW_M  (BIT(0))
#define SYS_TIMER_TARGET0_INT_RAW_V  0x1
#define SYS_TIMER_TARGET0_INT_RAW_S  0

#define SYS_TIMER_SYSTIMER_INT_CLR_REG          (DR_REG_SYS_TIMER_BASE + 0x006c)

/* SYS_TIMER_TARGET2_INT_CLR : WT [2] : 1'b0 ; */

/* Description: interupt2 clear */

#define SYS_TIMER_TARGET2_INT_CLR  (BIT(2))
#define SYS_TIMER_TARGET2_INT_CLR_M  (BIT(2))
#define SYS_TIMER_TARGET2_INT_CLR_V  0x1
#define SYS_TIMER_TARGET2_INT_CLR_S  2

/* SYS_TIMER_TARGET1_INT_CLR : WT [1] : 1'b0 ; */

/* Description: interupt1 clear */

#define SYS_TIMER_TARGET1_INT_CLR  (BIT(1))
#define SYS_TIMER_TARGET1_INT_CLR_M  (BIT(1))
#define SYS_TIMER_TARGET1_INT_CLR_V  0x1
#define SYS_TIMER_TARGET1_INT_CLR_S  1

/* SYS_TIMER_TARGET0_INT_CLR : WT [0] : 1'b0 ; */

/* Description: interupt0 clear */

#define SYS_TIMER_TARGET0_INT_CLR  (BIT(0))
#define SYS_TIMER_TARGET0_INT_CLR_M  (BIT(0))
#define SYS_TIMER_TARGET0_INT_CLR_V  0x1
#define SYS_TIMER_TARGET0_INT_CLR_S  0

#define SYS_TIMER_SYSTIMER_INT_ST_REG          (DR_REG_SYS_TIMER_BASE + 0x0070)

/* SYS_TIMER_TARGET2_INT_ST : RO [2] : 1'b0 ; */

#define SYS_TIMER_TARGET2_INT_ST  (BIT(2))
#define SYS_TIMER_TARGET2_INT_ST_M  (BIT(2))
#define SYS_TIMER_TARGET2_INT_ST_V  0x1
#define SYS_TIMER_TARGET2_INT_ST_S  2

/* SYS_TIMER_TARGET1_INT_ST : RO [1] : 1'b0 ; */

#define SYS_TIMER_TARGET1_INT_ST  (BIT(1))
#define SYS_TIMER_TARGET1_INT_ST_M  (BIT(1))
#define SYS_TIMER_TARGET1_INT_ST_V  0x1
#define SYS_TIMER_TARGET1_INT_ST_S  1

/* SYS_TIMER_TARGET0_INT_ST : RO [0] : 1'b0 ; */

#define SYS_TIMER_TARGET0_INT_ST  (BIT(0))
#define SYS_TIMER_TARGET0_INT_ST_M  (BIT(0))
#define SYS_TIMER_TARGET0_INT_ST_V  0x1
#define SYS_TIMER_TARGET0_INT_ST_S  0

#define SYS_TIMER_SYSTIMER_DATE_REG          (DR_REG_SYS_TIMER_BASE + 0x00fc)

/* SYS_TIMER_DATE : R/W [31:0] : 28'h2006171 ; */

#define SYS_TIMER_DATE  0xffffffff
#define SYS_TIMER_DATE_M  ((SYS_TIMER_DATE_V)<<(SYS_TIMER_DATE_S))
#define SYS_TIMER_DATE_V  0xffffffff
#define SYS_TIMER_DATE_S  0

#endif /* __ARCH_RISCV_SRC_ESP32C3_LEGACY_HARDWARE_ESP32C3_SYSTIMER_H */
