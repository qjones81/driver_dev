/* --COPYRIGHT--,BSD
 * Copyright (c) 2015, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/

#ifndef _DRV8323_H_
#define _DRV8323_H_

// **************************************************************************
// the includes

#include "stdbool.h"
#include "stdint.h"

// drivers

#include "stm32f4xx_hal.h"

// Port
typedef SPI_HandleTypeDef* SPI_Handle;
typedef GPIO_TypeDef* GPIO_Handle;
typedef uint16_t GPIO_Number_e;


//!
//! \defgroup DRV8323

//!
//! \ingroup DRV8323
//@{


#ifdef __cplusplus
extern "C" {
#endif


// **************************************************************************
// the defines

//! \brief Defines the address mask
//!
#define DRV8323_ADDR_MASK               (0x7800)


//! \brief Defines the data mask
//!
#define DRV8323_DATA_MASK               (0x07FF)


//! \brief Defines the R/W mask
//!
#define DRV8323_RW_MASK                 (0x8000)


//! \brief Defines the R/W mask
//!
#define DRV8323_FAULT_TYPE_MASK         (0x07FF)


//! \brief Fault 1 Status Register Address
#define DRV8323_STATUS1_ADDR 			(0x00)

//! \brief Fault 2 Status Register Address
#define DRV8323_STATUS2_ADDR 			(0x01)

//! \brief Driver Control Register Address
#define DRV8323_DRIVER_CTRL_ADDR 			(0x02)

//! \brief Gate Drive HS Control Register Address
#define DRV8323_GATE_DRIVE_HS_ADDR 			(0x03)

//! \brief Gate Drive LS Control Register Address
#define DRV8323_GATE_DRIVE_LS_ADDR 			(0x04)

//! \brief OCP Control Register Address
#define DRV8323_OCP_CTRL_ADDR 			(0x05)

//! \brief CSA Control Register Address
#define DRV8323_CSA_CTRL_ADDR 			(0x06)


//! \brief Defines the location of the VDS_LC (VDS Over Current fault on the C low-side MOSFET) bits in the Status 1 register
//!
#define DRV8323_STATUS1_VDS_LC_BITS   (1 << 0)

//! \brief Defines the location of the VDS_HC (VDS Over Current fault on the C high-side MOSFET) bits in the Status 1 register
//!
#define DRV8323_STATUS1_VDS_HC_BITS   (1 << 1)

//! \brief Defines the location of the VDS_LB (VDS Over Current fault on the B high-side MOSFET) bits in the Status 1 register
//!
#define DRV8323_STATUS1_VDS_LB_BITS   (1 << 2)

//! \brief Defines the location of the VDS_HB (VDS Over Current fault on the B high-side MOSFET) bits in the Status 1 register
//!
#define DRV8323_STATUS1_VDS_HB_BITS   (1 << 3)

//! \brief Defines the location of the VDS_LA (VDS Over Current fault on the A high-side MOSFET) bits in the Status 1 register
//!
#define DRV8323_STATUS1_VDS_LA_BITS   (1 << 4)

//! \brief Defines the location of the VDS_HA (VDS Over Current fault on the A high-side MOSFET) bits in the Status 1 register
//!
#define DRV8323_STATUS1_VDS_HA_BITS   (1 << 5)

//! \brief Defines the location of the OTSD (Over Temperature Shut Down) bits in the Status 1 register
//!
#define DRV8323_STATUS1_OTSD_BITS   (1 << 6)

//! \brief Defines the location of the UVLO (Under Voltage Lockout) bits in the Status 1 register
//!
#define DRV8323_STATUS1_UVLO_BITS   (1 << 7)

//! \brief Defines the location of the GDF (Gate Drive Fault) bits in the Status 1 register
//!
#define DRV8323_STATUS1_GDF_BITS   (1 << 8)

//! \brief Defines the location of the VDS_OCP (VDS monitor Over Current) bits in the Status 1 register
//!
#define DRV8323_STATUS1_VDS_OCP_BITS   (1 << 9)

//! \brief Defines the location of the FAULT bits in the Status 1 register
//!
#define DRV8323_STATUS1_FAULT_BITS   (1 << 10)



//! \brief Defines the location of the VGS_LC (VGS Gate Drive Fault on the C low-side MOSFET) bits in the Status 2 register
//!
#define DRV8323_STATUS2_VGS_LC_BITS   (1 << 0)

//! \brief Defines the location of the VGS_HC (VGS Gate Drive Fault on the C high-side MOSFET) bits in the Status 2 register
//!
#define DRV8323_STATUS2_VGS_HC_BITS   (1 << 1)

//! \brief Defines the location of the VGS_LB (VGS Gate Drive Fault on the B low-side MOSFET) bits in the Status 2 register
//!
#define DRV8323_STATUS2_VGS_LB_BITS   (1 << 2)

//! \brief Defines the location of the VGS_HB (VGS Gate Drive Fault on the B high-side MOSFET) bits in the Status 2 register
//!
#define DRV8323_STATUS2_VGS_HB_BITS   (1 << 3)

//! \brief Defines the location of the VGS_LA (VGS Gate Drive Fault on the A low-side MOSFET) bits in the Status 2 register
//!
#define DRV8323_STATUS2_VGS_LA_BITS   (1 << 4)

//! \brief Defines the location of the VGS_HA (VGS Gate Drive Fault on the A high-side MOSFET) bits in the Status 2 register
//!
#define DRV8323_STATUS2_VGS_HA_BITS   (1 << 5)

//! \brief Defines the location of the CPUV (Charge Pump Under Voltage Condition) bits in the Status 2 register
//!
#define DRV8323_STATUS2_CPUV_BITS   (1 << 6)

//! \brief Defines the location of the OTW (Over Temperature Warning) bits in the Status 2 register
//!
#define DRV8323_STATUS2_OTW_BITS   (1 << 7)

//! \brief Defines the location of the SC_OC (Over Current on phase C sense amplifier (DRV8323xS)) bits in the Status 2 register
//!
#define DRV8323_STATUS2_SC_OC_BITS   (1 << 8)

//! \brief Defines the location of the SC_OB (Over Current on phase B sense amplifier (DRV8323xS)) bits in the Status 2 register
//!
#define DRV8323_STATUS2_SB_OC_BITS   (1 << 9)

//! \brief Defines the location of the SC_OA (Over Current on phase A sense amplifier (DRV8323xS)) bits in the Status 2 register
//!
#define DRV8323_STATUS2_SA_OC_BITS   (1 << 10)



//! \brief Defines the location of the CLR_FLT (Latched Faults) bits in the Driver Control  register
//!
#define DRV8323_DRIVER_CTRL_CLR_FLT_BITS  (1 << 0)

//! \brief Defines the location of the BRAKE (Low-Side MOSFETs -> 1x PWM) bits in the Driver Control register
//!
#define DRV8323_DRIVER_CTRL_BRAKE_BITS  (1 << 1)

//! \brief Defines the location of the COAST (MOSFETs -> HI-Z) bits in the Driver Control register
//!
#define DRV8323_DRIVER_CTRL_COAST_BITS  (1 << 2)

//! \brief Defines the location of the 1PWM_DIR bits in the Driver Control register
//!
#define DRV8323_DRIVER_CTRL_1PWM_DIR_BITS  (1 << 3)

//! \brief Defines the location of the 1PWM_COM bits in the Driver Control register
//!
#define DRV8323_DRIVER_CTRL_1PWM_COM_BITS  (1 << 4)

//! \brief Defines the location of the PWM_MODE (00b = 6x PWM, 01b = 3x PWM, 10b = 1x PWM, 11b = Independent) bits in the Driver Control register
//!
#define DRV8323_DRIVER_CTRL_PWM_MODE_BITS  (3 << 5)

//! \brief Defines the location of the OTW_REP (0b = OTW not reported on nFAULT or FAULT bit) bits in the Driver Control register
//!
#define DRV8323_DRIVER_CTRL_OTW_REP_BITS  (1 << 7)

//! \brief Defines the location of the DIS_GDF (0b = Gate Drive Fault Enabled, 1b = Gate Drive Fault Disabled) bits in the Driver Control register
//!
#define DRV8323_DRIVER_CTRL_DIS_GDF_BITS  (1 << 8)

//! \brief Defines the location of the DIS_CPUV (0b = Charge Pump UVLO Fault Enabled, 1b = Charge Pump UVLO Disabled) bits in the Driver Control register
//!
#define DRV8323_DRIVER_CTRL_DIS_CPUV_BITS  (1 << 9)

//! \brief Defines the location of the RESERVED (Reserved) bits in the Driver Control register
//!
#define DRV8323_DRIVER_CTRL_RESERVED_BITS  (1 << 10)



//! \brief Defines the location of the IDRIVEN_HS (0000b = 10 mA
//!0000b = 20 mA
//!0001b = 60 mA
//!0010b = 120 mA
//!0011b = 160 mA
//!0100b = 240 mA
//!0101b = 280 mA
//!0110b = 340 mA
//!0111b = 380 mA
//!1000b = 520 mA
//!1001b = 660 mA
//!1010b = 740 mA
//!1011b = 880 mA
//!1100b = 1140 mA
//!1101b = 1360 mA
//!1110b = 1640 mA
//!**1111b = 2000 mA) bits in the Gate Drive HS register
//!
#define DRV8323_GATE_DRIVE_HS_IDRIVEN_HS_BITS  (16 << 0)

//! \brief Defines the location of the IDRIVEP_HS (0000b = 10 mA
//!0001b = 30 mA
//!0010b = 60 mA
//!0011b = 80 mA
//!0100b = 120 mA
//!0101b = 140 mA
//!0110b = 170 mA
//!0111b = 190 mA
//!1000b = 260 mA
//!1001b = 330 mA
//!1010b = 370 mA
//!1011b = 440 mA
//!1100b = 570 mA
//!1101b = 680 mA
//!1110b = 820 mA
//!**1111b = 1000 mA) bits in the Gate Drive HS register
//!
#define DRV8323_GATE_DRIVE_HS_IDRIVEP_HS_BITS  (16 << 4)

//! \brief Defines the location of the LOCK (110b = Register Writes Disabled/Locked, 011b = Register Writes Enabled/Unlocked) bits in the Gate Drive HS register
//!
#define DRV8323_GATE_DRIVE_HS_LOCK_BITS  (8 << 8)



//! \brief Defines the location of the IDRIVEN_LS (0000b = 10 mA
//!0000b = 20 mA
//!0001b = 60 mA
//!0010b = 120 mA
//!0011b = 160 mA
//!0100b = 240 mA
//!0101b = 280 mA
//!0110b = 340 mA
//!0111b = 380 mA
//!1000b = 520 mA
//!1001b = 660 mA
//!1010b = 740 mA
//!1011b = 880 mA
//!1100b = 1140 mA
//!1101b = 1360 mA
//!1110b = 1640 mA
//!**1111b = 2000 mA) bits in the Gate Drive LS register
//!
#define DRV8323_GATE_DRIVE_LS_IDRIVEN_LS_BITS  (16 << 0)

//! \brief Defines the location of the IDRIVEP_LS (0000b = 10 mA
//!0001b = 30 mA
//!0010b = 60 mA
//!0011b = 80 mA
//!0100b = 120 mA
//!0101b = 140 mA
//!0110b = 170 mA
//!0111b = 190 mA
//!1000b = 260 mA
//!1001b = 330 mA
//!1010b = 370 mA
//!1011b = 440 mA
//!1100b = 570 mA
//!1101b = 680 mA
//!1110b = 820 mA
//!**1111b = 1000 mA) bits in the Gate Drive LS register
//!
#define DRV8323_GATE_DRIVE_LS_IDRIVEP_LS_BITS  (16 << 4)

//! \brief Defines the location of the TDRIVE (00b = 500-ns peak gate-current drive time
//!01b = 1000-ns peak gate-current drive time
//!10b = 2000-ns peak gate-current drive time
//!**11b = 4000-ns peak gate-current drive time) bits in the Gate Drive LS register
//!
#define DRV8323_GATE_DRIVE_LS_TDRIVE_BITS  (3 << 8)


//! \brief Defines the location of the CBC bits in the Gate Drive LS register
//!In retry OCP_MODE, for both VDS_OCP and SEN_OCP, the
//!fault is automatically cleared when a PWM input is given
//!
#define DRV8323_GATE_DRIVE_LS_CBC_BITS    (1 << 10)




//! \brief Defines the location of the VDS_LVL(0000b = 0.06 V
//!0001b = 0.13 V
//!0010b = 0.2 V
//!0011b = 0.26 V
//!0100b = 0.31 V
//!0101b = 0.45 V
//!0110b = 0.53 V
//!0111b = 0.6 V
//!1000b = 0.68 V
//**!1001b = 0.75 V
//!1010b = 0.94 V
//!1011b = 1.13 V
//!1100b = 1.3 V
//!1101b = 1.5 V
//!1110b = 1.7 V
//!1111b = 1.88 V) bits in the OCP Control register
//!
#define DRV8323_OCP_CTRL_VDS_LVL_BITS    (16 << 0)

//! \brief Defines the location of the OCP_DEG(00b = Overcurrent deglitch of 2 탎
//**!01b = Overcurrent deglitch of 4 탎
//!10b = Overcurrent deglitch of 6 탎
//!11b = Overcurrent deglitch of 8 탎) bits in the OCP Control register
//!
#define DRV8323_OCP_CTRL_OCP_DEG_BITS    (3 << 4)

//! \brief Defines the location of the OCP_MODE(00b = Overcurrent causes a latched fault
//**!01b = Overcurrent causes an automatic retrying fault
//!10b = Overcurrent is report only but no action is taken
//!11b = Overcurrent is not reported and no action is taken) bits in the OCP Control register
//!
#define DRV8323_OCP_CTRL_OCP_MODE_BITS    (3 << 6)

//! \brief Defines the location of the DEAD_TIME(00b = 50-ns dead time
//**!01b = 100-ns dead time
//!10b = 200-ns dead time
//!11b = 400-ns dead time) bits in the OCP Control register
//!
#define DRV8323_OCP_CTRL_DEAD_TIME_BITS    (3 << 8)

//! \brief Defines the location of the TRETRY(0b = VDS_OCP and SEN_OCP retry time is 4 ms
//! 1b = VDS_OCP and SEN_OCP retry time is 50 탎) bits in the OCP Control register
//!
#define DRV8323_OCP_CTRL_TRETRY_BITS    (1 << 10)



//! \brief Defines the location of the SEN_LVL(00b = Sense OCP 0.25 V
//!01b = Sense OCP 0.5 V
//!10b = Sense OCP 0.75 V
//!**11b = Sense OCP 1 V) bits in the CSA Control register
//!
#define DRV8323_CSA_CTRL_SEN_LVL_BITS    (3 << 0)

//! \brief Defines the location of the CSA_CAL_C(**0b = Normal sense amplifier C operation
//!1b = Short inputs to sense amplifier C for offset calibration) bits in the CSA Control register
//!
#define DRV8323_CSA_CTRL_CSA_CAL_C_BITS    (1 << 2)

//! \brief Defines the location of the CSA_CAL_B(**0b = Normal sense amplifier B operation
//!1b = Short inputs to sense amplifier B for offset calibration) bits in the CSA Control register
//!
#define DRV8323_CSA_CTRL_CSA_CAL_B_BITS    (1 << 3)

//! \brief Defines the location of the CSA_CAL_A(**0b = Normal sense amplifier A operation
//!1b = Short inputs to sense amplifier A for offset calibration) bits in the CSA Control register
//!
#define DRV8323_CSA_CTRL_CSA_CAL_A_BITS    (1 << 4)

//! \brief Defines the location of the DIS_SEN(**0b = Sense overcurrent fault is enabled
//!1b = Sense overcurrent fault is disabled) bits in the CSA Control register
//!
#define DRV8323_CSA_CTRL_DIS_SEN_BITS    (1 << 5)

//! \brief Defines the location of the CSA_GAIN(00b = 5-V/V shunt amplifier gain
//!01b = 10-V/V shunt amplifier gain
//**!10b = 20-V/V shunt amplifier gain
//!11b = 40-V/V shunt amplifier gain) bits in the CSA Control register
//!
#define DRV8323_CSA_CTRL_CSA_GAIN_BITS    (3 << 6)

//! \brief Defines the location of the LS_REF(**0b = VDS_OCP for the low-side MOSFET is measured across SHx to SPx
//!1b = VDS_OCP for the low-side MOSFET is measured across SHx to SNx) bits in the CSA Control register
//!
#define DRV8323_CSA_CTRL_LS_REF_BITS    (1 << 8)

//! \brief Defines the location of the VREF_DIV(0b = Sense amplifier reference voltage is VREF (unidirectionalmode)
//!**1b = Sense amplifier reference voltage is VREF divided by 2) bits in the CSA Control register
//!
#define DRV8323_CSA_CTRL_VREF_DIV_BITS    (1 << 9)

//! \brief Defines the location of the CSA_FET(**0b = Sense amplifier positive input is SPx
//!1b = Sense amplifier positive input is SHx (also automatically sets the LS_REF bit to 1)) bits in the CSA Control register
//!
#define DRV8323_CSA_CTRL_CSA_FET_BITS    (1 << 10)


// **************************************************************************
// the typedefs

//! \brief Enumeration for the R/W modes
//!
typedef enum
{
  DRV8323_CtrlMode_Read = 1 << 15,   //!< Read Mode
  DRV8323_CtrlMode_Write = 0 << 15   //!< Write Mode
} DRV8323_CtrlMode_e;


//! \brief Enumeration for the fault types
//!
typedef enum
{
  DRV8323_FaultType_NoFault  = (0 << 0),  //!< No fault

  DRV8323_FaultType_VDS_LC = (1 << 0),  //!< FET Low side, Phase C Over Current fault
  DRV8323_FaultType_VGS_LC = (1 << 0),  //!< FET Low side, Gate Drive fault

  DRV8323_FaultType_VDS_HC = (1 << 1),  //!< FET High side, Phase C Over Current fault
  DRV8323_FaultType_VGS_HC = (1 << 1),  //!< FET High side, Gate Drive fault

  DRV8323_FaultType_VDS_LB = (1 << 2),  //!< FET Low side, Phase B Over Current fault
  DRV8323_FaultType_VGS_LB = (1 << 2),  //!< FET Low side, Gate Drive fault

  DRV8323_FaultType_VDS_HB = (1 << 3),  //!< FET High side, Phase B Over Current fault
  DRV8323_FaultType_VGS_HB = (1 << 3),  //!< FET High side, Gate Drive fault

  DRV8323_FaultType_VDS_LA = (1 << 4),  //!< FET Low side, Phase A Over Current fault
  DRV8323_FaultType_VGS_LA = (1 << 4),  //!< FET Low side, Gate Drive fault

  DRV8323_FaultType_VDS_HA = (1 << 5),  //!< FET High side, Phase A Over Current fault
  DRV8323_FaultType_VGS_HA = (1 << 5),  //!< FET High side, Gate Drive fault


  DRV8323_FaultType_OTSD = (1 << 6),  //!< Over Temperature Shutdown Fault
  DRV8323_FaultType_CPUV = (1 << 6),  //!< Charge Pump Under voltage Fault

  DRV8323_FaultType_UVLO = (1 << 7),  //!< Under Voltage Lockout Fault
  DRV8323_FaultType_OTW = (1 << 7),  //!< Over Temperature Warning Fault

  DRV8323_FaultType_GDF = (1 << 8),  //!< Under Voltage Lockout Fault
  DRV8323_FaultType_SC_OC = (1 << 8),  //!< Sense Amplifier, Over Current on Phase C Fault

  DRV8323_FaultType_VDS_OCP = (1 << 9),  //!< VDS monitor Over Current Fault
  DRV8323_FaultType_SC_OB = (1 << 9),  //!< Sense Amplifier, Over Current on Phase B Fault

  DRV8323_FaultType_FAULT = (1 << 10),  //!< Logic OR of FAULT status registers. Mirrors nFAULT pin
  DRV8323_FaultType_SC_OA = (1 << 10),  //!< Sense Amplifier, Over Current on Phase A Fault

} DRV8323_FaultType_e;


//! \brief Enumeration for Vsense Over Current Trip Protection Limit
//!
typedef enum
{
  DRV8323_SenseOCLevel_0p25_V  = (0 << 0),   //!< Sense OCP 0.25 V
  DRV8323_SenseOCLevel_0p5_V   = (1 << 0),   //!< Sense OCP 0.5 V
  DRV8323_SenseOCLevel_0p75_V  = (2 << 0),   //!< Sense OCP 0.75 V
  DRV8323_SenseOCLevel_1p0_V   = (3 << 0)    //!< OSense OCP 1.0 V
} DRV8323_SenseOCLevel_e;

//! \brief Enumeration for the DC calibration modes for current sense amplifiers
//!
typedef enum
{
  DRV8323_CSA_DCCalMode_C_Load   = (0 << 2),   //!< Sense amplifier C connected to load via input pins
  DRV8323_CSA_DCCalMode_C_NoLoad = (1 << 2),   //!< Sense amplifier C disconnected from load and input pins are shorted for offset calibration
  DRV8323_CSA_DCCalMode_B_Load   = (0 << 3),   //!< Sense amplifier B connected to load via input pins
  DRV8323_CSA_DCCalMode_B_NoLoad = (1 << 3),   //!< Sense amplifier B disconnected from load and input pins are shorted for offset calibration
  DRV8323_CSA_DCCalMode_A_Load   = (0 << 4),   //!< Sense amplifier A connected to load via input pins
  DRV8323_CSA_DCCalMode_A_NoLoad = (1 << 4)    //!< Sense amplifier A disconnected from load and input pins are shorted for offset calibration
} DRV8323_CSA_DCCalMode_e;


//! \brief Enumeration for the Sense Over Current Fault modes
//!
typedef enum 
{
  DRV8323_OcFaultMode_Normal      = (0 << 5),   //!< Over current fault is enabled
  DRV8323_OcFaultMode_Disabled    = (1 << 5)    //!< Over current fault is disabled
} DRV8323_OcOFaultMode_e;

//! \brief Enumeration for the shunt amplifier gains
//!
typedef enum
{
  DRV8323_CSA_Gain_5VpV  = (0 << 6),   //!< 5 V per V
  DRV8323_CSA_Gain_10VpV = (1 << 6),   //!< 10 V per V
  DRV8323_CSA_Gain_20VpV = (2 << 6),   //!< 20 V per V
  DRV8323_CSA_Gain_40VpV = (3 << 6)    //!< 40 V per V
} DRV8323_CSA_Gain_e;

//! \brief Enumeration for the V_Sense low-side reference
//!
typedef enum
{
  DRV8323_OcVDS_LS_Ref_SPx      = (0 << 8),   //!< VDS_OCP for the low-side MOSFET is measured across SHx to SPx
  DRV8323_OcVDS_LS_Ref_SNx      = (1 << 8),   //!< VDS_OCP for the low-side MOSFET is measured across SHx to SNx
} DRV8323_OcVDS_LS_Reference_e;

//! \brief Enumeration for the CSA mode
//!
typedef enum
{
  DRV8323_CSAMode_Unidirectional      = (0 << 9),   //!< Sense amplifier reference voltage is VREF (unidirectional mode)
  DRV8323_CSAMode_Bidirectional       = (1 << 9),   //!<  Sense amplifier reference voltage is VREF divided by 2 (bidirectional mode)
} DRV8323_CSA_DirectionMode_e;

//! \brief Enumeration for the CSA mode
//!
typedef enum
{
  DRV8323_CSASensePositive_SPx     = (0 << 10),   //!< Sense amplifier positive input is SPx
  DRV8323_CSASensePositive_SHx       = (1 << 10),   //!<  Sense amplifier positive input is SHx (also automatically sets the LS_REF bit to 1)
} DRV8323_CSA_SensePositive_e;




//! \brief Enumeration for the Vds level for th over current adjustment
//!
typedef enum
{
  DRV8323_VdsLevel_0p060_V =  (0 << 0),      //!< Vds = 0.060 V
  DRV8323_VdsLevel_0p13_V  =  (1 << 0),      //!< Vds = 0.13 V
  DRV8323_VdsLevel_0p20_V  =  (2 << 0),      //!< Vds = 0.2 V
  DRV8323_VdsLevel_0p26_V  =  (3 << 0),      //!< Vds = 0.26 V
  DRV8323_VdsLevel_0p31_V  =  (4 << 0),      //!< Vds = 0.31 V
  DRV8323_VdsLevel_0p45_V  =  (5 << 0),      //!< Vds = 0.45 V
  DRV8323_VdsLevel_0p53_V  =  (6 << 0),      //!< Vds = 0.53 V
  DRV8323_VdsLevel_0p60_V  =  (7 << 0),      //!< Vds = 0.6 V
  DRV8323_VdsLevel_0p68_V  =  (8 << 0),      //!< Vds = 0.68 V
  DRV8323_VdsLevel_0p75_V  =  (9 << 0),      //!< Vds = 0.75 V
  DRV8323_VdsLevel_0p94_V  = (10 << 0),      //!< Vds = 0.94 V
  DRV8323_VdsLevel_1p13_V  = (11 << 0),      //!< Vds = 1.13 V
  DRV8323_VdsLevel_1p30_V  = (12 << 0),      //!< Vds = 1.3 V
  DRV8323_VdsLevel_1p50_V  = (13 << 0),      //!< Vds = 1.5 V
  DRV8323_VdsLevel_1p70_V  = (14 << 0),      //!< Vds = 1.7 V
  DRV8323_VdsLevel_1p88_V  = (15 << 0)       //!< Vds = 1.88 V
} DRV8323_VdsLevel_e;


//! \brief Enumeration for the Over Current Deglitch modes
//!
typedef enum
{
  DRV8323_OcDeglitch_2us       = (0 << 4),   //!< Over Current deglitch of 2 탎
  DRV8323_OcDeglitch_4us       = (1 << 4),   //!< Over Current deglitch of 4 탎
  DRV8323_OcDeglitch_6us       = (2 << 4),   //!< Over Current deglitch of 6 탎
  DRV8323_OcDeglitch_8us       = (3 << 4)    //!< Over Current deglitch of 8 탎
} DRV8323_OcDeglitch_e;


//! \brief Enumeration for the Over Current modes
//!
typedef enum 
{
  DRV8323_OcMode_LatchShutdown  = (0 << 6),   //!< Over Current causes a latched fault
  DRV8323_OcMode_AutoRetry      = (1 << 6),   //!< Over Current causes an automatic retrying fault
  DRV8323_OcMode_ReportOnly     = (2 << 6),   //!< Over Current is report only but no action is taken
  DRV8323_OcMode_Disabled       = (3 << 6)    //!< Over Current is not reported and no action is taken
} DRV8323_OcMode_e;

//! \brief Enumeration for the Dead Time modes
//!
typedef enum
{
  DRV8323_DeadTime_50ns        = (0 << 8),   //!< Dead Time of 50-ns
  DRV8323_DeadTime_100ns       = (1 << 8),   //!< Dead Time of 100-ns
  DRV8323_DeadTime_200ns       = (2 << 8),   //!< Dead Time of 200-ns
  DRV8323_DeadTime_400ns       = (3 << 8)    //!< Dead Time of 400-ns
} DRV8323_DeadTime_e;

//! \brief Enumeration for the Over Current Retry modes
//!
typedef enum
{
  DRV8323_OcRetryTime_4ms     = (0 << 10),   //!< VDS_OCP and SEN_OCP retry time is 4 ms
  DRV8323_OcRetryTime_50us    = (1 << 10)    //!< VDS_OCP and SEN_OCP retry time is 50 us
} DRV8323_OcRetryTime_e;



//! \brief Enumeration for the IDrive Sink Peak Current modes
//!
typedef enum
{
  DRV8323_SnkPeakCurrent_20_mA =  (0 << 0),      //!< I = 20 mA
  DRV8323_SnkPeakCurrent_60_mA  =  (1 << 0),      //!< I = 60 mA
  DRV8323_SnkPeakCurrent_120_mA  =  (2 << 0),      //!< I = 120 mA
  DRV8323_SnkPeakCurrent_160_mA  =  (3 << 0),      //!< I = 160 mA
  DRV8323_SnkPeakCurrent_240_mA  =  (4 << 0),      //!< I = 240 mA
  DRV8323_SnkPeakCurrent_280_mA  =  (5 << 0),      //!< I = 280 mA
  DRV8323_SnkPeakCurrent_340_mA  =  (6 << 0),      //!< I = 340 mA
  DRV8323_SnkPeakCurrent_380_mA  =  (7 << 0),      //!< I = 380 mA
  DRV8323_SnkPeakCurrent_520_mA  =  (8 << 0),      //!< I = 520 mA
  DRV8323_SnkPeakCurrent_660_mA  =  (9 << 0),      //!< I = 660 mA
  DRV8323_SnkPeakCurrent_740_mA  = (10 << 0),      //!< I = 740 mA
  DRV8323_SnkPeakCurrent_880_mA  = (11 << 0),      //!< I = 880 mA
  DRV8323_SnkPeakCurrent_1140_mA  = (12 << 0),      //!< I = 1140 mA
  DRV8323_SnkPeakCurrent_1360_mA  = (13 << 0),      //!< I = 1360 mA
  DRV8323_SnkPeakCurrent_1640_mA  = (14 << 0),      //!< I = 1640 mA
  DRV8323_SnkPeakCurrent_2000_mA  = (15 << 0)       //!< I = 2000 mA
} DRV8323_IDrivePeakSinkGateCurrent_e;

//! \brief Enumeration for the IDrive Source Peak Current modes
//!
typedef enum
{
  DRV8323_SrcPeakCurrent_10_mA =  (0 << 4),      //!< I = 10 mA
  DRV8323_SrcPeakCurrent_30_mA  =  (1 << 4),      //!< I = 30 mA
  DRV8323_SrcPeakCurrent_60_mA  =  (2 << 4),      //!< I = 60 mA
  DRV8323_SrcPeakCurrent_80_mA  =  (3 << 4),      //!< I = 80 mA
  DRV8323_SrcPeakCurrent_120_mA  =  (4 << 4),      //!< I = 120 mA
  DRV8323_SrcPeakCurrent_140_mA  =  (5 << 4),      //!< I = 140 mA
  DRV8323_SrcPeakCurrent_170_mA  =  (6 << 4),      //!< I = 170 mA
  DRV8323_SrcPeakCurrent_190_mA  =  (7 << 4),      //!< I = 190 mA
  DRV8323_SrcPeakCurrent_260_mA  =  (8 << 4),      //!< I = 260 mA
  DRV8323_SrcPeakCurrent_330_mA  =  (9 << 4),      //!< I = 330 mA
  DRV8323_SrcPeakCurrent_370_mA  = (10 << 4),      //!< I = 370 mA
  DRV8323_SrcPeakCurrent_440_mA  = (11 << 4),      //!< I = 440 mA
  DRV8323_SrcPeakCurrent_570_mA  = (12 << 4),      //!< I = 570 mA
  DRV8323_SrcPeakCurrent_680_mA  = (13 << 4),      //!< I = 680 mA
  DRV8323_SrcPeakCurrent_820_mA  = (14 << 4),      //!< I = 820 mA
  DRV8323_SrcPeakCurrent_1000_mA  = (15 << 4)       //!< I = 1000 mA
} DRV8323_IDrivePeakSourceGateCurrent_e;


//! \brief Enumeration for the TDrive (peak gate-current drive time) modes
//!
typedef enum
{
  DRV8323_TDrive_Time_500ns        = (0 << 8),   //!< Drive time of 500-ns
  DRV8323_TDrive_Time_1000ns       = (1 << 8),   //!< Drive time of 1000-ns
  DRV8323_TDrive_Time_2000ns       = (2 << 8),   //!< Drive time of 2000-ns
  DRV8323_TDrive_Time_4000ns       = (3 << 8)    //!< Drive time of 4000-ns
} DRV8323_TDrive_Time_e;

//! \brief Enumeration for the Over Current Retry Clear modes
//!
typedef enum
{
	DRV8323_OcFaultNoClearPWM  = 0 << 10,   //!<
	DRV8323_OcFaultClearPWM    = 1 << 10    //!< In retry OCP_MODE, for both VDS_OCP and SEN_OCP, the fault is automatically cleared when a PWM input is given
} DRV8323_OcOFaultClearRetryMode_e;


//! \brief Enumeration for the Register Locking Mode
//!
typedef enum
{
  DRV8323_RegisterLock    = 6 << 8,   //!< Lock all registers but this one and address 0x02h bits 0:2
  DRV8323_RegisterUnlock =  3 << 8,   //!< Unlock all registers
} DRV8323_RegisterLockMode_e;



//! \brief Enumeration for the PWM modes
//!
typedef enum 
{
  DRV8323_PwmMode_Six_Inputs   = 0 << 5,   //!< 6x PWM Mode
  DRV8323_PwmMode_Three_Inputs = 1 << 5,   //!< 3x PWM Mode
  DRV8323_PwmMode_One_Inputs = 2 << 5,     //!< 1x PWM Mode
  DRV8323_PwmMode_Independent_Inputs = 3 << 5    //!< Independent PWM Mode
} DRV8323_PwmMode_e;



//! \brief Enumeration for the register names
//!
typedef enum 
{
  DRV8323_RegName_Fault_Status_1  = DRV8323_STATUS1_ADDR << 11,   //!< Fault Status Register 1
  DRV8323_RegName_Fault_Status_2  = DRV8323_STATUS2_ADDR << 11,   //!< Fault Status Register 2
  DRV8323_RegName_Driver_Control = DRV8323_DRIVER_CTRL_ADDR << 11,  //!< Driver Control Register
  DRV8323_RegName_Gate_Drive_HS_Control = DRV8323_GATE_DRIVE_HS_ADDR << 11 ,  //!< Gate Drive HS Control Register
  DRV8323_RegName_Gate_Drive_LS_Control = DRV8323_GATE_DRIVE_LS_ADDR << 11,  //!< Gate Drive LS Control Register
  DRV8323_RegName_OCP_Control =  DRV8323_OCP_CTRL_ADDR << 11,   //!< OCP Control Register
  DRV8323_RegName_CSA_Control =  DRV8323_CSA_CTRL_ADDR << 11,  //!< CSA Register
} DRV8323_RegName_e;


//! \brief Enumeration for the shunt amplifier number
//!
typedef enum 
{
  DRV8323_SenseAmpNumber_1 = 1,      //!< Shunt amplifier number 1
  DRV8323_SenseAmpNumber_2 = 2,       //!< Shunt amplifier number 2
  DRV8323_SenseAmpNumber_3 = 3       //!< Shunt amplifier number 3
} DRV8323_SenseAmpNumber_e;

typedef struct _DRV_SPI_8323_Fault_Stat1_t_
{
  bool                  FAULT;
  bool                  VDS_OCP;
  bool                  GDF;
  bool                  UVLO;
  bool                  OTSD;
  bool                  VDS_HA;
  bool                  VDS_LA;
  bool                  VDS_HB;
  bool                  VDS_LB;
  bool                  VDS_HC;
  bool                  VDS_LC;
}DRV_SPI_8323_Fault_Stat1_t_;

typedef struct _DRV_SPI_8323_Fault_Stat2_t_
{
  bool                  SA_OC;
  bool                  SB_OC;
  bool                  SC_OC;
  bool                  OTW;
  bool                  CPUV;
  bool                  VGS_HA;
  bool                  VGS_LA;
  bool                  VGS_HB;
  bool                  VGS_LB;
  bool                  VGS_HC;
  bool                  VGS_LC;
}DRV_SPI_8323_Fault_Stat2_t_;


typedef struct _DRV_SPI_8323_Driver_Control_t_
{
  bool                  RESERVED;
  bool                  DIS_CPUV;
  bool                  DIS_GDF;
  bool                  OTW_REP;
  DRV8323_PwmMode_e     PWM_MODE;
  bool                  PWM_1X_COM;
  bool                  PWM_1X_DIR;
  bool                  COAST;
  bool                  BRAKE;
  bool                  CLR_FLT;
}DRV_SPI_8323_Driver_Control_t_;

typedef struct _DRV_SPI_8323_Gate_Drive_HS_t_
{
  DRV8323_RegisterLockMode_e                      LOCK;
  DRV8323_IDrivePeakSourceGateCurrent_e          IDRIVEP_HS;
  DRV8323_IDrivePeakSinkGateCurrent_e            IDRIVEN_HS;
}DRV_SPI_8323_Gate_Drive_HS_t_;

typedef struct _DRV_SPI_8323_Gate_Drive_LS_t_
{
  DRV8323_OcOFaultClearRetryMode_e				 CBC;
  DRV8323_TDrive_Time_e                          TDRIVE;
  DRV8323_IDrivePeakSourceGateCurrent_e          IDRIVEP_LS;
  DRV8323_IDrivePeakSinkGateCurrent_e            IDRIVEN_LS;
}DRV_SPI_8323_Gate_Drive_LS_t_;


typedef struct _DRV_SPI_8323_OCP_Control_t_
{
	DRV8323_OcRetryTime_e				 		TRETRY;
	DRV8323_DeadTime_e                          DEAD_TIME;
	DRV8323_OcMode_e          					OCP_MODE;
	DRV8323_OcDeglitch_e            			OCP_DEG;
	DRV8323_VdsLevel_e							VDS_LVL;
}DRV_SPI_8323_OCP_Control_t_;

typedef struct _DRV_SPI_8323_CSA_Control_t_
{
	DRV8323_CSA_SensePositive_e					CSA_FET;
	DRV8323_CSA_DirectionMode_e					VREF_DIV;
	DRV8323_OcVDS_LS_Reference_e				LS_REF;
	DRV8323_CSA_Gain_e                          CSA_GAIN;
	DRV8323_OcOFaultMode_e          			DIS_SEN;
	DRV8323_CSA_DCCalMode_e            			CSA_CAL;
	DRV8323_SenseOCLevel_e						SEN_LVL;
}DRV_SPI_8323_CSA_Control_t_;


typedef struct _DRV_SPI_8323_Vars_t_
{
  DRV_SPI_8323_Fault_Stat1_t_     Fault_Stat_Reg_1;
  DRV_SPI_8323_Fault_Stat2_t_     Fault_Stat_Reg_2;
  DRV_SPI_8323_Driver_Control_t_     Driver_Ctrl_Reg;
  DRV_SPI_8323_Gate_Drive_HS_t_     Gate_Drive_HS_Reg;
  DRV_SPI_8323_Gate_Drive_LS_t_     Gate_Drive_LS_Reg;
  DRV_SPI_8323_OCP_Control_t_     OCP_Ctrl_Reg;
  DRV_SPI_8323_CSA_Control_t_     CSA_Ctrl_Reg;
  bool                  SndCmd;
  bool                  RcvCmd;
}DRV_SPI_8323_Vars_t;


//! \brief Defines the DRV8323 object
//!
typedef struct _DRV8323_Obj_
{
  SPI_Handle       spiHandle;                  //!< the handle for the serial peripheral interface
  GPIO_Handle      EngpioHandle;               //!< the gpio handle that is connected to the DRV8323 enable pin
  GPIO_Number_e    EngpioNumber;               //!< the gpio number that is connected to the DRV8323 enable pin
  GPIO_Handle      nCSgpioHandle;              //!< the gpio handle that is connected to the DRV8323 nCS pin
  GPIO_Number_e    nCSgpioNumber;               //!< the gpio number that is connected to the DRV8323 nCS pin
  bool             RxTimeOut;                  //!< the timeout flag for the RX fifo
  bool             enableTimeOut;              //!< the timeout flag for DRV8323 enable
} DRV8323_Obj;


//! \brief Defines the DRV8323 handle
//!
typedef struct _DRV8323_Obj_ *DRV8323_Handle;


//! \brief Defines the DRV8323 Word type
//!
typedef  uint16_t    DRV8323_Word_t;


// **************************************************************************
// the globals



// **************************************************************************
// the function prototypes


//! \brief     Builds the control word
//! \param[in] ctrlMode  The control mode
//! \param[in] regName   The register name
//! \param[in] data      The data
//! \return    The control word
static inline DRV8323_Word_t DRV8323_buildCtrlWord(const DRV8323_CtrlMode_e ctrlMode,
                                                   const DRV8323_RegName_e regName,
                                                   const uint16_t data)
{
  DRV8323_Word_t ctrlWord = ctrlMode | regName | (data & DRV8323_DATA_MASK);

  return(ctrlWord);
} // end of DRV8323_buildCtrlWord() function


//! \brief     Gets the DC calibration mode
//! \param[in] handle     The DRV8323 handle
//! \param[in] ampNumber  The sense amplifier number
//! \return    The DC calibration mode
extern DRV8323_CSA_DCCalMode_e DRV8323_getDcCalMode(DRV8323_Handle handle,
                                                const DRV8323_SenseAmpNumber_e ampNumber);

//! \brief     Enables the DRV8323
//! \param[in] handle     The DRV8323 handle
extern void DRV8323_enable(DRV8323_Handle handle);

//! \brief     Enables the DRV8323
//! \param[in] handle     The DRV8323 handle
extern void DRV8323_disable(DRV8323_Handle handle);

//! \brief     Gets the fault type
//! \param[in] handle     The DRV8323 handle
//! \return    The fault type
extern DRV8323_FaultType_e DRV8323_getFaultType(DRV8323_Handle handle);


//! \brief     Gets the over current level
//! \param[in] handle     The DRV8323 handle
//! \return    The over current level, V
extern DRV8323_VdsLevel_e DRV8323_getOcLevel(DRV8323_Handle handle);


//! \brief     Gets the over current mode
//! \param[in] handle     The DRV8323 handle
//! \return    The over current mode
extern DRV8323_OcMode_e DRV8323_getOcMode(DRV8323_Handle handle);

//! \brief     Gets the PWM mode
//! \param[in] handle     The DRV8323 handle
//! \return    The PWM mode
extern DRV8323_PwmMode_e DRV8323_getPwmMode(DRV8323_Handle handle);


//! \brief     Gets the shunt amplifier gain value
//! \param[in] handle     The DRV8323 handle
//! \return    The shunt amplifier gain value
extern DRV8323_CSA_Gain_e DRV8323_getCSAGain(DRV8323_Handle handle);


//! \brief     Gets the status register 1 value
//! \param[in] handle     The DRV8323 handle
//! \return    The status register1 value
extern uint16_t DRV8323_getFaultStatusRegister1(DRV8323_Handle handle);


//! \brief     Gets the status register 2 value
//! \param[in] handle     The DRV8323 handle
//! \return    The status register2 value
extern uint16_t DRV8323_getFaultStatusRegister2(DRV8323_Handle handle);


//! \brief     Initializes the DRV8323 object
//! \param[in] pMemory   A pointer to the memory for the DRV8323 object
//! \param[in] numBytes  The number of bytes allocated for the DRV8323 object, bytes
//! \return    The DRV8323 object handle
extern DRV8323_Handle DRV8323_init(void *pMemory,const size_t numBytes);


//! \brief     Determines if DRV8323 fault has occurred
//! \param[in] handle     The DRV8323 handle
//! \return    A boolean value denoting if a fault has occurred (true) or not (false)
extern bool DRV8323_isFault(DRV8323_Handle handle);

//! \brief     Reads data from the DRV8323 register
//! \param[in] handle   The DRV8323 handle
//! \param[in] regName  The register name
//! \return    The data value
extern uint16_t DRV8323_readSpi(DRV8323_Handle handle,const DRV8323_RegName_e regName);


//! \brief     Resets the DRV8323
//! \param[in] handle   The DRV8323 handle
extern void DRV8323_reset(DRV8323_Handle handle);


//! \brief     Resets the enable timeout flag
//! \param[in] handle   The DRV8323 handle
static inline void DRV8323_resetEnableTimeout(DRV8323_Handle handle)
{
  DRV8323_Obj *obj = (DRV8323_Obj *)handle;

  obj->enableTimeOut = false;

  return;
}


//! \brief     Resets the RX fifo timeout flag
//! \param[in] handle   The DRV8323 handle
static inline void DRV8323_resetRxTimeout(DRV8323_Handle handle)
{
  DRV8323_Obj *obj = (DRV8323_Obj *)handle;

  obj->RxTimeOut = false;

  return;
}

//! \brief     Sets the DC calibration mode
//! \param[in] handle     The DRV8323 handle
//! \param[in] ampNumber  The shunt amplifier number
//! \param[in] mode       The DC calibration mode
extern void DRV8323_setDcCalMode(DRV8323_Handle handle,
                                 const DRV8323_SenseAmpNumber_e ampNumber,
                                 const DRV8323_CSA_DCCalMode_e mode);

//! \brief     Sets the GPIO handle in the DRV8323
//! \param[in] handle     The DRV8323 handle
//! \param[in] gpioHandle  The GPIO handle to use
void DRV8323_setGpioHandle(DRV8323_Handle handle,GPIO_Handle gpioHandle);


//! \brief     Sets the GPIO number in the DRV8323
//! \param[in] handle     The DRV8323 handle
//! \param[in] gpioHandle  The GPIO number to use
void DRV8323_setGpioNumber(DRV8323_Handle handle,GPIO_Number_e gpioNumber);


//! \brief     Sets the over current level in terms of Vds
//! \param[in] handle    The DRV8323 handle
//! \param[in] VdsLevel  The over current level, V
extern void DRV8323_setOcLevel(DRV8323_Handle handle,const DRV8323_VdsLevel_e VdsLevel);


//! \brief     Sets the over current mode
//! \param[in] handle  The DRV8323 handle
//! \param[in] mode    The over current mode
extern void DRV8323_setOcMode(DRV8323_Handle handle,const DRV8323_OcMode_e mode);

//! \brief     Sets the PWM mode
//! \param[in] handle  The DRV8323 handle
//! \param[in] mode    The PWM mode
extern void DRV8323_setPwmMode(DRV8323_Handle handle,const DRV8323_PwmMode_e mode);


//! \brief     Sets the shunt amplifier gain value
//! \param[in] handle  The DRV8323 handle
//! \param[in] gain    The sense amplifier gain value
extern void DRV8323_setCSAGain(DRV8323_Handle handle,const DRV8323_CSA_Gain_e gain);


//! \brief     Sets the SPI handle in the DRV8323
//! \param[in] handle     The DRV8323 handle
//! \param[in] spiHandle  The SPI handle to use
void DRV8323_setSpiHandle(DRV8323_Handle handle,SPI_Handle spiHandle);


//! \brief     Writes data to the DRV8323 register
//! \param[in] handle   The DRV8323 handle
//! \param[in] regName  The register name
//! \param[in] data     The data value
extern void DRV8323_writeSpi(DRV8323_Handle handle,const DRV8323_RegName_e regName,const uint16_t data);


//! \brief     Interface to all 8323 SPI variables
//!
//! \details   Call this function periodically to be able to read the DRV8323 Status1, Status2,
//!            Control1, and Control2 registers and write the Control1 and Control2 registers.
//!            This function updates the members of the structure DRV_SPI_8301_Vars_t.
//!            <b>How to use in Setup</b>
//!            <b>Code</b>
//!            Add the structure declaration DRV_SPI_8323_Vars_t to your code
//!            Make sure the SPI and 8323 EN_Gate GPIO are setup for the 8301 by using HAL_init and HAL_setParams
//!            During code setup, call HAL_enableDrv and HAL_setupDrvSpi
//!            In background loop, call DRV8323_writeData and DRV8323_readData
//!            <b>How to use in Runtime</b>
//!            <b>Watch window</b>
//!            Add the structure, declared by DRV_SPI_8323_Vars_t above, to the watch window
//!            <b>Runtime</b>
//!            Pull down the menus from the DRV_SPI_8323_Vars_t strcuture to the desired setting
//!            Set SndCmd to send the settings to the DRV8323
//!            If a read of the DRV8323 registers is required, se RcvCmd
//!
//! \param[in] handle  The DRV8323 handle
//! \param[in] Spi_8323_Vars  The (DRV_SPI_8323_Vars_t) structure that contains all DRV8323 Status/Control register options
extern void DRV8323_writeData(DRV8323_Handle handle, DRV_SPI_8323_Vars_t *Spi_8323_Vars);


//! \param[in] handle  The DRV8323 handle
//! \param[in] Spi_8301_Vars  The (DRV_SPI_8301_Vars_t) structure that contains all DRV8323 Status/Control register options
extern void DRV8323_readData(DRV8323_Handle handle, DRV_SPI_8323_Vars_t *Spi_8323_Vars);


//! \brief     Initialize the interface to all 8301 SPI variables
//! \param[in] handle  The DRV8323 handle
extern void DRV8323_setupSpi(DRV8323_Handle handle, DRV_SPI_8323_Vars_t *Spi_8323_Vars);


// Read Faults

#ifdef __cplusplus
}
#endif // extern "C"

//@}  // ingroup

#endif // end of _DRV8323_H_ definition





