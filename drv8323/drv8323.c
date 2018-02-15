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
//! \file   drivers/drvic/drv8301/src/32b/f28x/f2806x/drv8301.c
//! \brief  Contains the various functions related to the DRV8323 object
//!
//! (C) Copyright 2015, Texas Instruments, Inc.


// **************************************************************************
// the includes

#include "assert.h"
#include <math.h>
#include "cmsis_os.h"

// drivers
#include "drv8323.h"


// **************************************************************************
// the defines


// **************************************************************************
// the globals


// **************************************************************************
// the function prototypes

void DRV8323_enable(DRV8323_Handle handle)
{

	HAL_GPIO_WritePin(handle->EngpioHandle, handle->EngpioNumber, GPIO_PIN_SET);
	osDelay(20);
	HAL_GPIO_WritePin(handle->EngpioHandle, handle->EngpioNumber, GPIO_PIN_RESET);
	osDelay(20);

	//Enable driver
	HAL_GPIO_WritePin(handle->EngpioHandle, handle->EngpioNumber, GPIO_PIN_SET);

	//Wait for driver to come online
	osDelay(10);

	// Make sure the Fault bit is not set during startup
	while((DRV8323_readSpi(handle,DRV8323_RegName_Fault_Status_1) & DRV8323_STATUS1_FAULT_BITS) != 0);

	// Wait for the DRV8323 registers to update
	osDelay(1);

	return;
}

void DRV8323_disable(DRV8323_Handle handle)
{
	//Disable driver
	HAL_GPIO_WritePin(handle->EngpioHandle, handle->EngpioNumber, GPIO_PIN_RESET);

	//Wait for driver to come online
	osDelay(10);

	return;
}

DRV8323_CSA_DCCalMode_e DRV8323_getDcCalMode(DRV8323_Handle handle,const DRV8323_SenseAmpNumber_e ampNumber)
{
  uint16_t data;

  // read data
  data = DRV8323_readSpi(handle,DRV8323_RegName_CSA_Control);

  // extract the bits
  if(ampNumber == DRV8323_SenseAmpNumber_1)
    {
      data &= (DRV8323_CSA_CTRL_CSA_CAL_A_BITS);

    }
  else if(ampNumber == DRV8323_SenseAmpNumber_2)
    {
      data &= (DRV8323_CSA_CTRL_CSA_CAL_B_BITS);
    }
  else if(ampNumber == DRV8323_SenseAmpNumber_3)
    {
      data &= (DRV8323_CSA_CTRL_CSA_CAL_C_BITS);
    }

  return((DRV8323_CSA_DCCalMode_e)data);
} // end of DRV8323_getDcCalMode() function


DRV8323_FaultType_e DRV8323_getFaultType(DRV8323_Handle handle)
{
  DRV8323_Word_t      readWord;
  DRV8323_FaultType_e faultType = DRV8323_FaultType_NoFault;


  // read the data
  readWord = DRV8323_readSpi(handle,DRV8323_RegName_Fault_Status_1);

  if(readWord & DRV8323_STATUS1_FAULT_BITS)
    {
      faultType = (DRV8323_FaultType_e)(readWord & DRV8323_FAULT_TYPE_MASK);

     /* if(faultType == DRV8323_FaultType_NoFault)
        {
          // read the data
          readWord = DRV8323_readSpi(handle,DRV8323_RegName_Status_2);

          if(readWord & DRV8323_STATUS2_GVDD_OV_BITS)
            {
              faultType = DRV8323_FaultType_GVDD_OV;
            }
        }*/
    }

  return(faultType);
} // end of DRV8323_getFaultType() function


DRV8323_VdsLevel_e DRV8323_getOcLevel(DRV8323_Handle handle)
{
  uint16_t data;

  // read data
  data = DRV8323_readSpi(handle,DRV8323_RegName_OCP_Control);

  // extract the bits
  data &= (DRV8323_OCP_CTRL_VDS_LVL_BITS);

  return((DRV8323_VdsLevel_e)data);
} // end of DRV8323_getOcLevel() function


DRV8323_OcMode_e DRV8323_getOcMode(DRV8323_Handle handle)
{
  uint16_t data;


  // read data
  data = DRV8323_readSpi(handle,DRV8323_RegName_OCP_Control);

  // extract the bits
  data &= (DRV8323_OCP_CTRL_OCP_MODE_BITS);

  return((DRV8323_OcMode_e)data);
} // end of DRV8323_getOcMode() function


DRV8323_PwmMode_e DRV8323_getPwmMode(DRV8323_Handle handle)
{
  uint16_t data;

  // read data
  data = DRV8323_readSpi(handle,DRV8323_RegName_Driver_Control);

  // extract the bits
  data &= (DRV8323_DRIVER_CTRL_PWM_MODE_BITS);

  return((DRV8323_PwmMode_e)data);
} // end of DRV8323_getPwmMode() function


DRV8323_CSA_Gain_e DRV8323_getCSAGain(DRV8323_Handle handle)
{
  uint16_t data;

  // read data
  data = DRV8323_readSpi(handle,DRV8323_RegName_CSA_Control);

  // extract the bits
  data &= (DRV8323_CSA_CTRL_CSA_GAIN_BITS);

  return((DRV8323_CSA_Gain_e)data);
} // end of DRV8323_getShuntAmpGain() function


DRV8323_Handle DRV8323_init(void *pMemory,const size_t numBytes)
{
  DRV8323_Handle handle;


  if(numBytes < sizeof(DRV8323_Obj))
    return((DRV8323_Handle)NULL);


  // assign the handle
  handle = (DRV8323_Handle)pMemory;

  DRV8323_resetRxTimeout(handle);
  DRV8323_resetEnableTimeout(handle);


  return(handle);
} // end of DRV8323_init() function


void DRV8323_setEnGpioHandle(DRV8323_Handle handle,GPIO_Handle gpioHandle)
{
  DRV8323_Obj *obj = (DRV8323_Obj *)handle;

  // initialize the gpio interface object
  obj->EngpioHandle = gpioHandle;

  return;
} // end of DRV8323_setGpioHandle() function


void DRV8323_setEnGpioNumber(DRV8323_Handle handle,GPIO_Number_e gpioNumber)
{
  DRV8323_Obj *obj = (DRV8323_Obj *)handle;

  // initialize the gpio interface object
  obj->EngpioNumber = gpioNumber;

  return;
} // end of DRV8323_setGpioNumber() function


void DRV8323_setnCSGpioHandle(DRV8323_Handle handle,GPIO_Handle gpioHandle)
{
  DRV8323_Obj *obj = (DRV8323_Obj *)handle;

  // initialize the gpio interface object
  obj->nCSgpioHandle = gpioHandle;

  return;
} // end of DRV8323_setGpioHandle() function


void DRV8323_setnCSGpioNumber(DRV8323_Handle handle,GPIO_Number_e gpioNumber)
{
  DRV8323_Obj *obj = (DRV8323_Obj *)handle;

  // initialize the gpio interface object
  obj->nCSgpioNumber = gpioNumber;

  return;
} // end of DRV8323_setGpioNumber() function


void DRV8323_setSpiHandle(DRV8323_Handle handle,SPI_Handle spiHandle)
{
  DRV8323_Obj *obj = (DRV8323_Obj *)handle;

  // initialize the serial peripheral interface object
  obj->spiHandle = spiHandle;

  return;
} // end of DRV8323_setSpiHandle() function


bool DRV8323_isFault(DRV8323_Handle handle)
{
  DRV8323_Word_t readWord;
  bool status=false;


  // read the data
  readWord = DRV8323_readSpi(handle,DRV8323_RegName_Fault_Status_1);

  if(readWord & DRV8323_STATUS1_FAULT_BITS)
    {
      status = true;
    }

  return(status);
} // end of DRV8323_isFault() function


uint16_t DRV8323_readSpi(DRV8323_Handle handle, const DRV8323_RegName_e regName)
{

  // Actuate chipselect
  HAL_GPIO_WritePin(handle->nCSgpioHandle, handle->nCSgpioNumber, GPIO_PIN_RESET);
  osDelay(1);

  // Do blocking read
  uint16_t controlword = (uint16_t)DRV8323_buildCtrlWord(DRV8323_CtrlMode_Read, regName, 0);
  uint16_t recbuff = 0xbeef;
  HAL_SPI_TransmitReceive(handle->spiHandle, (uint8_t*)(&controlword), (uint8_t*)(&recbuff), 1, 1000);

  // Actuate chipselect
  HAL_GPIO_WritePin(handle->nCSgpioHandle, handle->nCSgpioNumber, GPIO_PIN_SET);
  osDelay(1);

  assert(recbuff != 0xbeef);

  return(recbuff & DRV8323_DATA_MASK);
}  // end of DRV8323_readSpi() function


void DRV8323_reset(DRV8323_Handle handle)
{
	// TODO: Could also reset CLR_FLT over SPI
	HAL_GPIO_WritePin(handle->nCSgpioHandle, handle->EngpioNumber, GPIO_PIN_RESET);
	osDelay(10);
	HAL_GPIO_WritePin(handle->nCSgpioHandle, handle->EngpioNumber, GPIO_PIN_SET);

  return;
}  // end of DRV8323_reset() function

  
void DRV8323_setDcCalMode(DRV8323_Handle handle,const DRV8323_SenseAmpNumber_e ampNumber,const DRV8323_CSA_DCCalMode_e mode)
{
  uint16_t data;

  // read data
  data = DRV8323_readSpi(handle,DRV8323_RegName_CSA_Control);

  // clear the bits
  if(ampNumber == DRV8323_SenseAmpNumber_1)
    {
      data &= (~DRV8323_CSA_CTRL_CSA_CAL_A_BITS);

    }
  else if(ampNumber == DRV8323_SenseAmpNumber_2)
    {
      data &= (~DRV8323_CSA_CTRL_CSA_CAL_B_BITS);
    }
  else if(ampNumber == DRV8323_SenseAmpNumber_3)
    {
      data &= (~DRV8323_CSA_CTRL_CSA_CAL_C_BITS);
    }
  // set the bits
  data |= mode;

  // write the data
  DRV8323_writeSpi(handle,DRV8323_RegName_CSA_Control,data);

  return;
} // end of DRV8323_setDcCalMode() function


void DRV8323_setOcLevel(DRV8323_Handle handle,const DRV8323_VdsLevel_e VdsLevel)
{
  uint16_t data;

  // read data
  data = DRV8323_readSpi(handle,DRV8323_RegName_OCP_Control);

  // clear the bits
  data &= (~DRV8323_OCP_CTRL_VDS_LVL_BITS);

  // set the bits
  data |= VdsLevel;

  // write the data
  DRV8323_writeSpi(handle,DRV8323_RegName_OCP_Control,data);

  return;
} // end of DRV8323_setOcLevel() function


void DRV8323_setOcMode(DRV8323_Handle handle,const DRV8323_OcMode_e mode)
{
  uint16_t data;

  // read data
  data = DRV8323_readSpi(handle,DRV8323_RegName_OCP_Control);

  // clear the bits
  data &= (~DRV8323_OCP_CTRL_OCP_MODE_BITS);

  // set the bits
  data |= mode;

  // write the data
  DRV8323_writeSpi(handle,DRV8323_RegName_OCP_Control,data);

  return;
} // end of DRV8323_setOcMode() function



void DRV8323_setPwmMode(DRV8323_Handle handle,const DRV8323_PwmMode_e mode)
{
  uint16_t data;

  // read data
  data = DRV8323_readSpi(handle,DRV8323_RegName_Driver_Control);

  // clear the bits
  data &= (~DRV8323_DRIVER_CTRL_PWM_MODE_BITS);

  // set the bits
  data |= mode;


  // write the data
  DRV8323_writeSpi(handle,DRV8323_RegName_Driver_Control,data);

  data = DRV8323_readSpi(handle,DRV8323_RegName_Driver_Control);

  return;
} // end of DRV8323_setPwmMode() function


void DRV8323_setCSAGain(DRV8323_Handle handle,const DRV8323_CSA_Gain_e gain)
{
  uint16_t data;

  // read data
  data = DRV8323_readSpi(handle,DRV8323_RegName_CSA_Control);

  // clear the bits
  data &= (~DRV8323_CSA_CTRL_CSA_GAIN_BITS);

  // set the bits
  data |= gain;

  // write the data
  DRV8323_writeSpi(handle,DRV8323_RegName_CSA_Control,data);

  return;
} // end of DRV8323_setShuntAmpGain() function


void DRV8323_writeSpi(DRV8323_Handle handle, const DRV8323_RegName_e regName,const uint16_t data)
{
  // Actuate chipselect
  HAL_GPIO_WritePin(handle->nCSgpioHandle, handle->nCSgpioNumber, GPIO_PIN_RESET);
  osDelay(1);

  // Do blocking write
  uint16_t controlword = (uint16_t)DRV8323_buildCtrlWord(DRV8323_CtrlMode_Write, regName, data);
  HAL_SPI_Transmit(handle->spiHandle, (uint8_t*)(&controlword), 1, 1000);
  osDelay(1);

  // Actuate chipselect
  HAL_GPIO_WritePin(handle->nCSgpioHandle, handle->nCSgpioNumber, GPIO_PIN_SET);
  osDelay(1);

  return;
}  // end of DRV8323_writeSpi() function


void DRV8323_writeData(DRV8323_Handle handle, DRV_SPI_8323_Vars_t *Spi_8323_Vars)
{
  DRV8323_RegName_e  drvRegName;
  uint16_t drvDataNew;

  if(Spi_8323_Vars->SndCmd)
  {
    // Update Driver Control Register
    drvRegName = DRV8323_RegName_Driver_Control;
    drvDataNew = Spi_8323_Vars->Driver_Ctrl_Reg.RESERVED |  \
    		Spi_8323_Vars->Driver_Ctrl_Reg.DIS_CPUV   |  \
			Spi_8323_Vars->Driver_Ctrl_Reg.DIS_GDF     |  \
			Spi_8323_Vars->Driver_Ctrl_Reg.OTW_REP      |  \
			Spi_8323_Vars->Driver_Ctrl_Reg.PWM_MODE |	\
			Spi_8323_Vars->Driver_Ctrl_Reg.PWM_1X_COM |	\
			Spi_8323_Vars->Driver_Ctrl_Reg.PWM_1X_DIR |		\
			Spi_8323_Vars->Driver_Ctrl_Reg.COAST |	\
			Spi_8323_Vars->Driver_Ctrl_Reg.BRAKE |	\
			Spi_8323_Vars->Driver_Ctrl_Reg.CLR_FLT;
    DRV8323_writeSpi(handle,drvRegName,drvDataNew);

    // Update Gate Drive HS Register
    drvRegName = DRV8323_RegName_Gate_Drive_HS_Control;
    drvDataNew = Spi_8323_Vars->Gate_Drive_HS_Reg.LOCK |  \
    		Spi_8323_Vars->Gate_Drive_HS_Reg.IDRIVEP_HS   |  \
			Spi_8323_Vars->Gate_Drive_HS_Reg.IDRIVEN_HS;
    DRV8323_writeSpi(handle,drvRegName,drvDataNew);

    // Update Gate Drive LS Register
    drvRegName = DRV8323_RegName_Gate_Drive_LS_Control;
    drvDataNew = Spi_8323_Vars->Gate_Drive_LS_Reg.CBC |  \
    		Spi_8323_Vars->Gate_Drive_LS_Reg.TDRIVE   |  \
			Spi_8323_Vars->Gate_Drive_LS_Reg.IDRIVEP_LS | \
			Spi_8323_Vars->Gate_Drive_LS_Reg.IDRIVEN_LS;
    DRV8323_writeSpi(handle,drvRegName,drvDataNew);

    // Update OCP Control Register
    drvRegName = DRV8323_RegName_OCP_Control;
    drvDataNew = Spi_8323_Vars->OCP_Ctrl_Reg.TRETRY |  \
    		Spi_8323_Vars->OCP_Ctrl_Reg.DEAD_TIME   |  \
			Spi_8323_Vars->OCP_Ctrl_Reg.OCP_MODE     |  \
			Spi_8323_Vars->OCP_Ctrl_Reg.OCP_DEG      |  \
			Spi_8323_Vars->OCP_Ctrl_Reg.VDS_LVL;
    DRV8323_writeSpi(handle,drvRegName,drvDataNew);


    // Update CSA Control Register
    drvRegName = DRV8323_RegName_CSA_Control;
    drvDataNew = Spi_8323_Vars->CSA_Ctrl_Reg.CSA_FET |  \
    		Spi_8323_Vars->CSA_Ctrl_Reg.VREF_DIV   |  \
			Spi_8323_Vars->CSA_Ctrl_Reg.LS_REF     |  \
			Spi_8323_Vars->CSA_Ctrl_Reg.CSA_GAIN      |  \
			Spi_8323_Vars->CSA_Ctrl_Reg.DIS_SEN 	| \
			Spi_8323_Vars->CSA_Ctrl_Reg.CSA_CAL 	| \
			Spi_8323_Vars->CSA_Ctrl_Reg.SEN_LVL;
    DRV8323_writeSpi(handle,drvRegName,drvDataNew);


    Spi_8323_Vars->SndCmd = false;
  }

  return;
}  // end of DRV8323_writeData() function



void DRV8323_readData(DRV8323_Handle handle, DRV_SPI_8323_Vars_t *Spi_8323_Vars)
{
  DRV8323_RegName_e  drvRegName;
  uint16_t drvDataNew;


  if(Spi_8323_Vars->RcvCmd)
  {
    // Update Fault Status Register 1
    drvRegName = DRV8323_RegName_Fault_Status_1;
    drvDataNew = DRV8323_readSpi(handle,drvRegName);
    Spi_8323_Vars->Fault_Stat_Reg_1.FAULT = (bool)(drvDataNew & (uint16_t)DRV8323_STATUS1_FAULT_BITS);
    Spi_8323_Vars->Fault_Stat_Reg_1.VDS_OCP = (bool)(drvDataNew & (uint16_t)DRV8323_STATUS1_VDS_OCP_BITS);
    Spi_8323_Vars->Fault_Stat_Reg_1.GDF = (bool)(drvDataNew & (uint16_t)DRV8323_STATUS1_GDF_BITS);
    Spi_8323_Vars->Fault_Stat_Reg_1.UVLO = (bool)(drvDataNew & (uint16_t)DRV8323_STATUS1_UVLO_BITS);
    Spi_8323_Vars->Fault_Stat_Reg_1.OTSD = (bool)(drvDataNew & (uint16_t)DRV8323_STATUS1_OTSD_BITS);
    Spi_8323_Vars->Fault_Stat_Reg_1.VDS_HA = (bool)(drvDataNew & (uint16_t)DRV8323_STATUS1_VDS_HA_BITS);
    Spi_8323_Vars->Fault_Stat_Reg_1.VDS_LA = (bool)(drvDataNew & (uint16_t)DRV8323_STATUS1_VDS_LA_BITS);
    Spi_8323_Vars->Fault_Stat_Reg_1.VDS_HB = (bool)(drvDataNew & (uint16_t)DRV8323_STATUS1_VDS_HB_BITS);
    Spi_8323_Vars->Fault_Stat_Reg_1.VDS_LB = (bool)(drvDataNew & (uint16_t)DRV8323_STATUS1_VDS_LB_BITS);
    Spi_8323_Vars->Fault_Stat_Reg_1.VDS_HC = (bool)(drvDataNew & (uint16_t)DRV8323_STATUS1_VDS_HC_BITS);
    Spi_8323_Vars->Fault_Stat_Reg_1.VDS_LC = (bool)(drvDataNew & (uint16_t)DRV8323_STATUS1_VDS_LC_BITS);

    // Update Fault Status Register 2
    drvRegName = DRV8323_RegName_Fault_Status_2;
    drvDataNew = DRV8323_readSpi(handle,drvRegName);
    Spi_8323_Vars->Fault_Stat_Reg_2.SA_OC = (bool)(drvDataNew & (uint16_t)DRV8323_STATUS2_SA_OC_BITS);
    Spi_8323_Vars->Fault_Stat_Reg_2.SB_OC = (bool)(drvDataNew & (uint16_t)DRV8323_STATUS2_SB_OC_BITS);
    Spi_8323_Vars->Fault_Stat_Reg_2.SC_OC = (bool)(drvDataNew & (uint16_t)DRV8323_STATUS2_SC_OC_BITS);
    Spi_8323_Vars->Fault_Stat_Reg_2.OTW = (bool)(drvDataNew & (uint16_t)DRV8323_STATUS2_OTW_BITS);
    Spi_8323_Vars->Fault_Stat_Reg_2.CPUV = (bool)(drvDataNew & (uint16_t)DRV8323_STATUS2_CPUV_BITS);
    Spi_8323_Vars->Fault_Stat_Reg_2.VGS_HA = (bool)(drvDataNew & (uint16_t)DRV8323_STATUS2_VGS_HA_BITS);
    Spi_8323_Vars->Fault_Stat_Reg_2.VGS_LA = (bool)(drvDataNew & (uint16_t)DRV8323_STATUS2_VGS_LA_BITS);
    Spi_8323_Vars->Fault_Stat_Reg_2.VGS_HB = (bool)(drvDataNew & (uint16_t)DRV8323_STATUS2_VGS_HB_BITS);
    Spi_8323_Vars->Fault_Stat_Reg_2.VGS_LB = (bool)(drvDataNew & (uint16_t)DRV8323_STATUS2_VGS_LB_BITS);
    Spi_8323_Vars->Fault_Stat_Reg_2.VGS_HC = (bool)(drvDataNew & (uint16_t)DRV8323_STATUS2_VGS_HC_BITS);
    Spi_8323_Vars->Fault_Stat_Reg_2.VGS_LC = (bool)(drvDataNew & (uint16_t)DRV8323_STATUS2_VGS_LC_BITS);

    // Update Driver Control Register
    drvRegName = DRV8323_RegName_Driver_Control;
    drvDataNew = DRV8323_readSpi(handle,drvRegName);
    Spi_8323_Vars->Driver_Ctrl_Reg.RESERVED = (bool)(drvDataNew & (uint16_t)DRV8323_DRIVER_CTRL_RESERVED_BITS);
    Spi_8323_Vars->Driver_Ctrl_Reg.DIS_CPUV = (bool)(drvDataNew & (uint16_t)DRV8323_DRIVER_CTRL_DIS_CPUV_BITS);
    Spi_8323_Vars->Driver_Ctrl_Reg.DIS_GDF = (bool)(drvDataNew & (uint16_t)DRV8323_DRIVER_CTRL_DIS_GDF_BITS);
    Spi_8323_Vars->Driver_Ctrl_Reg.OTW_REP = (bool)(drvDataNew & (uint16_t)DRV8323_DRIVER_CTRL_OTW_REP_BITS);
    Spi_8323_Vars->Driver_Ctrl_Reg.PWM_MODE = (DRV8323_PwmMode_e)(drvDataNew & (uint16_t)DRV8323_DRIVER_CTRL_PWM_MODE_BITS);
    Spi_8323_Vars->Driver_Ctrl_Reg.PWM_1X_COM = (bool)(drvDataNew & (uint16_t)DRV8323_DRIVER_CTRL_1PWM_COM_BITS);
    Spi_8323_Vars->Driver_Ctrl_Reg.PWM_1X_DIR = (bool)(drvDataNew & (uint16_t)DRV8323_DRIVER_CTRL_1PWM_DIR_BITS);
    Spi_8323_Vars->Driver_Ctrl_Reg.COAST = (bool)(drvDataNew & (uint16_t)DRV8323_DRIVER_CTRL_COAST_BITS);
    Spi_8323_Vars->Driver_Ctrl_Reg.BRAKE = (bool)(drvDataNew & (uint16_t)DRV8323_DRIVER_CTRL_BRAKE_BITS);
    Spi_8323_Vars->Driver_Ctrl_Reg.CLR_FLT = (bool)(drvDataNew & (uint16_t)DRV8323_DRIVER_CTRL_CLR_FLT_BITS);

    // Update Gate Control HS Register
    drvRegName = DRV8323_RegName_Gate_Drive_HS_Control;
    drvDataNew = DRV8323_readSpi(handle,drvRegName);
    Spi_8323_Vars->Gate_Drive_HS_Reg.LOCK = (DRV8323_RegisterLockMode_e)(drvDataNew & (uint16_t)DRV8323_GATE_DRIVE_HS_LOCK_BITS);
    Spi_8323_Vars->Gate_Drive_HS_Reg.IDRIVEP_HS = (DRV8323_IDrivePeakSourceGateCurrent_e)(drvDataNew & (uint16_t)DRV8323_GATE_DRIVE_HS_IDRIVEP_HS_BITS);
    Spi_8323_Vars->Gate_Drive_HS_Reg.IDRIVEN_HS = (DRV8323_IDrivePeakSinkGateCurrent_e)(drvDataNew & (uint16_t)DRV8323_GATE_DRIVE_HS_IDRIVEN_HS_BITS);

    // Update Gate Control LS Register
    drvRegName = DRV8323_RegName_Gate_Drive_LS_Control;
    drvDataNew = DRV8323_readSpi(handle,drvRegName);
    Spi_8323_Vars->Gate_Drive_LS_Reg.CBC = (DRV8323_OcOFaultClearRetryMode_e)(drvDataNew & (uint16_t)DRV8323_GATE_DRIVE_LS_CBC_BITS);
    Spi_8323_Vars->Gate_Drive_LS_Reg.TDRIVE = (DRV8323_TDrive_Time_e)(drvDataNew & (uint16_t)DRV8323_GATE_DRIVE_LS_TDRIVE_BITS);
    Spi_8323_Vars->Gate_Drive_LS_Reg.IDRIVEP_LS = (DRV8323_IDrivePeakSourceGateCurrent_e)(drvDataNew & (uint16_t)DRV8323_GATE_DRIVE_LS_IDRIVEP_LS_BITS);
    Spi_8323_Vars->Gate_Drive_LS_Reg.IDRIVEN_LS = (DRV8323_IDrivePeakSinkGateCurrent_e)(drvDataNew & (uint16_t)DRV8323_GATE_DRIVE_LS_IDRIVEN_LS_BITS);

    // Update OCP Control Register
    drvRegName = DRV8323_RegName_OCP_Control;
    drvDataNew = DRV8323_readSpi(handle,drvRegName);
    Spi_8323_Vars->OCP_Ctrl_Reg.TRETRY = (DRV8323_OcRetryTime_e)(drvDataNew & (uint16_t)DRV8323_OCP_CTRL_TRETRY_BITS);
    Spi_8323_Vars->OCP_Ctrl_Reg.DEAD_TIME = (DRV8323_DeadTime_e)(drvDataNew & (uint16_t)DRV8323_OCP_CTRL_DEAD_TIME_BITS);
    Spi_8323_Vars->OCP_Ctrl_Reg.OCP_MODE = (DRV8323_OcMode_e)(drvDataNew & (uint16_t)DRV8323_OCP_CTRL_OCP_MODE_BITS);
    Spi_8323_Vars->OCP_Ctrl_Reg.OCP_DEG = (DRV8323_OcDeglitch_e)(drvDataNew & (uint16_t)DRV8323_OCP_CTRL_OCP_DEG_BITS);
    Spi_8323_Vars->OCP_Ctrl_Reg.VDS_LVL = (DRV8323_VdsLevel_e)(drvDataNew & (uint16_t)DRV8323_OCP_CTRL_VDS_LVL_BITS);

    // Update CSA Control Register
    drvRegName = DRV8323_RegName_CSA_Control;
    drvDataNew = DRV8323_readSpi(handle,drvRegName);
    Spi_8323_Vars->CSA_Ctrl_Reg.CSA_FET = (DRV8323_CSA_SensePositive_e)(drvDataNew & (uint16_t)DRV8323_CSA_CTRL_CSA_FET_BITS);
    Spi_8323_Vars->CSA_Ctrl_Reg.VREF_DIV = (DRV8323_CSA_DirectionMode_e)(drvDataNew & (uint16_t)DRV8323_CSA_CTRL_VREF_DIV_BITS);
    Spi_8323_Vars->CSA_Ctrl_Reg.LS_REF = (DRV8323_OcVDS_LS_Reference_e)(drvDataNew & (uint16_t)DRV8323_CSA_CTRL_LS_REF_BITS);
    Spi_8323_Vars->CSA_Ctrl_Reg.CSA_GAIN = (DRV8323_CSA_Gain_e)(drvDataNew & (uint16_t)DRV8323_CSA_CTRL_CSA_GAIN_BITS);
    Spi_8323_Vars->CSA_Ctrl_Reg.DIS_SEN = (DRV8323_OcOFaultMode_e)(drvDataNew & (uint16_t)DRV8323_CSA_CTRL_DIS_SEN_BITS);
    Spi_8323_Vars->CSA_Ctrl_Reg.CSA_CAL = (DRV8323_CSA_DCCalMode_e)(drvDataNew & (uint16_t)(DRV8323_CSA_CTRL_CSA_CAL_A_BITS | DRV8323_CSA_CTRL_CSA_CAL_B_BITS | DRV8323_CSA_CTRL_CSA_CAL_C_BITS));
    Spi_8323_Vars->CSA_Ctrl_Reg.SEN_LVL = (DRV8323_SenseOCLevel_e)(drvDataNew & (uint16_t)DRV8323_CSA_CTRL_SEN_LVL_BITS);

    Spi_8323_Vars->RcvCmd = false;
  }

  return;
}  // end of DRV8323_readData() function


void DRV8323_setupSpi(DRV8323_Handle handle, DRV_SPI_8323_Vars_t *Spi_8323_Vars)
{
  DRV8323_RegName_e  drvRegName;
  uint16_t drvDataNew;

//  Why impose hardcoded values?
//  Defaults should be device defaults or application level specified.
//  Setting other hardcoded here is just confusing!

#if 0
  // Update Control Register 1
  drvRegName = DRV8323_RegName_Control_1;
  drvDataNew = (DRV8323_PeakCurrent_0p25_A   | \
                DRV8323_Reset_Normal         | \
                DRV8323_PwmMode_Six_Inputs   | \
                DRV8323_OcMode_CurrentLimit  | \
                DRV8323_VdsLevel_0p730_V);
  DRV8323_writeSpi(handle,drvRegName,drvDataNew);

  // Update Control Register 2
  drvRegName = DRV8323_RegName_Control_2;
  drvDataNew = (DRV8323_OcTwMode_Both        | \
                DRV8323_ShuntAmpGain_10VpV   | \
                DRV8323_DcCalMode_Ch1_Load   | \
                DRV8323_DcCalMode_Ch2_Load   | \
                DRV8323_OcOffTimeMode_Normal);
  DRV8323_writeSpi(handle,drvRegName,drvDataNew);
#endif


  Spi_8323_Vars->SndCmd = false;
  Spi_8323_Vars->RcvCmd = false;


  // Wait for the DRV8323 registers to update
  osDelay(1);


  // Update Fault Status Register 1
      drvRegName = DRV8323_RegName_Fault_Status_1;
      drvDataNew = DRV8323_readSpi(handle,drvRegName);
      Spi_8323_Vars->Fault_Stat_Reg_1.FAULT = (bool)(drvDataNew & (uint16_t)DRV8323_STATUS1_FAULT_BITS);
      Spi_8323_Vars->Fault_Stat_Reg_1.VDS_OCP = (bool)(drvDataNew & (uint16_t)DRV8323_STATUS1_VDS_OCP_BITS);
      Spi_8323_Vars->Fault_Stat_Reg_1.GDF = (bool)(drvDataNew & (uint16_t)DRV8323_STATUS1_GDF_BITS);
      Spi_8323_Vars->Fault_Stat_Reg_1.UVLO = (bool)(drvDataNew & (uint16_t)DRV8323_STATUS1_UVLO_BITS);
      Spi_8323_Vars->Fault_Stat_Reg_1.OTSD = (bool)(drvDataNew & (uint16_t)DRV8323_STATUS1_OTSD_BITS);
      Spi_8323_Vars->Fault_Stat_Reg_1.VDS_HA = (bool)(drvDataNew & (uint16_t)DRV8323_STATUS1_VDS_HA_BITS);
      Spi_8323_Vars->Fault_Stat_Reg_1.VDS_LA = (bool)(drvDataNew & (uint16_t)DRV8323_STATUS1_VDS_LA_BITS);
      Spi_8323_Vars->Fault_Stat_Reg_1.VDS_HB = (bool)(drvDataNew & (uint16_t)DRV8323_STATUS1_VDS_HB_BITS);
      Spi_8323_Vars->Fault_Stat_Reg_1.VDS_LB = (bool)(drvDataNew & (uint16_t)DRV8323_STATUS1_VDS_LB_BITS);
      Spi_8323_Vars->Fault_Stat_Reg_1.VDS_HC = (bool)(drvDataNew & (uint16_t)DRV8323_STATUS1_VDS_HC_BITS);
      Spi_8323_Vars->Fault_Stat_Reg_1.VDS_LC = (bool)(drvDataNew & (uint16_t)DRV8323_STATUS1_VDS_LC_BITS);

      // Update Fault Status Register 2
      drvRegName = DRV8323_RegName_Fault_Status_2;
      drvDataNew = DRV8323_readSpi(handle,drvRegName);
      Spi_8323_Vars->Fault_Stat_Reg_2.SA_OC = (bool)(drvDataNew & (uint16_t)DRV8323_STATUS2_SA_OC_BITS);
      Spi_8323_Vars->Fault_Stat_Reg_2.SB_OC = (bool)(drvDataNew & (uint16_t)DRV8323_STATUS2_SB_OC_BITS);
      Spi_8323_Vars->Fault_Stat_Reg_2.SC_OC = (bool)(drvDataNew & (uint16_t)DRV8323_STATUS2_SC_OC_BITS);
      Spi_8323_Vars->Fault_Stat_Reg_2.OTW = (bool)(drvDataNew & (uint16_t)DRV8323_STATUS2_OTW_BITS);
      Spi_8323_Vars->Fault_Stat_Reg_2.CPUV = (bool)(drvDataNew & (uint16_t)DRV8323_STATUS2_CPUV_BITS);
      Spi_8323_Vars->Fault_Stat_Reg_2.VGS_HA = (bool)(drvDataNew & (uint16_t)DRV8323_STATUS2_VGS_HA_BITS);
      Spi_8323_Vars->Fault_Stat_Reg_2.VGS_LA = (bool)(drvDataNew & (uint16_t)DRV8323_STATUS2_VGS_LA_BITS);
      Spi_8323_Vars->Fault_Stat_Reg_2.VGS_HB = (bool)(drvDataNew & (uint16_t)DRV8323_STATUS2_VGS_HB_BITS);
      Spi_8323_Vars->Fault_Stat_Reg_2.VGS_LB = (bool)(drvDataNew & (uint16_t)DRV8323_STATUS2_VGS_LB_BITS);
      Spi_8323_Vars->Fault_Stat_Reg_2.VGS_HC = (bool)(drvDataNew & (uint16_t)DRV8323_STATUS2_VGS_HC_BITS);
      Spi_8323_Vars->Fault_Stat_Reg_2.VGS_LC = (bool)(drvDataNew & (uint16_t)DRV8323_STATUS2_VGS_LC_BITS);

      // Update Driver Control Register
      drvRegName = DRV8323_RegName_Driver_Control;
      drvDataNew = DRV8323_readSpi(handle,drvRegName);
      Spi_8323_Vars->Driver_Ctrl_Reg.RESERVED = (bool)(drvDataNew & (uint16_t)DRV8323_DRIVER_CTRL_RESERVED_BITS);
      Spi_8323_Vars->Driver_Ctrl_Reg.DIS_CPUV = (bool)(drvDataNew & (uint16_t)DRV8323_DRIVER_CTRL_DIS_CPUV_BITS);
      Spi_8323_Vars->Driver_Ctrl_Reg.DIS_GDF = (bool)(drvDataNew & (uint16_t)DRV8323_DRIVER_CTRL_DIS_GDF_BITS);
      Spi_8323_Vars->Driver_Ctrl_Reg.OTW_REP = (bool)(drvDataNew & (uint16_t)DRV8323_DRIVER_CTRL_OTW_REP_BITS);
      Spi_8323_Vars->Driver_Ctrl_Reg.PWM_MODE = (DRV8323_PwmMode_e)(drvDataNew & (uint16_t)DRV8323_DRIVER_CTRL_PWM_MODE_BITS);
      Spi_8323_Vars->Driver_Ctrl_Reg.PWM_1X_COM = (bool)(drvDataNew & (uint16_t)DRV8323_DRIVER_CTRL_1PWM_COM_BITS);
      Spi_8323_Vars->Driver_Ctrl_Reg.PWM_1X_DIR = (bool)(drvDataNew & (uint16_t)DRV8323_DRIVER_CTRL_1PWM_DIR_BITS);
      Spi_8323_Vars->Driver_Ctrl_Reg.COAST = (bool)(drvDataNew & (uint16_t)DRV8323_DRIVER_CTRL_COAST_BITS);
      Spi_8323_Vars->Driver_Ctrl_Reg.BRAKE = (bool)(drvDataNew & (uint16_t)DRV8323_DRIVER_CTRL_BRAKE_BITS);
      Spi_8323_Vars->Driver_Ctrl_Reg.CLR_FLT = (bool)(drvDataNew & (uint16_t)DRV8323_DRIVER_CTRL_CLR_FLT_BITS);

      // Update Gate Control HS Register
      drvRegName = DRV8323_RegName_Gate_Drive_HS_Control;
      drvDataNew = DRV8323_readSpi(handle,drvRegName);
      Spi_8323_Vars->Gate_Drive_HS_Reg.LOCK = (DRV8323_RegisterLockMode_e)(drvDataNew & (uint16_t)DRV8323_GATE_DRIVE_HS_LOCK_BITS);
      Spi_8323_Vars->Gate_Drive_HS_Reg.IDRIVEP_HS = (DRV8323_IDrivePeakSourceGateCurrent_e)(drvDataNew & (uint16_t)DRV8323_GATE_DRIVE_HS_IDRIVEP_HS_BITS);
      Spi_8323_Vars->Gate_Drive_HS_Reg.IDRIVEN_HS = (DRV8323_IDrivePeakSinkGateCurrent_e)(drvDataNew & (uint16_t)DRV8323_GATE_DRIVE_HS_IDRIVEN_HS_BITS);

      // Update Gate Control LS Register
      drvRegName = DRV8323_RegName_Gate_Drive_LS_Control;
      drvDataNew = DRV8323_readSpi(handle,drvRegName);
      Spi_8323_Vars->Gate_Drive_LS_Reg.CBC = (DRV8323_OcOFaultClearRetryMode_e)(drvDataNew & (uint16_t)DRV8323_GATE_DRIVE_LS_CBC_BITS);
      Spi_8323_Vars->Gate_Drive_LS_Reg.TDRIVE = (DRV8323_TDrive_Time_e)(drvDataNew & (uint16_t)DRV8323_GATE_DRIVE_LS_TDRIVE_BITS);
      Spi_8323_Vars->Gate_Drive_LS_Reg.IDRIVEP_LS = (DRV8323_IDrivePeakSourceGateCurrent_e)(drvDataNew & (uint16_t)DRV8323_GATE_DRIVE_LS_IDRIVEP_LS_BITS);
      Spi_8323_Vars->Gate_Drive_LS_Reg.IDRIVEN_LS = (DRV8323_IDrivePeakSinkGateCurrent_e)(drvDataNew & (uint16_t)DRV8323_GATE_DRIVE_LS_IDRIVEN_LS_BITS);

      // Update OCP Control Register
      drvRegName = DRV8323_RegName_OCP_Control;
      drvDataNew = DRV8323_readSpi(handle,drvRegName);
      Spi_8323_Vars->OCP_Ctrl_Reg.TRETRY = (DRV8323_OcRetryTime_e)(drvDataNew & (uint16_t)DRV8323_OCP_CTRL_TRETRY_BITS);
      Spi_8323_Vars->OCP_Ctrl_Reg.DEAD_TIME = (DRV8323_DeadTime_e)(drvDataNew & (uint16_t)DRV8323_OCP_CTRL_DEAD_TIME_BITS);
      Spi_8323_Vars->OCP_Ctrl_Reg.OCP_MODE = (DRV8323_OcMode_e)(drvDataNew & (uint16_t)DRV8323_OCP_CTRL_OCP_MODE_BITS);
      Spi_8323_Vars->OCP_Ctrl_Reg.OCP_DEG = (DRV8323_OcDeglitch_e)(drvDataNew & (uint16_t)DRV8323_OCP_CTRL_OCP_DEG_BITS);
      Spi_8323_Vars->OCP_Ctrl_Reg.VDS_LVL = (DRV8323_VdsLevel_e)(drvDataNew & (uint16_t)DRV8323_OCP_CTRL_VDS_LVL_BITS);

      // Update CSA Control Register
      drvRegName = DRV8323_RegName_CSA_Control;
      drvDataNew = DRV8323_readSpi(handle,drvRegName);
      Spi_8323_Vars->CSA_Ctrl_Reg.CSA_FET = (DRV8323_CSA_SensePositive_e)(drvDataNew & (uint16_t)DRV8323_CSA_CTRL_CSA_FET_BITS);
      Spi_8323_Vars->CSA_Ctrl_Reg.VREF_DIV = (DRV8323_CSA_DirectionMode_e)(drvDataNew & (uint16_t)DRV8323_CSA_CTRL_VREF_DIV_BITS);
      Spi_8323_Vars->CSA_Ctrl_Reg.LS_REF = (DRV8323_OcVDS_LS_Reference_e)(drvDataNew & (uint16_t)DRV8323_CSA_CTRL_LS_REF_BITS);
      Spi_8323_Vars->CSA_Ctrl_Reg.CSA_GAIN = (DRV8323_CSA_Gain_e)(drvDataNew & (uint16_t)DRV8323_CSA_CTRL_CSA_GAIN_BITS);
      Spi_8323_Vars->CSA_Ctrl_Reg.DIS_SEN = (DRV8323_OcOFaultMode_e)(drvDataNew & (uint16_t)DRV8323_CSA_CTRL_DIS_SEN_BITS);
      Spi_8323_Vars->CSA_Ctrl_Reg.CSA_CAL = (DRV8323_CSA_DCCalMode_e)(drvDataNew & (uint16_t)(DRV8323_CSA_CTRL_CSA_CAL_A_BITS | DRV8323_CSA_CTRL_CSA_CAL_B_BITS | DRV8323_CSA_CTRL_CSA_CAL_C_BITS));
      Spi_8323_Vars->CSA_Ctrl_Reg.SEN_LVL = (DRV8323_SenseOCLevel_e)(drvDataNew & (uint16_t)DRV8323_CSA_CTRL_SEN_LVL_BITS);

  return;
}


// end of file
