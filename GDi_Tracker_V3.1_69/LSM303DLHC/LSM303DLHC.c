
#include "stm32f30x.h"
#include "stm32f30x_exti.h"
#include "stm32f30x_i2c.h"
#include "stm32f30x_syscfg.h"
#include "LSM303DLHC.h"
#include "LedIndicator.h"

__IO uint32_t  LSM303DLHC_Timeout = LSM303DLHC_LONG_TIMEOUT; 

void AccelConfig(void)
{
  LSM303DLHCMag_InitTypeDef LSM303DLHC_InitStructure;
  LSM303DLHCAcc_InitTypeDef LSM303DLHCAcc_InitStructure;
  LSM303DLHCAcc_FilterConfigTypeDef LSM303DLHCFilter_InitStructure;
  
  /* Configure MEMS magnetometer main parameters: temp, working mode, full Scale and Data rate */
  LSM303DLHC_InitStructure.Temperature_Sensor = LSM303DLHC_TEMPSENSOR_DISABLE;
  LSM303DLHC_InitStructure.MagOutput_DataRate =LSM303DLHC_ODR_30_HZ ;
  LSM303DLHC_InitStructure.MagFull_Scale = LSM303DLHC_FS_8_1_GA;
  LSM303DLHC_InitStructure.Working_Mode = LSM303DLHC_CONTINUOS_CONVERSION;
  LSM303DLHC_MagInit(&LSM303DLHC_InitStructure);
  
   /* Fill the accelerometer structure */
  LSM303DLHCAcc_InitStructure.Power_Mode = LSM303DLHC_NORMAL_MODE;
  LSM303DLHCAcc_InitStructure.AccOutput_DataRate = LSM303DLHC_ODR_50_HZ;
  LSM303DLHCAcc_InitStructure.Axes_Enable= LSM303DLHC_AXES_ENABLE;
  LSM303DLHCAcc_InitStructure.AccFull_Scale = LSM303DLHC_FULLSCALE_2G;
  LSM303DLHCAcc_InitStructure.BlockData_Update = LSM303DLHC_BlockUpdate_Continuos;
  LSM303DLHCAcc_InitStructure.Endianness=LSM303DLHC_BLE_LSB;
  LSM303DLHCAcc_InitStructure.High_Resolution=LSM303DLHC_HR_ENABLE;
  /* Configure the accelerometer main parameters */
  LSM303DLHC_AccInit(&LSM303DLHCAcc_InitStructure);
  
  /* Fill the accelerometer LPF structure */
  LSM303DLHCFilter_InitStructure.HighPassFilter_Mode_Selection =LSM303DLHC_HPM_NORMAL_MODE;
  LSM303DLHCFilter_InitStructure.HighPassFilter_CutOff_Frequency = LSM303DLHC_HPFCF_16;
  LSM303DLHCFilter_InitStructure.HighPassFilter_AOI1 = LSM303DLHC_HPF_AOI1_DISABLE;
  LSM303DLHCFilter_InitStructure.HighPassFilter_AOI2 = LSM303DLHC_HPF_AOI2_DISABLE;

  /* Configure the accelerometer LPF main parameters */
  LSM303DLHC_AccFilterConfig(&LSM303DLHCFilter_InitStructure);
}



void LSM303DLHC_LowLevel_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;
  I2C_InitTypeDef  I2C_InitStructure;
  
  RCC_APB1PeriphClockCmd(LSM303DLHC_I2C_CLK, ENABLE);//Enable the I2C periph 
  RCC_AHBPeriphClockCmd(LSM303DLHC_I2C_SCK_GPIO_CLK | LSM303DLHC_I2C_SDA_GPIO_CLK , ENABLE);//Enable SCK and SDA GPIO clocks  
  RCC_AHBPeriphClockCmd(LSM303DLHC_I2C_INT1_GPIO_CLK, ENABLE);//Enable INT1 GPIO clock 
  RCC_AHBPeriphClockCmd(LSM303DLHC_I2C_INT2_GPIO_CLK, ENABLE);// Enable INT2 GPIO clock 
  RCC_AHBPeriphClockCmd(LSM303DLHC_DRDY_GPIO_CLK, ENABLE);// Enable DRDY clock 
  
  GPIO_PinAFConfig(LSM303DLHC_I2C_SCK_GPIO_PORT, LSM303DLHC_I2C_SCK_SOURCE, LSM303DLHC_I2C_SCK_AF);
  GPIO_PinAFConfig(LSM303DLHC_I2C_SDA_GPIO_PORT, LSM303DLHC_I2C_SDA_SOURCE, LSM303DLHC_I2C_SDA_AF);
  
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  
  GPIO_InitStructure.GPIO_Pin = LSM303DLHC_I2C_SCK_PIN;
  GPIO_Init(LSM303DLHC_I2C_SCK_GPIO_PORT, &GPIO_InitStructure);//I2C SCK pin configuration 
  
  GPIO_InitStructure.GPIO_Pin =  LSM303DLHC_I2C_SDA_PIN;
  GPIO_Init(LSM303DLHC_I2C_SDA_GPIO_PORT, &GPIO_InitStructure);// I2C SDA pin configuration 
  
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Pin = LSM303DLHC_DRDY_PIN;
  GPIO_Init(LSM303DLHC_DRDY_GPIO_PORT, &GPIO_InitStructure);// Mems DRDY pin configuration 
  
  SYSCFG_EXTILineConfig(LSM303DLHC_DRDY_EXTI_PORT_SOURCE, LSM303DLHC_DRDY_EXTI_PIN_SOURCE);//Connect EXTI Line to Mems DRDY Pin 
  
  EXTI_InitStructure.EXTI_Line = LSM303DLHC_DRDY_EXTI_LINE;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  
  EXTI_Init(&EXTI_InitStructure);
  
   //-------------I2C configuration -------------------------------------------------------
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStructure.I2C_AnalogFilter = I2C_AnalogFilter_Enable;
  I2C_InitStructure.I2C_DigitalFilter = 0x00;
  I2C_InitStructure.I2C_OwnAddress1 = 0x00;
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_InitStructure.I2C_Timing = 0x00902025;
  I2C_Init(LSM303DLHC_I2C, &I2C_InitStructure);//Apply LSM303DLHC_I2C configuration after enabling it 
  I2C_Cmd(LSM303DLHC_I2C, ENABLE);//LSM303DLHC_I2C Peripheral Enable 
  
   //Configure GPIO PINs to detect Interrupts 
  GPIO_InitStructure.GPIO_Pin = LSM303DLHC_I2C_INT1_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_Init(LSM303DLHC_I2C_INT1_GPIO_PORT, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = LSM303DLHC_I2C_INT2_PIN;
  GPIO_Init(LSM303DLHC_I2C_INT2_GPIO_PORT, &GPIO_InitStructure);
} 


void LSM303DLHC_AccFilterConfig(LSM303DLHCAcc_FilterConfigTypeDef *LSM303DLHC_FilterStruct) 
{
  uint8_t tmpreg;
  
  /* Read CTRL_REG2 register */
  LSM303DLHC_Read(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG2_A, &tmpreg, 1);
  
  tmpreg &= 0x0C;
  
  /* Configure MEMS: mode, cutoff frquency, Filter status, Click, AOI1 and AOI2 */
  tmpreg |= (uint8_t) (LSM303DLHC_FilterStruct->HighPassFilter_Mode_Selection |\
                      LSM303DLHC_FilterStruct->HighPassFilter_CutOff_Frequency|\
                      LSM303DLHC_FilterStruct->HighPassFilter_AOI1|\
                      LSM303DLHC_FilterStruct->HighPassFilter_AOI2);                             
  
  /* Write value to ACC MEMS CTRL_REG2 regsister */
  LSM303DLHC_Write(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG2_A, &tmpreg);
}

void LSM303DLHC_AccInit(LSM303DLHCAcc_InitTypeDef *LSM303DLHC_InitStruct)
{  
  uint8_t ctrl1 = 0x00, ctrl4 = 0x00;
  
  /* Configure the low level interface ---------------------------------------*/
  LSM303DLHC_LowLevel_Init();
  
  /* Configure MEMS: data rate, power mode, full scale and axes */
  ctrl1 |= (uint8_t) (LSM303DLHC_InitStruct->Power_Mode | LSM303DLHC_InitStruct->AccOutput_DataRate | \
                    LSM303DLHC_InitStruct->Axes_Enable);
  
  ctrl4 |= (uint8_t) (LSM303DLHC_InitStruct->BlockData_Update | LSM303DLHC_InitStruct->Endianness | \
                    LSM303DLHC_InitStruct->AccFull_Scale|LSM303DLHC_InitStruct->High_Resolution);
                    
  /* Write value to ACC MEMS CTRL_REG1 regsister */
  LSM303DLHC_Write(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG1_A, &ctrl1);
  
  /* Write value to ACC MEMS CTRL_REG4 regsister */
  LSM303DLHC_Write(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG4_A, &ctrl4);
}

void LSM303DLHC_MagInit(LSM303DLHCMag_InitTypeDef *LSM303DLHC_InitStruct)
{  
  uint8_t cra_regm = 0x00, crb_regm = 0x00, mr_regm = 0x00;
  
  /* Configure the low level interface ---------------------------------------*/
  LSM303DLHC_LowLevel_Init();
  
  /* Configure MEMS: temp and Data rate */
  cra_regm |= (uint8_t) (LSM303DLHC_InitStruct->Temperature_Sensor | LSM303DLHC_InitStruct->MagOutput_DataRate);
    
  /* Configure MEMS: full Scale */
  crb_regm |= (uint8_t) (LSM303DLHC_InitStruct->MagFull_Scale);
      
  /* Configure MEMS: working mode */
  mr_regm |= (uint8_t) (LSM303DLHC_InitStruct->Working_Mode);
                    
  /* Write value to Mag MEMS CRA_REG regsister */
  LSM303DLHC_Write(MAG_I2C_ADDRESS, LSM303DLHC_CRA_REG_M, &cra_regm);
  
  /* Write value to Mag MEMS CRB_REG regsister */
  LSM303DLHC_Write(MAG_I2C_ADDRESS, LSM303DLHC_CRB_REG_M, &crb_regm);

  /* Write value to Mag MEMS MR_REG regsister */
  LSM303DLHC_Write(MAG_I2C_ADDRESS, LSM303DLHC_MR_REG_M, &mr_regm);
}

uint16_t LSM303DLHC_Read(uint8_t DeviceAddr, uint8_t RegAddr, uint8_t* pBuffer, uint16_t NumByteToRead)
{  
	LED_ACCEL_ON();  
  /* Test on BUSY Flag */
  LSM303DLHC_Timeout = LSM303DLHC_LONG_TIMEOUT;
  while(I2C_GetFlagStatus(LSM303DLHC_I2C, I2C_ISR_BUSY) != RESET)
  {
    if((LSM303DLHC_Timeout--) == 0) return LSM303DLHC_TIMEOUT_UserCallback();
  }
  
  /* Configure slave address, nbytes, reload, end mode and start or stop generation */
  I2C_TransferHandling(LSM303DLHC_I2C, DeviceAddr, 1, I2C_SoftEnd_Mode, I2C_Generate_Start_Write);
  
  /* Wait until TXIS flag is set */
  LSM303DLHC_Timeout = LSM303DLHC_LONG_TIMEOUT;
  while(I2C_GetFlagStatus(LSM303DLHC_I2C, I2C_ISR_TXIS) == RESET)
  {
    if((LSM303DLHC_Timeout--) == 0) return LSM303DLHC_TIMEOUT_UserCallback();
  }
  
  if(NumByteToRead>1)
      RegAddr |= 0x80;

  
  /* Send Register address */
  I2C_SendData(LSM303DLHC_I2C, (uint8_t)RegAddr);
  
  /* Wait until TC flag is set */
  LSM303DLHC_Timeout = LSM303DLHC_LONG_TIMEOUT;
  while(I2C_GetFlagStatus(LSM303DLHC_I2C, I2C_ISR_TC) == RESET)
  {
    if((LSM303DLHC_Timeout--) == 0) return LSM303DLHC_TIMEOUT_UserCallback();
  }  
  
  /* Configure slave address, nbytes, reload, end mode and start or stop generation */
  I2C_TransferHandling(LSM303DLHC_I2C, DeviceAddr, NumByteToRead, I2C_AutoEnd_Mode, I2C_Generate_Start_Read);
  
  /* Wait until all data are received */
  while (NumByteToRead)
  {   
    /* Wait until RXNE flag is set */
    LSM303DLHC_Timeout = LSM303DLHC_LONG_TIMEOUT;
    while(I2C_GetFlagStatus(LSM303DLHC_I2C, I2C_ISR_RXNE) == RESET)    
    {
      if((LSM303DLHC_Timeout--) == 0) return LSM303DLHC_TIMEOUT_UserCallback();
    }
    
    /* Read data from RXDR */
    *pBuffer = I2C_ReceiveData(LSM303DLHC_I2C);
    /* Point to the next location where the byte read will be saved */
    pBuffer++;
    
    /* Decrement the read bytes counter */
    NumByteToRead--;
  } 
  
  /* Wait until STOPF flag is set */
  LSM303DLHC_Timeout = LSM303DLHC_LONG_TIMEOUT;
  while(I2C_GetFlagStatus(LSM303DLHC_I2C, I2C_ISR_STOPF) == RESET)   
  {
    if((LSM303DLHC_Timeout--) == 0) return LSM303DLHC_TIMEOUT_UserCallback();
  }
  
  /* Clear STOPF flag */
  I2C_ClearFlag(LSM303DLHC_I2C, I2C_ICR_STOPCF);
	LED_ACCEL_OFF();
  
  /* If all operations OK */
  return LSM303DLHC_OK;  
}  


uint16_t LSM303DLHC_Write(uint8_t DeviceAddr, uint8_t RegAddr, uint8_t* pBuffer)
{  
  /* Test on BUSY Flag */
  LSM303DLHC_Timeout = LSM303DLHC_LONG_TIMEOUT;
  while(I2C_GetFlagStatus(LSM303DLHC_I2C, I2C_ISR_BUSY) != RESET)
  {
    if((LSM303DLHC_Timeout--) == 0) return LSM303DLHC_TIMEOUT_UserCallback();
  }
  
  /* Configure slave address, nbytes, reload, end mode and start or stop generation */
  I2C_TransferHandling(LSM303DLHC_I2C, DeviceAddr, 1, I2C_Reload_Mode, I2C_Generate_Start_Write);
  
  /* Wait until TXIS flag is set */
  LSM303DLHC_Timeout = LSM303DLHC_LONG_TIMEOUT;  
  while(I2C_GetFlagStatus(LSM303DLHC_I2C, I2C_ISR_TXIS) == RESET)   
  {
    if((LSM303DLHC_Timeout--) == 0) return LSM303DLHC_TIMEOUT_UserCallback();
  }
  
  /* Send Register address */
  I2C_SendData(LSM303DLHC_I2C, (uint8_t) RegAddr);
  
  /* Wait until TCR flag is set */
  LSM303DLHC_Timeout = LSM303DLHC_LONG_TIMEOUT;
  while(I2C_GetFlagStatus(LSM303DLHC_I2C, I2C_ISR_TCR) == RESET)
  {
    if((LSM303DLHC_Timeout--) == 0) return LSM303DLHC_TIMEOUT_UserCallback();
  }
  
  /* Configure slave address, nbytes, reload, end mode and start or stop generation */
  I2C_TransferHandling(LSM303DLHC_I2C, DeviceAddr, 1, I2C_AutoEnd_Mode, I2C_No_StartStop);
       
  /* Wait until TXIS flag is set */
  LSM303DLHC_Timeout = LSM303DLHC_LONG_TIMEOUT;
  while(I2C_GetFlagStatus(LSM303DLHC_I2C, I2C_ISR_TXIS) == RESET)
  {
    if((LSM303DLHC_Timeout--) == 0) return LSM303DLHC_TIMEOUT_UserCallback();
  }  
    
  /* Write data to TXDR */
  I2C_SendData(LSM303DLHC_I2C, *pBuffer);
      
  /* Wait until STOPF flag is set */
  LSM303DLHC_Timeout = LSM303DLHC_LONG_TIMEOUT;
  while(I2C_GetFlagStatus(LSM303DLHC_I2C, I2C_ISR_STOPF) == RESET)
  {
    if((LSM303DLHC_Timeout--) == 0) return LSM303DLHC_TIMEOUT_UserCallback();
  }   
  
  /* Clear STOPF flag */
  I2C_ClearFlag(LSM303DLHC_I2C, I2C_ICR_STOPCF);
  
  return LSM303DLHC_OK;
}






