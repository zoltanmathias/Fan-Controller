 /**

******************************************************************************
  * @file    GPIO/GPIO_EXTI/Src/main.c
  * @author  MCD Application Team
  * @version V1.8.0
  * @date    21-April-2017
  * @brief   This example describes how to configure and use GPIOs through
  *          the STM32L4xx HAL API.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************


*/




/* Includes ------------------------------------------------------------------*/
#include "main.h"

/** @addtogroup STM32L4xx_HAL_Examples
  * @{
  */

/** @addtogroup GPIO_EXTI
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
__IO HAL_StatusTypeDef Hal_status;  

ADC_HandleTypeDef    Adc1_Handle;
ADC_ChannelConfTypeDef sConfig;

TIM_HandleTypeDef    Tim3_Handle, Tim4_Handle;
TIM_OC_InitTypeDef Tim3_OCInitStructure, Tim4_OCInitStructure;

uint16_t TIM4Prescaler;  
uint16_t TIM4Period;     
__IO uint16_t TIM4_CCR1_Val=0;

uint16_t TIM3_Prescaler;        //Timer to record temperature
__IO uint16_t TIM3_CCR4_Val;

__IO uint32_t ADC1ConvertedValue;   //changed to 32.

int selectCounter=0; // used to determine enter and exit of change setpoint mode(enter=odd, exit=even)
int button=2; //used to determine if up (0) or down (1) is pressed, init to 2, so start off with no button pressed
volatile double  setPoint=25;
int fanState=0; //determine state of fan 0=off, 1=low 2= med 3=high

double measuredTemp; 

char lcd_buffer[6];    // LCD display buffer


/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void Error_Handler(void);

void ADC_Config(void); 
void TIM3_Config(void);
void Show_Temperature(void);




/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */

void  TIM4_PWM_Config(void)   // timer 4, PB6 
{

  TIM4Prescaler=(uint16_t) (SystemCoreClock/ 50000) - 1;    //frequency is 50 Khz.
	TIM4Period=1000;   //period is 20ms 
	
	Tim4_Handle.Instance = TIM4; 
   
  Tim4_Handle.Init.Period = TIM4Period; 
  Tim4_Handle.Init.Prescaler = TIM4Prescaler;
  Tim4_Handle.Init.ClockDivision = 0;
  Tim4_Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
 
 
	if(HAL_TIM_PWM_Init(&Tim4_Handle) != HAL_OK)
  {
    Error_Handler();
  }
  
	//configuring the PWM channel
	Tim4_OCInitStructure.OCMode=  TIM_OCMODE_PWM1; 
	Tim4_OCInitStructure.OCFastMode=TIM_OCFAST_DISABLE;
	Tim4_OCInitStructure.OCPolarity=TIM_OCPOLARITY_HIGH;
	
	Tim4_OCInitStructure.Pulse=TIM4_CCR1_Val;
	
	if(HAL_TIM_PWM_ConfigChannel(&Tim4_Handle, &Tim4_OCInitStructure, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
	
	 if(HAL_TIM_PWM_Start(&Tim4_Handle, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }  
	
}


int main(void)
{
  /* STM32L4xx HAL library initialization:
       - Configure the Flash prefetch
       - Systick timer is configured by default as source of time base, but user 
         can eventually implement his proper time base source (a general purpose 
         timer for example or other time source), keeping in mind that Time base 
         duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and 
         handled in milliseconds basis.
       - Set NVIC Group Priority to 4 
       - Low Level Initialization
     */

  HAL_Init();
  HAL_NVIC_SetPriority(SysTick_IRQn, 0 ,0);
  SystemClock_Config();   

  HAL_InitTick(0x0000); // set systick's priority to the highest.

  
  BSP_LED_Init(LED4);
  BSP_LED_Init(LED5);


  BSP_LCD_GLASS_Init();
  
  BSP_JOY_Init(JOY_MODE_EXTI);  
  TIM3_Config();
  ADC_Config();
  Show_Temperature();

  while (1)
  {
    if(selectCounter==1)//enter change setTemp mode
    {
      switch(button)
      {
        case 0: //up, so increment by 0.5 celsius
          setPoint+=0.5;
					BSP_LED_Toggle(LED4);
          sprintf(lcd_buffer,"%0.7f",setPoint);
					BSP_LCD_GLASS_Clear();
          BSP_LCD_GLASS_DisplayString((uint8_t *)lcd_buffer);
          button=2;
          break;
        case 1: //down, so decrement by 0.5 celsius
          setPoint+=-0.5;
          sprintf(lcd_buffer,"%0.7f",setPoint);
					BSP_LED_Toggle(LED4);
					BSP_LCD_GLASS_Clear();
          BSP_LCD_GLASS_DisplayString((uint8_t *)lcd_buffer);
          button=2; //set button back to null basically
          break;
				default:
					break;
      }
    }
		
  } //end of while 1

}


/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follows :
  *            System Clock source            = MSI
  *            SYSCLK(Hz)                     = 4000000
  *            HCLK(Hz)                       = 4000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 1
  *            MSI Frequency(Hz)              = 4000000
  *            Flash Latency(WS)              = 0
  * @param  None
  * @retval None
  */


void SystemClock_Config(void)
{ 
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};                                            

 
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;            
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;  
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6; 
  RCC_OscInitStruct.MSICalibrationValue= RCC_MSICALIBRATION_DEFAULT;

  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40; 
  RCC_OscInitStruct.PLL.PLLR = 2;  
  RCC_OscInitStruct.PLL.PLLP = 7;  
  RCC_OscInitStruct.PLL.PLLQ = 4; 
  

  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    // Initialization Error 
    while(1);
  }

  // setting the clocks dividers 
  
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;  
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  
  
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) 
  {
    // Initialization Error 
    while(1);
  }

  __HAL_RCC_PWR_CLK_ENABLE(); // Enable Power Control clock

  if(HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE2) != HAL_OK)
  {
    // Initialization Error 
    while(1);
  }

  
  __HAL_RCC_PWR_CLK_DISABLE();  // Disable Power Control clock     
}


void Show_Temperature(void) 
{
	measuredTemp=0.02442*ADC1ConvertedValue; //voltage resolution is 3/4095, have to divide by amplification factor of 3
																					// then divide by temp sensitivty of 0.01 mV/C to get 0.02442*ADC1ConvertedVaue
	sprintf(lcd_buffer,"T%5.2f",measuredTemp);
	BSP_LCD_GLASS_DisplayString((uint8_t *)lcd_buffer);
}

void ADC_Config(void){    //use ADC1, channel 6, which connects to DMA1/channel1 or DMA2/channel3
  

  /******************
  
  1. Initialize the ADC low level resources by implementing the HAL_ADC_MspInit():
      a. Enable the ADC interface clock using __HAL_RCC_ADC_CLK_ENABLE()
      b. ADC pins configuration
          ? Enable the clock for the ADC GPIOs using the following function:
          __HAL_RCC_GPIOx_CLK_ENABLE()
          ? Configure these ADC pins in analog mode using HAL_GPIO_Init()
      c. In case of using interrupts (e.g. HAL_ADC_Start_IT())
          ? Configure the ADC interrupt priority using HAL_NVIC_SetPriority()
          ? Enable the ADC IRQ handler using HAL_NVIC_EnableIRQ()
          ? In ADC IRQ handler, call HAL_ADC_IRQHandler()
      d. In case of using DMA to control data transfer (e.g. HAL_ADC_Start_DMA())
          ? Enable the DMAx interface clock using
          __HAL_RCC_DMAx_CLK_ENABLE()
          ? Configure and enable two DMA streams stream for managing data transfer
          from peripheral to memory (output stream)
          ? Associate the initialized DMA handle to the CRYP DMA handle using
          __HAL_LINKDMA()
          ? Configure the priority and enable the NVIC for the transfer complete
          interrupt on the two DMA Streams. The output stream should have higher
          priority than the input stream.
  
  
  ************************/ 
    Adc1_Handle.Instance          = ADC1;
    
    if (HAL_ADC_DeInit(&Adc1_Handle) != HAL_OK)
    {
      Error_Handler();
    }
    
    Adc1_Handle.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;         // MUST USE THIS VALUE! OTHERWISE adc MAY STOP WORKING. //Asynchronous clock mode, input ADC clock not divided 
                                      //ADC_CLOCKPRESCALER_PCLK_DIV2;
      
    Adc1_Handle.Init.Resolution = ADC_RESOLUTION_12B; //can be 6b, 8b, 10b, or 12b. for 12b, it taks 12 clock cycle to convert
    Adc1_Handle.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    
    Adc1_Handle.Init.ScanConvMode = DISABLE;
    Adc1_Handle.Init.EOCSelection = ADC_EOC_SINGLE_CONV;//DISABLE;  //the HAL manual show the possible values are: ADC_EOC_SEQ_CONV, or ADC_EOC_SINGLE_CONV. 
    Adc1_Handle.Init.LowPowerAutoWait      = DISABLE;                     
  
  
    Adc1_Handle.Init.ContinuousConvMode = ENABLE;
    Adc1_Handle.Init.NbrOfConversion = 1; //this parameter' value ranges from 0000 to 1111,(this value will be written to ADC_SQR1 regitster)
                                            //0000--for 1 conversion, 0001---for 2 conversion, ......1111--for 16 conversion
                                            //here set as 1, is it correct?   
    Adc1_Handle.Init.DiscontinuousConvMode = DISABLE;
    Adc1_Handle.Init.NbrOfDiscConversion = 1;   
    
    //Adc1_Handle.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T3_CC4; //ADC control register 2 (ADC_CR2), defines certain external events to trigger
    Adc1_Handle.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    
    Adc1_Handle.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE; 
    //Adc1_Handle.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING; 
    
    Adc1_Handle.Init.DMAContinuousRequests = ENABLE;
    
    Adc1_Handle.Init.Overrun               = ADC_OVR_DATA_OVERWRITTEN;     
    Adc1_Handle.Init.OversamplingMode      = DISABLE;                       
    
    
    
    if(HAL_ADC_Init(&Adc1_Handle) != HAL_OK)
    {
      Error_Handler(); 
    }
    
    
  if (HAL_ADCEx_Calibration_Start(&Adc1_Handle, ADC_SINGLE_ENDED) !=  HAL_OK)  //necessary!!! otherwise, reading (DR) will be different.
  {
    Error_Handler();
  }
  
    
    
    //##-2- Configure ADC regular channel ###################################### 
    sConfig.Channel = ADC_CHANNEL_6;     //ADC1, IN6 is on PA1
    sConfig.Rank = ADC_REGULAR_RANK_1;  //1;
    sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;    
    
    sConfig.SingleDiff   = ADC_SINGLE_ENDED;            
    sConfig.OffsetNumber = ADC_OFFSET_NONE;            
    
    sConfig.Offset = 0;
    
    if(HAL_ADC_ConfigChannel(&Adc1_Handle, &sConfig) != HAL_OK)   
    {
      Error_Handler(); 
    }
  
     if(HAL_ADC_Start_DMA(&Adc1_Handle, (uint32_t*)&ADC1ConvertedValue, 1) != HAL_OK)
    {
      Error_Handler(); 
    }
    
} 

void  TIM3_Config(void)
{
  TIM3_Prescaler=(uint16_t)(SystemCoreClock/ 10000) - 1;  // 10Khz and 10 ticks making it one millisecond.
  TIM3_CCR4_Val=5000; 
  
  Tim3_Handle.Instance = TIM3; 
   
  Tim3_Handle.Init.Period = 65535;  //set it to max
  Tim3_Handle.Init.Prescaler = TIM3_Prescaler;
  Tim3_Handle.Init.ClockDivision = 0;
  Tim3_Handle.Init.CounterMode = TIM_COUNTERMODE_UP;

  if (HAL_TIM_OC_Init(&Tim3_Handle)!=HAL_OK) {  

        Error_Handler();
  }


    /* configuring the OC*/
    Tim3_OCInitStructure.OCMode=  TIM_OCMODE_TIMING; 
    Tim3_OCInitStructure.Pulse=TIM3_CCR4_Val;
    Tim3_OCInitStructure.OCPolarity=TIM_OCPOLARITY_HIGH;
    
    if (HAL_TIM_OC_ConfigChannel(&Tim3_Handle, &Tim3_OCInitStructure, TIM_CHANNEL_4) !=HAL_OK) {

  
        Error_Handler();
    }
            
    if (HAL_TIM_OC_Start_IT(&Tim3_Handle, TIM_CHANNEL_4)!=HAL_OK) { 
            Error_Handler();
    }
      
}

/**
  * @brief EXTI line detection callbacks
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  switch (GPIO_Pin) {
      case GPIO_PIN_0:                   //SELECT button          
            BSP_LED_Toggle(LED5);
						selectCounter++;
            selectCounter=selectCounter % 2;
            break;  

      case GPIO_PIN_1:     //left button            
              
              break;
      case GPIO_PIN_2:    //right button              to play again.
            
              break;
      case GPIO_PIN_3:    //up button             
              button=0;
							//TIM4_CCR1_Val+=100;
							//TIM4_PWM_Config();
							
              break;
      
      case GPIO_PIN_5:    //down button           
              button=1;
							//TIM4_CCR1_Val-=100;
							//TIM4_PWM_Config();
              break;
      
      default://
            //default
            break;
    } 
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef * htim) //see  stm32XXX_hal_tim.c for different callback function names. 
{                                                               //for timer4 
  if ((*htim).Instance==TIM3){
		if(selectCounter==0)Show_Temperature();
		if(measuredTemp<setPoint)//set CCR to 0 when less than setpoint
		{
			fanState=0;
			TIM4_CCR1_Val=0;
		}else{
			fanState=1;
			TIM4_CCR1_Val=250+(measuredTemp-setPoint)*150; //increase the pwm width linearly with an increase in the diff of measured and setpoint
			if(TIM4_CCR1_Val>1000)TIM4_CCR1_Val=1000;
		}
		
		__HAL_TIM_SET_COMPARE(&Tim4_Handle, TIM_CHANNEL_1,TIM4_CCR1_Val);
		TIM4_PWM_Config();
		

        __HAL_TIM_SET_COUNTER(htim, 0x0000);   //this maro is defined in stm32f4xx_hal_tim.h
   }
}
 
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef * htim){  //this is for TIM4_pwm
  
  __HAL_TIM_SET_COUNTER(htim, 0x0000);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* Adc1_Handle)
{
  ADC1ConvertedValue = HAL_ADC_GetValue(Adc1_Handle); //get converted value

}



static void Error_Handler(void)
{
  /* Turn LED4 on */
  BSP_LED_On(LED4);
  while(1)
  {
  }
}





#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(char *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/