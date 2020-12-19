#include "stm32l1xx.h"
#include "stm32l1xx_ll_system.h"
#include "stm32l1xx_ll_bus.h"
#include "stm32l1xx_ll_gpio.h"
#include "stm32l1xx_ll_pwr.h"
#include "stm32l1xx_ll_rcc.h"
#include "stm32l1xx_ll_utils.h"
#include "stm32l1xx_ll_tim.h"

#define TIMx_PSC			3200 
#define TIMx_ARR			100

//7segment
#define Num_0	(uint32_t) LL_GPIO_PIN_2  | LL_GPIO_PIN_10 | LL_GPIO_PIN_11 | LL_GPIO_PIN_12 | LL_GPIO_PIN_13 | LL_GPIO_PIN_14 
#define Num_1	(uint32_t) LL_GPIO_PIN_10 | LL_GPIO_PIN_11 
#define Num_2	(uint32_t) LL_GPIO_PIN_2  | LL_GPIO_PIN_10 | LL_GPIO_PIN_12 | LL_GPIO_PIN_13 | LL_GPIO_PIN_15 
#define Num_3	(uint32_t) LL_GPIO_PIN_2  | LL_GPIO_PIN_10 | LL_GPIO_PIN_11 | LL_GPIO_PIN_12 | LL_GPIO_PIN_15 
#define Num_4	(uint32_t) LL_GPIO_PIN_10 | LL_GPIO_PIN_11 | LL_GPIO_PIN_14 | LL_GPIO_PIN_15 
#define Num_5 (uint32_t) LL_GPIO_PIN_2 | LL_GPIO_PIN_14 | LL_GPIO_PIN_15 | LL_GPIO_PIN_11 | LL_GPIO_PIN_12
#define Num_6	(uint32_t) LL_GPIO_PIN_2  | LL_GPIO_PIN_11 | LL_GPIO_PIN_12 | LL_GPIO_PIN_13 | LL_GPIO_PIN_14 | LL_GPIO_PIN_15 
#define Num_7	(uint32_t) LL_GPIO_PIN_2  | LL_GPIO_PIN_10 | LL_GPIO_PIN_11  
#define Num_8	(uint32_t) LL_GPIO_PIN_2  | LL_GPIO_PIN_10 | LL_GPIO_PIN_11 | LL_GPIO_PIN_12 | LL_GPIO_PIN_13 | LL_GPIO_PIN_14 | LL_GPIO_PIN_15 
#define Num_9	(uint32_t) LL_GPIO_PIN_2  | LL_GPIO_PIN_10 | LL_GPIO_PIN_11 | LL_GPIO_PIN_12 | LL_GPIO_PIN_14 | LL_GPIO_PIN_15 

uint8_t i;
uint32_t seg0percent[4] = {Num_0,	Num_0, Num_0, Num_0};
uint32_t seg20percent[4] = {Num_0,	Num_0, Num_2, Num_0};	
uint32_t seg40percent[4] = {Num_0,	Num_0, Num_4, Num_0};
uint32_t seg60percent[4] = {Num_0,	Num_0, Num_6, Num_0};
uint32_t seg80percent[4] = {Num_0,	Num_0, Num_8, Num_0};
uint32_t seg100percent[4] = {Num_0,	Num_1, Num_0, Num_0};

uint32_t digit[4] = { LL_GPIO_PIN_0, LL_GPIO_PIN_1, LL_GPIO_PIN_2, LL_GPIO_PIN_3 };


uint16_t x = 0;

void SystemClock_Config(void);
void TIM_BASE_Config(void);
void TIM_OC_Config(void);
void GPIO_Config(void);
void EXTI_Config(void);

int main()
{
	SystemClock_Config();
	GPIO_Config();
	TIM_BASE_Config();
	EXTI_Config();
	
	LL_GPIO_SetOutputPin(GPIOB,LL_GPIO_PIN_7);
	LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_4);
	
	while(1)
	{
		TIM_OC_Config();
	}

}

void SevenSegment_Display()
{
	
	if (x == 0)
		{
			for(i=0;i<4;i++){
				LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_0 | LL_GPIO_PIN_1 | LL_GPIO_PIN_2 | LL_GPIO_PIN_3);
        LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_2 | LL_GPIO_PIN_10 | LL_GPIO_PIN_11 | LL_GPIO_PIN_12 | LL_GPIO_PIN_13 | LL_GPIO_PIN_14 | LL_GPIO_PIN_15); 
				LL_GPIO_SetOutputPin(GPIOB, seg0percent[i]);
				LL_GPIO_SetOutputPin(GPIOC, digit[i]);
				LL_mDelay(1);
			}
		}
		else if (x == 20)
		{
			for(i=0;i<4;i++){
				LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_0 | LL_GPIO_PIN_1 | LL_GPIO_PIN_2 | LL_GPIO_PIN_3);
        LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_2 | LL_GPIO_PIN_10 | LL_GPIO_PIN_11 | LL_GPIO_PIN_12 | LL_GPIO_PIN_13 | LL_GPIO_PIN_14 | LL_GPIO_PIN_15); 
				LL_GPIO_SetOutputPin(GPIOB, seg20percent[i]);
				LL_GPIO_SetOutputPin(GPIOC, digit[i]);
				LL_mDelay(1);
			}
		}
		else if (x == 40)
		{
			for(i=0;i<4;i++){
				LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_0 | LL_GPIO_PIN_1 | LL_GPIO_PIN_2 | LL_GPIO_PIN_3);
        LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_2 | LL_GPIO_PIN_10 | LL_GPIO_PIN_11 | LL_GPIO_PIN_12 | LL_GPIO_PIN_13 | LL_GPIO_PIN_14 | LL_GPIO_PIN_15); 
				LL_GPIO_SetOutputPin(GPIOB, seg40percent[i]);
				LL_GPIO_SetOutputPin(GPIOC, digit[i]);
				LL_mDelay(1);
			}
		}
		else if (x == 60)
		{
			for(i=0;i<4;i++){
				LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_0 | LL_GPIO_PIN_1 | LL_GPIO_PIN_2 | LL_GPIO_PIN_3);
        LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_2 | LL_GPIO_PIN_10 | LL_GPIO_PIN_11 | LL_GPIO_PIN_12 | LL_GPIO_PIN_13 | LL_GPIO_PIN_14 | LL_GPIO_PIN_15); 
				LL_GPIO_SetOutputPin(GPIOB, seg60percent[i]);
				LL_GPIO_SetOutputPin(GPIOC, digit[i]);
				LL_mDelay(1);
			}
			
		}
		else if (x == 80)
		{
			for(i=0;i<4;i++){
				LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_0 | LL_GPIO_PIN_1 | LL_GPIO_PIN_2 | LL_GPIO_PIN_3);
        LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_2 | LL_GPIO_PIN_10 | LL_GPIO_PIN_11 | LL_GPIO_PIN_12 | LL_GPIO_PIN_13 | LL_GPIO_PIN_14 | LL_GPIO_PIN_15); 
				LL_GPIO_SetOutputPin(GPIOB, seg80percent[i]);
				LL_GPIO_SetOutputPin(GPIOC, digit[i]);
				LL_mDelay(1);
			}	
			
		}
		else if (x == 100)
		{
			for(i=0;i<4;i++){
				LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_0 | LL_GPIO_PIN_1 | LL_GPIO_PIN_2 | LL_GPIO_PIN_3);
        LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_2 | LL_GPIO_PIN_10 | LL_GPIO_PIN_11 | LL_GPIO_PIN_12 | LL_GPIO_PIN_13 | LL_GPIO_PIN_14 | LL_GPIO_PIN_15); 
				LL_GPIO_SetOutputPin(GPIOB, seg100percent[i]);
				LL_GPIO_SetOutputPin(GPIOC, digit[i]);
				LL_mDelay(1);
			}
		}
}

void EXTI0_IRQHandler(void)
{
		EXTI->PR |= (1<<0);
		
		if(LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_0) == RESET) //Press OFF Switch
		{
			if (x >= 100)
				x = 0;
			else
				x = x+20;
		}	
 
}

void EXTI_Config()
{
		 RCC->APB2ENR |= (1<<0);								// Enable EXTICR2 (EXITCH Line0)

     SYSCFG->EXTICR[0] &= ~(15<<0);					// EXTICR1 :	EXTI0 enable to PA0
		 EXTI->IMR  |= (1<<0);      						// enable Interrupt masking registor to EXTI0 and EXTI1
     EXTI->FTSR |= (1<<0); 									// enable Falling edge at EXTI0 and EXTI1
			
     //Enable IRQ
     NVIC_EnableIRQ((IRQn_Type)6);
     NVIC_SetPriority((IRQn_Type)6,0);

}

void GPIO_Config(void)
{
	LL_GPIO_InitTypeDef l239d;
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
	
	l239d.Mode = LL_GPIO_MODE_OUTPUT;
	l239d.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	l239d.Pull = LL_GPIO_PULL_NO;
	l239d.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	l239d.Pin = LL_GPIO_PIN_4 | LL_GPIO_PIN_7;
	LL_GPIO_Init(GPIOB,&l239d);
	
	l239d.Mode = LL_GPIO_MODE_INPUT;
	l239d.Pin = LL_GPIO_PIN_0;
	LL_GPIO_Init(GPIOA,&l239d);
	
	l239d.Mode = LL_GPIO_MODE_ALTERNATE;
	l239d.Alternate = LL_GPIO_AF_2;
	l239d.Pin = LL_GPIO_PIN_5; 
	LL_GPIO_Init(GPIOB,&l239d);

	LL_GPIO_InitTypeDef Itc4727_initstruct;

 	Itc4727_initstruct.Mode = LL_GPIO_MODE_OUTPUT;
  Itc4727_initstruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  Itc4727_initstruct.Pull = LL_GPIO_PULL_NO;
  Itc4727_initstruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  Itc4727_initstruct.Pin = LL_GPIO_PIN_2 | LL_GPIO_PIN_10 | LL_GPIO_PIN_11 | LL_GPIO_PIN_12 | LL_GPIO_PIN_13 | LL_GPIO_PIN_14 | LL_GPIO_PIN_15;
  LL_GPIO_Init(GPIOB, &Itc4727_initstruct);

  Itc4727_initstruct.Pin = LL_GPIO_PIN_0 | LL_GPIO_PIN_1 | LL_GPIO_PIN_2 | LL_GPIO_PIN_3 ;
  LL_GPIO_Init(GPIOC , &Itc4727_initstruct);
}

void TIM_BASE_Config(void)
{
	LL_TIM_InitTypeDef timbase_initstructure;
	
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);
	
	timbase_initstructure.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
	timbase_initstructure.CounterMode = LL_TIM_COUNTERMODE_UP;
	timbase_initstructure.Autoreload = TIMx_ARR - 1;
	timbase_initstructure.Prescaler =  TIMx_PSC- 1;
	
	LL_TIM_Init(TIM3, &timbase_initstructure);

}

void TIM_OC_Config(void)
{
	LL_TIM_OC_InitTypeDef tim_oc_initstructure;
	
	TIM_BASE_Config();

	tim_oc_initstructure.OCState = LL_TIM_OCSTATE_DISABLE;
	tim_oc_initstructure.OCMode = LL_TIM_OCMODE_PWM1;
	tim_oc_initstructure.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
	tim_oc_initstructure.CompareValue = x; 
	LL_TIM_OC_Init(TIM3, LL_TIM_CHANNEL_CH2, &tim_oc_initstructure);
	
	/*Interrupt Configure*/
	NVIC_SetPriority(TIM3_IRQn, 29);
	NVIC_EnableIRQ(TIM3_IRQn);
	LL_TIM_EnableIT_CC2(TIM3);
	
	/*Start Output Compare in PWM Mode*/
	LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH2);
	LL_TIM_EnableCounter(TIM3);
}

void TIM3_IRQHandler(void)
{
	if(LL_TIM_IsActiveFlag_CC2(TIM3) == SET)
	{
		LL_TIM_ClearFlag_CC2(TIM3);
	}
}

void SystemClock_Config(void)
{
  /* Enable ACC64 access and set FLASH latency */ 
  LL_FLASH_Enable64bitAccess();; 
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);

  /* Set Voltage scale1 as MCU will run at 32MHz */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  
  /* Poll VOSF bit of in PWR_CSR. Wait until it is reset to 0 */
  while (LL_PWR_IsActiveFlag_VOSF() != 0)
  {
  };
  
  /* Enable HSI if not already activated*/
  if (LL_RCC_HSI_IsReady() == 0)
  {
    /* HSI configuration and activation */
    LL_RCC_HSI_Enable();
    while(LL_RCC_HSI_IsReady() != 1)
    {
    };
  }
  
	
  /* Main PLL configuration and activation */
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLL_MUL_6, LL_RCC_PLL_DIV_3);

  LL_RCC_PLL_Enable();
  while(LL_RCC_PLL_IsReady() != 1)
  {
  };
  
  /* Sysclk activation on the main PLL */
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  };
  
  /* Set APB1 & APB2 prescaler*/
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);

  /* Set systick to 1ms in using frequency set to 32MHz                             */
  /* This frequency can be calculated through LL RCC macro                          */
  /* ex: __LL_RCC_CALC_PLLCLK_FREQ (HSI_VALUE, LL_RCC_PLL_MUL_6, LL_RCC_PLL_DIV_3); */
  LL_Init1msTick(32000000);
  
  /* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
  LL_SetSystemCoreClock(32000000);
}

