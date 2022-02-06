#include "stm32l1xx.h"

#include "stm32l1xx_ll_system.h"
#include "stm32l1xx_ll_bus.h"
#include "stm32l1xx_ll_gpio.h"
#include "stm32l1xx_ll_rcc.h"
#include "stm32l1xx_ll_pwr.h"
#include "stm32l1xx_ll_utils.h"
#include "stm32l1xx_ll_adc.h"
#include "stm32l1xx_ll_usart.h"

#include "stm32l1xx_ll_lcd.h"
#include "stm32l152_glass_lcd.h"
#include "stm32l1xx_ll_tim.h"
#include "stdio.h"

#define E_O6					(uint16_t)1318
#define MUTE					(uint16_t) 1

/*for 10ms update event*/
#define TIMx_PSC			2 

/*Macro function for ARR calculation*/
#define ARR_CALCULATE(N) ((32000000) / ((TIMx_PSC) * (N)))


void TIM_BASE_Config(uint16_t);
void TIM_OC_Config(uint16_t);
void TIM_OC_GPIO_Config(void);
void TIM_BASE_DurationConfig(void);

void SystemClock_Config(void);

void TIMx_IC_Config(void)
{
		LL_GPIO_InitTypeDef timic_gpio;
		LL_TIM_InitTypeDef timbase;
		LL_TIM_IC_InitTypeDef timic;
		
		LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
		
		timic_gpio.Mode = LL_GPIO_MODE_ALTERNATE;
		timic_gpio.Pull = LL_GPIO_PULL_DOWN;
		timic_gpio.Pin = LL_GPIO_PIN_1 | LL_GPIO_PIN_3;
		timic_gpio.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
		timic_gpio.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
		timic_gpio.Alternate = LL_GPIO_AF_1;
		LL_GPIO_Init(GPIOA, &timic_gpio);
		
		LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);
		
		//TIM_IC Configure CH1
		timic.ICActiveInput = LL_TIM_ACTIVEINPUT_DIRECTTI;
		timic.ICFilter = LL_TIM_IC_FILTER_FDIV1;
		timic.ICPolarity = LL_TIM_IC_POLARITY_RISING;
		timic.ICPrescaler = LL_TIM_ICPSC_DIV1;
		LL_TIM_IC_Init(TIM2, LL_TIM_CHANNEL_CH2, &timic);
		
		//TIM_IC Configure CH4
		timic.ICPolarity = LL_TIM_IC_POLARITY_FALLING;
		LL_TIM_IC_Init(TIM2, LL_TIM_CHANNEL_CH4, &timic);
		
		NVIC_SetPriority(TIM2_IRQn, 0);
		NVIC_EnableIRQ(TIM2_IRQn);
		
		LL_TIM_EnableIT_CC2(TIM2);
		LL_TIM_EnableIT_CC4(TIM2);
		
		LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH2);
		LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH4);
		
		LL_TIM_EnableCounter(TIM2);
}

void GPIO_Config(void)
{
		LL_GPIO_InitTypeDef timic_gpio;
		
		LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
		
		timic_gpio.Mode = LL_GPIO_MODE_OUTPUT;
		timic_gpio.Pull = LL_GPIO_PULL_NO;
		timic_gpio.Pin = LL_GPIO_PIN_2;
		timic_gpio.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
		timic_gpio.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
		LL_GPIO_Init(GPIOA, &timic_gpio);
	
		LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_2);
}

uint16_t uwIC1 = 0;
uint16_t uwIC2 = 0;
uint16_t uwDiff = 0;
uint16_t uhICIndex = 0;
uint8_t state = 0;

float period = 0;
float distance = 0;
uint16_t i;

uint32_t digit[4] = {LL_GPIO_PIN_0 , LL_GPIO_PIN_1 , LL_GPIO_PIN_2 , LL_GPIO_PIN_3};


int main()
{
		uint32_t TIM2CLK;
		uint32_t PSC;
		uint32_t IC1PSC;
	
		SystemClock_Config();
		GPIO_Config();
		TIMx_IC_Config();
        TIM_OC_Config(ARR_CALCULATE(E_O6));
	    TIM_BASE_DurationConfig();
		while(1)
		{
			switch(state)
			{
				case 0:
					//Trigger measurement
					LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_2);
					LL_mDelay(1);
					LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_2);
                    GPIOB -> ODR &= ~(1<<4);
					state = 1;
				break;
				
				case 1:
					//Measure uptime using TIM_IC, if uhIndex == 2, rising and falling duration is measured
					if(uhICIndex == 1)
					{
							//Period calculation
							PSC = LL_TIM_GetPrescaler(TIM2) + 1;
							TIM2CLK = SystemCoreClock / PSC;
							IC1PSC = __LL_TIM_GET_ICPSC_RATIO(LL_TIM_IC_GetPrescaler(TIM2, LL_TIM_CHANNEL_CH1));
							
							period = (uwDiff*(PSC) * 1.0) / (TIM2CLK *IC1PSC * 1.0); //calculate uptime period
							distance = (period * 340) / 2; //meter unit
                            GPIOA -> ODR &= (0<<5);
                            LL_TIM_ClearFlag_UPDATE(TIM2);
			                LL_TIM_SetAutoReload(TIM4, MUTE); //Change ARR of Timer PWM
			                LL_TIM_SetCounter(TIM2, 0);
							uhICIndex = 0;
							state = 0;
					}
				break;
					
			}
			
			for(i=0;i<4;i++){
			LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_0 | LL_GPIO_PIN_1 | LL_GPIO_PIN_2 | LL_GPIO_PIN_3);//Write 0 to GPIOC
			LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_2 | LL_GPIO_PIN_10 | LL_GPIO_PIN_11 | LL_GPIO_PIN_12 | LL_GPIO_PIN_13 | LL_GPIO_PIN_14 | LL_GPIO_PIN_15);//Reser all segment
			LL_GPIO_SetOutputPin(GPIOC, digit[i]);
		}
			
	}
}


void TIM2_IRQHandler(void)
{	  
		if(LL_TIM_IsActiveFlag_CC2(TIM2) == SET)
		{
				//Detect rising edge
				uwIC1 = LL_TIM_IC_GetCaptureCH2(TIM2);
				LL_TIM_ClearFlag_CC2(TIM2);	
		}
		if(LL_TIM_IsActiveFlag_CC4(TIM2) == SET)
		{
				uwIC2 = LL_TIM_IC_GetCaptureCH4(TIM2);
				
				if(uwIC2 > uwIC1)
				{
					uwDiff = uwIC2 - uwIC1;
				}
				else if(uwIC2 < uwIC1)
				{
					uwDiff = ((LL_TIM_GetAutoReload(TIM2) - uwIC1) + uwIC2) + 1;
				}
				uhICIndex = 1;
				LL_TIM_ClearFlag_CC4(TIM2);
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



void TIM_OC_GPIO_Config(void)
{
	LL_GPIO_InitTypeDef gpio_initstructure;
	
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
	
	gpio_initstructure.Mode = LL_GPIO_MODE_ALTERNATE;
	gpio_initstructure.Alternate = LL_GPIO_AF_2;
	gpio_initstructure.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	gpio_initstructure.Pin = LL_GPIO_PIN_6;
	gpio_initstructure.Pull = LL_GPIO_PULL_NO;
	gpio_initstructure.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	LL_GPIO_Init(GPIOB, &gpio_initstructure);
}

void TIM_OC_Config(uint16_t note)
{
	LL_TIM_OC_InitTypeDef tim_oc_initstructure;
	
	TIM_OC_GPIO_Config();
	TIM_BASE_Config(note);
	
	tim_oc_initstructure.OCState = LL_TIM_OCSTATE_DISABLE;
	tim_oc_initstructure.OCMode = LL_TIM_OCMODE_PWM1;
	tim_oc_initstructure.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
	tim_oc_initstructure.CompareValue = LL_TIM_GetAutoReload(TIM4) / 2;
	LL_TIM_OC_Init(TIM4, LL_TIM_CHANNEL_CH1, &tim_oc_initstructure);
	/*Interrupt Configure*/
	NVIC_SetPriority(TIM4_IRQn, 1);
	NVIC_EnableIRQ(TIM4_IRQn);
	LL_TIM_EnableIT_CC1(TIM4);
	
	/*Start Output Compare in PWM Mode*/
	LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH1);
	LL_TIM_EnableCounter(TIM4);
}

void TIM_BASE_Config(uint16_t ARR)
{
	LL_TIM_InitTypeDef timbase_initstructure;
	
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM4);
	//Time-base configure
	timbase_initstructure.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
	timbase_initstructure.CounterMode = LL_TIM_COUNTERMODE_UP;
	timbase_initstructure.Autoreload = ARR - 1;
	timbase_initstructure.Prescaler =  TIMx_PSC- 1;
	LL_TIM_Init(TIM4, &timbase_initstructure);
	
	LL_TIM_EnableCounter(TIM4); 
}

void TIM_BASE_DurationConfig(void)
{
	LL_TIM_InitTypeDef timbase_initstructure;
	
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);
	//Time-base configure
	timbase_initstructure.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
	timbase_initstructure.CounterMode = LL_TIM_COUNTERMODE_UP;
	timbase_initstructure.Autoreload = 3000 - 1;
	timbase_initstructure.Prescaler =  32000 - 1;
	LL_TIM_Init(TIM2, &timbase_initstructure);
	
	LL_TIM_EnableCounter(TIM2); 
	LL_TIM_ClearFlag_UPDATE(TIM2); //Force clear update flag
}
