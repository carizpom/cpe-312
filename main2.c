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

#define released 0
#define pressed 1
#define E_06 (uint16_t)1318
#define MUTE (uint16_t)1

#define TIMx_PSC
#define ARR_CALCULATE(N) ((32000000) / ((TIMx_PSC) * (N)))

unsigned int sw,state;
char disp_str[] = "1811340015";
void SystemClock_Config(void);
uint16_t adc_data = 0;
uint8_t prog_state = 0;
uint8_t sw_cnt = 0;

void TIM_BASE_Config(uint16_t);
void TIM_OC_Config(uint16_t);
void TIM_OC_GPIO_Config(void);
void TIM_BASE_DurationConfig(void);



int main()
{
	RCC -> CR |= (1<<0);//Hi-speed Clk en
	while((RCC->CR & 0x02)==0x02);
	
	RCC -> APB2ENR |= (1<<9);//adc
	RCC -> APB2ENR |= (1<<0);//sys config
	SYSCFG -> EXTICR[0] &= ~(15<<0);
	
	EXTI -> IMR |= (1<<0);
	EXTI -> RTSR |= (1<<0);
	
	NVIC_EnableIRQ((IRQn_Type)6);
	NVIC_SetPriority((IRQn_Type)6,0);
	
	SystemClock_Config();
	LCD_GLASS_Init();	
	
	RCC -> AHBENR |= (1<<0)|(1<<1)|(1<<3); //enable GPIO ABCD
	
	GPIOA -> MODER &= ~(3<<0);//PA0
	GPIOA -> MODER |= (1<<24)|(3<<10);//LED
	GPIOB -> MODER |= (1<<12);//LED
	GPIOD -> MODER |= (1<<4);//LED
	
	ADC1->CR1 |= (1<<24)|(1<<11);
	ADC1->CR1 &= ~(7<<13);
	ADC1->CR2 &= ~(1<<11);//alignment
	ADC1->SMPR3 |= (2<<15);
	ADC1->SQR5 |= (5<<0);
	ADC1->CR2 |= (1<<0);//A/D Convert
	LCD_GLASS_DisplayString((uint8_t*)disp_str);
	
	RCC->APB1ENR |= (1<<29);
	GPIOA -> MODER |= (3<<8);//DAC
	DAC ->CR |=(1<<0);
	
	//Note_cpe-312
	TIM_OC_Config(ARR_CALCULATE(E_O6));
	TIM_BASE_DurationConfig();
	while(1)
	{	
		ADC1->CR2 |= (1<<30);//start regular conversion
		while((ADC1->SR & (1<<1))==0);//Regular channel end of conversion
		adc_data = ADC1->DR;
		sprintf(disp_str, "%d",adc_data);
		LCD_GLASS_Clear();
		LCD_GLASS_DisplayString((uint8_t*)disp_str);
		switch (prog_state) 
		{
			case 0:
				if(adc_data >=0 && adc_data <2)
				{
					GPIOB -> ODR &= ~(1<<6);
					GPIOA -> ODR &= (0<<12);
					GPIOD -> ODR &= (0<<2);
					DAC->DHR12R1 = 0x0FFF;
				}
				else if(adc_data >=2 && adc_data <=341)
				{
					GPIOB -> ODR |=  (1<<6);
					GPIOA -> ODR &= (0<<12);
					GPIOD -> ODR &= (0<<2);
					DAC->DHR12R1 = 0x0FFF;
					if(LL_TIM_IsActiveFlag_UPDATE(TIM2) == SET)
					{
						LL_TIM_ClearFlag_UPDATE(TIM2);
						LL_TIM_SetAutoReload(TIM4, MUTE); //Change ARR of Timer PWM
						LL_TIM_SetCounter(TIM2, 0);
					}
				}
				else if	(adc_data >=342 && adc_data <=682)
				{
					GPIOB -> ODR |= (1<<6);
					GPIOA -> ODR &=  (0<<12);
					GPIOD -> ODR |= (1<<2);
					DAC->DHR12R1 = 0x0FFF/2;
					if(LL_TIM_IsActiveFlag_UPDATE(TIM2) == SET)
					{
						LL_TIM_ClearFlag_UPDATE(TIM2);
						LL_TIM_SetAutoReload(TIM4, MUTE); //Change ARR of Timer PWM
						LL_TIM_SetCounter(TIM2, 0);
					}
				}
				else if	(adc_data >=683 && adc_data <=1023)
				{
					GPIOB -> ODR |=  (1<<6);
					GPIOA -> ODR |=  (1<<12);
					GPIOD -> ODR |= (1<<2);
					DAC->DHR12R1 = 0x00FA;
				}
				break;
			case 1:
				if(adc_data >=0 && adc_data < 2)
				{
					GPIOB -> ODR |= (1<<6);
					GPIOA -> ODR |= (1<<12);
					GPIOD -> ODR |= (1<<2);
					DAC->DHR12R1 = 0x00FA;
					if(LL_TIM_IsActiveFlag_UPDATE(TIM2) == SET)
					{
						LL_TIM_ClearFlag_UPDATE(TIM2);
						LL_TIM_SetAutoReload(TIM4, MUTE); //Change ARR of Timer PWM
						LL_TIM_SetCounter(TIM2, 0);
					}
				}
				else if(adc_data >=2 && adc_data <=341)
				{
					GPIOB -> ODR &= (0<<6);
					GPIOA -> ODR |= (1<<12);
					GPIOD -> ODR |= (1<<2);
					DAC->DHR12R1 = 0x0FFF/2;
					if(LL_TIM_IsActiveFlag_UPDATE(TIM2) == SET)
					{
						LL_TIM_ClearFlag_UPDATE(TIM2);
						LL_TIM_SetAutoReload(TIM4, MUTE); //Change ARR of Timer PWM
						LL_TIM_SetCounter(TIM2, 0);
					}
				}
				else if	(adc_data >=342 && adc_data <=682)
				{
					GPIOB -> ODR &= (0<<6);
					GPIOA -> ODR |=  (1<<12);
					GPIOD -> ODR &=  (0<<2);
					DAC->DHR12R1 = 0x0FFF;
					if(LL_TIM_IsActiveFlag_UPDATE(TIM2) == SET)
					{
						LL_TIM_ClearFlag_UPDATE(TIM2);
						LL_TIM_SetAutoReload(TIM4, MUTE); //Change ARR of Timer PWM
						LL_TIM_SetCounter(TIM2, 0);
					}
				}
				else if	(adc_data >=683 && adc_data <=1023)
				{
					GPIOB -> ODR &=  (0<<6);
					GPIOA -> ODR &=  (0<<12);
					GPIOD -> ODR &= (0<<2);
					DAC->DHR12R1 = 0x0FFF;
				}
				break;
			default:
				break;
		}
	}
	
}
//begin
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
//end edit
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
void EXTI0_IRQHandler(void)
{
	if((EXTI->PR & 0x01) == 0x01)
	{
		EXTI->PR |= (1<<0);
	}
	if((GPIOA -> IDR & 0x01) == 0x01)
		{prog_state ^= 1;}


}