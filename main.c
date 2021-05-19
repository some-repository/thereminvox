#include "stm32f10x.h"
#include "sin_array.h"

void RCC_init (void);
void pulse_gen_init (void);
void ADC_init (void);
void SysTick_init (void);
void measure (void);
void start_measure (void);
void planner (void);
void PWM_init (void);
void sin_gen (void);
void freq_calc (void);

const int TimerTick = 1800; //40 kHz

typedef struct
{
	int measurement_freq;
	int measurement_vol;
} measurement;

extern measurement mes = {0, 0};
extern int spp = 10; //количество выборок на период синусоиды, spp = sample_frequency/frequency
extern int frequency_div_100 = 400;
extern int frequency = 100; //глобальная, чтобы можно было смотреть отладчиком
extern int cal_tick = 0; //счетчик калибровки
extern int freq_calc_sum = 0; //усреднение показаний АЦП для расчета частоты
extern int tick = 0; //счетчик планировщика
extern int sin_tick = 0; //счетчик генератора синуса
extern int cal_sum = 0; // усреднение для калибровки

int main()
{
	RCC_init ();
	pulse_gen_init ();
	PWM_init ();
	ADC_init ();
	SysTick_init ();
	
	while (1)
	{

	}
}

void RCC_init (void)
{
	/* Reset HSEON, CSSON and PLLON bits */
	RCC->CR &= (uint32_t)0xFEF6FFFF;
	
	RCC -> CR |= RCC_CR_HSEON; // Включить генератор HSE
	while (!((RCC -> CR) & RCC_CR_HSERDY)); // Ожидание готовности HSE
	RCC -> CFGR &= ~RCC_CFGR_SW; // Очистить биты SW0, SW1
	RCC -> CFGR |= RCC_CFGR_SW_HSE; // Выбрать HSE для тактирования SW0=1	
	
	//Включение буфера предвыборки и настройка задержек flash
	FLASH->ACR |= FLASH_ACR_PRFTBE; // Enable Prefetch Buffer.
	FLASH->ACR &= ~FLASH_ACR_LATENCY; // Предочистка
	FLASH->ACR |= FLASH_ACR_LATENCY_2; // Если 48< SystemCoreClock <= 72, пропускать 2 такта
	
	RCC -> CFGR &= ~((RCC_CFGR_PLLSRC|RCC_CFGR_PLLXTPRE|RCC_CFGR_PLLMULL)); // Предочистка
	RCC -> CFGR |= RCC_CFGR_PLLSRC_HSE; // Тактировать PLL от HSE (8 MHz)
	RCC -> CFGR |= RCC_CFGR_PLLMULL9; // Умножать частоту на 9 (8*9=72 MHz)
	RCC -> CR |= RCC_CR_PLLON; // Запустить PLL
	while ((RCC->CR & RCC_CR_PLLRDY)==0) {} // Ожидание готовности PLL
	RCC -> CFGR &= ~RCC_CFGR_SW; // Очистить биты SW0, SW1
	RCC -> CFGR |= RCC_CFGR_SW_PLL; // Тактирование с выхода PLL
	while (((RCC -> CFGR) & RCC_CFGR_SWS) != 0x08) {} // Ожидание переключения на PLL
}

void pulse_gen_init (void)
{
	//Тактирование  GPIOA , TIM1, альтернативных функций порта
	RCC -> APB2ENR |= (RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN);
	RCC -> APB1ENR |= RCC_APB1ENR_TIM2EN;
        
    //PA1 AF open drain
	GPIOA -> CRL &= ~GPIO_CRL_CNF1; //clean
	GPIOA -> CRL |= (GPIO_CRL_CNF1_1 | GPIO_CRL_CNF1_0);

	GPIOA -> CRL &= ~GPIO_CRL_MODE1; //clean
	GPIOA -> CRL |= (GPIO_CRL_MODE1_1 | GPIO_CRL_MODE1_0); //MODE11 = Max Speed 50MHz
	
	//импульсы 5 мкс
	//делитель
	TIM2 -> PSC = 72 - 1;
	//значение перезагрузки
	TIM2 -> ARR = 10 - 1;
	//коэф. заполнения
	TIM2 -> CCR2 = 5;
	
	//настроим на выход канал 2, активный уровень низкий
	TIM2 -> CCER |= (TIM_CCER_CC2E | TIM_CCER_CC2P);
	//разрешим использовать выводы таймера как выходы
	TIM2 -> BDTR |= TIM_BDTR_MOE;
	//PWM mode 1, прямой ШИМ 2 канал
	TIM2 -> CCMR1 = ((TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1) & (~TIM_CCMR1_OC2M_0));
	//считаем вверх
	TIM2 -> CR1 &= ~TIM_CR1_DIR;
	//выравнивание по фронту, Fast PWM
	TIM2 -> CR1 &= ~TIM_CR1_CMS;
	TIM2 -> CR1 |= TIM_CR1_OPM;
	//enable interrupt
	TIM2 -> DIER |= TIM_DIER_UIE;
	NVIC_EnableIRQ(TIM2_IRQn);
}

void ADC_init (void)
{
	RCC -> APB2ENR |= RCC_APB2ENR_ADC1EN; // ?????????? ???????????? ???
	RCC -> CFGR &= ~RCC_CFGR_ADCPRE_0;  // ???????????? ??? = 10 (/6)
	RCC -> CFGR |=  RCC_CFGR_ADCPRE_1;
	
	ADC1 -> CR1 = 0;      //Очистка
	ADC1 -> CR2 = 0;
	
	ADC1 -> SMPR2 &= ~(ADC_SMPR2_SMP0_0 | ADC_SMPR2_SMP0_1 | ADC_SMPR2_SMP0_2); //канал 0, время выборки 1,5 цикла
	ADC1 -> SMPR2 &= ~(ADC_SMPR2_SMP3_0 | ADC_SMPR2_SMP3_1 | ADC_SMPR2_SMP3_2); //канал 3, время выборки 1,5 цикла
	
	//Настройка инжектированных каналов
	ADC1 -> JSQR = 0; //clean
	ADC1 -> JSQR |= ADC_JSQR_JL_0; // 2 преобразования
	ADC1 -> JSQR |= ADC_JSQR_JSQ1_0; // 1е преобразование - канал 0
	ADC1 -> JSQR |= (ADC_JSQR_JSQ4_0 | ADC_JSQR_JSQ4_1); // 2е преобразование - канал 3
	
	ADC1->CR2 &= ~ADC_CR2_CONT; //запрет непрерывного режима
	ADC1->CR1 |= ADC_CR1_SCAN; //режим сканирования
	
	ADC1->CR2 |= ADC_CR2_JEXTSEL; // источник запуска - JSWSTART
	ADC1->CR2 |= ADC_CR2_JEXTTRIG; // разрешение внешнего запуска для инжектированных каналов
	
	//калибровка АЦП
	ADC1 -> CR2 |= ADC_CR2_ADON; //включить АЦП
	for (int i = 1; i < 10000; i++); //задержка перед калибровкой
	ADC1->CR2 |= ADC_CR2_CAL; // запуск калибровки
	while (((ADC1 -> CR2) & ADC_CR2_CAL) != 0); //  ожидание окончания калибровки
	ADC1 -> CR2 &= ~ADC_CR2_ADON; //выключить АЦП
}

void SysTick_init (void)
{
	SysTick_Config(TimerTick);
}

void SysTick_Handler(void)
{
	planner ();
}

void start_measure (void)
{
	GPIOA -> CRL &= ~(GPIO_CRL_MODE3 | GPIO_CRL_CNF3); // PA3 - analog input
	GPIOA -> CRL &= ~ (GPIO_CRL_MODE0 | GPIO_CRL_CNF0); // PA0 - analog input
	
	TIM2 -> CR1 |= TIM_CR1_CEN; //single pulse
}

void measure (void)
{
	ADC1 -> CR2 |= ADC_CR2_ADON;
	ADC1 -> CR2 |= ADC_CR2_JSWSTART; // запуск инжектированной группы
	while(!(ADC1 -> SR & ADC_SR_EOC)); // ожидание завершения преобразования
	ADC1 -> SR &= ~ADC_SR_EOC;
	ADC1 -> CR2 &= ~ADC_CR2_ADON;
	
	mes.measurement_freq = (int)ADC1 -> JDR1; //PA0
	mes.measurement_vol = (int)ADC1 -> JDR2; //PA3
	
	GPIOA -> CRL &= ~(GPIO_CRL_MODE3 | GPIO_CRL_CNF3); // PA3 - push-pull output
	GPIOA -> CRL |= GPIO_CRL_MODE3_1;
	GPIOA -> CRL &= ~(GPIO_CRL_MODE0 | GPIO_CRL_CNF0); // PA0 - push-pull output
	GPIOA -> CRL |= GPIO_CRL_MODE0_1;
	
	GPIOA -> BSRR |= ((1 << 3) | 1); //установка высокого уровня на PA0 и PA3
}

void TIM2_IRQHandler(void) 
{
    TIM2 -> SR &= ~TIM_SR_UIF; //сброс флага прерывания
	measure ();
}

void planner (void) //планировщик
{
	sin_gen ();
	
	if ((tick % 80) == 0) //раз в 2 мс в течение цикла (100 мс)
	{
		start_measure (); //500 измерений в секунду
		freq_calc ();
	}
	
	if (tick < 4000)
	{
		tick ++;
	}
	else
	{
		tick = 0; //обнуление раз в 100 мс
	};
}

void PWM_init (void)
{
	RCC -> APB2ENR |= RCC_APB2ENR_TIM1EN;
	
	//PA8 AF push-pull
	GPIOA -> CRH &= ~GPIO_CRH_CNF8;		// ?????????? ???? CNF ??? ???? 8. ????? 00 - Push-Pull 
	GPIOA -> CRH |= GPIO_CRH_CNF8_1;		// ?????? ????? ??? 8 ?? ???? ????? CNF  = 10 (?????????????? ???????, Push-Pull)
 
	GPIOA -> CRH &= ~GPIO_CRH_MODE8;				// ?????????? ???? MODE ??? ???? 8
	GPIOA -> CRH |= (GPIO_CRH_MODE8_1 | GPIO_CRH_MODE8_0);	// ?????????? ??? MODE ??? ?????? ????. ????? MODE11 = Max Speed 50MHz
	
	//делитель
	TIM1 -> PSC = 7 - 1;
	//значение перезагрузки
	TIM1 -> ARR = 255 - 1;
	
	//настроим на выход канал 1, активный уровень высокий
	TIM1 -> CCER |= TIM_CCER_CC1E;
	TIM1 -> CCER &= ~TIM_CCER_CC1P;
	//разрешим использовать выводы таймера как выходы
	TIM1 -> BDTR |= TIM_BDTR_MOE;
	//PWM mode 1, прямой ШИМ 1 канал
	TIM1 -> CCMR1 = ((TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1) & (~TIM_CCMR1_OC1M_0));
	//считаем вверх
	TIM1 -> CR1 |= TIM_CR1_DIR;
	//выравнивание по фронту, Fast PWM
	TIM1 -> CR1 &= ~TIM_CR1_CMS;
	TIM1 -> CR1 |= TIM_CR1_OPM;
}

void sin_gen (void)
{
	if (sin_tick < spp)
	{
		sin_tick ++;
	}
	else
	{
		sin_tick = 0;
	}
	
	int number = frequency_div_100 * sin_tick; //номер элемента массива
	
	TIM1 -> CCR1 = sin_arr [number]; //коэффициент заполнения
	TIM1 -> CR1 |= TIM_CR1_CEN; //single pulse
}

void freq_calc (void)
{
	freq_calc_sum = freq_calc_sum + mes.measurement_freq;
	if (tick == 3920)
	{
		if (cal_tick < 10) //измерение базового уровня при старте, среднее за 500 отсчетов
		{
			cal_tick ++;
			freq_calc_sum = freq_calc_sum / 50;
			cal_sum = cal_sum + freq_calc_sum;
			if (cal_tick == 10)
			{
				cal_sum = cal_sum / 10;
			}
			frequency = 4000;
			frequency_div_100 = frequency / 100;
			spp = 40000 / frequency;
		}
		else
		{
			freq_calc_sum = (freq_calc_sum / 50) - cal_sum; //(среднее за 50 измерений в течение 100 мс) - (базовое значение)
			if (freq_calc_sum < 0)
			{
				freq_calc_sum = 0;
			}
			frequency = (int) (((freq_calc_sum / (4095.0 - cal_sum)) * 2 * 3900.0) + 100);
			if (frequency > 4000)
			{
				frequency = 4000;
			}
			frequency_div_100 = frequency / 100;
			spp = 40000 / frequency;
		}
		freq_calc_sum = 0;
	}
}
