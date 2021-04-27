#include "stm32f10x.h"

int RCC_init (void);
void MCO_init (void);
void PWM_init (void);
void ADC_init (void);

volatile int res_1 = 0;
volatile int res_2 = 0;

int main()
{
	RCC_init ();
	MCO_init ();
	PWM_init ();
	ADC_init ();
	
	
	
	while (1)
	{
		ADC1->CR2 |= ADC_CR2_JSWSTART; // запуск инжектированной группы
		while(!(ADC1->SR & ADC_SR_EOC)); // ожидание завершения преобразования
		ADC1->SR &= ~ADC_SR_EOC;
		res_1 = (int)ADC1 -> JDR1; //
		res_2 = (int)ADC1 -> JDR2; //
		
		for (int i = 1; i < 10000; i++); // задержка
	}
}

int RCC_init (void)
{
	/* Reset HSEON, CSSON and PLLON bits */
	RCC->CR &= (uint32_t)0xFEF6FFFF;
	
	RCC -> CR |= RCC_CR_HSEON; // Включить генератор HSE.
	while (!((RCC -> CR) & RCC_CR_HSERDY)) {}; // Ожидание готовности HSE.
	RCC -> CFGR &= ~RCC_CFGR_SW; // Очистить биты SW0, SW1.
	RCC -> CFGR |= RCC_CFGR_SW_HSE; // Выбрать HSE для тактирования SW0=1.	
	
	FLASH->ACR |= FLASH_ACR_PRFTBE; // Enable Prefetch Buffer.
	FLASH->ACR &= ~FLASH_ACR_LATENCY; // Предочистка.
	FLASH->ACR |= FLASH_ACR_LATENCY_2; // Если 48< SystemCoreClock <= 72, пропускать 2 такта.
	
	RCC -> CFGR &= ~((RCC_CFGR_PLLSRC|RCC_CFGR_PLLXTPRE|RCC_CFGR_PLLMULL)); // Предочистка.
	RCC -> CFGR |= RCC_CFGR_PLLSRC_HSE; // Тактировать PLL от HSE (8 MHz).
	RCC -> CFGR |= RCC_CFGR_PLLMULL9; // Умножать частоту на 9 (8*9=72 MHz).
	RCC -> CR |= RCC_CR_PLLON; // Запустить PLL.
	while ((RCC->CR & RCC_CR_PLLRDY)==0) {} // Ожидание готовности PLL.
	RCC -> CFGR &= ~RCC_CFGR_SW; // Очистить биты SW0, SW1.
	RCC -> CFGR |= RCC_CFGR_SW_PLL; // Тактирование с выхода PLL.
	while (((RCC -> CFGR) & RCC_CFGR_SWS) != 0x08) {} // Ожидание переключения на PLL.
	return 0;
}

void MCO_init (void)
{
	RCC -> APB2ENR 	|= RCC_APB2ENR_IOPAEN;		// Подаем тактирование на порт
 
	GPIOA -> CRH	&= ~GPIO_CRH_CNF8;		// Сбрасываем биты CNF для бита 8. Режим 00 - Push-Pull 
	GPIOA -> CRH	|= GPIO_CRH_CNF8_1;		// Ставим режим для 8 го бита режим CNF  = 10 (альтернативная функция, Push-Pull)
 
	GPIOA -> CRH	&= ~GPIO_CRH_MODE8;				// Сбрасываем биты MODE для бита 8
	GPIOA -> CRH	|= (GPIO_CRH_MODE8_1 | GPIO_CRH_MODE8_0);	// Выставляем бит MODE для пятого пина. Режим MODE11 = Max Speed 50MHz
 
	RCC -> CFGR	&= ~RCC_CFGR_MCO;		// Обнуляем MCO
	RCC -> CFGR	|= RCC_CFGR_MCO_PLL;		// Выставлем для MCO сигнал с PLL/2
	//RCC -> CFGR	|= RCC_CFGR_MCO_SYSCLK;		// Выставляем для МСО сигнал с SYSCLK
}

void PWM_init (void)
{
	// Тактирование  GPIOA , TIM1, альтернативных функций порта
	RCC->APB2ENR |= (RCC_APB2ENR_IOPAEN | RCC_APB2ENR_TIM1EN | RCC_APB2ENR_AFIOEN);
        
    //PA11 push-pull
	GPIOA -> CRH &= ~GPIO_CRH_CNF11; // Предочистка.
	GPIOA -> CRH |= GPIO_CRH_CNF11_1;

	GPIOA -> CRH	&= ~GPIO_CRH_MODE11; // Предочистка.
	GPIOA -> CRH	|= (GPIO_CRH_MODE11_1 | GPIO_CRH_MODE11_0); //Режим MODE11 = Max Speed 50MHz
	
	//100 kHz, коэф. заполнения 0,5
	//делитель
	TIM1 -> PSC = 72 - 1;
	//значение перезагрузки
    TIM1 -> ARR = 10 - 1;
	//коэф. заполнения
	TIM1 -> CCR4 = 5;
	
	//настроим на выход канал 4, активный уровень высокий 
	TIM1 -> CCER |= (TIM_CCER_CC4E & ~TIM_CCER_CC4P);
	//разрешим использовать выводы таймера как выходы
	TIM1 -> BDTR |= TIM_BDTR_MOE;
	//PWM mode 1, прямой ШИМ 4 канал
    TIM1 -> CCMR2 = ((TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1) & (~TIM_CCMR2_OC4M_0));
    //если надо настроить первый канал, это можно сделать так
    //TIM1->CCMR1 = TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1;
	//считаем вверх
	TIM1 -> CR1 &= ~TIM_CR1_DIR;
	//выравнивание по фронту, Fast PWM
	TIM1 -> CR1 &= ~TIM_CR1_CMS;
	//включаем счётчик
	TIM1 -> CR1 |= TIM_CR1_CEN;
}

void ADC_init (void)
{
	GPIOA -> CRL &= ~(GPIO_CRL_MODE3 | GPIO_CRL_CNF3); // PA3 - аналоговый вход
	GPIOA -> CRL &= ~ (GPIO_CRL_MODE0 | GPIO_CRL_CNF0); // PA0 - аналоговый вход
	
	RCC -> APB2ENR |= RCC_APB2ENR_ADC1EN; // разрешение тактирование АЦП
	RCC -> CFGR &= ~RCC_CFGR_ADCPRE_0;  // предделитель АЦП = 10 (/6)
	RCC -> CFGR |=  RCC_CFGR_ADCPRE_1;
	
	ADC1 -> CR1 = 0;      // запрещаем все в управляющих регистрах
	ADC1 -> CR2 = 0;
	
	ADC1 -> SMPR2 &= ~(ADC_SMPR2_SMP0_0 | ADC_SMPR2_SMP0_1 | ADC_SMPR2_SMP0_2); // время выборки 1,5 цикла
	ADC1 -> SMPR2 &= ~(ADC_SMPR2_SMP3_0 | ADC_SMPR2_SMP3_1 | ADC_SMPR2_SMP3_2); // время выборки 1,5 цикла
	
	//Настройка инжектированных каналов
	ADC1 -> JSQR = 0; //Очистка
	ADC1 -> JSQR |= ADC_JSQR_JL_0; // 2 преобразования
	ADC1 -> JSQR |= ADC_JSQR_JSQ1_0; // 1е преобразование - канал 0
	ADC1 -> JSQR |= (ADC_JSQR_JSQ4_0 | ADC_JSQR_JSQ4_1); // 2е преобразование - канал 3*/
	
	//ADC1 -> JSQR = 3279937;
	
	ADC1->CR2 &= ~ADC_CR2_CONT; // запрет непрерывного режима
	ADC1->CR1 |= ADC_CR1_SCAN; //режим сканирования
	
	ADC1->CR2 |= ADC_CR2_JEXTSEL; // источник запуска - JSWSTART
	ADC1->CR2 |= ADC_CR2_JEXTTRIG; // разрешение внешнего запуска для инжектированных каналов
	
	// калибровка
	ADC1 -> CR2 |= ADC_CR2_ADON; // разрешить АЦП
	for (int i = 1; i < 10000; i++); // задержка
	ADC1->CR2 |= ADC_CR2_CAL; // запуск калибровки
	while (((ADC1 -> CR2) & ADC_CR2_CAL) != 0); // ожидание окончания калибровки
	//ADC1->CR2 &= ~ADC_CR2_ADON; // запретить АЦП
}
