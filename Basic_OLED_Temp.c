#include "main.h"
#include "fonts.h"
#include "ssd1306.h"
#include "stm32f4xx.h"
#include <stdio.h>

I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart2;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);

static void ADC1_Init(void);
static uint16_t ADC1_Read(void);
static void delay_ms(volatile int ms);

float voltage;
float tempC;

#define NUM_SAMPLES 10

volatile uint8_t showFahrenheit = 0; // 0 = °C, 1 = °F

uint16_t readAverageADC(void) {
    uint32_t sum = 0;
    for (int i = 0; i < NUM_SAMPLES; i++) {
        sum += ADC1_Read();
        delay_ms(1);
    }
    return (uint16_t)(sum / NUM_SAMPLES);
}

uint8_t readButton(void) {
    // Return 1 if pressed
    return (GPIOA->IDR & (1 << 1)) == 0;
}

int main(void)
{
    /* === Enable clocks === */
	HAL_Init();
	SystemClock_Config();
	MX_GPIO_Init();
	MX_USART2_UART_Init();
	MX_I2C1_Init();
	SSD1306_Init();
	ADC1_Init();

	char buf[16];

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    /* === Set PA0 as analog input (temp sensor) === */
    GPIOA->MODER |= (3U << (0 * 2));
    GPIOA->PUPDR &= ~(3U << (0 * 2));

    /* === Set PA1 as input (button) === */
    GPIOA->MODER &= ~(3U << (1 * 2));         // Input mode
    GPIOA->PUPDR &= ~(3U << (1 * 2));
    GPIOA->PUPDR |= (1U << (1 * 2));          // Pull-up

    /* === Configure ADC1 === */
    ADC1->CR2 = 0;
    ADC1->SQR3 = 0;
    ADC1->SQR1 = 0;
    ADC1->SMPR2 |= (3U << 0);
    ADC1->CR2 |= ADC_CR2_ADON;

    // === Initialize OLED ===
    SSD1306_Init();
    SSD1306_Fill(0);
    SSD1306_UpdateScreen();

    uint8_t prevButton = 0;

    while (1)
    {
    	// --- BUTTON TOGGLE ---
		uint8_t currentButton = readButton();
		if (currentButton && !prevButton) { // detect rising edge
			showFahrenheit = !showFahrenheit;
			delay_ms(100); // debounce
		}
		prevButton = currentButton;

		// --- ADC Read ---
		uint16_t adc_val = readAverageADC();
		float voltage = (adc_val * 3.3f) / 4095.0f;
		float tempC = voltage * 100.0f;
		float displayTemp = showFahrenheit ? (tempC * 9.0f / 5.0f + 32.0f) : tempC;

		// --- Display ---
		char snum[8];
		int num = (int)displayTemp;
		itoa(num, snum, 10);

		SSD1306_Fill(0);
		SSD1306_GotoXY(30, 20);
		SSD1306_Puts(snum, &Font_16x26, 1);

		int x_deg = 30 + (strlen(snum) * 16);
		SSD1306_DrawCircle(x_deg + 4, 24, 3, 1);

		SSD1306_GotoXY(x_deg + 12, 20);
		if (showFahrenheit) {
			SSD1306_Puts("F", &Font_16x26, 1);
		} else {
			SSD1306_Puts("C", &Font_16x26, 1);
		}

		SSD1306_DrawLine(1, 54, 126, 54, 1);
		SSD1306_UpdateScreen();

		delay_ms(200);
    }

}


static void ADC1_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;   // Enable GPIOA clock
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;    // Enable ADC1 clock

    // PA0 analog mode
    GPIOA->MODER |= (3U << (0 * 2));
    GPIOA->PUPDR &= ~(3U << (0 * 2));

    ADC1->CR2 = 0;
    ADC1->SQR3 = 0;          // Channel 0
    ADC1->SQR1 = 0;          // 1 conversion
    ADC1->SMPR2 |= (3U << 0);
    ADC1->CR2 |= ADC_CR2_ADON;
}

static uint16_t ADC1_Read(void)
{
    ADC1->CR2 |= ADC_CR2_SWSTART;
    while (!(ADC1->SR & ADC_SR_EOC));
    return ADC1->DR;
}

static void delay_ms(volatile int ms)
{
    for (; ms > 0; ms--)
        for (volatile int i = 0; i < 16000; i++);
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 16;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
    RCC_OscInitStruct.PLL.PLLQ = 2;
    RCC_OscInitStruct.PLL.PLLR = 2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) Error_Handler();
}

static void MX_I2C1_Init(void)
{
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 400000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) Error_Handler();
}

static void MX_USART2_UART_Init(void)
{
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK) Error_Handler();
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin = B1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LD2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}
