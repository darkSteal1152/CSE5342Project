/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Temperature Monitor with OLED and Web Server
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ESP8266_STM32.h"
#include "fonts.h"
#include "ssd1306.h"
#include <stdio.h>
#include <string.h>

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
char ip_buf[16];
volatile uint8_t showFahrenheit = 0; // 0 = °C, 1 = °F
float currentTemp = 0.0f;
#define NUM_SAMPLES 10
#define TRIG_PIN 5   // PA5
#define ECHO_PIN 6   // PA6

volatile uint8_t freezeMode = 0;
uint32_t freezeTimer = 0;

float frozenTemp = 0;      // stored baseline temp (always °C)
int tempOffset = 1;        // right-side temp difference

uint8_t prevPA1 = 0;
uint8_t prevPA4 = 0;

float setTemp = 0;         // final adjusted temperature
int firstPress = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_UART4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void ADC1_Init(void);
static uint16_t ADC1_Read(void);
static void delay_ms(volatile int ms);
static void UpdateDisplay(void);
static void HandleWebServer(void);

/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len) {
    HAL_UART_Transmit(&huart1, (uint8_t *)ptr, len, 100);
    return len;
}

uint16_t readAverageADC(void) {
    uint32_t sum = 0;
    for (int i = 0; i < NUM_SAMPLES; i++) {
        sum += ADC1_Read();
        delay_ms(1);
    }
    return (uint16_t)(sum / NUM_SAMPLES);
}

uint8_t button_read(void) {
    return ((GPIOC->IDR & (1 << 1)) == 0);   // pressed = LOW
}

uint8_t read_button_PA1(void) {
    return ((GPIOA->IDR & (1 << 1)) == 0);  // pressed = LOW
}

uint8_t read_button_PA4(void) {
    return ((GPIOA->IDR & (1 << 4)) == 0);  // pressed = LOW
}

uint8_t read_button_PB0(void) {
    return ((GPIOB->IDR & (1 << 0)) == 0);  // pressed = LOW
}

uint8_t prevPB0 = 0;
volatile uint32_t oledTimer = 0;
volatile uint8_t oled_off = 0;

void hc_sr04_trigger(void) {
    GPIOA->BSRR = (1 << TRIG_PIN);
    delay_us(10);
    GPIOA->BSRR = (1 << (TRIG_PIN + 16));
}

uint32_t hc_sr04_read(void) {
    uint32_t count = 0;

    // Wait for ECHO to go HIGH
    while ((GPIOA->IDR & (1 << ECHO_PIN)) == 0);

    // Measure HIGH time
    while ((GPIOA->IDR & (1 << ECHO_PIN)) != 0) {
        count++;
        delay_us(1);
    }

    return count / 58;   // convert to cm
}

void delay_us(volatile uint32_t us) {
    while(us--) {
        for(volatile int i = 0; i < 16; i++); // Tune if needed
    }
}

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    /* MCU Configuration--------------------------------------------------------*/
    HAL_Init();
    SystemClock_Config();

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_UART4_Init();
    MX_USART1_UART_Init();
    MX_I2C1_Init();

    /* USER CODE BEGIN 2 */
    // Initialize ADC
    ADC1_Init();

    // Initialize OLED
    SSD1306_Init();
    SSD1306_Fill(0);

    uint8_t prevBtn = 0;

    while (1)
    {
    	GPIOB->ODR ^= (1 << 6);   // XOR to flip the pin
    	//delay_ms(500);

        // ===== BUTTON TOGGLE C/F =====
        uint8_t btn = button_read();
        if (btn && !prevBtn) {
            showFahrenheit ^= 1;   // toggle unit
            delay_ms(50);         // debounce
            oledTimer = 0;            // reset timer if LED is on
            oled_off = 0;
        }
        prevBtn = btn;

        // ===== READ TEMPERATURE =====
        uint16_t adc_val = readAverageADC();
        float voltage = (adc_val * 3.3f) / 4095.0f;
        currentTemp = voltage * 100.0f; // LM35 outputs 10mV/°C

        // ===== DISTANCE READ =====
        hc_sr04_trigger();
        uint32_t dist_cm = hc_sr04_read();

        /* LED alert < 20 cm */
        if (dist_cm < 20)
        {
            GPIOA->BSRR = (1 << 7);  // LED ON
            oledTimer = 0;            // reset timer if LED is on
            oled_off = 0;
        }
        else
        {
            GPIOA->BSRR = (1 << (7 + 16));   // LED OFF

            if (oledTimer == 0)               // start timer only once
                oledTimer = HAL_GetTick();

            // Check if 2 seconds passed
            if (oledTimer > 0 && (HAL_GetTick() - oledTimer >= 2000))
            {
                oled_off = 1;
                oledTimer = 0; // stop timer until next LED OFF
            }
        }

        // ===== PA1 / PA4 BUTTON HANDLING =====
        uint8_t b = read_button_PA1();
        uint8_t c = read_button_PA4();

        if ((b && !prevPA1) || (c && !prevPA4)) // new press
        {
        	firstPress = 1;
            if (!freezeMode)
            {
                freezeMode = 1;
                frozenTemp = currentTemp;
                freezeTimer = HAL_GetTick();

                if (b && !prevPA1) tempOffset = 1;
                if (c && !prevPA4) tempOffset = -1;
            }
            else
            {
                if (b && !prevPA1) tempOffset++;
                if (c && !prevPA4) tempOffset--;
                freezeTimer = HAL_GetTick();  // reset timer
            }

            setTemp = frozenTemp + tempOffset;
        }
        prevPA1 = b;
        prevPA4 = c;

        // ===== FREEZE MODE DISPLAY =====
        if (freezeMode)
        {
            SSD1306_Fill(0);

            char left[16], right[16];

            float displayLeft  = showFahrenheit ? (frozenTemp * 9.f/5.f + 32.f) : frozenTemp;
            float displayRight = showFahrenheit ? ((frozenTemp + tempOffset) * 9.f/5.f + 32.f) : (frozenTemp + tempOffset);

            sprintf(left,  "%d%c", (int)displayLeft,  showFahrenheit ? 'F' : 'C');
            sprintf(right, "%d%c", (int)displayRight, showFahrenheit ? 'F' : 'C');

            SSD1306_GotoXY(0, 20);
            SSD1306_Puts(left, &Font_16x26, 1);

            SSD1306_GotoXY(70, 20);
            SSD1306_Puts(right, &Font_16x26, 1);

            SSD1306_UpdateScreen();

            if (HAL_GetTick() - freezeTimer >= 10000)
                freezeMode = 0;

            delay_ms(20);
            oledTimer = 0;            // reset timer if LED is on
			oled_off = 0;
            continue;
        }

        // ===== NORMAL OLED UPDATE =====
        UpdateDisplay();

        delay_ms(120);

        // Handle web server requests if needed
        // HandleWebServer();
    }
}

static void UpdateDisplay(void)
{
    float displayTemp = showFahrenheit ?
                        (currentTemp * 9.0f / 5.0f + 32.0f) :
                        currentTemp;

    SSD1306_Fill(0);

    // Convert to integer for clean output like "23 C"
    char line1[32];
    sprintf(line1, "%d %c", (int)displayTemp, (showFahrenheit ? 'F' : 'C'));

    SSD1306_GotoXY(30, 20);
    SSD1306_Puts(line1, &Font_16x26, 1);

    uint8_t pb0 = read_button_PB0();
    if (pb0 && !prevPB0) {  // rising edge detected
        setTemp = -99;       // deactivate Heat/Cool
        oledTimer = 0;            // reset timer if LED is on
        oled_off = 0;
    }

    prevPB0 = pb0;

    // ===== SET TEMP HEAT / COOL DISPLAY =====
    char line2[32];
    if (oled_off == 1) {
    	SSD1306_Fill(0);
    } else {
		if (setTemp != -99)
		{
			if (fabs(currentTemp - (frozenTemp + tempOffset)) < 0.7f)
			{
				setTemp = -99;
			}
			else if (currentTemp < (frozenTemp + tempOffset))
			{
				float displaySet = showFahrenheit ? ((frozenTemp + tempOffset) * 9.f/5.f + 32.f) : (frozenTemp + tempOffset);
				sprintf(line2, "Heat: %d %c", (int)displaySet, showFahrenheit ? 'F' : 'C');
				SSD1306_GotoXY(20, 50);
				SSD1306_Puts(line2, &Font_7x10, 1);
			}
			else if (currentTemp > (frozenTemp + tempOffset))
			{
				float displaySet = showFahrenheit ? ((frozenTemp + tempOffset) * 9.f/5.f + 32.f) : (frozenTemp + tempOffset);
				sprintf(line2, "Cool: %d %c", (int)displaySet, showFahrenheit ? 'F' : 'C');
				if ((int)displaySet == 1 && firstPress == 0) {

				} else {
					SSD1306_GotoXY(20, 50);
					SSD1306_Puts(line2, &Font_7x10, 1);
				}
			}
		}
    }

    SSD1306_UpdateScreen();
}

static void ADC1_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    /* PC1 = BUTTON INPUT (pull-up) */
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
	GPIOC->MODER &= ~(3 << (1*2));      // input
	GPIOC->PUPDR &= ~(3 << (1*2));
	GPIOC->PUPDR |=  (1 << (1*2));      // pull-up

    // PA0 analog mode (MODER bits = 11)
    GPIOA->MODER &= ~(3U << (0 * 2));      // clear bits first
    GPIOA->MODER |=  (3U << (0 * 2));      // set analog

    /* PA1 = BUTTON INPUT (pull-up) */
    GPIOA->MODER &= ~(3 << (1 * 2));     // input mode
    GPIOA->PUPDR &= ~(3 << (1 * 2));
    GPIOA->PUPDR |=  (1 << (1 * 2));     // pull-up

    /* PA4 = BUTTON INPUT (pull-up) */
    GPIOA->MODER &= ~(3 << (4 * 2));     // input mode
    GPIOA->PUPDR &= ~(3 << (4 * 2));
    GPIOA->PUPDR |=  (1 << (4 * 2));     // pull-up

    /* PB0 = BUTTON INPUT (pull-up) */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;   // enable port B clock
    GPIOB->MODER &= ~(3 << (0 * 2)); // input mode
    GPIOB->PUPDR &= ~(3 << (0 * 2));
    GPIOB->PUPDR |=  (1 << (0 * 2)); // pull-up

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;    // enable GPIOB clock
    GPIOB->MODER &= ~(3 << (6*2));          // clear mode bits
    GPIOB->MODER |=  (1 << (6*2));          // output mode
    GPIOB->OTYPER &= ~(1 << 6);             // push-pull
    GPIOB->OSPEEDR |= (3 << (6*2));         // high speed
    GPIOB->PUPDR &= ~(3 << (6*2));          // no pull

    /* PA5 = TRIG OUTPUT */
    GPIOA->MODER &= ~(3 << (TRIG_PIN*2));
    GPIOA->MODER |=  (1 << (TRIG_PIN*2));
    GPIOA->OTYPER &= ~(1 << TRIG_PIN);
    GPIOA->OSPEEDR |= (3 << (TRIG_PIN*2));
    GPIOA->PUPDR &= ~(3 << (TRIG_PIN*2));

    /* PA6 = ECHO INPUT */
    GPIOA->MODER &= ~(3 << (ECHO_PIN*2));
    GPIOA->PUPDR &= ~(3 << (ECHO_PIN*2));   // no pull

    /* PA7 = LED OUTPUT */
    GPIOA->MODER &= ~(3 << (7*2));
    GPIOA->MODER |=  (1 << (7*2));

    ADC1->CR2 = 0;
    ADC1->SQR3 = 0;
    ADC1->SQR1 = 0;
    ADC1->SMPR2 |= (3U << 0);
    ADC1->CR2 |= ADC_CR2_ADON;
}

static uint16_t ADC1_Read(void)
{
    ADC1->CR2 |= ADC_CR2_SWSTART;
    uint32_t t0 = HAL_GetTick();
    while (!(ADC1->SR & ADC_SR_EOC)) {
        if ((HAL_GetTick() - t0) > 10) return 0; // timeout 10ms
    }
    return ADC1->DR;
}


static void delay_ms(volatile int ms)
{
    for (; ms > 0; ms--)
        for (volatile int i = 0; i < 16000; i++);
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 180;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 2;
    RCC_OscInitStruct.PLL.PLLR = 2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_PWREx_EnableOverDrive() != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    {
        Error_Handler();
    }
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

    /* Configure I2C1 pins: PB8 (SCL), PB9 (SDA) */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    __HAL_RCC_I2C1_CLK_ENABLE();
}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{
    huart4.Instance = UART4;
    huart4.Init.BaudRate = 115200;
    huart4.Init.WordLength = UART_WORDLENGTH_8B;
    huart4.Init.StopBits = UART_STOPBITS_1;
    huart4.Init.Parity = UART_PARITY_NONE;
    huart4.Init.Mode = UART_MODE_TX_RX;
    huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart4.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart4) != HAL_OK)
    {
        Error_Handler();
    }

    /* Configure UART4 pins: PC10 (TX), PC11 (RX) */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_UART4_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
        Error_Handler();
    }

    /* Configure USART1 pins: PA9 (TX), PA10 (RX) */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

static void MX_GPIO_Init(void)
{
    /* Enable clocks */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
}

void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
    }
}
