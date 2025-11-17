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

uint8_t readButton(void) {
    return (GPIOA->IDR & (1 << 1)) == 0;
}
/* USER CODE END 0 */

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
    SSD1306_GotoXY(10, 20);
    SSD1306_Puts("Connecting", &Font_11x18, 1);
    SSD1306_GotoXY(10, 40);
    SSD1306_Puts("to WiFi...", &Font_11x18, 1);
    SSD1306_UpdateScreen();

    printf("\r\n========================================\r\n");
    printf("Temperature Monitor with Web Server\r\n");
    printf("========================================\r\n");

    // Initialize ESP8266
    if (ESP_Init() != ESP8266_OK){
        USER_LOG("Failed to initialize.. Check Debug Log");
        SSD1306_Fill(0);
        SSD1306_GotoXY(10, 25);
        SSD1306_Puts("Init Failed", &Font_11x18, 1);
        SSD1306_UpdateScreen();
        Error_Handler();
    }

    // Connect to WiFi
    if (ESP_ConnectWiFi("ATT6rWYCTA", "eais66v+nmeh", ip_buf, sizeof(ip_buf)) != ESP8266_OK){
        USER_LOG("Failed to connect to wifi.. Check Debug Log");
        SSD1306_Fill(0);
        SSD1306_GotoXY(10, 25);
        SSD1306_Puts("WiFi Failed", &Font_11x18, 1);
        SSD1306_UpdateScreen();
        Error_Handler();
    }

    printf("\r\n========================================\r\n");
    printf("Successfully Connected!\r\n");
    printf("IP Address: %s\r\n", ip_buf);
    printf("========================================\r\n");

    // Show IP on OLED briefly
    SSD1306_Fill(0);
    SSD1306_GotoXY(5, 10);
    SSD1306_Puts("Connected!", &Font_11x18, 1);
    SSD1306_GotoXY(5, 30);
    SSD1306_Puts(ip_buf, &Font_7x10, 1);
    SSD1306_UpdateScreen();
    HAL_Delay(3000);

    // Start Web Server
    if (ESP_StartWebServer(80) != ESP8266_OK){
        USER_LOG("Failed to start web server");
        Error_Handler();
    }

    printf("\r\nWeb Server Running!\r\n");
    printf("Open browser and go to: http://%s\r\n", ip_buf);
    printf("========================================\r\n");

    uint8_t prevButton = 0;
    uint32_t lastTempUpdate = 0;
    uint32_t lastButtonCheck = 0;

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        // Update temperature every 200ms
        if (HAL_GetTick() - lastTempUpdate > 200)
        {
            lastTempUpdate = HAL_GetTick();

            // Read temperature
            uint16_t adc_val = readAverageADC();
            float voltage = (adc_val * 3.3f) / 4095.0f;
            currentTemp = voltage * 100.0f; // LM35 outputs 10mV per degree C

            // Update OLED display
            UpdateDisplay();
        }

        // Check physical button every 50ms (debounced)
        if (HAL_GetTick() - lastButtonCheck > 50)
        {
            lastButtonCheck = HAL_GetTick();

            uint8_t currentButton = readButton();
            if (currentButton && !prevButton) {
                showFahrenheit = !showFahrenheit;
                USER_LOG("Button pressed - Unit: %c", showFahrenheit ? 'F' : 'C');
                HAL_Delay(200); // Extra debounce delay
            }
            prevButton = currentButton;
        }

        // Handle web server requests
        HandleWebServer();

        /* USER CODE END WHILE */
    }
    /* USER CODE END 3 */
}

static void UpdateDisplay(void)
{
    float displayTemp = showFahrenheit ? (currentTemp * 9.0f / 5.0f + 32.0f) : currentTemp;

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
}

static void HandleWebServer(void)
{
    char request[512];

    // Check for incoming HTTP request
    ESP8266_Status status = ESP_CheckForClient(request, sizeof(request));

    if (status == ESP8266_OK)
    {
        USER_LOG("=== HTTP Request Received ===");

        // Check if toggle button was pressed
        if (strstr(request, "GET /toggle"))
        {
            showFahrenheit = !showFahrenheit;
            USER_LOG("Unit toggled to: %c", showFahrenheit ? 'F' : 'C');
        }

        // Prepare HTML response
        float displayTemp = showFahrenheit ? (currentTemp * 9.0f / 5.0f + 32.0f) : currentTemp;
        char unit = showFahrenheit ? 'F' : 'C';
        int temp_int = (int)displayTemp; // Convert to integer

        USER_LOG("Temp: %d%c, Building response...", temp_int, unit);

        char html[1536];
        snprintf(html, sizeof(html),
                    "HTTP/1.1 200 OK\r\n"
                    "Content-Type: text/html\r\n"
                    "Connection: close\r\n"
                    "\r\n"
                    "<!DOCTYPE html>"
                    "<html><head>"
                    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
                    "<meta http-equiv='refresh' content='2'>"
                    "<title>Temp Monitor</title>"
                    "<style>"
                    "body{font-family:Arial;text-align:center;margin:50px;background:#f0f0f0}"
                    ".box{background:white;padding:40px;border-radius:20px;box-shadow:0 4px 6px rgba(0,0,0,0.1);max-width:400px;margin:auto}"
                    ".temp{font-size:72px;font-weight:bold;color:#ff6b6b;margin:20px 0}"
                    ".unit{font-size:36px;color:#666}"
                    "button{background:#4CAF50;color:white;padding:15px 32px;font-size:18px;border:none;border-radius:8px;cursor:pointer;margin:20px}"
                    "button:hover{background:#45a049}"
                    ".info{color:#666;font-size:14px;margin-top:20px}"
                    "</style>"
                    "</head><body>"
                    "<div class='box'>"
                    "<h1>Temperature</h1>"
                    "<div class='temp'>%d<span class='unit'>%c</span></div>"
                    "<button onclick='location.href=\"/toggle\"'>Toggle Unit</button>"
                    "<div class='info'>Auto-refresh: 2s<br>IP: %s</div>"
                    "</div>"
                    "</body></html>",
                    temp_int, unit, ip_buf
                );

        USER_LOG("Sending response (%d bytes)", strlen(html));

        // Send response
        if (ESP_SendWebPage(html) == ESP8266_OK)
        {
            USER_LOG("Response sent successfully");
        }
        else
        {
            USER_LOG("Failed to send response");
        }

        ESP_CloseConnection();
        USER_LOG("=== Request Complete ===\r\n");
    }
}

static void ADC1_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    // PA0 analog mode
    GPIOA->MODER |= (3U << (0 * 2));
    GPIOA->PUPDR &= ~(3U << (0 * 2));

    // PA1 as input (button)
    GPIOA->MODER &= ~(3U << (1 * 2));
    GPIOA->PUPDR &= ~(3U << (1 * 2));
    GPIOA->PUPDR |= (1U << (1 * 2)); // Pull-up

    ADC1->CR2 = 0;
    ADC1->SQR3 = 0;
    ADC1->SQR1 = 0;
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

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif
