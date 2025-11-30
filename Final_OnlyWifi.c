/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Non-Blocking Temperature Monitor with OLED + Web Server
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include <string.h>

#include "ssd1306.h"
#include "fonts.h"
#include "ESP8266_STM32.h"

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
volatile float currentTemp = 0.0f;
volatile uint8_t showFahrenheit = 0;

char ip_buf[16];

#define TEMP_INTERVAL_MS     200
#define OLED_INTERVAL_MS     200
#define WEB_INTERVAL_MS      20

uint32_t lastTempTime = 0;
uint32_t lastOLEDTime = 0;
uint32_t lastWebCheck = 0;

/* USER CODE END PV */

/* Function Prototypes */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_UART4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void ADC1_Init(void);
static uint8_t ADC1_Ready(void);
static void ADC1_Start(void);
static uint16_t ADC1_GetValue(void);

static void Task_ReadTemperature(void);
static void Task_UpdateOLED(void);
static void Task_WebServer(void);


/* USER CODE BEGIN 0 */
int _write(int f, char *p, int len)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)p, len, 5);
    return len;
}
/* USER CODE END 0 */

/* MAIN PROGRAM ****************************************************************/
int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_UART4_Init();
    MX_USART1_UART_Init();
    MX_I2C1_Init();

    /* Init Analog + OLED */
    ADC1_Init();
    SSD1306_Init();

    SSD1306_Fill(0);
    SSD1306_GotoXY(10,20);
    SSD1306_Puts("Connecting..", &Font_11x18, 1);
    SSD1306_UpdateScreen();

    /* ESP INIT ----------------------------------------------------------- */
    if (ESP_Init() != ESP8266_OK)
    {
        SSD1306_Fill(0);
        SSD1306_Puts("ESP Fail", &Font_11x18, 1);
        SSD1306_UpdateScreen();
    }

    if (ESP_ConnectWiFi("SSD", "PASS", ip_buf, sizeof(ip_buf)) != ESP8266_OK)
    {
        SSD1306_Fill(0);
        SSD1306_Puts("WiFi Fail", &Font_11x18, 1);
        SSD1306_UpdateScreen();
    }

    ESP_StartWebServer(80);

    SSD1306_Fill(0);
    SSD1306_Puts("IP:", &Font_11x18, 1);
    SSD1306_GotoXY(0,30);
    SSD1306_Puts(ip_buf, &Font_7x10, 1);
    SSD1306_UpdateScreen();

    uint32_t ipScreenStart = HAL_GetTick();

    while (HAL_GetTick() - ipScreenStart <= 10000) {

    }

    /* MAIN LOOP ----------------------------------------------------------- */
    while (1)
    {
        uint32_t now = HAL_GetTick();

        if (now - lastTempTime >= TEMP_INTERVAL_MS)
        {
            lastTempTime = now;
            Task_ReadTemperature();
        }

        if (now - lastOLEDTime >= OLED_INTERVAL_MS)
        {
            lastOLEDTime = now;
            Task_UpdateOLED();
        }

        if (now - lastWebCheck >= WEB_INTERVAL_MS)
        {
            lastWebCheck = now;
            Task_WebServer();
        }
    }
}


/* ============================================================
 *  NON-BLOCKING TASK: TEMPERATURE READ
 * ============================================================ */
static void Task_ReadTemperature(void)
{
    static uint8_t adcRunning = 0;

    if (!adcRunning)
    {
        ADC1_Start();
        adcRunning = 1;
        return;
    }

    if (ADC1_Ready())
    {
        uint16_t val = ADC1_GetValue();
        float voltage = (val * 3.3f) / 4095.0f;
        currentTemp = voltage * 100.0f;
        adcRunning = 0;
    }
}


/* ============================================================
 *  NON-BLOCKING TASK: OLED UPDATE
 * ============================================================ */
static void Task_UpdateOLED(void)
{
    SSD1306_Fill(0);

    float t = currentTemp;
    if (showFahrenheit) t = t * 9/5 + 32;

    char buf[32];
    sprintf(buf, "%d %c", (int)t, showFahrenheit ? 'F':'C');

    SSD1306_GotoXY(20,20);
    SSD1306_Puts(buf, &Font_16x26, 1);

    SSD1306_UpdateScreen();
}


/* ============================================================
 *  NON-BLOCKING TASK: WEB SERVER
 * ============================================================ */
static void Task_WebServer(void)
{
    char req[400];
    if (ESP_CheckForClient(req,sizeof(req)) != ESP8266_OK)
        return;  // Nothing, return immediately (NON BLOCKING)

    if (strstr(req,"/toggle"))
        showFahrenheit ^= 1;

    float t = showFahrenheit ? currentTemp*9/5+32 : currentTemp;

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
						(int)t, (showFahrenheit?'F':'C'), ip_buf);

    ESP_SendWebPage(html);
    ESP_CloseConnection();
}


/* ============================================================
 *  ADC (Non-blocking)
 * ============================================================ */
static void ADC1_Init(void)
{
    __HAL_RCC_ADC1_CLK_ENABLE();
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    GPIOA->MODER |= (3 << (0*2));

    ADC1->CR2 = 0;
    ADC1->SQR3 = 0;   // channel 0
    ADC1->SMPR2 |= (3 << 0);
    ADC1->CR2 |= ADC_CR2_ADON;
}

static void ADC1_Start(void)
{
    ADC1->CR2 |= ADC_CR2_SWSTART;
}

static uint8_t ADC1_Ready(void)
{
    return (ADC1->SR & ADC_SR_EOC);
}

static uint16_t ADC1_GetValue(void)
{
    return ADC1->DR;
}

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
