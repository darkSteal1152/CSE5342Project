#include "main.h"
#include "fonts.h"
#include "ssd1306.h"
#include "stm32f4xx.h"
#include <stdio.h>

I2C_HandleTypeDef hi2c1;

void SystemClock_Config(void);
static void MX_I2C1_Init(void);

/* =================== USER DEFINES ====================== */
#define TRIG_PIN 5
#define ECHO_PIN 6

#define NUM_SAMPLES 10

volatile uint8_t freezeMode = 0;
uint32_t freezeTimer = 0;
float frozenTemp = 0;

int tempOffset = 1;

/* =================== DELAY FUNCTIONS =================== */
void delay_us(volatile uint32_t us) {
    while(us--) {
        for(volatile int i = 0; i < 16; i++); // Tune if needed
    }
}

static void delay_ms(volatile int ms) {
    for (; ms > 0; ms--)
        for (volatile int i = 0; i < 16000; i++);
}

/* =================== GPIO INIT FOR HC-SR04 ============= */
void gpio_init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    /* PC1 = BUTTON INPUT (pull-up) */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    GPIOC->MODER &= ~(3 << (1*2));      // input
    GPIOC->PUPDR &= ~(3 << (1*2));
    GPIOC->PUPDR |=  (1 << (1*2));      // pull-up

    /* PA1 = BUTTON INPUT (pull-up) */
    GPIOA->MODER &= ~(3 << (1 * 2));     // input mode
    GPIOA->PUPDR &= ~(3 << (1 * 2));
    GPIOA->PUPDR |=  (1 << (1 * 2));     // pull-up

    /* PA4 = BUTTON INPUT (pull-up) */
    GPIOA->MODER &= ~(3 << (4 * 2));     // input mode
    GPIOA->PUPDR &= ~(3 << (4 * 2));
    GPIOA->PUPDR |=  (1 << (4 * 2));     // pull-up

    /* PA5 = TRIG OUTPUT */
    GPIOA->MODER &= ~(3 << (TRIG_PIN*2));
    GPIOA->MODER |=  (1 << (TRIG_PIN*2));
    GPIOA->OTYPER &= ~(1 << TRIG_PIN);
    GPIOA->OSPEEDR |= (3 << (TRIG_PIN*2));
    GPIOA->PUPDR &= ~(3 << (TRIG_PIN*2));

    /* PA6 = ECHO INPUT */
    GPIOA->MODER &= ~(3 << (ECHO_PIN*2));
    GPIOA->PUPDR &= ~(3 << (ECHO_PIN*2)); // no pull

    /* PA7 = LED OUTPUT */
    GPIOA->MODER &= ~(3 << (7*2));
    GPIOA->MODER |=  (1 << (7*2));   // output mode
    GPIOA->OTYPER &= ~(1 << 7);
    GPIOA->OSPEEDR |= (2 << (7*2));
    GPIOA->PUPDR &= ~(3 << (7*2));
}

uint8_t button_read(void) {
    return ((GPIOC->IDR & (1 << 1)) == 0);   // pressed = LOW
}

uint8_t read_button_PA1(void) {
    return ((GPIOA->IDR & (1 << 1)) == 0);  // pressed = LOW
}

uint8_t read_button_PA4(void) {
    return ((GPIOA->IDR & (1 << 4)) == 0);   // PA4 low = pressed
}

volatile uint8_t showFahrenheit = 0;

/* =================== HC-SR04 ROUTINES ================== */

void hc_sr04_trigger(void) {
    GPIOA->BSRR = (1 << TRIG_PIN);
    delay_us(10);
    GPIOA->BSRR = (1 << (TRIG_PIN + 16));
}

uint32_t hc_sr04_read(void) {
    uint32_t count = 0;

    while((GPIOA->IDR & (1 << ECHO_PIN)) == 0);
    while((GPIOA->IDR & (1 << ECHO_PIN)) != 0) {
        count++;
        delay_us(1);
    }

    return count / 58; // cm
}

/* =================== ADC FUNCTIONS ===================== */
static void ADC1_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    GPIOA->MODER |= (3U << (0 * 2));

    ADC1->CR2 = 0;
    ADC1->SQR3 = 0;
    ADC1->SQR1 = 0;
    ADC1->SMPR2 |= (3U << 0);
    ADC1->CR2 |= ADC_CR2_ADON;
}

static uint16_t ADC1_Read(void) {
    ADC1->CR2 |= ADC_CR2_SWSTART;
    while(!(ADC1->SR & ADC_SR_EOC));
    return ADC1->DR;
}

uint16_t readAverageADC(void) {
    uint32_t sum = 0;
    for(int i=0;i<NUM_SAMPLES;i++) {
        sum += ADC1_Read();
        delay_ms(1);
    }
    return sum / NUM_SAMPLES;
}

/* ======================= OLED + MAIN ======================= */

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_I2C1_Init();
    ADC1_Init();
    gpio_init();

    SSD1306_Init();
    SSD1306_Fill(0);
    SSD1306_UpdateScreen();

    uint8_t prevBtn = 0;

    uint8_t prevPA1 = 0;
    uint8_t prevPA4 = 0;

    float setTemp = -99;

    while (1)
    {
        uint8_t b = read_button_PA1();
        uint8_t c = read_button_PA4();

        /* ===== BUTTON TOGGLE C/F ===== */
        uint8_t btn = button_read();
        if (btn && !prevBtn) {     // rising edge
            showFahrenheit ^= 1;   // toggle
            delay_ms(150);         // debounce
        }
        prevBtn = btn;

        /* ===== TEMPERATURE READ (always in C) ===== */
        uint16_t adc_val = readAverageADC();
        float voltage = adc_val * 3.3f / 4095.f;
        float tempC = voltage * 100.0f;   // internal value in Celsius

        /* ===== DISTANCE READ ===== */
        hc_sr04_trigger();
        uint32_t dist_cm = hc_sr04_read();

        /* LED alert < 10 cm */
        if(dist_cm < 10)
            GPIOA->BSRR = (1 << 7);
        else
            GPIOA->BSRR = (1 << (7+16));

        /* ===== PA1 / PA4 BUTTON HANDLING ===== */
        if ((b && !prevPA1) || (c && !prevPA4))   // new press
        {
            if (!freezeMode)
            {
                // Enter freeze mode
                freezeMode = 1;
                frozenTemp = tempC;        // always store in °C
                freezeTimer = HAL_GetTick();

                // Initialize tempOffset
                if (b && !prevPA1) tempOffset = 1;
                if (c && !prevPA4) tempOffset = -1;
            }
            else
            {
                // Already in freeze mode → adjust right side
                if (b && !prevPA1) tempOffset++;   // +1°C
                if (c && !prevPA4) tempOffset--;   // -1°C
                freezeTimer = HAL_GetTick();       // reset timer
            }

            setTemp = frozenTemp + tempOffset;
        }
        prevPA1 = b;
        prevPA4 = c;

        /* ===== DISPLAY ===== */
        if (freezeMode)
        {
            SSD1306_Fill(0);

            char left[16], right[16];

            // convert to display units
            float displayLeft = showFahrenheit ? (frozenTemp * 9.f/5.f + 32.f) : frozenTemp;
            float displayRight = showFahrenheit ? ((frozenTemp + tempOffset) * 9.f/5.f + 32.f) : (frozenTemp + tempOffset);

            sprintf(left, "%d%c", (int)displayLeft, showFahrenheit ? 'F':'C');
            sprintf(right, "%d%c", (int)displayRight, showFahrenheit ? 'F':'C');

            SSD1306_GotoXY(0, 20);
            SSD1306_Puts(left, &Font_16x26, 1);

            SSD1306_GotoXY(70, 20);
            SSD1306_Puts(right, &Font_16x26, 1);

            SSD1306_UpdateScreen();

            // Auto exit after 10 seconds
            if (HAL_GetTick() - freezeTimer >= 10000) freezeMode = 0;

            delay_ms(120);
            continue;
        }

        /* ===== NORMAL REAL-TIME MODE ===== */
        SSD1306_Fill(0);

        float tempDisplay = showFahrenheit ? (tempC * 9.f/5.f + 32.f) : tempC;
        char line1[32], line2[32];
        sprintf(line1, "%d %c", (int)tempDisplay, showFahrenheit ? 'F':'C');

        SSD1306_GotoXY(30, 20);
        SSD1306_Puts(line1, &Font_16x26, 1);

        // Compare in °C internally
        if (setTemp != -99)
        {
            if (fabs(tempC - (frozenTemp + tempOffset)) < 0.7f)
            {
                setTemp = -99;
            }
            else if (tempC < (frozenTemp + tempOffset))
            {
                float displaySet = showFahrenheit ? ((frozenTemp + tempOffset) * 9.f/5.f + 32.f) : (frozenTemp + tempOffset);
                sprintf(line2, "Heat: %d %c", (int)displaySet, showFahrenheit ? 'F':'C');
                SSD1306_GotoXY(20, 50);
                SSD1306_Puts(line2, &Font_7x10, 1);
            }
            else if (tempC > (frozenTemp + tempOffset))
            {
                float displaySet = showFahrenheit ? ((frozenTemp + tempOffset) * 9.f/5.f + 32.f) : (frozenTemp + tempOffset);
                sprintf(line2, "Cool: %d %c", (int)displaySet, showFahrenheit ? 'F':'C');
                SSD1306_GotoXY(20, 50);
                SSD1306_Puts(line2, &Font_7x10, 1);
            }
        }

        SSD1306_UpdateScreen();
        delay_ms(120);
    }

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

void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}
