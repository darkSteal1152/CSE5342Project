#include "stm32f4xx.h"

static void delay_ms(volatile int ms) {
    for (; ms > 0; ms--)
        for (volatile int i = 0; i < 16000; i++);
}

float voltage;
float tempC;

int main(void)
{
    /* === Enable clocks === */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;   // Enable GPIOA clock
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;    // Enable ADC1 clock

    /* === Set PA0 as analog === */
    GPIOA->MODER |= (3U << (0 * 2));  // 11 = analog
    GPIOA->PUPDR &= ~(3U << (0 * 2)); // No pull-up/down

    /* === Configure ADC1 === */
    ADC1->CR2 = 0;                     // Disable ADC for config
    ADC1->SQR3 = 0;                    // Channel 0 first conversion
    ADC1->SQR1 = 0;                    // Only 1 conversion
    ADC1->SMPR2 |= (3U << 0);          // Sampling time 56 cycles
    ADC1->CR2 |= ADC_CR2_ADON;         // Enable ADC

    while (1)
    {
        /* === Start conversion === */
        ADC1->CR2 |= ADC_CR2_SWSTART;

        /* === Wait for end of conversion === */
        while (!(ADC1->SR & ADC_SR_EOC));

        /* === Read result === */
        uint16_t adc_val = ADC1->DR; // 0–4095

        /* === Convert to temperature === */
        voltage = (adc_val * 3.3f) / 4095.0f;
        tempC = voltage * 100.0f;  // 10 mV per °C

        delay_ms(1000);
    }
}
