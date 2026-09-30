#include "lin_phy.h"

#include "gpio.h"
#include "usart.h"

extern UART_HandleTypeDef huart3;

static void C8_LinPhy_RestoreUartPins(void)
{
    GPIO_InitTypeDef gpio = {0};

    /* STM32F103 的 USART RX 没有独立 AF input mux：PB11 的正确
     * USART3_RX 电气配置就是浮空输入，不得改为 EXTI/Analog。 */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    gpio.Pin = GPIO_PIN_10;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gpio);

    gpio.Pin = GPIO_PIN_11;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &gpio);
}

void C8_LinPhy_Init(void)
{
    HAL_GPIO_WritePin(LIN_SLP_C8_GPIO_Port, LIN_SLP_C8_Pin, GPIO_PIN_SET);
    C8_LinPhy_RestoreUartPins();
}

void C8_LinPhy_SetNormal(void)
{
    HAL_GPIO_WritePin(LIN_SLP_C8_GPIO_Port, LIN_SLP_C8_Pin, GPIO_PIN_SET);
    C8_LinPhy_RestoreUartPins();
}

void C8_LinPhy_SetSleep(void)
{
    HAL_GPIO_WritePin(LIN_SLP_C8_GPIO_Port, LIN_SLP_C8_Pin, GPIO_PIN_RESET);
}

bool C8_LinPhy_SendWakePulse(void)
{
    uint32_t start = HAL_GetTick();

    /* LIN mode SBK generates a 13-bit dominant break. At 19200 bit/s this is
     * about 677 us, inside the LIN 2.2A 250 us..5 ms wake-up window. */
    huart3.Instance->CR1 &= ~USART_CR1_RE;
    if (HAL_LIN_SendBreak(&huart3) != HAL_OK) {
        huart3.Instance->CR1 |= USART_CR1_RE;
        return false;
    }
    while ((huart3.Instance->CR1 & USART_CR1_SBK) != 0U) {
        if ((uint32_t)(HAL_GetTick() - start) >= 2U) {
            huart3.Instance->CR1 |= USART_CR1_RE;
            return false;
        }
    }
    while ((huart3.Instance->SR & USART_SR_TC) == 0U) {
        if ((uint32_t)(HAL_GetTick() - start) >= 2U) {
            huart3.Instance->CR1 |= USART_CR1_RE;
            return false;
        }
    }
    huart3.Instance->CR1 |= USART_CR1_RE;
    return true;
}

bool C8_LinPhy_IsNormal(void)
{
    return HAL_GPIO_ReadPin(LIN_SLP_C8_GPIO_Port, LIN_SLP_C8_Pin) ==
           GPIO_PIN_SET;
}
