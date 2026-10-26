import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


class Version8ConfigTests(unittest.TestCase):
    def test_8mhz_clock_still_generates_168mhz(self):
        conf = (ROOT/'Inc/stm32f4xx_hal_conf.h').read_text(encoding='utf-8')
        main = (ROOT/'Src/main.c').read_text(encoding='utf-8')
        self.assertIn('#define HSE_VALUE     8000000U', conf)
        self.assertIn('RCC_OscInitStruct.PLL.PLLM = 4;', main)
        self.assertIn('RCC_OscInitStruct.PLL.PLLN = 168;', main)
        self.assertIn('RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;', main)

    def test_ultrasonic_is_pa0_pa1(self):
        source = (ROOT/'Src/hc_sr04.c').read_text(encoding='utf-8')
        self.assertIn('#define HC_SR04_TRIG_PIN        GPIO_PIN_0', source)
        self.assertIn('#define HC_SR04_ECHO_PIN        GPIO_PIN_1', source)

    def test_vision_is_usart6_pc6_pc7_and_console_is_usart1(self):
        main = (ROOT/'Src/main.c').read_text(encoding='utf-8')
        msp = (ROOT/'Src/stm32f4xx_hal_msp.c').read_text(encoding='utf-8')
        irq = (ROOT/'Src/stm32f4xx_it.c').read_text(encoding='utf-8')
        self.assertIn('Vision_Init(&huart6);', main)
        self.assertIn('HAL_UART_Receive(&huart1, &command', main)
        self.assertIn('GPIO_PIN_6 | GPIO_PIN_7', msp)
        self.assertIn('GPIO_AF8_USART6', msp)
        self.assertIn('HAL_NVIC_EnableIRQ(USART6_IRQn);', msp)
        self.assertIn('void USART6_IRQHandler(void)', irq)
        self.assertIn('GPIO_PIN_9 | GPIO_PIN_10', msp)
        self.assertIn('GPIO_AF7_USART1', msp)


if __name__ == '__main__':
    unittest.main()
