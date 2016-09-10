pwm-profiler
============

Profile lighting fixtures by logging PWM signals. The code assumes a minimum PWM
frequency of around 550Hz, faster is fine but slower frequencies would have to
be accomodated by adjusting the input capture timer's prescaler value. Note that
the measurements have maximum prescision at the minimum frequency.

Everything relate to PWM input capture and host communication is DMA driven to
make implementing the control logic easier so it should be very easy to
customize.

Tested on a [Maple Mini](https://github.com/leaflabs/maplemini) using an
STM32F103CB.

Hardware
--------

So far I'm just using this on a breadboard.

### Pinout

| STM32 Pin | Maple Pin | Function                |
|-----------|-----------|------------------------- |
| PA8       |        27 | TIM1_CH1, PWM input     |
| PA0       |        11 | TIM2_CH1, PWM input     |
| PA6       |         5 | TIM3_CH1, PWM input     |
| PB6       |        16 | TIM4_CH1, PWM input     |
| PA11      |        26 | USART1_TX, debug output |
| PA3       |         8 | USART2_RX, host comm    |
| PA2       |         7 | USART2_TX, host comm    |
| PB10      |         1 | USART3_TX, DMX output   |

### DMA channel mapping

| DMA channel | Periph Function |
|-------------|----------------- |
| CH1         | TIM4_CH1        |
| CH2         | TIM1_CH1        |
| CH5         | TIM2_CH1        |
| CH6         | TIM3_CH1        |
| CH7         | USART2_TX       |
