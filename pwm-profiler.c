/*
 * pwm-profiler: capture profile of lighting fixtures at PWM level
 * Copyright (C) 2016  Daniel Gröber <dxld@darkboxed.org>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <errno.h>
#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <inttypes.h>
#include <assert.h>
#include <string.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/cortex.h>
#include <libopencm3/cm3/systick.h>

#include "list.h"
#include "comm.h"

#define asizeof(xs) (sizeof(xs) / sizeof(*(xs)))

typedef uint16_t dma_buffer_t[2 * dma_chunk_size];

volatile uint32_t systick;
volatile dma_buffer_t __attribute__ ((aligned (4))) dma_buffers[4] = { {0} };
struct dma_header __attribute__ ((aligned (4))) dma_headers[4] = { {0} };

dma_buffer_t __attribute__ ((aligned (4))) dma_escape = { 0x4040 };

volatile uint32_t usart2_dma_in_progress = 0;
volatile uint32_t usart2_dma_bitmap = 0;

struct dma_transfer {
        struct list_head freelist;
        struct list_head queue;

        uint8_t   channel;
        uintptr_t mem_addr;
        uint16_t  count;
};

struct dma_transfer dma_queue_storage[32];

LIST_HEAD(dma_queue);
LIST_HEAD(dma_freelist);

#define init_static_list(storage, list_field, head) do {                \
                for(unsigned int i=0; i < asizeof(storage); i++) {      \
                        INIT_LIST_HEAD(&storage[i].list_field);         \
                        list_add(&storage[i].list_field, &head);        \
                }                                                       \
        } while(0)

struct dma_channel_periph {
        enum rcc_periph_clken rcc;
        uint32_t dma;
        uint8_t channel;
        uint16_t irq_mask;
        int irq;
};

struct timer_periph {
        int id;
        enum rcc_periph_clken rcc;
        uint32_t tim;

        const struct dma_channel_periph *dmap;

        uint16_t irq_mask;
        int irqs[3];

        uint32_t gpio;
        uint16_t gpio_pin;
};

struct usart_periph {
        int id;
        enum rcc_periph_clken rcc;
        uint32_t usart;
        uint32_t gpio;
        uint16_t gpio_rx_pin, gpio_tx_pin;

        uint16_t irq_mask;
        int irq;

        const struct dma_channel_periph *dmap;
};

const struct dma_channel_periph DMA1_CHANNEL1p = {
        .rcc = RCC_DMA1,
        .dma = DMA1,
        .channel = DMA_CHANNEL1,
        .irq_mask = DMA_CCR_TEIE | DMA_CCR_HTIE | DMA_CCR_TCIE,
        .irq = NVIC_DMA1_CHANNEL1_IRQ
};

const struct dma_channel_periph DMA1_CHANNEL2p = {
        .rcc = RCC_DMA1,
        .dma = DMA1,
        .channel = DMA_CHANNEL2,
        .irq_mask = DMA_CCR_TEIE | DMA_CCR_HTIE | DMA_CCR_TCIE,
        .irq = NVIC_DMA1_CHANNEL2_IRQ
};

const struct dma_channel_periph DMA1_CHANNEL5p = {
        .rcc = RCC_DMA1,
        .dma = DMA1,
        .channel = DMA_CHANNEL5,
        .irq_mask = DMA_CCR_TEIE | DMA_CCR_HTIE | DMA_CCR_TCIE,
        .irq = NVIC_DMA1_CHANNEL5_IRQ
};

const struct dma_channel_periph DMA1_CHANNEL6p = {
        .rcc = RCC_DMA1,
        .dma = DMA1,
        .channel = DMA_CHANNEL6,
        .irq_mask = DMA_CCR_TEIE | DMA_CCR_HTIE | DMA_CCR_TCIE,
        .irq = NVIC_DMA1_CHANNEL6_IRQ
};

const struct dma_channel_periph DMA1_CHANNEL7p = {
        .rcc = RCC_DMA1,
        .dma = DMA1,
        .channel = DMA_CHANNEL7,
        .irq_mask = DMA_CCR_TEIE | DMA_CCR_HTIE | DMA_CCR_TCIE,
        .irq = NVIC_DMA1_CHANNEL7_IRQ
};

const struct timer_periph TIM1p = {
        .id = 0,

        .rcc = RCC_TIM1,
        .tim = TIM1,

        .dmap = &DMA1_CHANNEL2p,

        .irq_mask = TIM_DIER_UIE /* | TIM_DIER_CC1IE */ | TIM_DIER_CC2IE
                  | TIM_DIER_TIE
                  | TIM_DIER_CC1DE,

        .irqs = { NVIC_TIM1_UP_IRQ, NVIC_TIM1_CC_IRQ, -1 },

        .gpio = GPIOA,
        .gpio_pin = GPIO8,
};

const struct timer_periph TIM2p = {
        .id = 1,

        .rcc = RCC_TIM2,
        .tim = TIM2,

        .dmap = &DMA1_CHANNEL5p,

        .irq_mask = TIM_DIER_UIE /* | TIM_DIER_CC1IE */ | TIM_DIER_CC2IE
                  | TIM_DIER_TIE
                  | TIM_DIER_CC1DE,

        .irqs = { NVIC_TIM2_IRQ, -1 },

        .gpio = GPIOA,
        .gpio_pin = GPIO0,
};

const struct timer_periph TIM3p = {
        .id = 2,

        .rcc = RCC_TIM3,
        .tim = TIM3,

        .dmap = &DMA1_CHANNEL6p,

        .irq_mask = TIM_DIER_UIE /* | TIM_DIER_CC1IE */ | TIM_DIER_CC2IE
                  | TIM_DIER_TIE
                  | TIM_DIER_CC1DE,

        .irqs = { NVIC_TIM3_IRQ, -1 },

        .gpio = GPIOA,
        .gpio_pin = GPIO6,
};

const struct timer_periph TIM4p = {
        .id = 3,

        .rcc = RCC_TIM4,
        .tim = TIM4,
        .dmap = &DMA1_CHANNEL1p,

        .irq_mask = TIM_DIER_UIE /* | TIM_DIER_CC1IE */ | TIM_DIER_CC2IE
                  | TIM_DIER_TIE
                  | TIM_DIER_CC1DE,

        .irqs = { NVIC_TIM4_IRQ, -1 },

        .gpio = GPIOB,
        .gpio_pin = GPIO6,
};

/* PRINTF USART */
const struct usart_periph USART1p = {
        .id = 0,
        .rcc = RCC_USART1,
        .usart = USART1,
        .gpio = GPIOA,
        .gpio_rx_pin = GPIO10,
        .gpio_tx_pin = GPIO9,
};

/* COMM USART */
const struct usart_periph USART2p = {
        .id = 1,
        .rcc = RCC_USART2,
        .usart = USART2,
        .gpio = GPIOA,
        .gpio_rx_pin = GPIO3,
        .gpio_tx_pin = GPIO2,

        .irq_mask = USART_CR1_TCIE | USART_CR1_RXNEIE,
        .irq = NVIC_USART2_IRQ,

        .dmap = &DMA1_CHANNEL7p,
};

/* DMX USART */
const struct usart_periph USART3p = {
        .id = 3,
        .rcc = RCC_USART3,
        .usart = USART3,
        .gpio = GPIOB,
        .gpio_rx_pin = GPIO11,
        .gpio_tx_pin = GPIO10,
};


/*
 * --------------------- UTIL ---------------------
 */

bool dma_is_channel_enabled(uint32_t dma, uint8_t channel)
{
	return DMA_CCR(dma, channel) & DMA_CCR_EN;
}

void usart_clear_flag(uint32_t usart, uint32_t flag) {
        USART_SR(usart) &= ~flag;
}

void usart_wait_send_complete(uint32_t usart)
{
	/* Wait until the data has been transferred into the shift register. */
	while ((USART_SR(usart) & USART_SR_TC) == 0);

        usart_clear_flag(usart, USART_SR_TC);
}


size_t list_length(struct list_head *head) {
        struct list_head *pos;
        size_t i=0;
        list_for_each(pos, head) {
                i++;
        }

        return i;
}




/*
 * --------------------- DMA ---------------------
 */

enum dma_tflags {
        DMA_FLAGS_NONE     = 0,
        DMA_CIRCULAR       = 1 << 0,
        DMA_PERIPHERAL_INC = 1 << 1,
        DMA_MEMORY_INC     = 1 << 2,
};

enum dma_dir {
        DMA_DIR_FROM_PERIPHERAL = 0xd3ad0093,
        DMA_DIR_FROM_MEMORY     = 0xd3ad0033,
};

void dma_init(struct dma_channel_periph d,

              uint32_t periph_addr, uint32_t periph_size,
              uint32_t mem_addr, uint32_t mem_size,
              uint16_t count,

              enum dma_tflags flags,
              enum dma_dir    dir
        )
{
        /* printf("dma_init %06x CH%02x %08x<->%08x %04x\n", */
        /*        (unsigned int)d.dma & 0xffffff, */
        /*        (unsigned int)d.channel, */
        /*        (unsigned int)periph_addr, */
        /*        (unsigned int)mem_addr, */
        /*        (unsigned int)count ); */

        rcc_periph_clock_enable(d.rcc);

        dma_disable_channel(d.dma, d.channel);
        dma_channel_reset(d.dma, d.channel);
        dma_set_peripheral_address(d.dma, d.channel, periph_addr);
        dma_set_memory_address(d.dma, d.channel, mem_addr);
        dma_set_number_of_data(d.dma, d.channel, count);
        dma_set_priority(d.dma, d.channel, DMA_CCR_PL_HIGH);

        dma_set_peripheral_size(d.dma, d.channel, periph_size);
        dma_set_memory_size(d.dma, d.channel, mem_size);

        if(flags & DMA_MEMORY_INC)
                dma_enable_memory_increment_mode(d.dma, d.channel);
        if(flags & DMA_PERIPHERAL_INC)
                dma_enable_peripheral_increment_mode(d.dma, d.channel);
        if(flags & DMA_CIRCULAR)
                dma_enable_circular_mode(d.dma, d.channel);

        if(dir == DMA_DIR_FROM_PERIPHERAL)
                dma_set_read_from_peripheral(d.dma, d.channel);
        else if(dir == DMA_DIR_FROM_MEMORY)
                dma_set_read_from_memory(d.dma, d.channel);
        else
                assert(false);

        nvic_enable_irq(d.irq);
        nvic_set_priority(d.irq, 0);

        DMA_CCR(d.dma, d.channel) |=
                d.irq_mask & (DMA_CCR_TEIE | DMA_CCR_HTIE | DMA_CCR_TCIE);

        dma_enable_channel(d.dma, d.channel);
}

void ic_dma_init(struct timer_periph t, uintptr_t mem_addr, uint16_t len)
{
        dma_init(*t.dmap,
                 (uint32_t)&TIM_DMAR(t.tim), DMA_CCR_PSIZE_32BIT,
                 (uint32_t)mem_addr, DMA_CCR_MSIZE_16BIT,
                 len, /* count*/
                 DMA_MEMORY_INC | DMA_CIRCULAR,
                 DMA_DIR_FROM_PERIPHERAL
                );
}

void usart_dma_init(struct usart_periph u, uintptr_t mem_addr, uint16_t len,
                    uint8_t source_channel)
{
        usart2_dma_in_progress = 1;

        usart2_dma_bitmap |= 1 << source_channel;

        usart_enable_tx_dma(u.usart);
        dma_init(*u.dmap,
                 (uint32_t)&USART_DR(u.usart), DMA_CCR_PSIZE_32BIT,
                 (uint32_t)mem_addr, DMA_CCR_MSIZE_8BIT,
                 len,
                 DMA_MEMORY_INC,
                 DMA_DIR_FROM_MEMORY
                );
}

void dma_queue_add(uint8_t channel, uintptr_t mem_addr, uint16_t count) {

        if(!usart2_dma_in_progress) {
                usart_dma_init(USART2p, mem_addr, count, channel);
                printf("QQT:%x ", channel);
                return;
        }

        if(list_empty(&dma_freelist)) {
                printf("DRP:%x ", channel);
                return;
        }

        printf("FR:%x ", list_length(&dma_freelist));

        struct dma_transfer *trans =
                list_first_entry(&dma_freelist, struct dma_transfer, freelist);

        trans->channel = channel;
        trans->mem_addr = mem_addr;
        trans->count = count;
        INIT_LIST_HEAD(&trans->queue);

        CM_ATOMIC_BLOCK() {
                list_del(&trans->freelist);
                list_add_tail(&trans->queue, &dma_queue);
        }
}

void dma_channel_error_isr(const struct dma_channel_periph d) {
        /* printf("\n"); */

        if(dma_get_interrupt_flag(d.dma, d.channel, DMA_TEIF)) {
                dma_clear_interrupt_flags(d.dma, d.channel, DMA_TEIF);
                printf("TE ");
                assert(false);
        }
}

void dma_channel_ic_complete_isr(const struct dma_channel_periph d, uint8_t tim_id)
{
        const uint32_t time = systick;
        const uint16_t len = dma_chunk_size;

        uintptr_t mem_addr = (uintptr_t)&dma_buffers[tim_id];
        struct dma_header *header = &dma_headers[tim_id];

        union {
                char str[4];
                uint32_t num;
        } preamble = { .str = "\0\0\r\n" };

        if(dma_get_interrupt_flag(d.dma, d.channel, DMA_TCIF)) {
                dma_clear_interrupt_flags(d.dma, d.channel, DMA_TCIF);

                printf("FTQ:%x ", d.channel);

                memset(header, 0, sizeof(*header));
                header->preamble = preamble.num;
                header->type = DMA_TYPE_UPPER;
                header->channel = tim_id;
                header->systick = __builtin_bswap32(time);


                dma_queue_add(d.channel, (uintptr_t)header, sizeof(*header));
                dma_queue_add(d.channel, mem_addr + len/2, len);

        } else if(dma_get_interrupt_flag(d.dma, d.channel, DMA_HTIF)) {
                dma_clear_interrupt_flags(d.dma, d.channel, DMA_HTIF);

                printf("HTQ:%x ", d.channel);

                memset(header, 0, sizeof(*header));
                header->preamble = preamble.num;
                header->type = DMA_TYPE_LOWER;
                header->channel = tim_id;
                header->systick = __builtin_bswap32(time);

                dma_queue_add(d.channel, (uintptr_t)header, sizeof(*header));
                dma_queue_add(d.channel, mem_addr, len);
        }
}

void dma_channel_usart_complete_isr(const struct dma_channel_periph d,
                                    const struct usart_periph u) {
        if(dma_get_interrupt_flag(d.dma, d.channel, DMA_TCIF)) {
                dma_clear_interrupt_flags(d.dma, d.channel, DMA_TCIF);
                printf(" UTC ");

                usart_disable_tx_dma(u.usart);
        }

        if(dma_get_interrupt_flag(d.dma, d.channel, DMA_HTIF)) {
                dma_clear_interrupt_flags(d.dma, d.channel, DMA_HTIF);
                /* printf("HT "); */
        }
}

/* TIM4 CH1 */
void dma1_channel1_isr(void) {
        dma_channel_error_isr(DMA1_CHANNEL1p);
        dma_channel_ic_complete_isr(DMA1_CHANNEL1p, 3);
}

/* TIM1 CH1 */
void dma1_channel2_isr(void) {
        dma_channel_error_isr(DMA1_CHANNEL2p);
        dma_channel_ic_complete_isr(DMA1_CHANNEL2p, 0);
}

/* TIM2 CH1 */
void dma1_channel5_isr(void) {
        dma_channel_error_isr(DMA1_CHANNEL5p);
        dma_channel_ic_complete_isr(DMA1_CHANNEL5p, 1);
}

/* TIM3 CH1 */
void dma1_channel6_isr(void) {
        dma_channel_error_isr(DMA1_CHANNEL6p);
        dma_channel_ic_complete_isr(DMA1_CHANNEL6p, 2);
}



/* CHANNEL7 <-> USART2 */
void dma1_channel7_isr(void) {
        dma_channel_error_isr(DMA1_CHANNEL7p);
        dma_channel_usart_complete_isr(DMA1_CHANNEL7p, USART2p);
}


/*
 * --------------------- IC ---------------------
 */


void ic_init(struct timer_periph t)
{
        printf("ic_init %d\n", t.id);
	rcc_periph_clock_enable(t.rcc);

        timer_reset(t.tim);

        int i = 0;
        for(; t.irqs[i] > 0; i++) {
                nvic_enable_irq(t.irqs[i]);
                nvic_set_priority(t.irqs[i], 0);
        }
        timer_enable_irq(t.tim, t.irq_mask);

        uint32_t dcr = (1 << 8) | 13;
        TIM_DCR(t.tim) = dcr;

        timer_set_mode(t.tim,
                       TIM_CR1_CKD_CK_INT,
                       TIM_CR1_CMS_EDGE,
                       TIM_CR1_DIR_UP);
        timer_set_period(t.tim, 0xffff);
        timer_set_prescaler(t.tim, 1);

        /* timer_update_on_overflow(t.tim); /\* UEV only on overflow *\/ */


        timer_ic_set_filter(t.tim, TIM_OC1, TIM_IC_CK_INT_N_2);
        timer_ic_set_filter(t.tim, TIM_OC2, TIM_IC_CK_INT_N_2);

        timer_ic_set_input(t.tim, TIM_IC1, TIM_IC_IN_TI1);
        timer_set_oc_polarity_low(t.tim, TIM_OC1);

        timer_ic_set_input(t.tim, TIM_IC2, TIM_IC_IN_TI1);
        timer_set_oc_polarity_high(t.tim, TIM_OC2);

        timer_slave_set_trigger(t.tim, TIM_SMCR_TS_TI1FP1);
        timer_slave_set_mode(t.tim, TIM_SMCR_SMS_RM);

        timer_ic_enable(t.tim, TIM_IC1);
        timer_ic_enable(t.tim, TIM_IC2);

        timer_enable_counter(t.tim);
}

void ic_all_init(void) {
        printf("ic_all_init\n");

        uint16_t count = sizeof(dma_buffers[0]) / 2;
        ic_dma_init(TIM1p, (uintptr_t)&dma_buffers[0], count);
        ic_dma_init(TIM2p, (uintptr_t)&dma_buffers[1], count);
        /* ic_dma_init(TIM3p, (uintptr_t)&dma_buffers[2], count); */
        ic_dma_init(TIM4p, (uintptr_t)&dma_buffers[3], count);

        ic_init(TIM1p);
        ic_init(TIM2p);
        /* ic_init(TIM3p); */
        ic_init(TIM4p);
}

void ic_all_reset(void) {
        printf("ic_all_reset\n");

        timer_reset(TIM1p.tim);
        timer_reset(TIM2p.tim);
        /* timer_reset(TIM3p.tim); */
        timer_reset(TIM4p.tim);
}

static volatile uint32_t tim_per_flags[4] = {0};
static volatile uint16_t tim_per[4] = {0};
static volatile uint16_t tim_duty[4] = {0};

void tim_ic_cc_isr(struct timer_periph t)
{
        /* printf("\nCC "); */

	if (timer_get_flag(t.tim, TIM_SR_CC1IF)) {
                timer_clear_flag(t.tim, TIM_SR_CC1IF);
                tim_per_flags[t.id] |= TIM_SR_CC1IF;

                /* if(t.id == 3) { */
                /*         gpio_toggle(GPIOC, GPIO13); */
                /*         gpio_toggle(GPIOC, GPIO13); */
                /*         gpio_toggle(GPIOC, GPIO13); */
                /*         gpio_toggle(GPIOC, GPIO13); */
                /* } */

                /* printf("CC1 "); */
        }

	if (timer_get_flag(t.tim, TIM_SR_CC2IF)) {
                timer_clear_flag(t.tim, TIM_SR_CC2IF);
                tim_per_flags[t.id] |= TIM_SR_CC2IF;

                /* if(t.id == 3) { */
                /*         gpio_toggle(GPIOC, GPIO13); */
                /*         gpio_toggle(GPIOC, GPIO13); */
                /* } */

                /* printf("CC2 "); */
        }

	/* if (timer_get_flag(t.tim, TIM_SR_CC3IF)) { */
        /*         timer_clear_flag(t.tim, TIM_SR_CC3IF); */

        /*         printf("CC3 "); */
        /* } */

	/* if (timer_get_flag(t.tim, TIM_SR_CC4IF)) { */
        /*         timer_clear_flag(t.tim, TIM_SR_CC4IF); */

        /*         printf("CC4 "); */
        /* } */


}

void tim_ic_ut_isr(struct timer_periph t) {
        /* printf("\nUT "); */

        /* if (timer_get_flag(t.tim, TIM_SR_TIF)) { printf("T "); } */
        /* if (timer_get_flag(t.tim, TIM_SR_UIF)) { printf("U "); } */
        /* if (timer_get_flag(t.tim, TIM_SR_BIF)) { printf("B "); } */
        /* if (timer_get_flag(t.tim, TIM_SR_COMIF)) { printf("COM "); } */
        /* if (timer_get_flag(t.tim, TIM_SR_CC1OF)) { printf("CX1 "); } */
        /* if (timer_get_flag(t.tim, TIM_SR_CC2OF)) { printf("CX2 "); } */
        /* if (timer_get_flag(t.tim, TIM_SR_CC3OF)) { printf("CX3 "); } */
        /* if (timer_get_flag(t.tim, TIM_SR_CC4OF)) { printf("CX4 "); } */

        if(timer_get_flag(t.tim, TIM_SR_UIF)) {
                /* if(t.id == 3) { */
                /*         gpio_toggle(GPIOC, GPIO13); */
                /*         gpio_toggle(GPIOC, GPIO13); */
                /* } */
        }


        if(timer_get_flag(t.tim, TIM_SR_TIF)) {
                timer_clear_flag(t.tim, TIM_SR_TIF);
		timer_clear_flag(t.tim, TIM_SR_UIF);

                /* tim_per_flags[t.id] = 0; */
                tim_per[t.id] = TIM_CCR1(t.tim);
                tim_duty[t.id] = TIM_CCR2(t.tim);
                if(t.id == 3) {
                        gpio_toggle(GPIOC, GPIO13);
                        gpio_toggle(GPIOC, GPIO13);
                }


                /* if(t.id == 3) */
                /*         printf("\nCC%04x:%04x ", (unsigned int)tim_per[t.id], tim_duty[t.id]); */



                /* printf("T "); */
                return;
        } else if (timer_get_flag(t.tim, TIM_SR_UIF)) {
		timer_clear_flag(t.tim, TIM_SR_UIF);

                /* printf("RU "); */

                /* if(tim_per_flags[t.id] == 0) { */
                /*         /\* printf("F "); *\/ */
                /*         uint16_t p = gpio_get(t.gpio, t.gpio_pin); */
                /*         if(p) { */
                /*                 tim_per[t.id] = 0xffff; */
                /*                 tim_duty[t.id] = 0xffff; */
                /*         } else { */
                /*                 tim_per[t.id] = 0xffff; */
                /*                 tim_duty[t.id] = 0; */
                /*         } */
                /* } */
                /* tim_per_flags[t.id] = 0; */

                /* printf("\n"); */

                return;
	}
}

void tim1_up_isr(void)
{
        tim_ic_ut_isr(TIM1p);
}

void tim1_cc_isr(void)
{
        tim_ic_cc_isr(TIM1p);
}

void tim2_isr(void) {
        tim_ic_cc_isr(TIM2p);
        tim_ic_ut_isr(TIM2p);
}

void tim3_isr(void) {
        tim_ic_cc_isr(TIM3p);
        tim_ic_ut_isr(TIM3p);
}

void tim4_isr(void) {
        tim_ic_cc_isr(TIM4p);
        tim_ic_ut_isr(TIM4p);
}

/*
 * --------------------- PWM ---------------------
 */

enum tim_oc_mode pwm_mode;
void pwm_init(struct timer_periph t)
{
        printf("pwm_init\n");


	rcc_periph_clock_enable(t.rcc);


	gpio_set_mode(t.gpio,
                      GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
                      t.gpio_pin);

	timer_reset(t.tim);

	timer_set_mode(t.tim,
                       TIM_CR1_CKD_CK_INT,
                       TIM_CR1_CMS_CENTER_1,
		       TIM_CR1_DIR_UP);
	timer_set_oc_mode(t.tim, TIM_OC1, TIM_OCM_PWM2);

        timer_enable_oc_preload(t.tim, TIM_OC1);
        timer_enable_preload(t.tim);
        timer_continuous_mode(t.tim);

	timer_set_prescaler(t.tim, 0);
	timer_set_period(t.tim, 0xfafb);
	timer_set_oc_value(t.tim, TIM_OC1, 0x7d7e);

        timer_set_oc_polarity_high(t.tim, TIM_OC1);
	timer_enable_oc_output(t.tim, TIM_OC1);
	timer_enable_break_main_output(t.tim);

        timer_generate_event(t.tim, TIM_EGR_UG);
	timer_enable_counter(t.tim);
}


/*
 * --------------------- USART ---------------------
 */

void usart_init(struct usart_periph u, uint32_t baud, uint32_t mode)
{
	rcc_periph_clock_enable(u.rcc);

	gpio_set_mode(u.gpio,
                      GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
                      u.gpio_tx_pin);
	gpio_set_mode(u.gpio,
                      GPIO_MODE_INPUT,
		      GPIO_CNF_INPUT_FLOAT,
                      u.gpio_rx_pin);

	usart_disable(u.usart);

	/* Setup UART parameters. */
	usart_set_baudrate(u.usart, baud);
	usart_set_databits(u.usart, 8);
	usart_set_stopbits(u.usart, USART_STOPBITS_2);
	usart_set_parity(u.usart, USART_PARITY_NONE);
	usart_set_flow_control(u.usart, USART_FLOWCONTROL_NONE);
	usart_set_mode(u.usart, mode);

        if(u.irq_mask) {
                USART_CR1(u.usart) |= u.irq_mask;
                nvic_enable_irq(u.irq);
                nvic_set_priority(u.irq, 0);
        }

        if(u.dmap) {
                usart_enable_tx_dma(u.usart);

        }

	/* Finally enable the USART. */
	usart_enable(u.usart);
}

int usart_dma_tx_complete(void) {
        if(list_empty(&dma_queue)) {
                printf("QEE ");
                return 0;
        }

        struct dma_transfer *trans =
                list_first_entry(&dma_queue, struct dma_transfer, queue);

        usart_dma_init(USART2p, trans->mem_addr, trans->count, trans->channel);
        printf("XXT:%x ", trans->channel);

        list_del(&trans->queue);
        list_add(&trans->freelist, &dma_freelist);

        return 1;
}

void usart2_isr(void)
{
        /* static volatile unsigned int cnt = 0; */

        if(usart_get_flag(USART2, USART_SR_TC)) {
                usart_clear_flag(USART2, USART_SR_TC);

                if(usart2_dma_in_progress) {
                        /* cnt = 0; */

                        if(usart_dma_tx_complete() <= 0)
                                usart2_dma_in_progress = 0;

                        printf("TXC\n");

                }

        } else if(usart_get_flag(USART2, USART_SR_RXNE)) {
                usart_clear_flag(USART2, USART_SR_RXNE);


                uint16_t data = usart_recv(USART2);


                if(data == 0x13 /* stop */) {
                        ic_all_reset();
                } else if(data == 0x11 /* continue*/) {
                        ic_all_init();
                } else if(data == 0x0D) {
                          dma_queue_add(255, (uintptr_t)&dma_escape, sizeof(dma_escape));
                }
        } else {
                /* cnt++; */
                assert(false);
        }

//SR
// 0b 1000 0000
//    7654 3210
//CR
// 0b 0010 0000 0100 1000
// 0b 0010 0000 0100 1000
//              7654 3210
}



/* This is a syscall for newlib */
int _write(int file, char *ptr, int len);
int _write(int file, char *ptr, int len)
{
	int i;

	if (file == STDOUT_FILENO || file == STDERR_FILENO) {
		for (i = 0; i < len; i++) {
			if (ptr[i] == '\n') {
				usart_send_blocking(USART1, '\r');
			}
			usart_send_blocking(USART1, ptr[i]);
		}
		return i;
	}
	errno = EIO;
	return -1;
}

void usart_dmx(struct usart_periph u, uint8_t *frame, uint32_t len)
{
	gpio_set_mode(u.gpio,
                      GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL,
                      u.gpio_tx_pin);

        gpio_set(u.gpio, u.gpio_tx_pin);

        cm_disable_interrupts();
        gpio_clear(u.gpio, u.gpio_tx_pin);

        for(uint32_t i=0; i < 3; i++) {
                usart_send_blocking(u.usart, 0x0);
        }
        while ((USART_SR(u.usart) & USART_SR_TC) == 0);

        gpio_set(u.gpio, u.gpio_tx_pin);

        for(uint32_t i=0; i < 1; i++)
                usart_send_blocking(u.usart, 0x0);
        while ((USART_SR(u.usart) & USART_SR_TC) == 0);

	gpio_set_mode(u.gpio,
                      GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
                      u.gpio_tx_pin);

        for(uint32_t i=0; i < len; i++) {
                usart_send_blocking(u.usart, frame[i]);
                /* while ((USART_SR(u.usart) & USART_SR_TC) == 0); */
                /* for(uint32_t i=0; i < 1000; i++) */
                /*         __asm__("nop"); */

        }

        while ((USART_SR(u.usart) & USART_SR_TC) == 0);
	gpio_set_mode(u.gpio,
                      GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL,
                      u.gpio_tx_pin);

        for(uint32_t i=0; i < 10000; i++)
                __asm__("nop");

        cm_enable_interrupts();
}

/*
 * --------------------- SYSTICK ---------------------
 */

void systick_init(void)
{
        systick_set_clocksource(STK_CSR_CLKSOURCE_AHB_DIV8);
        systick_set_reload(8999);
        systick_interrupt_enable();
        systick_counter_enable();
}

void sys_tick_handler(void)
{
        systick++;
}

/*
 * --------------------- MAIN ---------------------
 */

int main(void)
{
        cm_disable_interrupts();

        init_static_list(dma_queue_storage, freelist, dma_freelist);
        memset(&dma_escape, 0x4040, sizeof(dma_escape));

	rcc_clock_setup_in_hse_8mhz_out_72mhz();
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOC);
        rcc_periph_clock_enable(RCC_AFIO);

        /* toggle debug */
	gpio_set_mode(GPIOC,
                      GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL,
                      GPIO13);

        usart_init(USART1p, 1000000, USART_MODE_TX);
        usart_init(USART2p, 1000000, USART_MODE_TX_RX);
        usart_init(USART3p, 250000,  USART_MODE_TX);

        printf("\n\n\n\nhello world\n");

        systick_init();
        ic_all_init();
        pwm_init(TIM3p);

        printf("\n");

        cm_enable_interrupts();

        uint16_t per[4] = {0}, duty[4] = {0};
        uint8_t frame[513] = { 0, 0xff, 0, 0, 0, 0, 0, 0 };

        for(;;) {
                printf("__DBM__:%x ", (unsigned int)usart2_dma_bitmap);
                usart2_dma_bitmap = 0;
                /* printf("\n"); */
                /* printf("IC %04x %04x\n", tim_per[2], tim_duty[2]); */
                for(long i=0; i < 10000000L; i++) {
                        __asm__("nop");
                }
        }

        usart_dmx(USART3p, frame, 513);
        usart_dmx(USART3p, frame, 513);
        usart_dmx(USART3p, frame, 513);
        while(systick != 0x800);

        {
                int k = 0;
                for(;;) {
                        frame[2] = (k += 51);
                        usart_dmx(USART3p, frame, 20);
                        printf("F%06lx\n", systick);

                        for(long i=0; i < 10000000L; i++) {
                                __asm__("nop");
                        }

                }
        }

        for(;;);

        int i, j, k = 0;
	while (1) {
                frame[2] = (k += 51);
                usart_dmx(USART3p, frame, 20);
                printf("F%06lx\n", systick);
                /* printf("F%06lx %04x %04x %04x %04x\n", */
                /*        systick, frame[2], frame[3], frame[4], frame[5]); */

                for(j=0; j < 100; j++) {
                        bool changed = false;
                        for(i=0; i < 4; i++) {
                                if(tim_per[i] != per[i] || tim_duty[i] != duty[i]) {
                                        changed = true;
                                        j=0;
                                        break;
                                }
                        }

                        if(changed) {
                                printf("D%06lx ", systick);

                                for(i=0; i < 4; i++) {

                                        per[i] = tim_per[i];
                                        duty[i] = tim_duty[i];

                                        /* printf("%04x %04x ", */
                                        /*        tim_per[i], tim_duty[i]); */
                                }
                                printf("\n");
                        }
                }

                printf("\n");
                for(int l=0; l < 10000000; l++) {
                        __asm__("nop");
                }
        }

        return 0;
}
