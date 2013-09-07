/*
 * Copyright (C) 2013 Chuck McManis <cmcmanis@mcmanis.com>
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <libopencm3/stm32/f4/rcc.h>
#include <libopencm3/stm32/f4/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <stdint.h>


#define RECV_BUFSIZE 32

/* prototypes */
void msleep(uint32_t);
uint32_t mtime(void);
void uart_putc(char);
void uart_puts(char *);
char uart_getc(int);
char *uart_gets(void);
void uart_setup(void);
void bb_setup(int32_t);

/* 
 * SysTick support routines
 */

/* monotonically increasing number of milliseconds from reset
 * overflows every 49 days if you're wondering
 */
volatile uint32_t system_millis;

/* Called when systick fires */
void
sys_tick_handler(void) {
    system_millis++;
}

/* sleep for delay milliseconds */
void msleep(uint32_t delay) {
    uint32_t wake = system_millis + delay;
    while (wake > system_millis) ;
}

/* return the time */
uint32_t mtime() {
    return system_millis;
}

/* Set up a timer to create 1mS ticks. */
static void
systick_setup(void) {
    /* clock rate / 1000 to get 1mS interrupt rate */
    systick_set_reload(168000);
    systick_set_clocksource(1);
    systick_counter_enable();
    /* this done last */
    systick_interrupt_enable();
}

/* Set STM32 to 168 MHz. */
static void 
clock_setup(void)
{
	rcc_clock_setup_hse_3v3(&hse_8mhz_3v3[CLOCK_3V3_168MHZ]);

	/* Enable GPIOD clock for LED & USARTs. */
	rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPDEN);
	rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPCEN);

	/* Enable clocks for USART6. */
	rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_USART6EN);
}

static
void gpio_setup(void)
{
	/* Setup GPIO pin GPIO12 on GPIO port D for LED. */
	gpio_mode_setup(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO12);

	/* Setup GPIO pins for USART2 transmit. */
	gpio_mode_setup(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO6);
	gpio_mode_setup(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO7);

	gpio_set_output_options(GPIOC, GPIO_OTYPE_OD, GPIO_OSPEED_25MHZ, GPIO7);

	gpio_set_af(GPIOC, GPIO_AF8, GPIO6);
	gpio_set_af(GPIOC, GPIO_AF8, GPIO7);

}

/* The receive buffer, it can over flow if data comes in too quickly
 * that can happen if you're driving the other end of the serial port
 * with another computer
 */
static char recv_buf[RECV_BUFSIZE];
static int buf_ndx;
static int read_ndx;

/* 
 * Dead simple interrupt service routine, takes the character
 * and puts it into the receive queue.
 */
void usart6_isr() {
    recv_buf[buf_ndx++] = USART_DR(USART6);
    buf_ndx %= RECV_BUFSIZE;
}


static void
usart_setup(int32_t baud)
{
	/* Setup USART6 parameters. */
	usart_set_baudrate(USART6, baud);
	usart_set_databits(USART6, 8);
	usart_set_stopbits(USART6, USART_STOPBITS_1);
	usart_set_mode(USART6, USART_MODE_TX_RX);
	usart_set_parity(USART6, USART_PARITY_NONE);
	usart_set_flow_control(USART6, USART_FLOWCONTROL_NONE);

    /* Allow for receive interrupts */
    buf_ndx = 0;
    read_ndx = 0;
    nvic_enable_irq(NVIC_USART6_IRQ);

	/* Finally enable the USART. */
	usart_enable(USART6);
    usart_enable_rx_interrupt(USART6);
}


/*
 * Do chip initialization to set up the chip and peripherals
 *
 * Not sure I like this design pattern, basically we call "bb_setup()" 
 * which is a global, and then this code sets up all the things that
 * the bb "library" is going to be using. That said, I don't like the
 * fairly #ifdef laden style of Atmel or ST Micro either so I'm living
 * with this for now.
 */
void
bb_setup(int32_t baud) {
    clock_setup();
    systick_setup();
    gpio_setup();
    usart_setup(baud);
}

/*
 * uart_putc(char c)
 *
 * Write a character the uart (Blocking). This does what it says,
 * puts out a character to the serial port. If one is in the process
 * of being sent it waits until it finishes then puts this one out.
 */
void
uart_putc(char c) {
    while (!(USART_SR(USART6) & USART_SR_TXE)) {
            __asm__("NOP");
    }
    USART_DR(USART6) = (uint16_t)(c & 0xff);
}

/*
 * uart_puts(char *string)
 *
 * Write a NUL terminated string to the UART. This routine writes
 * the characters in order (adding LF to CR). 
 */
void
uart_puts(char *s) {
    while (*s) {
        if (*s == '\n') {
            uart_putc('\r');
        }
        uart_putc(*s++);
    }
}

/*
 * uart_getc(int wait)
 * 
 * Read a character from the UART. This has a flag 'wait' which
 * if it is non-zero will cause the routine to block until a
 * character shows up in the receive queue. If not set the routine
 * returns '0' immediately if there is no character.
 *
 * Note, this means it is only possible to trust that '0' was the
 * character sent if you call this in a blocking way.
 */
char
uart_getc(int wait) {
    char res;

    if ((read_ndx == buf_ndx) && ! wait) {
        return '\000';
    }
    while (read_ndx == buf_ndx) {
        __asm__("NOP");
    }
    res = recv_buf[read_ndx++];
    read_ndx %= RECV_BUFSIZE;
    return res;
}

#define GET_BUFSIZE 128
/*
 * char *uart_gets()
 *
 * This is the 'old school' getstring function. It allows for basic
 * editng (delete or backspace) and returns on the receipt of <CR>
 * And like libc of old, uses a statically allocated buffer to hold
 * the string which it returns. Saves on memory allocation but is
 * not multi-thread safe, and potentially a security threat.
 */
char *
uart_gets() {
    static char line_buf[GET_BUFSIZE];
    int ndx;
    char c;

    line_buf[0] = '\000';
    ndx = 0;
    // read until you see a <CR>
    while ((c = uart_getc(1)) != '\r') {
        // if you see a ^H or ^? (<DEL>) delete last character
        if ((c == '\010') || (c == '\177')) {
            if (ndx) {
                uart_puts("\010 \010"); // back up one space
                ndx--;
                line_buf[ndx] = '\000';
            }
        } else {
            line_buf[ndx] = c;
            ndx++;
            // if you're at the end of the buffer just return
            if (ndx < GET_BUFSIZE) {
                line_buf[ndx] = '\000';
            } else {
                line_buf[GET_BUFSIZE - 1] = '\000';
                break;
            }
        }
    }
    return &line_buf[0];
}

/*
 * The interesting bits
 */

char *stime(uint32_t);

/*
 * stime(uint32_t)
 *
 * Convert a number representing milliseconds into a 'time' string
 * of HHH:MM:SS.mmm where HHH is hours, MM is minutes, SS is seconds
 * and .mmm is fractions of a second.
 */
char *
stime(uint32_t t) {
    static char time_string[14];
    uint16_t msecs = t % 1000;
    uint8_t secs = (t / 1000) % 60;
    uint8_t mins = (t / 60000) % 60;
    uint16_t hrs = (t /3600000);

    // HH:MM:SS.mmm\0
    // 0123456789abc
    time_string[0] = (hrs % 100) + '0';
    time_string[1] = (hrs / 10) % 10 + '0';
    time_string[2] = hrs % 10 + '0';
    time_string[3] = ':';
    time_string[4] = (mins / 10)  % 10 + '0';
    time_string[5] = mins % 10 + '0';
    time_string[6] = ':';
    time_string[7] = (secs / 10)  % 10 + '0';
    time_string[8] = secs % 10 + '0';
    time_string[9] = '.';
    time_string[10] = (msecs / 100) % 10 + '0';
    time_string[11] = (msecs / 10) % 10 + '0';
    time_string[12] = msecs % 10 + '0';
    time_string[13] = 0;
    return &time_string[0];
}


int main(void)
{
    int pause = 1;
    uint32_t offset;
    uint32_t t;


    // setup 115,200 baud
    bb_setup(115200);

    // make some space
    uart_puts("\n\n");
    offset = mtime();
    t = mtime() - offset;
	/* Blink the LED on the board each time through the loop */
	while (1) {

        // hold time value if paused
        t = (pause) ? t : mtime() - offset;
		gpio_toggle(GPIOD, GPIO12);	/* LED on/off */
        uart_puts(stime(t));
        uart_putc('\r');
        switch (uart_getc(0)) {
            case 'r':
                offset = mtime();   // clear
                break;
            case 's':
                pause ^= 1;
                break;
            default:
                break;
        }
		msleep(100);
	}

	return 0;
}

