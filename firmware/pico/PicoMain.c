#include <stdint.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/uart.h"

#define LASER_PIN 16
#define LED_PIN 25
#define BAUDRATE 115200


#define SERVO_X_PIN 14
#define SERVO_Y_PIN 15
#define SERVO_MIN_US 1000
#define SERVO_MAX_US 2000
#define SERVO_PERIOD 20000

uint16_t angle_to_us(uint8_t angle) {
    return SERVO_MIN_US + ( (SERVO_MAX_US - SERVO_MIN_US) * angle) / 180;
}

void set_servo_angle(uint gpio_pin, uint8_t angle) {
    uint slice = pwm_gpio_to_slice_num(gpio_pin);
    uint chan  = pwm_gpio_to_channel(gpio_pin);

    uint16_t us = angle_to_us(angle);
    pwm_set_chan_level(slice, chan, us);
}



int main() {

    /* Set up LED_PIN & LASER_PIN*/

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN,true);

    gpio_init(LASER_PIN);
    gpio_set_dir(LASER_PIN,true);

    /* Set up PWM control*/

    gpio_set_function(SERVO_X_PIN, GPIO_FUNC_PWM);
    gpio_set_function(SERVO_Y_PIN, GPIO_FUNC_PWM);

    uint slicex = pwm_gpio_to_slice_num(SERVO_X_PIN);
    uint slicey = pwm_gpio_to_slice_num(SERVO_Y_PIN);

    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv(&cfg,125);
    pwm_config_set_wrap(&cfg, SERVO_PERIOD - 1);

    pwm_init(slicex, &cfg,true);
    pwm_set_enabled(slicex,true);
    pwm_init(slicey, &cfg,true);
    pwm_set_enabled(slicey,true);


    /* Set up UART communication*/

    uart_init(uart0, 115200);
    gpio_set_function(0, GPIO_FUNC_UART); // TX
    gpio_set_function(1,GPIO_FUNC_UART); // RX
    uart_set_format(uart0, 8, 1, UART_PARITY_NONE);

    enum {WAIT, READ_X, READ_Y,LASER} state = WAIT;

    uint8_t x = 90;
    uint8_t y = 90;
    bool laser = false;

    set_servo_angle(SERVO_X_PIN,x);
    set_servo_angle(SERVO_Y_PIN,y);
    gpio_put(LASER_PIN,false);

    while (true) {
        if (uart_is_readable(uart0)) {
            uint8_t b = uart_getc(uart0);
            gpio_put(LED_PIN, true);
            
            switch (state) {
                case WAIT:
                    if (b == 0xFF) {
                        state = READ_X;
                    }
                    break;
                case READ_X:
                    x = b;
                    state = READ_Y;
                    break;
                case READ_Y:
                    y = b;
                    state = LASER;
                    set_servo_angle(SERVO_X_PIN,x);
                    set_servo_angle(SERVO_Y_PIN,y);
                    break;
                case LASER:
                    state = WAIT;
                    if (b == 0x01) {
                        laser = true;
                    } else {
                        laser = false;
                    }
                    gpio_put(LASER_PIN,laser);
                    break;
                default: 
                    state = WAIT;
                    break;
            }
        }
    }

}