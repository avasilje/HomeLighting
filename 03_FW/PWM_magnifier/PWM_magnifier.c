#include <stdlib.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/io.h>
/*
 * PWM_magnifier.c
 *
 * Created: 2/21/2016 9:49:58 AM
 *  Author: solit
 */ 
uint16_t g_timer_cnt = 0;
uint8_t steady_val, steady_detected, steady_mask;
uint8_t fall_detected;

#include <avr/io.h>
#define IN_SW_0_MASK (1 << PINB7)
#define IN_SW_1_MASK (1 << PINB6)
#define IN_SW_2_MASK (1 << PINB0)
#define IN_SW_3_MASK (1 << PINB1)

#define IN_SW_0 PINB7
#define IN_SW_1 PINB6
#define IN_SW_2 PINB0
#define IN_SW_3 PINB1
#define IN_SW_MASK \
    ((1 << IN_SW_0)|\
     (1 << IN_SW_1)|\
     (1 << IN_SW_2)|\
     (1 << IN_SW_3))

#define OUT_SW0_0 PINB2
#define OUT_SW0_1 PINB3
#define OUT_SW0_2 PINB4

#define OUT_SW0_0_MASK (1 << OUT_SW0_0)
#define OUT_SW0_1_MASK (1 << OUT_SW0_1)
#define OUT_SW0_2_MASK (1 << OUT_SW0_2)

#define OUT_SW0_MASK \
    ((1 << OUT_SW0_0)|\
     (1 << OUT_SW0_1)|\
     (1 << OUT_SW0_2))

#define OUT_SW1_3    PIND5
#define OUT_SW1_3_MASK (1 << PIND5)
#define OUT_SW1_MASK (OUT_SW1_3_MASK)

#define ENABLE_PCINT  GIMSK |= (1 << PCIE)
#define DISABLE_PCINT  GIMSK &= ~(1 << PCIE)

#define ENABLE_FAST_PWM2 \
            TCCR2 = (1 << WGM20) |\
                    (1 << WGM21) |\
                    (1 << COM20) |\
                    (1 << COM21) |\
                    (1 << CS20 )

#define TIMER_CNT_SCALER 0

#define IN_SW_NUM 4
typedef struct in_sw_s {
    uint16_t fall_time;
    uint16_t fall_time_prev;
    uint16_t rise_time;
    uint8_t  pwm;
} in_sw_t;

in_sw_t in_sw[IN_SW_NUM];

#ifndef DBG_EN
#define DBG_EN 0
#endif

#if DBG_EN
#define DBG_VAR_NUM 24
uint16_t dbg_p[DBG_VAR_NUM];
uint8_t dbg_idx = 0;
#endif

void set_steady_state(uint8_t detected, uint8_t val)
{
    // Val 0 ==> On ==> output 1
    // Val 1 ==> Off ==> output 0
    if (detected & IN_SW_0_MASK){
        in_sw[0].pwm = 0;
        if (val & IN_SW_0_MASK) {
            // OUT_SW0_0 PINB2  ==> OC0A
            PORTB &= ~OUT_SW0_0_MASK;
        }else{
            PORTB |= OUT_SW0_0_MASK;
        }
        TCCR0A &= ~((1 << COM0A1) | (1 << COM0A0));
    }

    if (detected & IN_SW_1_MASK){
        if (val & IN_SW_1_MASK) {
            // OUT_SW0_1 ==> OC1A
            PORTB &= ~OUT_SW0_1_MASK;
        }else{
            PORTB |= OUT_SW0_1_MASK;
        }
        TCCR1A &= ~((1 << COM1A1) | (1 << COM1A0));

    }

    if (detected & IN_SW_2_MASK){
        if (val & IN_SW_2_MASK) {
            // OUT_SW0_2 ==> OC1B
            PORTB &= ~OUT_SW0_2_MASK;
        }else{
            PORTB |= OUT_SW0_2_MASK;
        }
        TCCR1A &= ~((1 << COM1B1) | (1 << COM1B0));
    }

    if (detected & IN_SW_3_MASK){
        if (val & IN_SW_3_MASK) {
            // OUT_SW1_3 ==> OC0B
            PORTD &= ~OUT_SW1_3_MASK;
        }else{
            PORTD |= OUT_SW1_3_MASK;
        }
        TCCR0A &= ~((1 << COM0B1) | (1 << COM0B0));
    }
}

void set_pwm(uint8_t mask, uint8_t pwm_val)
{
    uint8_t volatile *pwm_reg_ptr = NULL;

    // Selet PWM register
    switch(mask){
        case IN_SW_0_MASK:
            TCCR0A |= (1 << COM0A1);   // Clear OC0A on Compare Match
            pwm_reg_ptr = &OCR0A;
            break;
        case IN_SW_1_MASK:
            TCCR1A |= (1 << COM1A1);
            pwm_reg_ptr = &OCR1AL;
            break;
        case IN_SW_2_MASK:
            TCCR1A |= (1 << COM1B1);
            pwm_reg_ptr = &OCR1BL;
            break;
        case IN_SW_3_MASK:
            TCCR0A |= (1 << COM0B1);
            pwm_reg_ptr = &OCR0B;
            break;
    }

    // Avoid too small changes in order to avoid flickering
    if (pwm_reg_ptr && abs(pwm_val - *pwm_reg_ptr) > 2) {
        *pwm_reg_ptr = pwm_val;
    }

}

int main(void)
{
    DDRB = 0;

    // IN switches - pulled up inputs
    DDRB &= ~IN_SW_MASK; 
    PORTB |= IN_SW_MASK; 

    // Out switches - output 0
    DDRB |= OUT_SW0_MASK; 
    PORTB &= ~OUT_SW0_MASK;

    DDRD |= OUT_SW1_MASK; 
    PORTD &= ~OUT_SW1_MASK;

    PCMSK = IN_SW_MASK;
    ENABLE_PCINT;

    EIFR &= ~(1 << PCIF);

    // Set timer 0 & 1 to 8-bit Fast PWM, no prescaler
    TCCR0A = (1 << WGM00) | (1 << WGM01);
    TCCR0B = (1 << CS00);

    TCCR1A = (1 << WGM10);
    TCCR1B = (1 << WGM12) | (1 << CS10);

    // Enable Timer0 overflow interrupt
    TIMSK = (1<<TOIE0);

    // Consider all input as steady 1 (LED OFF) at startup.
    steady_val = IN_SW_MASK;
    steady_detected = IN_SW_MASK;
    set_steady_state(steady_detected, steady_val);

    sei();

    // Endless loop
    while(1)
    {
        uint8_t i, mask = 0;
        for (i = 0; i < IN_SW_NUM; i++) {
            
            switch (i) {
                case 0: mask = IN_SW_0_MASK; break;
                case 1: mask = IN_SW_1_MASK; break;
                case 2: mask = IN_SW_2_MASK; break;
                case 3: mask = IN_SW_3_MASK; break;
            }

            DISABLE_PCINT;
            if (fall_detected & mask) {
                fall_detected &= ~mask;

                if ( 0 == (steady_detected & mask)) {
                    int     width, period;
                    uint8_t pwm;

                    // Calc period 
                    width  = (uint16_t)(in_sw[i].rise_time - in_sw[i].fall_time_prev);
                    period = (uint16_t)(in_sw[i].fall_time - in_sw[i].fall_time_prev);

                    // Release interrupt as soon as possible
                    ENABLE_PCINT;

                    // X = (Width / period * 256) - 1
                    div_t divresult;
                    int   a1, a2;

                    divresult = div(width * 8, period);
                    a1 = divresult.quot * 32;
                    a2 = divresult.rem * 32;
                    a2 += (period >> 1);            // Rounding
                    divresult = div(a2, period);
                    pwm = a1 + divresult.quot - 1;
                    in_sw[i].pwm = pwm;

                    set_pwm(mask, pwm);
#if DBG_EN
                    //dbg_p[dbg_idx++] = g_timer_cnt - in_sw[0].fall_time;
                    dbg_p[dbg_idx++] = period;
                    if (dbg_idx == DBG_VAR_NUM) dbg_idx = 0;

                    dbg_p[dbg_idx++] = width;
                    if (dbg_idx == DBG_VAR_NUM) dbg_idx = 0;

                    dbg_p[dbg_idx++] = pwm;
                    if (dbg_idx == DBG_VAR_NUM) dbg_idx = 0;
#endif
                }
                in_sw[i].fall_time_prev = in_sw[i].fall_time;

            } // Switch fall detected 
            else {
                ENABLE_PCINT;
            }
        } // Input switch loop

        {   
            uint8_t _steady_val, _steady_detected; 
            cli();
                _steady_val = steady_val;
                _steady_detected = steady_detected;
            sei();

            if (_steady_detected & IN_SW_MASK){
                set_steady_state(_steady_detected, _steady_val);
            }
        }

    } // Endsteady_stateless loop
}

ISR(PCINT_vect) {
    static uint8_t prev_val;
    uint8_t curr_val;
    uint8_t rising;
    uint8_t falling;
    uint16_t timer_cnt;

    // Increase timer cnt 0 precision by its current value
    timer_cnt = (g_timer_cnt << TIMER_CNT_SCALER) | (TCNT0 >> (8 - TIMER_CNT_SCALER));

    // Get current input switch status
    curr_val = PINB; 

    rising = (curr_val ^ prev_val);
    falling = rising & prev_val;
    rising = rising & curr_val;
    prev_val = curr_val;

    steady_mask &= ~falling;
    steady_mask &= ~rising;

    if (falling & IN_SW_0_MASK) {
        // Calculates period 
        in_sw[0].fall_time = timer_cnt;
        fall_detected |= IN_SW_0_MASK;
    }

    if (falling & IN_SW_1_MASK) {
        // Calculates period 
        in_sw[1].fall_time = timer_cnt;
        fall_detected |= IN_SW_1_MASK;
    }

    if (falling & IN_SW_2_MASK) {
        // Calculates period 
        in_sw[2].fall_time = timer_cnt;
        fall_detected |= IN_SW_2_MASK;
    }

    if (falling & IN_SW_3_MASK) {
        // Calculates period 
        in_sw[3].fall_time = timer_cnt;
        fall_detected |= IN_SW_3_MASK;
    }

    // Note: The rise time is delayed by optocoupler for ~64usec (schematic specific)
    //       Compensate that delay 64usec ==> 2 g_timer_cnt's periods
    timer_cnt = timer_cnt - (2 << TIMER_CNT_SCALER);
    if (rising & IN_SW_0_MASK){
        in_sw[0].rise_time = timer_cnt;
    }
    if (rising & IN_SW_1_MASK){
        in_sw[1].rise_time = timer_cnt;
    }
    if (rising & IN_SW_2_MASK){
        in_sw[2].rise_time = timer_cnt;
    }
    if (rising & IN_SW_3_MASK){
        in_sw[3].rise_time = timer_cnt;
    }

}

// Interrupt trigered every 256 timer clocks and count periods
ISR(TIMER0_OVF_vect) {

    // 256 clocks @ 8 MHz ==> 32usec
    // 32ms ==> 1024 periods
    if ((g_timer_cnt & (1024-1)) == 0) {
        steady_val = PINB; 
        steady_detected = steady_mask;              // and no edges were detected
        steady_mask = IN_SW_MASK;                   // All changes have their chance for steady state
                                                    // The flag cleared upon rising/falling edge detection
    }

    g_timer_cnt++;
}
