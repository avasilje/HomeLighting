// Fuses: 0x99 0x23

#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/io.h>
#include <avr/eeprom.h>
#include "bathroom_lc.h"

void init_static();
void init_global();

void light_on();
void light_off();

void dimm_on(uint8 uc_led);
void highlight_toggle(uint8 uc_led);


//------ Global variables ---------------------

// ------- EEPROM ---------------
uint8 euca_signature[2] __attribute__ ((section (".config_param")));
uint8 euca_standby_intensity[LEDS_NUM] __attribute__ ((section (".config_param")));
uint8 euca_on_intensity[LEDS_NUM] __attribute__ ((section (".config_param")));

// ------- SRAM ----------------

LED_INFO gta_leds[LEDS_NUM];
SWITCH_INFO gta_switches[SWITCHES_NUM];

uint8  guc_sw_event_mask;

uint8  guc_curr_led_mode;

uint8  guc_leds_steady;

uint8  guc_curr_pwm;

uint8  guc_mem_guard;

uint8  guc_vent_on_timer, guc_vent_off_timer, guc_vent_timer_prescaler, guc_vent_mode;

#define INTENSITIES_NUM 19
const PWM_INTENSITY_TABLE guca_pwm_intensity_table[INTENSITIES_NUM] = 
{   { PWM_PERIOD_START_END(  0)},  // 0   (OFF)
    { PWM_PERIOD_START_END(  3)},  // 1   
    { PWM_PERIOD_START_END(  6)},  // 2   
    { PWM_PERIOD_START_END( 10)},  // 3   
    { PWM_PERIOD_START_END( 14)},  // 4   
    { PWM_PERIOD_START_END( 18)},  // 5   
    { PWM_PERIOD_START_END( 23)},  // 6   
    { PWM_PERIOD_START_END( 28)},  // 7   
    { PWM_PERIOD_START_END( 33)},  // 8   
    { PWM_PERIOD_START_END( 38)},  // 9   
    { PWM_PERIOD_START_END( 44)},  // 10  
    { PWM_PERIOD_START_END( 50)},  // 11  
    { PWM_PERIOD_START_END( 56)},  // 12  
    { PWM_PERIOD_START_END( 63)},  // 13  
    { PWM_PERIOD_START_END( 70)},  // 14  
    { PWM_PERIOD_START_END( 77)},  // 15  
    { PWM_PERIOD_START_END( 85)},  // 16  
    { PWM_PERIOD_START_END( 93)},  // 17  
    { PWM_PERIOD_START_END(100)}   // 18 100% (ON)
};


#define DIMM_ON_INTENSITIES_NUM 6
const uint8 guca_dimm_on_intensity_table[DIMM_ON_INTENSITIES_NUM] = 
{
    0,                  // OFF
    4,
    8,
    11,
    14,
    INTENSITIES_NUM-1   // MAX
};

#define DIMM_STDBY_INTENSITIES_NUM 6
const uint8 guca_dimm_stdby_intensity_table[DIMM_STDBY_INTENSITIES_NUM] = 
{
    0,                  // OFF
    2,
    3,
    5,
    6,
    8                  
};

//------- Internal function declaration -------
void light_on(){
    uint8 uc_i;

    for (uc_i = 0; uc_i < LEDS_NUM; uc_i ++)
    { // loop over all leds
        gta_leds[uc_i].uc_target_intensity = gta_leds[uc_i].uc_on_intensity;
        gta_leds[uc_i].uc_fade_timer = LED_FADEIN_PERIOD;
    }
}

void light_off(){
    uint8 uc_i;

    for (uc_i = 0; uc_i < LEDS_NUM; uc_i ++)
    { // loop over all leds
        gta_leds[uc_i].uc_target_intensity = gta_leds[uc_i].uc_off_intensity;
        gta_leds[uc_i].uc_fade_timer = LED_FADEOUT_PERIOD;
    }

}


void dimm_on(uint8 uc_led){

    uint8 uc_i;
    // Loop over intensities from DIMM ON table
    for (uc_i = 0; uc_i < DIMM_ON_INTENSITIES_NUM; uc_i++)
    {
        if (gta_leds[uc_led].uc_on_intensity < guca_dimm_on_intensity_table[uc_i])
        {
            gta_leds[uc_led].uc_on_intensity = guca_dimm_on_intensity_table[uc_i];
            break;
        }
    }

    if (uc_i == DIMM_ON_INTENSITIES_NUM)
    {
        gta_leds[uc_led].uc_on_intensity = guca_dimm_on_intensity_table[0];
    }

    gta_leds[uc_led].uc_target_intensity = gta_leds[uc_led].uc_on_intensity;
    gta_leds[uc_led].uc_fade_timer = LED_FADEIN_PERIOD;
}

void highlight_toggle(uint8 uc_led){

    if (gta_leds[uc_led].uc_highlighted)
    {
        gta_leds[uc_led].uc_target_intensity = gta_leds[uc_led].uc_on_intensity;   // Normal value
        gta_leds[uc_led].uc_fade_timer = LED_FADEOUT_PERIOD;
        gta_leds[uc_led].uc_highlighted = 0;
    }
    else
    {
        gta_leds[uc_led].uc_target_intensity = INTENSITIES_NUM-1;   // MAX value
        gta_leds[uc_led].uc_fade_timer = LED_FADEIN_PERIOD;
        gta_leds[uc_led].uc_highlighted = 1;
    }

}


//---------------------------------------------
int main() {

    uint8 uc_i;
    // --------------------------------------------------
    // --- Init perephirial devices
    // --------------------------------------------------

    // Set switch pins to pull-up inputs
    SW_DIR  &= ~SW_MASK_ALL;
    SW_PORT |= SW_MASK_ALL;

    // Set led to pull down
    LED_DIR  |= LED_MASK_ALL;
    LED_PORT &= ~LED_MASK_ALL;

    // timer - Timer0. Incrementing counting till UINT8_MAX
    TCCR0 = CNT_TCCRxB;
    TCNT0 = CNT_RELOAD;
    ENABLE_TIMER;


    // Wait a little just in case
    for(uc_i = 0; uc_i < 255U; uc_i++){

        __asm__ __volatile__ ("    nop\n    nop\n    nop\n    nop\n"\
                              "    nop\n    nop\n    nop\n    nop\n"\
                              "    nop\n    nop\n    nop\n    nop\n"\
                              "    nop\n    nop\n    nop\n    nop\n"
                                ::);
    }

    init_global();

    init_static();

    // Enable Interrupts
    sei();

    while(1){
        __asm__ __volatile__ ("    nop\n    nop\n    nop\n    nop\n"::);
        __asm__ __volatile__ ("    nop\n    nop\n    nop\n    nop\n"::);
        __asm__ __volatile__ ("    nop\n    nop\n    nop\n    nop\n"::);
        __asm__ __volatile__ ("    nop\n    nop\n    nop\n    nop\n"::);
    }

    return 0;
}

void init_global(){

    // Disable all event until ligth is in steady state after power on
    guc_sw_event_mask = 0;

    guc_curr_led_mode = LED_MODE_OFF_TRANS;
    guc_curr_pwm = 0;

    guc_vent_off_timer = -1;
    guc_vent_on_timer = -1;
    guc_vent_timer_prescaler = 0;
    guc_vent_mode = VENT_MODE_AUTO;

    // turn ON vent mode indicators (INDICATOR_ON->VENT_AUTO)
    LED_PORT |= (1<<VENT_INDICATOR_PIN);

}
void init_static(){

    uint8 uc_i;



    // --- LEDS init ---
    
    // Read EEPROM signature
    {   uint8 uca_sign[2];
        uca_sign[0] = eeprom_read_byte(&euca_signature[0]);
        uca_sign[1] = eeprom_read_byte(&euca_signature[1]);
        if (uca_sign[0] == 'O' && uca_sign[1] == 'K')
        { // EEPROM OK, get config data from EEPROM
            
            for (uc_i = 0; uc_i < LEDS_NUM; uc_i ++)
            {
                gta_leds[uc_i].uc_standby_intensity = eeprom_read_byte(&euca_standby_intensity[uc_i]);
                gta_leds[uc_i].uc_on_intensity = eeprom_read_byte(&euca_on_intensity[uc_i]);
            }

        } // End of EEPROM OK
        else
        { // EEPROM is clear
            // Write default values 
            gta_leds[0].uc_standby_intensity = 0;

            gta_leds[0].uc_on_intensity = 11;

            // Initialize EEPROM
            for (uc_i = 0; uc_i < LEDS_NUM; uc_i ++)
            {
                eeprom_write_byte(&euca_standby_intensity[uc_i], gta_leds[uc_i].uc_standby_intensity);
                eeprom_write_byte(&euca_on_intensity[uc_i], gta_leds[uc_i].uc_on_intensity);
            }

            // Write EEPROM signature
            eeprom_write_byte(&euca_signature[0], 'O');
            eeprom_write_byte(&euca_signature[1], 'K');
        
        }
        
    }


    for (uc_i = 0; uc_i < LEDS_NUM; uc_i ++)
    { // loop over all leds

        gta_leds[uc_i].uc_pin_mask = 1 << uc_i;

        // Constant (loaded from EEPROM)
        gta_leds[uc_i].uc_off_intensity = 0;
        gta_leds[uc_i].uc_hl_intensity = INTENSITIES_NUM-1;

        // Statics
        switch (guc_curr_led_mode){
            case LED_MODE_OFF:
                gta_leds[uc_i].uc_target_intensity = gta_leds[uc_i].uc_off_intensity;
                break;
            case LED_MODE_ON:
                gta_leds[uc_i].uc_target_intensity = gta_leds[uc_i].uc_on_intensity;
                break;
            case LED_MODE_STANDBY:
                gta_leds[uc_i].uc_target_intensity = gta_leds[uc_i].uc_standby_intensity;
                break;
        }
        gta_leds[uc_i].uc_current_intensity = 0;
        gta_leds[uc_i].uc_fade_timer = LED_FADEIN_PERIOD;
        gta_leds[uc_i].uc_highlighted = 0;

    }

    // --- SWITCHES init ---
    for (uc_i = 0; uc_i < SWITCHES_NUM; uc_i ++)
    { // loop over all switches
        gta_switches[uc_i].uc_prev_pin = SW_PIN_RELEASED;
        gta_switches[uc_i].uc_prev_sw = SW_PIN_RELEASED;
        gta_switches[uc_i].uc_hold_timer = 0;
        gta_switches[uc_i].uc_debounce_timer = 0;
        gta_switches[uc_i].uc_pin_num = uc_i;
        gta_switches[uc_i].uc_switch_type = SW_TYPE_BUTT;       // mark all switches as a button, detector marked below
    }

    gta_switches[2].uc_switch_type = SW_TYPE_DETECT;


}


void check_switches(){

    uint8  uc_i;
    uint8  uc_sw_state;
    uint8  uc_sw_pins;
    uint8  uc_curr_pin;

    uc_sw_pins = SW_PIN;

    for (uc_i = 0; uc_i < SWITCHES_NUM; uc_i++)
    { // Loop over all switches 

        uc_sw_state = gta_switches[uc_i].uc_prev_sw;
        uc_curr_pin = (uc_sw_pins >> gta_switches[uc_i].uc_pin_num) & 1;

        // ------------------------------------
        // --- debouncing
        // -----------------------------------
        if (uc_curr_pin != gta_switches[uc_i].uc_prev_pin)
        {// current pin state differs from previous

            // increment debounce timer 
            gta_switches[uc_i].uc_debounce_timer ++;
            
            if (gta_switches[uc_i].uc_debounce_timer == SW_DEBOUNCE_TIMER)
            { // debounce timer expired
                gta_switches[uc_i].uc_prev_pin = uc_curr_pin;

                // modify current switch state (pressed/released)
                uc_sw_state = uc_curr_pin;
                gta_switches[uc_i].uc_debounce_timer = 0;

            }
        }
        else
        {
            gta_switches[uc_i].uc_debounce_timer = 0;
        }

        // ------------------------------------
        // --- transition proceed
        // -----------------------------------
        gta_switches[uc_i].uc_event = 0;

        if (uc_sw_state == SW_PIN_PRESSED && gta_switches[uc_i].uc_prev_sw == SW_PIN_RELEASED)
        { // released->pressed transition
            // clear hold timer
            gta_switches[uc_i].uc_hold_timer = 0;
            gta_switches[uc_i].uc_event = SW_OFF_ON;                                    // ---         OFF -> ON       ---
        }
        else if (uc_sw_state == SW_PIN_RELEASED && gta_switches[uc_i].uc_prev_sw == SW_PIN_PRESSED )
        { // pressed->released transition
            if (gta_switches[uc_i].uc_hold_timer < SW_HOLD_TIMER)
            {
                gta_switches[uc_i].uc_event = SW_ON_OFF;                                // ---         ON -> OFF        ---
            }
            else
            {
                gta_switches[uc_i].uc_event = SW_HOLD_OFF;                              // ---         HOLD -> OFF      ---
            }
        }
        else if (uc_sw_state == SW_PIN_PRESSED && gta_switches[uc_i].uc_prev_sw == SW_PIN_PRESSED)
        {
            if (gta_switches[uc_i].uc_hold_timer < SW_HOLD_TIMER_NEXT)
            { // hold timer not saturated yet

                // increase hold timer & check saturation
                gta_switches[uc_i].uc_hold_timer ++;

                if (gta_switches[uc_i].uc_hold_timer == SW_HOLD_TIMER)
                {// report "switch hold" if timer saturated
                    gta_switches[uc_i].uc_event = SW_ON_HOLD;                           // ---         ON -> HOLD      ---
                }

                if (gta_switches[uc_i].uc_hold_timer == SW_HOLD_TIMER_NEXT)
                {// report "switch hold" if timer saturated
                    gta_switches[uc_i].uc_event = SW_ON_HOLD_NEXT;                      // ---         ON -> HOLD NEXT ---
                    gta_switches[uc_i].uc_hold_timer = SW_HOLD_TIMER;
                }
            } 
        } // End of sw holdon

        gta_switches[uc_i].uc_prev_sw = uc_sw_state;

    } // End of swich loop
            
}
void light_switch(uint8 uc_event){

    uint8 uc_switch = 0;    // switch 0 
    uint8 uc_led = 0;       // controls LED 0 only

    if (guc_curr_led_mode == LED_MODE_OFF)
    {

        if (uc_event == SW_OFF_ON)  // Led OFF mode                 // LED OFF mode
        { // switch pressed

            light_on();

            // Disable all switches excluding just pressed
            guc_sw_event_mask = (1 << uc_switch);
        }

        else if (uc_event == SW_ON_HOLD)  // LED OFF mode
        { // switch pressed for a first time

            // Light control switch
            gta_leds[uc_led].uc_target_intensity = guca_dimm_on_intensity_table[1];
            gta_leds[uc_led].uc_fade_timer = LED_FADEOUT_PERIOD;

        }

        else if (uc_event == SW_ON_HOLD_NEXT)  // LED OFF mode
        { // switch pressed for a first time
            dimm_on(uc_led);
        }
        
        else if (uc_event == SW_HOLD_OFF || uc_event == SW_ON_OFF)  // Led OFF mode
        { // switch released

            if (uc_event == SW_HOLD_OFF)
            {
                eeprom_write_byte(&euca_on_intensity[uc_led], gta_leds[uc_led].uc_on_intensity);
            }

            // Set current mode to TRANSITION to ON
            guc_curr_led_mode = LED_MODE_ON_TRANS;
            // Disable all switches until leds in steady state
            guc_sw_event_mask = 0;
        }

    } // End of LED OFF state

    // ----------------------------------------------------
    // --- LED ON
    // ----------------------------------------------------
    else if (guc_curr_led_mode == LED_MODE_ON)
    {

        if (uc_event == SW_OFF_ON)                                      // LED ON state  OFF->ON
        { // switch pressed
            // Disable all switches excluding just pressed
            guc_sw_event_mask = (1 << uc_switch);
        }

        else if (uc_event == SW_ON_HOLD)                                // LED ON state ON->HOLD
        { // switch pressed
            highlight_toggle(uc_led);
        }

        else if (uc_event == SW_HOLD_OFF)                               // LED ON state HOLD->OFF
        { // switch released after HOLD
            // Keep current state
            // Set current mode to TRANSITION to ON
            guc_curr_led_mode = LED_MODE_ON_TRANS;
            // Disable all switches until leds in steady state
            guc_sw_event_mask = 0;
        }

        else if (uc_event == SW_ON_OFF)                                 // LED ON state ON->OFF
        { // switch released after ON
            light_off();

            // Set current mode to TRANSITION to OFF
            guc_curr_led_mode = LED_MODE_OFF_TRANS;
            // Disable all switches until leds in steady state
            guc_sw_event_mask = 0;
        }

    } // End of LED ON mode 


    return;

}

void sw_behavior_control(){

    uint8 uc_i, uc_event, uc_switch;

    uc_event = 0;

    // get the last unmasked event 
    // clear all other events
    for (uc_i = 0; uc_i < SWITCHES_NUM; uc_i++)
    { // Loop over all switches 

        // skip switch disabled by mask
        if (gta_switches[uc_i].uc_event && (guc_sw_event_mask & (1 << uc_i))){
            uc_event = gta_switches[uc_i].uc_event;
            uc_switch = uc_i;

            if (gta_switches[uc_i].uc_switch_type == SW_TYPE_DETECT) 
                uc_event += 10; 
        }
        gta_switches[uc_i].uc_event = 0;
    }


    // ----------------------------------------------------
    // --- LED ON/OFF states
    // ----------------------------------------------------
    if (uc_event && uc_switch == 0)
    {
        light_switch(uc_event);
        return;
    }
    // ----------------------------------------------
    // --- Lighting transient states
    // ----------------------------------------------

    if (guc_curr_led_mode == LED_MODE_ON_TRANS)
    {
        if (guc_leds_steady)
        {
            guc_sw_event_mask = 0xFF;
            guc_curr_led_mode = LED_MODE_ON;
        }

    } // End of TRANSITION to ON

    else if (guc_curr_led_mode == LED_MODE_OFF_TRANS)
    {
        if (guc_leds_steady)
        {
            guc_sw_event_mask = 0xFF;
            guc_curr_led_mode = LED_MODE_OFF;
        }
    } // End of TRANSITION to OFF


    // ----------------------------------------------
    // --- VENT mode switch
    // ----------------------------------------------
    if (uc_event == SW_ON_OFF && uc_switch == 1)
    { // Toggle VENT mode on each switch ON->OFF transition.  AUTO->OFF->ON->AUTO
        switch (guc_vent_mode)
        {
        case VENT_MODE_AUTO: 
            guc_vent_mode = VENT_MODE_OFF;

            // disable ventilation
            LED_PORT &= ~(1<<VENT_CONTROL_PIN);

            // turn OFF vent mode indicators
            LED_PORT &= ~(1<<VENT_INDICATOR_PIN);
            break;

        case VENT_MODE_OFF: 
            guc_vent_mode = VENT_MODE_ON;

            // Enable ventilation
            LED_PORT |= (1<<VENT_CONTROL_PIN);

            break;

        case VENT_MODE_ON:
            guc_vent_mode = VENT_MODE_AUTO;

            // disable ventilation
            LED_PORT &= ~(1<<VENT_CONTROL_PIN);

            // turn ON vent mode indicators
            LED_PORT |= (1<<VENT_INDICATOR_PIN);
            break;
        }

        return;
    }

    // ----------------------------------------------
    // --- Movement detector Vent control
    // ----------------------------------------------
    if (uc_event && uc_switch == 2){
        if (uc_event == DET_OFF_ON)
        {
            // Enable OFF timer. Counting only in case of ON timer is stopped
            guc_vent_off_timer = VENT_TIMER_OFF;
        }
        else if (uc_event == DET_ON_OFF || uc_event == DET_HOLD_OFF)
        { // Movement detected

            // Disabel ventilation OFF timer
            guc_vent_off_timer = 0xFF;

           // Enable timer if vent not enabled yet and timer not started
            if ( (guc_vent_on_timer == 0xFF) && (LED_PORT & (1<<VENT_CONTROL_PIN)) == 0)
            {
                // Enable ventilation ON timer
                guc_vent_on_timer = VENT_TIMER_ON;

            }
        }
    }

    return;
}

void leds_intensity_control(){
    
    uint8 uc_i;
    uint8 uc_target_intensity, uc_current_intensity;
    
    guc_leds_steady = 1;

    for (uc_i = 0; uc_i < LEDS_NUM; uc_i ++)
    { // Loop over all leds
        
        uc_target_intensity = gta_leds[uc_i].uc_target_intensity;
        uc_current_intensity = gta_leds[uc_i].uc_current_intensity;

        if (uc_target_intensity == uc_current_intensity){
            continue;
        }

        guc_leds_steady = 0;

        // Leds intensity in transition
        // check FADE timer
        gta_leds[uc_i].uc_fade_timer --;
        if ( gta_leds[uc_i].uc_fade_timer == 0)
        { // FADE IN or FADE OUT 

            if (uc_target_intensity > uc_current_intensity)
            { // FADE IN
                uc_current_intensity ++;
                gta_leds[uc_i].uc_fade_timer = LED_FADEIN_PERIOD;
            }
            else if (uc_target_intensity < uc_current_intensity)
            { // FADE OUT
                uc_current_intensity --;
                gta_leds[uc_i].uc_fade_timer = LED_FADEOUT_PERIOD;
            }

            gta_leds[uc_i].uc_current_intensity = uc_current_intensity;

        } // End of FADE IN/OUT

    } // End all leds loop

    return;
}

void leds_pwm_control(){

    uint8 uc_i;
    uint8 uc_led_port_value;
    uint8 uc_current_intensity;


    uc_led_port_value = LED_PORT;

    for (uc_i = 0; uc_i < LEDS_NUM; uc_i ++)
    { // Loop over all leds
    
        // get current led intensity
        uc_current_intensity = gta_leds[uc_i].uc_current_intensity;
        if (guc_curr_pwm == guca_pwm_intensity_table[uc_current_intensity].uc_pwm_start)
        { // Led ON
            uc_led_port_value |= gta_leds[uc_i].uc_pin_mask;
        }

        if (guc_curr_pwm == guca_pwm_intensity_table[uc_current_intensity].uc_pwm_end)
        { // Led OFF
            uc_led_port_value &= ~gta_leds[uc_i].uc_pin_mask;
        }

    } // End all leds loop

    LED_PORT = uc_led_port_value;

}

void vent_control(){

    guc_vent_timer_prescaler ++;
    if (guc_vent_timer_prescaler != VENT_TIMER_PRESCALER)
        return;

    guc_vent_timer_prescaler = 0;
    // VENT timers 0..254 - running, 255 - stoped

    if (guc_vent_mode != VENT_MODE_AUTO)
        return;

    // Proceed VENT ON timer    
    if (guc_vent_on_timer != 0xFF)
    {
        guc_vent_on_timer --;
        if (guc_vent_on_timer == 0)
        {
            // Enable ventilation
            LED_PORT |= (1<<VENT_CONTROL_PIN);
        }
    }
    // Proceed VENT OFF timer, only if ON timer stopped
    else if (guc_vent_off_timer != 0xFF)
    {
        guc_vent_off_timer --;
        if (guc_vent_off_timer == 0)
        {
            // disable ventilation
            LED_PORT &= ~(1<<VENT_CONTROL_PIN);
        }
    }

    return;
}

ISR(TIMER0_OVF_vect) {

    // --------------------------------------------------
    // --- Reload Timer
    // --------------------------------------------------
    TCNT0 = CNT_RELOAD;

    // Update global PWM counter
    guc_curr_pwm ++;
    if (guc_curr_pwm == PWM_PERIOD)
    {
        guc_curr_pwm = 0;
    }

    // --------------------------------------------------
    // --- Check switches
    // --- results in globals: guc_sw_event_hold, guc_sw_event_push
    // --------------------------------------------------
    if (guc_curr_pwm == 1)
    {
        check_switches();
    }

    // --------------------------------------------------
    // --- Switch behavior control 
    // --------------------------------------------------
    if (guc_curr_pwm == 2)
    {
        sw_behavior_control(); 
    }

    // --------------------------------------------------
    // --- Leds control 
    // --------------------------------------------------
    if (guc_curr_pwm == 0)
    { // Modify intensity on PWM period only
        leds_intensity_control();
    }

    // --------------------------------------------------
    // --- Led PWM control
    // --------------------------------------------------
    leds_pwm_control();


    // --------------------------------------------------
    // --- VENT control
    // --------------------------------------------------
    if (guc_curr_pwm == 3)
    {
        vent_control();   
    }
}



// Self write bodyguard
void dummy1(){
	uint8 uc_i;

	for (uc_i = 0; uc_i < 200; uc_i++){
		while(1){
	        __asm__ __volatile__ ("nop" ::);
	        __asm__ __volatile__ ("nop" ::);
	        __asm__ __volatile__ ("nop" ::);
	        __asm__ __volatile__ ("nop" ::);
			guc_mem_guard = 0;
	        guc_mem_guard = 0;
	        guc_mem_guard = 0;
	        guc_mem_guard = 0;
		}
	}
	guc_mem_guard = 0;
	guc_mem_guard = 0;
	guc_mem_guard = 0;
	guc_mem_guard = 0;
}
