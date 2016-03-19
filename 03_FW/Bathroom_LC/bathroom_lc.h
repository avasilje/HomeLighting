/*
Clone header file
*/
#include <stdint.h>
typedef int8_t      int8;   
typedef uint8_t     uint8;  
typedef int16_t     int16;
typedef uint16_t    uint16;
typedef int32_t     int32;
typedef uint32_t    uint32;
typedef int64_t     int64;
typedef uint64_t    uint64; 


// INFO structure for particular light source
typedef struct {

    // Static
    uint8  uc_current_intensity;
    uint8  uc_target_intensity;
    uint8  uc_fade_timer;
    uint8  uc_highlighted;    

    // Constant (loaded from EEPROM)
    uint8  uc_standby_intensity;
    uint8  uc_on_intensity;
    uint8  uc_off_intensity;    
    uint8  uc_hl_intensity;

    uint8  uc_pin_mask;
}LED_INFO;

typedef struct {
    uint8 uc_event;         // OFF_ON, ON_OFF, ON_HOLD, HOLD_OFF
    uint8 uc_prev_pin;      // 0/1 not debounced    
    uint8 uc_prev_sw;       // pressed/released deboinced
    uint8 uc_hold_timer;
    uint8 uc_debounce_timer;
    uint8 uc_pin_num;
    uint8 uc_switch_type;   // button/movement detector
}SWITCH_INFO;

typedef struct {
        uint8  uc_pwm_start;
        uint8  uc_pwm_end;
}PWM_INTENSITY_TABLE;

#define DISABLE_TIMER    TIMSK = 0            // Disable Overflow Interrupt
#define ENABLE_TIMER     TIMSK = (1<<TOIE0)   // Enable Overflow Interrupt
#define CNT_RELOAD       (0xFF - 252)          // 

#define CNT_TCCRxB       2    // Prescaler value (2->1/8)

// ---------------------------------
// --- SWITCH specific
// ---------------------------------
#define SW_PORT  PORTA
#define SW_DIR   DDRA
#define SW_PIN   PINA

#define SW_PIN0   PINA0
#define SW_PIN1   PINA1
#define SW_PIN2   PINA2
#define SW_PIN3   PINA3
#define SW_PIN4   PINA4
#define SW_PIN5   PINA5

#define SW_MASK_ALL ((1 << SW_PIN0) |\
                     (1 << SW_PIN1) |\
                     (1 << SW_PIN2) |\
                     (1 << SW_PIN3) |\
                     (1 << SW_PIN4) |\
                     (1 << SW_PIN5))

// ---------------------------------
// --- LEDS specific                                
// ---------------------------------


#define LED_PORT  PORTB
#define LED_DIR   DDRB
#define LED_PIN   PINB

#define LED_PIN0   PINB0
#define LED_PIN1   PINB1
#define LED_PIN2   PINB2
#define LED_PIN3   PINB3

#define VENT_INDICATOR_PIN  PINB1
#define VENT_CONTROL_PIN    PINB2



#define LED_MASK_ALL ((1 << LED_PIN0) |\
                      (1 << LED_PIN1) |\
                      (1 << LED_PIN2) |\
                      (1 << LED_PIN3))

#define LED_MODE_X3         0
#define LED_MODE_STANDBY    1
#define LED_MODE_ON         2
#define LED_MODE_OFF        3

#define LED_MODE_ON_TRANS      4
#define LED_MODE_OFF_TRANS     5
#define LED_MODE_STANDBY_TRANS 6

#define LED_FADEIN_PERIOD     1     // in 10ms ticks
#define LED_FADEOUT_PERIOD    1     // in 10ms ticks


#define LEDS_NUM            1
#define LED_MODES_NUM       3
#define SWITCHES_NUM        3
#define SWITCH_MODES_NUM    2
#define DETECTORS_NUM       0
#define DETECTOR_MODES_NUM  0


#define SW_ON_OFF        1
#define SW_OFF_ON        2
#define SW_ON_HOLD       3
#define SW_ON_HOLD_NEXT  4
#define SW_HOLD_OFF      5

#define DET_ON_OFF       (SW_ON_OFF       + 10)
#define DET_OFF_ON       (SW_OFF_ON       + 10)
#define DET_ON_HOLD      (SW_ON_HOLD      + 10)
#define DET_ON_HOLD_NEXT (SW_ON_HOLD_NEXT + 10)
#define DET_HOLD_OFF     (SW_HOLD_OFF     + 10)

#define SW_DEBOUNCE_TIMER     5  // in 10ms ticks
#define SW_HOLD_TIMER       100  // in 10ms ticks
#define SW_HOLD_TIMER_NEXT  150  // in 10ms ticks
#define SW_PIN_PRESSED  0 
#define SW_PIN_RELEASED 1

#define SW_TYPE_DETECT 1
#define SW_TYPE_BUTT   2


#define PWM_PERIOD 20   // 100Hz
#define PWM_PERIOD_START_END(percent) ((PWM_PERIOD - (PWM_PERIOD*percent)/100)/2), ((PWM_PERIOD + (PWM_PERIOD*percent)/100)/2)

#define VENT_TIMER_PRESCALER 100     // 1 sec
#define VENT_TIMER_ON        (50+1)  // in sec
#define VENT_TIMER_OFF       (180+1) // in sec

#define VENT_MODE_ON         1       // Forced ON
#define VENT_MODE_OFF        2       // Forced OFF
#define VENT_MODE_AUTO       3       // Depends on movement detector (switch 2)
