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
    uint8  uc_current_intensity;        // Is the index in intensity table
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
        uint8  uc_ocr;          // Valud 
}PWM_INTENSITY_TABLE;

#define DISABLE_TIMER    TIMSK = 0            // Disable Overflow Interrupt
#define ENABLE_TIMER     TIMSK = (1<<TOIE0)   // Enable Overflow Interrupt
#define CNT_RELOAD       (0xFF - 63)          // @8MHz, @1/64 => 504usec

#define CNT_TCCRxB       3    // Prescaler value (2->1/8; 3->1/64)

// Fast PWM
// OC2 active. Set on Match, clear on TOP
// No prescaler
#define ENABLE_FAST_PWM2 \
            TCCR2 = (1 << WGM20) |\
                    (1 << WGM21) |\
                    (1 << COM20) |\
                    (1 << COM21) |\
                    (1 << CS20 )

#define DISABLE_FAST_PWM2  TCCR2 = 0

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

#define SW_MASK_ALL \
    ((1 << SW_PIN0)|\
     (1 << SW_PIN1)|\
     (1 << SW_PIN2)|\
     (1 << SW_PIN3)|\
     (1 << SW_PIN4)|\
     (1 << SW_PIN5))

// ---------------------------------
// --- VENT specific                                
// ---------------------------------
#define VENT_CONTROL_PIN    PINB1 
#define VENT_PORT  PORTB
#define VENT_DIR   DDRB
#define VENT_PIN   PINB
    
// ---------------------------------
// --- LEDS specific                                
// ---------------------------------

#define LED_PORT  PORTD
#define LED_DIR   DDRD
#define LED_PIN   PIND

#define LED_PIN0   PIND7

#define LED_MASK_ALL (1 << LED_PIN0)

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
#define SWITCHES_NUM        2
#define SWITCH_MODES_NUM    2
#define DETECTORS_NUM       0
#define DETECTOR_MODES_NUM  0


#define VENT_MODE_ON         1       // Forced ON
#define VENT_MODE_OFF        2       // Forced OFF
#define VENT_MODE_AUTO       3       // Depends on movement detector (switch 2)


#define SW_ON_OFF   1
#define SW_OFF_ON   2
#define SW_ON_HOLD  3
#define SW_HOLD_OFF 4

#define DET_ON_OFF   11
#define DET_OFF_ON   12
#define DET_ON_HOLD  13
#define DET_HOLD_OFF 14

#define SW_DEBOUNCE_TIMER     5  // in 10ms ticks
#define SW_HOLD_TIMER       100  // in 10ms ticks
#define SW_HOLD_TIMER_NEXT  150  // in 10ms ticks
#define SW_PIN_PRESSED  0 
#define SW_PIN_RELEASED 1

#define SW_TYPE_BUTT   2

#define MAX_FASTPWM 255
#define PWM_PERIOD 20   // 100Hz    10ms. Precision 504usec
//                                                 Software PWM Start                          Software PWM End                 Fast PWM OCR value
#define PWM_PERIOD_START_END(percent) ((PWM_PERIOD - (PWM_PERIOD*percent)/100)/2), ((PWM_PERIOD + (PWM_PERIOD*percent)/100)/2), (MAX_FASTPWM - (MAX_FASTPWM*(percent))/100)
