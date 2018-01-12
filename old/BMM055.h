#ifndef _BMM_055_H_
#define _BMM_055_H_

#include <Arduino.h>
#include "i2c_multiwii.h"

//#define PROMICRO
#define SERIAL_DEBUG_MAG

// -- Defined in config.h & def.h respectively -- 
#define BMM055
#define MAG 1	

// -- Defined in def.h --
/**************************  atmega32u4 (Promicro)  ***********************************/
#if defined(PROMICRO)
  #if defined(MICROWII)
    #define A32U4ALLPINS 
  #endif
  #if !defined(TEENSY20)
    #define LEDPIN_PINMODE             //
    #define LEDPIN_TOGGLE              PIND |= 1<<5;     //switch LEDPIN state (Port D5)
    #if !defined(PROMICRO10)
      #define LEDPIN_OFF                 PORTD |= (1<<5);
      #define LEDPIN_ON                  PORTD &= ~(1<<5);  
    #else
      #define LEDPIN_OFF                PORTD &= ~(1<<5);
      #define LEDPIN_ON                 PORTD |= (1<<5);
    #endif
  #else
    #define LEDPIN_PINMODE           DDRD |= (1<<6);
    #define LEDPIN_OFF               PORTD &= ~(1<<6);
    #define LEDPIN_ON                PORTD |= (1<<6);   
    #define LEDPIN_TOGGLE            PIND |= 1<<6;     //switch LEDPIN state (Port D6)  
  #endif
  #if defined(D8BUZZER)
    #define BUZZERPIN_PINMODE          DDRB |= (1<<4);
    #if defined PILOTLAMP
      #define    PL_PIN_ON            PORTB |= 1<<4;
      #define    PL_PIN_OFF           PORTB &= ~(1<<4);
    #else
      #define BUZZERPIN_ON               PORTB |= 1<<4;
      #define BUZZERPIN_OFF              PORTB &= ~(1<<4); 
    #endif 
    
  #elif defined(A32U4ALLPINS)
    #define BUZZERPIN_PINMODE          DDRD |= (1<<4);
    #if defined PILOTLAMP
      #define    PL_PIN_ON    PORTD |= 1<<4;
      #define    PL_PIN_OFF   PORTD &= ~(1<<4);
    #else
      #define BUZZERPIN_ON               PORTD |= 1<<4;
      #define BUZZERPIN_OFF              PORTD &= ~(1<<4);  
    #endif  
  #else
    #define BUZZERPIN_PINMODE          DDRD |= (1<<3);
    #if defined PILOTLAMP
      #define    PL_PIN_ON    PORTD |= 1<<3;
      #define    PL_PIN_OFF   PORTD &= ~(1<<3);
    #else
      #define BUZZERPIN_ON               PORTD |= 1<<3;
      #define BUZZERPIN_OFF              PORTD &= ~(1<<3); 
    #endif
  #endif
  #define POWERPIN_PINMODE           //
  #define POWERPIN_ON                //
  #define POWERPIN_OFF               //
  #define I2C_PULLUPS_ENABLE         PORTD |= 1<<0; PORTD |= 1<<1;   // PIN 2&3 (SDA&SCL)
  #define I2C_PULLUPS_DISABLE        PORTD &= ~(1<<0); PORTD &= ~(1<<1);
  #define PINMODE_LCD                DDRD |= (1<<2);
  #define LCDPIN_OFF                 PORTD &= ~1;
  #define LCDPIN_ON                  PORTD |= 1;
  #define STABLEPIN_PINMODE          ;
  #define STABLEPIN_ON               ;
  #define STABLEPIN_OFF              ;
  #define PPM_PIN_INTERRUPT          DDRE &= ~(1 << 6);PORTE |= (1 << 6); EICRB |= (1 << ISC61)|(1 << ISC60); EIMSK |= (1 << INT6);
  #if !defined(SPEK_SERIAL_PORT)
    #define SPEK_SERIAL_PORT         1
  #endif
  #define USB_CDC_TX                 3
  #define USB_CDC_RX                 2
  
  //soft PWM Pins  
  #define SOFT_PWM_1_PIN_HIGH        PORTD |= 1<<4;
  #define SOFT_PWM_1_PIN_LOW         PORTD &= ~(1<<4);
  #define SOFT_PWM_2_PIN_HIGH        PORTF |= 1<<5;
  #define SOFT_PWM_2_PIN_LOW         PORTF &= ~(1<<5);
  #if !defined(A32U4ALLPINS)
    #define SOFT_PWM_3_PIN_HIGH        PORTF |= 1<<7;
    #define SOFT_PWM_3_PIN_LOW         PORTF &= ~(1<<7);
    #define SOFT_PWM_4_PIN_HIGH        PORTF |= 1<<6;
    #define SOFT_PWM_4_PIN_LOW         PORTF &= ~(1<<6);
    #define SW_PWM_P3                  A1        
    #define SW_PWM_P4                  A0
  #else
    #define SOFT_PWM_3_PIN_HIGH        PORTF |= 1<<4;
    #define SOFT_PWM_3_PIN_LOW         PORTF &= ~(1<<4);
    #define SOFT_PWM_4_PIN_HIGH        PORTF |= 1<<5;
    #define SOFT_PWM_4_PIN_LOW         PORTF &= ~(1<<5); 
    #define SW_PWM_P3                  A2        
    #define SW_PWM_P4                  A3 
  #endif
  
  // Servos
  #define SERVO_1_PINMODE   DDRF |= (1<<7); // A0
  #define SERVO_1_PIN_HIGH  PORTF|= 1<<7;
  #define SERVO_1_PIN_LOW   PORTF &= ~(1<<7);
  #define SERVO_2_PINMODE   DDRF |= (1<<6); // A1
  #define SERVO_2_PIN_HIGH  PORTF |= 1<<6;
  #define SERVO_2_PIN_LOW   PORTF &= ~(1<<6);
  #define SERVO_3_PINMODE   DDRF |= (1<<5); // A2
  #define SERVO_3_PIN_HIGH  PORTF |= 1<<5;
  #define SERVO_3_PIN_LOW   PORTF &= ~(1<<5);
  #if !defined(A32U4ALLPINS)
    #define SERVO_4_PINMODE   DDRD |= (1<<4); // 4
    #define SERVO_4_PIN_HIGH  PORTD |= 1<<4;
    #define SERVO_4_PIN_LOW   PORTD &= ~(1<<4);
  #else
    #define SERVO_4_PINMODE   DDRF |= (1<<4); // A3
    #define SERVO_4_PIN_HIGH  PORTF |= 1<<4;
    #define SERVO_4_PIN_LOW   PORTF &= ~(1<<4);  
  #endif
  #define SERVO_5_PINMODE   DDRC |= (1<<6); // 5
  #define SERVO_5_PIN_HIGH  PORTC|= 1<<6;
  #define SERVO_5_PIN_LOW   PORTC &= ~(1<<6);
  #define SERVO_6_PINMODE   DDRD |= (1<<7); // 6
  #define SERVO_6_PIN_HIGH  PORTD |= 1<<7;
  #define SERVO_6_PIN_LOW   PORTD &= ~(1<<7);
  #define SERVO_7_PINMODE   DDRB |= (1<<6); // 10
  #define SERVO_7_PIN_HIGH  PORTB |= 1<<6;
  #define SERVO_7_PIN_LOW   PORTB &= ~(1<<6);
  #define SERVO_8_PINMODE   DDRB |= (1<<5); // 9
  #define SERVO_8_PIN_HIGH  PORTB |= 1<<5;
  #define SERVO_8_PIN_LOW   PORTB &= ~(1<<5);
  
  //Standart RX
  #define THROTTLEPIN                  3
  #if defined(A32U4ALLPINS)
    #define ROLLPIN                    6
    #define PITCHPIN                   2
    #define YAWPIN                     4
    #define AUX1PIN                    5
  #else
    #define ROLLPIN                    4
    #define PITCHPIN                   5
    #define YAWPIN                     2
    #define AUX1PIN                    6
  #endif
  #define AUX2PIN                      7 
  #define AUX3PIN                      1 // unused 
  #define AUX4PIN                      0 // unused 
  #if !defined(RCAUX2PIND17)
    #define PCINT_PIN_COUNT          4
    #define PCINT_RX_BITS            (1<<1),(1<<2),(1<<3),(1<<4)
  #else
    #define PCINT_PIN_COUNT          5 // one more bit (PB0) is added in RX code
    #define PCINT_RX_BITS            (1<<1),(1<<2),(1<<3),(1<<4),(1<<0)
  #endif
  #define PCINT_RX_PORT                PORTB
  #define PCINT_RX_MASK                PCMSK0
  #define PCIR_PORT_BIT                (1<<0)
  #define RX_PC_INTERRUPT              PCINT0_vect
  #define RX_PCINT_PIN_PORT            PINB

  #if !defined(A32U4ALLPINS) && !defined(TEENSY20)
    #define V_BATPIN                  A3    // Analog PIN 3
  #elif defined(A32U4ALLPINS)
    #define V_BATPIN                  A4    // Analog PIN 4
  #else
    #define V_BATPIN                  A2    // Analog PIN 3
  #endif
  #if !defined(TEENSY20)
    #define PSENSORPIN                A2    // Analog PIN 2 
  #else
    #define PSENSORPIN                A2    // Analog PIN 2 
  #endif
#endif

// -- Defined in types.h --
enum rc {
  ROLL,
  PITCH,
  YAW,
  THROTTLE,
  AUX1,
  AUX2,
  AUX3,
  AUX4,
  AUX5,
  AUX6,
  AUX7,
  AUX8
};

typedef struct {
  int16_t  accSmooth[3];
  int16_t  gyroData[3];
  int16_t  magADC[3];
  int16_t  gyroADC[3];
  int16_t  accADC[3];
} imu_t;

typedef struct {
  uint8_t OK_TO_ARM :1 ;
  uint8_t ARMED :1 ;
  uint8_t I2C_INIT_DONE :1 ; // For i2c gps we have to now when i2c init is done, so we can update parameters to the i2cgps from eeprom (at startup it is done in setup())
  uint8_t ACC_CALIBRATED :1 ;
  uint8_t NUNCHUKDATA :1 ;
  uint8_t ANGLE_MODE :1 ;
  uint8_t HORIZON_MODE :1 ;
  uint8_t MAG_MODE :1 ;
  uint8_t BARO_MODE :1 ;
  uint8_t GPS_HOME_MODE :1 ;
  uint8_t GPS_HOLD_MODE :1 ;
  uint8_t HEADFREE_MODE :1 ;
  uint8_t PASSTHRU_MODE :1 ;
  uint8_t GPS_FIX :1 ;
  uint8_t GPS_FIX_HOME :1 ;
  uint8_t SMALL_ANGLES_25 :1 ;
  uint8_t CALIBRATE_MAG :1 ;
  uint8_t VARIO_MODE :1;
} flags_struct_t;

typedef struct {
  uint8_t currentSet;
  int16_t accZero[3];
  int16_t magZero[3];
  uint16_t flashsum;
  uint8_t checksum;      // MUST BE ON LAST POSITION OF STRUCTURE !
} global_conf_t;


void Mag_init();
void Device_Mag_getADC();


#endif // _BMM_085_H_
