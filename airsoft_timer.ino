#include <NewTone.h>
#include <Wire.h>
#include "LiquidCrystal_I2C.h"

#define DEBUG

// pins
#define ARM_BUT 2
#define DISARM_BUT 3
#define SET_BUT 4
#define BUZZER 5
#define BLINK_MOS 6
#define LAST30_MOS 7
#define EXPLODED_MOS 8
#define ARMDIS_BUT 9

// beeps frequency in Hz
#define NORMAL_BEEP 1000
#define FAST_BEEP 2000
#define EXPLOSION_BEEP 2500

#define BEEP_LENGTH 100
#define CONTACT_BOUNCE_DELAY 400
#define FAST_BEEP_SECONDS 30
#define ULTRA_FAST_BEEP_SECONDS 10

// LCD settings
#define I2C_ADDR 0x27

LiquidCrystal_I2C lcd(I2C_ADDR, 16, 2);

void setup()
{
  delay(100);
  
  #ifdef DEBUG
  Serial.begin(9600);
  #endif
  
  //setting up display
  lcd.begin();
  lcd.backlight();
  
  lcd.clear();
  lcd.home();
  
  //setting up buttons
  pinMode(ARM_BUT, INPUT_PULLUP);
  pinMode(DISARM_BUT, INPUT_PULLUP);
  pinMode(SET_BUT, INPUT_PULLUP);
  pinMode(ARMDIS_BUT, INPUT_PULLUP);

  //setting up interrupts
  attachInterrupt(digitalPinToInterrupt(ARM_BUT), arm_pressed_int, FALLING);
  attachInterrupt(digitalPinToInterrupt(DISARM_BUT), disarm_pressed_int, FALLING);

  //setting up outputs
  pinMode(BUZZER, OUTPUT);
  pinMode(BLINK_MOS, OUTPUT);
  pinMode(LAST30_MOS, OUTPUT);
  pinMode(EXPLODED_MOS, OUTPUT);

  //setup timer2
  // TIMER 2 for interrupt frequency 125 Hz:
  cli(); 
  TCNT2  = 0; 
  // set compare match register for 125 Hz increments
  TCCR2A = (1 << WGM21);
  // set 1024 prescaler
  TCCR2B = (1 << CS22) | (1 << CS21) | (1 << CS20);
  OCR2A = 125;
  sei(); // allow interrupts

  turn_on_timer();
}

//interrupt flags
volatile int arm_pres = 0;
volatile int disarm_pres = 0;
//status flags
int ticking = 0;
int minutes = 1;
int show_settings = 1;
int screen_blink = 1;
int second_tick = 1;
int force_refresh = 1;
int int_blink_mos = 0;
int int_tone = 0;
int int_ultrafast_beep = 0;
int exploded = 0;
int disarmed = 0;
//time counting
int total_seconds = 0;
int seconds_left = 0;

void loop()
{
  // handling buttons
  if(digitalRead(SET_BUT) == LOW){
    if(!ticking){
      noNewTone(BUZZER);
      turn_on_timer();

      if(show_settings){
        minutes++;
        if(minutes > 10) minutes = 1;
      }
      if(exploded){
        digitalWrite(EXPLODED_MOS, LOW);
        digitalWrite(LAST30_MOS, LOW);
        exploded = 0;
      }

      disarmed = 0;
      show_settings = 1;
      force_refresh = 1;
      
      delay(CONTACT_BOUNCE_DELAY);
    }
  }else if(digitalRead(ARMDIS_BUT) == LOW){
    if(!exploded){
      if(ticking){
        disarm_pres = 1;
      } else {
        arm_pres = 1;
      }
      delay(CONTACT_BOUNCE_DELAY);
    }
  }
  
  if(arm_pres){
    if(!ticking && !exploded){
      ticking = 1;
      if(!disarmed){
        total_seconds = 60 * minutes;
        seconds_left = total_seconds;
        disarmed = 0;
        
        lcd.home();
        lcd.clear();
        lcd.setCursor(4, 0);
        lcd.print("ARMING");
        delay(300);
        lcd.print(".");
        delay(300);
        lcd.print(".");
        delay(300);
        lcd.print(".");
        delay(300);
      }
      
      show_settings = 0;
      force_refresh = 1;
      
      TCNT2 = 0;
      second_tick = 0;
      turn_on_timer();
      
      lcd.clear();
    }
    
    arm_pres = 0;
  } else if(disarm_pres && !exploded){
    if(ticking){
      turn_off_timer();
      digitalWrite(BLINK_MOS, LOW);
      
      ticking = 0;
      force_refresh = 1;
      disarmed = 1;
    }
    
    disarm_pres = 0;
  }
  
  // showing screen
  if(force_refresh || second_tick){
    if(force_refresh){
    
      TCNT2 = 0;
    }
    
    lcd.home();

    if(show_settings){   
      lcd.clear();
      if(screen_blink){   
        lcd.setCursor(1, 0);
        lcd.print("SET: ");
        if(minutes < 10) lcd.print("0");
        lcd.print(minutes);
        lcd.print(":00");

        if(!force_refresh) screen_blink = 0;
      } else {
        screen_blink = 1;
      }
    } else {
      if(ticking && !exploded){
        // counting seconds
        if(!force_refresh) seconds_left--;
        if(seconds_left == 0) exploded = 1;

        #ifdef DEBUG
        Serial.println(seconds_left + " seconds left");
        #endif
        
        // beeping 
        if(seconds_left < ULTRA_FAST_BEEP_SECONDS){
          NewTone(BUZZER, FAST_BEEP, BEEP_LENGTH);
          // beep three times in interrupt
          int_tone = 3;
          int_ultrafast_beep = 1;
          
          // second mosfet
          digitalWrite(LAST30_MOS, HIGH);
        } else if(seconds_left < FAST_BEEP_SECONDS) {
          NewTone(BUZZER, FAST_BEEP, BEEP_LENGTH);
          // beep second time in interrupt
          int_tone = 1;

          // second mosfet
          digitalWrite(LAST30_MOS, HIGH);
        } else {
          NewTone(BUZZER, NORMAL_BEEP, BEEP_LENGTH);
        }
        // mosfet 1 blinking
        digitalWrite(BLINK_MOS, HIGH);
        // disable mosfet in interrupt
        int_blink_mos = 1;
        
        lcd.setCursor(5, 1);
        lcd.print("ARMED");
      } else if(!ticking && !exploded) {
        lcd.setCursor(4, 1);
        lcd.print("DISARMED");
      } else if (exploded){
        ticking = 0;
        NewTone(BUZZER, EXPLOSION_BEEP);
        digitalWrite(EXPLODED_MOS, HIGH);
        
        lcd.setCursor(2, 1);
        lcd.print("OUT OF TIME");
      }
      
      lcd.setCursor(5, 0);
      if(seconds_left/60 < 10) lcd.print("0");
      lcd.print(seconds_left/60);
      lcd.print(":");
      if(seconds_left%60 < 10) lcd.print("0");
      lcd.print(seconds_left%60);
    }
    
    force_refresh = 0;
    second_tick = 0;
  }
}

int timer_counter = 0;
int mosfet_blink_counter = 0;
int tone_counter = 0;
ISR(TIMER2_COMPA_vect){
  timer_counter++;
  if(timer_counter == 125){
    second_tick = 1;
    timer_counter = 0;
  }  

  // ~100 ms
  if(int_blink_mos){
    mosfet_blink_counter++;
    if(mosfet_blink_counter == 12){
      digitalWrite(BLINK_MOS, LOW);
      
      int_blink_mos = 0;
      mosfet_blink_counter = 0;
    }
  }

  // ~500 ms
  int count_goal;
  if(int_tone){
    tone_counter++;
    if(int_ultrafast_beep) {
      count_goal = 25;
    } else {
      count_goal = 50;
    }
    if(tone_counter == count_goal){
      NewTone(BUZZER, NORMAL_BEEP, BEEP_LENGTH);

      // second mosfet blink
      digitalWrite(BLINK_MOS, HIGH);
      int_blink_mos = 1;
      
      int_tone--;
      if(!int_tone) int_ultrafast_beep = 0;
      tone_counter = 0;
    }
  }
}

volatile unsigned long previous_time = 0;
void arm_pressed_int(){
  if(millis() - previous_time > CONTACT_BOUNCE_DELAY){
    arm_pres = 1;
    previous_time = millis();
  }
}

void disarm_pressed_int(){
  if(millis() - previous_time > CONTACT_BOUNCE_DELAY){
    disarm_pres = 1;
    previous_time = millis();
  }
}

void turn_on_timer(){
  TIMSK2 |= (1 << OCIE2A);
}

void turn_off_timer(){
  TIMSK2 = 0;
}


