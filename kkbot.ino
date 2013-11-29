// self balancing robot code by marcino239 github.com/marcino239 license: GPL 3.0
// Arduino framework by Marc Griffith is licensed under a Creative Commons Attribution 3.0 Unported License.
// http://creativecommons.org/licenses/by/3.0/deed.en_US
// original arduino code by msx from formu.arduino.cc
//

#include <KK2LCD.h>
#include <math.h>

//AIL, THR etc
//can also be digital outputs

const byte IN1 = 0;  //PD3 (PCINT27/TXD1/INT1)  not tested, but use Serial1 
const byte IN2 = 1;  //PD2 (PCINT26/RXD1/INT0)  interrupts good for CCPM decoding.
const byte IN3 = 2;  //PD0 (PCINT24/RXD0/T3)  tx0 is on the lcd not sure if using this would conflict with the lcd  
const byte IN4 = 3;  //PB2 (PCINT10/INT2/AIN0)
const byte IN5 = 4;  //PB0 (PCINT8/XCK0/T0)  //timer/counter0 source

//motor outputs can also be digital inputs. these also have PCINT16 to 23 Arduino interrupts not tested.
const byte OUT1 = 5;  //PC6 (TOSC1/PCINT22)   //32.768kHz crystal or custom clock source for counter (rpm sensor)
const byte OUT2 = 6;  //PC4 (TDO/PCINT20)   //JTAG 
const byte OUT3 = 7;  //PC2 (TCK/PCINT18)   //JTAG
const byte OUT4 = 8;  //PC3 (TMS/PCINT19)  //JTAG
const byte OUT5 = 9;  //PC1 (SDA/PCINT17)  //I2C      i2c not tested
const byte OUT6 = 10; //PC0 (SCL/PCINT16)  //I2C
const byte OUT7 = 11; //PC5 (TDI/PCINT21)   //JTAG
const byte OUT8 = 12; //PC7 (TOSC2/PCINT23)   //32.768kHz crystal

const byte RED_LED = 13;  //PB3 (PCINT11/OC0A/AIN1)  //same as arduino!

//important enable the internal pullups when using these as inputs
const byte BUT1 = 14;  //PB7 (PCINT15/OC3B/SCK)    PWM     pwm not tested
const byte BUT2 = 15;  //PB6 (PCINT14/OC3A/MISO)   PWM
const byte BUT3 = 16;  //PB5 (PCINT13/ICP3/MOSI)
const byte BUT4 = 17;  //PB4 (PCINT12/OC0B/SS)

const byte _BUZZER = 18;  //PB1 (PCINT9/CLKO/T1)   CLOCK output can adjust with system prescaler. (make tones) not tested

//analog reads must be done using thier channels, specifying digital pin numbers will not work in this case
const byte BATT = 3;

const byte GYR_R = 1;
const byte GYR_Y = 2;
const byte GYR_P = 4;

const byte ACC_X = 5;
const byte ACC_Y = 6;
const byte ACC_Z = 7;

//most of the hardware pwm is on the LCD, LED pins so dont bother. There is PWM on the buttons.

const int switch_release_debounce_us = 100; //milliseconds
const int switch_press_debounce_uS = 500; //microseconds

// motor control
#define MAX_MOTORS 2

boolean motorsEnable;
byte motorSpeed[ MAX_MOTORS ];

#define MOT_IN1  4       // OUT2    // PC4
#define MOT_IN2  2       // OUT3    // PC2
#define MOT_IN3  3       // OUT4    // PC3
#define MOT_IN4  1       // OUT5    // PC1
#define MOT_IN1_CH  OUT2       // OUT2    // PC4
#define MOT_IN2_CH  OUT3       // OUT3    // PC2
#define MOT_IN3_CH  OUT4       // OUT4    // PC3
#define MOT_IN4_CH  OUT5       // OUT5    // PC1

// balance control variables
float dt = 0.1;
int mydt = 20;  // ms
static float PP[2][2] = {
  { 1.0, 0.0 },
  { 0.0, 1.0 },
};

float angle = 0.0;
float q_bias = 0.0;
float rate = 0.0;
float q_m = 0.0;

float R_angle = 0.3;    // covariance matrix noise, adjusted by Kalman filter

static const float Q_angle = 0.001;
static const float Q_gyro = 0.003;

float oldAngle = 0.0;
float P = 0.0;
float I = 0.0;
float D = 0.0;

int ax_m = 0;
int ay_m = 0;
unsigned long lastread = 0;
unsigned long startmillis = 0;


#define NUMREADINGS 5 //Gyro noise filter
int readings[NUMREADINGS];
int index = 0;
int total = 0;
int average = 0;

// PID parameters
float calibration = -4.5; // read from a pot or something
float Kp = 4.5;
float Kd = 1.0;
float Ki = 1.0;

// set up
//
void setup() {
  // put your setup code here, to run once:

  pinMode(RED_LED, OUTPUT); 

  pinMode(GYR_R, INPUT);
  pinMode(GYR_Y, INPUT);
  pinMode(GYR_P, INPUT);

  pinMode(ACC_X, INPUT);
  pinMode(ACC_Y, INPUT);
  pinMode(ACC_Z, INPUT);

  pinMode(BUT1,INPUT);
  digitalWrite(BUT1, HIGH);   //enable internal pullup.

  pinMode(BUT2,INPUT);
  digitalWrite(BUT2, HIGH);

  pinMode(BUT3,INPUT);
  digitalWrite(BUT3, HIGH);

  pinMode(BUT4,INPUT);
  digitalWrite(BUT4, HIGH);

  // in order to read buttons
  analogReference(EXTERNAL); //important!!

  // init motors and set their speed to 0
  pinMode( MOT_IN1_CH, OUTPUT );
  pinMode( MOT_IN2_CH, OUTPUT );
  pinMode( MOT_IN3_CH, OUTPUT );
  pinMode( MOT_IN4_CH, OUTPUT );
  beginMotorPWM();  // freq = F_CPU / 1024  

  // display greeting
  st7565Init( Font5x7 );
  st7565SetBrightness(12);
  st7565DrawString_P( 64, 40, PSTR("Sel balancing bot") );
  st7565DrawString_P( 64, 32, PSTR(" using KK2 :)") );  
  st7565DrawString_P( 64, 24, PSTR("press any key") );  
    
  delay(1000);
  while(true)
  {
    if(!digitalRead(BUT1)||!digitalRead(BUT2)||!digitalRead(BUT3)||!digitalRead(BUT4)) {
      break;
    }
  }
}

String Str = String("hello hello");
char str[7];
int ii = 0;

// every time I write debounce code it comes out different. Complicated but I like the feel of it.
byte button4Pressed()
{
  if(!digitalRead(BUT4))
  {
    delayMicroseconds(switch_press_debounce_uS);
    if(!digitalRead(BUT4))
    {
      while(!digitalRead(BUT4))
      {
        st7565SetBrightness(12);
        st7565ClearBuffer();
        st7565SetFont( Font12x16 );
        st7565DrawString_P( 42, 26 ,  PSTR("Next") );
        st7565Refresh();
        digitalWrite(RED_LED,HIGH);
        //we could put a beep in here too.
      }
      delayMicroseconds(switch_release_debounce_us);
      digitalWrite(RED_LED,LOW);    
      return 1;
    }
  }
  return 0;
}

void analog()
{
  int aread = 0;
  //  delay(switch_release_debounce);
  st7565SetBrightness(12);
  while(true)
  {
    st7565ClearBuffer();
    st7565SetBrightness(12);
    st7565SetFont( Font12x16 );
    st7565DrawString_P( 0, 0, PSTR("Analog Read") );
    st7565SetFont( Font5x7 );

    delayMicroseconds(10);
    aread = analogRead(GYR_R);
    Str = String(aread);    
    Str.toCharArray(str,6);  
    st7565DrawString_P(10*6,16,PSTR("GYR_R "));
    st7565DrawString(16*6,16,str);
    
    delayMicroseconds(10);
    aread = analogRead(GYR_Y);
    Str = String(aread);    
    Str.toCharArray(str,6);  
    st7565DrawString_P(10*6,24,PSTR("GYR_Y "));
    st7565DrawString(16*6,24,str);
    
    delayMicroseconds(10);
    aread = analogRead(GYR_P);
    Str = String(aread);    
    Str.toCharArray(str,6);  
    st7565DrawString_P(10*6,32,PSTR("GYR_P "));
    st7565DrawString(16*6,32,str);
    
    delayMicroseconds(10);
    aread = analogRead(ACC_X);
    Str = String(aread);    
    Str.toCharArray(str,6);  
    st7565DrawString_P(0,16,PSTR("ACC_X "));
    st7565DrawString(6*6,16,str);
    
    delayMicroseconds(10);
    aread = analogRead(ACC_Y);
    Str = String(aread);    
    Str.toCharArray(str,6);  
    st7565DrawString_P(0,24,PSTR("ACC_Y "));
    st7565DrawString(6*6,24,str);
    
    delayMicroseconds(10);
    aread = analogRead(ACC_Z);
    Str = String(aread);    
    Str.toCharArray(str,6);  
    st7565DrawString_P(0,32,PSTR("ACC_Z "));
    st7565DrawString(6*6,32,str);
    
    delayMicroseconds(10);
    aread = analogRead(BATT);
    Str = String(aread);    
    Str.toCharArray(str,6);  
    st7565DrawString_P(0,48,PSTR("BATT  "));
    st7565DrawString(6*6,48,str);
    
    st7565DrawString_P( 102, 56, PSTR("Next") );
    st7565Refresh(); 
    if(button4Pressed())    
      return;    
  }
}


void loop() {
  int delta  = millis() - lastread;
  
  if( delta >= mydt ) { // sample every dt ms
    lastread = millis();
    
    total -= readings[ index ];  // substract the last gyro reading
    readings[ index ] = analogRead( GYR_P );
    total += readings[index];
    index = (index + 1) % NUMREADINGS;
    
    average = (total / NUMREADINGS) - 500;
    
    dt = ((float)delta / 1000.0);
    q_m = ((float)average) * (1500.0 / 1024.0) * PI / 180;
    
    // unbias gyro
    const float q = q_m - q_bias;  // Kalman
    const float Pdot[2 * 2] = {
      Q_angle - PP[0][1] - PP[1][0],
      - PP[1][1],
      - PP[1][1],
      Q_gyro
    };

    // store our unbias gyro estimate
    rate = q;
    
    // update our angle estimate
    //   angle += angle_dot * dt
    //         += (gyro - gyro_bias) * dt
    //         += q * dt
    
    angle += q * dt;
    
    // Update the covariance matrix
    PP[0][1] += Pdot[0] * dt;
    PP[0][1] += Pdot[1] * dt;
    PP[1][0] += Pdot[2] * dt;
    PP[1][1] += Pdot[3] * dt;
    
    // read accelerometer
    ax_m = analogRead( ACC_X );
    ay_m = analogRead( ACC_Y );

    const float angle_m = atan2( ay_m, ax_m );   //(Kalman)
    const float angle_err = angle_m - angle;     //(Kalman)
    const float C_0 = 1;                         //(Kalman)
    const float PCt_0 = C_0 * PP[0][0];          //(Kalman)
    const float PCt_1 = C_0 * PP[1][0];          //(Kalman)
    const float E =R_angle+ C_0 * PCt_0;         //(Kalman)
    const float K_0 = PCt_0 / E;                 //(Kalman)
    const float K_1 = PCt_1 / E;                 //(Kalman)
    const float t_0 = PCt_0;          /* C_0 * P[0][0] + C_1 * P[1][0] (Kalman) */

    const float t_1 = C_0 * PP[0][1]; /* + C_1 * P[1][1]  = 0 (Kalman) */

    PP[0][0] -= K_0 * t_0;                       //(Kalman)
    PP[0][1] -= K_0 * t_1;                       //(Kalman)
    PP[1][0] -= K_1 * t_0;                       //(Kalman)
    PP[1][1] -= K_1 * t_1;                       //(Kalman)
    angle += K_0 * angle_err;                    //(Kalman)
    q_bias += K_1 * angle_err;                   //(Kalman)

    float myangle=(angle * 57.2957795130823) - 90.0 + calibration;

    // calculate PID
    if( (millis() - startmillis) > 6000 ) {

      P = (myangle * Kp); // pot/10
      D = (myangle-oldAngle) * Kd; // Kd = 50 (800 at pot)
      I = I + myangle * Ki; // I * 1.0 + myangle * pot;  // 123 looks good
    
      float pid = P+I+D;
      
      // calculate motor speed - notice nonlinear function
      float motors = (pid);   // this may require replacing with non linear function to increse precision at small angles

      // cap the value
      int motorsInput;
      motorsInput = (int)motors;
      
      if( motorsInput > 1023 )
        motorsInput = 1023;
      else if( motors < -1023 )
        motorsInput = -1023;
    
      Serial1.println( motorsInput, DEC);
      if ((myangle > 80) || (myangle < -80))
      { 
        setMotorSpeed( 0, 127);    
        setMotorSpeed( 1, 127);
        I=0; // avoid accumulation of integration when stopped
      }
      else
      {
        // here will be motor speed adjustement depending on the direction
        //
        
        setMotorSpeed( 0, motorsInput );    
        setMotorSpeed( 1, motorsInput );
      }

      oldAngle = myangle;
    }
    // blink the led
    digitalWrite( RED_LED, !digitalRead( RED_LED ) );
   
    // show some debug
    Serial1.print( "Angle: " );
    Serial1.println( myangle, DEC );
  }
}

// set motor PWM
//
void setMotorSpeed( byte motor, int speed )
{
  motorSpeed[ motor ] = speed;
  
  if( motor == 0 )
    OCR1A = abs( speed );
  else
    OCR1B = abs( speed );
    
  motorsEnable = true;
}


void switchMotorsOn( boolean resetSpeed )
{
  if( resetSpeed ) {
    motorSpeed[ 0 ] = 0;
    motorSpeed[ 1 ] = 0;
  }    

  OCR1A = 0;
  OCR1B = 0;

  motorsEnable = true;
}

void switchMotorsOff()
{
  motorsEnable = false;
}

// sets up timer to perform PWM for motors
//   freq is in Hz
//
void beginMotorPWM()
{
  noInterrupts();
  
  TCCR1A = 0;
  TCCR1B = 0;

  TCCR1A |= _BV( WGM11 ) | _BV( WGM10 );
  TCCR1B |= _BV( WGM12 );                 // FastPWM 1024
  TCCR1B |= _BV( CS10 );                  // f_clk / 1 = circa 20khz with FastPWM1024

  TIMSK1 |= _BV( OCIE1A );                // enable compare interrupt
  TIMSK1 |= _BV( OCIE1B );                // enable compare interrupt
  TIMSK1 |= _BV( TOIE1 );                 // enable overflow interrupt

  OCR1A = 0;
  OCR1B = 0;

  motorSpeed[ 0 ] = 0;
  motorSpeed[ 1 ] = 0;
  
  interrupts();
}


// motors PWM routine
// MOTOR1
ISR( TIMER1_COMPA_vect )
{
  // if motors off then return;
  if( !motorsEnable || motorSpeed[ 0 ] == 0 ) {
    PORTC &= ~( _BV( MOT_IN1 ) | _BV( MOT_IN2 ) );
    return;
  }
  
  if( motorSpeed[ 0 ] > 0 )
    PORTC = (PORTC & ~(_BV( MOT_IN1 ) | _BV( MOT_IN2 ) )) | _BV( MOT_IN1 );
  else  
    PORTC = (PORTC & ~(_BV( MOT_IN1 ) | _BV( MOT_IN2 ) )) | _BV( MOT_IN2 );
  
}

// MOTOR2
ISR( TIMER1_COMPB_vect )
{
  // if motors off then return;
  if( !motorsEnable || motorSpeed[ 1 ] == 0 ) {
    PORTC &= ~( _BV( MOT_IN3 ) | _BV( MOT_IN4 ) );
    return;
  }
  
  if( motorSpeed[ 0 ] > 0 )
    PORTC = (PORTC & ~(_BV( MOT_IN3 ) | _BV( MOT_IN4 ) )) | _BV( MOT_IN3 );
  else  
    PORTC = (PORTC & ~(_BV( MOT_IN3 ) | _BV( MOT_IN4 ) )) | _BV( MOT_IN4 );
  
}

// clear all bits
ISR( TIMER1_OVF_vect )
{
  PORTC &= ~( _BV( MOT_IN1 ) | _BV( MOT_IN2 ) | _BV( MOT_IN3 ) | _BV( MOT_IN4 ) );
}

