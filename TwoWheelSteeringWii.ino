#include "WiiRemote.h"

#include <MemoryFree.h>
#include <Servo.h>

#define PIN_STEERING_SIGNAL        2
#define PIN_ESC_SIGNAL             4

#define PIN_HEAD_LIGHT_SIGNAL     14
#define PIN_BACK_LIGHT_SIGNAL     15

#define PIN_FIRE_SIGNAL           17

#define PIN_STEERING_SELECT       16

#define SERIAL_DEBUG               0    // 0: active mode, 1: serial debug mode

enum eAngle
{
  STEERING_ANGLE_MAX           = 165,   // to right
  STEERING_ANGLE_CENTER        = 90,
  STEERING_ANGLE_MIN           = 15,    // to left
  STEERING_ANGLE_STEP          = 5,

  STEERING_ANGLE_MAX_INVERT    = 165,   // to right
  STEERING_ANGLE_CENTER_INVERT = 90,
  STEERING_ANGLE_MIN_INVERT    = 15,    // to left

  THROTTLE_ANGLE_MAX           = 160,   // 80,
  THROTTLE_ANGLE_CENTER        = 90,
  THROTTLE_ANGLE_MIN           = 10,
};

enum eServoPulse
{
  SERVO_PULSE_MAX              = 2400,  // to left
  SERVO_PULSE_NEUTRAL          = 1550,  // 1500 Futaba compatible, 1.55msec
  SERVO_PULSE_MIN              = 600,   // to right

  SERVO_PULSE_MAX_INVERT       = 600,   // to right
  SERVO_PULSE_NEUTRAL_INVERT   = 1450,  // 1460 Futaba compatible, 1.55msec
  SERVO_PULSE_MIN_INVERT       = 2400,  // to left
};

enum eESCPulse
{
/*
 *  Futaba timing
 *
 *  0us     1072us         1522us          1922us
 *   +---------*------------+-*-+-------------*
 *   |   n/a   |   forwad   |d|d|   Reverse   |
 *   +---------*------------+-*-+-------------*
 *          Max Forwad     Neutral         Max Reverse
 *
 *   d: dead zone, +10us and -10us
 */
  ESC_PULSE_NEUTRAL     = 1522,
  ESC_PULSE_BRAKE       = 1600,
  ESC_PULSE_FWD_MAX     = 800,   //1200, // 1072
  ESC_PULSE_FWD_MIN     = 1510,
  ESC_PULSE_FWD_3RD     = (ESC_PULSE_FWD_MIN - 240),
  ESC_PULSE_FWD_2ND     = (ESC_PULSE_FWD_MIN - 160),
  ESC_PULSE_FWD_1ST     = (ESC_PULSE_FWD_MIN - 80),

  ESC_PULSE_REV_MAX     = 1700,  // 1922
  ESC_PULSE_REV_FIX     = 1650,
  ESC_PULSE_REV_MIN     = 1600,
};

enum eGear
{
  GEAR_1ST = 1,
  GEAR_2ND = 2,
  GEAR_3RD = 3,
};

WiiRemote wiiremote;

Servo SteeringServo;
Servo ESC;

void setup()
{
#if SERIAL_DEBUG
  Serial.begin(115200);

  Serial.print("\r\nfreeMemory() reports: ");
  Serial.print(freeMemory(), DEC);
  Serial.println("Serial connect...");
#endif

  SteeringServo.attach(PIN_STEERING_SIGNAL);
  SteeringServo.writeMicroseconds(SERVO_PULSE_NEUTRAL);

  ESC.attach(PIN_ESC_SIGNAL);
  ESC.writeMicroseconds(ESC_PULSE_NEUTRAL);

  pinMode(PIN_HEAD_LIGHT_SIGNAL, OUTPUT);
  pinMode(PIN_BACK_LIGHT_SIGNAL, OUTPUT);
  digitalWrite(PIN_HEAD_LIGHT_SIGNAL, LOW);
  digitalWrite(PIN_BACK_LIGHT_SIGNAL, LOW);

  pinMode(PIN_FIRE_SIGNAL, OUTPUT);
  digitalWrite(PIN_FIRE_SIGNAL, LOW);

  pinMode(PIN_STEERING_SELECT, INPUT);
  digitalWrite(PIN_STEERING_SELECT, LOW);

  wiiremote.init();

  /*
  unsigned char wiiremote_bdaddr[6] = {0x00, 0x1e, 0x35, 0xda, 0x48, 0xbc};
  wiiremote.setBDAddress(wiiremote_bdaddr, 6);
  wiiremote.setBDAddressMode(BD_ADDR_FIXED);
  */

#if SERIAL_DEBUG
  Serial.println("Wiimote connecting...");
  Serial.println("Please press 1 button and 2 button simultaneously");
#endif
}

void loop()
{
  wiiremote.task(&myapp);
}

int steering_angle          = STEERING_ANGLE_CENTER;
int steering_angle_invert   = STEERING_ANGLE_CENTER_INVERT;
int old_steering_angle      = STEERING_ANGLE_CENTER;
bool analog_throttle        = false;  // false = use "One" button as throttle
int throttle_angle          = THROTTLE_ANGLE_CENTER;
int gear                    = GEAR_1ST;
int pulse_steering;
int pulse_esc;
bool fire                   = false;                  // fire
bool head_light             = false;                  // head light
bool back_light             = false;                  // back light

void myapp(void)
{
#if SERIAL_DEBUG
  Serial.print("\r\n");
#endif

  /* Steering */
  steering_angle = getSteeringAngle();
  steering_angle_invert = getSteeringAngleInvert();
  if (digitalRead(PIN_STEERING_SELECT) == HIGH) {
    pulse_steering = map(steering_angle,
                         STEERING_ANGLE_MIN, STEERING_ANGLE_MAX,
                         SERVO_PULSE_MAX, SERVO_PULSE_MIN);
    SteeringServo.writeMicroseconds(pulse_steering);
  } else {
    pulse_steering = map(steering_angle_invert,
                         STEERING_ANGLE_MIN_INVERT, STEERING_ANGLE_MAX_INVERT,
                         SERVO_PULSE_MAX_INVERT, SERVO_PULSE_MIN_INVERT);
    SteeringServo.writeMicroseconds(pulse_steering);
  }
  if (wiiremote.buttonPressed(WIIREMOTE_UP)) {
    steering_angle = STEERING_ANGLE_MIN;
    steering_angle_invert = STEERING_ANGLE_MIN_INVERT;
    if (digitalRead(PIN_STEERING_SELECT) == HIGH) {
      pulse_steering = map(steering_angle, STEERING_ANGLE_MIN, STEERING_ANGLE_MAX, SERVO_PULSE_MAX, SERVO_PULSE_MIN);
      SteeringServo.writeMicroseconds(pulse_steering);
    } else {
      pulse_steering = map(steering_angle_invert, STEERING_ANGLE_MIN_INVERT, STEERING_ANGLE_MAX_INVERT, SERVO_PULSE_MAX_INVERT, SERVO_PULSE_MIN_INVERT);
      SteeringServo.writeMicroseconds(pulse_steering);
    }
  } else if (wiiremote.buttonPressed(WIIREMOTE_DOWN)) {
    steering_angle = STEERING_ANGLE_MAX;
    steering_angle_invert = STEERING_ANGLE_MAX_INVERT;
    if (digitalRead(PIN_STEERING_SELECT) == HIGH) {
      pulse_steering = map(steering_angle, STEERING_ANGLE_MIN, STEERING_ANGLE_MAX, SERVO_PULSE_MAX, SERVO_PULSE_MIN);
      SteeringServo.writeMicroseconds(pulse_steering);
    } else {
      pulse_steering = map(steering_angle_invert, STEERING_ANGLE_MIN_INVERT, STEERING_ANGLE_MAX_INVERT, SERVO_PULSE_MAX_INVERT, SERVO_PULSE_MIN_INVERT);
      SteeringServo.writeMicroseconds(pulse_steering);
    }
  }

#if SERIAL_DEBUG
  Serial.print("\tServo=");
  Serial.print(pulse_steering);
#endif

  /* Brake and Throttle */
  if (wiiremote.buttonPressed(WIIREMOTE_ONE)) {
    if (pulse_esc < ESC_PULSE_NEUTRAL) {
      // moving forward before press "One"
      brake();
      pulse_esc = ESC_PULSE_NEUTRAL;
    } else {
      // while stopping or moving backward, keep moving backward
      pulse_esc = ESC_PULSE_REV_FIX;
    }
  } else {
    if (analog_throttle) {
      throttle_angle = getThrottleAngle();
      pulse_esc = map(throttle_angle,
                      THROTTLE_ANGLE_MIN, THROTTLE_ANGLE_MAX,
                      ESC_PULSE_FWD_MIN, ESC_PULSE_FWD_MAX);
    } else if (wiiremote.buttonPressed(WIIREMOTE_TWO)) {
      switch (gear) {
       case GEAR_1ST:
        pulse_esc = ESC_PULSE_FWD_1ST;
        break;
       case GEAR_2ND:
        pulse_esc = ESC_PULSE_FWD_2ND;
        break;
       case GEAR_3RD:
        pulse_esc = ESC_PULSE_FWD_3RD;
        break;
       default:
        pulse_esc = ESC_PULSE_NEUTRAL;
        break;
      }
    } else {
      pulse_esc = ESC_PULSE_NEUTRAL;
    }
  }
  ESC.writeMicroseconds(pulse_esc);
  //delay(15);
#if SERIAL_DEBUG
  Serial.print("\tESC=");
  Serial.print(pulse_esc);
#endif

  /* Throttle mode */
  if (wiiremote.buttonClicked(WIIREMOTE_HOME)) {
    analog_throttle = !analog_throttle;
    if (analog_throttle) {
      wiiremote.setLED(WIIREMOTE_LED4); // analog mode
    } else {
      wiiremote.setLED(WIIREMOTE_LED1); // fixed mode, 1st gear
      gear = GEAR_1ST;
    }
  }

  /* Shift up or down */
  if (!analog_throttle) {
    if (wiiremote.buttonClicked(WIIREMOTE_RIGHT)) {
      shiftUp();
    } else if (wiiremote.buttonClicked(WIIREMOTE_LEFT)) {
      shiftDown();
    }
  }

  /* Fire */
  if (wiiremote.buttonPressed(WIIREMOTE_A)) {
    digitalWrite(PIN_FIRE_SIGNAL, HIGH);
  } else {
    digitalWrite(PIN_FIRE_SIGNAL, LOW);
  }

  /* Head light LED */
  if (wiiremote.buttonClicked(WIIREMOTE_PLUS)) {
    head_light = !head_light;
    if (head_light) {
      digitalWrite(PIN_HEAD_LIGHT_SIGNAL, HIGH);
    } else {
      digitalWrite(PIN_HEAD_LIGHT_SIGNAL, LOW);
    }
  }

  /* Back light LED */
  if (wiiremote.buttonClicked(WIIREMOTE_MINUS)) {
    back_light = !back_light;
    if (back_light) {
      digitalWrite(PIN_BACK_LIGHT_SIGNAL, HIGH);
    } else {
      digitalWrite(PIN_BACK_LIGHT_SIGNAL, LOW);
    }
  }

} // myapp

int getSteeringAngle(void)
{
  double rad;
  int deg;

  rad = acos((double) wiiremote.Report.Accel.Y);
  deg = (int) (rad * 180.0 / PI);

  /* clipping */
  if (deg > STEERING_ANGLE_MAX) { deg = STEERING_ANGLE_MAX; }
  if (deg < STEERING_ANGLE_MIN) { deg = STEERING_ANGLE_MIN; }

  return deg;
}

int getSteeringAngleInvert(void)
{
  double rad;
  int deg;

  rad = acos((double) wiiremote.Report.Accel.Y);
  deg = (int) (rad * 180.0 / PI);

  /* clipping */
  if (deg > STEERING_ANGLE_MAX_INVERT) { deg = STEERING_ANGLE_MAX_INVERT; }
  if (deg < STEERING_ANGLE_MIN_INVERT) { deg = STEERING_ANGLE_MIN_INVERT; }

  return deg;
}

int getThrottleAngle(void)
{
  double rad;
  double compensate_z;
  int deg;

  rad = asin((double) wiiremote.Report.Accel.Y);
  compensate_z = (double) wiiremote.Report.Accel.Z / cos(rad);

  rad = asin(compensate_z);
  deg = (int) (rad * 180.0 / PI);

  /* clipping */
  if (deg > THROTTLE_ANGLE_MAX) { deg = THROTTLE_ANGLE_MAX; }
  if (deg < THROTTLE_ANGLE_MIN) { deg = THROTTLE_ANGLE_MIN; }

  return deg;
}

inline void brake(void)
{
  ESC.writeMicroseconds(ESC_PULSE_BRAKE);
  delay(15);
  ESC.writeMicroseconds(ESC_PULSE_NEUTRAL);
  delay(15);
}

inline void shiftUp(void)
{
  if (gear < GEAR_3RD) {
    gear++;
    wiiremote.setLED( (WIIREMOTE_LED1 << (gear - GEAR_1ST)) );
  }
}

inline void shiftDown(void)
{
  if (gear > GEAR_1ST) {
    gear--;
    wiiremote.setLED( (WIIREMOTE_LED1 << (gear - GEAR_1ST)) );
  }
}

