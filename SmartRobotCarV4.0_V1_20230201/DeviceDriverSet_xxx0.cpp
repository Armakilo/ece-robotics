/*
 * @Author: ELEGOO
 * @Date: 2019-10-22 11:59:09
 * @LastEditTime: 2020-12-29 16:07:48
 * @LastEditors: Changhua
 * @Description: Smart Robot Car V4.0
 * @FilePath: 
 */

#include "movement.h"
#include "DeviceDriverSet_xxx0.h"
//#include "PinChangeInt.h"
#include <avr/wdt.h>
static void
delay_xxx(uint16_t _ms)
{
  wdt_reset();
  for (unsigned long i = 0; i < _ms; i++)
  {
    delay(1);
  }
}
/*RBG LED
  The car has ONE WS2812 status LED (PIN_RBGLED = D4 = PD4). It used to be driven by FastLED, which
  takes ~2 KB of flash; that space is needed for the position estimate (EKF). This sends the 24 bits
  directly, using the 16 MHz AVR timing loop from Adafruit_NeoPixel (bit = 20 cycles = 1.25 us; a 1
  stays high ~0.8 us, a 0 ~0.4 us). CRGB from FastLED.h is still used for the colours.*/
static uint8_t led_brightness = 255;

static void ws2812_show(const CRGB *leds, uint8_t n)
{
  uint8_t buf[3 * NUM_LEDS + 1]; // GRB order, scaled by the brightness (+1: the loop reads one ahead)
  for (uint8_t k = 0; k < n; k++)
  {
    buf[3 * k] = ((uint16_t)leds[k].g * (led_brightness + 1)) >> 8;
    buf[3 * k + 1] = ((uint16_t)leds[k].r * (led_brightness + 1)) >> 8;
    buf[3 * k + 2] = ((uint16_t)leds[k].b * (led_brightness + 1)) >> 8;
  }
  buf[3 * n] = 0;
  volatile uint8_t *port = &PORTD;
  uint8_t *ptr = buf;
  uint8_t mask = _BV(4); // D4
  uint8_t sreg = SREG;
  cli();
  // volatile (as in Adafruit_NeoPixel): otherwise the compiler can put 'next' and 'lo' (equal at the
  // start) in the same register, and the loop then overwrites 'lo' and never drives the pin low
  volatile uint8_t hi = PORTD | mask, lo = PORTD & ~mask, next = lo, bit = 8, b = *ptr++;
  volatile uint16_t count = 3 * n;
  asm volatile(
      "head20:"                    "\n\t" // Clk  Pseudocode    (T =  0)
      "st   %a[port],  %[hi]"      "\n\t" // 2    PORT = hi     (T =  2)
      "sbrc %[byte],  7"           "\n\t" // 1-2  if(b & 128)
      "mov  %[next], %[hi]"        "\n\t" // 0-1   next = hi    (T =  4)
      "dec  %[bit]"                "\n\t" // 1    bit--         (T =  5)
      "st   %a[port],  %[next]"    "\n\t" // 2    PORT = next   (T =  7)
      "mov  %[next] ,  %[lo]"      "\n\t" // 1    next = lo     (T =  8)
      "breq nextbyte20"            "\n\t" // 1-2  if(bit == 0)
      "rol  %[byte]"               "\n\t" // 1    b <<= 1       (T = 10)
      "rjmp .+0"                   "\n\t" // 2    nop nop       (T = 12)
      "nop"                        "\n\t" // 1    nop           (T = 13)
      "st   %a[port],  %[lo]"      "\n\t" // 2    PORT = lo     (T = 15)
      "nop"                        "\n\t" // 1    nop           (T = 16)
      "rjmp .+0"                   "\n\t" // 2    nop nop       (T = 18)
      "rjmp head20"                "\n\t" // 2    -> head20 (next bit out)
      "nextbyte20:"                "\n\t" //                    (T = 10)
      "ldi  %[bit]  ,  8"          "\n\t" // 1    bit = 8       (T = 11)
      "ld   %[byte] ,  %a[ptr]+"   "\n\t" // 2    b = *ptr++    (T = 13)
      "st   %a[port], %[lo]"       "\n\t" // 2    PORT = lo     (T = 15)
      "nop"                        "\n\t" // 1    nop           (T = 16)
      "sbiw %[count], 1"           "\n\t" // 2    i--           (T = 18)
      "brne head20"                "\n"   // 2    if(i != 0) -> (next byte)
      : [port] "+e"(port), [byte] "+r"(b), [bit] "+r"(bit), [next] "+r"(next), [count] "+w"(count),
        [ptr] "+e"(ptr)
      : [hi] "r"(hi), [lo] "r"(lo));
  SREG = sreg;
  delayMicroseconds(60); // latch
}

void DeviceDriverSet_RBGLED::DeviceDriverSet_RBGLED_xxx(uint16_t Duration, uint8_t Traversal_Number, CRGB colour)
{
  if (NUM_LEDS < Traversal_Number)
  {
    Traversal_Number = NUM_LEDS;
  }
  for (int Number = 0; Number < Traversal_Number; Number++)
  {
    leds[Number] = colour;
    ws2812_show(leds, NUM_LEDS);
    delay_xxx(Duration);
  }
}
void DeviceDriverSet_RBGLED::DeviceDriverSet_RBGLED_Init(uint8_t set_Brightness)
{
  pinMode(PIN_RBGLED, OUTPUT);
  digitalWrite(PIN_RBGLED, LOW);
  led_brightness = set_Brightness;
}
#if _Test_DeviceDriverSet
void DeviceDriverSet_RBGLED::DeviceDriverSet_RBGLED_Test(void)
{
  leds[0] = CRGB::White;
  ws2812_show(leds, NUM_LEDS);
  delay_xxx(50);
  DeviceDriverSet_RBGLED_xxx(50 /*Duration*/, 5 /*Traversal_Number*/, CRGB::Black);
}
#endif

void DeviceDriverSet_RBGLED::DeviceDriverSet_RBGLED_Color(uint8_t LED_s, uint8_t r, uint8_t g, uint8_t b)
{
  if (LED_s > NUM_LEDS)
    return;
  if (LED_s == NUM_LEDS)
  {
    for (uint8_t k = 0; k < NUM_LEDS; k++)
      leds[k] = CRGB(r, g, b);
  }
  else
  {
    leds[LED_s] = CRGB(r, g, b);
  }
  ws2812_show(leds, NUM_LEDS);
}

/*Key*/
uint8_t DeviceDriverSet_Key::keyValue = 0;

static void attachPinChangeInterrupt_GetKeyValue(void)
{
  DeviceDriverSet_Key Key;
  static uint32_t keyValue_time = 0;
  static uint8_t keyValue_temp = 0;
  if ((millis() - keyValue_time) > 500)
  {
    keyValue_temp++;
    keyValue_time = millis();
    if (keyValue_temp > keyValue_Max)
    {
      keyValue_temp = 0;
    }
    Key.keyValue = keyValue_temp;
  }
}
void DeviceDriverSet_Key::DeviceDriverSet_Key_Init(void)
{
  pinMode(PIN_Key, INPUT_PULLUP);
  //attachPinChangeInterrupt(PIN_Key, attachPinChangeInterrupt_GetKeyValue, FALLING);
  attachInterrupt(0, attachPinChangeInterrupt_GetKeyValue, FALLING);
}

#if _Test_DeviceDriverSet
void DeviceDriverSet_Key::DeviceDriverSet_Key_Test(void)
{
  Serial.println(DeviceDriverSet_Key::keyValue);
}
#endif

void DeviceDriverSet_Key::DeviceDriverSet_key_Get(uint8_t *get_keyValue)
{
  *get_keyValue = keyValue;
}

/*ITR20001 Detection*/
bool DeviceDriverSet_ITR20001::DeviceDriverSet_ITR20001_Init(void)
{
  pinMode(PIN_ITR20001xxxL, INPUT);
  pinMode(PIN_ITR20001xxxM, INPUT);
  pinMode(PIN_ITR20001xxxR, INPUT);
  return false;
}
int DeviceDriverSet_ITR20001::DeviceDriverSet_ITR20001_getAnaloguexxx_L(void)
{
  return analogRead(PIN_ITR20001xxxL);
}
int DeviceDriverSet_ITR20001::DeviceDriverSet_ITR20001_getAnaloguexxx_M(void)
{
  return analogRead(PIN_ITR20001xxxM);
}
int DeviceDriverSet_ITR20001::DeviceDriverSet_ITR20001_getAnaloguexxx_R(void)
{
  return analogRead(PIN_ITR20001xxxR);
}
#if _Test_DeviceDriverSet
void DeviceDriverSet_ITR20001::DeviceDriverSet_ITR20001_Test(void)
{
  Serial.print("\tL=");
  Serial.print(analogRead(PIN_ITR20001xxxL));

  Serial.print("\tM=");
  Serial.print(analogRead(PIN_ITR20001xxxM));

  Serial.print("\tR=");
  Serial.println(analogRead(PIN_ITR20001xxxR));
}
#endif

/*Voltage Detection*/
void DeviceDriverSet_Voltage::DeviceDriverSet_Voltage_Init(void)
{
  pinMode(PIN_Voltage, INPUT);
  //analogReference(INTERNAL);
}
float DeviceDriverSet_Voltage::DeviceDriverSet_Voltage_getAnalogue(void)
{
  //float Voltage = ((analogRead(PIN_Voltage) * 5.00 / 1024) * 7.67); //7.66666=((10 + 1.50) / 1.50)
  float Voltage = (analogRead(PIN_Voltage) * 0.0375);
  Voltage = Voltage + (Voltage * 0.08); //Compensation 8%
  //return (analogRead(PIN_Voltage) * 5.00 / 1024) * ((10 + 1.50) / 1.50); //Read voltage value
  return Voltage;
}

#if _Test_DeviceDriverSet
void DeviceDriverSet_Voltage::DeviceDriverSet_Voltage_Test(void)
{
  //float Voltage = ((analogRead(PIN_Voltage) * 5.00 / 1024) * 7.67); //7.66666=((10 + 1.50) / 1.50)
  float Voltage = (analogRead(PIN_Voltage) * 0.0375); //7.66666=((10 + 1.50) / 1.50)
  Voltage = Voltage + (Voltage * 0.08);               //Compensation 8%
  //Serial.println(analogRead(PIN_Voltage) * 4.97 / 1024);
  Serial.println(Voltage);
}
#endif
/*Motor control*/
void DeviceDriverSet_Motor::DeviceDriverSet_Motor_Init(void)
{
  pinMode(PIN_Motor_PWMA, OUTPUT);
  pinMode(PIN_Motor_PWMB, OUTPUT);
  pinMode(PIN_Motor_AIN_1, OUTPUT);
  pinMode(PIN_Motor_BIN_1, OUTPUT);
  pinMode(PIN_Motor_STBY, OUTPUT);
}

#if _Test_DeviceDriverSet
void DeviceDriverSet_Motor::DeviceDriverSet_Motor_Test(void)
{
  //A...Right
  //B...Left
  digitalWrite(PIN_Motor_STBY, HIGH);

  digitalWrite(PIN_Motor_AIN_1, HIGH);
  analogWrite(PIN_Motor_PWMA, 100);
  digitalWrite(PIN_Motor_BIN_1, HIGH);
  analogWrite(PIN_Motor_PWMB, 100);
  delay_xxx(1000);

  digitalWrite(PIN_Motor_STBY, LOW);
  delay_xxx(1000);
  digitalWrite(PIN_Motor_STBY, HIGH);
  digitalWrite(PIN_Motor_AIN_1, LOW);
  analogWrite(PIN_Motor_PWMA, 100);
  digitalWrite(PIN_Motor_BIN_1, LOW);
  analogWrite(PIN_Motor_PWMB, 100);

  delay_xxx(1000);
}
#endif

/*
 Motor_control：AB / movement direction and speed
*/
// Last PWM sent to each side, + = forward, - = backward. Read by the kinematic model
// (Kinematics_Update in ApplicationFunctionSet_xxx0.cpp). A = RIGHT side, B = LEFT side.
int16_t g_cmdR = 0;
int16_t g_cmdL = 0;

void DeviceDriverSet_Motor::DeviceDriverSet_Motor_control(boolean direction_A, uint8_t speed_A, //Group A motor parameters
                                                          boolean direction_B, uint8_t speed_B, //Group B motor parameters
                                                          boolean controlED                     //AB enable setting (true)
                                                          )                                     //Motor control
{
  // Record the command for the kinematic model. Note: direction_* is a boolean, so
  // direction_void (3) arrives as "true"; every stop in this code also passes speed 0,
  // so the recorded value is 0 in that case either way.
  if (controlED == control_enable)
  {
    g_cmdR = direction_A ? (int16_t)speed_A : -(int16_t)speed_A;
    g_cmdL = direction_B ? (int16_t)speed_B : -(int16_t)speed_B;
  }
  else
  {
    g_cmdR = 0;
    g_cmdL = 0;
  }

  if (controlED == control_enable) //Enable motot control？
  {
    digitalWrite(PIN_Motor_STBY, HIGH);
    { //A...Right

      switch (direction_A) //movement direction control
      {
      case direction_just:
        digitalWrite(PIN_Motor_AIN_1, HIGH);
        analogWrite(PIN_Motor_PWMA, speed_A);
        break;
      case direction_back:

        digitalWrite(PIN_Motor_AIN_1, LOW);
        analogWrite(PIN_Motor_PWMA, speed_A);
        break;
      case direction_void:
        analogWrite(PIN_Motor_PWMA, 0);
        digitalWrite(PIN_Motor_STBY, LOW);
        break;
      default:
        analogWrite(PIN_Motor_PWMA, 0);
        digitalWrite(PIN_Motor_STBY, LOW);
        break;
      }
    }

    { //B...Left
      switch (direction_B)
      {
      case direction_just:
        digitalWrite(PIN_Motor_BIN_1, HIGH);

        analogWrite(PIN_Motor_PWMB, speed_B);
        break;
      case direction_back:
        digitalWrite(PIN_Motor_BIN_1, LOW);
        analogWrite(PIN_Motor_PWMB, speed_B);
        break;
      case direction_void:
        analogWrite(PIN_Motor_PWMB, 0);
        digitalWrite(PIN_Motor_STBY, LOW);
        break;
      default:
        analogWrite(PIN_Motor_PWMB, 0);
        digitalWrite(PIN_Motor_STBY, LOW);
        break;
      }
    }
  }
  else
  {
    digitalWrite(PIN_Motor_STBY, LOW);
    return;
  }
}

/*ULTRASONIC*/
//#include <NewPing.h>
// NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
void DeviceDriverSet_ULTRASONIC::DeviceDriverSet_ULTRASONIC_Init(void)
{
  pinMode(ECHO_PIN, INPUT); //Ultrasonic module initialization
  pinMode(TRIG_PIN, OUTPUT);
}
void DeviceDriverSet_ULTRASONIC::DeviceDriverSet_ULTRASONIC_Get(uint16_t *ULTRASONIC_Get /*out*/)
{
  unsigned int tempda_x = 0;
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  tempda_x = ((unsigned int)pulseIn(ECHO_PIN, HIGH, 10000) / 58);
  // *ULTRASONIC_Get = tempda_x;

  if (tempda_x > 150)
  {
    *ULTRASONIC_Get = 150;
  }
  else
  {
    *ULTRASONIC_Get = tempda_x;
  }
  // sonar.ping() / US_ROUNDTRIP_CM; // Send ping, get ping time in microseconds (uS).
}

#if _Test_DeviceDriverSet
void DeviceDriverSet_ULTRASONIC::DeviceDriverSet_ULTRASONIC_Test(void)
{

  unsigned int tempda = 0;
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  tempda = ((unsigned int)pulseIn(ECHO_PIN, HIGH) / 58);

  // if (tempda_x > 50)
  // {
  //   tempda_x = 50;
  // }

  // // return tempda;
  // return tempda_x;

  Serial.print("ULTRASONIC=");
  Serial.print(tempda); // Convert ping time to distance and print result (0 = outside set distance range, no ping echo)
  Serial.println("cm");
}

#endif

/*Servo*/

Servo myservo; // create servo object to control a servo
void DeviceDriverSet_Servo::DeviceDriverSet_Servo_Init(unsigned int Position_angle)
{
  myservo.attach(PIN_Servo_z, 500, 2400); //500: 0 degree  2400: 180 degree
  myservo.attach(PIN_Servo_z);
  myservo.write(Position_angle); //sets the servo position according to the 90（middle）
  delay_xxx(500);

  myservo.attach(PIN_Servo_y, 500, 2400); //500: 0 degree  2400: 180 degree
  myservo.attach(PIN_Servo_y);
  myservo.write(Position_angle); //sets the servo position according to the 90（middle）
  delay_xxx(500);
  myservo.detach();
}
#if _Test_DeviceDriverSet
void DeviceDriverSet_Servo::DeviceDriverSet_Servo_Test(void)
{
  for (;;)
  {
    myservo.attach(PIN_Servo_z);
    myservo.write(180);
    delay_xxx(500);
    myservo.write(0);
    delay_xxx(500);
  }

  // for (uint8_t i = 0; i < 6; i++)
  // {
  //   myservo.write(30 * i);
  //   delay(500);
  // }
  // for (uint8_t i = 6; i > 0; i--)
  // {
  //   myservo.write(30 * i);
  //   delay(500);
  // }

  // myservo.attach(PIN_Servo_y);

  // for (uint8_t i = 0; i < 6; i++)
  // {
  //   myservo.write(30 * i);
  //   delay(500);
  // }
  // for (uint8_t i = 6; i > 0; i--)
  // {
  //   myservo.write(30 * i);
  //   delay(500);
  // }
}
#endif

/*0.17sec/60degree(4.8v)*/
void DeviceDriverSet_Servo::DeviceDriverSet_Servo_control(unsigned int Position_angle)
{
  myservo.attach(PIN_Servo_z);
  myservo.write(Position_angle);
  delay_kinematics(450);
  myservo.detach();
}
//Servo motor control:Servo motor number and position angle
void DeviceDriverSet_Servo::DeviceDriverSet_Servo_controls(uint8_t Servo, unsigned int Position_angle)
{
  if (Servo == 1 || Servo == 3) //Servo_z
  {
    if (Position_angle <= 1) //minimum angle control
    {
      Position_angle = 1;
    }
    if (Position_angle >= 17) //maximum angle control
    {
      Position_angle = 17;
    }
    myservo.attach(PIN_Servo_z);
    myservo.write(10 * Position_angle);
    delay_xxx(500);
  }
  if (Servo == 2 || Servo == 3) //Servo_y
  {

    if (Position_angle <= 3) //minimum angle control
    {
      Position_angle = 3;
    }
    if (Position_angle >= 11) //maximum angle control
    {
      Position_angle = 11;
    }
    myservo.attach(PIN_Servo_y);
    myservo.write(10 * Position_angle);
    delay_xxx(500);
  }
  myservo.detach();
}

/*IRrecv*/
IRrecv irrecv(RECV_PIN); //  Create an infrared receive drive object
decode_results results;  //  Create decoding object
void DeviceDriverSet_IRrecv::DeviceDriverSet_IRrecv_Init(void)
{
  irrecv.enableIRIn(); //Enable infrared communication NEC
}
bool DeviceDriverSet_IRrecv::DeviceDriverSet_IRrecv_Get(uint8_t *IRrecv_Get /*out*/)
{
  if (irrecv.decode(&results))
  {
    IR_PreMillis = millis();
    switch (results.value)
    {
    case /* constant-expression */ aRECV_upper:
    case /* constant-expression */ bRECV_upper:
      /* code */ *IRrecv_Get = 1;
      break;
    case /* constant-expression */ aRECV_lower:
    case /* constant-expression */ bRECV_lower:
      /* code */ *IRrecv_Get = 2;
      break;
    case /* constant-expression */ aRECV_Left:
    case /* constant-expression */ bRECV_Left:
      /* code */ *IRrecv_Get = 3;
      break;
    case /* constant-expression */ aRECV_right:
    case /* constant-expression */ bRECV_right:
      /* code */ *IRrecv_Get = 4;
      break;
    case /* constant-expression */ aRECV_ok:
    case /* constant-expression */ bRECV_ok:
      /* code */ *IRrecv_Get = 5;
      break;

    case /* constant-expression */ aRECV_1:
    case /* constant-expression */ bRECV_1:
      /* code */ *IRrecv_Get = 6;
      break;
    case /* constant-expression */ aRECV_2:
    case /* constant-expression */ bRECV_2:
      /* code */ *IRrecv_Get = 7;
      break;
    case /* constant-expression */ aRECV_3:
    case /* constant-expression */ bRECV_3:
      /* code */ *IRrecv_Get = 8;
      break;
    case /* constant-expression */ aRECV_4:
    case /* constant-expression */ bRECV_4:
      /* code */ *IRrecv_Get = 9;
      break;
    case /* constant-expression */ aRECV_5:
    case /* constant-expression */ bRECV_5:
      /* code */ *IRrecv_Get = 10;
      break;
    case /* constant-expression */ aRECV_6:
    case /* constant-expression */ bRECV_6:
      /* code */ *IRrecv_Get = 11;
      break;
    case /* constant-expression */ aRECV_7:
    case /* constant-expression */ bRECV_7:
      /* code */ *IRrecv_Get = 12;
      break;
    case /* constant-expression */ aRECV_8:
    case /* constant-expression */ bRECV_8:
      /* code */ *IRrecv_Get = 13;
      break;
    case /* constant-expression */ aRECV_9:
    case /* constant-expression */ bRECV_9:
      /* code */ *IRrecv_Get = 14;
      break;
    default:
      // *IRrecv_Get = 5;
      irrecv.resume();
      return false;
      break;
    }
    irrecv.resume();
    return true;
  }
  else
  {
    return false;
  }
}

#if _Test_DeviceDriverSet
void DeviceDriverSet_IRrecv::DeviceDriverSet_IRrecv_Test(void)
{
  if (irrecv.decode(&results))
  {
    Serial.print("IRrecv_Test:");
    Serial.println(results.value);
    irrecv.resume();
  }
}
#endif
