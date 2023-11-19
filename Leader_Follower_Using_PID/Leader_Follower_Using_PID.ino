#include "motors.h"
#include "linesensor.h"
#include "encoders.h"
#include "kinematics.h"
#include "pid.h"
#include "encoders.h"
#include "PololuBuzzer.h"
#include "timer3.h"

volatile float l_speed_t3;
volatile float r_speed_t3;

#define LED_PIN 13
#define left_power_pin 10
#define right_power_pin 9
#define left_direction_pin 16
#define right_direction_pin 15// Pin to activate the orange LED

PID_c spd_pid_left; // PID for Left or Right Wheel
PID_c spd_pid_right;

unsigned long update_ts; //timestamp

float ave_e1_spd; // low pass filter of speed

long count_e1_last; // for difference in encoder counts

boolean led_state;
// Variable to "remember" the state
// of the LED, and toggle it.
LineSensor_c line_sensors;
Motors_c motors;
Encoders_c encoders;
Kinematics_c kinematics;
float e0;
float e1;
float initl, initr = 0;
float x;
float y;
float theta;
PololuBuzzer buzzer;

void retard(unsigned long num) {
  unsigned long startTime = millis();
  while (millis() - startTime < num) {
  }
}


void encoderval()
{ // Kinematics
  e0 = encoders.left_enc();
  e1 = encoders.right_enc();

  float garbage0 = e0 - initl;
  float garbage1 = e1 - initr;

  initl = e0;
  initr = e1;
  kinematics.update(garbage0, garbage1);
  x = kinematics.xcord();
  y = kinematics.ycord();
  theta = kinematics.theta_rad();
  Serial.print("x:");
  Serial.print(x);
  Serial.print(" -- ");
  Serial.print("y:");
  Serial.print(y);
  Serial.print(" -- ");
  Serial.print("theta:");
  Serial.print(theta);
  Serial.println("");
}

void angle_1() {
  analogWrite(left_power_pin, 0);
  analogWrite(right_power_pin, 0);
  retard(1500);
  digitalWrite(left_direction_pin, FWD);
  digitalWrite(right_direction_pin, REV);
  analogWrite(left_power_pin, 20);
  analogWrite(right_power_pin, 20);
  retard(1300);
  digitalWrite(left_direction_pin, FWD);
  digitalWrite(right_direction_pin, FWD);
  analogWrite(left_power_pin, 50);
  analogWrite(right_power_pin, 50);
  retard(7300);
  while (1) {
    analogWrite(left_power_pin, 0);
    analogWrite(right_power_pin, 0);
  }
}

void angle_2() {
  analogWrite(left_power_pin, 0);
  analogWrite(right_power_pin, 0);
  retard(1500);
  digitalWrite(left_direction_pin, REV);
  digitalWrite(right_direction_pin, FWD);
  analogWrite(left_power_pin, 20);
  analogWrite(right_power_pin, 20);
  retard(2400);
  digitalWrite(left_direction_pin, FWD);
  digitalWrite(right_direction_pin, FWD);
  analogWrite(left_power_pin, 50);
  analogWrite(right_power_pin, 50);
  retard(5300);
  while (1) {
    analogWrite(left_power_pin, 0);
    analogWrite(right_power_pin, 0);
  }
}

char state;
int line;
int colour = 1000;

// put your setup code here, to run once:
void setup() {

  line_sensors.init();
  motors.initialise();
  encoders.setupEncoder0();
  encoders.setupEncoder1();

  //Setup kp, ki, kd.
  spd_pid_left.init(110.0,1.1,-200.0);
  spd_pid_right.init(110.0,1.1,-200.0);

  // Just to compare the speed measurement
  setupTimer3();

  Serial.begin(9600);
  retard(1000);
  Serial.println("*RESET*");

  update_ts = millis();

  // Set initial value for our last e1 count within loop
  count_e1_last = count_e1;

  motors.setMotorPower(0, 0);

  //ave_e1_last = 0.0;

  //Reset PID before we use it
  spd_pid_left.reset();

  // Set LED pin as an output
  pinMode( LED_PIN, OUTPUT );

  // Set initial state of the LED
  led_state = false;
  line = 0;
  state = "ignore";

}


// put your main code here, to run repeatedly:
void loop() {

  unsigned long elapsed; //Calculate difference in time

  //Last time, Current time
  elapsed = millis() - update_ts;

  // Calculate speed estimate (For now just for the left motor)
  if (elapsed > 20) { // every 20ms
    update_ts = millis();

    long diff_e1; // difference in encoder count
    float e1_speed; // Speed calculated for e1

    // Calculate differences in e1
    diff_e1 = count_e1 - count_e1_last; // Difference in encoder count
    count_e1_last = count_e1; // Speed calculated for e1

    // Encoder counts per ms
    e1_speed = (float)diff_e1;
    e1_speed /= (float)elapsed; // Actual elapsed ms

    ave_e1_spd = (ave_e1_spd * 0.7) + (e1_speed* 0.3);

    // Encoder counts per ms
    Serial.print(e1_speed*100);
    Serial.print(",");
    Serial.print(l_speed_t3*100);
    Serial.print(",");
    Serial.print(ave_e1_spd*100);
    Serial.print("\n");

    float demand;
    float pwm;
// analogWRITE( L_PWM, 1.8);
    demand = 1.8; //1.8 encoder counts per ms
//    pwm(1.8) = spd_pid_left.update(ave_e1_speed, demand); // (measurement, demand)

    // P - term = 
    // I - term
    // D - term
    motors.setMotorPower(pwm, 0);

    Serial.print(l_speed_t3);
    Serial.print(",");
    Serial.print(spd_pid_left.last_error);
    Serial.print("");
    Serial.println(demand);

  }// end of millis()block

  //if(millis() - pid_test_ts > 1000){
   // pid_test_ts = millis();
   //demand = demand * - 1;
 // }
  //Line Sensor
  digitalWrite(LED_PIN, led_state);

  //Serial.println("loop");
  //Serial.println(encoders.left_enc());
  //Serial.println(encoders.right_enc());
  float reading[5];

  for ( int i = 0; i < 5; i++ ) {

    // This line calls a function within your class
    // and stores the returned value.
    reading[i] = line_sensors.readLineSensor( i );

    Serial.print( reading[i] );
    Serial.print(", ");
  }

  Serial.println(""); // to create a new line


  // Using an if statement to toggle a variable
  // with each call of loop()
  if ( led_state == true ) {
    led_state = false;
  } else {
    led_state = true;
  }

  if (reading[2] >= colour && line < 2)
  {
    buzzer.playFrequency(1000, 200, 15);
    retard(100);
    line = line + 1;
    if (line == 1) {
      retard(300);
    }
    if (line == 2)
    {
      state = "proceed";
      Serial.println("straight proceed");

    }

  }
  else if (line < 2) {
    motors.setDir(LOW, LOW);         // Move forwards.
    motors.setMotorPower(20, 20);

  }


  else if (line == 2) {

    if (reading[1] > colour) {
      motors.setDir(LOW, LOW);
      motors.setMotorPower(0, 20);              //Turn left
    }
    else if (reading[2] > colour) {
      motors.setDir(LOW, LOW);
      motors.setMotorPower(20, 20);             // Move forward
    } else if (reading[3] > colour) {
      motors.setDir(LOW, LOW);                // Turn right
      motors.setMotorPower(20, 0);
    } else if (reading[0] > colour) {
      motors.setDir(LOW, LOW);
      motors.setMotorPower(20, 20);          // Turn sharp left
      retard(300);
      motors.setDir(HIGH, LOW);
      motors.setMotorPower(20, 20);
      retard(800);
    } else if (reading[4] > colour) {
      motors.setDir(LOW, LOW);
      motors.setMotorPower(20, 20);         //Turn sharp right
      retard(300);
      motors.setDir(LOW, HIGH);
      motors.setMotorPower(20, 20);
      retard(800);
    } else {

      if (reading[0] <= colour && reading[1] <= colour && reading[2] <= colour && reading[3] <= colour && reading [4] <= colour && y > 100) {

        buzzer.playFrequency(1000, 200, 15);
        retard(100);
        while (1) {
          motors.setDir(LOW, LOW);
          motors.setMotorPower(0, 0);
          angle_1();
        }
      }

      else if (reading[0] <= colour && reading[1] <= colour && reading[2] <= colour && reading[3] <= colour && reading [4] <= colour && y < -90) {

        buzzer.playFrequency(1000, 200, 15);
        retard(100);
        while (1) {
          motors.setDir(LOW, LOW);
          motors.setMotorPower(0, 0);
          angle_2();
        }
      }

      else if (reading[0] <= colour && reading[1] <= colour && reading[2] <= colour && reading[3] <= colour && reading [4] <= colour) {

        motors.setDir(LOW, LOW);
        motors.setMotorPower(20, 20);
        retard(500);
        motors.setDir(LOW, HIGH);
        motors.setMotorPower(30, 30);   //Taking U-Turn
        retard(950);
      }

    }
    encoderval();
  }


}


// We use the variable to set the
// debug led on or off on the 3Pi+
