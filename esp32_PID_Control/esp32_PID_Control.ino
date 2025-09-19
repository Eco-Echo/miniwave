/*Notes
Get gyro data
If error, blink LED
Figure out voltage stuff
Figure out PWM output
Use gyro to drive servo
Tune a PID to control servo as a test
*/

#include <Adafruit_BNO08x.h>
#include <ESP32Servo.h>
#include <Arduino.h>

//define pins for CS and INT (don't fully understand this)
#define BNO08X_CS 10
#define BNO08X_INT 9
#define BNO08X_RESET -1 //no reset for I2C
#define BUTTON_PIN 0
#define LED_BUILTIN 13

Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

struct euler_t {
  float yaw;
  float pitch;
  float roll;
} ypr;

Servo myservo1;
Servo myservo2;

int servoPin1 = 5;
int servoPin2 = 6;
int pos = 0;

//numbers needed for PID
long deltaTime, lastTime;
double pid_proportional, pid_integral, pid_derivative, pid_error, pid_previous_error, pid_actual, pid_output = 0;
double kp, ki, kd;
double pid_setpoint = 0;
/*
struct PID_Controller {
  double proportional;
  double integral;
  double derivative;
  double kp;
  double ki;
  double kd;
  double actual;
  double error;
  double setpoint;
  double output;
}pid;
*/

//bool isPaused = false;
float printInterval = 50;
float lastPrintTime;
float currentTime;
float lastBlinkTime;




// #define FAST_MODE

#ifdef FAST_MODE
  // Top frequency is reported to be 1000Hz (but freq is somewhat variable)
  sh2_SensorId_t reportType = SH2_GYRO_INTEGRATED_RV;
  long reportIntervalUs = 2000;
#else
  // Top frequency is about 250Hz but this report is more accurate
  sh2_SensorId_t reportType = SH2_ARVR_STABILIZED_RV;
  long reportIntervalUs = 5000;//default 5000 5ms
#endif
void setReports(sh2_SensorId_t reportType, long report_interval) {
  Serial.println("Setting desired reports");
  if (! bno08x.enableReport(reportType, report_interval)) {
    Serial.println("Could not enable stabilized remote vector");
  }
}


void setup() {

  //set up button use
  pinMode(BUTTON_PIN, INPUT_PULLUP); //use internal pullup resistor for button
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(115200);
  //check for serial monitor connection, or allow user to exit with boot button press
  while ((!Serial) && (digitalRead(BUTTON_PIN) == HIGH)) {
    delay(10);
    digitalWrite(LED_BUILTIN, HIGH);
  }

  digitalWrite(LED_BUILTIN, LOW);

  Serial.println("Gyro PID test");
  
  //if the bno08x fails to begin I2C, output an error
  if (!bno08x.begin_I2C()) {
    Serial.println("Failed to find BNO08X chip");
    while (1) { delay(10); }
  }
  Serial.println("BNO08X Found!");

  setReports(reportType, reportIntervalUs);

  Serial.println("Reading Events");
  delay(100);

	// Allow allocation of all timers
	ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);
	myservo1.setPeriodHertz(50);    // standard 50 hz servo
	myservo2.setPeriodHertz(50);    // standard 50 hz servo
	myservo1.attach(servoPin1, 1000, 2000); // attaches the servo on pin 5 to the servo object
	myservo2.attach(servoPin2, 1000, 2000); // attaches the servo on pin 6 to the servo object
	// using default min/max of 1000us and 2000us

  //PID Setup Stuff:
  kp = 2.5;
  ki = 0.002;
  kd = 0.00;
  lastTime = 0;
  //he had a flat line function here that I deleted

}



//CUSTOM FUNCTIONS START --------------------------------------------------------------------------------------

//convert quaternion to degrees using funky math
void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees = false) {

  float sqr = sq(qr);
  float sqi = sq(qi);
  float sqj = sq(qj);
  float sqk = sq(qk);

  ypr->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
  ypr->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
  ypr->roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));


  if (degrees == true) {
    ypr->yaw *= RAD_TO_DEG; //rad_to_deg is a constant that is 180/pi
    ypr->pitch *= RAD_TO_DEG; //the operator *= is the same as a = a*b, so this is pitch = pitch* rad_to_deg
    ypr->roll *= RAD_TO_DEG;
  }
}

void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, euler_t* ypr, bool degrees = false) {
    quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

void quaternionToEulerGI(sh2_GyroIntegratedRV_t* rotational_vector, euler_t* ypr, bool degrees = false) {
    quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

//PID function
double pid(double pid_error)  {
  pid_proportional = pid_error;
    if ((pid_error > 0 && pid_previous_error < 0) || (pid_error < 0 && pid_previous_error > 0)) {
      pid_integral = 0;
      Serial.print("PID LIMIT HIT, INTEGRAL RESET");
    }
    else {
      pid_integral += pid_error * deltaTime;
    }
  pid_derivative = (pid_error - pid_previous_error) / deltaTime;
  pid_previous_error = pid_error;
  pid_output = (kp*pid_proportional) + (ki * pid_integral) + (kd * pid_derivative);
  Serial.print("P:"); Serial.print(pid_proportional);              Serial.print("\t");
  Serial.print("I:"); Serial.print(pid_integral);                  Serial.print("\t");
  Serial.print("D:"); Serial.print(pid_derivative);                Serial.print("\t");
  return pid_output;
}

/*
void pidReset() {
  pid_integral = 0;
  pid_output = 0;
}
*/
//CUSTOM FUNCTIONS END --------------------------------------------------------------------------------------



void loop() {
  // put your main code here, to run repeatedly:
  if (bno08x.wasReset()) {
    Serial.println("Sensor was reset");
    setReports(reportType, reportIntervalUs);
  }

  if (bno08x.getSensorEvent(&sensorValue)) {
    // in this demo only one report type will be received depending on FAST_MODE define (above)
    switch (sensorValue.sensorId) {
      case SH2_ARVR_STABILIZED_RV:
        quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr, true);
        break;
      case SH2_GYRO_INTEGRATED_RV:
        // faster (more noise?)
        quaternionToEulerGI(&sensorValue.un.gyroIntegratedRV, &ypr, true);
        break;
  }

  static long lastTime = 0;
  long now = millis(); //start counting milliseconds
  deltaTime = now - lastTime;
  //Serial.print("Loop_Time(ms):"); Serial.print(deltaTime);       Serial.print("\t"); //print loop time
  lastTime = now;
  

  //PID Loop Stuff:
  if ((ypr.pitch < -30) || (ypr.pitch > 30)) {
    pid_integral = 0;
    pid_output = 0;
    if (currentTime - lastBlinkTime >= 50) {
      digitalWrite(LED_BUILTIN, HIGH);
    }
    if (currentTime - lastBlinkTime >= 100) {
      digitalWrite(LED_BUILTIN, LOW);
      lastBlinkTime = currentTime;
    }
    Serial.println("ANGLE LIMIT HIT");
  }
  else {
    pid_actual = ypr.pitch;
    pid_error = pid_setpoint - pid_actual;
    pid_output = pid(pid_error);
    //Serial.print("Output:");  Serial.print(output);                   Serial.print("\t");
}

  pos = 90 - (pid_output); 
      myservo1.write(180 - pos);
      myservo2.write(pos);

  }

//A print timer for slower actions
currentTime = millis();
  if (currentTime - lastPrintTime >= printInterval) {
    lastPrintTime = currentTime;

    //print bars in graph
    Serial.print("Top_Stop:"); Serial.print(60);                      Serial.print("\t");
    Serial.print("Bottom_Stop:"); Serial.print(-60);                  Serial.print("\t");

    //Serial.print("Loop_Time(ms):"); Serial.print(deltaTime);        Serial.print("\t"); //print loop time
    //Serial.print("Accuracy:"); Serial.print(sensorValue.status);    Serial.print("\t");  // This is accuracy in the range of 0 to 3
    //Serial.print("Yaw:"); Serial.print(ypr.yaw);                    Serial.print("\t");
    //Serial.print("Pitch:"); Serial.print(ypr.pitch);                Serial.print("\t");
    //erial.print("Roll:"); Serial.print(ypr.roll);                  Serial.print("\t");
    Serial.print("Actual:");    Serial.print(pid_actual);                 Serial.print("\t");
    Serial.print("Setpoint:");  Serial.print(pid_setpoint);               Serial.print("\t");
    Serial.print("Output:");  Serial.print(pid_output);                   Serial.print("\t");
    Serial.print("Error:"); Serial.println(pid_error);
  }
}
