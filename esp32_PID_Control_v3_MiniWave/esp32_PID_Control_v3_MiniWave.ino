/*Notes
Get gyro data
If error, blink LED
Figure out voltage stuff
Figure out PWM output
Use gyro to drive servo
Tune a PID to control servo as a test

To Do:
Create PID within struct
Set up yaw PID
Wire up power
Set up remote control
Tune PIDs
*/

#include <Adafruit_BNO08x.h>
//#include <ESP32Servo.h>
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

//Servo leftMotor;
//Servo rightMotor;

//int servoMotorPinLeft = 5;
//int servoMotorPinRight = 6;
int pos = 0;
int LEFT_MOTOR_PWM_PIN = 5;
int LEFT_REVERSE_PIN = 9;
int RIGHT_MOTOR_PWM_PIN = 6;
int RIGHT_REVERSE_PIN = 10;

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
  double previous_error;
};
PID_Controller pitchPID; //create the two controllers based on PID_Controller "prefab"
PID_Controller yawPID;

float pitchDrive;
float yawDrive;
float inputLeft;
float inputRight;
float currentYaw;
//float pitchOffset = 7.5; //degrees to offset the pitch by
bool integralReset = false;
int integralLimit = 3000;


//bool isPaused = false;
float printInterval = 50;
float lastPrintTime;
float currentTime;
float lastBlinkTime;
long deltaTime;
long lastTime;




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
  pinMode(LEFT_REVERSE_PIN, OUTPUT);
  pinMode(RIGHT_REVERSE_PIN, OUTPUT);

  digitalWrite(LEFT_REVERSE_PIN, HIGH);
	analogWrite(LEFT_MOTOR_PWM_PIN, 0);
	digitalWrite(RIGHT_REVERSE_PIN, HIGH);
	analogWrite(RIGHT_MOTOR_PWM_PIN, 0);

  Serial.begin(115200);
  //check for serial monitor connection, or allow user to exit with boot button press
  
  
  while ((!Serial) && (digitalRead(BUTTON_PIN) == HIGH)) {
    delay(10);
    digitalWrite(LED_BUILTIN, HIGH);
  }
  
  delay(200);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(200);
  digitalWrite(LED_BUILTIN, LOW);
  delay(200);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(200);
  digitalWrite(LED_BUILTIN, LOW);
  delay(200);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(200);
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
	//ESP32PWM::allocateTimer(0);
	//ESP32PWM::allocateTimer(1);
	//ESP32PWM::allocateTimer(2);
	//ESP32PWM::allocateTimer(3);
	//leftMotor.setPeriodHertz(50);    // standard 50 hz servo
	//rightMotor.setPeriodHertz(50);    // standard 50 hz servo
	//leftMotor.attach(servoMotorPinLeft, 1000, 2000); // attaches the servo on pin 5 to the servo object
	//rightMotor.attach(servoMotorPinRight, 1000, 2000); // attaches the servo on pin 6 to the servo object
	// using default min/max of 1000us and 2000us

  //PID Setup Stuff:
  pitchPID.kp = 18; //8
  pitchPID.ki = 0.01; //.015
  pitchPID.kd = 500; //3
  pitchPID.setpoint = -7.5; //add a setpoint other than 0 here


  yawPID.kp = 0; //1
  yawPID.ki = 0.00; //0
  yawPID.kd = 0; //0
  currentYaw = ypr.yaw;
  yawPID.setpoint = currentYaw;

  lastTime = millis();
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
double calculate_pid(PID_Controller* pid, double actual)  {
  //double proportional;
  //double derivative;
  //double output;
  //double error; //already defined

  pid->error = pid->setpoint - pid->actual;
  pid->proportional = pid->error;
    if (((pid->error > 0 && pid->previous_error < 0) || (pid->error < 0 && pid->previous_error > 0)) && (integralReset == true)) {
      pid->integral = 0;
      //pid->integral += pid->error * deltaTime; //added this for testing, this is the copy
      //Serial.print("PID LIMIT HIT, INTEGRAL RESET");
    }
    else {
      pid->integral += pid->error * deltaTime; //calculate the integral
      pid->integral = constrain(pid->integral, -integralLimit, integralLimit); //constrain the integral (wip)
    }
    if (deltaTime > 0) {
        pid->derivative = (pid->error - pid->previous_error) / deltaTime;
    }
    else {
      pid->derivative = 0;
    }
  pid->previous_error = pid->error;
  pid->output = (pid->kp*pid->proportional) + (pid->ki * pid->integral) + (pid->kd * pid->derivative);
  ///Serial.print("P:"); Serial.print(pitchPID.proportional);              Serial.print("\t");
  //Serial.print("I:"); Serial.print(pitchPID.integral);                  Serial.print("\t");
  //Serial.print("D:"); Serial.print(pitchPID.derivative);                Serial.print("\t");
  return pid->output;
}

void driveInput(double pitchDrive, double yawDrive) {

  //calculate left motor drive
  inputLeft = pitchDrive - yawDrive;
  inputLeft = constrain(inputLeft, -255, 255);
  //Serial.print("leftDrive:"); Serial.print(inputLeft);
  if (inputLeft < 0) {
    digitalWrite(LEFT_REVERSE_PIN, LOW); //Pull low to activate reverse mode
    analogWrite(LEFT_MOTOR_PWM_PIN, abs(inputLeft));
  }
  else {
    digitalWrite(LEFT_REVERSE_PIN, HIGH); //DEFAULT FOR THESE PINS IS HIGH
    analogWrite(LEFT_MOTOR_PWM_PIN, abs(inputLeft));
  }

 //calculate right motor drive
  inputRight = pitchDrive + yawDrive;
  inputRight = constrain(inputRight, -255, 255);
  //Serial.print("\trightDrive:");Serial.println(inputRight);
    if (inputRight < 0) {
    digitalWrite(RIGHT_REVERSE_PIN, LOW); 
    analogWrite(RIGHT_MOTOR_PWM_PIN, abs(inputRight));
  }
  else {
    digitalWrite(RIGHT_REVERSE_PIN, HIGH); 
    analogWrite(RIGHT_MOTOR_PWM_PIN, abs(inputRight));
  }
}

void checkForCommand() {
  if (Serial.available() > 0) {
    char command = toupper(Serial.read()); //toupper makes it ignore case

    delay(2); //wait very briefly for new command

    if (Serial.available() > 0) {
      float value = Serial.parseFloat();

      switch (command) {
        case 'P':
          pitchPID.kp = value;
          Serial.print("PID P set to: "); Serial.println(pitchPID.kp);
          break;

        case 'I':
          pitchPID.ki = value;
          Serial.print("PID I set to: "); Serial.println(pitchPID.ki);
          break;

        case 'D':
          pitchPID.kd = value;
          Serial.print("PID D set to: "); Serial.println(pitchPID.kd);
          break;

        case 'Y':
          yawPID.kp = value;
          Serial.print("Yaw PID P set to: "); Serial.println(yawPID.kp);
          break;

        case 'S':
          pitchPID.setpoint = value;
          Serial.print("New setpoint: "); Serial.print(pitchPID.setpoint); Serial.println("degrees");

        default:
          Serial.println("Unknown command. USE P, I, D, or Y followed by a number.");
          break;
      }
    }
    while (Serial.available() > 0) {
      Serial.read(); //clear any leftover characters from the feed by reading and discarding them
    }
  }
}


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

    long now = millis(); //start counting milliseconds
    deltaTime = now - lastTime;
    //Serial.print("Loop_Time(ms):"); Serial.print(deltaTime);       Serial.print("\t"); //print loop time
    lastTime = now;
    

    //PID Loop Stuff:
    checkForCommand();

    pitchPID.actual = ypr.pitch;// + pitchOffset; //add in an offset to the pitch
    yawPID.actual = ypr.yaw;

    if ((ypr.pitch > -20) && (ypr.pitch < 20)) {
      digitalWrite(LED_BUILTIN, LOW);
      pitchPID.output = calculate_pid(&pitchPID, pitchPID.actual);
      yawPID.output = calculate_pid(&yawPID, yawPID.actual);

      pitchDrive = (pitchPID.output);
      yawDrive = (yawPID.output);
      driveInput(pitchDrive, yawDrive);
    }
    else {
      pitchPID.output = 0;
      yawPID.output = 0;
      analogWrite(RIGHT_MOTOR_PWM_PIN, 0);
      analogWrite(LEFT_MOTOR_PWM_PIN, 0);
      digitalWrite(LED_BUILTIN, HIGH);
      //Serial.println("ANGLE LIMIT HIT");
      yawPID.setpoint = ypr.yaw; //set new yaw setpoint to current yaw when bot is tipped back up
      pitchPID.integral = 0;
      yawPID.integral = 0;
      currentYaw = ypr.yaw;
    }
  }



//A print timer for slower actions
currentTime = millis();
  if (currentTime - lastPrintTime >= printInterval) {
    lastPrintTime = currentTime;

    //print bars in graph
    Serial.print("Top_Stop:"); Serial.print(60);                      Serial.print("\t");
    Serial.print("Bottom_Stop:"); Serial.print(-60);                  //Serial.print("\t");

    //Serial.print("Loop_Time(ms):"); Serial.print(deltaTime);        Serial.print("\t"); //print loop time
    //Serial.print("Accuracy:"); Serial.print(sensorValue.status);    Serial.print("\t");  // This is accuracy in the range of 0 to 3
    //Serial.print("Yaw:"); Serial.print(ypr.yaw);                    Serial.print("\t");
    //Serial.print("Pitch:"); Serial.print(ypr.pitch);                Serial.print("\t");
    //erial.print("Roll:"); Serial.print(ypr.roll);                  Serial.print("\t");
    //Serial.print("Actual:");    Serial.print(pitchPID.actual);                 Serial.print("\t");
    Serial.print("\tSetpoint:");  Serial.print(pitchPID.setpoint);               //Serial.print("\t");
    Serial.print("\tOutput:");  Serial.print(pitchPID.output);                   //Serial.print("\t");
    //Serial.print("\tError:"); Serial.print(pitchPID.error);
    //Serial.print("\tleftDrive:"); Serial.println(inputLeft);
    //Serial.print("\trightDrive:");Serial.println(inputRight);
    //Serial.print("\tpitchDrive:"); Serial.print(pitchDrive);
    //Serial.print("\tyawDrive:"); Serial.print(yawDrive);
    Serial.print("\tP:");  Serial.print(pitchPID.kp); 
    Serial.print("\tI:");  Serial.print(pitchPID.ki);
    Serial.print("\tD:");  Serial.print(pitchPID.kd);
    Serial.print("\tYaw P:");  Serial.print(yawPID.kp);
    Serial.println();
  }
}
