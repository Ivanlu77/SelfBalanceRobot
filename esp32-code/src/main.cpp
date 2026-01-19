#include <Arduino.h>
#include <SPI.h>
#include <TimerInterrupt_Generic.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <step.h>
#include <HardwareSerial.h>
#include <cmath>

HardwareSerial espSerial(2);
// The Stepper pins
const int STEPPER1_DIR_PIN  = 16;
const int STEPPER1_STEP_PIN = 17;
const int STEPPER2_DIR_PIN  = 4;
const int STEPPER2_STEP_PIN = 14;
const int STEPPER_EN_PIN    = 15; 

//ADC pins
const int ADC_CS_PIN        = 5;
const int ADC_SCK_PIN       = 18;
const int ADC_MISO_PIN      = 19;
const int ADC_MOSI_PIN      = 23;

// Diagnostic pin for oscilloscope
const int TOGGLE_PIN        = 32;

const int UPDATE_INTERVAL = 1000; // Update interval in milliseconds
const int PRINT_INTERVAL    = 500;
const int LOOP_INTERVAL     = 10;
const int STEPPER_INTERVAL_US = 20;

const float kx = 5.0;
const float VREF = 4.096;

// PID coefficients
float kp1 = 1848;
float ki1 = 0;
float kd1 = 20; ;//800
float kp2 = -0.002;//-0.0030,-0.003215;
float ki2 = 0;//-0.00007
float kd2 = 0.0;
float kp_turn = 50.0;//250  
float ki_turn = 0.0;
float kd_turn = 0.0;

//input values
float targetYaw = 0.0; 
float targetSpeed=0.0;

//bias values of devices, may need to be changed
float targetAnglebias=0.126;//0.149
float yawbias=0.145;  // Updated to eliminate 0.145 drift when stationary
float gyro_ybias=0.035;

// variable used in pid
static float angleIntegral = 0.0;
static float lastAngleError = 0.0;
static float lastAcce=0.0;
static float speedIntegral = 0.0;
static float lastSpeedError = 0.0;
float acce=0.0;
float turnIntegral = 0, lastTurnError = 0;
float lastTurn=0.0;
float motorCommand=0.0;
float speed = 0.0;
float targetAngle = 0.0;
float output = 0.0;
float accAngle=0.0;
float gyroRate=0.0;
float dt=0.0;
float lastTargetAngle=0.0;
float currentYaw=0.0000;
float alpha=0.0;
float actualSpeed=0.0;
float speedError=0.0;
float speedDerivative=0.0;
float angleError=0.0;
float angleDerivative=0.0;
float yawError=0.0;
float turnDerivative=0.0;
float turnOutput=0.0;

// variables for odometry
float odometryX=0.0;
float odometryY=0.0;
float odometryD=0.0;
float lastOdometryD=0.0;  // Add variable to track previous total distance
float odometryYawAngle=0.0; // This will be calculated from MPU6050 gyroscope
float lastLeftPosition = 0.0;
float lastRightPosition = 0.0;
const float WHEEL_BASE = 0.12; // 12cm
static unsigned long lastSendTime = 0;
const int SEND_INTERVAL = 500; // 100ms = 10Hz
const float GYRO_THRESHOLD = 0.05;
const float ACCEL_THRESHOLD = 0.2;      // ���Լ��ٶ���ֵ (m/s?)�����ڴ�ֵ��Ϊû�������˶�
const float SPEED_THRESHOLD = 0.01;     // �ٶ���ֵ (m/s)�����ڴ�ֵ��Ϊ������ֹ
const float GYRO_DEADBAND = 0.01;       // Gyro deadband after bias correction - reduced from 0.03

// Add variable for gyroscope-based yaw calculation
float gyroYawRate = 0.0; // Current yaw rate from gyroscope
float relativeYawAngle = 0.0; // Relative angle change from initial state

//power variables
float Imotor=0;
float Vmotor=0;
float Pmotor=0.0;
float accPmotor=0.0;
float Idevice=0.0;
float Vdevice=0.0;
float Pdevice=0.0;
float accPdevice=0.0;
float Vbattery=0.0;
float accVbattery=0.0;
float motorctr=0.0;

float Ptotal=0.0;
float aveVbattery=0.0;
float batteryEnergy=0.0;
float batteryPercentage=0.0;
float lastBatteryPercentage=100;


//Global objects
ESP32Timer ITimer(3);
Adafruit_MPU6050 mpu;         //Default pins for I2C are SCL: IO22, SDA: IO21

step step1(STEPPER_INTERVAL_US,STEPPER1_STEP_PIN,STEPPER1_DIR_PIN );
step step2(STEPPER_INTERVAL_US,STEPPER2_STEP_PIN,STEPPER2_DIR_PIN );

//Interrupt Service Routine for motor update
//Note: ESP32 doesn't support floating point calculations in an ISR
bool TimerHandler(void * timerNo)
{
  static bool toggle = false;

  //Update the stepper motors
  step1.runStepper();
  step2.runStepper();

  //Indicate that the ISR is running
  digitalWrite(TOGGLE_PIN,toggle);  
  toggle = !toggle;
	return true;
}

uint16_t readADC(uint8_t channel)
{
  uint8_t TX0 = 0x06 | (channel >> 2);  // Command Byte 0 = Start bit + single-ended mode + MSB of channel
  uint8_t txByte1 = (channel & 0x03) << 6;  // Command Byte 1 = Remaining 2 bits of channel

  digitalWrite(ADC_CS_PIN, LOW); 
   SPI.transfer(TX0);                    // Send Command Byte 0
  uint8_t RX0 = SPI.transfer(txByte1);      // Send Command Byte 1 and receive high byte of result
  uint8_t rxByte1 = SPI.transfer(0x00);     // Send dummy byte and receive low byte of result

  digitalWrite(ADC_CS_PIN, HIGH); 
  uint16_t result = ((RX0 & 0x0F) << 8) | rxByte1; // Combine high and low byte into 12-bit result
  return result;
}

void setup()
{
  Serial.begin(115200);
  pinMode(TOGGLE_PIN,OUTPUT);
  espSerial.begin(115200, SERIAL_8N1, 26, 27);  // rx=26,tx=27;

  // Try to initialize Accelerometer/Gyroscope
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

  //Attach motor update ISR to timer to run every STEPPER_INTERVAL_US μs
  if (!ITimer.attachInterruptInterval(STEPPER_INTERVAL_US, TimerHandler)) {
    Serial.println("Failed to start stepper interrupt");
    while (1) delay(10);
    }
  Serial.println("Initialised Interrupt for Stepper");

  //Set motor acceleration values
 
  //Enable the stepper motor drivers
  pinMode(STEPPER_EN_PIN,OUTPUT);
  digitalWrite(STEPPER_EN_PIN, false);

  //Set up ADC and SPI
  pinMode(ADC_CS_PIN, OUTPUT);
  digitalWrite(ADC_CS_PIN, HIGH);
  SPI.begin(ADC_SCK_PIN, ADC_MISO_PIN, ADC_MOSI_PIN, ADC_CS_PIN);

}

double batteryRemainingEnergy(double V) {
    double term1 = 2284.5665 / (1 + exp(-11.3488 * (V - 13.0604)));
    double term2 = 1587.1881 / (1 + exp(-3.0477 * (V - 11.3727)));
    double term3 = 20445.1209 / (1 + exp(-3.5074 * (V - 15.3522)));
    
    return term1 + term2 + term3 + 247.8728;
}

void loop()
{
  //Static variables are initialised once and then the value is remembered betweeen subsequent calls to this function
  static unsigned long printTimer = 0;  //time of the next print
  static unsigned long updateTimer = 0; //time of the next update
  static unsigned long loopTimer = 0;   //time of the next control update
  static float tiltx = 0.0;             //current tilt angle
  
  //Run the control loop every LOOP_INTERVAL ms
  
  if (millis() > loopTimer) {
    loopTimer += LOOP_INTERVAL;
    static unsigned long lastTime = 0;
    unsigned long now = millis();

    if (espSerial.available()) {
      String receivedData = espSerial.readStringUntil('\n');
      // Serial.print("Received: ");
      // Serial.println(receivedData);
      int commaIndex = receivedData.indexOf(',');
      if (commaIndex != -1) {
        String xPart = receivedData.substring(0, commaIndex);
        String yPart = receivedData.substring(commaIndex + 1);
        targetSpeed = xPart.toFloat();
        targetYaw = yPart.toFloat();
      }
    }

    // if(now<5000){
    //   targetSpeed=0;
    //   targetYaw=0;
    // }else if (now<7000){
    //   targetSpeed=0;
    //   targetYaw=0.2;
    // }else if(now<9000){
    //   targetSpeed=0;
    //   targetYaw=0;
    // }else if(now<11000){
    //   targetSpeed=0;
    //   targetYaw=-0.2;

    // }else{
    //   targetSpeed=0;
    //   targetYaw=0;
    // }

    dt = (now - lastTime) / 1000.0;
    lastTime = now;

    // Fetch data from MPU6050
    //targetAngle = 0.0;
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    accAngle = atan2(a.acceleration.z, sqrt(a.acceleration.x * a.acceleration.x + a.acceleration.y * a.acceleration.y));
    gyroRate = g.gyro.y-gyro_ybias;
    alpha = 0.98;
    tiltx =( alpha * (tiltx + gyroRate * dt) + (1 - alpha) * accAngle);

    // Outer loop: velocity control loop
    actualSpeed = -(step1.getSpeedRad());
    speedError = targetSpeed - actualSpeed;
    speedIntegral += speedError * dt;
    speedIntegral = constrain(speedIntegral, -100, 100);
    speedDerivative = (speedError - lastSpeedError) / dt;
    targetAngle= (kp2 * speedError + ki2 * speedIntegral + kd2 * speedDerivative)+targetAnglebias;
    targetAngle=0.7*targetAngle+0.3*lastTargetAngle;
    lastTargetAngle=targetAngle;
    lastSpeedError = speedError;

    // Inner loop: attitude control loop
    angleError = targetAngle - tiltx;
    angleIntegral += angleError * dt;
    angleIntegral = constrain(angleIntegral, -300, 300);
    angleDerivative = (angleError - lastAngleError) / dt;
    acce = (kp1 * angleError + ki1 * angleIntegral + kd1 * angleDerivative);
    acce= acce*0.7+lastAcce*0.3;
    lastAcce=acce;
    lastAngleError = angleError;

    //yaw control loop, indepandent from two loops above
    currentYaw = g.gyro.x-yawbias;  // Changed from g.gyro.z to g.gyro.x for vertical placement
    yawError = targetYaw - currentYaw;
    turnIntegral += yawError * dt;
    turnDerivative = (yawError - lastTurnError) / dt;
    turnOutput = kp_turn * yawError + ki_turn * turnIntegral + kd_turn * turnDerivative;
    turnOutput=turnOutput*0.7+lastTurn*0.3;
    lastTurn=turnOutput;
    lastTurnError = yawError;

    //odomemtry
    
    // Get current wheel positions (in meters)
    float currentLeftPosition = step2.getPositionDis();   // step2 left wheel
    float currentRightPosition = -step1.getPositionDis();  // step1 right wheel
    
    // Calculate wheel displacement since last update
    float deltaLeft = currentLeftPosition - lastLeftPosition;
    float deltaRight = currentRightPosition - lastRightPosition;
    
    // Calculate robot displacement (forward/backward from wheels)
    float deltaDistance = (deltaLeft + deltaRight) / 2.0;
    
    // Use MPU6050 gyroscope for yaw angle calculation
    float rawGyroYawRate = g.gyro.x;  // Raw gyro reading
    gyroYawRate = rawGyroYawRate - yawbias;  // Apply bias correction
    
    // Apply deadband to reduce drift accumulation when stationary
    if (abs(gyroYawRate) < GYRO_DEADBAND) {
        gyroYawRate = 0.0;  // Set to zero if within deadband
    }
    
    // Update yaw angle using gyroscope integration
    if (abs(gyroYawRate) > 0.0) {  // Only integrate if there's actual rotation
        float deltaYaw = gyroYawRate * dt;  // Calculate angle change this iteration
        odometryYawAngle += deltaYaw;  // Integrate gyro rate to get absolute angle for odometry
        relativeYawAngle += deltaYaw;  // Track relative angle change from initial state
    }
    
    // Calculate position update using wheel-based distance and gyro-based angle
    odometryX += deltaDistance * cos(odometryYawAngle);
    odometryY += deltaDistance * sin(odometryYawAngle);
    odometryD += deltaDistance;
    
    // update last data
    lastLeftPosition = currentLeftPosition;
    lastRightPosition = currentRightPosition;
    
    // Send odometry data
    if(millis() - lastSendTime >= SEND_INTERVAL){
      // Calculate distance moved since last data point
      float deltaDistanceSinceLastSend = odometryD - lastOdometryD;
      
      String dataString = String(relativeYawAngle * 57.296, 2) + "," + String(deltaDistanceSinceLastSend, 6) + "," + String(Ptotal, 3) + "," + String(batteryPercentage, 3);
      espSerial.println(dataString);

      Serial.print(" | RelativeAngle(deg): ");
      Serial.print(relativeYawAngle * 57.296,2);  // Convert to degrees
      Serial.print(" | DeltaDistance: ");
      Serial.print(deltaDistanceSinceLastSend,6);
      Serial.print(" | Ptotal: ");
      Serial.print(Ptotal, 3);
      Serial.print(" | batteryPercentage: ");
      Serial.println(batteryPercentage, 3);

      // Update last distance for next calculation
      lastOdometryD = odometryD;
      lastSendTime = millis();
      
      if(targetSpeed == 0 && targetYaw == 0){
        step1.resetPosition();
        step2.resetPosition();
        lastLeftPosition = 0.0;
        lastRightPosition = 0.0;
        // Only reset position, but keep angle information
        odometryX = 0.0;
        odometryY = 0.0;
        odometryD = 0.0;
        lastOdometryD = 0.0;  // Reset last distance tracking as well
        // Do NOT reset angles - keep accumulated angle information
        // odometryYawAngle = 0.0;  // Commented out to preserve angle
        // relativeYawAngle = 0.0;  // Commented out to preserve relative angle
      }
    }

    acce = constrain(acce, -120, 120);
    step1.setAccelerationRad(abs(acce+turnOutput));
    step2.setAccelerationRad(abs(acce-turnOutput));
    
    //  if(targetSpeed=0){
    //    if(targetYaw>0){
    //      step1.setAccelerationRad(abs(acce+turnOutput+yawError*2000));
    //      step2.setAccelerationRad(abs(acce-turnOutput));
    //    }else if(targetYaw<0){
    //      step1.setAccelerationRad(abs(acce+turnOutput));
    //      step2.setAccelerationRad(abs(acce-turnOutput));
    //    }
    //   }
    if(acce+turnOutput>0){
      step1.setTargetSpeedRad(-(20));
    }else {
      step1.setTargetSpeedRad((20));
    }
    if(acce-turnOutput>0){
      step2.setTargetSpeedRad((20));
    }else {
      step2.setTargetSpeedRad(-(20));
    }

    Imotor=((readADC(0) * VREF / 4095.0f - 0.1618f) / 14.077f / 0.1f);
    Imotor = Imotor > 0.0f ? Imotor : 0.0f;
    Vmotor = ((readADC(1) * VREF) / 4095.0f) * 10.0f;
    Pmotor = Imotor * Vmotor;
    accPmotor += Pmotor;

    Idevice = (readADC(2) * VREF / 4095.0f + 0.5704f) / 119.8f / 0.01f;
    Idevice = Idevice > 0.0f ? Idevice : 0.0f;
    Vdevice = 5.17f;
    Pdevice = Idevice * Vdevice;
    accPdevice += Pdevice;

    Vbattery = (readADC(3) * VREF) / 4095.0f * 10.0f;
    accVbattery += Vbattery;
    motorctr++;
  }

  //Print updates every PRINT_INTERVAL ms
  //Line format: X-axis tilt, Motor speed, A0 Voltage
  if (millis() > printTimer) {
    //if (millis() > printTimer) {
    printTimer += PRINT_INTERVAL;
    // Serial.print("ACC Angle: ");
    // Serial.print(accAngle);
    // Serial.print(" deg\t");
    // Serial.print("GYRO Rate: ");
    // Serial.print(gyroRate);
    // Serial.print(" dt: ");
    // Serial.print(dt, 4);
    // Serial.println(" deA/s");
    // Serial.print("tiltx (deg): ");
    // Serial.println(tiltx,2);  // 原本单位不清，现在你要的话可以换成角度显�??
    // Serial.print(" | targetAngle: ");
    // Serial.print(targetAngle-targetAnglebias);
    // Serial.print(" | error: ");
    // Serial.print(targetAngle - tiltx);
    // Serial.print(" | output1: ");
    // Serial.print(acce+turnOutput,2);
    // Serial.print(" | output2: ");
    // Serial.print(acce-turnOutput,2);
    // Serial.print(" | motorSpeed1: ");
    // Serial.print(step1.getSpeedRad(),2);
    // Serial.print(" | motorSpeed2: ");
    // Serial.print(step2.getSpeedRad(),2);
    // Serial.print(" | targetSpeed: ");
    // Serial.print(targetSpeed,2);
    // Serial.print(" |gyro.z: ");
    // Serial.print(currentYaw,2);

    // Serial.print(" | position1: ");
    // Serial.print(step1.getPositionDis());
    // Serial.print(" | position2: ");
    // Serial.print(step2.getPositionDis());
    // Serial.print(" | odometryX: ");
    // Serial.print(odometryX);
    // Serial.print(" | odometryY: ");
    // Serial.print(odometryY);
    // Serial.print(" | odometryYawAngle: ");
    // Serial.println(odometryYawAngle);

    //Serial.print(" | millis: ");
    //Serial.print(millis());
    //Serial.print(" | ADC(A0): ");
    //Serial.println((readADC(0) * VREF) / 4095.0);
    
  }

  if (millis() > updateTimer) {
    updateTimer += UPDATE_INTERVAL;
    Ptotal = (accPmotor + accPdevice) / motorctr;
    aveVbattery = accVbattery / motorctr;
    batteryEnergy = batteryRemainingEnergy(aveVbattery);
    batteryPercentage = (batteryEnergy / 24300) * 100.0;
    batteryPercentage = constrain(batteryPercentage, 0.0, 100.0);
    batteryPercentage = batteryPercentage < lastBatteryPercentage ? batteryPercentage : lastBatteryPercentage;
    lastBatteryPercentage = batteryPercentage;
    motorctr = 0.0;
    accPmotor = 0.0;
    accPdevice = 0.0;
    accVbattery = 0.0;
  }

}