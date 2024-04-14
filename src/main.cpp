
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif


MPU6050 mpu;

#define INTERRUPT_PIN 21 // use pin 2 on Arduino Uno & most boards
#define LED_PIN 2      // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;        // [w, x, y, z]         quaternion container
VectorInt16 aa;      // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;  // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld; // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity; // [x, y, z]            gravity vector
float euler[3];      // [psi, theta, phi]    Euler angle container
float ypr[3];        // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = {'$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n'};

volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
  mpuInterrupt = true;
}

void setup()
{
// join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // initialize serial communication
  // (115200 chosen because it is required for Teapot Demo output, but it's
  // really up to you depending on your project)
  Serial.begin(115200);

  // initialize device
  mpu.initialize();

  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_8);

  // Configurar el rango completo del giroscopio a ±2000°/s
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);

  pinMode(INTERRUPT_PIN, INPUT);

mpu.testConnection();


delay(3000);
  // load and configure the DMP
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  // mpu.setXAccelOffset(820);
  // mpu.setYAccelOffset(-1054);
  // mpu.setZAccelOffset(1548);
  mpu.setXAccelOffset(820);
  mpu.setYAccelOffset(-1120);
  mpu.setZAccelOffset(1548);

  mpu.setXGyroOffset(113.00000);
  mpu.setYGyroOffset(14.00000);
  mpu.setZGyroOffset(-8.00000);

  // make sure it worked (returns 0 if so)
  if (devStatus == 0)
  {
    // Calibration Time: generate offsets and calibrate our MPU6050
    // mpu.CalibrateAccel(100);
    // mpu.CalibrateGyro(100);
    mpu.PrintActiveOffsets();
    //delay(5000);
    // turn on the DMP, now that it's ready

    mpu.setDMPEnabled(true);

    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    dmpReady = true;

    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  // configure LED for output
  pinMode(LED_PIN, OUTPUT);
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
float at_ant=0.0;
float omega_ant=0.0;

void loop()
{
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);


// Convertir unidades raw a unidades físicas
float a_x = ax / (2048.0*8) * 9.81;         // en m/s^2
float a_y = ay / (2048.0*8) * 9.81;       // en m/s^2
float a_z = az / (2048.0*8) * 9.81;       // en m/s^2
float omega_x = gx / 16.4 * (PI / 180); // en rad/s
float omega_y = gy / 16.4 * (PI / 180); // en rad/s
float omega_z = gz / 16.4 * (PI / 180); // en rad/s

float a_t=sqrt(a_x*a_x+a_y*a_y+a_z*a_z)-9.81;

float omega_t2 = sqrt(omega_x * omega_x + omega_y * omega_y + omega_z * omega_z);

if(a_t!=at_ant || omega_t2!=omega_ant )
{
// Serial.print("ax: ");
// Serial.print(a_x);
// Serial.print("  ay: ");
// Serial.print(a_y);
// Serial.print("  az: ");
// Serial.print(a_z);
at_ant=a_t;
omega_ant=omega_t2;
Serial.print("  A: ");
Serial.print(a_t);
Serial.print(" VAt: ");
Serial.print(omega_t2);
Serial.print(" millis: ");
Serial.println(millis());
}

blinkState = !blinkState;
digitalWrite(LED_PIN, blinkState);
//delay(5);
}
