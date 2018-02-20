#include <Arduino.h>

#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <Syslog.h>
#include <I2Cdev.h>
#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file
#include "Wire.h"

#include "config.h"
#include "update.h"
#include "log.h"

#define MPU_POWER_PIN  14
#define MPU_INT_PIN    12
#define I2C_SDA 4
#define I2C_SCL 5

/* maximum time to wait for a new interrupt */
#define MAX_INT_WAIT_MS   2000

/* maximum consecutive errors after interrupt signal */
#define MAX_INT_ERRORS    500
uint16_t intErrors = 0;

MPU6050 mpu;

WiFiUDP udpSyslogClient;
WiFiUDP udpSender;
WiFiUDP udpControl;
Syslog syslog(udpSyslogClient, CONTROLLER, SYSLOG_PORT, HOSTNAME, APP_NAME, LOG_KERN);

struct {
  unsigned long last_frame;
  unsigned long last_packet_check;
  unsigned long last_control_check;
} ts;

uint16_t frame_cnt = 0;

void eventWiFi(WiFiEvent_t event) {
  switch (event) {
    case WIFI_EVENT_STAMODE_CONNECTED:
      break;

    case WIFI_EVENT_STAMODE_DISCONNECTED:
      // Maybe find a better solution for this.
      ESP.restart();
      break;

    case WIFI_EVENT_STAMODE_AUTHMODE_CHANGE:
      break;

    case WIFI_EVENT_STAMODE_GOT_IP:
      break;

    case WIFI_EVENT_STAMODE_DHCP_TIMEOUT:
      break;

    case WIFI_EVENT_SOFTAPMODE_STACONNECTED:
      break;

    case WIFI_EVENT_SOFTAPMODE_STADISCONNECTED:
      break;

    case WIFI_EVENT_SOFTAPMODE_PROBEREQRECVED:
      break;
  }
}

void netSetup() {
  WiFi.onEvent(eventWiFi);
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
}

// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
//#define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_EULER

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_YAWPITCHROLL

// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
//#define OUTPUT_READABLE_REALACCEL

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
//#define OUTPUT_READABLE_WORLDACCEL

// uncomment "OUTPUT_TEAPOT" if you want output that matches the
// format used for the InvenSense teapot demo
//#define OUTPUT_TEAPOT
//#define OUTPUT_TEAPOT_UDP
#define UDP_SEND_COMPRESSED

typedef union {
	struct {
		uint16_t quat_w;
		uint8_t _1[2];
		uint16_t quat_x;
		uint8_t _2[2];
		uint16_t quat_y;
		uint8_t _3[2];
		uint16_t quat_z;
		uint8_t _4[2];
		uint16_t gyro_x;
		uint8_t _5[2];
		uint16_t gyro_y;
		uint8_t _6[2];
		uint16_t gyro_z;
		uint8_t _7[2];
		uint16_t acc_x;
		uint8_t _8[2];
		uint16_t acc_y;
		uint8_t _9[2];
		uint16_t acc_z;
		uint8_t _10[64 - 38];
	} data;
	uint8_t buff[64];
} fifo_t;


#ifdef UDP_SEND_COMPRESSED

typedef struct {
		uint32_t time;
		int16_t quat[4];
		uint16_t cnt;
} udp_data_t;

#define SEND_RATE 1000 / 30


typedef union {
	udp_data_t data;
	uint8_t buff[sizeof(udp_data_t)];
} udp_packet_t;

udp_packet_t compressedPacket;
uint16_t compressedSend = 0;
uint16_t compressedPacketCount = 0;

#endif

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;    // return status after each device operation (0 = success,
                      // !0 = error)
uint16_t packetSize;  // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;   // count of all bytes currently in FIFO
fifo_t fifoBuffer;  // FIFO storage buffer

// orientation/motion vars
Quaternion q;    // [w, x, y, z]         quaternion container
VectorInt16 aa;  // [x, y, z]            accel sensor measurements
VectorInt16
    aaReal;  // [x, y, z]            gravity-free accel sensor measurements
VectorInt16
    aaWorld;  // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;  // [x, y, z]            gravity vector
float euler[3];       // [psi, theta, phi]    Euler angle container
float
    ypr[3];  // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector



// packet structure for InvenSense teapot demo
typedef union {
	struct {
		char preamble[2];
		uint16_t w;
		uint16_t x;
		uint16_t y;
		uint16_t z;
		uint8_t _;
		uint8_t cnt;
	} components;
	uint8_t buff[14];
} teapot_t;

teapot_t teapotPacket = { .buff = {'$', 0x02, 0, 0,    0,    0,    0,
                            0,   0,    0, 0x00, 0x00, '\r', '\n'} };

#define NUM_PACKETS 10
uint8_t packet_pos = 0;
uint8_t udpPackets[14 * NUM_PACKETS];

volatile bool mpuInterrupt =
    false;  // indicates whether MPU interrupt pin has gone high
void ICACHE_RAM_ATTR dmpDataReady() { mpuInterrupt = true; }

void init_gyro() {

  // hard reset MPU to make sure it will communicate with us
  pinMode(MPU_POWER_PIN, OUTPUT);
  digitalWrite(MPU_POWER_PIN, LOW);
  delay(10);
  digitalWrite(MPU_POWER_PIN, HIGH);
  delay(10);

// join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);  // 400kHz I2C clock. Comment this line if having
                          // compilation difficulties

  // initialize device
  LOG(LOG_INFO, "Initializing I2C devices....");
  mpu.initialize();
  pinMode(MPU_INT_PIN, INPUT);

  // verify connection
  LOG(LOG_INFO, "Testing device connections...");
  LOG(LOG_INFO, mpu.testConnection() ? "MPU6050 connection successful"
                                      : "MPU6050 connection failed");

  // load and configure the DMP
  LOG(LOG_INFO, "Initializing DMP...");
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);  // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    LOG(LOG_INFO, "Enabling DMP...");
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    LOG(LOG_INFO, "Enabling interrupt detection (Arduino external interrupt 0)...");
    attachInterrupt(digitalPinToInterrupt(MPU_INT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use
    // it
    LOG(LOG_INFO, F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    LOGF(LOG_ERR, "DMP Initialization failed (code %d)\n", devStatus);
  }
}

void setup() {
#ifdef SERIAL_OUTPUT
  Serial.begin(115200);
  Serial.setDebugOutput(true);
#endif

  // disable sleep mode for better data rate
  WiFi.setSleepMode(WIFI_NONE_SLEEP);

  netSetup();

  LOG(LOG_INFO, "Controller was (re)booted - check for updates");
  UPDATE(CONTROLLER, UPDATE_PORT, UPDATE_EP);

  init_gyro();
  udpControl.begin(7001);

  LOG(LOG_INFO, "Starting normal operation");
}

void check_control() {
  size_t packetSize = udpControl.parsePacket();
  if (0 != packetSize) {
    unsigned char command = udpControl.read();
    LOGF(LOG_INFO, "Control command: %02x.", command);

    switch (command) {
      case 0x01:
        LOGF(LOG_CRIT, "Reboot controller");
	      delay(50);
	      ESP.restart();
	    break;
      default:
        LOGF(LOG_WARNING, "unknown command %d", command);
    }
  }
}

void loop() {
  // if programming failed, try again with reboot
  if (!dmpReady) {
    LOG(LOG_ERR, "DMP not ready - reboot");
    ESP.restart();
  }

  uint16_t intWaitTime = 0;
  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize) {
      // other program behavior stuff here
      // .
      // .
      // .
      // if you are really paranoid you can frequently test in between other
      // stuff to see if mpuInterrupt is true, and if so, "break;" from the
      // while() loop to immediately process the MPU data
      // .
      // .
      // .
      /* something went wrong and the MPU isn't talking to us anymore */
      if (intWaitTime >= MAX_INT_WAIT_MS) {
        LOG(LOG_ERR, "Interrupt is dead - reboot");
        ESP.restart();
      } else {
        // Delay for one millisecond here to give the WiFi stuff time to act
        // and increase counter
        delay(1);
        intWaitTime++;
      }
  }

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
      // reset so we can continue cleanly
      mpu.resetFIFO();
      LOG(LOG_WARNING, "FIFO overflow!");

      /* reboot when this happens again too often  */
      intErrors++;

  // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {

      /* reset int error counter */
      intErrors = 0;

      // wait for correct available data length, should be a VERY short wait
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

      // read a packet from FIFO
      mpu.getFIFOBytes(fifoBuffer.buff, packetSize);

      // track FIFO count here in case there is > 1 packet available
      // (this lets us immediately read more without waiting for an interrupt)
      fifoCount -= packetSize;

      #ifdef OUTPUT_READABLE_QUATERNION
      #ifndef SERIAL_OUTPUT
      #error define SERIAL_OUTPUT to enable serial output of values
      #endif
          // display quaternion values in easy matrix form: w x y z
          mpu.dmpGetQuaternion(&q, fifoBuffer.buff);
          Serial.print("quat\t");
          Serial.print(q.w);
          Serial.print("\t");
          Serial.print(q.x);
          Serial.print("\t");
          Serial.print(q.y);
          Serial.print("\t");
          Serial.println(q.z);
      #endif

      #ifdef OUTPUT_READABLE_EULER
      #ifndef SERIAL_OUTPUT
      #error define SERIAL_OUTPUT to enable serial output of values
      #endif
          // display Euler angles in degrees
          mpu.dmpGetQuaternion(&q, fifoBuffer.buff);
          mpu.dmpGetEuler(euler, &q);
          Serial.print("euler\t");
          Serial.print(euler[0] * 180/M_PI);
          Serial.print("\t");
          Serial.print(euler[1] * 180/M_PI);
          Serial.print("\t");
          Serial.println(euler[2] * 180/M_PI);
      #endif

      #ifdef OUTPUT_READABLE_YAWPITCHROLL
      #ifndef SERIAL_OUTPUT
      #error define SERIAL_OUTPUT to enable serial output of values
      #endif
          // display Euler angles in degrees
          mpu.dmpGetQuaternion(&q, fifoBuffer.buff);
          mpu.dmpGetGravity(&gravity, &q);
          mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
          Serial.print("ypr\t");
          Serial.print(ypr[0] * 180/M_PI);
          Serial.print("\t");
          Serial.print(ypr[1] * 180/M_PI);
          Serial.print("\t");
          Serial.println(ypr[2] * 180/M_PI);
      #endif

      #ifdef OUTPUT_READABLE_REALACCEL
      #ifndef SERIAL_OUTPUT
      #error define SERIAL_OUTPUT to enable serial output of values
      #endif
          // display real acceleration, adjusted to remove gravity
          mpu.dmpGetQuaternion(&q, fifoBuffer.buff);
          mpu.dmpGetAccel(&aa, fifoBuffer.buff);
          mpu.dmpGetGravity(&gravity, &q);
          mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
          Serial.print("areal\t");
          Serial.print(aaReal.x);
          Serial.print("\t");
          Serial.print(aaReal.y);
          Serial.print("\t");
          Serial.println(aaReal.z);
      #endif

      #ifdef OUTPUT_READABLE_WORLDACCEL
      #ifndef SERIAL_OUTPUT
      #error define SERIAL_OUTPUT to enable serial output of values
      #endif
        // display initial world-frame acceleration, adjusted to remove gravity
          // and rotated based on known orientation from quaternion
          mpu.dmpGetQuaternion(&q, fifoBuffer.buff);
          mpu.dmpGetAccel(&aa, fifoBuffer.buff);
          mpu.dmpGetGravity(&gravity, &q);
          mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
          mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
          Serial.print("aworld\t");
          Serial.print(aaWorld.x);
          Serial.print("\t");
          Serial.print(aaWorld.y);
          Serial.print("\t");
          Serial.println(aaWorld.z);
      #endif

      #ifdef OUTPUT_TEAPOT
          // display quaternion values in InvenSense Teapot demo format:
	  teapotPacket.components.w = fifoBuffer.data.quat_w;
	  teapotPacket.components.x = fifoBuffer.data.quat_x;
	  teapotPacket.components.y = fifoBuffer.data.quat_y;
	  teapotPacket.components.z = fifoBuffer.data.quat_z;
          Serial.write(teapotPacket.buff, 14);
          teapotPacket.components.cnt++; // packetCount, loops at 0xFF on purpose
      #endif

      #ifdef OUTPUT_TEAPOT_UDP
      // display quaternion values in InvenSense Teapot demo format:
	  teapotPacket.components.w = fifoBuffer.data.quat_w;
	  teapotPacket.components.x = fifoBuffer.data.quat_x;
	  teapotPacket.components.y = fifoBuffer.data.quat_y;
	  teapotPacket.components.z = fifoBuffer.data.quat_z;
          if (packet_pos < NUM_PACKETS) {
            memcpy(udpPackets + packet_pos * NUM_PACKETS, teapotPacket.buff, sizeof(teapot_t));
            packet_pos ++;
          } else {
            packet_pos = 0;
            udpSender.beginPacket(UDP_SERVER, SEND_PORT);
            udpSender.write(udpPackets, 14 * NUM_PACKETS);
            udpSender.endPacket();
          }

          teapotPacket.components.cnt++; // packetCount, loops at 0xFF on purpose
      #endif

      #ifdef UDP_SEND_COMPRESSED
	  uint32_t currentTime = millis();

          if (currentTime > (compressedSend + SEND_RATE)) {
            compressedPacket.data.time = currentTime;
            mpu.dmpGetQuaternion(compressedPacket.data.quat, fifoBuffer.buff);
            compressedPacket.data.cnt = compressedPacketCount++;

            udpSender.beginPacket(UDP_SERVER, SEND_PORT);
            udpSender.write(compressedPacket.buff, sizeof(udp_data_t));
            udpSender.endPacket();
            compressedSend = currentTime;
          }

      #endif
  }
  
#if 0
  else {
    /* interrupt occured without the DMP_INT bit (Bit 1) set
     * this is an error
     */
     //LOG(LOG_WARNING, "interrupt error");
     intErrors++;
  }
  if (intErrors >= MAX_INT_ERRORS) {
    /* try reboot */
    LOG(LOG_ERR, "Too many erroneous interrupts - reboot");
    //ESP.reset();
    intErrors = 0;
  }
#endif
}
