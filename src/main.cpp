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

MPU6050 mpu;

WiFiUDP udpSyslogClient;
WiFiUDP udpSender;
WiFiUDP udpControl;
Syslog syslog(udpSyslogClient, CONTROLLER, SYSLOG_PORT, HOSTNAME, APP_NAME, LOG_KERN);

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

uint16_t packetSize;  // expected DMP packet size (default is 42 bytes)
bool dmpReady = false;


typedef struct {
  fifo_t fifo;
  unsigned long timestamp;
} fifo_entry_t;

#define FIFO_RING_LEN 25

volatile uint8_t fifo_pos = 0;
fifo_entry_t fifo_ring[FIFO_RING_LEN];
volatile uint16_t intErrors= 0;


void ICACHE_RAM_ATTR readDataISR() {
  uint8_t mpuIntStatus = mpu.getIntStatus();
  uint16_t fifoCount = mpu.getFIFOCount();

  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
      // reset so we can continue cleanly
      mpu.resetFIFO();
      LOG(LOG_WARNING, "FIFO overflow!");

      /* reboot when this happens again too often  */
      intErrors++;

  // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
    // get current FIFO count
    do {
      volatile uint8_t nex_pos = (fifo_pos + 1) % FIFO_RING_LEN;

      /* reset int error counter */
      intErrors = 0;

      // wait for correct available data length, should be a VERY short wait
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

      // read a packet from FIFO
      mpu.getFIFOBytes(fifo_ring[nex_pos].fifo.buff, packetSize);
      fifo_ring[nex_pos].timestamp = millis();
      nex_pos = nex_pos;

      // track FIFO count here in case there is > 1 packet available
      // (this lets us immediately read more without waiting for an interrupt)
      fifoCount -= packetSize;
    } while (fifoCount != 0);
  }
}


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

  // MPU control/status vars
  uint8_t devStatus;    // return status after each device operation (0 = success,
                        // !0 = error)

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
    attachInterrupt(digitalPinToInterrupt(MPU_INT_PIN), readDataISR, RISING);

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
    WiFi.disconnect();
    ESP.restart();
  }

  uint32_t currentTime = millis();

  if (currentTime > (compressedSend + SEND_RATE)) {
    fifo_entry_t entry;
    memcpy(&entry, &(fifo_ring[fifo_pos]), sizeof(fifo_entry_t));

    if ((entry.timestamp + MAX_INT_WAIT_MS) > currentTime) {
      LOG(LOG_ERR, "Interrupt is dead - reboot");
      WiFi.disconnect();
      ESP.restart();
    }

    if (intErrors > MAX_INT_ERRORS) {
      LOG(LOG_ERR, "Too many erroneous interrupts - reboot");
      delay(10);
      WiFi.disconnect();
      ESP.restart();

    }

    compressedPacket.data.time = entry.timestamp;
    mpu.dmpGetQuaternion(compressedPacket.data.quat, entry.fifo.buff);
    compressedPacket.data.cnt = compressedPacketCount++;

    udpSender.beginPacket(UDP_SERVER, SEND_PORT);
    udpSender.write(compressedPacket.buff, sizeof(udp_data_t));
    udpSender.endPacket();
    compressedSend = currentTime;
  }
}
