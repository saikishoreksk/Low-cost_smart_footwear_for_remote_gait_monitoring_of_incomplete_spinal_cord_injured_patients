// Ensure correct credentials to connect to your WiFi Network.
char ssid[] = "*****";
char pass[] = "*****";
// Ensure that the credentials here allow you to publish and subscribe to the ThingSpeak channel.
#define channelID *****
const char mqttUserName[] = "*****"; 
const char clientID[] = "*****";
const char mqttPass[] = "*****";

#define ESP8266BOARD

#include <PubSubClient.h>
  #include <ESP8266WiFi.h>
  
#ifdef USESECUREMQTT
  #include <WiFiClientSecure.h>
  #define mqttPort 8883
  WiFiClientSecure client; 
#else
  #define mqttPort 1883
  WiFiClient client;
#endif


#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

const char* server = "mqtt3.thingspeak.com";
int status = WL_IDLE_STATUS; 
long lastPublishMillis = 0;
int connectionDelay = 1;
int updateInterval = 2;
PubSubClient mqttClient( client );

MPU6050 mpu;

bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

Quaternion q;
VectorInt16 aa;
VectorInt16 aaReal;
VectorInt16 aaWorld;
VectorFloat gravity;

#define INTERRUPT_PIN 15 // use pin 15 on ESP8266

int pin=A0;
int sensor;
bool collectData = false;
bool datatransfer=false;

volatile bool mpuInterrupt = false;

float velocityX = 0.0;
float velocityY = 0.0;
float velocityZ = 0.0;
float displacementX = 0.0;
float displacementY = 0.0;
float displacementZ = 0.0;
float previousAccelerationX = 0.0;
float previousAccelerationY = 0.0;
float previousAccelerationZ = 0.0;

float gaitspeed=0.0;
float stridelength=0.0;
float cadence=0.0; 
float samplecount=0.0;
float stridewidth=0.0;

void ICACHE_RAM_ATTR dmpDataReady() {
    mpuInterrupt = true;
}

void mpu_setup() {
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000);
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);
    

    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();
    mpu.setXGyroOffset(90);//90R,71L
    mpu.setYGyroOffset(5);//5R,-92
    mpu.setZGyroOffset(7);//7R,20L
    mpu.setZAccelOffset(1339);//1339R,1324L

    if (devStatus == 0) {
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        
        mpuIntStatus = mpu.getIntStatus();

        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
}


void mpu_loop() {
    static unsigned long prevTime = 0;

    if (!dmpReady) return;

    if (!mpuInterrupt && fifoCount < packetSize) return;

    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();

    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));
    } else if (mpuIntStatus & 0x02) {
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;

        mpu.dmpGetQuaternion(&q, fifoBuffer);//orientation of the sensor in quaternion form
        mpu.dmpGetAccel(&aa, fifoBuffer);//accelerometer data is from the calibrated sensors
        mpu.dmpGetGravity(&gravity, &q);//gravity component in different degree of freedom is achieved
        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);//gravity component in different degree of freedom is removed
        mpu.dmpConvertToWorldFrame(&aaWorld, &aaReal, &q);//acceleration values are projected from sensor frame to global frame

         unsigned long currentTime = micros();
            unsigned long timeInterval = currentTime - prevTime;
            prevTime = currentTime;

            // Convert time interval to seconds
            float timeIntervalSeconds = timeInterval / 1000000.0;
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            mpu.dmpConvertToWorldFrame(&aaWorld, &aaReal, &q);
            aaReal.x=aaWorld.x*9.8/8192;
            aaReal.y=aaWorld.y*9.8/8192;
            aaReal.z=aaWorld.z*9.8/8192;

            // Update displacement calculation
            displacementX += (0.25 * (abs(aaReal.x) + abs(previousAccelerationX)) * timeIntervalSeconds * timeIntervalSeconds) + (abs(velocityX) * timeIntervalSeconds);
            displacementY += (0.25 * (abs(aaReal.y) + abs(previousAccelerationY)) * timeIntervalSeconds * timeIntervalSeconds) + (abs(velocityY) * timeIntervalSeconds);
            displacementZ += (0.25 * (abs(aaReal.z) + abs(previousAccelerationZ)) * timeIntervalSeconds * timeIntervalSeconds) + (abs(velocityZ) * timeIntervalSeconds);

            //gaitspeed+=sqrt(velocityX*velocityX+velocityY*velocityY)*100;
            gaitspeed+=abs(velocityY);
            samplecount+=1.0;
            
            // Update instantaneous velocity calculation
            velocityX += (0.5 * (aaReal.x + previousAccelerationX) * timeIntervalSeconds);
            velocityY += (0.5 * (aaReal.y + previousAccelerationY) * timeIntervalSeconds);
            velocityZ += (0.5 * (aaReal.z + previousAccelerationZ) * timeIntervalSeconds);

            // Output data
            Serial.print(aaReal.x);
            Serial.print(",");
            Serial.print(aaReal.y);
            Serial.print(",");
            Serial.print(aaReal.z);
            Serial.print(",");
            Serial.print(velocityX);
            Serial.print(",");
            Serial.print(velocityY);
            Serial.print(",");
            Serial.print(velocityZ);
            Serial.print(",");
            Serial.print(displacementX*100);
            Serial.print(",");
            Serial.print(displacementY*100);
            Serial.print(",");
            Serial.print(displacementZ*100);
            Serial.println();

            // Update previous acceleration values
            previousAccelerationX = aaReal.x;
            previousAccelerationY = aaReal.y;
            previousAccelerationZ = aaReal.z;
    }
}

void mqttPublish(long pubChannelID, String message) {
  String topicString ="channels/" + String( pubChannelID ) + "/publish";
  mqttClient.publish( topicString.c_str(), message.c_str() );
}

// Connect to WiFi.
void connectWifi()
{
  Serial.print( "Connecting to Wi-Fi..." );
  // Loop until WiFi connection is successful
  #ifdef ESP8266BOARD
    while ( WiFi.waitForConnectResult() != WL_CONNECTED ) {
  #else
    while ( WiFi.status() != WL_CONNECTED ) {
  #endif
    WiFi.begin( ssid, pass );
    delay( connectionDelay*1000 );
    Serial.print( WiFi.status() ); 
  }
  Serial.println( "Connected to Wi-Fi." );
}

// Connect to MQTT server.
void mqttConnect() {
  // Loop until connected.
  while ( !mqttClient.connected() )
  {
    // Connect to the MQTT broker.
    if ( mqttClient.connect( clientID, mqttUserName, mqttPass ) ) {
      Serial.print( "MQTT to " );
      Serial.print( server );
      Serial.print (" at port ");
      Serial.print( mqttPort );
      Serial.println( " successful." );
    } else {
      Serial.print( "MQTT connection failed, rc = " );
      Serial.print( mqttClient.state() );
      Serial.println( " Will try again in a few seconds" );
      delay( connectionDelay*1000 );
    }
  }
}

void setup() {
  Serial.begin( 115200 );
  // Delay to allow serial monitor to come up.
  delay(3000);
  // Connect to Wi-Fi network.
  connectWifi();
  // Configure the MQTT client
  mqttClient.setServer( server, mqttPort ); 
  // Set the MQTT message handler function.
  mqttClient.setCallback( mqttSubscriptionCallback );
  // Set the buffer to handle the returned JSON. NOTE: A buffer overflow of the message buffer will result in your callback not being invoked.
  mqttClient.setBufferSize( 2048 );

    mpu_setup();
}

void loop(void) {
sensor=analogRead(pin);

    if (sensor<5) {
        collectData = true;
        
    } 
if (sensor>5 && datatransfer==false) {
        collectData = false;
        datatransfer = true;
    }

    if (!collectData&&datatransfer) {
     stridelength=displacementY*100;
  stridewidth=displacementX*100;
      gaitspeed=gaitspeed*100/samplecount;
      cadence=60*2*gaitspeed/displacementY;

      if (WiFi.status() != WL_CONNECTED) {
      connectWifi();
      }
      // Connect if MQTT client is not connected and resubscribe to channel updates.
      if (!mqttClient.connected()) {
        mqttConnect(); 
      }
      mqttSubscribe( channelID );
      
      // Call the loop to maintain connection to the server.
      mqttClient.loop(); 
     
      mqttPublish( channelID, (String("field1=")+String(stridelength)+String("&")+String("field2=")+String(gaitspeed)+String("&")+String("field3=")+String(cadence)+String("&")+String("field4=")+String(stridewidth)));

      Serial.print(stridelength);
      Serial.print(",");
      Serial.print(gaitspeed);
      Serial.print(",");
      Serial.println(cadence);
      
      datatransfer=false;
        // Reset velocity and displacement values to zero
        velocityX = 0.0;
        velocityY = 0.0;
        velocityZ = 0.0;
        displacementX = 0.0;
        displacementY = 0.0;
        displacementZ = 0.0;

        previousAccelerationX = 0.0;
        previousAccelerationY = 0.0;
        previousAccelerationZ = 0.0;

        gaitspeed=0.0;
        stridelength=0.0;
        cadence=0.0;
        samplecount=0.0;
         stridewidth=0.0;
    }

    if (collectData) {
        mpu_loop();
    }

}
