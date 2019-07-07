#ifndef _ESP32_ANALOG_WRITE_
#define _ESP32_ANALOG_WRITE_
#include <Arduino.h>
#include "esp_wpa2.h"
#include <inttypes.h>
#include <Stream.h>
#include <WiFiUdp.h>
#include <WiFi.h>
#include <HTTPClient.h>

#pragma region "Analog write"
typedef struct analog_write_channel
{
  int8_t pin;
  double frequency;
  uint8_t resolution;
} analog_write_channel_t;

int analogWriteChannel(uint8_t pin);

void analogWriteFrequency(double frequency);
void analogWriteFrequency(uint8_t pin, double frequency);

void analogWriteResolution(uint8_t resolution);
void analogWriteResolution(uint8_t pin, uint8_t resolution);

void analogWrite(uint8_t pin, uint32_t value, uint32_t valueMax = 255);

#endif
#ifndef SoftwareSerial_h
#define SoftwareSerial_h

// This class is compatible with the corresponding AVR one,
// the constructor however has an optional rx buffer size.
// Speed up to 115200 can be used.

class SoftwareSerial : public Stream
{
  public:
    SoftwareSerial(int receivePin, int transmitPin, bool inverse_logic = false, unsigned int buffSize = 64);
    ~SoftwareSerial();

    void begin(long speed);
    long baudRate();
    void setTransmitEnablePin(int transmitEnablePin);

    bool overflow();
    int peek();

    virtual size_t write(uint8_t byte);
    virtual int read();
    virtual int available();
    virtual void flush();
    operator bool()
    {
      return m_rxValid || m_txValid;
    }

    // Disable or enable interrupts on the rx pin
    void enableRx(bool on);

    void rxRead();

    // AVR compatibility methods
    bool listen()
    {
      enableRx(true);
      return true;
    }
    void end()
    {
      stopListening();
    }
    bool isListening()
    {
      return m_rxEnabled;
    }
    bool stopListening()
    {
      enableRx(false);
      return true;
    }

    using Print::write;

  private:
    bool isValidGPIOpin(int pin);

    // Member variables
    int m_rxPin, m_txPin, m_txEnablePin;
    bool m_rxValid, m_rxEnabled;
    bool m_txValid, m_txEnableValid;
    bool m_invert;
    bool m_overflow;
    unsigned long m_bitTime;
    unsigned int m_inPos, m_outPos;
    int m_buffSize;
    uint8_t *m_buffer;
};

// If only one tx or rx wanted then use this as parameter for the unused pin
#define SW_SERIAL_UNUSED_PIN -1

#endif

#define _ESP32_ANALOG_WRITE_
#define SoftwareSerial_h
#pragma endregion

int incomingByte = 0;

int sensorValue;

uint32_t speed = 1023; //0 to 1023

uint8_t enA = 16; //d0
int in1 = 5;      //d1
int in2 = 4;      //d2

uint8_t enB = 14; //d5
int in3 = 18;     //d3
int in4 = 2;      //d4

//wifi stuff
const char *ssid = "robtest";        // wifi network name
const char *password = "iliketoast"; // wifi network password
//const char* ssid = "eduroam"; // your ssid

WiFiUDP Udp;
unsigned int localUdpPort = 1998;
char incomingPacket[255];
WiFiClient client;
WiFiServer server(80);
void setup()
{
  Serial.begin(115200);
  delay(10);
  Serial.println("Motor test!");
  client = server.available(); // Listen for incoming clients
  // We start by connecting to a WiFi network
  Serial.print("Connecting to ");
  Serial.println(ssid);
  /*
    WiFi.disconnect(true);  //disconnect form wifi to set new wifi connection
    WiFi.mode(WIFI_STA); //init wifi mode
    esp_wifi_sta_wpa2_ent_set_identity((uint8_t *)EAP_IDENTITY, strlen(EAP_IDENTITY)); //provide identity
    esp_wifi_sta_wpa2_ent_set_username((uint8_t *)EAP_IDENTITY, strlen(EAP_IDENTITY)); //provide username --> identity and username is same
    esp_wifi_sta_wpa2_ent_set_password((uint8_t *)EAP_PASSWORD, strlen(EAP_PASSWORD)); //provide password
    esp_wpa2_config_t config = WPA2_CONFIG_INIT_DEFAULT(); //set config settings to default
    esp_wifi_sta_wpa2_ent_enable(&config); //set config settings to enable function
  */
  WiFi.begin(ssid, password);
  //WiFi.begin("eduroam");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.println("Starting UDP");
  Udp.begin(localUdpPort);

  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
}
float xposition, yposition, angle;
void right()
{
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);

  analogWrite(enA, speed);

  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);

  analogWrite(enB, speed);


  xposition -= cosf(angle * 0.01745329f);
  yposition -= sinf(angle * 0.01745329f);
}
void left()
{

  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);

  analogWrite(enA, speed);

  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);

  analogWrite(enB, speed);


  xposition += cosf(angle * 0.01745329f);
  yposition += sinf(angle * 0.01745329f);
}

void forward()
{
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);

  analogWrite(enA, speed);

  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);

  analogWrite(enB, speed);
  if (angle <= 360)
  {
    angle += 1;
  }
  else
  {
    angle = 0;
  }
}

void backward()
{

  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);

  analogWrite(enA, speed);

  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);

  analogWrite(enB, speed);
  if (angle >= 0)
  {
    angle -= 1;
  }
  else
  {
    angle = 0;
  }
}

void stop()
{
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

void ListenPacketRoutine()
{
  //listen for packets
  int packetSize = Udp.parsePacket();
  if (packetSize)
  {
    int len = Udp.read(incomingPacket, 255);
    Serial.printf("UDP packet contents: %s\n", incomingPacket);
    if (incomingPacket[0] == 'f')
    {
      forward();
    }
    else if (incomingPacket[0] == 'b')
    {
      backward();
    }
    else if (incomingPacket[0] == 'l')
    {
      left();
    }
    else if (incomingPacket[0] == 'r')
    {
      right();
    }
    else if (incomingPacket[0] == 's')
    {
      stop();
    }
    else if (incomingPacket[0] == 'a')
    {
      String url = "http://192.168.137.1/luxdata/data.php?luminosity=" + String(sensorValue) + "&xposition=" + String(xposition) + "&yposition=" + String(yposition); //ip for the localhost database
      HTTPClient httpClient;
      //format your POST request.
      httpClient.begin(url);
      int httpResponseCode = httpClient.GET();
      if (httpResponseCode > 0)
      {
        //check for a return code - This is more for debugging.
        String response = httpClient.getString();
        Serial.println(httpResponseCode);
        Serial.println(response);
      }
      else
      {
        Serial.print("Error on sending post");
        Serial.println(httpResponseCode);
      }
      //closde the HTTP request.
      httpClient.end();

      Serial.print("Valor do sensor: ");
      Serial.println(sensorValue);
    }
  }
}
/*
  void ListenKeyboardRoutine()
  {

  if (Serial.available() > 0)
  {
    incomingByte = Serial.read();
  }

  switch (incomingByte)
  {
  case 's':
  {
    stop();
    Serial.println("Stop\n");
    incomingByte = '*';
  }

  break;

  case 'f':
  {
    forward();

    Serial.println("Forward\n");
    incomingByte = '*';
  }
  break;

  case 'b':
  {
    backward();

    Serial.println("Backward\n");
    incomingByte = '*';
  }
  break;

  case 'r':
  {
    right();
    Serial.println("Rotate Right\n");
    incomingByte = '*';
  }
  break;

  case 'l':
  {
    left();
    Serial.println("Rotate Left\n");
    incomingByte = '*';
  }
  break;

  case 'a':
  {

    sensorValue = analogRead(A0);
    delay(1000);
    Serial.print("Valor do sensor: ");
    Serial.println(sensorValue);
  }
  break;
  }
  }
*/
analog_write_channel_t _analog_write_channels[16] = {
  { -1, 5000, 13},
  { -1, 5000, 13},
  { -1, 5000, 13},
  { -1, 5000, 13},
  { -1, 5000, 13},
  { -1, 5000, 13},
  { -1, 5000, 13},
  { -1, 5000, 13},
  { -1, 5000, 13},
  { -1, 5000, 13},
  { -1, 5000, 13},
  { -1, 5000, 13},
  { -1, 5000, 13},
  { -1, 5000, 13},
  { -1, 5000, 13},
  { -1, 5000, 13}
};

int analogWriteChannel(uint8_t pin)
{
  int channel = -1;

  // Check if pin already attached to a channel
  for (uint8_t i = 0; i < 16; i++)
  {
    if (_analog_write_channels[i].pin == pin)
    {
      channel = i;
      break;
    }
  }

  // If not, attach it to a free channel
  if (channel == -1)
  {
    for (uint8_t i = 0; i < 16; i++)
    {
      if (_analog_write_channels[i].pin == -1)
      {
        _analog_write_channels[i].pin = pin;
        channel = i;
        ledcSetup(channel, _analog_write_channels[i].frequency, _analog_write_channels[i].resolution);
        ledcAttachPin(pin, channel);
        break;
      }
    }
  }

  return channel;
}

void analogWriteFrequency(double frequency)
{
  for (uint8_t i = 0; i < 16; i++)
  {
    _analog_write_channels[i].frequency = frequency;
  }
}

void analogWriteFrequency(uint8_t pin, double frequency)
{
  int channel = analogWriteChannel(pin);

  // Make sure the pin was attached to a channel, if not do nothing
  if (channel != -1 && channel < 16)
  {
    _analog_write_channels[channel].frequency = frequency;
  }
}

void analogWriteResolution(uint8_t resolution)
{
  for (uint8_t i = 0; i < 16; i++)
  {
    _analog_write_channels[i].resolution = resolution;
  }
}

void analogWriteResolution(uint8_t pin, uint8_t resolution)
{
  int channel = analogWriteChannel(pin);

  // Make sure the pin was attached to a channel, if not do nothing
  if (channel != -1 && channel < 16)
  {
    _analog_write_channels[channel].resolution = resolution;
  }
}

void analogWrite(uint8_t pin, uint32_t value, uint32_t valueMax)
{
  int channel = analogWriteChannel(pin);

  // Make sure the pin was attached to a channel, if not do nothing
  if (channel != -1 && channel < 16)
  {
    uint8_t resolution = _analog_write_channels[channel].resolution;
    uint32_t levels = pow(2, resolution);
    uint32_t duty = ((levels - 1) / valueMax) * min(value, valueMax);

    // write duty to LEDC
    ledcWrite(channel, duty);
  }
}

void loop()
{
  sensorValue = analogRead(A0) * 0.1;
  ListenPacketRoutine();
  //ListenKeyboardRoutine();
}
