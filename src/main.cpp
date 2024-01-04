#include <Arduino.h>
#include <ModbusRtu.h>
#include "Wire.h"
#include <ArduinoJson.h>
#include <StreamUtils.h>

byte inputs = 0;
unsigned long interval = 500;
unsigned long previousMillis = 0;

unsigned long intervalEncoder = 1000;
unsigned long previousMillisEncoder = 0;

int newValue;

const int M1_Lora = 5;
String idDevice = "WSL0001";
int volumeRange[10] = {450, 400, 350, 300, 250, 200, 150, 100, 50, 0};
int volumeSet;

#define A_PHASE 14
#define B_PHASE 15
int value = 0; // Assign a value to the token bit
int temp = 0;

int volumeSend = 0;
float tempSend;
float phSend;
int tdsSend;
int ecSend;
int turbSend;

void interruptA()
{
  if (digitalRead(B_PHASE) == LOW)
  {
    value++;
    if (value > 600)
    {
      newValue = newValue + 1;
      value = 0;
    }
  }
  else
  {
    value++;
    if (value > 600)
    {
      value = 0;
      newValue = newValue - 1;
    }
    if (value < 0)
    {
      value = 0;
    }
  }
}

void interruptB()
{
  if (digitalRead(A_PHASE) == LOW)
  {
    value++;
    if (value > 600)
    {
      value = 0;
      newValue = newValue - 5;
    }
    if (value < 0)
    {
      value = 0;
    }
  }
  else
  {
    value++;
    if (value > 600)
    {
      newValue = newValue + 5;
      value = 0;
    }
    // value = value / 20;
  }
}

Modbus master(0, Serial1); // this is master and RS-232 or USB-FTDI

#define RS485_RX 16
#define RS485_TX 17

uint16_t modBusTX(byte slave, byte fungsi, uint16_t alamatAwal, uint16_t jumlah);
uint16_t calcCRC(byte *data, byte panjang);

uint16_t modBusTX(byte slave, byte fungsi, uint16_t alamatAwal, uint16_t jumlah)
{
  byte slaveAddress = slave;
  byte functionCode = fungsi;
  uint16_t startAddress = alamatAwal;
  uint16_t numberOfPoints = jumlah;
  byte frame[8];
  frame[0] = slaveAddress;
  frame[1] = functionCode;
  frame[2] = (uint16_t)(startAddress >> 8);
  frame[3] = (uint16_t)startAddress;
  frame[4] = (uint16_t)(numberOfPoints >> 8);
  frame[5] = (uint16_t)numberOfPoints;
  uint16_t crc = calcCRC(frame, sizeof(frame));
  frame[6] = lowByte(crc);
  frame[7] = highByte(crc);
  for (int i = 0; i < sizeof(frame); i++)
  {
    Serial1.write(frame[i]);
  }
}

uint16_t calcCRC(byte *data, byte panjang)
{
  int i;
  uint16_t crc = 0xFFFF;
  for (byte p = 0; p < panjang - 2; p++)
  {
    crc ^= data[p];
    for (i = 0; i < 8; ++i)
    {
      if (crc & 0x01)
        crc = (crc >> 1) ^ 0xA001;
      else
        crc = (crc >> 1);
    }
  }
  return crc;
}

float dataSensorPH_PH = 0;
float dataSensorPH_Temp = 0;

void bacaSensorPH()
{
  char inChar;
  modBusTX(1, 3, 0, 2);
}

uint16_t au16data[7]; //!< data array for modbus network sharing
uint8_t u8state;      //!< machine state
uint8_t u8query;      //!< pointer to message query
modbus_t polling[7];
// modbus_t polling[2]; for PH
unsigned long u32wait;

void setup()
{
  Serial.begin(115200);
  Serial1.begin(9600, SERIAL_8N1, 16, 17); //, SERIAL_8N1, RS485_RX, RS485_TX);
  // Serial2.begin(19200, SERIAL_8N1, 16, 17);

  // for tds-temp
  polling[0].u8id = 1;           // slave address
  polling[0].u8fct = 3;          // function code (this one is write a single register)
  polling[0].u16RegAdd = 0;      // start address in slave
  polling[0].u16CoilsNo = 1;     // number of elements (coils or registers) to read
  polling[0].au16reg = au16data; // pointer to a memory array in the Arduino

  // for tds-ec
  polling[1].u8id = 1;               // slave address
  polling[1].u8fct = 3;              // function code (this one is write a single register)
  polling[1].u16RegAdd = 2;          // start address in slave
  polling[1].u16CoilsNo = 1;         // number of elements (coils or registers) to read
  polling[1].au16reg = au16data + 1; // pointer to a memory array in the Arduino

  // for tds-salinity
  polling[2].u8id = 1;
  polling[2].u8fct = 3;
  polling[2].u16RegAdd = 3;
  polling[2].u16CoilsNo = 1;
  polling[2].au16reg = au16data + 2;

  // for tds-tds
  polling[3].u8id = 1;
  polling[3].u8fct = 3;
  polling[3].u16RegAdd = 4;
  polling[3].u16CoilsNo = 1;
  polling[3].au16reg = au16data + 3;

  // for PH
  polling[4].u8id = 2;
  polling[4].u8fct = 3;
  polling[4].u16RegAdd = 0;
  polling[4].u16CoilsNo = 1;
  polling[4].au16reg = au16data + 4;

  polling[5].u8id = 2;
  polling[5].u8fct = 3;
  polling[5].u16RegAdd = 1;
  polling[5].u16CoilsNo = 1;
  polling[5].au16reg = au16data + 5;

  polling[6].u8id = 3;
  polling[6].u8fct = 3;
  polling[6].u16RegAdd = 0;
  polling[6].u16CoilsNo = 1;
  polling[6].au16reg = au16data + 6;

  master.start();
  master.setTimeOut(5000); // if there is no answer in 5000 ms, roll over

  u32wait = millis() + 1000;
  u8state = u8query = 0;

  pinMode(A_PHASE, INPUT_PULLUP);
  pinMode(B_PHASE, INPUT_PULLUP);
  Wire.begin();
  Wire.write(0x01); // IODIRA register

  pinMode(M1_Lora, OUTPUT);
  digitalWrite(M1_Lora, LOW);

  attachInterrupt(A_PHASE, interruptA, RISING); // Interrupt trigger mode: RISING
  attachInterrupt(B_PHASE, interruptB, RISING);
}

void dataUplink()
{

  volumeSend = newValue;

  StaticJsonDocument<192> dataUp;
  dataUp["Type"] = "Volume";
  dataUp["SN"] = idDevice;
  dataUp["Data0"] = String(volumeSend);
  dataUp["Data1"] = String(tempSend);
  dataUp["Data2"] = String(phSend);
  dataUp["Data3"] = String(tdsSend);
  dataUp["Data4"] = String(ecSend);
  dataUp["Data5"] = String(turbSend);
  serializeJson(dataUp, Serial2);
}

void dataDownlink()
{
  if (Serial2.available() > 0)
  {
    StaticJsonDocument<200> datadown;
    DeserializationError error = deserializeJson(datadown, Serial2);
    if (error)
    {
      Serial.print("deserializeJson() failed Downlink: ");
      Serial.println(error.c_str());
      return;
    }
    int list = datadown["list"];           // 1
    const char *gwyid = datadown["gwyid"]; // "MELGSL001"

    const char *downlink_0_id = datadown["downlink"][0]["id"];       // "WSL0001"
    const char *downlink_0_gwyid = datadown["downlink"][0]["gwyid"]; // "MELGSL001"
    if (String(downlink_0_id) == idDevice)
    {
      Serial.print("ID :");
      Serial.println(downlink_0_id);
      dataUplink();
    }
  }
}

void loop()
{
  // dataDownlink();
  // if (newValue > (volumeSet + 50))
  // {
  //   newValue = newValue - 15;
  // }
  // if (newValue < volumeSet)
  // {
  //   newValue = newValue + 10;
  // }

  switch (u8state)
  {
  case 0:
    if (millis() > u32wait)
      u8state++; // wait state
    break;
  case 1:
    master.query(polling[u8query]); // send query (only once)
    u8state++;
    u8query++;
    if (u8query > sizeof(au16data))
      u8query = 0;
    break;
  case 2:
    master.poll(); // check incoming messages
    if (master.getState() == COM_IDLE)
    {
      u8state = 0;
      u32wait = millis() + 1000;

      // Serial.println(au16data[0]); // Or do something else!
      uint8_t dataH1 = (uint8_t)((au16data[0] & 0xFF00) >> 8);
      uint8_t dataL1 = (uint8_t)(au16data[0] & 0x00FF);
      float tds_temp = (dataH1 * 256 + dataL1) / 100.0;
      tempSend = tds_temp;
      Serial.printf("tds_temp: %f", tds_temp);

      uint8_t dataH2 = (uint8_t)((au16data[1] & 0xFF00) >> 8);
      uint8_t dataL2 = (uint8_t)(au16data[1] & 0x00FF);
      int tds_ec = (dataH2 * 256 + dataL2);
      ecSend = tds_ec;
      Serial.printf(" tds_ec: %d", tds_ec);

      uint8_t dataH3 = (uint8_t)((au16data[2] & 0xFF00) >> 8);
      uint8_t dataL3 = (uint8_t)(au16data[2] & 0x00FF);
      int tds_saliniity = (dataH3 * 256 + dataL3);
      Serial.printf(" tds_saliniity: %d", tds_saliniity);

      uint8_t dataH4 = (uint8_t)((au16data[3] & 0xFF00) >> 8);
      uint8_t dataL4 = (uint8_t)(au16data[3] & 0x00FF);
      int tds_tds = (dataH4 * 256 + dataL4);
      tdsSend = tds_tds;
      Serial.printf(" tds_tds: %d", tds_tds);

      // Serial.println(au16data[0]); // Or do something else!
      uint8_t dataH5 = (uint8_t)((au16data[4] & 0xFF00) >> 8);
      uint8_t dataL5 = (uint8_t)(au16data[4] & 0x00FF);
      float temp = (dataH5 * 256 + dataL5) / 100.0;
      Serial.printf(" ph-TEMP: %f", temp);

      uint8_t dataH6 = (uint8_t)((au16data[5] & 0xFF00) >> 8);
      uint8_t dataL6 = (uint8_t)(au16data[5] & 0x00FF);
      float ph = (dataH6 * 256 + dataL6) / 100.0;
      phSend = ph;
      Serial.printf(" ph-PH: %f", ph);

      uint8_t dataH7 = (uint8_t)((au16data[6] & 0xFF00) >> 8);
      uint8_t dataL7 = (uint8_t)(au16data[6] & 0x00FF);
      float turbidity = (dataH7 * 256 + dataL7) / 10.0;
      Serial.printf(" turbidity: %f", turbidity);

      Serial.println();
    }
    break;
  }

  // if (millis() - previousMillisEncoder > intervalEncoder)
  // {

  //   // noInterrupts();
  //   Serial.print("Value : ");
  //   Serial.print(value);
  //   Serial.print(" New Value : ");
  //   Serial.println(newValue);
  //   // interrupts();
  //   previousMillisEncoder = millis();
  // }

  // if (millis() - previousMillis > interval)
  // {
  //   for (int i = 0; i < 8; i++)
  //   {
  //     Wire.beginTransmission(0x24);
  //     Wire.write(0x12); // set MCP23017 memory pointer to GPIOB address
  //     Wire.endTransmission();
  //     Wire.requestFrom(0x24, 1); // request one byte of data from MCP20317
  //     inputs = Wire.read();      // store the incoming byte into "inputs"
  //     if (bitRead(inputs, i) == 0)
  //     {
  //       volumeSet = volumeRange[i];
  //       volumeSend = volumeSet;
  //       newValue = volumeSet;
  //       // Serial.println(volumeSend);
  //     }
  //     // Serial.print(bitRead(inputs, i));
  //   }

  //   for (int i = 0; i < 8; i++)
  //   {
  //     Wire.beginTransmission(0x24);
  //     Wire.write(0x13); // set MCP23017 memory pointer to GPIOB address
  //     Wire.endTransmission();
  //     Wire.requestFrom(0x24, 1); // request one byte of data from MCP20317
  //     inputs = Wire.read();      // store the incoming byte into "inputs"
  //     if (bitRead(inputs, i) == 0)
  //     {
  //       volumeSet = volumeRange[i + 8];

  //       newValue = volumeSet;
  //       volumeSend = newValue;

  //     }

  //   }

  //   previousMillis = millis();
  // }
}
