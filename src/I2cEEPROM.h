/* Write and read long data string to I2C EEPROM
   maximum is 65536 bit. i.e. address stops at 65535.
   Extra data will wrap over to address 0

   Rongzhong Li
   August 2018

   Copyright (c) 2018 Petoi LLC.

  The MIT license

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
*/
#ifdef M5CORE2
#include <EEPROM.h>   //Use the EEPROM library to store the calibration parameters this is created at the top level (OpenCatEsp32.ino)
#else //BiBoard and BiBoard2
#include <Wire.h>
#define DEVICE_ADDRESS 0x54    //Address of eeprom chip
#endif

#define WIRE_BUFFER 30 //Arduino wire allows 32 byte buffer, with 2 byte for address.
#define WIRE_LIMIT 16 //That leaves 30 bytes for data. use 16 to balance each writes
#define PAGE_LIMIT 32 //AT24C32D 32-byte Page Write Mode. Partial Page Writes Allowed
#define SIZE 65536/16
#define EEPROM_SIZE (65536/16)
bool EEPROMOverflow = false;



#define EEPROM_BIRTHMARK_ADDRESS 0
#define EEPROM_IMU 1
#define EEPROM_CALIB 20
#define EEPROM_BLE_NAME 36
#define EEPROM_RESERVED 50
#define SERIAL_BUFF 100



void i2cDetect() {  
#ifdef M5CORE2
PTL("In i2cDetect");
  //Initialize the EEPROM to size = EEPROM_SIZE
if (!EEPROM.begin(EEPROM_SIZE)) {  //Request storage of SIZE size(success return 1).  )
    Serial.println(
        "\nFailed to initialise EEPROM!");
    delay(1000000); 
} 
else{
  Serial.println("EEPROM Config OK");
  //Serial.println("Testing EEPROM");
  //EEPROM.writeUChar(EEPROM_CALIB, 0x10);
  //Serial.print("C0: ");
  //Serial.println(EEPROM.readUChar(EEPROM_CALIB));
}
#else
  Wire.begin();
  byte error, address;
  int nDevices;

  Serial.println("Scanning I2C network...");
  nDevices = 0;
  for (address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("- I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error == 4)
    {
      Serial.print("- Unknown error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("- No I2C devices found");
  else
    Serial.println("- done");
#endif
}

void i2c_eeprom_write_byte( unsigned int eeaddress, byte data ) {
  
#ifdef M5CORE2
  EEPROM.writeUChar(eeaddress, data);
  EEPROM.commit();
#else
  int rdata = data;
  Wire.beginTransmission(DEVICE_ADDRESS);
  Wire.write((int)(eeaddress >> 8)); // MSB
  Wire.write((int)(eeaddress & 0xFF)); // LSB
  Wire.write(rdata);
  Wire.endTransmission();
  delay(5);  // needs 5ms for write
#endif
}

byte i2c_eeprom_read_byte( unsigned int eeaddress ) {
  byte rdata = 0xFF;
#ifdef M5CORE2
  rdata = EEPROM.readUChar(eeaddress);
#else
  Wire.beginTransmission(DEVICE_ADDRESS);
  Wire.write((int)(eeaddress >> 8)); // MSB
  Wire.write((int)(eeaddress & 0xFF)); // LSB
  Wire.endTransmission();
  Wire.requestFrom(DEVICE_ADDRESS, 1);
  if (Wire.available()) rdata = Wire.read();
#endif
  return rdata;
}


//This function will write a 2-byte integer to the EEPROM at the specified address and address + 1
void i2c_eeprom_write_int16(unsigned int eeaddress, int16_t p_value)
{
#ifdef M5CORE2
  EEPROM.writeShort(eeaddress, p_value);
  EEPROM.commit();
#else  
  byte lowByte = ((p_value >> 0) & 0xFF);
  byte highByte = ((p_value >> 8) & 0xFF);
  Wire.beginTransmission(DEVICE_ADDRESS);
  Wire.write((int)(eeaddress >> 8)); // MSB
  Wire.write((int)(eeaddress & 0xFF)); // LSB
  Wire.write(lowByte);
  Wire.write(highByte);
  Wire.endTransmission();
  delay(5);  // needs 5ms for write

  //  EEPROM.update(p_address, lowByte);
  //  EEPROM.update(p_address + 1, highByte);
#endif
}

//This function will read a 2-byte integer from the EEPROM at the specified address and address + 1
int16_t i2c_eeprom_read_int16(unsigned int eeaddress)
{
#ifdef M5CORE2
  int16_t dataRead = EEPROM.readShort(eeaddress);
  //Serial.print("Read Short:");
  //Serial.println(dataRead);  
  return(dataRead);
#else
  Wire.beginTransmission(DEVICE_ADDRESS);
  Wire.write((int)(eeaddress >> 8)); // MSB
  Wire.write((int)(eeaddress & 0xFF)); // LSB
  Wire.endTransmission();
  Wire.requestFrom(DEVICE_ADDRESS, 2);
  byte lowByte = Wire.read();
  byte highByte = Wire.read();
  return (int16_t((lowByte << 0) & 0xFF) + ((highByte << 8) & 0xFF00));
#endif   
}


void i2c_eeprom_read_buffer( unsigned int eeaddress, byte *buffer, int length ) {
#ifdef M5CORE2
  EEPROM.readBytes(eeaddress,buffer,length);
  PTL(buffer[0]);
  buffer[0] = EEPROM.readUChar(eeaddress);
  PTL(buffer[0]);
#else
  Wire.beginTransmission(DEVICE_ADDRESS);
  Wire.write((int)(eeaddress >> 8)); // MSB
  Wire.write((int)(eeaddress & 0xFF)); // LSB
  Wire.endTransmission();
  Wire.requestFrom(DEVICE_ADDRESS, length);
  int c = 0;
  for ( c = 0; c < length; c++ ) {
    if (Wire.available()) buffer[c] = Wire.read();
    //    PT((char)buffer[c]);
  }
#endif
}


void writeLong(unsigned int eeAddress, char *data, int len) {
  //byte locationInPage = eeAddress % PAGE_LIMIT;
  i2c_eeprom_write_byte(eeAddress++, len);
  //  PTL("write " + String(len) + " bytes");
#ifdef M5CORE2
  EEPROM.writeBytes(eeAddress, data, len);
  EEPROM.commit();
#else
  int writtenToEE = 0;
  while (len > 0) {
    Wire.beginTransmission(DEVICE_ADDRESS);
    Wire.write((int)((eeAddress) >> 8));   // MSB
    Wire.write((int)((eeAddress) & 0xFF)); // LSB
    //    PT("* current address: ");
    //    PT((unsigned int)eeAddress);
    //    PTL("\t0 1 2 3 4 5 6 7 8 9 a b c d e f ");
    //    PT("\t\t\t\t");
    byte writtenToWire = 0;
    do
    {
      if (eeAddress == SIZE) {
        PTL();
        PTL("EEPROM overflow!\n");
      }
      //      PT(data[writtenToEE]);
      //      PT(" ");
      Wire.write((byte)data[writtenToEE]);
      writtenToWire++;
      writtenToEE++;
      eeAddress++;
      len--;
    } while (len > 0 && (eeAddress  % PAGE_LIMIT ) && writtenToWire < WIRE_LIMIT);
    Wire.endTransmission();
    delay(6);  // needs 5ms for page write
    //    PTL();
    //    PTL("wrote " + String(writtenToWire) + " bytes.");
  }
  //  PTL("finish writing");
#endif
}

void readLong(unsigned int eeAddress, char *data) {
  int len = i2c_eeprom_read_byte(eeAddress++);
  PTL("read " + String(len) + " bytes");
#ifdef M5CORE2
  i2c_eeprom_read_buffer(eeAddress, (byte*) data, len);  
#else
  int readFromEE = 0;
  int readToWire = 0;

  Wire.beginTransmission(DEVICE_ADDRESS);
  Wire.write((int)((eeAddress) >> 8));   // MSB
  Wire.write((int)((eeAddress) & 0xFF)); // LSB
  Wire.endTransmission();
  while (len > 0) {
    Wire.requestFrom(DEVICE_ADDRESS, min(WIRE_BUFFER, len));
    readToWire = 0;
    do {
      if (Wire.available()) data[readFromEE] = Wire.read();
      PT( (char)data[readFromEE]);
      readFromEE++;
    } while (--len > 0 && ++readToWire < WIRE_BUFFER);
    PTL();
  }
#endif
  PTL("finish reading");
}


bool newBoardQ(unsigned int eeaddress) {
  return i2c_eeprom_read_byte(eeaddress) != BIRTHMARK;
}

char data[] = " The quick brown fox jumps over the lazy dog. \
The five boxing wizards jump quickly. Pack my box with five dozen liquor jugs."; // data to write

//char data[]={16,-3,5,7,9};

void genBleID(int suffixDigits = 2) {
  const char *prefix = "Bittle";  
  /*
  const char *prefix =
#ifdef BITTLE
    "Bittle"
#elif defined NYBBLE
    "Nybble" 
#else
    "Cub"
#endif
*/
    ;
  int prelen = strlen(prefix);
  //  PTL(prelen);
  char * id = new char [prelen + suffixDigits + 1];
  strcpy(id, prefix);
  for (int i = 0; i < suffixDigits; i++) {
    int temp = esp_random() % 16;
    sprintf(id + prelen + i, "%X", temp);
  }
  id[prelen + suffixDigits] = '\0';
  //  for (int i = 0; i < prelen + suffixDigits; i++) {
  //    i2c_eeprom_write_byte(EEPROM_BLE_NAME + 1 + i, id[i]);
  //  }
  Serial.println(id);
  writeLong(EEPROM_BLE_NAME, id, prelen + suffixDigits);
}

char* readBleID() {
  int idLen = i2c_eeprom_read_byte(EEPROM_BLE_NAME);
  char * id = new char [idLen + 1];
  //  readLong(EEPROM_BLE_NAME, id);
  for (int i = 0; i < idLen; i++) {
    id[i] = i2c_eeprom_read_byte(EEPROM_BLE_NAME + 1 + i);
  }
  id[idLen] = '\0';
  Serial.print("Bluetooth name: ");
  Serial.println(id);
  return id;
  
}

int dataLen(int8_t p) {
  byte skillHeader = p > 0 ? 4 : 7;
  int frameSize = p > 1 ?
                  WALKING_DOF :       //gait
                  p == 1 ? DOF : //posture
                  DOF + 4;            //behavior
  int len = skillHeader + abs(p) * frameSize;
  return len;
}
void i2cEepromSetup()
{
#ifdef M5CORE2
#else
  Wire.begin(); // initialise the connection
  delay(1);
#endif
  newBoard = newBoardQ(EEPROM_BIRTHMARK_ADDRESS);

  if (newBoard) {
    PTLF("Set up the new board...");
    playMelody(melodyInit, sizeof(melodyInit) / 2);
    PTF("- Name the new robot as: ");
    genBleID();
#ifndef AUTO_INIT
    PTL("- Reset the joints' calibration offsets? (Y/n): ");
    while (!Serial.available());
    char choice = Serial.read();
    PTL(choice);
    if (choice == 'Y' || choice == 'y') {
#else
    PTL("- Reset the joints' calibration offsets...");
#endif
      for (byte c = 0; c < DOF; c++)
        i2c_eeprom_write_byte(EEPROM_CALIB + c, 0);
#ifndef AUTO_INIT
    }
#endif
  }
}
void copydataFromBufferToI2cEeprom(unsigned int eeAddress, int8_t *dataBuffer) {
  int len = dataLen(dataBuffer[0]) + 1;
#ifdef M5CORE2
  EEPROM.writeBytes(eeAddress, dataBuffer, len);
  EEPROM.commit();
#else
  int writtenToEE = 0;
  while (len > 0) {
    Wire.beginTransmission(DEVICE_ADDRESS);
    Wire.write((int)((eeAddress) >> 8));   // MSB
    Wire.write((int)((eeAddress) & 0xFF)); // LSB
    byte writtenToWire = 0;
    do {
      if (eeAddress == EEPROM_SIZE) {
        PTL();
        PTLF("I2C EEPROM overflow! Delete some skills!\n");
        EEPROMOverflow = true;
#ifdef BUZZER
        beep(10, 100, 50, 2);
#endif
        return;
      }
      Wire.write((byte)dataBuffer[writtenToEE++]);
      writtenToWire++;
      eeAddress++;
    } while ((--len > 0 ) && (eeAddress  % PAGE_LIMIT ) && (writtenToWire < WIRE_LIMIT));//be careful with the chained conditions
    //self-increment may not work as expected
    Wire.endTransmission();
    delay(6);  // needs 5ms for page write
    //    PTL("\nwrote " + String(writtenToWire) + " bytes.");
  }
  delay(6);
  //  PTLF("finish copying to I2C EEPROM");
#endif
}
void loadDataFromI2cEeprom(unsigned int eeAddress) {
#ifdef M5CORE2
  dataBuffer[0] = i2c_eeprom_read_byte(eeAddress);
  int bufferLen = dataLen(dataBuffer[0]);
  EEPROM.readBytes(eeAddress+1,&dataBuffer[1],bufferLen);
#else
  Wire.beginTransmission(DEVICE_ADDRESS);
  Wire.write((int)((eeAddress) >> 8));   // MSB
  Wire.write((int)((eeAddress) & 0xFF)); // LSB
  Wire.endTransmission();
  Wire.requestFrom((uint8_t)DEVICE_ADDRESS, (uint8_t)1);
  dataBuffer[0] = Wire.read();
  int bufferLen = dataLen(dataBuffer[0]);
  //      int tail = bufferLen;
  int readFromEE = 0;
  int readToWire = 0;
  while (bufferLen > 0) {
    //PTL("request " + String(min(WIRE_BUFFER, len)));
    Wire.requestFrom((uint8_t)DEVICE_ADDRESS, (uint8_t)min(WIRE_BUFFER, bufferLen));
    readToWire = 0;
    do {
      if (Wire.available()) dataBuffer[1 + readFromEE++] = Wire.read();
      //      PT( (int8_t)dataBuffer[readFromEE - 1]);
      //      PT('\t');
    } while (--bufferLen > 0 && ++readToWire < WIRE_BUFFER);
    //    PTL();
  }
  //      dataBuffer[tail] = '\0';
#endif
}

void saveCalib(int8_t *var) {
  PTL("In saveCalib");
  for (byte s = 0; s < DOF; s++) {
    //i2c_eeprom_write_byte(EEPROM_CALIB + s, var[s]);
    EEPROM.write(EEPROM_CALIB + s, var[s]);
    PTL(var[s]);
    calibratedZeroPosition[s] = zeroPosition[s] + float(var[s])  * rotationDirection[s];
  }
  EEPROM.commit();
  delay(100);
}
