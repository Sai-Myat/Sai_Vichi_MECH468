/*
  Upload Data to IoT Server ThingSpeak (https://thingspeak.com/):
  Support Devices: LoRa Shield + Arduino 
  
  Example sketch showing how to read Temperature and Humidity from DHT11 sensor,  
  Then send the value to LoRa Gateway, the LoRa Gateway will send the value to the 
  IoT server
  It is designed to work with the other sketch dht11_server. 
  modified 24 11 2016
  by Edwin Chen <support@dragino.com>
  Dragino Technology Co., Limited
*/

#include <SPI.h>
#include <RH_RF95.h>

RH_RF95 rf95;

#define dht_dpin A0 // Use A0 pin as Data pin for DHT11. 
byte bGlobalErr;
char dht_dat[5]; // Store Sensor Data
char node_id[3] = {1,1,1}; //LoRa End Node ID
float frequency = 860.0;
unsigned int count = 1;

//SDM (Sound Detection Module)
#include <SoftwareSerial.h>

const int soundPin = A1; //sound sensor attach to A1
char db_str[4]; //Store (dB) Sensor data

//For Internal Temperature Sensor and Servo
//#include <Servo.h>
//#include <ServoTimer2.h>

const int sensePin = A2;  //This is the Arduino Pin that will read the sensor output attach to A2
int sensorInput = 0;    //The variable we will use to store the sensor input
//double Voltage = 0;
double temp;        //The variable we will use to store temperature in degrees. 

int servoPin = 3;
//ServoTimer2 servo1;

//UV Parts
const int UVPin = A3; // UV 
float sensorVoltageUV; 
float sensorValueUV;
char uv_str[2];


//////////>>>>>STOP HERE_*_*_*_*_CHECK PIN CONNECTION AGAIN AND AGAIN<<<<<<<<<<////////

void setup()
{
    InitDHT();
    Serial.begin(9600);
//    servo1.attach(servoPin);// Define the servo pin
    
    if (!rf95.init())
        Serial.println("init failed");
    // Setup ISM frequency
    rf95.setFrequency(frequency);
    // Setup Power,dBm
    rf95.setTxPower(13);
    
    Serial.println("LoRa End Node Example --"); 
    Serial.println("    DHT11 Temperature and Humidity Sensor\n");
    Serial.print("LoRa End Node ID: ");

    for(int i = 0;i < 3; i++)
    {
        Serial.print(node_id[i],HEX);
    }
    Serial.println();
}

void InitDHT()
{
    pinMode(dht_dpin,OUTPUT);//Set A0 to output
    digitalWrite(dht_dpin,HIGH);//Pull high A0
}

//Get Sensor Data
void ReadDHT()
{
    bGlobalErr=0;
    byte dht_in;
    byte i;
        
    //pinMode(dht_dpin,OUTPUT);
    digitalWrite(dht_dpin,LOW);//Pull Low A0 and send signal
    delay(30);//Delay > 18ms so DHT11 can get the start signal
        
    digitalWrite(dht_dpin,HIGH);
    delayMicroseconds(40);//Check the high level time to see if the data is 0 or 1
    pinMode(dht_dpin,INPUT);
    // delayMicroseconds(40);
    dht_in=digitalRead(dht_dpin);//Get A0 Status
    
    //   Serial.println(dht_in,DEC);
    if(dht_in)
    {
        bGlobalErr=1;
        return;
    }
    delayMicroseconds(80);//DHT11 send response, pull low A0 80us
    dht_in=digitalRead(dht_dpin);
    
    if(!dht_in)
    {
        bGlobalErr=2;
        return;
    }
    delayMicroseconds(80);//DHT11 send response, pull low A0 80us
    for (i=0; i<5; i++)//Get sensor data
    dht_dat[i] = read_dht_dat();
    pinMode(dht_dpin,OUTPUT);
    digitalWrite(dht_dpin,HIGH);//release signal and wait for next signal
    byte dht_check_sum = dht_dat[0]+dht_dat[1]+dht_dat[2]+dht_dat[3];//calculate check sum
    if(dht_dat[4]!= dht_check_sum)//check sum mismatch
        {bGlobalErr=3;}
};

byte read_dht_dat(){
    byte i = 0;
    byte result=0;
    for(i=0; i< 8; i++)
    {
        while(digitalRead(dht_dpin)==LOW);//wait 50us
        delayMicroseconds(30);//Check the high level time to see if the data is 0 or 1
        if (digitalRead(dht_dpin)==HIGH)
        result |=(1<<(7-i));//
        while (digitalRead(dht_dpin)==HIGH);//Get High, Wait for next data sampleing. 
    }
    return result;
}
uint16_t calcByte(uint16_t crc, uint8_t b)
{
    uint32_t i;
    crc = crc ^ (uint32_t)b << 8;
    
    for ( i = 0; i < 8; i++)
    {
        if ((crc & 0x8000) == 0x8000)
            crc = crc << 1 ^ 0x1021;
        else
            crc = crc << 1;
    }
    return crc & 0xffff;
}

uint16_t CRC16(uint8_t *pBuffer,uint32_t length)
{
    uint16_t wCRC16=0;
    uint32_t i;
    if (( pBuffer==0 )||( length==0 ))
    {
      return 0;
    }
    for ( i = 0; i < length; i++)
    { 
      wCRC16 = calcByte(wCRC16, pBuffer[i]);
    }
    return wCRC16;
}

//###@@***//
//###@@***//
void loop()
{
  Upload_To_Gateway();
  MeasuredSound();
  readVcc();
  TEMP();
//  servo(); 
}

void Upload_To_Gateway()
{
  Serial.print("###########    ");
  Serial.print("COUNT=");
  Serial.print(count);
  Serial.println("    ###########");
  count++;
  ReadDHT();
  MeasuredSound();
  char data[50] = {0} ;
  int dataLength = 11; // Payload Length
  // Use data[0], data[1],data[2] as Node ID
  data[0] = node_id[0] ;
  data[1] = node_id[1] ;
  data[2] = node_id[2] ;
  data[3] = dht_dat[0];//Get Humidity Integer Part
  data[4] = dht_dat[1];//Get Humidity Decimal Part
  data[5] = dht_dat[2];//Get Temperature Integer Part
  data[6] = dht_dat[3];//Get Temperature Decimal Part

  //SDM Parts
  data[7] = db_str[0];//Get Sound (dB) Integer Part
  data[8] = db_str[1];//Get Sound (dB) Decimal Part

  //UV Parts
  data[9] = uv_str[0];//Get UV Integar Part
  data[10] = uv_str[1];//Get UV Decimal Part

  
  switch (bGlobalErr)
  {
    case 0:
      Serial.print("Current humidity = ");
      Serial.print(data[3], DEC);//Show humidity
      Serial.print(".");
      Serial.print(data[4], DEC);//Show humidity
      Serial.print("%  ");
      Serial.print("temperature = ");
      Serial.print(data[5], DEC);//Show temperature
      Serial.print(".");
      Serial.print(data[6], DEC);//Show temperature
      Serial.print("C  ");

      //SDM Parts
      Serial.print("Current Sound (dB) = ");
      Serial.print(data[7], DEC);//Show Sound dB
      Serial.print(".");
      Serial.print(data[8], DEC);//Show Sound dB
      Serial.println("dB  ");

      //UV Parts
      Serial.print("UV = ");
      Serial.print(data[9], DEC);//Show UV
      Serial.print(".");
      Serial.print(data[10], DEC);//Show UV
      Serial.println("UV  ");
      
      break;
    case 1:
      Serial.println("Error 1: DHT start condition 1 not met.");
      break;
    case 2:
      Serial.println("Error 2: DHT start condition 2 not met.");
      break;
    case 3:
      Serial.println("Error 3: DHT checksum error.");
      break;
    default:
      Serial.println("Error: Unrecognized code encountered.");
      break;
  }
    
  uint16_t crcData = CRC16((unsigned char*)data,dataLength);//get CRC DATA
  //Serial.println(crcData,HEX);
    
  Serial.print("Data to be sent(without CRC): ");
    
  int i;
  for(i = 0;i < dataLength; i++)
  {
    Serial.print(data[i],HEX);
    Serial.print(" ");
  }
  Serial.println();
        
  unsigned char sendBuf[50]={0};

  for(i = 0;i < dataLength;i++)
  {
    sendBuf[i] = data[i] ;
  }
    
  sendBuf[dataLength] = (unsigned char)crcData; // Add CRC to LoRa Data
  sendBuf[dataLength+1] = (unsigned char)(crcData>>8); // Add CRC to LoRa Data

  Serial.print("Data to be sent(with CRC):    ");
  for(i = 0;i < (dataLength +2); i++)
  {
    Serial.print(sendBuf[i],HEX);
    Serial.print(" ");
  }
  Serial.println();

  rf95.send(sendBuf, dataLength+2);//Send LoRa Data
     
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];//Reply data array
  uint8_t len = sizeof(buf);//reply data length

  if (rf95.waitAvailableTimeout(3000))// Check If there is reply in 3 seconds.
  {
    // Should be a reply message for us now   
    if (rf95.recv(buf, &len))//check if reply message is correct
    {
      if(buf[0] == node_id[0] ||buf[1] == node_id[2] ||buf[2] == node_id[2] ) // Check if reply message has the our node ID
      {
        pinMode(4, OUTPUT);
        digitalWrite(4, HIGH);
        Serial.print("Got Reply from Gateway: ");//print reply
        Serial.println((char*)buf);
              
        delay(400);
        digitalWrite(4, LOW); 
        //Serial.print("RSSI: ");  // print RSSI
        //Serial.println(rf95.lastRssi(), DEC);        
      }    
    }
    else
    {
      Serial.println("recv failed");//
      rf95.send(sendBuf, strlen((char*)sendBuf));//resend if no reply
    }
 }
 else
 {
  Serial.println("No reply, is LoRa gateway running?");//No signal reply
  rf95.send(sendBuf, strlen((char*)sendBuf));//resend data
 }
 delay(1000); // Send sensor data every 30 seconds
 Serial.println("");
}

//###>>>>>>>New Function Start HERE<<<<<<<###//
// From this part is SDM
void MeasuredSound()
{
  int value = analogRead(soundPin);//read the value of A0 
  //read ref voltage
  float ref_volt = float(readVcc())/1000.0;
  
  //preallocate
  float dbValue;
  
  
  
  // read and convert analog data to dB
    dbValue = (analogRead(soundPin)/1024.0)*ref_volt*50.0;    
    dtostrf(dbValue,1,2,db_str);
    //Serial.println(db_str);
    delay(80);
}
long readVcc() 
{
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif  

  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both

  long result = (high<<8) | low;

  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}

//###>>>>>>>New Function Start HERE<<<<<<<###//
void TEMP()
{
  sensorInput = analogRead(sensePin);
  temp = (double)sensorInput / 1024;       //find percentage of input reading
  temp = temp * 5;                 //multiply by 5V to get voltage
  temp = temp - 0.5;               //Subtract the offset 
  temp = temp * 100;               //Convert to degrees 

  Serial.print("Current Temperature: ");
  Serial.println(temp);
  delay(1000);
}
/*void servo()
{
  if(temp >= 22)
  {
    servo1.write(170);
  }
  else if(temp <= 20)
  {
    servo1.write(5);
  }
}
*/
void UV()
{
  sensorValueUV = analogRead(UVPin);
  sensorVoltageUV = sensorValueUV/1024*3.3;
 /* Serial.print("UV sensor reading = ");
  Serial.print(sensorValueUV);
  Serial.println("");
  Serial.print("UV sensor voltage = ");
  Serial.print(sensorVoltageUV);
  Serial.println(" V");
  delay(1000); 
*/
  dtostrf(sensorValueUV,1,2,uv_str);
}


