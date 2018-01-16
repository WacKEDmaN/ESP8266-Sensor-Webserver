#include "user_interface.h"
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
//#include <ESP8266mDNS.h>
#include <TimeLib.h>
#include <WiFiUdp.h>
#include <SPI.h>
#include <FS.h>
#include <Wire.h>
#include <SparkFunBME280.h>
#include "quaternionFilters.h"
#include "MPU9250.h"

#define DBG_OUTPUT_PORT Serial

#define DEBUG true

FSInfo fs_info;
uint32_t realSize = ESP.getFlashChipRealSize();

float fileTotalKB = (float)fs_info.totalBytes / 1024.0; 
float fileUsedKB = (float)fs_info.usedBytes / 1024.0; 

float flashChipSize = (float)ESP.getFlashChipSize() / 1024.0 / 1024.0;
float realFlashChipSize = (float)ESP.getFlashChipRealSize() / 1024.0 / 1024.0;
float flashFreq = (float)ESP.getFlashChipSpeed() / 1000.0 / 1000.0;
FlashMode_t ideMode = ESP.getFlashChipMode();

ADC_MODE(ADC_VCC);

#define PCF8591 (0x90>> 1) // I2C bus address
#define ADC0 0x00 // control bytes for reading individual ADCs
#define ADC1 0x01
#define ADC2 0x02
#define ADC3 0x03
byte value0, value1, value2, value3;

BME280 mySensor; // start BMP280

MPU9250 myIMU; // start MPU9250

const int chipSelect = 4;




String millis2time() {
  String Time = "";
  unsigned long ss;
  byte mm, hh;
  ss = millis() / 1000;
  hh = ss / 3600;
  mm = (ss - hh * 3600) / 60;
  ss = (ss - hh * 3600) - mm * 60;
  if (hh < 10)Time += "0";
  Time += (String)hh + ":";
  if (mm < 10)Time += "0";
  Time += (String)mm + ":";
  if (ss < 10)Time += "0";
  Time += (String)ss;
  return String(Time);
}

void doMPU9250() {
   myIMU.readAccelData(myIMU.accelCount);  // Read the x/y/z adc values
    myIMU.getAres();

    myIMU.ax = (float)myIMU.accelCount[0]*myIMU.aRes; // - accelBias[0];
    myIMU.ay = (float)myIMU.accelCount[1]*myIMU.aRes; // - accelBias[1];
    myIMU.az = (float)myIMU.accelCount[2]*myIMU.aRes; // - accelBias[2];

    myIMU.readGyroData(myIMU.gyroCount);  // Read the x/y/z adc values
    myIMU.getGres();

    myIMU.gx = (float)myIMU.gyroCount[0]*myIMU.gRes;
    myIMU.gy = (float)myIMU.gyroCount[1]*myIMU.gRes;
    myIMU.gz = (float)myIMU.gyroCount[2]*myIMU.gRes;

    myIMU.readMagData(myIMU.magCount);  // Read the x/y/z adc values
    myIMU.getMres();
    myIMU.magbias[0] = +470.;
    myIMU.magbias[1] = +120.;
    myIMU.magbias[2] = +125.;

    myIMU.mx = (float)myIMU.magCount[0]*myIMU.mRes*myIMU.magCalibration[0] -
               myIMU.magbias[0];
    myIMU.my = (float)myIMU.magCount[1]*myIMU.mRes*myIMU.magCalibration[1] -
               myIMU.magbias[1];
    myIMU.mz = (float)myIMU.magCount[2]*myIMU.mRes*myIMU.magCalibration[2] -
               myIMU.magbias[2];

  myIMU.updateTime();

  MahonyQuaternionUpdate(myIMU.ax, myIMU.ay, myIMU.az, myIMU.gx*DEG_TO_RAD,
                         myIMU.gy*DEG_TO_RAD, myIMU.gz*DEG_TO_RAD, myIMU.my,
                         myIMU.mx, myIMU.mz, myIMU.deltat);
                 
  myIMU.delt_t = millis() - myIMU.count;

      myIMU.yaw   = atan2(2.0f * (*(getQ()+1) * *(getQ()+2) + *getQ() *
                    *(getQ()+3)), *getQ() * *getQ() + *(getQ()+1) * *(getQ()+1)
                    - *(getQ()+2) * *(getQ()+2) - *(getQ()+3) * *(getQ()+3));
      myIMU.pitch = -asin(2.0f * (*(getQ()+1) * *(getQ()+3) - *getQ() *
                    *(getQ()+2)));
      myIMU.roll  = atan2(2.0f * (*getQ() * *(getQ()+1) + *(getQ()+2) *
                    *(getQ()+3)), *getQ() * *getQ() - *(getQ()+1) * *(getQ()+1)
                    - *(getQ()+2) * *(getQ()+2) + *(getQ()+3) * *(getQ()+3));
      myIMU.pitch *= RAD_TO_DEG;
      myIMU.yaw   *= RAD_TO_DEG;

      myIMU.yaw   = 12.23; // Magnetic Declination 
      myIMU.roll  *= RAD_TO_DEG;

      myIMU.count = millis();
      myIMU.sumCount = 0;
      myIMU.sum = 0;

      myIMU.tempCount = myIMU.readTempData();  // Read the adc values
        // Temperature in degrees Centigrade
        myIMU.temperature = ((float) myIMU.tempCount) / 333.87 + 21.0;
        
    } 

void doPCF8591() {
  Wire.beginTransmission(PCF8591); // wake up PCF8591
  Wire.write(ADC0); // control byte - read ADC0
  Wire.endTransmission(); // end tranmission
  Wire.requestFrom(PCF8591, 2);
  value0=Wire.read();
  value0=Wire.read();
  Wire.beginTransmission(PCF8591); // wake up PCF8591
  Wire.write(ADC1); // control byte - read ADC1
  Wire.endTransmission(); // end tranmission
  Wire.requestFrom(PCF8591, 2);
  value1=Wire.read();
  value1=Wire.read();
  Wire.beginTransmission(PCF8591); // wake up PCF8591
  Wire.write(ADC2); // control byte - read ADC2
  Wire.endTransmission(); // end tranmission
  Wire.requestFrom(PCF8591, 2);
  value2=Wire.read();
  value2=Wire.read();
  Wire.beginTransmission(PCF8591); // wake up PCF8591
  Wire.write(ADC3); // control byte - read ADC3
  Wire.endTransmission(); // end tranmission
  Wire.requestFrom(PCF8591, 2);
  value3=Wire.read();
  value3=Wire.read();
}
//format bytes
String formatBytes(size_t bytes){
  if (bytes < 1024){
    return String(bytes)+"B";
  } else if(bytes < (1024 * 1024)){
    return String(bytes/1024.0)+"KB";
  } else if(bytes < (1024 * 1024 * 1024)){
    return String(bytes/1024.0/1024.0)+"MB";
  } else {
    return String(bytes/1024.0/1024.0/1024.0)+"GB";
  }
}


void deleteFirstLine(){
    File original = SPIFFS.open("/tempc.txt", "r");
    String name_ = original.name();
    //DBG_OUTPUT_PORT.println(name_);
    if (!original) {
      DBG_OUTPUT_PORT.print("- failed to open file "); Serial.println("/tempc.txt");
    }else{
      //DBG_OUTPUT_PORT.print("- DELETING FROM FILE "); Serial.println("/tempc.txt");
      //We skip the first line
     original.readStringUntil('\n');
     File temporary = SPIFFS.open("/temp.txt", "w+");
     if(!temporary){
      DBG_OUTPUT_PORT.println("-- failed to open temporary file "); 
     }else{
      while(original.available()){
        temporary.print(original.readStringUntil('\n')+"\n");
      }
      temporary.close(); 
     }
     original.close();    
     
     if(DEBUG == 1){   
         if(SPIFFS.remove("/tempc.txt")){
            DBG_OUTPUT_PORT.println("Old file succesfully deleted");
         }else{
            DBG_OUTPUT_PORT.println("Couldn't delete file");
         }
         if(SPIFFS.rename("/temp.txt","/tempc.txt")){
            DBG_OUTPUT_PORT.println("Succesfully renamed");
         }else{
            DBG_OUTPUT_PORT.println("Couldn't rename file");
         } 
      }else{
        SPIFFS.remove("/tempc.txt");
        SPIFFS.rename("/temp.txt","/tempc.txt");
      }
    } 
}

void writeFiles() {
  String tempString = "";
  tempString += String(millis2time());
  tempString += ", ";
  tempString += String(hour()) + ":" + String(minute()) + ":" + String(second());
  tempString += ", ";
  tempString += String(mySensor.readTempC());
  tempString += ", ";
  tempString += String(mySensor.readTempF());
  
  File f = SPIFFS.open("/tempc.txt", "a");
  if (!f) {
      DBG_OUTPUT_PORT.println("file open failed");
  }
  if (f.size() > 5120) {
    f.close();
    //SPIFFS.remove("/tempc.txt");
    deleteFirstLine();
  }
  f.println(tempString);
  f.close();
}

void doSout() {
    SPIFFS.info(fs_info);
    doMPU9250();
    doPCF8591();
    String sout = "{";
    sout += "\"uptime\":\""+String(millis2time())+"\"";
    //json += ", \"analog\":"+String(analogRead(A0));
    //json += ", \"gpio\":"+String((uint32_t)(((GPI | GPO) & 0xFFFF) | ((GP16I & 0x01) << 16)));
    sout += ", \"hour\":\""+String(hourFormat12())+"\"";    
    if (minute() < 10) {
      sout +=", \"minute\":\"0"+String(minute())+"\"";
    }
    else {
    sout += ", \"minute\":\""+String(minute())+"\"";
    }
    if (second() < 10) {
      sout +=", \"second\":\"0"+String(second())+"\"";
    }
    else {
    sout += ", \"second\":\""+String(second())+"\"";
    }
    
    if (weekday() == 1) {
      sout += ", \"weekday\":\"Sunday\"";
    }
    if (weekday() == 2) {
      sout += ", \"weekday\":\"Monday\"";
    }
    if (weekday() == 3) {
      sout += ", \"weekday\":\"Tuesday\"";
    }
    if (weekday() == 4) {
      sout += ", \"weekday\":\"Wednesday\"";
    }
    if (weekday() == 5) {
      sout += ", \"weekday\":\"Thursday\"";
    }
    if (weekday() == 6) {
      sout += ", \"weekday\":\"Friday\"";
    }
    if (weekday() == 7) {
      sout += ", \"weekday\":\"Saturday\"";
    }
    sout += ", \"day\":\""+String(day())+"\"";
    sout += ", \"month\":\""+String(month())+"\"";
    sout += ", \"year\":\""+String(year())+"\"";
    if (isAM() == 1) {
      sout += ", \"isAM\": \"AM\"";
    }
    else {
      sout += ", \"isAM\": \"PM\"";
    }
    float vccd = (ESP.getVcc());
    sout += ", \"vcc\":\""+String((vccd/1000),2)+"\"";
    sout += ", \"rssi\":\""+String(WiFi.RSSI())+"\"";
    sout += ", \"cpufreq\":\""+String(ESP.getCpuFreqMHz())+"\"";
    sout += ", \"heap\":\""+String(ESP.getFreeHeap())+"\"";
    sout += ", \"corever\":\""+String(ESP.getCoreVersion())+"\"";
    String cid = String(ESP.getChipId(),HEX);
    cid.toUpperCase();
    sout += ", \"chipid\":\""+cid+"\"";
    sout += ", \"sdkver\":\""+String(ESP.getSdkVersion())+"\"";
    sout += ", \"bootver\":\""+String(ESP.getBootVersion())+"\"";
    sout += ", \"bootmode\":\""+String(ESP.getBootMode())+"\"";
    String fid = String(ESP.getFlashChipId(),HEX);
    fid.toUpperCase();
    sout += ", \"flashid\":\""+fid+"\"";
    sout += ", \"flashsize\":\""+String(flashChipSize,2)+"\"";
    sout += ", \"flashfreq\":\""+String(flashFreq,2)+"\"";
    sout += ", \"mode\":\""+String((ideMode == FM_QIO ? "QIO" : ideMode == FM_QOUT ? "QOUT" : ideMode == FM_DIO ? "DIO" : ideMode == FM_DOUT ? "DOUT" : "UNKNOWN"))+"\"";

    sout += ", \"fstotal\":\""+String((fs_info.totalBytes/1024.0))+"\"";
    sout += ", \"fsused\":\""+String((fs_info.usedBytes/1024.0))+"\"";
    sout += ", \"blocksize\":\""+String(fs_info.blockSize)+"\"";
    sout += ", \"pagesize\":\""+String(fs_info.pageSize)+"\"";
    sout += ", \"maxopenfiles\":\""+String(fs_info.maxOpenFiles)+"\"";
    sout += ", \"maxpathlen\":\""+String(fs_info.maxPathLength)+"\"";

    sout += ", \"PCFvalue0\":\""+String(value0)+"\"";
    sout += ", \"PCFvalue1\":\""+String(value1)+"\"";
    sout += ", \"PCFvalue2\":\""+String(value2)+"\"";
    sout += ", \"PCFvalue3\":\""+String(value3)+"\"";

    sout += ", \"tempc\":\""+String(mySensor.readTempC())+"\"";
    sout += ", \"tempf\":\""+String(mySensor.readTempF())+"\"";
    sout += ", \"humidity\":\""+String(mySensor.readFloatHumidity(), 2)+"\"";
    sout += ", \"pressure\":\""+String((mySensor.readFloatPressure()/100), 2)+"\"";
    
    //json += ", \"temp\":\""+String(float(myIMU.readTempData()/100))+"\"";
    sout += ", \"temp\":\""+String(float(myIMU.temperature))+"\"";
    
    sout += ", \"ax\":\""+String((int)1000*myIMU.ax)+"\"";
    sout += ", \"ay\":\""+String((int)1000*myIMU.ay)+"\"";
    sout += ", \"az\":\""+String((int)1000*myIMU.az)+"\"";
    sout += ", \"gx\":\""+String(myIMU.gx)+"\"";
    sout += ", \"gy\":\""+String(myIMU.gy)+"\"";
    sout += ", \"gz\":\""+String(myIMU.gz)+"\"";
    sout += ", \"mx\":\""+String(myIMU.mx)+"\"";
    sout += ", \"my\":\""+String(myIMU.my)+"\"";
    sout += ", \"mz\":\""+String(myIMU.mz)+"\"";
    
    //json += ", \"array\": [\""+String(myArray[0])+","+String(myArray[1])+","+String(myArray[2])+","+String(myArray[3])+","+String(myArray[4])+","+String(myArray[5])+","+String(myArray[6])+","+String(myArray[7])+","+String(myArray[8])+","+String(myArray[9])+"]\"";
    
    sout += "}";
    DBG_OUTPUT_PORT.println(sout);
    sout = String();

}

