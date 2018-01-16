#include "./functions.h"
#include "./wifi.h"

//int myArray[10] = {1,2,3,4,5,6,7,8,9,10};



void setup(void){
  DBG_OUTPUT_PORT.begin(115200);
  DBG_OUTPUT_PORT.print("\n");
  DBG_OUTPUT_PORT.setDebugOutput(true);
  SPIFFS.begin();
  {
    Dir dir = SPIFFS.openDir("/");
    while (dir.next()) {    
      String fileName = dir.fileName();
      size_t fileSize = dir.fileSize();
      DBG_OUTPUT_PORT.printf("FS File: %s, size: %s\n", fileName.c_str(), formatBytes(fileSize).c_str());
    }
    DBG_OUTPUT_PORT.printf("\n");
  }

  SPIFFS.info(fs_info);
  

  //WIFI INIT
  DBG_OUTPUT_PORT.printf("Connecting to %s\n", ssid);
  if (String(WiFi.SSID()) != String(ssid)) {
    WiFi.begin(ssid, password);
  }
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    DBG_OUTPUT_PORT.print(".");
  }
  DBG_OUTPUT_PORT.println("");
  DBG_OUTPUT_PORT.print("Connected! IP address: ");
  DBG_OUTPUT_PORT.println(WiFi.localIP());
  DBG_OUTPUT_PORT.print("RSSI:");
  DBG_OUTPUT_PORT.println(rssi);

  //MDNS.begin(host);
  //DBG_OUTPUT_PORT.print("Open http://");
  //DBG_OUTPUT_PORT.print(host);
  //DBG_OUTPUT_PORT.println(".local/edit to see the file browser");
  
  DBG_OUTPUT_PORT.println("Starting UDP");
  Udp.begin(localPort);
  DBG_OUTPUT_PORT.print("Local port: ");
  DBG_OUTPUT_PORT.println(Udp.localPort());
  DBG_OUTPUT_PORT.println("waiting for sync");
  setSyncProvider(getNtpTime);
  setSyncInterval(86400); // check NTP every 24hrs (60*60*24)


// sensor setup
  DBG_OUTPUT_PORT.println("Setting up BME280....");
  mySensor.settings.commInterface = I2C_MODE;
  mySensor.settings.I2CAddress = 0x76;  //bme280 address
  mySensor.settings.runMode = 3; //0, Sleep mode, 1 or 2, Forced mode, 3, Normal mode
  mySensor.settings.tStandby = 0; //0, 0.5ms, 1, 62.5ms, 2, 125ms, 3, 250ms, 4, 500ms, 5, 1000ms, 6, 10ms, 7, 20ms
  mySensor.settings.filter = 0; // 0, filter off, 1,coefficients = 2, 2,coefficients = 4, 3,coefficients = 8, 4, coefficients = 16
  mySensor.settings.tempOverSample = 1; //  1 through 5, oversampling *1, *2, *4, *8, *16 respectively
  mySensor.settings.pressOverSample = 1; //  1 through 5, oversampling *1, *2, *4, *8, *16 respectively
  mySensor.settings.humidOverSample = 1; //  1 through 5, oversampling *1, *2, *4, *8, *16 respectively
  delay(20);  //Make sure sensor had enough time to turn on. BME280 requires 2ms to start up.
  DBG_OUTPUT_PORT.print("Started BME280... result of .begin(): 0x");
  DBG_OUTPUT_PORT.println(mySensor.begin(), HEX); // start BME280
  DBG_OUTPUT_PORT.println(" ");

  DBG_OUTPUT_PORT.println("Setting up MPY9250....");
  // Read the WHO_AM_I register, this is a good test of communication
  byte c = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  DBG_OUTPUT_PORT.print("MPU9250 "); Serial.print("I AM "); Serial.print(c, HEX);
  DBG_OUTPUT_PORT.print(" I should be "); Serial.println(0x71, HEX);

  if (c == 0x71) // WHO_AM_I should always be 0x68
  {
    DBG_OUTPUT_PORT.println("MPU9250 is online...");

    myIMU.MPU9250SelfTest(myIMU.SelfTest);
    DBG_OUTPUT_PORT.print("x-axis self test: acceleration trim within : ");
    DBG_OUTPUT_PORT.print(myIMU.SelfTest[0],1); Serial.println("% of factory value");
    DBG_OUTPUT_PORT.print("y-axis self test: acceleration trim within : ");
    DBG_OUTPUT_PORT.print(myIMU.SelfTest[1],1); Serial.println("% of factory value");
    DBG_OUTPUT_PORT.print("z-axis self test: acceleration trim within : ");
    DBG_OUTPUT_PORT.print(myIMU.SelfTest[2],1); Serial.println("% of factory value");
    DBG_OUTPUT_PORT.print("x-axis self test: gyration trim within : ");
    DBG_OUTPUT_PORT.print(myIMU.SelfTest[3],1); Serial.println("% of factory value");
    DBG_OUTPUT_PORT.print("y-axis self test: gyration trim within : ");
    DBG_OUTPUT_PORT.print(myIMU.SelfTest[4],1); Serial.println("% of factory value");
    DBG_OUTPUT_PORT.print("z-axis self test: gyration trim within : ");
    DBG_OUTPUT_PORT.print(myIMU.SelfTest[5],1); Serial.println("% of factory value");

    myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);
    
    myIMU.initMPU9250();
    DBG_OUTPUT_PORT.println("MPU9250 initialized for active data mode....");

    byte d = myIMU.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
    DBG_OUTPUT_PORT.print("AK8963 "); Serial.print("I AM "); Serial.print(d, HEX);
    DBG_OUTPUT_PORT.print(" I should be "); Serial.println(0x48, HEX);

    myIMU.initAK8963(myIMU.magCalibration);
    // Initialize device for active mode read of magnetometer
    DBG_OUTPUT_PORT.println("AK8963 initialized for active data mode....");

      DBG_OUTPUT_PORT.print("X-Axis sensitivity adjustment value ");
      DBG_OUTPUT_PORT.println(myIMU.magCalibration[0], 2);
      DBG_OUTPUT_PORT.print("Y-Axis sensitivity adjustment value ");
      DBG_OUTPUT_PORT.println(myIMU.magCalibration[1], 2);
      DBG_OUTPUT_PORT.print("Z-Axis sensitivity adjustment value ");
      DBG_OUTPUT_PORT.println(myIMU.magCalibration[2], 2);

  } // if (c == 0x71)
  else
  {
    DBG_OUTPUT_PORT.print("Could not connect to MPU9250: 0x");
    DBG_OUTPUT_PORT.println(c, HEX);
    while(1) ; // Loop forever if communication doesn't happen
  }

// server setup
  DBG_OUTPUT_PORT.println("Starting Servers");
  //SERVER INIT
  //list directory
  server.on("/list", HTTP_GET, handleFileList);
  //load editor
  server.on("/edit", HTTP_GET, [](){
    if(!handleFileRead("/edit.htm")) server.send(404, "text/plain", "FileNotFound");
  });
  //create file
  server.on("/edit", HTTP_PUT, handleFileCreate);
  //delete file
  server.on("/edit", HTTP_DELETE, handleFileDelete);
  //first callback is called after the request has ended with all parsed arguments
  //second callback handles file uploads at that location
  server.on("/edit", HTTP_POST, [](){ server.send(200, "text/plain", ""); }, handleFileUpload);

  //called when the url is not defined here
  //use it to load content from SPIFFS
  server.onNotFound([](){
    if(!handleFileRead(server.uri()))
      DBG_OUTPUT_PORT.println(" NOT FOUND!");
      server.send(404, "text/plain", "FileNotFound");
      
  });


// -------------- Process JSON Here! ---------------
  //get heap status, analog input value and all GPIO statuses in one json call
    server.on("/all", HTTP_GET, [](){
    //do json
    SPIFFS.info(fs_info);
    doMPU9250();
    doPCF8591();
    String json = "{";
    json += "\"uptime\":\""+String(millis2time())+"\"";
    //json += ", \"analog\":"+String(analogRead(A0));
    //json += ", \"gpio\":"+String((uint32_t)(((GPI | GPO) & 0xFFFF) | ((GP16I & 0x01) << 16)));
    json += ", \"hour\":\""+String(hourFormat12())+"\"";    
    if (minute() < 10) {
      json +=", \"minute\":\"0"+String(minute())+"\"";
    }
    else {
    json += ", \"minute\":\""+String(minute())+"\"";
    }
    if (second() < 10) {
      json +=", \"second\":\"0"+String(second())+"\"";
    }
    else {
    json += ", \"second\":\""+String(second())+"\"";
    }
    
    if (weekday() == 1) {
      json += ", \"weekday\":\"Sunday\"";
    }
    if (weekday() == 2) {
      json += ", \"weekday\":\"Monday\"";
    }
    if (weekday() == 3) {
      json += ", \"weekday\":\"Tuesday\"";
    }
    if (weekday() == 4) {
      json += ", \"weekday\":\"Wednesday\"";
    }
    if (weekday() == 5) {
      json += ", \"weekday\":\"Thursday\"";
    }
    if (weekday() == 6) {
      json += ", \"weekday\":\"Friday\"";
    }
    if (weekday() == 7) {
      json += ", \"weekday\":\"Saturday\"";
    }
    json += ", \"day\":\""+String(day())+"\"";
    json += ", \"month\":\""+String(month())+"\"";
    json += ", \"year\":\""+String(year())+"\"";
    if (isAM() == 1) {
      json += ", \"isAM\": \"AM\"";
    }
    else {
      json += ", \"isAM\": \"PM\"";
    }
    float vccd = (ESP.getVcc());
    json += ", \"vcc\":\""+String((vccd/1000),2)+"\"";
    json += ", \"rssi\":\""+String(WiFi.RSSI())+"\"";
    json += ", \"cpufreq\":\""+String(ESP.getCpuFreqMHz())+"\"";
    json += ", \"heap\":\""+String(ESP.getFreeHeap())+"\"";
    json += ", \"corever\":\""+String(ESP.getCoreVersion())+"\"";
    String cid = String(ESP.getChipId(),HEX);
    cid.toUpperCase();
    json += ", \"chipid\":\""+cid+"\"";
    json += ", \"sdkver\":\""+String(ESP.getSdkVersion())+"\"";
    json += ", \"bootver\":\""+String(ESP.getBootVersion())+"\"";
    json += ", \"bootmode\":\""+String(ESP.getBootMode())+"\"";
    String fid = String(ESP.getFlashChipId(),HEX);
    fid.toUpperCase();
    json += ", \"flashid\":\""+fid+"\"";
    json += ", \"flashsize\":\""+String(flashChipSize,2)+"\"";
    json += ", \"flashfreq\":\""+String(flashFreq,2)+"\"";
    json += ", \"mode\":\""+String((ideMode == FM_QIO ? "QIO" : ideMode == FM_QOUT ? "QOUT" : ideMode == FM_DIO ? "DIO" : ideMode == FM_DOUT ? "DOUT" : "UNKNOWN"))+"\"";

    json += ", \"fstotal\":\""+String((fs_info.totalBytes/1024.0))+"\"";
    json += ", \"fsused\":\""+String((fs_info.usedBytes/1024.0))+"\"";
    json += ", \"blocksize\":\""+String(fs_info.blockSize)+"\"";
    json += ", \"pagesize\":\""+String(fs_info.pageSize)+"\"";
    json += ", \"maxopenfiles\":\""+String(fs_info.maxOpenFiles)+"\"";
    json += ", \"maxpathlen\":\""+String(fs_info.maxPathLength)+"\"";

    json += ", \"PCFvalue0\":\""+String(value0)+"\"";
    json += ", \"PCFvalue1\":\""+String(value1)+"\"";
    json += ", \"PCFvalue2\":\""+String(value2)+"\"";
    json += ", \"PCFvalue3\":\""+String(value3)+"\"";

    json += ", \"tempc\":\""+String(mySensor.readTempC())+"\"";
    json += ", \"tempf\":\""+String(mySensor.readTempF())+"\"";
    json += ", \"humidity\":\""+String(mySensor.readFloatHumidity(), 2)+"\"";
    json += ", \"pressure\":\""+String((mySensor.readFloatPressure()/100), 2)+"\"";
    
    //json += ", \"temp\":\""+String(float(myIMU.readTempData()/100))+"\"";
    json += ", \"temp\":\""+String(float(myIMU.temperature))+"\"";
    
    json += ", \"ax\":\""+String((int)1000*myIMU.ax)+"\"";
    json += ", \"ay\":\""+String((int)1000*myIMU.ay)+"\"";
    json += ", \"az\":\""+String((int)1000*myIMU.az)+"\"";
    json += ", \"gx\":\""+String(myIMU.gx)+"\"";
    json += ", \"gy\":\""+String(myIMU.gy)+"\"";
    json += ", \"gz\":\""+String(myIMU.gz)+"\"";
    json += ", \"mx\":\""+String(myIMU.mx)+"\"";
    json += ", \"my\":\""+String(myIMU.my)+"\"";
    json += ", \"mz\":\""+String(myIMU.mz)+"\"";
    
    //json += ", \"array\": [\""+String(myArray[0])+","+String(myArray[1])+","+String(myArray[2])+","+String(myArray[3])+","+String(myArray[4])+","+String(myArray[5])+","+String(myArray[6])+","+String(myArray[7])+","+String(myArray[8])+","+String(myArray[9])+"]\"";
    
    json += "}";
    server.send(200, "text/json", json);
    json = String();
  });
  
  server.begin();
  DBG_OUTPUT_PORT.println("HTTP server started");

}
 
void loop(void){
  server.handleClient();
    if (millis() > wait000) {
    // send vars to serial out
    doSout();
    wait000 = millis() + 1000UL;
  }

}
