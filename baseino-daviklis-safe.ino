#include <OneWire.h>   //https://www.pjrc.com/teensy/td_libs_OneWire.html
#include <DallasTemperature.h> //https://github.com/milesburton/Arduino-Temperature-Control-Library

#define _DISABLE_TLS_ //leidzia prisijungti prie Wi-Fi
#include <ESP8266WiFi.h>

//#define _DEBUG_   //uncomment to show thinger.io debug info in Serial Monitor
#include <ThingerESP8266.h>  //https://github.com/thinger-io/Arduino-Library/blob/master/src/ThingerESP8266.h

#define oneWireBus 12 // Data wire is plugged into port 12 on the Arduino
OneWire oneWire(oneWireBus);  // Setup a oneWire instance to communicate with any OneWire devices
DallasTemperature sensors(&oneWire);  // Pass our oneWire reference to Dallas Temperature.
//sensors addresses
const DeviceAddress OrasSensorAddr={ 0x28, 0xC1, 0xE3, 0x16, 0xA8, 0x1, 0x3C, 0x19 };
const DeviceAddress VanduoBaseinasSensorAddr={ 0x28, 0x92, 0xA7, 0x56, 0xB5, 0x1, 0x3C, 0x22 };
const DeviceAddress VanduoTvenkinysSensorAddr={ 0x28, 0xA8, 0xDC, 0xA8, 0xC, 0x0, 0x0, 0x92 };
const DeviceAddress OrasBaseinasSensorAddr={ 0x28, 0x8A, 0xE2, 0x8F, 0x19, 0x20, 0x6, 0xF9 };

#define USERNAME "############"
#define DEVICE_ID "############"
#define DEVICE_CREDENTIAL "############"

#define SSID_NAME "############"
#define SSID_PASSWORD "############"

ThingerESP8266 thing(USERNAME, DEVICE_ID, DEVICE_CREDENTIAL); //nurodoma, kokiam prietaisui siųsti informaciją

const int analogInPin = A0; //ADC pin - A0
const double ADCK = 0.0041015625;   //= 4.2 / 1024

void setup()
{
  Serial.begin(115200);

  thing.add_wifi(SSID_NAME, SSID_PASSWORD); //prisijungiama prie Wi-Fi tinklo

  thing["RSSI"] >>[](pson & out)  //RSSI - Received signal strength indication - Wi-Fi stiprumas (-30 - -90)
  {
    out["RSSI"] = WiFi.RSSI();
  };

  thing["Voltage"] >>[](pson & out)
  {
    int sensorValue = analogRead(analogInPin);  //ADC skaicius 0-1024
    //Serial.println(sensorValue);

    double baterijosItampa = sensorValue * ADCK; //ADC i voltage baterijos
    out = baterijosItampa;
  };

  thing["TemperaturosAbi"] >>[](pson & out)
  {
    sensors.requestTemperatures();

    double TemperaturaLaukoOro = sensors.getTempC(OrasSensorAddr);
    if (TemperaturaLaukoOro != DEVICE_DISCONNECTED_C)
      out["TemperaturaLaukoOro"] = TemperaturaLaukoOro;

    double TemperaturaBaseinoOro = sensors.getTempC(OrasBaseinasSensorAddr);
    if (TemperaturaBaseinoOro != DEVICE_DISCONNECTED_C)
      out["TemperaturaBaseinoOro"] = TemperaturaBaseinoOro;

    double TemperaturaTvenkinioVandens = sensors.getTempC(VanduoTvenkinysSensorAddr);
    if (TemperaturaTvenkinioVandens != DEVICE_DISCONNECTED_C)
      out["TemperaturaTvenkinioVandens"] = TemperaturaTvenkinioVandens;

    double TemperaturaBaseinoVandens= sensors.getTempC(VanduoBaseinasSensorAddr);
    if (TemperaturaBaseinoVandens != DEVICE_DISCONNECTED_C)
      out["TemperaturaBaseinoVandens"] = TemperaturaBaseinoVandens;
  };

  sensors.begin();  // Start the DS18B20 sensor
}

//uncomment if want to test everything
//#define TEST_HW    

void loop()
{
  #ifdef TEST_HW

  //kodas testavimui
  //tikriname ar viskas veikia

  //temp. davikliai

  //Start the DS18B20 sensors
  //need call every time to refresh connected device count
  sensors.begin();

  Serial.print("getDeviceCount()=");
  int DeviceCount=sensors.getDeviceCount();
  Serial.println(DeviceCount);

  sensors.requestTemperatures();

  for(int i=0;i<DeviceCount;i++)
  {
    Serial.print("t[");
    Serial.print(i);
    Serial.print("]");

    //show connected sensor address
    Serial.print(" addr=");
    DeviceAddress addr;
    if(sensors.getAddress(addr, i))
    {
      for(int j=0;j<8;j++)
      {
        Serial.print(" 0x");
        Serial.print(addr[j], HEX);
        Serial.print(",");
      }
    }
    else
    {
      Serial.print("ERR");
    }
    
    Serial.print(" t=");
    double t=sensors.getTempCByIndex(i);
    if (t != DEVICE_DISCONNECTED_C)
      Serial.println(t);
    else
      Serial.println("ERR");
  }

  //batarke

  Serial.print("Battery ADC=");
  int adcraw=analogRead(analogInPin);
  Serial.print(adcraw);
  Serial.print(" ");
  Serial.println(adcraw * ADCK);

  delay(20000);
    
  #else
  
  //kodas normaliam darbui

  //handles the connection
  //on first call it will try to connect to WiFi
  thing.handle(); 

  //if WiFi was connected - send our data
  if(thing.is_connected())
  {
    //įrašome duomenis į tam skirtus saugojimo "kibirus"
    
    bool state;  //state after write_bucket(); sometimes is is false but data was really sent ???
    
    //Serial.print("Sending TemperaturosAbi...");
    state=thing.write_bucket("TemperaturosAbi", "TemperaturosAbi", true);
    //Serial.println(state);
    
    //Serial.print("Sending RSSI...");
    state=thing.write_bucket("RSSI", "RSSI", true);
    //Serial.println(state);
    
    //Serial.print("Sending Voltage...");
    state=thing.write_bucket("Voltage", "Voltage", true);
    //Serial.println(state);

    ESP.deepSleep(60*60*1e+06, WAKE_RF_DEFAULT); // jeigu buvo sėkmingas prisijungimas - deep sleep 1h
    //ESP.deepSleep(3*60*1e+06, WAKE_RF_DEFAULT); // jeigu buvo sėkmingas prisijungimas - deep sleep 3 min
  }
  else
  {
    //we failed to connect
    //Serial.println("thinger.io connection failed!");
    ESP.deepSleep(30*60*1e+06, WAKE_RF_DEFAULT);  //jeigu buvo nesėkmingas prisijungimas prie Wi-Fi - deep sleep 30 min
    //ESP.deepSleep(1*60*1e+06, WAKE_RF_DEFAULT);  //jeigu buvo nesėkmingas prisijungimas prie Wi-Fi - deep sleep 1 min
  }

  #endif  //TEST_HW
}
