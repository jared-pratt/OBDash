#pragma once
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEClient.h>
#include <BLEScan.h>
#include <BLERemoteService.h>
#include <BLERemoteCharacteristic.h>

#define ELM327_BT_NAME    "OBDBLE"
#define OBD_SERVICE_UUID  "0000fff0-0000-1000-8000-00805f9b34fb"
#define OBD_CHAR_UUID     "0000fff1-0000-1000-8000-00805f9b34fb"

class OBDReader {
public:
  float rpm=0,speed_kph=0,coolant_c=0,load_pct=0;
  float throttle=0,iat_c=0,maf_g_s=0,batt_v=12.6f;
  float stft=0,ltft=0,timing=0;
  bool  connected=false;

  bool begin(){
    Serial.println(); Serial.println("===== BLE OBD START =====");
    Serial.printf("Looking for BLE OBD device named: %s\n",ELM327_BT_NAME);
    if(!_bleStarted){BLEDevice::init("CarCoach");_bleStarted=true;}
    if(!_findDevice()){Serial.println("BLE scan failed.");connected=false;return false;}
    Serial.printf("Found BLE OBD device at: %s\n",_addr.c_str());
    if(!_connect()){Serial.println("BLE connect failed.");connected=false;return false;}
    Serial.println("BLE connected. Initialising ELM327...");
    connected=_init();
    if(connected){Serial.println("ELM327 init OK.");Serial.println("===== BLE OBD READY =====");}
    else Serial.println("ELM327 init failed.");
    return connected;
  }

  void probePIDs(){
    struct Candidate{ uint8_t pid; const char* name; };
    static const Candidate list[]={
      {0x0B,"MAP - intake manifold pressure (kPa)"},
      {0x0E,"Timing advance (deg before TDC)"},
      {0x14,"O2 sensor B1S1 voltage (V)"},
      {0x15,"O2 sensor B1S2 voltage (V)"},
      {0x1F,"Engine run time (seconds)"},
      {0x2F,"Fuel level (%)"},
      {0x33,"Barometric pressure (kPa)"},
      {0x43,"Absolute load value (%)"},
      {0x47,"Absolute throttle position B (%)"},
      {0x49,"Accelerator pedal position D (%)"},
      {0x4A,"Accelerator pedal position E (%)"},
      {0x5C,"Engine oil temperature (C)"},
      {0x5E,"Engine fuel rate (L/hr)"},
    };
    Serial.println("\n===== PID PROBE =====");
    for(auto& c : list){
      // Use longer timeout and retry once — BLE adapter is slow on non-standard PIDs
      float v=0; bool ok=false;
      char buf[8]; snprintf(buf,sizeof(buf),"01%02X\r",c.pid);
      for(int attempt=0;attempt<2&&!ok;attempt++){
        _rx="";_gotPrompt=false;
        _writeChar->writeValue((uint8_t*)buf,strlen(buf),false);
        uint32_t t0=millis();
        while(millis()-t0<600&&!_gotPrompt)delay(5);
        String r=_rx;r.replace("\r","");r.replace("\n","");r.replace(">","");
        r.replace("SEARCHING...","");r.replace("SEARCHING","");r.trim();
        Serial.printf("  RAW  0x%02X  attempt%d: '%s'\n",c.pid,attempt+1,r.c_str());
        if(r.length()>0&&r.indexOf("NO DATA")<0&&r.indexOf("UNABLE")<0&&r.indexOf("STOPPED")<0){
          bool parsed=false; float pv=_parse(c.pid,r,parsed);
          if(parsed){v=pv;ok=true;}
        }
        delay(200);
      }
      if(ok) Serial.printf("  OK   0x%02X  %-42s  val=%.2f\n",c.pid,c.name,v);
      else   Serial.printf("  NO   0x%02X  %s\n",c.pid,c.name);
    }
    Serial.println("===== PROBE DONE =====\n");
  }

  // flags: 0x001=RPM 0x002=speed 0x004=coolant 0x008=load 0x010=throttle
  //        0x020=MAF 0x040=battery 0x080=IAT 0x100=STFT 0x200=LTFT 0x400=timing
  void pollAll(uint16_t flags=0x07F){
    if(!connected)return;
    float v;
    if((flags&0x001)&&_readPID(0x0C,v))rpm=v;
    if((flags&0x002)&&_readPID(0x0D,v))speed_kph=v;
    if((flags&0x004)&&_readPID(0x05,v))coolant_c=v;
    if((flags&0x008)&&_readPID(0x04,v))load_pct=v;
    if((flags&0x010)&&_readPID(0x11,v))throttle=v;
    if((flags&0x020)&&_readPID(0x10,v))maf_g_s=v;
    if((flags&0x040)&&_readPID(0x42,v))batt_v=v;
    if((flags&0x080)&&_readPID(0x0F,v))iat_c=v;
    if((flags&0x100)&&_readPID(0x06,v))stft=v;
    if((flags&0x200)&&_readPID(0x07,v))ltft=v;
    if((flags&0x400)&&_readPID(0x0E,v))timing=v;
  }

private:
  BLEClient* _client=nullptr;
  BLERemoteCharacteristic* _writeChar=nullptr;
  BLERemoteCharacteristic* _notifyChar=nullptr;
  String _addr,_rx;
  bool _bleStarted=false,_gotPrompt=false;
  static OBDReader* _active;

  static void notifyCallback(BLERemoteCharacteristic*,uint8_t* data,size_t length,bool){
    if(!_active)return;
    for(size_t i=0;i<length;i++){char c=(char)data[i];_active->_rx+=c;if(c=='>')_active->_gotPrompt=true;}
  }

  bool _findDevice(){
    class ScanCb:public BLEAdvertisedDeviceCallbacks{
    public:
      String* dest;bool found=false;
      void onResult(BLEAdvertisedDevice d)override{
        if(!found&&d.haveName()&&String(d.getName().c_str())==ELM327_BT_NAME){
          *dest=String(d.getAddress().toString().c_str());found=true;BLEDevice::getScan()->stop();
        }
      }
    };
    ScanCb cb;cb.dest=&_addr;
    BLEScan* scan=BLEDevice::getScan();
    scan->setAdvertisedDeviceCallbacks(&cb,false);
    scan->setActiveScan(true);scan->setInterval(100);scan->setWindow(99);
    Serial.println("Scanning for OBDBLE...");
    scan->start(6,false);scan->clearResults();
    if(cb.found)Serial.printf("Found at %s\n",_addr.c_str());
    else         Serial.println("OBDBLE not found.");
    return cb.found;
  }

  bool _connect(){
    _active=this;
    _client=BLEDevice::createClient();
    Serial.printf("Connecting to %s...\n",_addr.c_str());
    if(!_client->connect(BLEAddress(_addr.c_str()))){Serial.println("client->connect() failed.");return false;}
    Serial.println("Connected. Locating OBD service...");
    _writeChar=nullptr;_notifyChar=nullptr;
    BLERemoteService* svc=_client->getService(BLEUUID(OBD_SERVICE_UUID));
    if(!svc){Serial.println("OBD service (FFF0) not found.");return false;}
    BLERemoteCharacteristic* ch=svc->getCharacteristic(BLEUUID(OBD_CHAR_UUID));
    if(!ch){Serial.println("OBD characteristic (FFF1) not found.");return false;}
    if(!ch->canWrite()&&!ch->canWriteNoResponse()){Serial.println("FFF1 is not writable.");return false;}
    if(!ch->canNotify()){Serial.println("FFF1 does not support notify.");return false;}
    _writeChar=_notifyChar=ch;
    Serial.println("Using FFF1 for write + notify.");
    _notifyChar->registerForNotify(notifyCallback);
    delay(300);return true;
  }

  bool _init(){
    struct Cmd{const char* txt;uint32_t ms;};
    Cmd cmds[]={{"ATZ\r",2500},{"ATE0\r",800},{"ATL0\r",800},{"ATS0\r",800},{"ATH0\r",800},{"ATSP0\r",1500}};
    for(auto& c:cmds){
      String r=_cmd(c.txt,c.ms);
      if(r.length()==0&&strcmp(c.txt,"ATZ\r")!=0){
        Serial.print("ELM init command failed: ");Serial.println(c.txt);return false;
      }
    }
    // Warm-up query: let ATSP0 finish SEARCHING before pollAll() starts (up to 10s)
    Serial.println("Waiting for protocol detection...");
    _rx="";_gotPrompt=false;
    _writeChar->writeValue((uint8_t*)"0100\r",5,false);
    uint32_t t=millis();
    while(millis()-t<10000&&!_gotPrompt)delay(10);
    _rx="";_gotPrompt=false;
    Serial.println("Protocol detection done.");
    return true;
  }

  bool _readPID(uint8_t pid,float& out){
    char buf[8];snprintf(buf,sizeof(buf),"01%02X\r",pid);
    String resp=_cmd(buf,250);
    if(!resp.length())return false;
    if(resp.indexOf("NO DATA")>=0||resp.indexOf("UNABLE")>=0||
       resp.indexOf("STOPPED")>=0||resp.indexOf("?")>=0)return false;
    bool ok=false;float parsed=_parse(pid,resp,ok);
    if(!ok){
      Serial.print("Parse failed for PID ");Serial.print(pid,HEX);
      Serial.print(" resp: ");Serial.println(resp);return false;
    }
    out=parsed;return true;
  }

  String _cmd(const char* txt,uint32_t timeout_ms){
    if(!_client||!_client->isConnected()||!_writeChar){connected=false;return "";}
    _rx="";_gotPrompt=false;
    Serial.print("CMD: ");Serial.print(txt);
    _writeChar->writeValue((uint8_t*)txt,strlen(txt),false);
    uint32_t t0=millis();
    while(millis()-t0<timeout_ms){if(_gotPrompt)break;delay(5);}
    String r=_rx;r.replace("\r","");r.replace("\n","");r.replace(">","");r.trim();
    Serial.print("RESP: ");Serial.println(r);
    return r;
  }

  String _cleanHex(String resp){
    resp.toUpperCase();resp.replace(" ","");resp.replace("\r","");resp.replace("\n","");
    resp.replace(">","");resp.replace("SEARCHING...","");resp.replace("SEARCHING","");
    resp.trim();return resp;
  }

  float _parse(uint8_t pid,String& resp,bool& ok){
    ok=false;String clean=_cleanHex(resp);
    char header[8];snprintf(header,sizeof(header),"41%02X",pid);
    int idx=clean.indexOf(header);if(idx<0)return 0;
    String d=clean.substring(idx+4);
    auto byte_=[&](int i)->int{
      int s=i*2;if(s+2>(int)d.length())return 0;
      return(int)strtol(d.substring(s,s+2).c_str(),nullptr,16);
    };
    int A=byte_(0),B=byte_(1);ok=true;
    switch(pid){
      case 0x04:return A*100.0f/255.0f;
      case 0x05:return A-40.0f;
      case 0x06:return A/1.28f-100.0f;   // short term fuel trim %
      case 0x07:return A/1.28f-100.0f;   // long term fuel trim %
      case 0x0C:return(float)((A<<8)|B)/4.0f;
      case 0x0D:return(float)A;
      case 0x0F:return A-40.0f;
      case 0x10:return(float)((A<<8)|B)/100.0f;
      case 0x0E:return A/2.0f-64.0f;
      case 0x11:return A*100.0f/255.0f;
      case 0x45:return A*100.0f/255.0f;
      case 0x42:return(float)(((A<<8)|B))/1000.0f;
      default:  return(float)A;
    }
  }
};
OBDReader* OBDReader::_active=nullptr;
