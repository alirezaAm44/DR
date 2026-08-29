#include <SPI.h>
#include <DJIRonin.h>
#include <DJIRonin/transport/mcp2515/MCP2515Driver.h>
#include <SBUSNanoTx/SBUSNanoTx.h>
using namespace dji::ronin;

// Arduino Nano / ATmega328P field-test configuration.
// A1=Yaw, A2=Pitch, D2=Record, D8=SBUS, D10=MCP2515 CS.
// Ronin SBUS polarity is configured in Ronin software; no extra
// inversion logic is needed in the CAN path.
const bool ENABLE_CAN = true;
const bool ENABLE_SBUS = true;
const bool SBUS_INVERTED = false; // Must match the polarity selected in Ronin software.
const uint8_t JOY_YAW=A1, JOY_PITCH=A2, REC_PIN=2, CAN_CS=10;
const int JOY_CENTER=512, DEAD_ZONE=40;
const int16_t DJI_MIN=-7500, DJI_MAX=7500;
const uint16_t SBUS_MIN=352, SBUS_MID=1024, SBUS_MAX=1696;
const uint8_t CH_YAW=0, CH_PITCH=1, CH_RECORD=3;
const uint32_t JOY_MS=20, SBUS_MS=14, INFO_MS=500, STATUS_MS=250, DEBOUNCE_MS=40;

// IMPORTANT: MCP2515Driver is part of the DR library and must be included
// explicitly. The previous example used the type without including its header.
MCP2515Driver can(CAN_CS);
DJIRonin ronin(can);
PacketBuilder builder;
SBUSNanoTx sbus(SBUS_INVERTED);

uint32_t canOk=0, canFail=0, infoTx=0, infoRx=0, infoNoRx=0, infoErr=0;
uint32_t sbusOk=0, sbusDrop=0;
uint32_t tJoy=0,tSbus=0,tInfo=0,tStatus=0;
int16_t yaw=0,pitch=0,roll=0;
bool recording=false, buttonStable=HIGH, buttonLast=HIGH;
uint32_t buttonChangedAt=0;

const char* errorName(Error e){
  switch(e){
    case Error::Ok:return "Ok"; case Error::InvalidParameter:return "InvalidParameter";
    case Error::PacketTooLarge:return "PacketTooLarge"; case Error::Crc16Mismatch:return "Crc16Mismatch";
    case Error::Crc32Mismatch:return "Crc32Mismatch"; case Error::InvalidSof:return "InvalidSof";
    case Error::InvalidLength:return "InvalidLength"; case Error::InvalidResponse:return "InvalidResponse";
    case Error::Timeout:return "Timeout"; case Error::TransportError:return "TransportError";
    case Error::ProtocolVersionMismatch:return "ProtocolVersionMismatch"; case Error::UnknownCommand:return "UnknownCommand";
    case Error::BufferTooSmall:return "BufferTooSmall"; case Error::NotInitialized:return "NotInitialized";
    case Error::NoData:return "NoData"; default:return "Unknown";
  }
}

void printHex(const uint8_t* d,size_t n){
  for(size_t i=0;i<n;i++){if(d[i]<16)Serial.print('0');Serial.print(d[i],HEX);Serial.print(i+1==n?'\n':' ');}
}
void logPacket(const char* s,const PacketBuffer& p){
  Serial.print('[');Serial.print(s);Serial.print(F("] len="));Serial.println(p.length);printHex(p.data,p.length);
}

int16_t mapJoy(int raw,bool invert){
  int d=raw-JOY_CENTER;if(abs(d)<=DEAD_ZONE)return 0;if(invert)d=-d;
  if(d>0){long v=(long)(d-DEAD_ZONE)*DJI_MAX/(1023-JOY_CENTER-DEAD_ZONE);if(v>DJI_MAX)v=DJI_MAX;return(int16_t)v;}
  long v=(long)(d+DEAD_ZONE)*DJI_MAX/(JOY_CENTER-DEAD_ZONE);if(v<DJI_MIN)v=DJI_MIN;return(int16_t)v;
}
uint16_t djiToSbus(int16_t v){long half=((long)SBUS_MAX-SBUS_MIN)/2;long o=SBUS_MID+((long)v*half)/DJI_MAX;if(o<SBUS_MIN)o=SBUS_MIN;if(o>SBUS_MAX)o=SBUS_MAX;return(uint16_t)o;}

bool buildJoy(PacketBuffer& out){
  JoystickPayload p; p.device_type=(uint8_t)ControllerType::Joystick;p.pitch_speed=pitch;p.roll_speed=roll;p.yaw_speed=yaw;
  auto r=builder.buildCommand(GimbalCmd::CMDSET,GimbalCmd::ExternalDeviceControl,p,ReplyRequirement::NoReply,out);
  if(r.isError()){Serial.print(F("[CAN BUILDER ERROR] "));Serial.println(errorName(r.error()));return false;}return true;
}
bool buildInfo(PacketBuffer& out){
  ObtainInfoRequestPayload p;p.ctrl_byte=0x01;
  auto r=builder.buildCommand(GimbalCmd::CMDSET,GimbalCmd::ObtainInformation,p,ReplyRequirement::ReplyRequired,out);
  if(r.isError()){Serial.print(F("[INFO BUILDER ERROR] "));Serial.println(errorName(r.error()));return false;}return true;
}

void requestInfo(){
  PacketBuffer p;if(!buildInfo(p)){infoErr++;return;}infoTx++;logPacket("CAN TX INFO",p);
  auto tx=ronin.sendPacket(p);if(tx.isError()){infoErr++;Serial.print(F("[CAN INFO TX ERROR] "));Serial.println(errorName(tx.error()));return;}
  Serial.println(F("[CAN INFO] request sent; waiting 100 ms..."));
  auto rx=ronin.receivePacket(100);
  if(rx.isError()){infoNoRx++;Serial.print(F("[CAN INFO] no valid response: "));Serial.println(errorName(rx.error()));return;}
  infoRx++;const ParsedPacket& q=rx.value();
  Serial.println(F("[CAN INFO] *** RONIN RESPONSE ***"));
  Serial.print(F(" CmdSet=0x"));Serial.println(q.command.cmdSet,HEX);
  Serial.print(F(" CmdID=0x"));Serial.println(q.command.cmdId,HEX);
  Serial.print(F(" Length="));Serial.println(q.header.length);
  Serial.print(F(" Seq="));Serial.println(q.header.seq);
  Serial.print(F(" Payload="));Serial.println(q.payloadLen);
  if(q.hasReturnCode){Serial.print(F(" ReturnCode=0x"));Serial.println((uint8_t)q.returnCode,HEX);}
}

bool sendJoy(){
  PacketBuffer p;if(!buildJoy(p))return false;
  Serial.print(F("[CAN JOY] rawA1="));Serial.print(analogRead(JOY_YAW));Serial.print(F(" rawA2="));Serial.print(analogRead(JOY_PITCH));
  Serial.print(F(" Yaw="));Serial.print(yaw);Serial.print(F(" Pitch="));Serial.println(pitch);logPacket("CAN TX JOY",p);
  auto r=ronin.sendPacket(p);if(r.isError()){Serial.print(F("[CAN JOY TX ERROR] "));Serial.println(errorName(r.error()));return false;}return true;
}

void sendSbus(){
  uint16_t ch[SBUSNanoTx::CHANNELS];for(uint8_t i=0;i<SBUSNanoTx::CHANNELS;i++)ch[i]=SBUS_MID;
  ch[CH_YAW]=djiToSbus(yaw);ch[CH_PITCH]=djiToSbus(pitch);ch[CH_RECORD]=recording?SBUS_MAX:SBUS_MIN;
  if(!sbus.write(ch,false,false,false,false)){sbusDrop++;Serial.println(F("[SBUS ERROR] TX busy"));return;}
  sbusOk++;Serial.print(F("[SBUS TX] CH1="));Serial.print(ch[CH_YAW]);Serial.print(F(" CH2="));Serial.print(ch[CH_PITCH]);Serial.print(F(" CH4="));Serial.print(ch[CH_RECORD]);Serial.println(recording?F(" START"):F(" STOP"));
}

void handleButton(){
  bool raw=digitalRead(REC_PIN);if(raw!=buttonLast){buttonChangedAt=millis();buttonLast=raw;}if(millis()-buttonChangedAt<DEBOUNCE_MS)return;
  if(buttonStable==HIGH&&raw==LOW){recording=!recording;Serial.print(F("[REC] "));Serial.println(recording?F("START"):F("STOP"));
    if(ENABLE_CAN){auto r=recording?ronin.camera.recordStart():ronin.camera.recordStop();if(r.isError()){Serial.print(F("[REC CAN ERROR] "));Serial.println(errorName(r.error()));}else Serial.println(F("[REC CAN] command sent"));}}
  buttonStable=raw;
}

void setup(){
  Serial.begin(115200);delay(500);pinMode(REC_PIN,INPUT_PULLUP);builder.resetSequence(0);
  Serial.println(F("=== DJI R SDK / ARDUINO NANO / CAN + SBUS ==="));
  Serial.println(F("A1=Yaw A2=Pitch D2=Record D8=SBUS D10=MCP2515 CS"));
  Serial.println(F("CAN=1Mbps, MCP2515=8MHz, Joystick=20ms"));
  Serial.print(F("SBUS polarity="));Serial.println(SBUS_INVERTED?F("INVERTED"):F("NORMAL"));
  Serial.println(F("Set the same SBUS polarity in Ronin software."));
  if(ENABLE_CAN){auto r=ronin.begin();if(r.isError()){Serial.print(F("[CAN INIT ERROR] "));Serial.println(errorName(r.error()));}else{Serial.println(F("[CAN OK]"));requestInfo();}}
  if(ENABLE_SBUS){if(sbus.begin())Serial.println(F("[SBUS OK] Timer2/D8 transmitter ready"));else Serial.println(F("[SBUS INIT ERROR]"));}
  Serial.println(F("[RUN] No CAN response stops nothing; SBUS continues independently."));
}

void loop(){
  uint32_t now=millis();yaw=mapJoy(analogRead(JOY_YAW),false);pitch=mapJoy(analogRead(JOY_PITCH),true);roll=0;handleButton();
  if(ENABLE_CAN&&now-tJoy>=JOY_MS){tJoy=now;if(sendJoy())canOk++;else canFail++;}
  if(ENABLE_CAN&&now-tInfo>=INFO_MS){tInfo=now;requestInfo();}
  if(ENABLE_SBUS&&now-tSbus>=SBUS_MS){tSbus=now;sendSbus();}
  if(now-tStatus>=STATUS_MS){tStatus=now;Serial.print(F("[STATUS] Y="));Serial.print(yaw);Serial.print(F(" P="));Serial.print(pitch);Serial.print(F(" REC="));Serial.print(recording?F("ON"):F("OFF"));
    Serial.print(F(" | CAN ok="));Serial.print(canOk);Serial.print(F(" fail="));Serial.print(canFail);Serial.print(F(" infoTX="));Serial.print(infoTx);Serial.print(F(" infoRX="));Serial.print(infoRx);Serial.print(F(" noRX="));Serial.print(infoNoRx);
    Serial.print(F(" | SBUS frames="));Serial.print(sbusOk);Serial.print(F(" drop="));Serial.print(sbusDrop);Serial.print(F(" busy="));Serial.println(sbus.busy()?F("YES"):F("NO"));}
}
