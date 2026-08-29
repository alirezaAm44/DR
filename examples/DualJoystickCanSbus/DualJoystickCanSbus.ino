/**
 * DualJoystickCanSbus.ino
 * DJI Ronin R SDK + MCP2515 CAN + SBUS diagnostic test.
 * Uses the standard <mcp2515.h> library directly. MCP2515Driver.h is NOT used.
 * Serial Monitor: 115200 baud.
 *
 * Logs: startup, CAN init, joystick raw/mapped values, CAN TX packets/frames,
 * CAN RX frames, Ronin packet parsing/return codes, attitude, record events,
 * SBUS channels, counters and errors.
 */
#include <SPI.h>
#include <SoftwareSerial.h>
#include <mcp2515.h>
#include <DJIRonin.h>
using namespace dji::ronin;

const bool ENABLE_CAN=true;
const bool ENABLE_SBUS=true;
const bool USE_SBUS_MOTION=true;
const bool LOG_JOYSTICK=true;
const bool LOG_CAN_TX=true;
const bool LOG_CAN_RX_FRAMES=true;
const bool LOG_CAN_RX_PACKET=true;
const bool LOG_SBUS=true;
const bool LOG_SBUS_FRAME_HEX=false;
const bool LOG_STATUS=true;

const int JOYSTICK_X_PIN=A1, JOYSTICK_Y_PIN=A2, JOYSTICK_SW_PIN=2;
const int MCP2515_CS_PIN=10, SBUS_TX_PIN=8;
const CAN_CLOCK MCP_CLOCK=MCP_8MHZ;
const int JOYSTICK_CENTER=512, DEAD_ZONE=40;
const int16_t DJI_MIN=-7500, DJI_MAX=7500;
const uint16_t SBUS_MIN=352, SBUS_MID=1024, SBUS_MAX=1696;
const unsigned long SBUS_INTERVAL_MS=14;
const uint8_t SBUS_CH_YAW=0, SBUS_CH_PITCH=1, SBUS_CH_ROLL=3;
SoftwareSerial sbusSerial(9,SBUS_TX_PIN);
const unsigned long JOYSTICK_INTERVAL_MS=20, INFO_INTERVAL_MS=200, STATUS_PRINT_MS=200, DEBOUNCE_MS=40;

MCP2515 mcp2515(MCP2515_CS_PIN);
PacketBuilder builder;
PacketParser parser;
unsigned long lastJoystickMs=0,lastInfoMs=0,lastStatusMs=0,lastSbusMs=0,lastDebounceMs=0;
uint32_t txJoystickCount=0,txInfoCount=0,txCameraCount=0,txSbusCount=0,txCanFrameCount=0,rxCanFrameCount=0,rxPacketCount=0,rxErrorCount=0,canSendFailCount=0,buildErrorCount=0;
int16_t lastYaw=0,lastPitch=0,lastRoll=0,attYaw=0,attRoll=0,attPitch=0;
int lastRawX=JOYSTICK_CENTER,lastRawY=JOYSTICK_CENTER;
bool haveAttitude=false,isRecording=false,lastButtonStable=HIGH,lastButtonRead=HIGH;
uint8_t rxAcc[MAX_PACKET_SIZE];
uint16_t rxAccLen=0,rxExpectedLen=0,sbusChannels[16];

void printHex(const uint8_t* d,size_t n){for(size_t i=0;i<n;i++){if(d[i]<0x10)Serial.print('0');Serial.print(d[i],HEX);if(i+1<n)Serial.print(' ');}Serial.println();}
void printCanFrame(const struct can_frame& f,const char* dir){Serial.print(dir);Serial.print(F(" ID=0x"));Serial.print(f.can_id&0x7FF,HEX);Serial.print(F(" DLC="));Serial.print(f.can_dlc);Serial.print(F(" DATA="));printHex(f.data,f.can_dlc);}
int16_t mapJoystickToDJI(int raw,bool invert=false){int d=raw-JOYSTICK_CENTER;if(abs(d)<DEAD_ZONE)return 0;if(invert)d=-d;long m=(long)d*DJI_MAX/(JOYSTICK_CENTER-DEAD_ZONE);if(m>DJI_MAX)m=DJI_MAX;if(m<DJI_MIN)m=DJI_MIN;return(int16_t)m;}
uint16_t mapDjiToSbus(int16_t v){long span=(long)SBUS_MAX-SBUS_MIN;long m=(long)SBUS_MID+((long)v*(span/2))/DJI_MAX;if(m<SBUS_MIN)m=SBUS_MIN;if(m>SBUS_MAX)m=SBUS_MAX;return(uint16_t)m;}
void sbusResetChannels(){for(uint8_t i=0;i<16;i++)sbusChannels[i]=SBUS_MID;}
void updateSbusFromJoystick(int16_t y,int16_t p,int16_t r){sbusResetChannels();if(!USE_SBUS_MOTION)return;sbusChannels[SBUS_CH_YAW]=mapDjiToSbus(y);sbusChannels[SBUS_CH_PITCH]=mapDjiToSbus(p);sbusChannels[SBUS_CH_ROLL]=mapDjiToSbus(r);}
void sbusPackAndSend(){uint8_t packet[25];packet[0]=0x0F;uint8_t bi=1,bc=0;uint32_t bb=0;for(uint8_t ch=0;ch<16;ch++){uint16_t v=sbusChannels[ch]&0x07FF;bb|=((uint32_t)v)<<bc;bc+=11;while(bc>=8){packet[bi++]=(uint8_t)(bb&0xFF);bb>>=8;bc-=8;}}if(bc>0&&bi<23)packet[bi++]=(uint8_t)(bb&0xFF);while(bi<23)packet[bi++]=0;packet[23]=0;packet[24]=0;for(uint8_t i=0;i<25;i++)sbusSerial.write(packet[i]);txSbusCount++;if(LOG_SBUS){Serial.print(F("[SBUS TX] #"));Serial.print(txSbusCount);Serial.print(F(" CH1="));Serial.print(sbusChannels[0]);Serial.print(F(" CH2="));Serial.print(sbusChannels[1]);Serial.print(F(" CH4="));Serial.print(sbusChannels[3]);if(LOG_SBUS_FRAME_HEX){Serial.print(F(" FRAME="));printHex(packet,25);}else Serial.println();}}

bool sendPacketMultiFrame(const PacketBuffer& packet,const char* label){size_t off=0;uint8_t fi=0,total=(packet.length+7)/8;if(LOG_CAN_TX){Serial.print(F("[CAN TX PACKET] "));Serial.print(label);Serial.print(F(" len="));Serial.print(packet.length);Serial.print(F(" frames="));Serial.println(total);Serial.print(F("  PACKET: "));printHex(packet.data,packet.length);}while(off<packet.length){struct can_frame f;f.can_id=CAN_ID_TX;f.can_dlc=0;while(f.can_dlc<8&&off<packet.length)f.data[f.can_dlc++]=packet.data[off++];fi++;MCP2515::ERROR e=mcp2515.sendMessage(&f);if(e!=MCP2515::ERROR_OK){canSendFailCount++;Serial.print(F("[CAN TX ERROR] "));Serial.print(label);Serial.print(F(" frame="));Serial.print(fi);Serial.print(F(" error="));Serial.println((int)e);return false;}txCanFrameCount++;if(LOG_CAN_TX){Serial.print(F("  TX frame "));Serial.print(fi);Serial.print('/');Serial.print(total);Serial.print(F(": "));printHex(f.data,f.can_dlc);}}return true;}

void handleCompleteRxPacket(const uint8_t* data,uint16_t len){rxPacketCount++;Serial.println(F("========== RONIN RX PACKET =========="));Serial.print(F("Length="));Serial.println(len);Serial.print(F("RAW: "));printHex(data,len);auto result=parser.parse(data,len);if(result.isError()){rxErrorCount++;Serial.println(F("[RX PARSE ERROR] PacketParser rejected packet"));Serial.println(F("======================================"));return;}ParsedPacket pkt=result.value();Serial.print(F("[RONIN RX] CmdSet=0x"));Serial.print(pkt.command.cmdSet,HEX);Serial.print(F(" CmdID=0x"));Serial.println(pkt.command.cmdId,HEX);Serial.print(F("[RONIN RX] PayloadLen="));Serial.println(pkt.payloadLen);if(pkt.hasReturnCode){uint8_t rc=static_cast<uint8_t>(pkt.returnCode);Serial.print(F("[RONIN RX] ReturnCode=0x"));if(rc<0x10)Serial.print('0');Serial.println(rc,HEX);if(pkt.returnCode==ReturnCode::Success)Serial.println(F("[RONIN RX] SUCCESS"));else{rxErrorCount++;Serial.println(F("[RONIN RX] ERROR"));}}else Serial.println(F("[RONIN RX] No return code"));if(pkt.command.cmdSet==GimbalCmd::CMDSET&&pkt.command.cmdId==GimbalCmd::ObtainInformation&&pkt.payloadLen>=sizeof(ObtainInfoReplyPayload)){const ObtainInfoReplyPayload* rep=reinterpret_cast<const ObtainInfoReplyPayload*>(pkt.payload);haveAttitude=true;attYaw=rep->yaw;attRoll=rep->roll;attPitch=rep->pitch;Serial.println(F("[ATTITUDE] ObtainInformation reply"));Serial.print(F("  Yaw="));Serial.print(attYaw/10.0f,1);Serial.println(F(" deg"));Serial.print(F("  Roll="));Serial.print(attRoll/10.0f,1);Serial.println(F(" deg"));Serial.print(F("  Pitch="));Serial.print(attPitch/10.0f,1);Serial.println(F(" deg"));}Serial.println(F("======================================"));}
void pollIncoming(){if(!ENABLE_CAN)return;struct can_frame f;while(mcp2515.readMessage(&f)==MCP2515::ERROR_OK){rxCanFrameCount++;if(LOG_CAN_RX_FRAMES)printCanFrame(f,"[CAN RX]");if((f.can_id&0x7FF)!=CAN_ID_RX){if(LOG_CAN_RX_FRAMES)Serial.println(F("  -> ignored (CAN ID != CAN_ID_RX)"));continue;}for(uint8_t i=0;i<f.can_dlc;i++){uint8_t b=f.data[i];if(rxAccLen==0){if(b!=SOF)continue;rxAcc[0]=b;rxAccLen=1;rxExpectedLen=0;continue;}if(rxAccLen<MAX_PACKET_SIZE)rxAcc[rxAccLen++]=b;else{rxErrorCount++;Serial.println(F("[RX ERROR] accumulator overflow"));rxAccLen=0;rxExpectedLen=0;continue;}if(rxAccLen==3){uint16_t vl=(uint16_t)rxAcc[1]|((uint16_t)rxAcc[2]<<8);rxExpectedLen=vl&LENGTH_MASK;if(LOG_CAN_RX_PACKET){Serial.print(F("[RX] ExpectedLen="));Serial.println(rxExpectedLen);}if(rxExpectedLen<MIN_PACKET_SIZE||rxExpectedLen>MAX_PACKET_SIZE){rxErrorCount++;Serial.println(F("[RX ERROR] invalid length"));rxAccLen=0;rxExpectedLen=0;continue;}}if(rxExpectedLen>0&&rxAccLen>=rxExpectedLen){handleCompleteRxPacket(rxAcc,rxExpectedLen);rxAccLen=0;rxExpectedLen=0;}}}}

bool sendJoystick(int16_t yaw,int16_t pitch,int16_t roll){JoystickPayload p;p.device_type=static_cast<uint8_t>(ControllerType::Joystick);p.pitch_speed=pitch;p.roll_speed=roll;p.yaw_speed=yaw;PacketBuffer packet;auto r=builder.buildCommand(GimbalCmd::CMDSET,GimbalCmd::ExternalDeviceControl,p,ReplyRequirement::NoReply,packet);if(r.isError()){buildErrorCount++;Serial.println(F("[BUILD ERROR] ExternalDeviceControl"));return false;}return sendPacketMultiFrame(packet,"Joystick");}
bool sendObtainInfo(uint8_t infoType){ObtainInfoRequestPayload p;p.ctrl_byte=infoType;PacketBuffer packet;auto r=builder.buildCommand(GimbalCmd::CMDSET,GimbalCmd::ObtainInformation,p,ReplyRequirement::ReplyRequired,packet);if(r.isError()){buildErrorCount++;Serial.println(F("[BUILD ERROR] ObtainInformation"));return false;}return sendPacketMultiFrame(packet,"ObtainInformation");}
bool sendCameraRecord(bool start){CameraMotionPayload p;p.command=static_cast<uint16_t>(start?CameraMotionCommand::StartRecording:CameraMotionCommand::StopRecording);PacketBuffer packet;auto r=builder.buildCommand(CameraCmd::CMDSET,CameraCmd::Motion,p,ReplyRequirement::NoReply,packet);if(r.isError()){buildErrorCount++;Serial.println(F("[BUILD ERROR] Camera Motion"));return false;}bool ok=sendPacketMultiFrame(packet,start?"Camera START":"Camera STOP");if(ok){txCameraCount++;Serial.print(F(">>> CAMERA "));Serial.println(start?F("START"):F("STOP"));}return ok;}
void handleRecordButton(){bool reading=digitalRead(JOYSTICK_SW_PIN);if(reading!=lastButtonRead){lastDebounceMs=millis();lastButtonRead=reading;}if(millis()-lastDebounceMs<DEBOUNCE_MS)return;if(lastButtonStable==HIGH&&reading==LOW){isRecording=!isRecording;Serial.print(F("[BUTTON] Record -> "));Serial.println(isRecording?F("START"):F("STOP"));if(ENABLE_CAN)sendCameraRecord(isRecording);}lastButtonStable=reading;}
void printStatus(){Serial.println(F("---------- STATUS ----------"));Serial.print(F("RAW X/Y="));Serial.print(lastRawX);Serial.print('/');Serial.println(lastRawY);Serial.print(F("DJI Y/P/R="));Serial.print(lastYaw);Serial.print('/');Serial.print(lastPitch);Serial.print('/');Serial.println(lastRoll);Serial.print(F("CAN TX joystick/info/camera="));Serial.print(txJoystickCount);Serial.print('/');Serial.print(txInfoCount);Serial.print('/');Serial.println(txCameraCount);Serial.print(F("CAN TX frames/fail="));Serial.print(txCanFrameCount);Serial.print('/');Serial.println(canSendFailCount);Serial.print(F("CAN RX frames/packets/errors="));Serial.print(rxCanFrameCount);Serial.print('/');Serial.print(rxPacketCount);Serial.print('/');Serial.println(rxErrorCount);Serial.print(F("Build errors="));Serial.println(buildErrorCount);Serial.print(F("SBUS TX="));Serial.println(txSbusCount);Serial.print(F("Recording="));Serial.println(isRecording?F("ON"):F("OFF"));if(haveAttitude){Serial.print(F("Attitude Y/R/P="));Serial.print(attYaw/10.0f,1);Serial.print('/');Serial.print(attRoll/10.0f,1);Serial.print('/');Serial.println(attPitch/10.0f,1);}else Serial.println(F("Attitude=no valid reply yet"));Serial.println(F("----------------------------"));}

void setup(){Serial.begin(115200);delay(500);pinMode(JOYSTICK_SW_PIN,INPUT_PULLUP);Serial.println();Serial.println(F("=============================================="));Serial.println(F(" DJI RONIN CAN + SBUS DIAGNOSTIC TEST"));Serial.println(F(" DualJoystickCanSbus"));Serial.println(F("=============================================="));Serial.println(F("[INFO] MCP2515Driver.h is NOT required."));Serial.println(F("[INFO] Using standard <mcp2515.h>."));Serial.println(F("[INFO] Serial Monitor = 115200 baud"));Serial.print(F("[CONFIG] CAN="));Serial.print(ENABLE_CAN?F("ON"):F("OFF"));Serial.print(F(" SBUS="));Serial.print(ENABLE_SBUS?F("ON"):F("OFF"));Serial.print(F(" SBUS_MOTION="));Serial.println(USE_SBUS_MOTION?F("ON"):F("OFF"));if(ENABLE_SBUS){sbusSerial.begin(100000);sbusResetChannels();Serial.println(F("[SBUS] TX D8 -> 74HC04 -> RSA pin 3"));Serial.println(F("[SBUS] 100000 baud; SoftwareSerial is not true 8E2."));}else Serial.println(F("[SBUS] DISABLED"));if(ENABLE_CAN){Serial.println(F("[CAN] SPI.begin()"));SPI.begin();Serial.println(F("[CAN] MCP2515 reset"));mcp2515.reset();delay(50);Serial.println(F("[CAN] 1000 kbps / 8 MHz"));if(mcp2515.setBitrate(CAN_1000KBPS,MCP_CLOCK)!=MCP2515::ERROR_OK){Serial.println(F("[CAN ERROR] setBitrate failed"));while(1){}}Serial.println(F("[CAN] Normal mode"));if(mcp2515.setNormalMode()!=MCP2515::ERROR_OK){Serial.println(F("[CAN ERROR] setNormalMode failed"));while(1){}}mcp2515.setFilterMask(MCP2515::MASK0,false,0);mcp2515.setFilterMask(MCP2515::MASK1,false,0);mcp2515.setFilter(MCP2515::RXF0,false,0);builder.resetSequence(0);Serial.println(F("[CAN] READY"));Serial.print(F("[CAN] TX=0x"));Serial.print(CAN_ID_TX,HEX);Serial.print(F(" RX=0x"));Serial.println(CAN_ID_RX,HEX);}else Serial.println(F("[CAN] DISABLED"));Serial.println(F("[WIRING] CANH->RSA4 CANL->RSA2 GND->RSA6"));Serial.println(F("[WIRING] AD_COM->10k/47k->GND; SBUS->74HC04->RSA3"));Serial.println(F("=============================================="));Serial.println(F(" TEST STARTED"));}

void loop(){if(ENABLE_CAN){pollIncoming();handleRecordButton();}unsigned long now=millis();lastRawX=analogRead(JOYSTICK_X_PIN);lastRawY=analogRead(JOYSTICK_Y_PIN);lastYaw=mapJoystickToDJI(lastRawX,false);lastPitch=mapJoystickToDJI(lastRawY,true);lastRoll=0;if(ENABLE_CAN&&now-lastJoystickMs>=JOYSTICK_INTERVAL_MS){lastJoystickMs=now;if(LOG_JOYSTICK){Serial.print(F("[JOYSTICK] RAW X="));Serial.print(lastRawX);Serial.print(F(" Y="));Serial.print(lastRawY);Serial.print(F(" -> Yaw="));Serial.print(lastYaw);Serial.print(F(" Pitch="));Serial.print(lastPitch);Serial.print(F(" Roll="));Serial.println(lastRoll);}if(sendJoystick(lastYaw,lastPitch,lastRoll))txJoystickCount++;}if(ENABLE_CAN&&now-lastInfoMs>=INFO_INTERVAL_MS){lastInfoMs=now;Serial.println(F("[RONIN TX] ObtainInformation type=0x01"));if(sendObtainInfo(0x01))txInfoCount++;}if(ENABLE_SBUS&&now-lastSbusMs>=SBUS_INTERVAL_MS){lastSbusMs=now;updateSbusFromJoystick(lastYaw,lastPitch,lastRoll);sbusPackAndSend();}if(LOG_STATUS&&now-lastStatusMs>=STATUS_PRINT_MS){lastStatusMs=now;printStatus();}}
