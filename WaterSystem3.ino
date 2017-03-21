#include <EEPROM.h>

#define  DEBUG_PRINT
#include <Wire.h>
//#include "/home/charlotte/sketchbook/libraries/LiquidCrystal_I2C2004V1/LiquidCrystal_I2C.h"
#include <LiquidCrystal_I2C.h>
//#define  USINGHALLSENSOR
//#define  LCDCONNECTED
/*
WaterSystem.ino
developed by James Deric Lee
expanded and maintained by J.Deric Lee & Gary Lee
*/

#include <avr/wdt.h>
//#include <Time/Time.h>
//#include "/home/charlotte/sketchbook/libraries/OneWire/OneWire.h"
//#include  "/home/charlotte/sketchbook/libraries/NewliquidCrystal/LiquidCrystal_I2C.h"

#define EEPROM_WGPM                        0
#define EEPROM_CIST_VOL_AT_CENTER_FILLING  1
#define EEPROM_FILL_DELAY_SECS             2
#define EEPROM_CIST_VOL_AT_CENTER_DRAWING  3
#define EEPROM_PGPS                        4
#define EEPROM_PUMP_OFF_PSI                5
#define EEPROM_PUMP_ON_PSI                 6
#define EEPROM_TOTAL_GAL_PRESSURE_PUMP_L     7
#define EEPROM_TOTAL_GAL_PRESSURE_PUMP_M	 10
#define EEPROM_TOTAL_GAL_PRESSURE_PUMP_B4to7  9
#define EEPROM_TOTAL_GAL_COMPRESSOR_B0to3     9 
#define EEPROM_TOTAL_GAL_COMPRESSOR_L        8
#define EEPROM_TOTAL_GAL_COMPRESSOR_M		 11
#define EEPROM_UPDATE(BITE,NEWVAL) {if((byte)NEWVAL!=EEPROM.read(BITE))EEPROM.write(BITE,(byte)NEWVAL);}

#define	DEFAULT_WGPM	14.55 //11.00
#define DEFAULT_PGPM    7.5  //7.0
#define	CIST_VOL_AT_CENTER_FILLING	34.50
#define  DEFAULT_FILL_DELAY_SECS  20.0
#define	CIST_VOL_AT_CENTER_DRAWING	29.0
#define	DEFAULT_PGPS	0.027
#define  PUMP_OFF_PSI  45.0
#define  PUMP_ON_PSI  37.0
#define AVGR_LOOPS  10
#define THROW_EXCEPTION(EXCEPTION)  {if(wsException!=EXCEPTION){wsException=EXCEPTION; wsExceptionHandler();}}
#define LCDP(COL,ROW,ARG)  {lcd.setCursor(COL,ROW);lcd.print(ARG);}
#define LCDPF(COL,ROW,ARG,RESOLUTION)  {lcd.setCursor(COL,ROW);lcd.print(ARG,RESOLUTION);}
//Analog pins
#define pinPressureSensorTPSI  A0
// Digital Pins
const int ACssr = 6;                // Air Compressor SSR control 
const int DCpump = 7;               // 50V DC water Pump SSR
const int sysOnline = 4;            // System Main Switch  
const int pinCisCritHi=13;        //voltage high=critically high
const int pinCisCritLo=10;        //voltage high=critically low
const int pinPumpTestSw=8;
const int pinCistCenterSensor=9;  //voltage high=level >= half
const int pinHighLoadSig=5;
const int pinPumpCounter=3; //pin 3 is interrupt 1
#define pinAlarm 12

int LoopDelay=125;

#ifndef	LCDCONNECTED
//Addr: 0x3F, 20 chars & 4 lines
LiquidCrystal_I2C lcd(0x3F,20,4); 
#endif


#define  chCistern 0
#define  chDrain   1
#define  chFill    2
//#define  chDnArrow  
byte  chDfillArrow[8]{0,1,18,20,24,30,0};
byte  chDdrainArrow[8]{0,16,9,5,3,15,0};
byte  chDcisternH[8]{0,0,31,31,31,31,31};
byte  chDcisternLH[8]{17,17,17,17,17,31,31};
byte  chDcisternUH[8]{17,17,31,31,31,31,31};
byte  chDnothing[8]{0,0,0,0,0,0,0};

byte  chDcisternCalc[8]{17,17,17,17,17,17,17};

void setCisternChar(float ratio)
{
  int i;
  for (i=6;i>=0;i--)
  {
    if(ratio>((6.0-(float)i)*(1.0/6.0)))chDcisternCalc[i]=31; else chDcisternCalc[i]=17;
  }
  lcd.createChar(chCistern,chDcisternCalc);
}

void idleLoop(void);
void myDelay(unsigned long milSec);
void commandHandler(char *cmd, char*val);

volatile int pumpCounter=0;

void PumpCounterISR(void){if(++pumpCounter>2)pumpCounter=2;};

unsigned long elapsedMillis(unsigned long startT, unsigned long endT)
{
  if (startT>endT) 
  {
    return ((0xFFFFFFFF - startT) + endT);
  }
  else
  {
    return (endT-startT);
  }
}

// **************************************************

class digitalSwitch
{
  int ioPin;
  boolean invert;
  boolean locked1, locked2, locked3;
  byte  * chOff;
  byte  * chOn;
  byte  chNum;
  int  chRow,chCol;
  char dsName[12];
public:
  digitalSwitch(char *,int,boolean);
  int status(void){return (digitalRead(ioPin)==((invert) ? LOW : HIGH));};
  void on(void);
  void off(void);//{digitalWrite(ioPin,((invert) ? HIGH : LOW));if(chNum<8)lcd.createChar(chNum,chOff);};  
  void LockOut1(boolean newVal){locked1=newVal; if(locked1==true)off();};
  boolean LockOut1(void){return locked1;};
  void LockOut2(boolean newVal){locked2=newVal; if(locked2==true)off();};
  boolean LockOut2(void){return locked2;};
  void LockOut3(boolean newVal){locked3=newVal; if(locked3==true)off();};
  boolean LockOut3(void){return locked3;};
  void print(char *);
  void  chInit(byte charNum, byte* offCh, byte* onCh,int col,int row)
        {
          chOff=offCh; chOn=onCh; chNum=charNum; chRow=row, chCol=col;
          if(chNum<8){lcd.createChar(chNum,chOff);lcd.setCursor(chCol,chRow);lcd.write(chNum);}
        };
};
void digitalSwitch::off(void)
{
  digitalWrite(ioPin,((invert) ? HIGH : LOW));
  print("Off");
  if(chNum<8)
  {
    lcd.createChar(chNum,chOff);
  }
}  
void digitalSwitch::on(void)
{
  if((locked1==false)&&(locked2==false)&&(locked3==false))
  {
    digitalWrite(ioPin,((invert) ? LOW : HIGH));
    print("On");
    if(chNum<8)
    {
      lcd.createChar(chNum,chOn);
    }
  }
  else
  {
    Serial.print(dsName);
    Serial.print(locked1);
    Serial.print(locked2);
    Serial.println(locked3);
  }
}
void digitalSwitch::print(char *msg)
{
#ifdef DEBUG_PRINT
  Serial.print(dsName);Serial.print(F(" : "));Serial.println(msg);
#endif
}
digitalSwitch::digitalSwitch(char *objName,int pin, boolean invertLogic=false)
{
  ioPin=pin;
  dsName[11]='\0';
  strncpy(dsName,objName,11);
  locked1=false;locked2=false;locked3=false;
  pinMode(ioPin,OUTPUT);
  invert=invertLogic;
  chNum=0xFF;
  off();
}

digitalSwitch WellCompressor("Well Comp",ACssr/*,false,chFill,chDnothing,chDfillArrow,19,0*/), WaterPressurePump("Pres Pump",DCpump/*,false,chDrain,chDnothing,chDdrainArrow,19,2*/);

// **************************************************

class wsAlarm:protected digitalSwitch
{
	unsigned long st, dur;
//	class digitalSwitch  sounder(pinAlarm);
public:
	void poll(void);
	void sound(unsigned long duration);
	wsAlarm(char *objName,int IOpin):digitalSwitch(objName,IOpin){st=0;dur=0;};
};
void wsAlarm::sound(unsigned long duration)
{
	if(duration==0){off();return;}
	dur=duration;
	on();
	st=millis();
}
void wsAlarm::poll(void)	
{
	if((dur>0)&&(elapsedMillis(st,millis())>=dur))
	{
		off();
		dur=0;
	}
}	
wsAlarm wsAlert("Alarm",pinAlarm);	

enum wsExceptions{
  NO_EXCEPTION,
  CISTERN_LOW,
  CISTERN_CRIT_LOW,
  CISTERN_LOW_SENSE_NORM,
  CISTERN_HIGH,
  CISTERN_CRIT_HIGH,
  CISTERN_HIGH_SENSE_NORM,
  CISTERN_HALF,
  CISTERN_NORM,
  SENSE_ERR_HI_LO,
  SENSE_ERR_HI_HALF,
  SENSE_ERR_LO_HALF,
  PSI_OVER_MAX,
  PSI_UNDER_MIN,
  LOAD_NORM,
  LOAD_HIGH
}wsException=NO_EXCEPTION;

void wsExceptionHandler(void);
// ****************************************************************************

class averager
{
 long readings[AVGR_LOOPS];
 long total;
 int index;
 
public:
  int ioPin;
  averager(int);
  void apoll(void);
  int avgValue(void) { return (total/AVGR_LOOPS);};
};

averager::averager(int pin=0)
{
  int i;
  ioPin=pin;
  total=0;index=0;
  for (i=0;i<AVGR_LOOPS;i++)
  {
    //readings[i]=0;
    apoll();
  }
}
void averager::apoll(void)
{
  int reading=analogRead(ioPin);  //Serial.println(reading);
  total-=readings[index];
  readings[index]=reading;
  total+=reading;
  if (++index >= AVGR_LOOPS) index=0;
}
//************************************************
class analogSensor:public averager  //second version -- using inheritance
{
  long fl, fh, tl, th;
  float lThres, hThres,hThreslimit;
  wsExceptions LwsE, HwsE;
  int resolution;
public:
  analogSensor(int,long,long,long,long,float,wsExceptions,float,wsExceptions,int,float);
  long lValue();
  float value();
  void poll(boolean);
  void  lowThresholdValue(float newVal){lThres=newVal;};
  float lowThresholdValue(void){return lThres;};
  void   highThresholdValue(float newVal){ hThres=((newVal>hThreslimit) ? hThreslimit : newVal);};
  float  highThresholdValue(void){return hThres;};  
  void  range(float low,float high){lThres=low; hThres=high;};
};
void analogSensor::poll(boolean initilize=false) 
{
  apoll();
  if(initilize==false)
  {
     if(value()<lThres)
     {
       wsException=LwsE;//PSI_UNDER_MIN;
       wsExceptionHandler();
     }
     if(value()>hThres)
     {
         wsException=HwsE;//PSI_OVER_MAX;
         wsExceptionHandler();
     }
   }
}
analogSensor::analogSensor(int pin,long fromLow, long fromHigh, long toLow, long toHigh, 
            float lowThreshold, wsExceptions lowExcept, float hiThreshold, wsExceptions hiExcept, int res=10, float hiTresholdLimit=56.0):averager(pin)
{
  ioPin=pin;
  fl=fromLow;
  fh=fromHigh;
  tl=toLow;
  th=toHigh;
   lThres=lowThreshold; 
   hThres=hiThreshold;
   LwsE=lowExcept;
   HwsE=hiExcept;
   resolution=res;
   hThreslimit=hiTresholdLimit;
   {
     int i;
     for(i=0;i<AVGR_LOOPS;i++){poll(true);}
   }
}

long analogSensor::lValue()
{
  return map((long)avgValue(),fl,fh,tl,th);
}

float analogSensor::value()
{
  return ((float)map((long)avgValue(),fl,fh, tl*resolution, th*resolution)/(float)resolution);
}

//************************************************  

class digitalSensor
{
  boolean state, invert;
  int count, ioPin, threshold;
public:
  digitalSensor(int,int,boolean,int);
  void (*onChange)(boolean);
  void poll();
  boolean read(void);
};
boolean digitalSensor::read(void)
{
  boolean logic=((invert) ? (!state) : state);
  return logic; 
}
digitalSensor::digitalSensor(int pin,int loopCountToChange=10, boolean invertLogic=false, int pm=INPUT_PULLUP)
{
  pinMode((ioPin=pin),pm);
  onChange=NULL;
  threshold=loopCountToChange;
  invert=invertLogic;
  state=digitalRead(ioPin);
}
void digitalSensor::poll()
{
  boolean reading = digitalRead(ioPin);
  if(reading!=state)
  {
    if(++count>=threshold)
    {
      state=reading;
      if (onChange!=NULL) onChange(state);
      count=0;
    }
  }else count=0;
}

//**********************************************************************
#ifdef  LCDCONNECTED
class LCDcontrol:LiquidCrystal_I2C
{
  
public:
//  LCDcontrol(uint8_t lcd_Addr,uint8_t lcd_cols,uint8_t lcd_rows) : LiquidCrystal_I2C(lcd_Addr,lcd_cols,lcd_rows);
  LCDcontrol(uint8_t,uint8_t,uint8_t);
  void   CRprint(int,int,char *);
  void  CRprint(int,int,float);
};

LCDcontrol::LCDcontrol(uint8_t lcd_Addr,uint8_t lcd_cols,uint8_t lcd_rows) : LiquidCrystal_I2C(lcd_Addr,lcd_cols,lcd_rows)
{
  init(); 
//  begin();
  backlight();
  setCursor(4, 1);
  print("Please Wait");
  delay(1000);
}

void LCDcontrol::CRprint(int col, int row, char *text)
{
  setCursor(col, row);  
  print(text);
}
void  LCDcontrol::CRprint(int col,int row,float val)
{
  setCursor(col, row);
  print(ftoa(val));
}
LCDcontrol lcd(0x3F,20,4);
#endif
//********************************************************************

class textBuffer
{
  char buff[32];
  int index;
public:
//  textBuffer();
  int put(byte ch);
  char * getText(void);
  void TBreset(void){index=0;};
  textBuffer(void){index=0;};
};

char * textBuffer::getText(void)
{
  buff[index]=0;
  return buff;
}

int textBuffer::put(byte ch)
{
  if (index>=(sizeof(buff)/sizeof(char))) return -1;
  if ((ch>=32) && (ch<=126)) buff[index++]=toupper(ch);
  if (ch==8) index--;
  return buff[index];
}

//***************************************************************

class serialIO : public textBuffer
{
  enum modes
  {
    waiting,
    cmdRead,
    valueRead
  } mode;
  char cmd[32];
  char val[32];
  void (*commandRecieved)(char *,char *);
public:
  serialIO(int,void (*func)(char*,char*));
  void poll(void);
  void SIOreset(void);
};
void serialIO::poll(void)
{
  while (Serial.available()) 
  {
    byte inByte;
    inByte = Serial.read();
    //check for control char or delimiters
//    if(mode==waiting)
//    {
      if(inByte=='<')
      {
        mode=cmdRead;
        Serial.write(inByte);
        TBreset();
        return;
      }
    if(mode==waiting)      return;
//    }
 /*   else*/ if(mode==cmdRead)
    {
      if((inByte==':')||(inByte=='='))
      {
        strcpy(cmd,getText());
        mode=valueRead;
        TBreset();
        return;
      }
      else if(inByte=='>')
      {
        strcpy(cmd,getText());
        mode=waiting;
        TBreset();
        strcpy(val,"0");
        commandRecieved(cmd,val);
      }
    }
    else if(mode==valueRead)
    {
      if(inByte=='>')
      {
        strcpy(val,getText());
        mode=waiting;
        TBreset();
        commandRecieved(cmd,val);
        return;
      }
    } 
    if(put(inByte)<0)  //overflow returns -1
    {
      TBreset();
 //     SIOreset();  Don't remember what this was and wouldn't compile due to undefined function
    }
  }
}

serialIO::serialIO(int baud, void (*func)(char* /*cmd*/,char* /*value*/))
{
  mode=waiting;
/*  Serial.begin(baud); does not seem to work when called from here.  Works in setup function
  while (!Serial) 
  {
    ; // wait for serial port to connect. Needed for Leonardo only
  }*/
  commandRecieved=func;
}

// ***********************************************************************************************

digitalSensor topCistern(pinCisCritHi), midCistern(pinCistCenterSensor,20), bottomCistern(pinCisCritLo);
digitalSensor OnLine(sysOnline,3), HighAmpLoad(pinHighLoadSig,3), PumpTest(pinPumpTestSw,3);
analogSensor TankPSI(pinPressureSensorTPSI,99, 919, 0, 100,PUMP_ON_PSI,PSI_UNDER_MIN,PUMP_OFF_PSI,PSI_OVER_MAX);


//*******************************************************************************************

class cistern
{
  float  capacity;    //cistern size in gallons
  float  headspace;          //headspace in gallons and level of top sensor -- hard stop of compressor
  float  bottomspace;   //minimum liquid to draw properly and level of bottom sensor -- pressure pump hard stopped
  float  normFull;    //normal level considered full and compressor stops
  float  normEmpty;    //normal level considered empty and compressor starts
  float  fillGPM;  //average GPM filling from well
  float  fillDelaySecs;   //average seconds before water arrives from well after compressor starts - seconds water flows after compressor stops i.e. cycle overhead in seconds
  float  drawGPM;  //average GPM drawn by pressure pump
  float  drawGPStroke;
  float  level;        //Assumed water level. Constant is initial assumpsion of cistern level and will be corrected by sensor change
  unsigned long  mil, compressorStartMil;
  float pumpTotal, wellTotal;
  float pumpCCA, wellCCA;
  bool calibrated;
  
public:
  void  update(boolean, boolean, boolean, boolean, boolean);
  float volume(void){return level;};
  void  volume(float newVal){level=newVal;};
  digitalSensor *dsT, *dsM, *dsB;
  void topSensorChanged(boolean);
  void midSensorChanged(boolean);
  void lowSensorChanged(boolean);
  float  fillGPMval(void){return fillGPM;};
  void   fillGPMval(float newVal){fillGPM=newVal;};
  float  fillDelay(void){return fillDelaySecs;};
  void   fillSelay(float  FDsecs){fillDelaySecs=FDsecs;};
#ifndef USINGHALLSENSOR
  float  drawGPMval(void){return drawGPM;};
  void  drawGPMval(float newVal){drawGPM=newVal;};
#else
  float  drawGPSval(void){return drawGPStroke;};
  void  drawGPSval(float newVal){drawGPStroke=newVal;};
#endif
  float  TotalGalPressurePump(void){return pumpTotal;};
  float  TotalGalCompressor(void){return wellTotal;};
  void  TotalGalPressurePump(float newVal){pumpTotal=newVal;};
  void  TotalGalCompressor(float newVal){wellTotal=newVal;};  
  float  WCCA(void){return wellCCA;};
  float  PCCA(void){return pumpCCA;};
  int status(void){return ((volume()<normEmpty)?(-1):(volume()>normFull)?1:0);};
  cistern(void);
};
cistern::cistern(void)
{
  calibrated=false;
  capacity=60.0;    //cistern size in gallons
  headspace=5.0;          //headspace in gallons and level of top sensor -- hard stop of compressor
  bottomspace=10.0;   //minimum liquid to draw properly and level of bottom sensor -- pressure pump hard stopped
  normFull=50;    //normal level considered full and compressor stops
  normEmpty=15.0;    //normal level considered empty and compressor starts
  fillGPM=DEFAULT_WGPM;  //average GPM filling from well
  fillDelaySecs=DEFAULT_FILL_DELAY_SECS;   //average seconds before water arrives from well after compressor starts - seconds water flows after compressor stops i.e. cycle overhead in seconds
#ifndef USINGHALLSENSOR
  drawGPM=DEFAULT_PGPM;  //average GPM drawn by pressure pump
#else
  drawGPStroke=DEFAULT_PGPS;
#endif
  level=30.0;        //Assumed water level. Constant is initial assumpsion of cistern level and will be corrected by sensor change
  mil=0; 
  compressorStartMil=0;
  pumpTotal=0.0; wellTotal=0.0;
  pumpCCA=0.0; wellCCA=0.0;
}

void cistern::topSensorChanged(boolean newStateTop)
{
  volume(capacity-headspace);
  if(newStateTop==HIGH)
  {
    THROW_EXCEPTION(CISTERN_CRIT_HIGH);
    if(bottomCistern.read()==HIGH)
    { 
      wsException=SENSE_ERR_HI_LO; 
      wsExceptionHandler();
      return;
    }
    if(midCistern.read()==LOW)
    { 
      wsException=SENSE_ERR_HI_HALF; 
      wsExceptionHandler();
      return;
    }
  }
  else THROW_EXCEPTION(CISTERN_HIGH_SENSE_NORM);
}
void cistern::midSensorChanged(boolean newState)
{
  float beforeV=volume();
  Serial.print(F("Cistern vol before = ")); Serial.print(volume()); Serial.print(F("...after = "));
  if(WellCompressor.status()==LOW)
  {
    volume(29.0);
    if(calibrated)
    {
		TotalGalPressurePump(TotalGalPressurePump()+(beforeV-volume()));
		pumpCCA-=(beforeV-volume());
	}
    lcd.createChar(chCistern,chDcisternLH);
  }
  else
  {
    volume(CIST_VOL_AT_CENTER_FILLING);
    if(calibrated)
    {
		TotalGalCompressor(TotalGalCompressor()+(beforeV-volume()));
		wellCCA-=(beforeV-volume());
	}
    lcd.createChar(chCistern,chDcisternUH);
  }
  calibrated=true;
  THROW_EXCEPTION(CISTERN_HALF); 
}
void cistern::lowSensorChanged(boolean newState)
{
  volume(bottomspace);
  if(newState==HIGH){ THROW_EXCEPTION(CISTERN_CRIT_LOW);}
  else THROW_EXCEPTION(CISTERN_LOW_SENSE_NORM);
}

void cistern::update(boolean filling, boolean drawing, boolean full, boolean half, boolean empty)
{
  unsigned long thisMil=millis();
  unsigned long elapsed;
  int volumeBefore=(volume());
   elapsed=elapsedMillis(mil, thisMil);
  mil=thisMil;
  if (filling)
  { 
    if ((fillDelaySecs*1000) < elapsedMillis(compressorStartMil, thisMil))
   {
     float waterAdded=((float) elapsed * (fillGPM / (60.0 * 1000.0)));
      //level += (elapsed * (fillGPM / (60 * 1000)));
  //    Serial.print("elapsed   ");Serial.println(elapsed);
      volume(volume() + waterAdded );
      wellTotal+=waterAdded;
      LCDPF(5,1,wellTotal,0);
 //     if (volume() >= normFull) WellCompressor.off();
      if (volume() >= normFull) THROW_EXCEPTION(CISTERN_HIGH);
    }
  }
  else
  {
    compressorStartMil=thisMil;
  }
  if (drawing)
  {
#ifdef USINGHALLSENSOR
   if(pumpCounter!=0)
   {
      float waterRemoved=(drawGPStroke*pumpCounter);
      //Serial.println(pumpCounter,5);  
      volume(volume() - waterRemoved);
      pumpTotal+=waterRemoved;  
      LCDPF(0,1,pumpTotal,0);
      pumpCounter=0;
   } 
#else
  {
    float waterRemoved=((float) elapsed * (drawGPM / (60.0 * 1000.0)));
    volume(volume() - waterRemoved);
    pumpTotal+=waterRemoved;
    LCDPF(0,1,pumpTotal,0);
  }
#endif
//    Serial.print(HighAmpLoad.read());Serial.println(filling);
 //   if ((!filling) && (volume() <= normEmpty) &&(!HighAmpLoad.read())) WellCompressor.on();
 //   if (volume() <= bottomspace) WaterPressurePump.off();
  }  
  if((volume() > normEmpty)&&(volumeBefore <= normEmpty)&&(dsB->read()!=HIGH)&&(dsT->read()!=HIGH)) THROW_EXCEPTION(CISTERN_NORM);
  if(volume() <= normEmpty) THROW_EXCEPTION(CISTERN_LOW); 
  if(volume()<= bottomspace) THROW_EXCEPTION(CISTERN_CRIT_LOW);
#ifdef  LCDCONNECTED
  lcd.CRprint(10,2,ftoa(volume()));
  lcd.CRprint(10,3,ftoa(TankPSI.value()));
#else
  lcd.setCursor(4,2); lcd.print(volume(),1);
  lcd.setCursor(9,2); lcd.print(normEmpty,0);
  lcd.setCursor(14,2);lcd.print(normFull,0);
  lcd.setCursor(4,3); lcd.print(TankPSI.value(),1);
  lcd.setCursor(9,3); lcd.print(TankPSI.lowThresholdValue(),0);
  lcd.setCursor(14,3);lcd.print(TankPSI.highThresholdValue(),0);  
  setCisternChar((volume()-bottomspace)/(normFull-headspace));  
#endif
}

//*******************************************************************************************


serialIO SerialObj(9600,&commandHandler);
cistern cist;

void  updatePersTotals(float PPG, float WCG)
{
  byte  ppg_l, ppg_m, wcg_l, wcg_m, highB;
 unsigned long conv;
  conv=(unsigned long) (PPG+0.5);
  highB=(byte)(((conv >> 16) << 4) & 0xF0);
  ppg_m=highByte(conv);
  ppg_l=lowByte(conv);  
  conv=int(WCG+0.5);
  highB+=(byte)((conv >> 16) & 0x0F);
  wcg_m=highByte(conv);  
  wcg_l=lowByte(conv);
  EEPROM_UPDATE(EEPROM_TOTAL_GAL_PRESSURE_PUMP_L,ppg_l);
  EEPROM_UPDATE(EEPROM_TOTAL_GAL_PRESSURE_PUMP_M,ppg_m);
  EEPROM_UPDATE(EEPROM_TOTAL_GAL_COMPRESSOR_L,wcg_l);
  EEPROM_UPDATE(EEPROM_TOTAL_GAL_COMPRESSOR_M,wcg_m);
  EEPROM_UPDATE(EEPROM_TOTAL_GAL_COMPRESSOR_B0to3,highB);
}

void importPersTotals(void)
{
  int conv;
  conv=EEPROM.read(EEPROM_TOTAL_GAL_PRESSURE_PUMP_L);
  conv+=(EEPROM.read(EEPROM_TOTAL_GAL_PRESSURE_PUMP_M)*0xFF);
  conv+=((EEPROM.read(EEPROM_TOTAL_GAL_PRESSURE_PUMP_B4to7) >> 4) * 0xFFFF);
  cist.TotalGalPressurePump((float)conv);
  conv=EEPROM.read(EEPROM_TOTAL_GAL_COMPRESSOR_L);
  conv+=(EEPROM.read(EEPROM_TOTAL_GAL_COMPRESSOR_M)*0xFF);
  conv+=((EEPROM.read(EEPROM_TOTAL_GAL_COMPRESSOR_B0to3) & (byte) 0x0F)*0xFFFF);
  cist.TotalGalCompressor((float)conv);
}
void topTankSensorChanged(boolean newState){ cist.topSensorChanged(newState); }
void midTankSensorChanged(boolean newState){ cist.midSensorChanged(newState); }  //expermiting with callback function
void botTankSensorChanged(boolean newState){ cist.lowSensorChanged(newState); }
void HiAmpLoadChanged(boolean newState){ THROW_EXCEPTION((newState) ?  LOAD_HIGH : LOAD_NORM);}
void OnLineChanged(boolean newState){ WaterPressurePump.LockOut1(newState);  WellCompressor.LockOut1(newState);}
void PumpTestSwChanged(boolean newState){if(newState==LOW){if(WaterPressurePump.status()==LOW)pumpCounter=0; WaterPressurePump.on();}}

void wsExceptionHandler(void)
{
  switch (wsException)
  {
    case  PSI_OVER_MAX:
      if(WaterPressurePump.status()==HIGH)
      {
        WaterPressurePump.off();
        Serial.print(TankPSI.value());
        Serial.println(F(" PSI_OVER_MAX "));
        updatePersTotals(cist.TotalGalPressurePump(), cist.TotalGalCompressor());
      }
      break;
    case  CISTERN_CRIT_LOW:
      WaterPressurePump.LockOut2(true);
      wsAlert.sound(20);
      Serial.print(F(" CRITICAL "));
    case  CISTERN_LOW:      
      Serial.print(F("CISTERN LOW "));
      Serial.println(cist.volume());
      WellCompressor.on();
      break;
    case CISTERN_LOW_SENSE_NORM:
      WaterPressurePump.LockOut2(false);
      break;
    case  CISTERN_CRIT_HIGH:
      WellCompressor.LockOut2(true);
      wsAlert.sound(20);
      Serial.print(F(" CRITICAL "));
    case  CISTERN_HIGH:
      Serial.print(F("CISTERN_HIGH "));
      Serial.println(cist.volume());
      WellCompressor.off();
      updatePersTotals(cist.TotalGalPressurePump(), cist.TotalGalCompressor());
      break;
    case   CISTERN_HIGH_SENSE_NORM:
      WellCompressor.LockOut2(false);
      break;
    case CISTERN_NORM:
      WaterPressurePump.LockOut2(false);
      //Serial.println("CISTERN_NORM");
      break;
    case   CISTERN_HALF:
      Serial.print(cist.volume());
      Serial.println(F("  CISTERN_HALF"));
//      WaterPressurePump.LockOut2(false);
      break;
    case  SENSE_ERR_HI_LO:
      Serial.println(F("SENSE_ERR_HI_LO"));
      break;
    case  SENSE_ERR_HI_HALF:
      Serial.println(F("SENSE_ERR_HI_HALF"));
      break;
    case  SENSE_ERR_LO_HALF:
      Serial.println(F("SENSE_ERR_LO_HALF"));
      break;
    case  PSI_UNDER_MIN:
 //     Serial.print("cist.status cist.volume  ");Serial.print(cist.status());Serial.println(cist.volume());
      if(WaterPressurePump.status()==LOW)
      {
        Serial.print(cist.status()); 
        Serial.print(topCistern.read());
        Serial.print(midCistern.read());
        Serial.println(bottomCistern.read());
        if((cist.status()<0) && (bottomCistern.read()!=1) && (midCistern.read()!=0)) cist.volume(20.0);
        if((cist.status()<0) && (bottomCistern.read()!=1) && (topCistern.read()!=0)) cist.volume(20.0);        
        if (bottomCistern.read()!=1) WaterPressurePump.LockOut2(false);
        if((cist.status()>=0) && ((topCistern.read()!=0) || (midCistern.read()!=0) || bottomCistern.read()!=1))
        {
          pumpCounter=0;
          WaterPressurePump.on();
          Serial.print(TankPSI.value());
          Serial.println(F(" PSI_UNDER_MIN "));
        }
      }
      break;
    case LOAD_HIGH:
      WellCompressor.LockOut3(true);
      wsAlert.sound(25);
      Serial.println(F("LOAD_HIGH WellCompressor.off();"));
      break;
    case LOAD_NORM:
      WellCompressor.LockOut3(false);
      if(cist.status()<0)WellCompressor.on();
      Serial.println(F("LOAD_NORM -- if(cist.status()<0)WellCompressor.on();"));
      break;
    case NO_EXCEPTION:
      Serial.println(F("NO_EXCEPTION"));
      break;
    default:
      Serial.println(F("default"));
      break;
  }
}

#ifndef	LCDCONNECTED
void LCDinit(void)
{
  lcd.init(); 
  lcd.backlight();
//  lcd.setCursor(4, 1);
//  lcd.print("Please Wait");
  delay(1000);
  lcd.createChar(chCistern,chDcisternH);
  lcd.createChar(chDrain,chDdrainArrow);
  lcd.createChar(chFill,chDfillArrow);
}
#endif

void poll(void)
 { 
  topCistern.poll();
  midCistern.poll();
  bottomCistern.poll();  
  TankPSI.poll(); //Serial.println(TankPSI.value());
  OnLine.poll(); 
  HighAmpLoad.poll();
  PumpTest.poll();
}

void setup(void)
{
int i;	
  wdt_disable();  //turn off watchdog timer in case setup takes too long

  Serial.begin(9600);
  while (!Serial) {
    // wait for serial port to connect. Needed for Leonardo only
  }
  

#ifndef	LCDCONNECTED
  LCDinit();
#endif

delay(2000);
  Serial.println(F("START"));
  TankPSI.poll();
  topCistern.onChange=&topTankSensorChanged;  
  midCistern.onChange=&midTankSensorChanged; //expermiting with callback function
  bottomCistern.onChange=&botTankSensorChanged;
  HighAmpLoad.onChange=&HiAmpLoadChanged;
  OnLine.onChange=&OnLineChanged;
  PumpTest.onChange=&PumpTestSwChanged;
  
  cist.dsT = &topCistern;
  cist.dsM = &midCistern;
  cist.dsB = &bottomCistern;
    pinMode(3,INPUT_PULLUP);
    attachInterrupt(1,PumpCounterISR,FALLING);
#ifdef  LCDCONNECTED
    lcd.CRprint(0,2,"Volume ");
    lcd.CRprint(0,3,"Pressure ");
#else    
    lcd.setCursor(0,2);lcd.print("Vol        ");
    lcd.setCursor(0,3);lcd.print("PSI        ");
    lcd.setCursor(18,1);lcd.write(byte(chCistern));
//    lcd.setCursor(19,2);lcd.write(byte(chDrain));
//    lcd.setCursor(19,0);lcd.write(byte(chFill));
#endif
    WellCompressor.chInit(chFill,chDnothing,chDfillArrow,19,0);
    WaterPressurePump.chInit(chDrain,chDnothing,chDdrainArrow,19,2);
    for(i=0;i<10;i++){ poll();};
    if(bottomCistern.read()==HIGH){ WellCompressor.on(); cist.volume(10.0);}// volue const need to be calculated from cist values
    if(midCistern.read()==HIGH) cist.volume(40.0); else cist.volume(20.0);// volue const need to be calculated from cist values
    if(topCistern.read()==HIGH) cist.volume(60.0);
    if(OnLine.read()==HIGH){OnLineChanged(true);Serial.print(F("OnLine.read()==HIGH. Calling OnLineChanged(true)"));};
//    Serial.print("EPROM ");
    cist.fillGPMval(((float)(EEPROM.read(EEPROM_WGPM)))/40.0+7.25);
#ifdef USINGHALLSENSOR
    cist.drawGPSval((((float)(EEPROM.read(EEPROM_PGPS)))+120.0)/10000);
#endif
 //   EEPROM.write(0,(byte)16);
    importPersTotals();
    wdt_enable(WDTO_8S); //enable watchdog timer  
}

void UpdateLCD(void)
{
  LCDP(0,0,topCistern.read());  
  LCDP(2,0,midCistern.read());
  LCDP(4,0,bottomCistern.read());
  LCDP(6,0,OnLine.read());
  LCDP(8,0,HighAmpLoad.read());
  LCDP(10,0,PumpTest.read());
}

void loop(void)
{
  wsException=NO_EXCEPTION;
  poll();/*
  topCistern.poll();
  midCistern.poll();
  bottomCistern.poll();  
  TankPSI.poll(); //Serial.println(TankPSI.value());
  OnLine.poll(); 
  HighAmpLoad.poll();
  PumpTest.poll();*/
  cist.update(WellCompressor.status(), WaterPressurePump.status(), topCistern.read(), midCistern.read(), bottomCistern.read());
  //Serial.println(cist.volume());
  myDelay(LoopDelay);
  SerialObj.poll();
  UpdateLCD();
  wdt_reset();// restart time on watchdog timer
}

#define CMDPICK(CMD,ACTION)  if(strcmp(cmd,CMD) ==0){ACTION;return;}
#define CMDPICKT(CMD,TEST,ACTION)  if((strcmp(cmd,CMD) ==0)&&(TEST)){ACTION;return;}

void commandHandler(char *cmd, char*val)  //process commands recieved from the serial port
{
   Serial.print(cmd);
   Serial.print("="); 
   Serial.println(val);
   if(strcmp(cmd,"GETSET") ==0)
  {
#ifndef	USINGHALLSENSOR	  
    Serial.print(F("PumpGPM = ")); Serial.println(cist.drawGPMval());
#else    
    Serial.print(F("PumpGPStroke = ")); Serial.println(cist.drawGPSval(),5);
#endif    
    Serial.print(F("WellGPM = ")); Serial.println(cist.fillGPMval());
    Serial.print(F("PSI Min = ")); Serial.println(TankPSI.lowThresholdValue());
    Serial.print(F("PSI Max = ")); Serial.println(TankPSI.highThresholdValue());
    Serial.print(F("Cistern level = ")); Serial.println(cist.volume(),1);
    Serial.print(F("Cistern status and sensors "));
    Serial.print(cist.status()); 
    Serial.print(topCistern.read());
    Serial.print(midCistern.read());
    Serial.println(bottomCistern.read());    
    Serial.print(F("Tank PSI = ")); Serial.println(TankPSI.value(),1);
    Serial.print(F("Well Total = "));Serial.println(cist.TotalGalCompressor(),0);
    Serial.print(F("Pump Total = "));Serial.println(cist.TotalGalPressurePump(),0);
    Serial.print(F("Well Center Correction Adjustment = "));Serial.println(cist.WCCA(),1);
    Serial.print(F("Pump Center Correction Adjustment = "));Serial.println(cist.PCCA(),1);
  }
   
#ifndef	USINGHALLSENSOR	
  CMDPICK("GPMP",cist.drawGPMval(atof(val)));
#else  
  CMDPICK("GPSP",cist.drawGPSval(atof(val));EEPROM_UPDATE(EEPROM_PGPS,((atof(val)*10000.0-120.0))));
//  if(strcmp(cmd,"GPSP") ==0){
#endif  
  CMDPICK("CVOL",cist.volume(atof(val)));
  CMDPICK("LCDINIT",LCDinit);
  CMDPICK("LD",LoopDelay=atoi(val));
  if(strcmp(cmd,"GPMC") ==0){cist.fillGPMval(atof(val));EEPROM_UPDATE(EEPROM_WGPM,((atof(val)-7.25)*40.0));}
  CMDPICKT("PSIMN",TankPSI.highThresholdValue()>atof(val),TankPSI.lowThresholdValue(atof(val)));
  if((strcmp(cmd,"PSIMX") ==0)&&(TankPSI.lowThresholdValue()<atof(val)))TankPSI.highThresholdValue(atof(val));
  if(strcmp(cmd,"NITE") ==0)
  {
    TankPSI.range(20.0,22.0);
    if(cist.volume()<40.0)WellCompressor.on();
    return;
  }
  CMDPICK("BATH",TankPSI.range(45.0,50.0));
  CMDPICK("DAY",TankPSI.range(PUMP_ON_PSI,PUMP_OFF_PSI));
  if(strcmp(cmd,"PUMPSTART") ==0)
  {
    WaterPressurePump.LockOut1(false);
    if(WaterPressurePump.status()==LOW)pumpCounter=0;
    WaterPressurePump.on();
    return;
  }
  if(strcmp(cmd,"PUMPSTOP") ==0)   WaterPressurePump.off();
  CMDPICK("PUMPLCK",WaterPressurePump.LockOut1(true);)
  CMDPICK("PUMPULCK",WaterPressurePump.LockOut1(false);)
  CMDPICK("COMPLCK",WellCompressor.LockOut1(true));
  CMDPICK("COMPULCK",WellCompressor.LockOut1(false));
  CMDPICK("COMPSTART",WellCompressor.on());
  CMDPICK("COMPSTOP",WellCompressor.off());
  CMDPICK("ALARM",wsAlert.sound((unsigned long)atol(val)));
  
  
  if(strcmp(cmd,"HELP")==0)
  {
    Serial.println(F("<CMD=value> or just <CCMD>"));  
#ifndef	USINGHALLSENSOR	 	  
    Serial.println(F("GPMP= Gallons per minute Pump"));
#else
    Serial.println(F("GPSP= Gallons per stroke Pump"));
#endif   
    Serial.println(F("CVOL= Set Cistern volume level"));
    Serial.println(F("GPMC= Gallons per minute from Well Compressor"));
    Serial.println(F("PSIMN= Pressure Pump on pressure"));
    Serial.println(F("PSIMX= Pressure Pump Off Pressure"));
    Serial.println(F("PUMPSTART Manually start pressure pump"));
    Serial.println(F("PUMPSTOP Manually stop pressure pump"));
    Serial.println(F("PUMPLCK Lock pressur pump off"));
    Serial.println(F("PUMPULCK Unlock pressure pump"));
    Serial.println(F("COMPSTART Manually start compressor"));
    Serial.println(F("COMPSTOP Manually stop the compressor"));
    Serial.println(F("COMPLCK Lock compressor off"));
    Serial.println(F("COMPULCK Unlock compressor"));
    Serial.println(F("LCDINIT Reset LCD -- ???"));
    Serial.println(F("LD Set loop delay ms"));
    Serial.println(F("NITE  DAY  BATH"));    
  }
}

void myDelay(unsigned long milSec)
{
	unsigned long st=millis();
	while(elapsedMillis(st,millis())<milSec){idleLoop(); wdt_reset();};          //TEST TEST TEST!!!!!!!!!!!!!!
}
void idleLoop(void)
{
	wsAlert.poll();
}
