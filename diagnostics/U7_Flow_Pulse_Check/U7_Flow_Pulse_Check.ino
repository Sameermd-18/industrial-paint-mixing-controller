/* U7 standalone flow pulse diagnostic. Replaces normal U7 firmware while testing.
   FS1 GPIO33 blue/flush, FS2 GPIO25 yellow, FS3 GPIO26 green.
   Serial Monitor 115200 baud. Send R to reset counts.
   Filtered counts discard rising edges within 1000 us of an accepted edge.
   This filter is diagnostic, not a calibrated production setting. */
#include <Arduino.h>
#include <ctype.h>
constexpr uint8_t FS1=33, FS2=25, FS3=26;
constexpr uint32_t MIN_GAP_US=1000;
volatile uint32_t raw[3]={0,0,0}, filtered[3]={0,0,0}, last[3]={0,0,0};
void IRAM_ATTR count0(){uint32_t t=micros(); ++raw[0]; if(t-last[0]>=MIN_GAP_US){++filtered[0];last[0]=t;}}
void IRAM_ATTR count1(){uint32_t t=micros(); ++raw[1]; if(t-last[1]>=MIN_GAP_US){++filtered[1];last[1]=t;}}
void IRAM_ATTR count2(){uint32_t t=micros(); ++raw[2]; if(t-last[2]>=MIN_GAP_US){++filtered[2];last[2]=t;}}
void resetCounts(){
  noInterrupts();
  uint32_t t=micros()-MIN_GAP_US;
  for(int i=0;i<3;i++){raw[i]=filtered[i]=0;last[i]=t;}
  interrupts();
}
void setup(){
  Serial.begin(115200);
  pinMode(FS1,INPUT);pinMode(FS2,INPUT);pinMode(FS3,INPUT);
  resetCounts();
  attachInterrupt(digitalPinToInterrupt(FS1),count0,RISING);
  attachInterrupt(digitalPinToInterrupt(FS2),count1,RISING);
  attachInterrupt(digitalPinToInterrupt(FS3),count2,RISING);
  Serial.println("U7 flow diagnostics: filtered/raw, FS1 blue/flush, FS2 yellow, FS3 green");
}
void loop(){
  while(Serial.available()) if(toupper((unsigned char)Serial.read())=='R') {resetCounts();Serial.println("Reset");}
  static uint32_t lastPrint=0;
  if(millis()-lastPrint>=1000){
    lastPrint=millis();
    uint32_t r[3],f[3];
    noInterrupts();
    for(int i=0;i<3;i++){r[i]=raw[i];f[i]=filtered[i];}
    interrupts();
    Serial.printf("FS1 blue/flush %lu/%lu | FS2 yellow %lu/%lu | FS3 green %lu/%lu\n",
      (unsigned long)f[0],(unsigned long)r[0],
      (unsigned long)f[1],(unsigned long)r[1],
      (unsigned long)f[2],(unsigned long)r[2]);
  }
}
