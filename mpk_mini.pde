////////////////////////////////////////////////////////////////////////////////
// MPK-MINI USB-MIDI to SERIAL MIDI Converter
// adapted to mpk-mini
// added support for: MIDI IN (via interrupt) and MIDI program change messages
// 
// by John Pollard - jp3.co
// john (at) jp3 [dot] co
// January 2012
//
// 
// THIS IS AN ADAPTATION OF:
// LPK25 USB-MIDI to Serial MIDI converter
// for use with USB Host Shield from Circuitsathome.com
// credit for original code by Collin Cunningham - makezine.com, narbotic.com
// src: https://cdn.makezine.com/make/2010/12/LPK25_Host_pde-101203a.zip/
// article: https://makezine.com/article/technology/arduino/usbhacking/
// 
/////////////////////////////////////////////////////////////////////////////////

// Note from John:
//
// 1. You need the LEGACY usb host shield libraries 
//    found here: https://github.com/felis/USB_Host_Shield/tree/dev
//
// 2. You must use an old version of Arduino (0.23)
//    here: http://code.google.com/p/arduino/downloads/list?q=0023
//
// 3. MIDI OUT pinout is as follows
//    Arduino digital pin 1 (TX) connected to MIDI jack pin 5
//    MIDI jack pin 2 connected to ground
//    MIDI jack pin 4 connected to +5V through 220-ohm resistor

//SPI NO LONGER NEEDED
//#include <Spi.h>  // NOTE: alternate Spi Library from Circuitsathome.com
#include <Max3421e.h>
#include <Usb.h>
#include <avr/pgmspace.h>

//	For general USB-MIDI compliance, we need to determine which endpoint has a direction of IN
//	and likely also need to get the poll duration(EP_POLL), not sure if necessary though
//	

/*   USB ERROR CODES
 hrSUCCESS   0x00
 hrBUSY      0x01
 hrBADREQ    0x02
 hrUNDEF     0x03
 hrNAK       0x04
 hrSTALL     0x05
 hrTOGERR    0x06
 hrWRONGPID  0x07
 hrBADBC     0x08
 hrPIDERR    0x09
 hrPKTERR    0x0A
 hrCRCERR    0x0B
 hrKERR      0x0C
 hrJERR      0x0D
 hrTIMEOUT   0x0E
 hrBABBLE    0x0F
 */
 
// MIDI standard
 
//////////////////////////
// MIDI MESAGES 
// midi.org/techspecs/
//////////////////////////
// STATUS BYTES
// 0x80 == noteOn
// 0x90 == noteOff
// 0xA0 == afterTouch
// 0xB0 == controlChange
// 0xC0 - 0xCF == programChange
//////////////////////////
//DATA BYTE 1
// note# == (0-127)
// or
// control# == (0-119)
// or
// program# (0-127)
//////////////////////////
// DATA BYTE 2
// velocity == (0-127)
// or
// controlVal == (0-127)
// 
// DO NOT TRANSMIT FOR programChange
//////////////////////////


//DEBUGGING
#define DEBUG 0             //Change to 1 to enable Arduino IDE debugging
#define DEBUG_PORT 28800    // port to debug w/ Ardunio IDE

// MIDI IN SUPPORT

//Change this variable to 1 to enable midi input.
#define MIDI_IN 1
int midiInPin = 2;


/* LPK25 data taken from descriptors */
// descriptors retrieved using descriptor_parser.pde (Usb library example sketch)
#define LPK25_ADDR            1
#define LPK25_VID_LO          0xFFFFFFE8  //VENDOR ID AKai = 0xFFFFFFE8    KORG = 0x44
#define LPK25_VID_HI          0x09   //VENDOR ID Part2: 0x90 for both?
#define LPK25_PID_LO          0x7C  //Device ID : Akai MKP-Mini 0x7C  LPK: 0x76  KorgNano: 0x0D
#define LPK25_PID_HI          0x00   //0x01 for korg?
#define LPK25_CONFIG          1
#define LPK25_IF              1
#define LPK25_NUM_EP          2
#define EP_MAXPKTSIZE         64
#define EP_BULK               0x02 
#define EP_POLL               0x00        // 0x0B for Korg nanoKey
#define CONTROL_EP            0
#define OUTPUT_EP             1
#define INPUT_EP              1
#define LPK25_01_REPORT_LEN   0x09
#define LPK25_DESCR_LEN       0x0C
//Check if MIDI Data is in Program Change Range
#define PROGRAM_CHANGE_MIN    0xC0
#define PROGRAM_CHANGE_MAX    0xCF

EP_RECORD ep_record[ LPK25_NUM_EP ];  //endpoint record structure for the LPK25 controller

char descrBuf[ 12 ] = { 0 };	//buffer for device description data
char buf[ 64 ] = { 0 };		//buffer for USB-MIDI data   NOTE: I had to increase this from Collin's buffer size. -John
char oldBuf[ 4 ] = { 0 };		//buffer for old USB-MIDI data
byte outBuf[ 3 ] = { 0 };		//buffer for outgoing MIDI data


byte midiByte;

MAX3421E Max;
USB Usb;

void setup() {
  
  // Using interrupt 0 on digital pin 2.  // This is the MIDI-IN pin.
  if(MIDI_IN == 1){
    pinMode(midiInPin, INPUT);
    digitalWrite(midiInPin, LOW);
  }
 
  
  if (DEBUG==1) {
    Serial.begin( DEBUG_PORT );
  } else {
    Serial.begin( 31250 );  //MIDI baud rate, no worky with Arduino IDE
  }
  Serial.flush();
  if(MIDI_IN == 1){attachInterrupt(0, serialInterrupt, CHANGE);}
  Max.powerOn();
  //delay(200);
}

void loop() {

  Max.Task();
  Usb.Task(); 
  //Serial.println(Usb.getUsbTaskState(),HEX);

// MIDI THRU CODE



//END MIDI THRU CODE


  if( Usb.getUsbTaskState() == USB_STATE_CONFIGURING ) {  //wait for addressing state
    if (DEBUG==1) {  Serial.println("Initializing"); }
    LPK25_init();
    Usb.setUsbTaskState( USB_STATE_RUNNING );
  }
  if( Usb.getUsbTaskState() == USB_STATE_RUNNING ) {  //poll the LPK25 Controller 
   if (DEBUG==1) { Serial.println("loop");}
    LPK25_poll();
  }
}

/* Initialize LPK25 Controller */
void LPK25_init( void ){

  byte rcode = 0;  //return code
  byte i;

  /* Initialize data structures for endpoints of device 1*/
  ep_record[ CONTROL_EP ] = *( Usb.getDevTableEntry( 0,0 ));  //copy endpoint 0 parameters
  ep_record[ OUTPUT_EP ].epAddr = 0x01;    // Output endpoint, 0x02 for Korg nanoKey
  ep_record[ OUTPUT_EP ].Attr  = EP_BULK;
  ep_record[ OUTPUT_EP ].MaxPktSize = EP_MAXPKTSIZE;
  ep_record[ OUTPUT_EP ].Interval  = EP_POLL;
  ep_record[ OUTPUT_EP ].sndToggle = bmSNDTOG0;
  ep_record[ OUTPUT_EP ].rcvToggle = bmRCVTOG0;
  ep_record[ INPUT_EP	].epAddr = 0x01;			// Input endpoint, 0x02 for Korg nanoKey 
  ep_record[ INPUT_EP ].Attr  = EP_BULK;
  ep_record[ INPUT_EP ].MaxPktSize = EP_MAXPKTSIZE;
  ep_record[ INPUT_EP ].Interval  = EP_POLL;
  ep_record[ INPUT_EP ].sndToggle = bmSNDTOG0;
  ep_record[ INPUT_EP ].rcvToggle = bmRCVTOG0;

  Usb.setDevTableEntry( LPK25_ADDR, ep_record );             //plug kbd.endpoint parameters to devtable

  /* read the device descriptor and check VID and PID*/
  rcode = Usb.getDevDescr( LPK25_ADDR, ep_record[ CONTROL_EP ].epAddr, LPK25_DESCR_LEN, descrBuf );
  if( rcode ) {
    if (DEBUG==1) {
      Serial.print("Error attempting read device descriptor. Return code :");
      Serial.println( rcode, HEX );
    }
    while(1);  //stop
  }
//test descrBuf
  if (DEBUG==1) {
    Serial.print("Description Buffer data:  ");
    Serial.print(descrBuf[0], HEX);
    Serial.print(" - ");
    Serial.print(descrBuf[1], HEX);
    Serial.print(" - ");
    Serial.print(descrBuf[2], HEX);
    Serial.print(" - ");
    Serial.print(descrBuf[3], HEX);
    Serial.print(" - ");
    Serial.print(descrBuf[4], HEX);
    Serial.print(" - ");
    Serial.print(descrBuf[5], HEX);
    Serial.print(" - ");
    Serial.print(descrBuf[6], HEX);
    Serial.print(" - ");
    Serial.print(descrBuf[7], HEX);
    Serial.print(" - ");
    Serial.print(descrBuf[8], HEX);
    Serial.print(" - ");
    Serial.print(descrBuf[9], HEX);
    Serial.print(" - ");
    Serial.print(descrBuf[10], HEX);
    Serial.print(" - ");
    Serial.print(descrBuf[11], HEX);
    Serial.println("");
    Serial.println("");
  }
  //
  // Test device for Akai LPK25
  if((descrBuf[ 8 ] != LPK25_VID_LO) || (descrBuf[ 9 ] != LPK25_VID_HI) || (descrBuf[ 10 ] != LPK25_PID_LO) || (descrBuf[ 11 ] != LPK25_PID_HI) ){
    if (DEBUG==1) {
      Serial.print("Unsupported USB Device");
      Serial.println( rcode, HEX );
    }
    while(1);  //stop
  }


  // Test device for Korg nanoKey
  // if((descrBuf[ 8 ] != 0x44) || (descrBuf[ 9 ] != 0x09) || (descrBuf[ 10 ] != 0x0D) || (descrBuf[ 11 ] != 0x01) ){
 //  Serial.print("Unsupported USB Device");
 //  Serial.println( rcode, HEX );
 //  while(1);  //stop   
//   }



  /* Configure device */
  rcode = Usb.setConf( LPK25_ADDR, ep_record[ CONTROL_EP ].epAddr, LPK25_CONFIG );                    
  if( rcode ) {
    if (DEBUG==1) {
      Serial.print("Error attempting to configure LPK25. Return code :");
      Serial.println( rcode, HEX );
    }
    while(1);  //stop
  }
 if(DEBUG == 1) {
   Serial.println("Device initialized");
    Serial.println("");
    Serial.println("");
    Serial.println("");
   }
  delay(200);
}

// Poll LPK25 and print result
void LPK25_poll( void ){

 byte rcode = 0;     //return code
 /* poll LPK25 */
 rcode = Usb.inTransfer( LPK25_ADDR, ep_record[ INPUT_EP ].epAddr, LPK25_01_REPORT_LEN, buf );
 if( rcode != 0 ) {
    if (DEBUG==1) { Serial.print("Usb.inTransferReturn (4 is normal): "); Serial.println( rcode, HEX );}
    return;
 }
 process_report();
 return;
}


void process_report(void){
  byte i, codeIndexNumber;
  for( i = 0; i < 4; i++) {  //check for new information
    if( buf[ i ] != oldBuf[ i ] ) { //new info in buffer
      break;
    }
  }
  if( i == 4 ) {
    if(DEBUG == 1){  Serial.println("all bytes the same");}
    return;  //all bytes are the same
  }

 else{
    outBuf[0] = byte(buf[1]);
    outBuf[1] = byte(buf[2]);
    outBuf[2] = byte(buf[3]);
   
   if (DEBUG==1) {
    Serial.println("");
    Serial.print(outBuf[0],BIN);
    Serial.print(" - "); 
    Serial.print(outBuf[1],BIN); 
    Serial.print(" - ");
   }
  
  
  
  //MIDI Output
  if (byte(buf[1]) >= PROGRAM_CHANGE_MIN && byte(buf[1]) <= PROGRAM_CHANGE_MAX){
    outBuf[2] = byte(0);
    
    if (DEBUG==1) {
      Serial.println("Program Change");
    } else {
      // Actual MIDI output IF its a program change
      Serial.write(outBuf,2);
    }
  } else {  // Program change Else
    if (DEBUG==1) {
      Serial.print(outBuf[2],BIN); 
      Serial.println("");
     } else {
       // Actual MIDI output if not program change
      Serial.write(outBuf, 3);
     }
  }// End program change Else

// save by copying to oldBuf
  for( i = 0; i < 4; i++) {  //check for new information
    oldBuf[ i ] = buf[ i ];
    }
  }	
  return;
}

//INTerrupt  

volatile boolean inService = false;



void serialInterrupt()
{
  // Trick: since Serial I/O in interrupt driven, we must reenable interrupts while in this Interrupt Service Routine.
  // But doing so will cause the routine to be called while nested, causing problems.
  // So we mark that we are already in service.

  // Already in service? Do nothing.
  if (inService) return;

  // You were not in service. Now you are.
  inService = true;
  
  // Reenable interrupts, to allow Serial to work. We do this only if inService is false.
  interrupts();
  
  // Allow serial to read at least one byte.
  while(!Serial.available());

  // read the incoming byte and echo it out of the midi out.
    midiByte = Serial.read();
    outBuf[0]=midiByte;
    Serial.write(outBuf, 1);
    
  // Job done. You are no longer in service.
  inService = false;
}
