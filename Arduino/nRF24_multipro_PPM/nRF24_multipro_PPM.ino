/*
 ******************************************************************************
 This is a fork of the Multi-Protocol nRF24L01 Tx project
 from goebish on RCgroups / github. 
 This serial communication of this program is from perrytsao on github.com
 
 This version accepts either PPM or serial port strings and converts
 them to ppm commands which are then transmitted via
 the nRF24L01.
 The purpose of this code is to enable control over the Eachine E010 
 drone via code running on a PC or a FS-iA6B reciever connected via PPM. 

 This code reads channel values over serial from a Python program. 
 The first 4 channels are for control of the drone. The 4th channel selects 
 if the channel values from the python program should be sent to the drone 
 or if the channel values from the PPM of the attached receiver should be sent

 This code was tested on the Arduino Uno and nRF24L01 module.

 I believe this code will remain compatible with goebish's
 nRF24L01 Multi-Protocol board.  A way to
 connect to the serial port will be needed (such as the FTDI).

 *********************************************************************************

 ##########################################
 #####   MultiProtocol nRF24L01 Tx   ######
 ##########################################
 #        by goebish on rcgroups          #
 #                                        #
 #   Parts of this project are derived    #
 #     from existing work, thanks to:     #
 #                                        #
 #   - PhracturedBlue for DeviationTX     #
 #   - victzh for XN297 emulation layer   #
 #   - Hasi for Arduino PPM decoder       #
 #   - hexfet, midelic, closedsink ...    #
 ##########################################


 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License.
 If not, see <http://www.gnu.org/licenses/>.
 */

#include <util/atomic.h>
#include <EEPROM.h>
#include "iface_nrf24l01.h"
#include <string.h>
#include <PPMReader.h>

int interruptPin = 2;
int channelAmount = 4;
PPMReader ppm(interruptPin, channelAmount);

int control = 1000;

unsigned long lastString = 0;
unsigned long currentTime = 0;
unsigned long ElapsedTime = 0;

// ############ Wiring ################
//SPI Comm.pins with nRF24L01
#define MOSI_pin  3  // MOSI - D3
#define SCK_pin   4  // SCK  - D4
#define CE_pin    5  // CE   - D5
#define MISO_pin  A0 // MISO - A0
#define CS_pin    A1 // CS   - A1

#define ledPin    13 // LED  - D13

// SPI outputs
#define MOSI_on PORTD |= _BV(3)  // PD3
#define MOSI_off PORTD &= ~_BV(3)// PD3
#define SCK_on PORTD |= _BV(4)   // PD4
#define SCK_off PORTD &= ~_BV(4) // PD4
#define CE_on PORTD |= _BV(5)    // PD5
#define CE_off PORTD &= ~_BV(5)  // PD5
#define CS_on PORTC |= _BV(1)    // PC1
#define CS_off PORTC &= ~_BV(1)  // PC1
// SPI input
#define  MISO_on (PINC & _BV(0)) // PC0

#define RF_POWER TX_POWER_80mW

// tune ppm input for "special" transmitters
// #define SPEKTRUM // TAER, 1100-1900, AIL & RUD reversed

// PPM stream settings
#define CHANNELS 5 // number of channels in ppm stream, 12 ideally
enum chan_order{
    THROTTLE,
    AILERON,
    ELEVATOR,
    RUDDER,
    AUX1,  // (CH5)  led light, or 3 pos. rate on CX-10, H7, or inverted flight on H101
    AUX2,  // (CH6)  flip control
    AUX3,  // (CH7)  still camera (snapshot)
    AUX4,  // (CH8)  video camera
    AUX5,  // (CH9)  headless
    AUX6,  // (CH10) calibrate Y (V2x2), pitch trim (H7), RTH (Bayang, H20), 360deg flip mode (H8-3D, H22)
    AUX7,  // (CH11) calibrate X (V2x2), roll trim (H7)
    AUX8,  // (CH12) Reset / Rebind
};

#define PPM_MIN 1000
#define PPM_SAFE_THROTTLE 1050
#define PPM_MID 1500
#define PPM_MAX 2000
#define PPM_MIN_COMMAND 1300
#define PPM_MAX_COMMAND 1700
#define GET_FLAG(ch, mask) (ppm2[ch] > PPM_MAX_COMMAND ? mask : 0)
#define GET_FLAG_INV(ch, mask) (ppm2[ch] < PPM_MIN_COMMAND ? mask : 0)

// supported protocols
enum {
    PROTO_V2X2 = 0,     // WLToys V2x2, JXD JD38x, JD39x, JJRC H6C, Yizhan Tarantula X6 ...
    PROTO_CG023,        // EAchine CG023, CG032, 3D X4
    PROTO_CX10_BLUE,    // Cheerson CX-10 blue board, newer red board, CX-10A, CX-10C, Floureon FX-10, CX-Stars (todo: add DM007 variant)
    PROTO_CX10_GREEN,   // Cheerson CX-10 green board
    PROTO_H7,           // EAchine H7, MoonTop M99xx
    PROTO_BAYANG,       // EAchine H8(C) mini, H10, BayangToys X6, X7, X9, JJRC JJ850, Floureon H101
    PROTO_SYMAX5C1,     // Syma X5C-1 (not older X5C), X11, X11C, X12
    PROTO_YD829,        // YD-829, YD-829C, YD-822 ...
    PROTO_H8_3D,        // EAchine H8 mini 3D, JJRC H20, H22
    PROTO_MJX,          // MJX X600 (can be changed to Weilihua WLH08, X800 or H26D)
    PROTO_SYMAXOLD,     // Syma X5C, X2
    PROTO_HISKY,        // HiSky RXs, HFP80, HCP80/100, FBL70/80/90/100, FF120, HMX120, WLToys v933/944/955 ...
    PROTO_KN,           // KN (WLToys variant) V930/931/939/966/977/988
    PROTO_YD717,        // Cheerson CX-10 red (older version)/CX11/CX205/CX30, JXD389/390/391/393, SH6057/6043/6044/6046/6047, FY326Q7, WLToys v252 Pro/v343, XinXun X28/X30/X33/X39/X40
    PROTO_FQ777124,     // FQ777-124 pocket drone
    PROTO_E010,         // EAchine E010, NiHui NH-010, JJRC H36 mini
    PROTO_END
};

// EEPROM locationss
enum{
    ee_PROTOCOL_ID = 0,
    ee_TXID0,
    ee_TXID1,
    ee_TXID2,
    ee_TXID3
};

uint16_t overrun_cnt=0;
uint8_t transmitterID[4];
uint8_t current_protocol;
static volatile bool ppm_ok = false;
uint8_t packet[32];
static bool reset=true;
volatile uint16_t Servo_data[12];
static uint16_t ppm2[12] = {PPM_MIN,PPM_MID,PPM_MID,PPM_MID,PPM_MIN,PPM_MIN,
                           PPM_MIN,PPM_MID,PPM_MID,PPM_MID,PPM_MID,PPM_MID,};
int throttle = 1000;
int aileron, elevator, rudder = 1500;

String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete
char *p, *i;
char* c = new char[200 + 1]; // match 200 characters reserved for inputString later
char* errpt;
uint8_t ppm_cnt;

void setup()
{
    
    randomSeed((analogRead(A4) & 0x1F) | (analogRead(A5) << 5));
    pinMode(ledPin, OUTPUT);
    digitalWrite(ledPin, LOW); //start LED off
    pinMode(MOSI_pin, OUTPUT);
    pinMode(SCK_pin, OUTPUT);
    pinMode(CS_pin, OUTPUT);
    pinMode(CE_pin, OUTPUT);
    pinMode(MISO_pin, INPUT);

    // PPM ISR setup
    // attachInterrupt(digitalPinToInterrupt(PPM_pin), ISR_ppm, CHANGE);
    TCCR1A = 0;  //reset timer1
    TCCR1B = 0;
    TCCR1B |= (1 << CS11);  //set timer1 to increment every 1 us @ 8MHz, 0.5 us @16MHz

    set_txid(false);

    // Serial port input/output setup
    Serial.begin(115200);
    // reserve 200 bytes for the inputString:
    inputString.reserve(200);
}

void loop()
{
    uint32_t timeout=0;
    // reset / rebind
    //Serial.println("begin loop");
    if(reset || ppm2[AUX8] > PPM_MAX_COMMAND) {
        reset = false;
        Serial.println("selecting protocol");
        selectProtocol();
        Serial.println("selected protocol.");
        NRF24L01_Reset();
        Serial.println("nrf24l01 reset.");
        NRF24L01_Initialize();
        Serial.println("nrf24l01 init.");
        init_protocol();
        Serial.println("init protocol complete.");
    }
    // process protocol
    //Serial.println("processing protocol.");
    switch(current_protocol) {
        case PROTO_E010:
            timeout = process_MJX();
            break;     
    }
    
    overrun_cnt=0;

    currentTime = millis();
    
    if (stringComplete) {
        //Serial.println(inputString);
        // process string

       lastString = millis();
        
       strcpy(c, inputString.c_str());
       p = strtok_r(c,",",&i); // returns substring up to first "," delimiter
       ppm_cnt=0;
       while (p !=0){
         int val=strtol(p, &errpt, 10);
         if (!*errpt) {
           if (ppm_cnt == 4)
           {
              control = val;
           }  
           else
           {
              ppm2[ppm_cnt]=val;
           }
           
           }
           p = strtok_r(NULL,",",&i);
           ppm_cnt+=1;
         }

 

         // clear the string:
         inputString = "";
         stringComplete = false;
     }

     // Read the string from the serial buffer
     while (Serial.available()) {
       // get the new byte:
       char inChar = (char)Serial.read();
       // if the incoming character is a newline, set a flag
       // so the main loop can do something about it:
       if (inChar == '\n') {
         stringComplete = true;
       }
       else {
         // add it to the inputString:
         inputString += inChar;
       }
     }
     
    if (control == 1000) //Overwrite serial channels with signal from PPM
    {     
        digitalWrite(ledPin, LOW); //start LED off
        ppm2[0] = ppm.latestValidChannelValue(3, 0);
        ppm2[1] = ppm.latestValidChannelValue(1, 0);
        ppm2[2] = ppm.latestValidChannelValue(2, 0);
        ppm2[3] = ppm.latestValidChannelValue(4, 0);
    }

    if (control == 2000) //Accept serial channels 
    {
      digitalWrite(ledPin, HIGH); //start LED ON
    }
    ppm2[4] = 1500;
    ppm2[5] = 1500;

    ElapsedTime = currentTime - lastString;

    if (ElapsedTime >= 500)
    {
      ppm2[0] = 1000;
      ppm2[1] = 1500;
      ppm2[2] = 1500;
      ppm2[3] = 1500;
    }
  
    // wait before sending next packet
    while(micros() < timeout) // timeout for CX-10 blue = 6000microseconds.
    {
      // overrun_cnt+=1;
      };
}

void set_txid(bool renew)
{
    uint8_t i;
    for(i=0; i<4; i++)
        transmitterID[i] = EEPROM.read(ee_TXID0+i);
    if(renew || (transmitterID[0]==0xFF && transmitterID[1]==0x0FF)) {
        for(i=0; i<4; i++) {
            transmitterID[i] = random() & 0xFF;
            EEPROM.update(ee_TXID0+i, transmitterID[i]);
        }
    }
}

void selectProtocol()
{
    // Modified and commented out lines so that Cheerson CX-10 Blue is always selected
    // wait for multiple complete ppm frames
    ppm_ok = false;

    current_protocol = PROTO_E010; // EAchine E010, NiHui NH-010, JJRC H36 mini
    
    // update eeprom
    EEPROM.update(ee_PROTOCOL_ID, current_protocol);
    // wait for safe throttle
}

void init_protocol()
{
    switch(current_protocol) {
        case PROTO_E010:
            MJX_init();
            MJX_bind();
            break;
    }
}


      
      
