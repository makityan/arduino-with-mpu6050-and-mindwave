////////////////////////////////////////////////////////////////////////
// Arduino Bluetooth Interface with Brainsense
// 
// This is example code provided by Pantech Prolabs. and is provided
// license free.
////////////////////////////////////////////////////////////////////////

#include <boarddefs.h>
#include <IRremote.h>
#include <IRremoteInt.h>
#include <ir_Lego_PF_BitStreamEncoder.h>

IRsend fuck;
int sensorPin = 12;
int sensorPinUP = 10;
int sensorPinDOWN = 9;

#define BAUDRATE 57600
#define DEBUGOUTPUT 0


#define BLUELED1 5
#define BLUELED2 6
#define BLUELED3 7

#define GREENLED1 8
#define GREENLED2 9
#define GREENLED3 10

#define REDLED1 11

#define powercontrol 10

// checksum variables
byte generatedChecksum = 0;
byte checksum = 0; 
int payloadLength = 0;
byte payloadData[64] = {0};
byte poorQuality = 0;
byte attention = 0;
byte meditation = 0;

// system variables
long lastReceivedPacket = 0;
boolean bigPacket = false;

//////////////////////////
// Microprocessor Setup //
//////////////////////////
void setup() 

{
//  Serial.begin(9600);
 pinMode(sensorPin, INPUT);
  digitalWrite(sensorPin, HIGH);
   pinMode(sensorPinUP, INPUT);
  digitalWrite(sensorPinUP, HIGH);
     pinMode(sensorPinDOWN, INPUT);
  digitalWrite(sensorPinDOWN, HIGH);
  
  pinMode(BLUELED1, OUTPUT);
  pinMode(BLUELED2, OUTPUT);
  pinMode(BLUELED3, OUTPUT);
  pinMode(GREENLED1, OUTPUT);
  pinMode(GREENLED2, OUTPUT);
  pinMode(GREENLED3, OUTPUT);
  pinMode(REDLED1, OUTPUT);


  Serial.begin(BAUDRATE);           // USB
}

////////////////////////////////
// Read data from Serial UART //
////////////////////////////////
byte ReadOneByte() 

{
  int ByteRead;
  while(!Serial.available());
  ByteRead = Serial.read();

#if DEBUGOUTPUT  
  Serial.print((char)ByteRead);   // echo the same byte out the USB serial (for debug purposes)
#endif

  return ByteRead;
}

/////////////
//MAIN LOOP//
/////////////
void loop() 

{
  int count=0;
int sensorValue = digitalRead(sensorPin); 
int sensorValueUP = digitalRead(sensorPinUP); 
int sensorValueDOWN = digitalRead(sensorPinDOWN); 
  // Look for sync bytes
  if(ReadOneByte() == 170) 
  {
    if(ReadOneByte() == 170) 
    {
        payloadLength = ReadOneByte();
      
        if(payloadLength > 169)                      //Payload length can not be greater than 169
        return;
        generatedChecksum = 0;        
        for(int i = 0; i < payloadLength; i++) 
        {  
        payloadData[i] = ReadOneByte();            //Read payload into memory
        generatedChecksum += payloadData[i];
        }   

        checksum = ReadOneByte();                      //Read checksum byte from stream      
        generatedChecksum = 255 - generatedChecksum;   //Take one's compliment of generated checksum

        if(checksum == generatedChecksum) 
        {    
          poorQuality = 200;
          attention = 0;
          meditation = 0;

          for(int i = 0; i < payloadLength; i++) 
          {                                          // Parse the payload
          switch (payloadData[i]) 
          {
          case 2:
            i++;            
            poorQuality = payloadData[i];
            bigPacket = true;            
            break;
          case 4:
            i++;
            attention = payloadData[i];                        
            break;
          case 5:
            i++;
            meditation = payloadData[i];
            break;
          case 0x80:
            i = i + 3;
            break;
          case 0x83:
            i = i + 25;      
            break;
          default:
            break;
          } // switch
        } // for loop

#if !DEBUGOUTPUT

        // *** Add your code here ***

        if(bigPacket) 
        {
          if(poorQuality == 0)
              digitalWrite(GREENLED3, HIGH);
          else
              digitalWrite(GREENLED3, LOW);
          
          Serial.print("PoorQuality: ");
          Serial.print(poorQuality, DEC);
          Serial.print(" Attention: ");
          Serial.print(attention, DEC);
          Serial.print(" Time since last packet: ");
          Serial.print(millis() - lastReceivedPacket, DEC);
          lastReceivedPacket = millis();
          Serial.print("\n");

          switch(attention / 14) 
          {
          case 0:
            digitalWrite(BLUELED1, LOW);
            digitalWrite(BLUELED2, LOW);
            digitalWrite(BLUELED3, LOW);  
            digitalWrite(GREENLED1, LOW);
            digitalWrite(GREENLED2, LOW);
            digitalWrite(GREENLED3, LOW);
            digitalWrite(REDLED1, LOW);
             count==0;
            break;
          case 1:

            digitalWrite(BLUELED1, HIGH);
            digitalWrite(BLUELED2, LOW);
            digitalWrite(BLUELED3, LOW);
            digitalWrite(GREENLED1, LOW);
            digitalWrite(GREENLED2, LOW);
            digitalWrite(GREENLED3, LOW);
            digitalWrite(REDLED1, LOW);

            break;
          case 2:
            digitalWrite(BLUELED1, HIGH);
            digitalWrite(BLUELED2, HIGH);
            digitalWrite(BLUELED3, LOW);
            digitalWrite(GREENLED1, LOW);
            digitalWrite(GREENLED2, LOW);
            digitalWrite(GREENLED3, LOW);
            digitalWrite(REDLED1, LOW);

            break;
          case 3:              
            digitalWrite(BLUELED1, HIGH);
            digitalWrite(BLUELED2, HIGH);
            digitalWrite(BLUELED3, HIGH);
            digitalWrite(GREENLED1, LOW);
            digitalWrite(GREENLED2, LOW);
            digitalWrite(GREENLED3, LOW);
            digitalWrite(REDLED1, LOW);    
                        if(count==0){
                        for (int i = 0; i < 3; i++) {
fuck.sendPanasonic(0x555A, 0xF148688B); // Sonyフォーマットでテレビ電源ONの信号
delay(40);
                        }
                        count++;
      Serial.print("FUCK");
} 
            break; 
          case 4:              
            digitalWrite(BLUELED1, HIGH);
            digitalWrite(BLUELED2, HIGH);
            digitalWrite(BLUELED3, HIGH);
            digitalWrite(GREENLED1, HIGH);
            digitalWrite(GREENLED2, LOW);
            digitalWrite(GREENLED3, LOW);
            digitalWrite(REDLED1, LOW);    
for (int i = 0; i < 3; i++) {
fuck.sendPanasonic(0x555A, 0xF1484889); // Sonyフォーマットでテレビ電源ONの信号
delay(40);
      Serial.print("DOWN");

}   
            break; 
          case 5:              
            digitalWrite(BLUELED1, HIGH);
            digitalWrite(BLUELED2, HIGH);
            digitalWrite(BLUELED3, HIGH);
            digitalWrite(GREENLED1, HIGH);
            digitalWrite(GREENLED2, HIGH);
            digitalWrite(GREENLED3, LOW);
            digitalWrite(REDLED1, LOW);    
            for (int i = 0; i < 3; i++) {
fuck.sendPanasonic(0x555A, 0xF1488885); // Sonyフォーマットでテレビ電源ONの信号
delay(40);
      Serial.print("UP");
}


            break; 
          case 6:              
            digitalWrite(BLUELED1, HIGH);
            digitalWrite(BLUELED2, HIGH);
            digitalWrite(BLUELED3, HIGH);
            digitalWrite(GREENLED1, HIGH);
            digitalWrite(GREENLED2, HIGH);
            digitalWrite(GREENLED3, HIGH);
            digitalWrite(REDLED1, LOW);    
            
            break; 
          case 7:              
            digitalWrite(BLUELED1, HIGH);
            digitalWrite(BLUELED2, HIGH);
            digitalWrite(BLUELED3, HIGH);
            digitalWrite(GREENLED1, HIGH);
            digitalWrite(GREENLED2, HIGH);
            digitalWrite(GREENLED3, HIGH);
            digitalWrite(REDLED1, HIGH);    

            break; 
 
          }                    
        }
#endif        
        bigPacket = false;      
      }
      else {
        // Checksum Error
      }  // end if else for checksum
    } // end if read 0xAA byte
  } // end if read 0xAA byte

}
