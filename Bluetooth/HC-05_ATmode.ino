#include <SoftwareSerial.h>

#define bt_power 7
#define bt_key_power 8
#define indication_led 13

SoftwareSerial BT(A0, A1); // RX | TX

void setup()
{
  // set the pins to OUTPUT
  pinMode(bt_power, OUTPUT);  
  pinMode(bt_key_power, OUTPUT);
  pinMode(indication_led, OUTPUT);
  
  // set the pins to LOW
  digitalWrite(bt_power, LOW);
  digitalWrite(bt_key_power, LOW);
  digitalWrite(indication_led, LOW);
  
  /************************************************
  Setting the pins to low is important because 
  in order for us to get into AT mode the key pin
  has to be set to Ground FIRST. Many tutorials out
  there fail to mention this important fact and 
  therefore many people have problems with getting 
  into the AT mode of the HC-05
  ************************************************/
  
  // make sure the key has been LOW for a bit
  delay(100);
  
  // set the key pin to High
  digitalWrite(bt_key_power, HIGH);
  
  // small delay
  delay(100);
  
  // now power on the BT
  digitalWrite(bt_power, HIGH);
  
  // start our serial so we can send and recieve
  // information from the BT module
  Serial.begin(9600);
  // initiate the BT serial at 38400 which is the default 
  // speed at which the BT AT mode operates at
  BT.begin(38400);
  
  // self explanatory
  Serial.write("For a list of commands, visit: \n");
  Serial.write("Type AT commands  \n\n");
  
  // process complete turn on led 13
  digitalWrite(indication_led, HIGH);
  
  // Send an "AT" command to the AT (without quotes)
  // if response is OK, then we are connected
  // and ready to program the BT module
 }

void loop()
{

  // listen for a response from the HC-05 and write it to the serial monitor
  if (BT.available())
    Serial.write(BT.read());

  // listen for user input and send it to the HC-05
  if (Serial.available())
    BT.write(Serial.read());
    
}
