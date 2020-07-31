#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(7, 8); // CE, CSN
const byte address[6] = "00001";
float controlValues[2];

void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();
}
void loop() {
  int forwardValue = analogRead(A1);
  int turnValue = analogRead(A5);

  int mapVal1 = map(forwardValue, 0, 1023, -512, 512);
  float angleValue = (float) mapVal1/200;

  int mapVal2 = map(turnValue, 0, 1023, -512, 512);
  float turnAngleValue = (float) mapVal2/5;

  controlValues[0] = angleValue;
  controlValues[1] = turnAngleValue;
 
  radio.write(&controlValues, sizeof(controlValues));
  
  Serial.print("forward: ");
  Serial.print(angleValue);
  Serial.print( " turn: ");
  Serial.println(turnAngleValue);
}
