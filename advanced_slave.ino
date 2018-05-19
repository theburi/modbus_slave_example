/**
 *  Modbus slave example 2:
 *  The purpose of this example is to link the Arduino digital and analog
 *	pins to an external device.
 *
 *  Recommended Modbus Master: QModbus
 *  http://qmodbus.sourceforge.net/
 */

// https://github.com/smarmengol/Modbus-Master-Slave-for-Arduino/blob/master/ModbusRtu.h
#include <ModbusRtu.h>
#include <SoftwareSerial.h>

#define ID   1

Modbus slave(ID, 11, 4); // ID of slave, Send PIN, Transmission Active
boolean led;
int8_t state = 0;
unsigned long tempus;
SoftwareSerial modbusSerial(10, 11);

// data array for modbus network sharing
uint16_t au16data[16] = {
  3, 415, 265, 4, 2, 7182, 28182, 8, 0, 0, 0, 0, 0, 0, 1, -1 };

/**
 *  Setup procedure
 */
void setup() {
  
  Serial.begin(115200);
  Serial.print("SETUP ");
  
//  io_setup(); // I/O settings

  // start communication
  slave.begin(&modbusSerial, 9600 );
  tempus = millis() + 100;
  digitalWrite(13, HIGH );
  Serial.println("COMPLETE");
}

/**
 *  Loop procedure
 */
void loop() {
  // poll messages
  // blink led pin on each valid message
  state = slave.poll( au16data, 16 );
  if (state > 4) {   
    Serial.println((String) "State " + state + " Errors " + slave.getErrCnt() + " IN " + slave.getInCnt() + " OUT " + slave.getOutCnt());
    tempus = millis() + 50;
    digitalWrite(13, HIGH);
  }
  if (millis() > tempus) digitalWrite(13, LOW );

  // link the Arduino pins to the Modbus array
//  io_poll();

} 

/**
 * pin maping:
 * 2 - digital input
 * 3 - digital input
 * 4 - digital input
 * 5 - digital input
 * 6 - digital output
 * 7 - digital output
 * 8 - digital output
 * 9 - digital output
 * 10 - analog output
 * 11 - analog output
 * 14 - analog input
 * 15 - analog input
 *
 * pin 13 is reserved to show a successful query
 */
void io_setup() {
  // define i/o
  pinMode(5, INPUT);
  pinMode(6, INPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(13, OUTPUT);

  digitalWrite(7, LOW );
  digitalWrite(8, LOW );
  digitalWrite(9, LOW );
  digitalWrite(13, HIGH ); // this is for the UNO led pin
  analogWrite(10, 0 );
  analogWrite(11, 0 );
}

/**
 *  Link between the Arduino pins and the Modbus array
 */
void io_poll() {
  // get digital inputs -> au16data[0]
  bitWrite( au16data[0], 0, digitalRead( 5 ));
  bitWrite( au16data[0], 1, digitalRead( 6 ));

  // set digital outputs -> au16data[1]
  digitalWrite( 7, bitRead( au16data[1], 5 ));
  digitalWrite( 8, bitRead( au16data[1], 6 ));

  // set analog outputs
  analogWrite( 10, au16data[2] );
  analogWrite( 11, au16data[3] );

  // read analog inputs
  au16data[4] = analogRead( 0 );
  au16data[5] = analogRead( 1 );

  // diagnose communication
  au16data[6] = slave.getInCnt();
  au16data[7] = slave.getOutCnt();
  au16data[8] = slave.getErrCnt();
}
