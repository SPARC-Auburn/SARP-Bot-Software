/* Encoder Library - Basic Example
 * http://www.pjrc.com/teensy/td_libs_Encoder.html
 *
 * This example code is in the public domain.
 */
#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>

// Change these two numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
Encoder myEnc(5, 6);
Encoder myEnc2(11, 12);
//   avoid using pins with LEDs attached

void setup() {
  delay(1000);
  pinMode(13,OUTPUT);

  Serial.begin(115200);
  digitalWrite(13,HIGH);
}
int32_t a; 
int32_t b;
bool state = false;
void loop() {
  if (state){
    digitalWrite(13,LOW);
    state = false;
  }
  else{
    digitalWrite(13,HIGH);
    state = true;
  }
  a = myEnc.read();
  b = myEnc2.read();
  int64_t x = a;
  Serial.print("<");
  Serial.print(a);
  Serial.print(",");
  Serial.print(b);
  Serial.println(">");

}
