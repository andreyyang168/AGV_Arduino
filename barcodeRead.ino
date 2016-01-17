#include <PS2Keyboard.h>
const int DataPin = 30;
const int IRQpin =  3;

PS2Keyboard keyboard;

void setup() {
  keyboard.begin(DataPin, IRQpin);

  delay(1000);
  Serial.begin(9600);
  Serial.println("Keyboard Test:");  
}

void loop() {
  if (keyboard.available()) {

    char c = keyboard.read();
    
    if (c == PS2_ENTER) {
      Serial.println();
    } else {
      Serial.print(c);
    }

  }
  delay(350);
}
