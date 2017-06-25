// Code to get analog data from 0, 1, 2 inputs
// and sended by serial port
void setup () {
  Serial.begin(9600);
}

void loop () {
  if(Serial.available()) {
    char c = Serial.read();
    if (c=='B') {
      Serial.print(analogRead(0));
      Serial.print(",");
      Serial.print(analogRead(1));
      Serial.print(",");
      Serial.println(analogRead(2));
      }
  }
  delay(100);
}

