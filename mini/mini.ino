
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
                  Serial.print(analogRead(2));
                        } 
            }
            delay(100); 
            }
            

