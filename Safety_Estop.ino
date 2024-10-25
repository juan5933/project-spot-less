int safety = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  
}

void loop() {
  
  
 int Estop = analogRead(A0);
 int reset = analogRead(A5);
 
 if (Estop > 100){
  safety = 1;
 }
 
 if (safety == 1 && reset > 100){
   safety = 0;
  }

  Serial.println(safety);
  delay(100);


}
