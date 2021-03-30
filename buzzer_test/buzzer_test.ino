byte buzzer = 2;

void setup() {
  pinMode(buzzer,OUTPUT);
  for(int i = 1; i<10;i++){
    tone(buzzer,2200);
    delay(30);
    tone(buzzer,2200);
    noTone(buzzer);
    delay(1000);
  }
}

void loop() {

}
