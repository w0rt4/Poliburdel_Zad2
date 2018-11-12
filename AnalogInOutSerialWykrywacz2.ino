#define resetTrigger 5
#define lowSens 2
#define medSens 3
#define highSens 4


// These constants won't change. They're used to give names to the pins used:
const int analogInPin = A2;  // Analog input pin that the potentiometer is attached to

int looptime = 2; // [ms]
int sensorValue = 0;        // value read from the pot
int outputValue = 0;        // value output to the PWM (analog out)
bool dataTransfer = false;
bool plot = false; // czy wysyłać wyświetlanie czy nie
bool state = false; // określa czy jest stan wysoki na buzie
int counter1 = 0;
int counter2 = 0;
int time1 = looptime * 5;
int dataIn = 0;


void setup() {
  pinMode(resetTrigger, OUTPUT);
  pinMode(lowSens, OUTPUT);
  pinMode(medSens, OUTPUT);
  pinMode(highSens, OUTPUT);
  digitalWrite(resetTrigger, HIGH);
  digitalWrite(lowSens, HIGH);
  digitalWrite(medSens, HIGH);
  digitalWrite(highSens, HIGH);
  Serial.begin(115200);
}

void loop() {

  start:

  if (Serial.available()){
    dataIn = Serial.read();
    switch (dataIn) {
      case 'a': // kalibracja/reset -> ustaw stan wysoki na pinie przez 1 sekunde okolo
      digitalWrite(resetTrigger, LOW);
      delay(2000); 
      digitalWrite(resetTrigger, HIGH);
      break;

      case 'b': // sensitivity low
      digitalWrite(lowSens, LOW);
      delay(2000); 
      digitalWrite(lowSens, HIGH);
      break;

      case 'c': // sensitivity medium
      digitalWrite(medSens, LOW);
      delay(2000); 
      digitalWrite(medSens, HIGH);
      break;

      case 'd': // sensitivity high
      digitalWrite(highSens, LOW);
      delay(2000); 
      digitalWrite(highSens, HIGH);
      break;

      case 'e': // start data transfer
      dataTransfer = true;
      counter1 = 1;
      counter2 = 0;
      break;

      case 'f': // stop data transfer
      dataTransfer = false;
      break;
      
    }
  }

  if (dataTransfer == true){
    
    sensorValue = analogRead(analogInPin);
    outputValue = map(sensorValue, 0, 1023, 0, 255);

    if (plot == true){
      Serial.print(outputValue);
      Serial.print(" ");
      Serial.print(0);
      Serial.print(" ");
      Serial.println(255); 
    }
    
    if (outputValue >= 180){ // sprawdzanie czy jest stan wysoki na buzie
      counter2 = counter2 + counter1;
      counter1 = 1;
    }
    else {
      if (counter2 == 0){
        goto start; // jesli jest niski i nie było przed chwila wysokiego to od poczatku
      }
      else {
        counter1++;
        if (counter1 >= time1){
          Serial.println(counter2);// send counter2
          counter1 = 1;
          counter2 = 0;
        }
        else {
          goto start;
        }
      }
    }
    
  }

  delay(looptime);
}
