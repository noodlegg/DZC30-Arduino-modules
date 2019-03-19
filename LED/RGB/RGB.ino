#define BPIN 6
#define RPIN 5
#define GPIN 3
int selectPin;
int pinCounter;

// Color arrays
int black[3] =  {0, 0, 0};
int white[3] =  {100, 100, 100};
int red[3]   =  {100, 0, 0};
int green[3] =  {0, 100, 0};
int blue[3]  =  {0, 0, 100};
int yellow[3] = {40, 95, 0};
int dimWhite[3] = {30, 30, 30};

void setup() {
  // put your setup code here, to run once:
  pinMode(RPIN, OUTPUT);
  pinMode(GPIN, OUTPUT);
  pinMode(BPIN, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  displayColor(red);
  displayColor(white);
  displayColor(blue);
}

void displayColor(int color[3]) {
  for (int i = 0; i < 3; i++) {
    cycleRGBPins(pinCounter);
    pinCounter++;
    analogWrite(selectPin, color[i]);
    Serial.print(selectPin);
    Serial.print(" is set to value: ");
    Serial.println(color[i]);
  }
  delay(1000);
}

// After calling this function, do "pinCount++" to cycle through the pins
void cycleRGBPins(int pinCounter) {
  int pin = pinCounter % 3;
  switch (pin) {
    case (0):
      selectPin = RPIN;
      break;
    case (1):
      selectPin = GPIN;
      break;
    case (2):
      selectPin = BPIN;
      break;
  }
}

void randomPin() {
  int randNum = random(1, 4);
  switch(randNum) {
    case (1):
      selectPin = RPIN;
      break;
    case (2):
      selectPin = GPIN;
      break;
    case (3):
      selectPin = BPIN;
      break;
  }
}
