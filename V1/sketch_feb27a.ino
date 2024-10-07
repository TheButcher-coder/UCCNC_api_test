
// constants won't change. They're used here to set pin numbers:
const int buttonPin = A0;  // the number of the pushbutton pin
const int rst_button = 2;
//const int ledPin = 13;    // the number of the LED pin

// variables will change:
int buttonState = 0;  // variable for reading the pushbutton status
int rst_state=0;
void setup() {
  // initialize the LED pin as an output:
  //pinMode(ledPin, OUTPUT);
  // initialize the pushbutton pin as an input:
  pinMode(buttonPin, INPUT);
  pinMode(rst_button, INPUT);
  pinMode(13, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  int butt, rst;
  // read the state of the pushbutton value:
  buttonState = analogRead(buttonPin);
  rst_state = digitalRead(rst_button);
  //Serial.println(buttonState);
  // check if the pushbutton is pressed. If it is, the buttonState is HIGH:

  if(buttonState > 90) {
    digitalWrite(13, HIGH);
    Serial.print("1,");
    Serial.println(!rst_state);
  }
  else {
    digitalWrite(13, LOW);
    Serial.print("0,");
    Serial.println(!rst_state);
  }
  delay(1000);
  /*
  if(buttonState == HIGH) butt = 1;
  else butt = 0;

  if(rst_state == HIGH) rst = 1;
  else rst = 0;
  //Serial.print(butt);
  //Serial.print(",");
  //Serial.print(rst);
  //Serial.print("\n");
  //Serial.print(char(butt) + ", " + char(rst) + "\n");
  delay(50);
*/

/*
  if (buttonState == HIGH) {
    // turn LED on:
    digitalWrite(13, HIGH);
    Serial.println("1");
  } else {
    // turn LED off:
    digitalWrite(13, LOW);
    Serial.println("0");
  }
  */
  //delay(1000);
}
