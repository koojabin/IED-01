#define PIN_LED 13
unsigned int count, toggle;

void setup() {
  pinMode(PIN_LED, OUTPUT);
  Serial.begin(115200); // Initialize serial port
  while (!Serial) {
    ; // wait for serial port to connect.
  }
  Serial.println("Hello World!");
  count = toggle = 0;
  digitalWrite(PIN_LED, toggle); // turn off LED.
}

void loop() {
  Serial.println(++count);
  toggle_state(toggle); //toggle LED value.
}

void toggle_state(int toggle){
  digitalWrite(PIN_LED, toggle);
  delay(1000);
  toggle = 1;
  digitalWrite(PIN_LED, toggle);
  delay(1000);
}
