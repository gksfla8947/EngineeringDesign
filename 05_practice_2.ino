#define PIN_LED 7
unsigned int count, toggle;

void setup() {
  pinMode(PIN_LED, OUTPUT);
  Serial.begin(115200); // Initialize serial port
  while (!Serial) {
    ; // wait for serial port to connect.
  }
  Serial.println("Hello World!");
  count = toggle = 0;
  digitalWrite(PIN_LED, 1); // turn off LED.
}

void loop() {
  if(count == 0) {
    digitalWrite(PIN_LED, 0);
    delay(1000);
  }
  else if(count > 10) {
    digitalWrite(PIN_LED, 1);
    while(1) {}
  }
  else {
    toggle = toggle_state(toggle);
    digitalWrite(PIN_LED, toggle);
    delay(100);
  }
  count += 1;
  Serial.println(count);
}

int toggle_state(int toggle) {
  if(toggle == 0) { toggle = 1; }
  else { toggle = 0; }
  return toggle;
}
