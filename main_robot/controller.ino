// Define pin numbers
const int transmitPin = 2;  // Change this to your chosen pin

void setup() {
  pinMode(transmitPin, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  const char* message = "Hello, receiver!";
  Serial.println("Transmitting: ");
  sendMessage(message);
  delay(1000);  // Wait for a second before sending the next message
}

void sendMessage(const char* message) {
  for (int i = 0; message[i] != '\0'; i++) {
    char currentChar = message[i];
    sendByte(currentChar);
  }
  sendByte('\0');  // Null-terminate the message
}

void sendByte(char byteToSend) {
  for (int i = 7; i >= 0; i--) {
    bool bitToSend = (byteToSend >> i) & 1;
    digitalWrite(transmitPin, bitToSend);
    delayMicroseconds(500);  // Adjust this delay according to your requirements
  }
  digitalWrite(transmitPin, LOW);  // End of byte
  delayMicroseconds(500);  // Adjust this delay according to your requirements
}
