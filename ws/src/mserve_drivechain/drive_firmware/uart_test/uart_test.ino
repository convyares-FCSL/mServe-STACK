// Minimal UART link test for the DDSM Driver HAT.
// - Prints a heartbeat line every 2s.
// - Echoes back any line received on Serial (the Pi GPIO UART link).

unsigned long lastHeartbeat = 0;
const unsigned long HEARTBEAT_INTERVAL_MS = 2000;
unsigned long heartbeatCount = 0;
String line;

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("{\"T\":1,\"msg\":\"boot\"}");
}

void loop() {
  unsigned long now = millis();
  if (now - lastHeartbeat >= HEARTBEAT_INTERVAL_MS) {
    lastHeartbeat = now;
    heartbeatCount++;
    Serial.print("{\"T\":1,\"hb\":");
    Serial.print(heartbeatCount);
    Serial.print(",\"ms\":");
    Serial.print(now);
    Serial.println("}");
  }

  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n') {
      Serial.print("{\"T\":2,\"echo\":\"");
      Serial.print(line);
      Serial.println("\"}");
      line = "";
    } else if (c != '\r') {
      line += c;
    }
  }
}
