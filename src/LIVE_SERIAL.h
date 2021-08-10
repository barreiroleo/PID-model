void LIVESERIAL(String id, String value) {
    Serial.print(id);
    Serial.print(F(":"));
    Serial.println(value);
}

void LIVESERIAL_MILLIS(String id, String value, unsigned long ms) {
    Serial.print(id);  Serial.print(F(":"));
    Serial.print(value);  Serial.print(F("@"));
    Serial.println(ms);
}