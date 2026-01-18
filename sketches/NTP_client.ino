#include <WiFi.h>
#include <time.h>

const char* ssid = "GPS_NTP_Server";     // Connect to your NTP server
const char* password = "gpstime123";
const char* ntpServer = "192.168.4.1";   // Fixed IP of NTP server

void setup() {
  Serial.begin(115200);
  
  // Connect to NTP server's WiFi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to GPS NTP Server");
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println("\nConnected!");
  
  // Sync time
  configTime(0, 0, ntpServer);
  
  struct tm timeinfo;
  if (getLocalTime(&timeinfo)) {
    Serial.println(&timeinfo, "Synced: %A, %B %d %Y %H:%M:%S");
  }
}

void loop() {
  struct tm timeinfo;
  if (getLocalTime(&timeinfo)) {
    Serial.println(&timeinfo, "%H:%M:%S");
  }
  delay(1000);
}