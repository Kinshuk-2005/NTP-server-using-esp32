#include <WiFi.h>
#include <WiFiUdp.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>

// Access Point credentials - other devices will connect to this
const char* ap_ssid = "GPS_NTP_Server";
const char* ap_password = "gpstime123";  // Minimum 8 characters

// GPS settings
HardwareSerial gpsSerial(2);
TinyGPSPlus gps;

// GPS pins
#define GPS_RX_PIN 16
#define GPS_TX_PIN 17
#define PPS_PIN 4  // PPS input pin

// NTP settings
WiFiUDP udp;
const int NTP_PACKET_SIZE = 48;
byte packetBuffer[NTP_PACKET_SIZE];
unsigned int localPort = 123;

// Access Point IP configuration
IPAddress local_IP(192, 168, 4, 1);      // ESP32 IP
IPAddress gateway(192, 168, 4, 1);       // Gateway IP
IPAddress subnet(255, 255, 255, 0);      // Subnet mask

// PPS tracking
volatile unsigned long ppsTimestamp = 0;
volatile bool ppsReceived = false;
portMUX_TYPE ppsMux = portMUX_INITIALIZER_UNLOCKED;

// GPS time at last PPS
struct {
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
  unsigned long ppsTime;
  bool valid;
} lastPPSTime;

bool timeValid = false;

// PPS interrupt handler
void IRAM_ATTR ppsInterrupt() {
  portENTER_CRITICAL_ISR(&ppsMux);
  ppsTimestamp = micros();
  ppsReceived = true;
  portEXIT_CRITICAL_ISR(&ppsMux);
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n=== ESP32 GPS-PPS NTP Server (AP Mode) ===");
  
  // Initialize PPS pin
  pinMode(PPS_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PPS_PIN), ppsInterrupt, RISING);
  Serial.printf("PPS pin initialized on GPIO %d\n", PPS_PIN);
  
  // Initialize GPS
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Serial.println("GPS initialized on UART2");
  Serial.println("Waiting for GPS fix...");
  
  // Configure Access Point
  Serial.println("\nConfiguring Access Point...");
  
  WiFi.softAPConfig(local_IP, gateway, subnet);
  WiFi.softAP(ap_ssid, ap_password);
  
  IPAddress apIP = WiFi.softAPIP();
  Serial.println("Access Point created!");
  Serial.println("================================");
  Serial.printf("Network Name (SSID): %s\n", ap_ssid);
  Serial.printf("Password: %s\n", ap_password);
  Serial.printf("NTP Server IP: %s\n", apIP.toString().c_str());
  Serial.println("================================");
  
  // Start UDP server
  udp.begin(localPort);
  Serial.printf("\nNTP Server listening on port %d\n", localPort);
  
  // Wait for GPS fix and PPS sync
  Serial.println("\nWaiting for GPS fix and PPS sync...");
  while (!timeValid) {
    updateGPS();
    delay(100);
  }
  
  Serial.println("\n*** NTP Server Ready ***");
  Serial.println("High-precision GPS-PPS time source active");
  Serial.println("\nClients can now:");
  Serial.printf("1. Connect to WiFi: %s\n", ap_ssid);
  Serial.printf("2. Use NTP Server: %s\n", apIP.toString().c_str());
}

void loop() {
  updateGPS();
  handleNTPRequests();
  
  // Status update every 10 seconds
  static unsigned long lastStatus = 0;
  if (millis() - lastStatus > 10000) {
    lastStatus = millis();
    printStatus();
  }
  
  delay(1);
}

void updateGPS() {
  // Process GPS data
  while (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      // Check if we got updated time
      if (gps.time.isUpdated() && gps.date.isValid()) {
        // Check if PPS was received recently
        portENTER_CRITICAL(&ppsMux);
        bool pps = ppsReceived;
        unsigned long ppsTime = ppsTimestamp;
        ppsReceived = false;
        portEXIT_CRITICAL(&ppsMux);
        
        if (pps) {
          // Store GPS time aligned with PPS pulse
          lastPPSTime.year = gps.date.year();
          lastPPSTime.month = gps.date.month();
          lastPPSTime.day = gps.date.day();
          lastPPSTime.hour = gps.time.hour();
          lastPPSTime.minute = gps.time.minute();
          lastPPSTime.second = gps.time.second();
          lastPPSTime.ppsTime = ppsTime;
          lastPPSTime.valid = true;
          timeValid = true;
        }
      }
    }
  }
}

void handleNTPRequests() {
  int packetSize = udp.parsePacket();
  
  if (packetSize) {
    if (!lastPPSTime.valid) {
      Serial.println("NTP request ignored - no PPS sync yet");
      return;
    }
    
    // Get current precise time
    unsigned long currentMicros = micros();
    unsigned long microsElapsed = currentMicros - lastPPSTime.ppsTime;
    
    // Ignore if too long since last PPS (>2 seconds = bad sync)
    if (microsElapsed > 2000000) {
      Serial.println("NTP request ignored - PPS too old");
      return;
    }
    
    Serial.printf("NTP request from %s:%d\n", 
                  udp.remoteIP().toString().c_str(), udp.remotePort());
    
    // Read request
    udp.read(packetBuffer, NTP_PACKET_SIZE);
    
    // Build response
    memset(packetBuffer, 0, NTP_PACKET_SIZE);
    
    // NTP header - Stratum 1 GPS server
    packetBuffer[0] = 0b00100100;   // LI=0, VN=4, Mode=4
    packetBuffer[1] = 1;             // Stratum 1 (GPS)
    packetBuffer[2] = 6;             // Poll interval
    packetBuffer[3] = 0xFA;          // Precision (-6 = ~15 microseconds)
    
    // Root delay (microseconds in NTP short format)
    packetBuffer[4] = 0;
    packetBuffer[5] = 0;
    packetBuffer[6] = 0;
    packetBuffer[7] = 10;  // ~2.3 microseconds
    
    // Root dispersion
    packetBuffer[8] = 0;
    packetBuffer[9] = 0;
    packetBuffer[10] = 0;
    packetBuffer[11] = 10;
    
    // Reference ID - 'GPS\0'
    packetBuffer[12] = 'G';
    packetBuffer[13] = 'P';
    packetBuffer[14] = 'S';
    packetBuffer[15] = 0;
    
    // Calculate NTP timestamp
    unsigned long ntpSeconds;
    unsigned long ntpFraction;
    getPreciseNTPTime(microsElapsed, &ntpSeconds, &ntpFraction);
    
    // Reference Timestamp (last PPS sync)
    unsigned long refSeconds = getUnixTime(lastPPSTime.year, lastPPSTime.month, 
                                           lastPPSTime.day, lastPPSTime.hour, 
                                           lastPPSTime.minute, lastPPSTime.second) + 2208988800UL;
    packetBuffer[16] = (refSeconds >> 24) & 0xFF;
    packetBuffer[17] = (refSeconds >> 16) & 0xFF;
    packetBuffer[18] = (refSeconds >> 8) & 0xFF;
    packetBuffer[19] = refSeconds & 0xFF;
    packetBuffer[20] = 0;
    packetBuffer[21] = 0;
    packetBuffer[22] = 0;
    packetBuffer[23] = 0;
    
    // Receive Timestamp
    packetBuffer[32] = (ntpSeconds >> 24) & 0xFF;
    packetBuffer[33] = (ntpSeconds >> 16) & 0xFF;
    packetBuffer[34] = (ntpSeconds >> 8) & 0xFF;
    packetBuffer[35] = ntpSeconds & 0xFF;
    packetBuffer[36] = (ntpFraction >> 24) & 0xFF;
    packetBuffer[37] = (ntpFraction >> 16) & 0xFF;
    packetBuffer[38] = (ntpFraction >> 8) & 0xFF;
    packetBuffer[39] = ntpFraction & 0xFF;
    
    // Transmit Timestamp
    packetBuffer[40] = (ntpSeconds >> 24) & 0xFF;
    packetBuffer[41] = (ntpSeconds >> 16) & 0xFF;
    packetBuffer[42] = (ntpSeconds >> 8) & 0xFF;
    packetBuffer[43] = ntpSeconds & 0xFF;
    packetBuffer[44] = (ntpFraction >> 24) & 0xFF;
    packetBuffer[45] = (ntpFraction >> 16) & 0xFF;
    packetBuffer[46] = (ntpFraction >> 8) & 0xFF;
    packetBuffer[47] = ntpFraction & 0xFF;
    
    // Send response
    udp.beginPacket(udp.remoteIP(), udp.remotePort());
    udp.write(packetBuffer, NTP_PACKET_SIZE);
    udp.endPacket();
    
    Serial.printf("Response sent - Time: %04d-%02d-%02d %02d:%02d:%02d.%06lu UTC\n",
                  lastPPSTime.year, lastPPSTime.month, lastPPSTime.day,
                  lastPPSTime.hour, lastPPSTime.minute, lastPPSTime.second,
                  microsElapsed);
  }
}

void getPreciseNTPTime(unsigned long microsElapsed, unsigned long* seconds, unsigned long* fraction) {
  // Start with GPS time at last PPS
  unsigned long sec = lastPPSTime.second;
  unsigned long min = lastPPSTime.minute;
  unsigned long hr = lastPPSTime.hour;
  unsigned long day = lastPPSTime.day;
  unsigned long mon = lastPPSTime.month;
  unsigned long yr = lastPPSTime.year;
  
  // Add elapsed seconds
  unsigned long addSeconds = microsElapsed / 1000000;
  unsigned long remainingMicros = microsElapsed % 1000000;
  
  sec += addSeconds;
  while (sec >= 60) {
    sec -= 60;
    min++;
    if (min >= 60) {
      min = 0;
      hr++;
      if (hr >= 24) {
        hr = 0;
        day++;
      }
    }
  }
  
  // Convert to Unix timestamp
  unsigned long unixTime = getUnixTime(yr, mon, day, hr, min, sec);
  
  // Convert to NTP timestamp
  *seconds = unixTime + 2208988800UL;
  
  // Convert microseconds to NTP fraction (2^32 / 1,000,000)
  *fraction = ((unsigned long long)remainingMicros * 4294967296ULL) / 1000000ULL;
}

unsigned long getUnixTime(uint16_t yr, uint8_t mon, uint8_t day, uint8_t hr, uint8_t min, uint8_t sec) {
  struct tm t;
  t.tm_year = yr - 1900;
  t.tm_mon = mon - 1;
  t.tm_mday = day;
  t.tm_hour = hr;
  t.tm_min = min;
  t.tm_sec = sec;
  return mktime(&t);
}

void printStatus() {
  Serial.println("\n--- Status ---");
  Serial.printf("Connected Clients: %d\n", WiFi.softAPgetStationNum());
  Serial.printf("GPS Satellites: %d\n", gps.satellites.value());
  Serial.printf("HDOP: %.2f\n", gps.hdop.hdop());
  
  if (lastPPSTime.valid) {
    unsigned long age = (micros() - lastPPSTime.ppsTime) / 1000;
    Serial.printf("Last PPS: %lu ms ago\n", age);
    Serial.printf("Current Time: %04d-%02d-%02d %02d:%02d:%02d UTC\n",
                  lastPPSTime.year, lastPPSTime.month, lastPPSTime.day,
                  lastPPSTime.hour, lastPPSTime.minute, lastPPSTime.second);
    Serial.printf("Sync Status: %s\n", age < 2000 ? "SYNCED" : "WARNING: No recent PPS");
  } else {
    Serial.println("PPS Status: Waiting for sync...");
  }
  Serial.println("--------------\n");
}
