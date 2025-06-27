#include <DHT.h>
#include <SPI.h>
#include <SD.h>

// -- DHT11 Setup --
#define DHTPIN 23           // GPIO pin where DHT11 is connected
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);
float temperature, humidity;

// -- SD Card Setup --
#define SD_CS 13            // Chip select pin
SPIClass *hspi = nullptr;
File logFile;

void setup() {
  Serial.begin(115200);
  delay(1000);  // Time to open Serial Monitor

  // Initialize DHT11 sensor
  dht.begin();

  // Init SPI and SD
  hspi = new SPIClass(HSPI);
  hspi->begin(14, 2, 15, 13);  // SCK, MISO, MOSI

  if (!SD.begin(SD_CS, *hspi)) {
    Serial.println("SD Card Mount Failed!");
    return;
  }

  Serial.println("SD Card initialized.");

  // Open or create log file
  logFile = SD.open("/dht_log.csv", FILE_APPEND);
  if (!logFile) {
    Serial.println("Failed to open log file!");
  } else {
    logFile.println("temperature,humidity");  // CSV header
    logFile.close();
  }
}

void loop() {
  // Read temperature and humidity
  humidity = dht.readHumidity();
  temperature = dht.readTemperature();

  // Check if readings are valid
  if (isnan(humidity) || isnan(temperature)) {
    Serial.println("Failed to read from DHT sensor!");
    delay(1000);
    return;
  }

  // === Plot CSV ===
  Serial.print(temperature); Serial.print(",");
  Serial.println(humidity);

  // === Log to SD Card ===
  logFile = SD.open("/dht_log.csv", FILE_APPEND);
  if (logFile) {
    logFile.print(temperature); logFile.print(",");
    logFile.println(humidity);
    logFile.close();
  }

  delay(2000);  // DHT11 can be read every ~2 seconds
}