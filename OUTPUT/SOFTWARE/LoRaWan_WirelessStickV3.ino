#include "SD.h"
#include "FS.h"
#include "SPI.h"

// PINOUT SD CARD
// 7 6 5 4 probleme
// 34 35 36 37 probleme
#define SD_CS 34     //10:dev board mandalou ok
#define SPI_MOSI 35  //11:dev board mandalou ok
#define SPI_SCK 36   //12:dev board mandalou ok
#define SPI_MISO 37  //13:dev board mandalou ok

#define PIN_BUTTON 0

// static const uint8_t SS    = 34;  // 5
// static const uint8_t MOSI  = 35;  // 23
// static const uint8_t MISO  = 36;  // 19
// static const uint8_t SCK   = 37;  // 18

// Not every pin combination works. After a lot of tests, I found a combination that actually works:

// SD_CS – GPIO26
// SD_MOSI – GPIO21
// SD_CLK – GPIO20
// SD_MISO – GPIO19

SPIClass spi1;

int intNbAudioFileInDir = 0;
#include "LoRaWan_APP.h"



//_________________________ Temp Interne ____________________________


// ________________________________  POC Wireless stick with ESP32-S3  ________________________________
/* OTAA para*/
uint8_t devEui[] = { 0x22, 0x32, 0x33, 0x00, 0x00, 0x88, 0x88, 0x02 };
uint8_t appEui[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint8_t appKey[] = { 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88 };

/* ABP para*/
uint8_t nwkSKey[] = { 0x22, 0x25, 0x52, 0xE6, 0x45, 0x42, 0x92, 0x34, 0x3E, 0x8B, 0x01, 0x73, 0x50, 0xAD, 0xE5, 0x0C };
uint8_t appSKey[] = { 0x9A, 0xC5, 0xA5, 0xAD, 0xEF, 0x6C, 0xC2, 0xA9, 0xEE, 0x61, 0xDF, 0xF4, 0xDD, 0xCD, 0xC6, 0xA8 };
uint32_t devAddr = (uint32_t)0x260BBBEE;

/*LoraWan channelsmask, default channels 0-7*/
uint16_t userChannelsMask[6] = { 0x00FF, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 };

/*LoraWan region, select in arduino IDE tools*/
LoRaMacRegion_t loraWanRegion = ACTIVE_REGION;

/*LoraWan Class, Class A and Class C are supported*/
DeviceClass_t loraWanClass = CLASS_A;

/*the application data transmission duty cycle.  value in [ms].*/
uint32_t appTxDutyCycle = 60000;

/*OTAA or ABP*/
bool overTheAirActivation = false;  ///False = ABP

/*ADR enable*/
bool loraWanAdr = true;

/* Indicates if the node is sending confirmed or unconfirmed messages */
bool isTxConfirmed = true;  // pas d'accusé de reception

/* Application port */
uint8_t appPort = 2;
/*!
* Number of trials to transmit the frame, if the LoRaMAC layer did not
* receive an acknowledgment. The MAC performs a datarate adaptation,
* according to the LoRaWAN Specification V1.0.2, chapter 18.4, according
* to the following table:
*
* Transmission nb | Data Rate
* ----------------|-----------
* 1 (first)       | DR
* 2               | DR
* 3               | max(DR-1,0)
* 4               | max(DR-1,0)
* 5               | max(DR-2,0)
* 6               | max(DR-2,0)
* 7               | max(DR-3,0)
* 8               | max(DR-3,0)
*
* Note, that if NbTrials is set to 1 or 2, the MAC will not decrease
* the datarate, in case the LoRaMAC layer did not receive an acknowledgment
*/
uint8_t confirmedNbTrials = 4;

/* Prepares the payload of the frame */
static void prepareTxFrame(uint8_t port) {
  /*appData size is LORAWAN_APP_DATA_MAX_SIZE which is defined in "commissioning.h".
  *appDataSize max value is LORAWAN_APP_DATA_MAX_SIZE.
  *if enabled AT, don't modify LORAWAN_APP_DATA_MAX_SIZE, it may cause system hanging or failure.
  *if disabled AT, LORAWAN_APP_DATA_MAX_SIZE can be modified, the max value is reference to lorawan region and SF.
  *for example, if use REGION_CN470, 
  *the max value for different DR can be found in MaxPayloadOfDatarateCN470 refer to DataratesCN470 and BandwidthsCN470 in "RegionCN470.h".
  */
  appDataSize = 4;
  appData[0] = 0x00;
  appData[1] = 0x01;
  appData[2] = 0x02;
  appData[3] = 0x03;


//   appDataSize = lpp.getSize();

//   appData = lpp.getBuffer();
}

//if true, next uplink will add MOTE_MAC_DEVICE_TIME_REQ


void print_wakeup_reason() {
  esp_sleep_wakeup_cause_t wakeup_reason;
  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch (wakeup_reason) {
    case ESP_SLEEP_WAKEUP_EXT0: Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1: Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER: Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD: Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP: Serial.println("Wakeup caused by ULP program"); break;
    default: Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason); break;
  }
}


void initSDCard() {
  if (!SD.begin(SD_CS, spi1)) {
    Serial.println("Card Mount Failed");
    return;
  }
  uint8_t cardType = SD.cardType();

  if (cardType == CARD_NONE) {
    Serial.println("No SD card attached");
    return;
  }

  Serial.print("SD Card Type: ");
  if (cardType == CARD_MMC) {
    Serial.println("MMC");
  } else if (cardType == CARD_SD) {
    Serial.println("SDSC");
  } else if (cardType == CARD_SDHC) {
    Serial.println("SDHC");
  } else {
    Serial.println("UNKNOWN");
  }
  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize);
}



void listDir(fs::FS &fs, const char *dirname, uint8_t levels) {
  Serial.printf("Listing directory: %s\n", dirname);

  File root = fs.open(dirname);
  if (!root) {
    Serial.println("Failed to open directory");
    return;
  }
  if (!root.isDirectory()) {
    Serial.println("Not a directory");
    return;
  }

  File file = root.openNextFile();
  while (file) {
    if (file.isDirectory()) {
      intNbAudioFileInDir = 0;
      //Serial.print("  DIR : ");
      //Serial.println(file.name());
      if (levels) {
        char addslash[100];
        strcpy(addslash, "/");
        strcat(addslash, file.name());
        //Serial.println(addslash);
        listDir(fs, addslash, levels - 1);
        //listDir(fs,file.name(), levels - 1);
      }
    } else {

      intNbAudioFileInDir++;
      char addfullpath[100];
      Serial.print("  FILE: ");
      strcpy(addfullpath, dirname);
      strcat(addfullpath, "/");
      strcat(addfullpath, file.name());
      //Serial.print(file.name());
      Serial.print(addfullpath);
      Serial.print("  SIZE: ");
      Serial.println(file.size());
    }
    file = root.openNextFile();
  }

  Serial.print("File number:");
  Serial.println(intNbAudioFileInDir);
}

void createDir(fs::FS &fs, const char *path) {
  Serial.printf("Creating Dir: %s\n", path);
  if (fs.mkdir(path)) {
    Serial.println("Dir created");
  } else {
    Serial.println("mkdir failed");
  }
}

void removeDir(fs::FS &fs, const char *path) {
  Serial.printf("Removing Dir: %s\n", path);
  if (fs.rmdir(path)) {
    Serial.println("Dir removed");
  } else {
    Serial.println("rmdir failed");
  }
}

void readFile(fs::FS &fs, const char *path) {
  Serial.printf("Reading file: %s\n", path);

  File file = fs.open(path);
  if (!file) {
    Serial.println("Failed to open file for reading");
    return;
  }

  Serial.print("Read from file: ");
  while (file.available()) {
    Serial.write(file.read());
  }
  file.close();
}

void writeFile(fs::FS &fs, const char *path, const char *message) {
  Serial.printf("Writing file: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  if (file.print(message)) {
    Serial.println("File written");
  } else {
    Serial.println("Write failed");
  }
  file.close();
}

void appendFile(fs::FS &fs, const char *path, const char *message) {
  Serial.printf("Appending to file: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    Serial.println("Failed to open file for appending");
    return;
  }
  if (file.print(message)) {
    Serial.println("Message appended");
  } else {
    Serial.println("Append failed");
  }
  file.close();
}

void renameFile(fs::FS &fs, const char *path1, const char *path2) {
  Serial.printf("Renaming file %s to %s\n", path1, path2);
  if (fs.rename(path1, path2)) {
    Serial.println("File renamed");
  } else {
    Serial.println("Rename failed");
  }
}

void deleteFile(fs::FS &fs, const char *path) {
  Serial.printf("Deleting file: %s\n", path);
  if (fs.remove(path)) {
    Serial.println("File deleted");
  } else {
    Serial.println("Delete failed");
  }
}

void testFileIO(fs::FS &fs, const char *path) {
  File file = fs.open(path);
  static uint8_t buf[512];
  size_t len = 0;
  uint32_t start = millis();
  uint32_t end = start;
  if (file) {
    len = file.size();
    size_t flen = len;
    start = millis();
    while (len) {
      size_t toRead = len;
      if (toRead > 512) {
        toRead = 512;
      }
      file.read(buf, toRead);
      len -= toRead;
    }
    end = millis() - start;
    Serial.printf("%u bytes read for %u ms\n", flen, end);
    file.close();
  } else {
    Serial.println("Failed to open file for reading");
  }


  file = fs.open(path, FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }

  size_t i;
  start = millis();
  for (i = 0; i < 2048; i++) {
    file.write(buf, 512);
  }
  end = millis() - start;
  Serial.printf("%u bytes written for %u ms\n", 2048 * 512, end);
  file.close();
}


void setup() {
  Serial.begin(115200);
  Mcu.begin();
  deviceState = DEVICE_STATE_INIT;
      Serial.println("************************************************************");
  Serial.println("START PROGRAM");
  Serial.println("************************************************************");

  Serial.print("MOSI: ");
  Serial.println(MOSI);
  Serial.print("MISO: ");
  Serial.println(MISO);
  Serial.print("SCK: ");
  Serial.println(SCK);
  Serial.print("SS: ");
  Serial.println(SS);

  // put your setup code here, to run once:
  //sd card
  pinMode(SD_CS, OUTPUT);
  digitalWrite(SD_CS, LOW);
  delay(10);
  digitalWrite(SD_CS, HIGH);
  //SPI.begin(SCK, MISO, MOSI, SS);
  //SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, SD_CS);  //SPI.beginn(Clock, MISO, MOSI, ChipSelect);

  SPIClass(1);
  spi1.begin(SPI_SCK, SPI_MISO, SPI_MOSI, SD_CS);

  Serial.print("MOSI: ");
  Serial.println(SPI_MOSI);
  Serial.print("MISO: ");
  Serial.println(SPI_MISO);
  Serial.print("SCK: ");
  Serial.println(SPI_SCK);
  Serial.print("SS: ");
  Serial.println(SD_CS);

  //SD card
  initSDCard();

  Serial.println("dir racine");
  listDir(SD, "/", 0);
  Serial.println("Contenu carte SD : ");
  intNbAudioFileInDir = 0;
  listDir(SD, "/", 1);
  //
  //  intNbAudioFileInDir = 0;
  //  listDir(SD, "/05", 0);
  //
  //  Serial.println("Contenu dossier 01 : ");
  //  intNbAudioFileInDir = 0;
  //  listDir(SD, "/01", 1);
  //  Serial.println("dir 01");
  //  listDir(SD, "/01", 0);

  //  writeFile(SD, "/hello.txt", "Hello ");
  //  appendFile(SD, "/hello.txt", "World!\n");
  //  readFile(SD, "/hello.txt");
  //  deleteFile(SD, "/foo.txt");
  //  renameFile(SD, "/hello.txt", "/foo.txt");
  //  readFile(SD, "/foo.txt");
  //  testFileIO(SD, "/test.txt");
  Serial.printf("Total space: %lluMB\n", SD.totalBytes() / (1024 * 1024));
  Serial.printf("Used space: %lluMB\n", SD.usedBytes() / (1024 * 1024));

  //memoire PSRAM
  Serial.println((String) "Memoire disponible dans la PSRAM : " + ESP.getFreePsram());
}

void loop() {

  switch (deviceState) {
    case DEVICE_STATE_INIT:
      {
#if (LORAWAN_DEVEUI_AUTO)
        LoRaWAN.generateDeveuiByChipID();
#endif
        LoRaWAN.init(loraWanClass, loraWanRegion);
        break;
      }
    case DEVICE_STATE_JOIN:
      {
        LoRaWAN.join();
        break;
      }
    case DEVICE_STATE_SEND:
      {
        prepareTxFrame(appPort);
        LoRaWAN.send();
        deviceState = DEVICE_STATE_CYCLE;
        break;
      }
    case DEVICE_STATE_CYCLE:
      {
        // Schedule next packet transmission
        txDutyCycleTime = appTxDutyCycle + randr( -APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND );
        LoRaWAN.cycle(txDutyCycleTime);
        deviceState = DEVICE_STATE_SLEEP;
        break;
      }
    case DEVICE_STATE_SLEEP:
      {
        LoRaWAN.sleep(loraWanClass);
        break;
      }
    default:
      {
        deviceState = DEVICE_STATE_INIT;
        break;
      }
  }
}

// void printVariables() {
//   lpp.reset();

//   int humidity = random(0, 300);
//   Serial.print(F(",humidity="));
//   Serial.print(humidity, 1);
//   lpp.addRelativeHumidity(3, humidity);

//   int temp = random(0, 200);
//   Serial.print(F(",tempf="));
//   Serial.print(temp, 1);
//   lpp.addTemperature(4, temp);

//   int pressure = random(0, 2000);
//   Serial.print(F(",pressure="));
//   Serial.print((pressure / 100.0), 2);
//   lpp.addBarometricPressure(7, (pressure / 100.0));

//   int batt_lvl = random(0, 3.3);
//   Serial.print(F(",batt_lvl="));
//   Serial.print(batt_lvl, 2);
//   lpp.addAnalogInput(8, batt_lvl);
// }
