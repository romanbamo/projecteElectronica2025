#include <Arduino.h>
#include <arduinoFFT.h>
#include <SPI.h>
#include <string.h>
#include <queue>
#include <BLEServer.h>
#include <BLEDevice.h>
#include <BLE2902.h>

using namespace std;

#define BUFFER_SIZE 5000
#define LLINDAR 5872025//70%
#define SAMPLING_RATE 250
#define SERVICE_UUID        "4cbd1aa2-2e2b-4a51-9be0-a577dcf27eec"
#define CHARACTERISTIC_UUID "7a6ffc80-ef27-4a4d-a8a6-56d93f8feff3"

const int START = 33;
const int RESET = 27;
const int DRDY = 21;
const int GPIO1 = 15;
const int GPIO2 = 4;
const int SPI_SCK = 18;
const int SPI_MISO = 19;
const int SPI_MOSI = 23;
const int CS = 32;
const int TX_PIN = 17;
const int RX_PIN = 16;

volatile bool dataReadyFlag = false;

bool bufferPle = false;

int ecgIndex = 0;
int fsInterpolat = 4;
int nsamples = 0;

float ecg;
float resp;
float potenciaLF = 0.0;
float potenciaHF = 0.0;
float stress = 0.0;
float dt = 1.0 / SAMPLING_RATE;
float mostraAnterior;
float respAnterior;

unsigned long lastPeriod;
unsigned long last_sns_time = 0;

float ecgBuffer[BUFFER_SIZE];

float samplingTime = (float)((1.0 / SAMPLING_RATE) * 1000);  // temps de mostreig en milisegons

queue<uint8_t> buffer;

String data = "";

struct Control{
  uint8_t address;
  uint8_t config;
};

byte readRegister(byte regAddress) {
  byte value;

  digitalWrite(CS, LOW);
  SPI.transfer(0x20 | regAddress);
  SPI.transfer(0x00);

  value = SPI.transfer(0x00);
  delay(500);
  digitalWrite(CS, HIGH);

  return value;
}

void writeRegister(byte regAddress, byte value) {
  digitalWrite(CS, LOW);
  SPI.transfer(0x40 | regAddress);
  SPI.transfer(0x00);
  SPI.transfer(value);
  digitalWrite(CS, HIGH);
}

void sendCommand(uint8_t cmd) {
  digitalWrite(CS, LOW);
  SPI.transfer(cmd);
  digitalWrite(CS, HIGH);
  delayMicroseconds(3);
}

// Llegim 9 bytes per SPI i els posem a la cua
void readSPIData() {
    digitalWrite(CS, LOW);
    for (int i = 0; i < 9; i++) {
        uint8_t byte = SPI.transfer(0x00);
        buffer.push(byte);
    }
    digitalWrite(CS, HIGH);
}

void IRAM_ATTR dataReadyISR() {
  dataReadyFlag = true;
}

void guardarECG(float mostra) { 
  ecgBuffer[ecgIndex] = mostra;
  ecgIndex = (ecgIndex + 1);
  if (ecgIndex >= BUFFER_SIZE) {
      ecgIndex = 0;
      bufferPle = true;
  }
}
pair<float, float> processSamples() {
  int32_t ecgData = 0;
  int32_t respData = 0;
  Serial.println("Dintre de processSample");
  Serial.println("Mida buffer");
  Serial.println(buffer.size());
  while (buffer.size() >= 9) {
      Serial.println("Dintre del while processSample");
      Serial.println("Mida buffer");
      Serial.println(buffer.size());
      // STATUS 3 bytes
      uint32_t status = 0;
      for (int i = 0; i < 3; ++i) {
          Serial.println("Mida buffer 1");
          Serial.println(buffer.size());
          status <<= 8;
          status |= buffer.front(); buffer.pop();
      }

      // CH1 (RESP) - 24 bits
      uint32_t ch1 = 0;
      for (int i = 0; i < 3; ++i) {
          Serial.println("Mida buffer 2");
          Serial.println(buffer.size());
          ch1 <<= 8;
          ch1 |= buffer.front(); buffer.pop();
      }

      // CH2 (ECG) - 24 bits
      uint32_t ch2 = 0;
      for (int i = 0; i < 3; ++i) {
          Serial.println("Mida buffer 3");
          Serial.println(buffer.size());
          ch2 <<= 8;
          ch2 |= buffer.front(); buffer.pop();
      }

      // Convertir a enter amb signe
      if (ch1 & 0x800000) ch1 |= 0xFF000000;
      if (ch2 & 0x800000) ch2 |= 0xFF000000;

      ecgData = (int32_t)ch2;
      respData = (int32_t)ch1;
  }
  return {(float)ecgData, (float)respData};
}

vector<int> detectarPicsR(const float* rrVector, int mida, int llindar) {
  Serial.println("Dintre de detectarPicsR");
  vector<int> indexPicsR;
  for (int i = 1; i < mida - 1; ++i) {
      if (rrVector[i] > llindar && rrVector[i] > rrVector[i-1] && rrVector[i] > rrVector[i+1]) {
          indexPicsR.push_back(i);
      }
  }
  return indexPicsR;
}

vector<float> interpolarRR(const vector<float>& rrIntervals, float fsInterpolat = 4.0) {
  Serial.println("Dintre de interpolarRR");
  vector<float> tempsRR;
  float temps = 0;
  for (float val : rrIntervals) {
      temps += val;
      tempsRR.push_back(temps);
  }
  //////////
  Serial.println("Temps RR calculat: ");
  for (float tt : tempsRR) {
    Serial.println(tt);
  }
  /////////////
  vector<float> tempsUniforme;
  float t = 0;
  while (t <= tempsRR.back()) {
      tempsUniforme.push_back(t);
      t += 1.0 / fsInterpolat;
  }
  ///////////////
  Serial.println("Temps Uniforme calculat: ");
  for (float tu : tempsUniforme) {
    Serial.println(tu);
  }
  /////////////
  vector<float> rrInterpolat;
  int j = 0;
  for (float tu : tempsUniforme) {
      while (j < tempsRR.size() - 1 && tempsRR[j+1] < tu) j++;
      if (j >= tempsRR.size() - 1) break;
      float y0 = rrIntervals[j];
      float y1 = rrIntervals[j + 1];
      float x0 = tempsRR[j];
      float x1 = tempsRR[j + 1];
      float y = y0 + ((y1 - y0) / (x1 - x0)) * (tu - x0);
      rrInterpolat.push_back(y);
  }
  /////////////////
  Serial.println("RR Interpolat calculat: ");
  for (float rr : rrInterpolat) {
    Serial.println(rr);
  }
  ////////////////
  return rrInterpolat;
}

vector<float> calcularFFT(const vector<float>& senyal, float fsInterpolat = 0.4) {
  Serial.println("Dintre de calcularFFT");
  int N = senyal.size();
  double vReal[N];
  double vImag[N];
  for (int i = 0; i < N; ++i) {
      vReal[i] = senyal[i];
      vImag[i] = 0;
  }
  ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, N, fsInterpolat);
  FFT.windowing(vReal, N, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.compute(vReal, vImag, N, FFT_FORWARD);
  FFT.complexToMagnitude(vReal, vImag, N);
  vector<float> magnitud(vReal, vReal + N/2);
  return magnitud;
}

pair<float, float> calcularPotenciesLF_HF(const vector<float>& magnitud, float fsInterpolat = 0.4) {
  Serial.println("Dintre de calcularPotenciesLF_HF");
  int N = magnitud.size() * 2;
  float resolucioFreq = fsInterpolat / N;
  int idxLFini = int(0.04 / resolucioFreq);
  int idxLFend = int(0.15 / resolucioFreq);
  int idxHFini = int(0.15 / resolucioFreq);
  int idxHFend = int(0.4 / resolucioFreq);
  float potLF = 0.0;
  float potHF = 0.0;
  for (int i = idxLFini; i <= idxLFend; ++i) {
      potLF += magnitud[i] * magnitud[i];
  }
  for (int i = idxHFini; i <= idxHFend; ++i) {
      potHF += magnitud[i] * magnitud[i];
  }
  return {potLF, potHF};
}

void resetEcgBuffer(){
  for(int i = 0; i < BUFFER_SIZE; i++){
    ecgBuffer[i]= 0;
  }
  ecgIndex = 0;
}

void processarBuffer() {
  //Detectar pics R
  /////////////////
  Serial.println("Dintre de processarBuffer");
  Serial.println("Mostra ECGbuffer: ");
  for (int i = 0; i < ecgIndex; i++) {
    Serial.println(ecgBuffer[i]);
  }
  //////////////
  vector<int> rPeaks = detectarPicsR(ecgBuffer, ecgIndex, LLINDAR);
  /////////////
  Serial.println("Pics calculats: ");
  for (float peak : rPeaks) {
    Serial.println(peak);
  }
  ////////////
  resetEcgBuffer();
  Serial.println("Pics detectats i ecgBuffer resetejat");
  Serial.println("EcgIndex: " + String(ecgIndex));
  //Calcular intervals R-R
  vector<float> rrIntervals;
  for (int i = 1; i < rPeaks.size(); ++i) {
      float rr = (rPeaks[i] - rPeaks[i-1]) * dt;
      rrIntervals.push_back(rr);
  }
  Serial.println("rrIntervals calculats:");
  Serial.println(rrIntervals.size());
  for (float rr : rrIntervals) {
    Serial.println(rr);
  }
  //Interpolar a 4 Hz
  vector<float> rrInterpolat = interpolarRR(rrIntervals);
  Serial.println("rrInterpolat calculats:");
  Serial.println(rrInterpolat.size());
  for (float rrI : rrInterpolat) {
    Serial.println(rrI);
  }
  //FFT
  vector<float> magnitudsFFT = calcularFFT(rrInterpolat, fsInterpolat);
  Serial.println("magnitudsFFT calculats:");
  Serial.println(magnitudsFFT.size());
  for (float FFt : magnitudsFFT) {
    Serial.println(FFt);
  }
  //Separar bandes
  auto potencies = calcularPotenciesLF_HF(magnitudsFFT);
  potenciaLF = potencies.first;
  potenciaHF = potencies.second;
  stress = potenciaLF / potenciaHF;
  // Guardar resultats
  data += String(potenciaLF, 1) + "," + String(potenciaHF, 1) + "," + String(stress, 1) + ";";
}

float convertToMillivolts(long rawValue, float vRef = 2.42, int gain = 6) {
  const float fullScale = 8388607.0; // 2^23 - 1
  float voltage = (rawValue / fullScale) * (vRef / gain); // en Volts
  return voltage * 1000.0; // en mV
  }

void sendLargeData(BLECharacteristic *pTxCharacteristic, const String& data) {
  int start = 0;
  int dataLength = data.length();
  while (start < dataLength) {
    int semicolonIndex = data.indexOf(';', start);
    if (semicolonIndex == -1) break;
    String chunk = data.substring(start, semicolonIndex);
    pTxCharacteristic->setValue(chunk.c_str());
    pTxCharacteristic->notify();
    delay(20);
    start = semicolonIndex + 1;
  }
}

//paràmetres BLE
bool deviceConnected = false;
bool oldDeviceConnected = false;
BLEServer *pServer = NULL;
BLECharacteristic *pTxCharacteristic;

class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer){
    deviceConnected = true;
  };
  void onDisconnect(BLEServer* pServer){
    deviceConnected = false;
  }
};


void setup() {

  delay(5000);

  pinMode(START, OUTPUT);
  pinMode(RESET, OUTPUT);
  pinMode(DRDY, INPUT_PULLUP);//DRDY es actiu baix
  attachInterrupt(digitalPinToInterrupt(DRDY), dataReadyISR, FALLING);
  pinMode(GPIO1, INPUT);
  pinMode(GPIO2, OUTPUT);

  digitalWrite(START, LOW);
  digitalWrite(RESET, HIGH);
  digitalWrite(GPIO2, LOW);

  Control wakeup = {0x02, 0b00000010};
  Control rDataC = {0x10, 0b00010000}; 
  Control sDataC = {0x11, 0b00010001};
  //Inicialització de variables de configuració
  Control id = { 0x00, 0b01110011 };//OK
///////////Versio 2/////////////////
  Control config1 = { 0x01, 0b00000001};//Converió continua, 250SPS
  Control config2 = { 0x02, 0b10100000};//LeadOff deshabilitat, Habilitem referència interna a 2.42V, deshabilitem sortida de rellotge i tests
  Control loff = {0x03, 0b00010000};//Threshold 95%, 6nA corrent de lead-off, detecció DC 
  Control ch1set = {0x04, 0b00000000};//Guany 6, Normal electrode input
  Control ch2set = {0x05, 0b00000000};//Guany 6, Normal electrode input
  Control rldSens = {0x06, 0b01101100};//ChopFrequency fMod/4, RLD leadOff deshabilitat, RLD no conectat a cap entrada.
  Control loffSens = {0x07, 0b00000000};//LeadOff deshabilitat
  Control loffStat = {0x08, 0b00000000};//Ignorat per que LOFFSENS no s'utilitza
  Control resp1 = {0x09, 0b11010010};//Modulació i Demodulació activa amb 45 graus de desfase i rellotge intern
  Control resp2 = {0x0A, 0b00000011};//Freq 32kHz i RLDREF interna.
  Control gpio = {0x0B, 0b00000100};//GPIO2 output, GPIO1 input

////////////////////////////////////////
  Serial.begin(115200);
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, CS);
  pinMode(CS, OUTPUT);
  digitalWrite(CS, HIGH);
  SPI.beginTransaction(SPISettings(300000, MSBFIRST, SPI_MODE1));

  //Inicialització de comunicació bluetooth    
  BLEDevice::init("ESP32-ROMA-BARRERA");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pTxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID,
                          BLECharacteristic::PROPERTY_READ   |
                          BLECharacteristic::PROPERTY_WRITE  |
                          BLECharacteristic::PROPERTY_NOTIFY);

  pTxCharacteristic->addDescriptor(new BLE2902());
  pService->start();

  //Comença a fer advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);
  BLEDevice::startAdvertising();
  
  // Reset hardware
  digitalWrite(RESET, LOW);
  delay(1000);
  digitalWrite(RESET, HIGH);
  delay(1000);
  
  // Enviar WAKEUP i SDATAC
  sendCommand(wakeup.config);
  delayMicroseconds(8);
  sendCommand(sDataC.config);
  delayMicroseconds(3);
  
  delay(100);

  ////////////////////Versio 2//////////////////
  writeRegister(config2.address, config2.config);
  Serial.println("Config2");
  delayMicroseconds(3);
  writeRegister(config1.address, config1.config);
  Serial.println("Config1");
  delayMicroseconds(3);
  writeRegister(loff.address, loff.config);
  Serial.println("Loff");
  delayMicroseconds(3);
  writeRegister(ch1set.address, ch1set.config);
  Serial.println("CH1");
  delayMicroseconds(3);
  writeRegister(ch2set.address, ch2set.config);
  Serial.println("CH2");
  delayMicroseconds(3);
  writeRegister(rldSens.address, rldSens.config);
  Serial.println("RLD");
  delayMicroseconds(3);
  writeRegister(loffSens.address, loffSens.config);
  Serial.println("LoffSens");
  delayMicroseconds(3);
  writeRegister(loffStat.address, loffStat.config);
  Serial.println("LoffStat");
  delayMicroseconds(3);
  writeRegister(resp1.address, resp1.config);
  Serial.println("Resp1");
  delayMicroseconds(3);
  writeRegister(resp2.address, resp2.config);
  Serial.println("Resp2");
  delayMicroseconds(3);
  writeRegister(gpio.address, gpio.config);
  Serial.println("GPIO");
  delayMicroseconds(3);
  /////////////////////////////////////////////
  
  digitalWrite(START, HIGH);

  byte idADS = readRegister(0x00);
  Serial.print("ID llegida: 0x");
  Serial.println(idADS, HEX);
  
  if(idADS == id.config){
    Serial.println(" ID correcta");
  } else {
    Serial.println(" ID incorrecta o xip no respon");
    Serial.println(idADS);
  }

  delay(1000);

  sendCommand(rDataC.config);
  delayMicroseconds(3);

  lastPeriod = micros();

  delay(500);
  
}

void loop() {

  unsigned long presentPeriod = micros();
  if ((presentPeriod-lastPeriod) >= samplingTime){
    lastPeriod = presentPeriod;
    if(dataReadyFlag) {
      dataReadyFlag = false;
      readSPIData();
    }

    Serial.println(buffer.size());


    auto biosenyals = processSamples(); // Processar les 3 mostres de 24 bits
    ecg = biosenyals.first;
    resp = biosenyals.second;

    resp = convertToMillivolts(resp)+20;

    if(abs(resp)<40) respAnterior = resp;

    if(abs(respAnterior-resp)>20) resp = respAnterior;

    //if (abs(ecg) > 100000) ecg = mostraAnterior;

    //if (abs(ecg) < 100000) mostraAnterior = ecg;

    Serial.println("ECG:");
    Serial.println(ecg);
    Serial.println("Resp:");
    Serial.println(resp);
    data += String((convertToMillivolts(ecg)-14), 2) + ",";
    data += String((resp), 2) + ",";
    guardarECG(ecg);
    //Cambiar condició de buffer ple per haber passat 2 minuts i fer la mida del buffer molt més gran
    if (millis() - last_sns_time > 120000) {
      Serial.println("Han passat 1'5s");
      Serial.println("N mostres ecg registrades:");
      Serial.println(ecgIndex);
      processarBuffer();
      last_sns_time = millis();

    } else {
      data += String(potenciaLF, 1) + "," + String(potenciaHF, 1) + "," + String(stress, 1) + ";";;
    }
    nsamples++;
    Serial.println(data);
  }

  if (nsamples >= 10){
    if (deviceConnected){
      sendLargeData(pTxCharacteristic,data);
      data = "";
    }
    if(!deviceConnected && oldDeviceConnected){
      delay(100);
      pServer -> startAdvertising();
      oldDeviceConnected = deviceConnected;
    }
    if (deviceConnected && !oldDeviceConnected){
      oldDeviceConnected = deviceConnected;
    }
    nsamples = 0;
  }

  //delay(10);
}
