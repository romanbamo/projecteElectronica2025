#include <Arduino.h>
#include <arduinoFFT.h>
#include <string>
#include <queue>
#include <SPI.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

using namespace std;

#define SERVICE_UUID        "4cbd1aa2-2e2b-4a51-9be0-a577dcf27eec"
#define CHARACTERISTIC_UUID "7a6ffc80-ef27-4a4d-a8a6-56d93f8feff3"
#define BUFFER_SIZE 50
#define RR_BUFFER_SIZE 1024
#define SAMPLING_RATE 500
#define MAX_CHUNK_SIZE 20
#define LLINDAR 5872025 //70% d'escala

//Definició de pins
const int START = 33;
const int RESET = 27;
const int DRDY = 21;
const int GPIO1 = 15;
const int GPIO2 = 4;
const int MISO_PIN = 19;
const int MOSI_PIN = 23;
const int SCLK = 18;
const int CS = 32;
const int TX_PIN = 17;
const int RX_PIN = 16;

volatile bool dataReadyFlag = false; 

queue<uint8_t> buffer;
queue<int32_t> rrBuffer;

int32_t ecgBuffer[BUFFER_SIZE];
int32_t respBuffer[BUFFER_SIZE];
int ecgIndex = 0;
int respIndex = 0;

struct Control{
    uint8_t address;
    uint8_t config;
};


unsigned long lastPeriod = micros();

float ecg;
float resp;

bool bufferPle = false;
bool comunicacioSPI_OK = false;
bool notif_enviada = false;

float potenciaLF = 0.0;
float potenciaHF = 0.0;
float stress = 0.0;

//////////////////////////////////////////////////////////////////////
uint16_t samplingTime = (uint16_t)((1.0 / SAMPLING_RATE) * 1000000);  // temps de mostreig en milisegons
//unsigned long samplingTime =2000; //microsegons
uint32_t nsamples = 0; // compta el numero de mostres


unsigned long last_sns_time = 0;

String data = ""; // string que conté les BUFFER_SIZE mostres d'ECG, respiració, SNS, PNS ibuffer SS, separats per comes i amb 2 decimals cadascú



// Temps i càlculs de les mostres
float temps = 0.0;
float tempsResp=0.0;
int fs = 250;
float dt = 1.0 / fs;


void guardarECG(int32_t mostra) {
    ecgBuffer[ecgIndex] = mostra;
    ecgIndex = (ecgIndex + 1);
    if (ecgIndex >= RR_BUFFER_SIZE) {
        ecgIndex = 0;
        bufferPle = true;
    }
}


// void guardarResp(int32_t mostra) {
//     respBuffer[respIndex] = mostra;
//     respIndex = (respIndex + 1) % BUFFER_SIZE;
// }

// Llegim 9 bytes per SPI i els posem a la cua
void readSPIData() {
    digitalWrite(CS, LOW);
    for (int i = 0; i < 9; i++) {
        uint8_t byte = SPI.transfer(0x00);
        buffer.push(byte);
    }
    digitalWrite(CS, HIGH);
}

// Processar blocs de 3 bytes (24 bits)
pair<float, float> processSamples() {
    int32_t ecgData = 0;
    int32_t respData = 0;
    while (buffer.size() >= 9) {
        // STATUS 3 bytes
        uint32_t status = 0;
        status |= ((uint32_t)buffer.front()) << 16; buffer.pop();
        status |= ((uint32_t)buffer.front()) << 8;  buffer.pop();
        status |= ((uint32_t)buffer.front());       buffer.pop();

        // CH1 (RESP) - 24 bits
        uint32_t ch1 = 0;
        ch1 |= ((uint32_t)buffer.front()) << 16; buffer.pop();
        ch1 |= ((uint32_t)buffer.front()) << 8;  buffer.pop();
        ch1 |= ((uint32_t)buffer.front());       buffer.pop();

        // CH2 (ECG) - 24 bits
        uint32_t ch2 = 0;
        ch2 |= ((uint32_t)buffer.front()) << 16; buffer.pop();
        ch2 |= ((uint32_t)buffer.front()) << 8;  buffer.pop();
        ch2 |= ((uint32_t)buffer.front());       buffer.pop();

        // Convertir a enter amb signe
        if (ch1 & 0x800000) ch1 |= 0xFF000000;
        if (ch2 & 0x800000) ch2 |= 0xFF000000;

        ecgData = (int32_t)ch2;
        respData = (int32_t)ch1;
    }
    return {(float)ecgData, (float)respData};
}


byte readRegister(byte regAddress) {
    byte value;
  
    digitalWrite(CS, LOW);
    SPI.transfer(0x20 | regAddress);
    SPI.transfer(0x00);
  
    delayMicroseconds(3);// Pausa segons datasheet
    value = SPI.transfer(0x00);
  
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

vector<int> detectarPicsR(const int32_t* rrVector, int mida, int32_t llindar) {
    vector<int> indexPicsR;
    for (int i = 1; i < mida - 1; ++i) {
        if (rrVector[i] > llindar && rrVector[i] > rrVector[i-1] && rrVector[i] > rrVector[i+1]) {
            indexPicsR.push_back(i);
        }
    }
    return indexPicsR;
}


vector<float> interpolarRR(const vector<float>& rrIntervals, float fsInterpolat = 4.0) {
    vector<float> tempsRR;
    float temps = 0;
    for (float val : rrIntervals) {
        temps += val;
        tempsRR.push_back(temps);
    }

    vector<float> tempsUniforme;
    float t = 0;
    while (t <= tempsRR.back()) {
        tempsUniforme.push_back(t);
        t += 1.0 / fsInterpolat;
    }

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

    return rrInterpolat;
}

vector<float> calcularFFT(const vector<float>& senyal, float fsInterpolat = 0.4) {
    int N = senyal.size();
    double vReal[N];
    double vImag[N];
    //int samples = samplingFrequency * fs;

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


void processarBuffer() {
    //Detectar pics R
    vector<int32_t> rrVector;
    queue<int32_t> tempQueue = rrBuffer;  // Copia temporal para no perder datos

    while (!tempQueue.empty()) {
        rrVector.push_back(tempQueue.front());
        tempQueue.pop();
    }

    vector<int> rPeaks = detectarPicsR(rrVector.data(), rrVector.size(), LLINDAR);

    //Calcular intervals R-R
    vector<float> rrIntervals;
    for (int i = 1; i < rPeaks.size(); ++i) {
        float rr = (rPeaks[i] - rPeaks[i-1]) * dt;
        rrIntervals.push_back(rr);
    }

    //Interpolar a 4 Hz
    vector<float> rrInterpolat = interpolarRR(rrIntervals);

    //FFT
    vector<float> magnitudsFFT = calcularFFT(rrInterpolat, fs);

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

//Control de dades llestes
void IRAM_ATTR dataReadyISR() {
    dataReadyFlag = true;
  }

// void sendLargeData(BLECharacteristic *pTxCharacteristic, const String& data) {
//   int dataLength = data.length();
  
//   // Enviar les dades per fragments petits
//   for (int i = 0; i < dataLength; i += MAX_CHUNK_SIZE) {
//       String chunk = data.substring(i, i + MAX_CHUNK_SIZE);
//       pTxCharacteristic->setValue(chunk.c_str());
//       pTxCharacteristic->notify();  // O enviar l'escriptura
//       delay(20);  // Afegir un petit retard per evitar bloquejos
//   }
// }

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
    //Configuració de pins
    pinMode(START, OUTPUT);
    pinMode(RESET, OUTPUT);
    pinMode(DRDY, INPUT_PULLUP);//DRDY es actiu baix
    attachInterrupt(digitalPinToInterrupt(DRDY), dataReadyISR, FALLING);
    pinMode(GPIO1, INPUT);
    pinMode(GPIO2, OUTPUT);

    digitalWrite(START, LOW);
    digitalWrite(RESET, LOW);
    digitalWrite(GPIO2, LOW);

    
    

    //Comandes de sistema SPI
    Control start = {0x08, 0b00001000};//No l'utilitzem per que ho fem per Hardware
    Control stop = {0x0A, 0b00001010};//No l'utilitzem per que ho fem per Hardware
    Control reset = {0x06, 0b00000110};//No l'utilitzem per que ho fem per Hardware
    Control wakeup = {0x02, 0b00000010};
    Control rDataC = {0x10, 0b00010000}; 
    Control sDataC = {0x11, 0b00010001};
    //Inicialització de variables de configuració
    Control id = { 0x00, 0b01110011 };//OK
    Control config1 = { 0x01, 0b00000001};//Converió continua, 250SPS
    Control config2 = { 0x02, 0b10100000};//LeadOff deshabilitat, Habilitem referència interna a 2.42V, deshabilitem sortida de rellotge i tests
    Control loff = {0x03, 0b00010000};//Threshold 95%, 6nA corrent de lead-off, detecció DC 
    Control ch1set = {0x04, 0b11010001};//Guany 8, Input shorted
    Control ch2set = {0x05, 0b01100000};//Guany 12, Normal Input
    Control rldSens = {0x06, 0b00000000};//ChopFrequency fMod/16, RLD leadOff deshabilitat, RLD no conectat a cap entrada.
    Control loffSens = {0x07, 0b00000000};//LeadOff deshabilitat
    Control loffStat = {0x08, 0b00000000};//Ignorat per que LOFFSENS no s'utilitza
    Control resp1 = {0x09, 0b11000010};//Modulació i Demodulació activa amb 0 graus de desfase i rellotge intern
    Control resp2 = {0x0A, 0b00000011};//Freq 32kHz i RLDREF interna.
    Control gpio = {0x0B, 0b00000100};//GPIO2 output, GPIO1 input


    // Inicialització de la comunicació sèrie
    Serial.begin(115200);
    SPI.begin(SCLK, MISO_PIN, MOSI_PIN, CS); // SCK, MISO, MOSI, CS
    pinMode(CS, OUTPUT);
    digitalWrite(CS, HIGH);
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, 1));//Mode comunicació 1 (CPOL=0, CPHA=1)

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
        
    //Configuració de l'ADS1292R
    //writeRegister(reset.address, reset.config);//Reset abans d'escriures res
    writeRegister(wakeup.address, wakeup.config);
    delayMicroseconds(8);//4 tCLK per activar WakeUp
    digitalWrite(RESET, HIGH);
    delay(1000);//Espera d'un segon segons datasheet
    digitalWrite(RESET, LOW);
    delayMicroseconds(36);//18 tclk segons datasheet
    writeRegister(sDataC.address, sDataC.config);//Mode DATAC deshabilitat per poder configurar el dispositiu
    delayMicroseconds(3);
    byte idADS = readRegister(id.address);
    delayMicroseconds(3);
    if(idADS == id.config){
      comunicacioSPI_OK = true;
    }
    else {
        comunicacioSPI_OK = false;
    }
  
    writeRegister(config2.address, config2.config);
    delayMicroseconds(3);
    writeRegister(config1.address, config1.config);
    delayMicroseconds(3);
    writeRegister(loff.address, loff.config);
    delayMicroseconds(3);
    writeRegister(ch1set.address, ch1set.config);
    delayMicroseconds(3);
    writeRegister(ch2set.address, ch2set.config);
    delayMicroseconds(3);
    writeRegister(rldSens.address, rldSens.config);
    delayMicroseconds(3);
    writeRegister(loffSens.address, loffSens.config);
    delayMicroseconds(3);
    writeRegister(loffStat.address, loffStat.config);
    delayMicroseconds(3);
    writeRegister(resp1.address, resp1.config);
    delayMicroseconds(3);
    writeRegister(resp2.address, resp2.config);
    delayMicroseconds(3);
    writeRegister(gpio.address, gpio.config);
    delayMicroseconds(3);

    //Inicialitzem l'ADS1292R
    
    //writeRegister(start.address, start.config);//Inicialitzem conversions
    digitalWrite(START, HIGH);
    writeRegister(rDataC.address, rDataC.config);//Habilitaem mode lectura continua
    delay(10000);
}


void loop() {

  if (deviceConnected && comunicacioSPI_OK && !notif_enviada) {
    String notificacio = "¿SPI establerta";
    pTxCharacteristic->setValue(notificacio.c_str());
    pTxCharacteristic->notify();
    notif_enviada = true; // Per evitar enviar-ho més d'un cop
  }else{
    if(deviceConnected && !comunicacioSPI_OK && !notif_enviada){
      String notificacio = "¿SPI NO establerta";
      pTxCharacteristic->setValue(notificacio.c_str());
      pTxCharacteristic->notify();
      notif_enviada = true;
    }

  }


  unsigned long presentPeriod = micros();
  if ((presentPeriod-lastPeriod) >= samplingTime){
 
    lastPeriod = presentPeriod;
    
    if(dataReadyFlag) {
      dataReadyFlag = false;
      readSPIData();
    }

    auto biosenyals = processSamples(); // Processar les 3 mostres de 24 bits
    ecg = biosenyals.first;
    resp = biosenyals.second;
    guardarECG(ecg);
    //guardarResp(resp);
    data += String(convertToMillivolts(ecg), 2) + ",";
    data += String(convertToMillivolts(resp), 2) + ",";
    delayMicroseconds(10);//9 mostres amb una freq de 250SPS

    if(bufferPle){
      bufferPle=false;

      for (int i = 0; i < BUFFER_SIZE; ++i) {
        rrBuffer.push(ecgBuffer[i]);
      }

      if (rrBuffer.size()>= RR_BUFFER_SIZE){
        for (int i = 0; i < BUFFER_SIZE && !rrBuffer.empty(); ++i) {
          rrBuffer.pop();
        }
      }
    }
    // Incrementar el temps per al següent mostreig
    // temps += dt;
    // tempsResp +=dt;
 
  // Cada 1.5 minuts (cada ~90000 ms), afegeix valors SNS, PNS i estrès

    if (millis() - last_sns_time > 90000) {
      processarBuffer();
      last_sns_time = millis();
    } else {
      data += String(potenciaLF, 1) + "," + String(potenciaHF, 1) + "," + String(stress, 1) + ";";;
    }
    nsamples++;
  }

  if (nsamples >= 50){
    if (deviceConnected){
      sendLargeData(pTxCharacteristic,data);
      // pTxCharacteristic->setValue(data.c_str());
      // pTxCharacteristic->notify();
      data = "";
    }
    
    if(!deviceConnected && oldDeviceConnected){
      delay(500);
      pServer -> startAdvertising();
      oldDeviceConnected = deviceConnected;
    }

    if (deviceConnected && !oldDeviceConnected){
      oldDeviceConnected = deviceConnected;
    }
    
    nsamples = 0;
    
  }
  
}

////////////////////////////////////////////////////////////