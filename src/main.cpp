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
#define SAMPLING_RATE 250

#define MAX_CHUNK_SIZE 20

//Definició de pins
const int START = 33;
const int RESET = 27;
const int DRDY =21;
const int GPIO1 = 15;
const int GPIO2 = 4;
const int MISO = 19;
const int MOSI = 23;
const int SCLK = 18;
const int CS = 32;
const int TX = 17;
const int RX = 16;

volatile bool dataReadyFlag = false; 

queue<uint8_t> buffer;

int32_t ecgBuffer[BUFFER_SIZE];
int32_t respBuffer[BUFFER_SIZE];
int32_t rrBuffer[BUFFER_RR]
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

float potenciaLF;
float potenciaHF;
float stress;

//////////////////////////////////////////////////////////////////////
u_int16_t samplingTime =(1/SAMPLING_RATE)*1000000;  // temps de mostreig en milisegons
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
    if (ecgIndex >= BUFFER_SIZE) {
        ecgIndex = 0;
        bufferPle = true;
    }
}


void guardarResp(int32_t mostra) {
    respBuffer[respIndex] = mostra;
    respIndex = (respIndex + 1) % BUFFER_SIZE;
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

// Processar blocs de 3 bytes (24 bits)
uint32_t processSamples() {
    int32_t ecg;
    int32_t resp;
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

        int32_t ecg = (int32_t)ch2;
        int32_t resp = (int32_t)ch1;

        // Mostrar els valors
        Serial.print("ECG: "); Serial.print(ecg);
        Serial.print(" | RESP: "); Serial.println(resp);
    }
    return (float)ecg, (float)resp;
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

vector<int> detectarPicsR(const int32_t* rrBuffer, int mida, int32_t llindar) {
    vector<int> indexPicsR;
    for (int i = 1; i < mida - 1; ++i) {
        if (rrBuffer[i] > llindar && rrBuffer[i] > rrBuffer[i-1] && rrBuffer[i] > rrBuffer[i+1]) {
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

vector<float> calcularFFT(const vector<float>& senyal, float fs) {
    int N = senyal.size();
    double vReal[N];
    double vImag[N];

    for (int i = 0; i < N; ++i) {
        vReal[i] = senyal[i];
        vImag[i] = 0;
    }

    FFT.Windowing(vReal, N, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT.Compute(vReal, vImag, N, FFT_FORWARD);
    FFT.ComplexToMagnitude(vReal, vImag, N);

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
    vector<int> rPeaks = detectarPicsR(rrBuffer, RR_BUFFER_SIZE);

    //Calcular intervals R-R
    vector<float> rrIntervals;
    for (int i = 1; i < rPeaks.size(); ++i) {
        float rr = (rPeaks[i] - rPeaks[i-1]) * dt;
        rrIntervals.push_back(rr);
    }

    //Interpolar a 4 Hz
    vector<float> rrInterpolat = interpolarRR(rrIntervals);

    //FFT
    vector<float> magnitudsFFT = calculaFFT(rrInterpolat);

    //Separar bandes
    potenciaLF, potenciaHF = calcularPotencia(magnitudsFFT);

    stress = potenciaLF / potenciaHF;

    // Guardar resultats
    data += String(potenciaLF, 1) + "," + String(potenciaHF, 1) + "," + String(stress, 1) + ";";
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
    pinMode(START, OUTPUT);//No l'utilitzem per que ho fem per SPI
    pinMode(RESET, OUTPUT);//No l'utilitzem per que ho fem per SPI
    pinMode(DRDY, INPUT_PULLUP);  // Normalmente DRDY es activo bajo
    attachInterrupt(digitalPinToInterrupt(DRDY), dataReadyISR, FALLING);
    pinMode(GPIO1, INPUT);
    pinMode(GPIO2, OUTPUT);
    

    //Comandes de sistema SPI
    Control start = {0x08, 0b00001000};
    Control stop = {0x0A, 0b00001010};
    Control reset = {0x06, 0b00000110};
    Control wakeup = {0x02, 0b00000010};
    Control rDataC = {0x10, 0b00010000}; 
    Control sDataC = {0x11, 0b00010001};
    //Inicialització de variables de configuració
    Control id = { 0x00, 0b01110011 };
    Control config1 = { 0x01, 0b00000010};//Converió continua, 500SPS
    Control config2 = { 0x02, 0b10100000};//LeadOff deshabilitat, Habilitem referència interna a 2.42V, deshabilitem sortida de rellotge i tests
    Control loff = {0x03, 0b00010000};//Threshold 95%, 6nA corrent de lead-off, detecció DC 
    Control ch1set = {0x04, 0b00000000};//Guany 6, Normal electrode input
    Control ch2set = {0x05, 0b00000000};//Guany 6, Normal electrode input
    Control rldSens = {0x06, 0b11000000};//ChopFrequency fMod/4, RLD leadOff deshabilitat, RLD no conectat a cap entrada.
    Control loffSens = {0x07, 0b00000000};//LeadOff deshabilitat
    Control loffStat = {0x08, 0b00000000};//Ignorat per que LOFFSENS no s'utilitza
    Control resp1 = {0x09, 0b11000010};//Modulació i Demodulació activa amb 0 graus de desfase i rellotge intern
    Control resp2 = {0x0A, 0b00000011};//Freq 32kHz i RLDREF interna.
    Control gpio = {0x0B, 0b00000100};//GPIO2 output, GPIO1 input


    // Inicialització de la comunicació sèrie
    Serial.begin(115200);
    SPI.begin(SCLK, MISO, MOSI, CS); // SCK, MISO, MOSI, CS
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
    Serial.println("waiting a client connection to notify...");
        
    //Configuració de l'ADS1292R
    writeRegister(reset.address, reset.config);//Reset abans d'escriures res
    writeRegister(sDataC.address, sDataC.config);//Mode DATAC deshabilitat per poder configurar el dispositiu
    writeRegister(id.address, id.config);
    writeRegister(config1.address, config1.config);
    writeRegister(config2.address, config2.config);
    writeRegister(loff.address, loff.config);
    writeRegister(ch1set.address, ch1set.config);
    writeRegister(ch2set.address, ch2set.config);
    writeRegister(rldSens.address, rldSens.config);
    writeRegister(loffSens.address, loffSens.config);
    writeRegister(loffStat.address, loffStat.config);
    writeRegister(resp1.address, resp1.config);
    writeRegister(resp2.address, resp2.config);
    writeRegister(gpio.address, gpio.config);

    //Inicialitzem l'ADS1292R
    writeRegister(wakeup.address, wakeup.config);
    writeRegister(start.address, start.config);//Inicialitzem conversions
    writeRegister(rDataC.address, rDataC.config);//Habilitaem mode lectura continua
    
  
}


void loop() {


    if(dataReadyFlag) {
        dataReadyFlag = false;
        readSPIData();
    }

    ecg, resp = processSamples(); // Processar les 3 mostres de 24 bits
    guardarECG(ecg);
    guardarResp(resp);
    data += String(ecg, 2) + ",";
    data += String(resp, 2) + ",";
    delayMicroseconds(36);//9 mostres amb una freq de 250SPS

    if(bufferPle){
        bufferPle=false;
        rrBuffer += ecgBuffer;
    }






  unsigned long presentPeriod = micros();
 // Serial.print("resta: ");
  //Serial.println(presentPeriod-lastPeriod);
  if ((presentPeriod-lastPeriod) >= samplingTime){
 
    lastPeriod = presentPeriod;
    
    
    // Incrementar el temps per al següent mostreig
    temps += dt;
    tempsResp +=dt;

    nsamples++;
 
  // Cada 1.5 minuts (cada ~90000 ms), afegeix valors SNS, PNS i estrès

  if (millis() - last_sns_time > 90000) {
    processarBuffer()
    last_sns_time = millis();
  } else {
    data += "0.0,0.0,0.0;";
  }
  }

  if (nsamples >= 50){
    if (deviceConnected){
      sendLargeData(pTxCharacteristic,data);
      // pTxCharacteristic->setValue(data.c_str());
      // pTxCharacteristic->notify();
      Serial.println("Dada enviada");
      Serial.println(data);
      Serial.print("nsamples: ");
      Serial.println(nsamples);
    }
    
    if(!deviceConnected && oldDeviceConnected){
      delay(500);
      pServer -> startAdvertising();
      Serial.println("Start advertising");
      oldDeviceConnected = deviceConnected;
    }

    if (deviceConnected && !oldDeviceConnected){
      Serial.println("Imprimeixo alguna cosa");
      oldDeviceConnected = deviceConnected;
    }
    
    nsamples = 0;
    data = "";
  }
  
}
