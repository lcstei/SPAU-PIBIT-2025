#define TINY_GSM_MODEM_SIM7600
#include <Arduino.h>

struct SIM800L_DATA {
    // Definições para a rede móvel
    const char* apn = "claro.com.br";
    const char* gprsUser = "claro";
    const char* gprsPass = "claro";

    // --- Configurações MQTT ---
    // const char* mqtt_server = "191.252.102.197";
    const char* mqtt_server = "test.mosquitto.org";
    const int mqtt_port = 1883;
    const char* mqtt_client_id = "TSERT";
    // const char* mqtt_user = "adm_tester";
    // const char* mqtt_pass = "tester";
    const char* mqtt_user = "";
    const char* mqtt_pass = "";
    const char* mqtt_topic_pub = "TEST/WS";

    const int PWRKEY = 27;
    uint32_t lastTime = 0;  // Tempo em que o último envio foi feito
} SIM;

#include <PubSubClient.h>
// #include <HardwareSerial.h>
#include <TinyGsmClient.h>

#define MODEM_TX 32
#define MODEM_RX 33
#define MODEM_EN 26
// #define SIM800_MODEM_RST 35

HardwareSerial gsmSerial(1);
TinyGsm modemGSM(gsmSerial);
TinyGsmClient gsmClient(modemGSM);
PubSubClient mqttClient(gsmClient);

// SENSORS CONFIG
#define DIR_PIN 12
#define SPEED_PIN 14
#define PLV_PIN 13
#define VBAT_PIN 27

#define SD_CS 5
#define SD_SCK 18
#define SD_MISO 19
#define SD_MOSI 23

#define numDir 8

#define SAMPLE_TIME 3000  // Período de amostragem em ms
#define INTERVAL 10000

#include <RTClib.h>
RTC_DS1307 rtc;

struct SpeedSensor {
    unsigned long pulseCount = 0;
    // unsigned long sampleStart = 0;
    float metersPerPulse = (2 * PI * 0.1) / 2;  // 0.314159 m/pulso
    uint8_t lastState = HIGH;
    float speed = 0;  // Velocidade média em m/s
} spd;

// Estrutura para o sensor de frequência
struct RainSensor {
    volatile unsigned long pulseCount = 0;
    volatile unsigned long lastPulseMicros = 0;
    bool isRaining = false;
    const float cupVolume = 0.2794;  // m³ por pulso (0.2794 litros)
    const float funilArea = 0.01;    // Área do funil em m² (10 cm²)

    float getRainVolume() {
        // Calcula o volume de chuva em litros
        return (pulseCount * cupVolume) / funilArea;  // Convertendo m³ para litros
    }
} plv;

struct DirSensor {
    // O sensor precisa ser apontado para o norte quando instalado
    // a calibração é baseada na direção que o norte aponta

    int ADCValue = 0;  // Valor lido do pino analógico
    String ultima_pos = "";

    const float tolerancia = 0.05;  // ±5%

    // Nomes dos pontos cardeais
    const String pontos[numDir] = {"N", "NE", "E", "SE", "S", "SW", "W", "NW"};

    // Valores esperados para cada ponto
    float valoresEsperados[numDir] = {
        2970,  // Valor ADC para N
        1700,  // Valor ADC para NE
        240,   // Valor ADC para E
        600,   // Valor ADC para SE
        1000,  // Valor ADC para S
        2360,  // Valor ADC para SW
        3900,  // Valor ADC para W
        3500   // Valor ADC para NW
    };

    String lerDirecao() {
        int leitura = analogRead(DIR_PIN);

        for (int i = 0; i < numDir; i++) {
            float margem = valoresEsperados[i] * tolerancia;
            float limiteInf = valoresEsperados[i] - margem;
            float limiteSup = valoresEsperados[i] + margem;

            if (leitura >= limiteInf && leitura <= limiteSup) {
                ultima_pos = pontos[i];
            }
        }

        return ultima_pos;
    }

} dir;

struct ws_data_t {
    float latitude;
    float longitude;
} ws_data;

void IRAM_ATTR pulseISR() {
    plv.pulseCount++;
    plv.lastPulseMicros = micros();
    plv.isRaining = true;  // Define que está chovendo quando um pulso é detectado
};

uint64_t UpdateInterval =
    0.3 * 60 * 1000000;       // e.g.(0.33(min) * 60(s)) * 1000000(us) --> Sleep time
float BATTERY_VOLTAGE = 0.0;  // Variável para armazenar a tensão da bateria

uint64_t SampleTime = 0;  // Variável para armazenar o tempo passado

// SD Card config
#include <SPI.h>
#include <SdFat.h>

SdFs SD;
FsFile dataFile;

#define FILE_WRITE (O_RDWR | O_CREAT | O_AT_END)
#define FILE_APPEND FAPPEND
#define SD_CS_PIN 5
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, DEDICATED_SPI, SD_SCK_MHZ(6))

// Declaração de funções
void setupGSM();
void connectMQTTServer();
void publishMQTT();
String createJsonString();
void printData();
bool INIT_SD();
void INIT_RTC();
void WriteToCSV();
void Deep_Sleep_Now();
void getBattery();
void Calibrate_RTC();
void SetupGPS();
float convertToDecimal(String, String);
void readGPS();

void setup() {
    Serial.begin(115200);
    gsmSerial.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX, false);
    if (!gsmSerial) {  // If the object did not initialize, then its configuration is invalid
        Serial.println("Invalid EspSoftwareSerial pin configuration, check config");
        while (1) {  // Don't continue with invalid configuration
            delay(1000);
        }
    }
    delay(3000);  // Aguarda o módulo iniciar

    pinMode(SPEED_PIN, INPUT_PULLDOWN);
    pinMode(VBAT_PIN, INPUT);
    pinMode(DIR_PIN, INPUT);
    pinMode(PLV_PIN, INPUT_PULLUP);  // Configura o pino do sensor de chuva como entrada com pull-up
    pinMode(LED_BUILTIN, OUTPUT);    // Configura o LED como saída

    pinMode(MODEM_EN, OUTPUT);
    digitalWrite(MODEM_EN, HIGH);  // ATIVA o módulo GSM inicialmente

    SampleTime = millis();

    INIT_SD();
    INIT_RTC();

    attachInterrupt(digitalPinToInterrupt(PLV_PIN), pulseISR, RISING);
}

void loop() {
    // Verifica o estado atual do pino
    uint8_t currentState = digitalRead(SPEED_PIN);

    // Tempo decorrido desde o boot em milissegundos
    unsigned long now = millis();

    // Detecta borda de subida (pulso)
    if (currentState == LOW && spd.lastState == HIGH) {
        spd.pulseCount++;
    }
    spd.lastState = currentState;

    // Verifica se passaram 5 segundos

    if (millis() - SampleTime >= SAMPLE_TIME) {
        // Calcula velocidade média
        float speed = 0;
        if (spd.pulseCount > 0) {
            float distance = spd.pulseCount * spd.metersPerPulse;
            speed = distance / (SAMPLE_TIME / 1000.0);  // m/s
            spd.speed = speed;                          // Armazena a velocidade média
        }

        // Reinicia a contagem
        spd.pulseCount = 0;
        SampleTime = millis();
        getBattery();
        readGPS();     // Lê os dados do GPS
        printData();   // Imprime os dados no Serial Monitor
        WriteToCSV();  // Escreve os dados no cartão SD
        setupGSM();
        connectMQTTServer();
        publishMQTT();  // Publica os dados no servidor MQTT
    }
}

void setupGSM() {
    Serial.println("\n--- SETUO GSM ---");

    Serial.println("[GSM] Tentando inicializar módulo A7670SA com TinyGSM...");
    delay(3000);
    // Inicializa o modemGSM com um timeout maior
    if (!modemGSM.init()) {
        Serial.println(
            "[GSM] Falha ao inicializar o modemGSM, verificando status ou reiniciando...");
        modemGSM.restart();
        if (!modemGSM.init()) {
            Serial.println("[GSM] Reinicialização falhou. Reiniciando ESP32...");
            delay(1000);
            Deep_Sleep_Now();
        }
    }
    Serial.println("[GSM] Módulo A7670SA inicializado com sucesso!");

    // --- Mostrar parâmetros do módulo ---
    Serial.println("\n--- Parâmetros do Módulo ---");

    Serial.print("[GSM] Status da Rede: ");
    Serial.println(modemGSM.isNetworkConnected() ? "Conectado" : "Desconectado");

    Serial.print("[GSM] Status GPRS: ");
    Serial.println(modemGSM.isGprsConnected() ? "Conectado" : "Desconectado");

    Serial.print("[GSM] Endereço IP Local: ");
    Serial.println(modemGSM.getLocalIP());

    Serial.println("---------------------------\n");

    // --- Conectar à rede GPRS ---
    Serial.print("[GSM] Conectando GPRS com APN: ");
    Serial.print(SIM.apn);
    if (SIM.gprsUser[0] != '\0') {
        Serial.print(", Usuário: ");
        Serial.print(SIM.gprsUser);
    }
    Serial.println("...");

    if (!modemGSM.gprsConnect(SIM.apn, SIM.gprsUser, SIM.gprsPass)) {
        Serial.println("[GSM] Falha ao conectar GPRS!");
        Deep_Sleep_Now();
    } else {
        Serial.println("[GSM] GPRS Conectado!");
        Serial.print("[GSM] IP Local: ");
        Serial.println(modemGSM.getLocalIP());
    }

    SetupGPS();
}

void connectMQTTServer() {
    // --- Configurar MQTT ---
    mqttClient.setServer(SIM.mqtt_server, SIM.mqtt_port);
    // mqttClient.setCallback(callback); // Se você quiser receber mensagens

    Serial.println("\n--- SETUP MQTT ---");
    if (!mqttClient.connected()) {
        Serial.print("[MQTT] Tentando conectar MQTT ao servidor: ");
        Serial.print(SIM.mqtt_server);
        Serial.print("...");
        if (mqttClient.connect(SIM.mqtt_client_id, SIM.mqtt_user, SIM.mqtt_pass)) {
            Serial.println("[MQTT] Conectado ao MQTT!");
            // mqttClient.subscribe(mqtt_topic_sub); // Se quiser subscrever
        } else {
            Serial.print("[MQTT] Falha ao conectar MQTT, rc=");
            Serial.println(mqttClient.state());
            Deep_Sleep_Now();
        }
    }
}

void publishMQTT() {
    // Se desconectou do server MQTT
    if (!mqttClient.connected()) {
        // Mandamos conectar
        connectMQTTServer();
    }

    // Cria o json que iremos enviar para o server MQTT
    String msg = createJsonString();

    Serial.print("[MQTT] Publish message: ");
    Serial.println(msg);

    if (mqttClient.publish(SIM.mqtt_topic_pub, msg.c_str())) {
        Serial.println("[MQTT] Mensagem MQTT enviada com sucesso!");
    } else {
        Serial.println("[MQTT] Falha ao enviar mensagem MQTT.");
    }
    Deep_Sleep_Now();
}

String createJsonString() {
    String data = "{";
    data += "\"spd\":";
    data += String(spd.speed, 2);
    data += ",";
    data += "\"dir\":";
    data += dir.lerDirecao();
    data += ",";
    data += "\"plv\":";
    data += String(plv.getRainVolume(), 2);
    data += ",";
    data += "\"vbat\":";
    data += String(BATTERY_VOLTAGE, 2);
    data += ",";
    DateTime now = rtc.now();
    data += "\"timestamp\":";
    data += "\"";
    data += String(now.year(), DEC);
    data += "-";
    data += String(now.month(), DEC);
    data += "-";
    data += String(now.day(), DEC);
    data += " ";
    data += String(now.hour(), DEC);
    data += ":";
    data += String(now.minute(), DEC);
    data += ":";
    data += String(now.second(), DEC);
    data += "\"";
    data += ",";
    data += "\"longitude\":";
    data += String(ws_data.longitude, 6);
    data += ",";
    data += "\"latitude\":";
    data += String(ws_data.longitude, 6);
    data += "}";
    return data;
}

void printData() {
    Serial.println();
    Serial.println("\n--- Dados ---");

    Serial.println();
    Serial.print("[SPEED] ");
    Serial.print(spd.speed, 2);
    Serial.println(" [m/s]");

    Serial.print("[PLV] ");
    Serial.print(plv.getRainVolume(), 2);
    Serial.println(" [mm]");

    Serial.print("[DIR] ");
    Serial.print(dir.lerDirecao());
    Serial.println(" [°]");

    Serial.print("[VBAT] ");
    Serial.print(BATTERY_VOLTAGE, 2);
    Serial.println(" [V]");

    Serial.print("[RTC] ");
    DateTime now = rtc.now();
    Serial.print(now.year(), DEC);
    Serial.print("-");
    Serial.print(now.month(), DEC);
    Serial.print("-");
    Serial.print(now.day(), DEC);
    Serial.print(" ");
    Serial.print(now.hour(), DEC);
    Serial.print(":");
    Serial.print(now.minute(), DEC);
    Serial.print(":");
    Serial.print(now.second(), DEC);
    Serial.println();

    Serial.print("[GPS] ");
    Serial.print("Lat: ");
    Serial.print(ws_data.latitude, 6);
    Serial.print(", Lon: ");
    Serial.println(ws_data.longitude, 6);
}

bool INIT_SD() {
    SPI.begin(SD_SCK, SD_MISO, SD_MOSI);
    Serial.println("[SD] Ininicialização");

    if (!SD.begin(SdSpiConfig(SD_CS, DEDICATED_SPI, SD_SCK_MHZ(6)))) {
        Serial.println("[SD] Falha na inicialização");
        return false;
    }

    Serial.println("[SD] Cartão SD inicializado com sucesso");
    Serial.print("[SD] Sistema de arquivos: ");

    switch (SD.fatType()) {
        case FAT_TYPE_EXFAT:
            Serial.println("[SD] exFAT");
            break;
        case FAT_TYPE_FAT32:
            Serial.println("[SD] FAT32");
            break;
        case FAT_TYPE_FAT16:
            Serial.println("[SD] FAT16");
            break;
        default:
            Serial.println("[SD] Desconhecido");
            break;
    }

    return true;
}

void INIT_RTC() {
    if (!rtc.begin()) {
        Serial.println("[RTC] Couldn't find RTC");
        Deep_Sleep_Now();
    }
}

void WriteToCSV() {
    DateTime timestamp = rtc.now();   // Obtém os valores dos sensores
    float SPEED = spd.speed;          // Velocidade do vento
    float PLV = plv.getRainVolume();  // Volume de chuva
    float bat_v = BATTERY_VOLTAGE;    // Tensão da bateria
    String DIR = dir.lerDirecao();  // Direção do vento, Formata o nome do arquivo com a data atual
    Serial.println("[SD] Writing data to CSV file...");

    char fileName[25];
    sprintf(fileName, "WSPLV1_%02d_%02d_%04d.csv", timestamp.day(), timestamp.month(),
            timestamp.year());
    Serial.print("[SD] Writing to file: ");
    Serial.println(fileName);
    char timeBuffer[20];

    snprintf(timeBuffer, sizeof(timeBuffer), "%02d:%02d:%02d", timestamp.hour(), timestamp.minute(),
             timestamp.second());

    String data = String(timeBuffer) + ";" + String(SPEED) + ";" + String(DIR) + ";" + String(PLV) +
                  ";" + String(bat_v) + "," + String(ws_data.longitude, 6) + "," +
                  String(ws_data.latitude, 6);

    data.replace(".", ",");

    // Check if the file exists or create it
    if (!SD.exists(fileName)) {
        Calibrate_RTC();
        // If the file doesn't exist, create it
        FsFile myFile = SD.open(fileName, FILE_WRITE);

        if (myFile) {
            // Write header to the file (assuming CSV header format)
            myFile.println(
                "Timestamp;WindSpeed [m/s];WindDirection;Rain [mm];BatVoltage,longitude,latitude");
            myFile.close();
            Serial.println("[SD] Header written to file: " + String(fileName));
        } else {
            Serial.println("[SD] Error creating file: " + String(fileName));
            return;
        }
    }

    // Open the file for appending data
    FsFile myFile = SD.open(fileName, FILE_WRITE);

    if (myFile) {
        // Write sensor data to the file
        myFile.println(data);
        myFile.close();
        Serial.println("[SD] Data written to file: " + String(fileName));
        Serial.print("[SD] Data: ");
        Serial.println(data);

    } else {
        Serial.println("[SD] Error opening file: " + String(fileName));
    }
}

void Deep_Sleep_Now() {
    esp_sleep_enable_timer_wakeup(UpdateInterval);
    Serial.println("ESP is tired, going to sleep.");
    Serial.flush();
    esp_deep_sleep_start();
    // ESP.restart();
    delay(2000);
}

void getBattery() {
    // Reading Battery Level in %
    // const float R1 = 4.7e3;      // Resistor R1 value in ohms
    // const float R2 = 1e3;        // Resistor R2 value in ohms
    float Vin;

    // Serial.print("adc val: ");
    Vin = map(analogRead(VBAT_PIN), 0, 4095, 0.0, 3.3);  // formula for calculating voltage out
    // Serial.println(Vin);

    // BATTERY_VOLTAGE = Vin * ((R2+1)*(R2+R1))/(R2*(R2-1));  // formula for calculating voltage in
    // ((R2+1)*(R2+R1))/(R2*(R2-1)) = 5.7;  // Resistor divider formula for battery voltage
    // BATTERY_VOLTAGE = Vin * 5.7;  // formula for calculating voltage in
    BATTERY_VOLTAGE = Vin * 9.25;
}

void Calibrate_RTC() {
    // Set NTP server and timezone (0 = GMT)
    modemGSM.sendAT("+CNTP=\"a.ntp.br\",-12");
    modemGSM.waitResponse(10000L);

    String ntpString = "";

    // Start NTP synchronization
    Serial.println("Requesting NTP sync...");
    modemGSM.sendAT("+CNTP");
    if (modemGSM.waitResponse(10000L, "OK")) {
        Serial.println("NTP sync requested.");
    } else {
        Serial.println("NTP sync failed.");
    }

    delay(5000);  // Give some time to sync

    // Read time from modemGSM
    Serial.println("Getting current time...");
    modemGSM.sendAT("+CCLK?");
    if (modemGSM.waitResponse(5000L, "+CCLK:")) {
        ntpString = modemGSM.stream.readStringUntil('\n');
        Serial.print("Modem Time: ");
        Serial.println(ntpString);
    } else {
        Serial.println("Failed to get time.");
    }

    int offsetIndex = ntpString.indexOf('-');
    if (offsetIndex != -1) {
        ntpString = ntpString.substring(0, offsetIndex);
    }

    // Quebra a string em data e hora
    int commaIndex = ntpString.indexOf(',');
    String dataStr = ntpString.substring(1, commaIndex);   
    String horaStr = ntpString.substring(commaIndex + 1);  
    // Corrige a ordem dos campos
    int ano = 2000 + dataStr.substring(1, 3).toInt();  
    int mes = dataStr.substring(4, 6).toInt();         
    int dia = dataStr.substring(8, 10).toInt();        
    int hora = horaStr.substring(0, 2).toInt();     
    int minuto = horaStr.substring(3, 5).toInt();   
    int segundo = horaStr.substring(6, 8).toInt();  

    // Ajusta o RTC
    rtc.adjust(DateTime(ano, mes, dia, hora, minuto, segundo));
    Serial.println("RTC calibrado com sucesso!");
}

float convertToDecimal(String raw, String direction) {
    float degMin = raw.toFloat();
    int degrees = (direction == "N" || direction == "S") ? int(degMin / 100) : int(degMin / 100);
    float minutes = degMin - (degrees * 100);
    float decimal = degrees + (minutes / 60.0);
    if (direction == "S" || direction == "W") decimal *= -1;
    return decimal;
}

void SetupGPS() {
    modemGSM.sendAT("+CGPS=1,1");
    modemGSM.waitResponse(5000L, "OK");
    Serial.println("GPS ativado.");
}

void readGPS() {
    modemGSM.sendAT("+CGPSINFO");
    if (modemGSM.waitResponse(1000L, "+CGPSINFO:")) {
        String gpsData = modemGSM.stream.readStringUntil('\n');
        gpsData.trim();
        // Serial.print("GPS Data: ");
        // Serial.println(gpsData);

        if (gpsData.startsWith(",")) {
            Serial.println("Sem fix de GPS ainda.");
            return;
        }

        // Parse
        int h, m, s, d, mo, y;
        char lat[16], latDir, lon[16], lonDir;
        sscanf(gpsData.c_str(), "%2d%2d%2d.0,%[^,],%c,%[^,],%c,,,%2d%2d%2d", &h, &m, &s, lat,
               &latDir, lon, &lonDir, &d, &mo, &y);

        ws_data.latitude = convertToDecimal(String(lat), String(latDir));
        ws_data.longitude = convertToDecimal(String(lon), String(lonDir));
    } else {
        Serial.println("Falha ao obter dados GPS.");
    }
}