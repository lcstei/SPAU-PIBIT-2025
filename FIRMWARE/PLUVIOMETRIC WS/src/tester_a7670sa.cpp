#include <HardwareSerial.h>
#include <TinyGsmClient.h>

// Define serial pins for A7670SA (adjust if needed)
#define MODEM_RX 33
#define MODEM_TX 32
#define MODEM_BAUD 115200

// APN for your SIM card provider (change as needed)
const char apn[] = "claro.com.br";
const char user[] = "claro";
const char pass[] = "claro";

// Initialize hardware serial for modem
HardwareSerial SerialAT(1);  // Use UART1

TinyGsm modem(SerialAT);

void setup() {
    Serial.begin(115200);
    pinMode(26, OUTPUT);
    digitalWrite(26, HIGH);  // ATIVA o módulo GSM inicialmente
    delay(10);

    // Start communication with the modem
    SerialAT.begin(MODEM_BAUD, SERIAL_8N1, MODEM_RX, MODEM_TX);
    delay(3000);

    Serial.println("Initializing modem...");
    modem.restart();
    Serial.println("Modem restarted.");

    // Connect to network
    Serial.print("Connecting to network...");
    if (!modem.waitForNetwork(30000)) {
        Serial.println(" FAIL");
        return;
    }
    Serial.println(" OK");

    // GPRS connection
    Serial.print("Connecting to GPRS...");
    if (!modem.gprsConnect(apn, user, pass)) {
        Serial.println(" FAIL");
        return;
    }
    Serial.println(" OK");

    // Set NTP server and timezone (0 = GMT)
    modem.sendAT("+CNTP=\"a.ntp.br\",-12");
    modem.waitResponse(10000L);

    String ntpString = "";

    // Start NTP synchronization
    Serial.println("Requesting NTP sync...");
    modem.sendAT("+CNTP");
    if (modem.waitResponse(10000L, "OK")) {
        Serial.println("NTP sync requested.");
    } else {
        Serial.println("NTP sync failed.");
    }

    delay(5000);  // Give some time to sync

    // Read time from modem
    Serial.println("Getting current time...");
    modem.sendAT("+CCLK?");
    if (modem.waitResponse(5000L, "+CCLK:")) {
        ntpString = modem.stream.readStringUntil('\n');
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

    // Confirma no serial
    Serial.print("Parsed DateTime: ");
    Serial.print(ano);
    Serial.print("-");
    Serial.print(mes);
    Serial.print("-");
    Serial.print(dia);
    Serial.print(" ");
    Serial.print(hora);
    Serial.print(":");
    Serial.print(minuto);
    Serial.print(":");
    Serial.println(segundo);
}

void loop() {
    // Nothing here
}
