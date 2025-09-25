#pragma once

// Configuração de pinos
#define SD_CS 5
#define SD_SCK 18
#define SD_MISO 19
#define SD_MOSI 23

#define RTC_SDA 21
#define RTC_SCL 22

// Configurações do sistema
#define LOG_INTERVAL 5000  // Intervalo entre leituras (ms)
#define FILE_BASE_NAME "dados"
#define MAX_FILE_SIZE 10  // Tamanho máximo em MB antes de rotacionar

// Estrutura de dados
struct SensorData {
    DateTime timestamp;
    float temperature;
    float humidity;
    float pressure;
};