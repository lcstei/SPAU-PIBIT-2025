#include <Arduino.h>
#include <SPI.h>
#include <SdFat.h>

// Pino do Chip Select (CS) para o módulo SD
#define SD_CS 5

// Configura o objeto da biblioteca SdFat
SdFat sd;

// Configura o objeto do arquivo
FsFile myFile;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ; // Aguarda a inicialização do monitor serial
  }

  Serial.println("Iniciando o cartao SD com a biblioteca SdFat...");

  // Inicializa o cartao SD
  if (!sd.begin(SD_CS, SPI_HALF_SPEED)) {
    Serial.println("Falha na inicializacao do cartao SD. Verifique as conexoes e a formatacao.");
    sd.initErrorHalt(); // Mostra o erro e para a execucao
  }

  Serial.println("Cartao SD inicializado com sucesso.");

  // Exemplo de criacao e escrita de arquivo
  Serial.println("Criando e escrevendo no arquivo log.txt...");
  myFile = sd.open("log.txt", FILE_WRITE);

  if (myFile) {
    myFile.println("Ola, PlatformIO e SdFat!");
    myFile.close();
    Serial.println("Escrita concluida.");
  } else {
    Serial.println("Erro ao abrir log.txt para escrita.");
  }

  // Exemplo de leitura de arquivo
  Serial.println("\nLendo o conteudo do arquivo log.txt...");
  myFile = sd.open("log.txt");
  if (myFile) {
    // Le e exibe o conteudo linha por linha
    while (myFile.available()) {
      Serial.write(myFile.read());
    }
    myFile.close();
  } else {
    Serial.println("Erro ao abrir log.txt para leitura.");
  }
}

void loop() {
  // Nada para fazer no loop neste exemplo
}