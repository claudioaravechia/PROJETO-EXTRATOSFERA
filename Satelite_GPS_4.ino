#include <SD.h>
#include <Adafruit_BMP085.h>
#include <Adafruit_GFX.h>
#include "dht.h"
#include "RTClib.h"  //INCLUSÃO DA BIBLIOTECA
#include <Adafruit_PWMServoDriver.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>


#define GPS_RX 4
#define GPS_TX 3
#define GPS_Serial_Baud 9600


TinyGPS gps;
SoftwareSerial gpsSerial(GPS_RX, GPS_TX);

// INSTANCIANDO OBJETOS
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// DECLARAÇÃO DE FUNÇÕES
void SoltarBalao();
void beginServos();
void writeServos(int nServo, int posicao);
void GravarDadosCartao();

byte pinoCS = 10;  //Pino CS don= modulo de gravacao de cartao
File myFile;
bool IniciaArquivo = true;
int Contador = 0;
int GravaCartao = 1;

float flat, flon;
unsigned long age;

// pushbutton para iniciar a gravacao do cartao
const int button = 8;  // entrada digital - pushbutton
const int ledPin = 2;  // saída digital -  LED

float AltitudeAbertura = 520;  // teste de abertura do servo

Adafruit_BMP085 bmp180;

//PINO ANALÓGICO UTILIZADO PELO DHT11
const int pinoDHT11 = A2;  
dht DHT;                   //VARIÁVEL DO TIPO DHT

RTC_DS1307 rtc;  //OBJETO DO TIPO RTC_DS1307
//DECLARAÇÃO DOS DIAS DA SEMANA
char daysOfTheWeek[7][12] = { "Domingo", "Segunda", "Terça", "Quarta", "Quinta", "Sexta", "Sábado" };

void setup() {
  // INICIALIZA O SERVO
  beginServos();  
  delay(300);
  //COLOCA O SERVO NA POSICAO 90 GRAUS
  writeServos(0, 90);

  Serial.begin(GPS_Serial_Baud);
  gpsSerial.begin(GPS_Serial_Baud);

  // INICIALIZA O SENSOR BMP180
  if (!bmp180.begin()) {
    Serial.println("Sensor nao encontrado !!");
    while (1) {}
  }

  // INICIALIZA O RELOGIO DO SISTEMA
  if (!rtc.begin()) {                         // SE O RTC NÃO FOR INICIALIZADO, FAZ
    Serial.println("DS1307 não encontrado");  //IMPRIME O TEXTO NO MONITOR SERIAL
    while (1)
      ;  //SEMPRE ENTRE NO LOOP
  }
  if (!rtc.isrunning()) {               //SE RTC NÃO ESTIVER SENDO EXECUTADO, FAZ
    Serial.println("DS1307 rodando!");  //IMPRIME O TEXTO NO MONITOR SERIAL
    //rtc.adjust(DateTime(2018, 7, 5, 15, 33, 15)); //(ANO), (MÊS), (DIA), (HORA), (MINUTOS), (SEGUNDOS)
  }
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));  //CAPTURA A DATA E HORA EM QUE O SKETCH É COMPILADO
  delay(100);                                      //INTERVALO DE 100 MILISSEGUNDOS

  pinMode(pinoCS, OUTPUT);  //Define o pinoSS como saida
  if (SD.begin())           //Inicializa o SD Card
  {
    Serial.println("SD Card pronto para uso.");  //Imprime na tela
  } else {
    Serial.println("Falha na inicialização do SD Card.");
    return;
  }
  // define pinos de saída e entrada
  pinMode(ledPin, OUTPUT);  //Define ledPin  como saída
  pinMode(button, INPUT_PULLUP);
  digitalWrite(ledPin, LOW);
}
void loop() {
  if (digitalRead(button) == 0) {
    GravaCartao = 0;
    digitalWrite(ledPin, HIGH);
    Serial.println(" inicio");
    delay(150);
  }

  bool newData = false;
  unsigned long chars;

  // For one second we parse GPS data and report some key values
  for (unsigned long start = millis(); millis() - start < 1000;) {
    while (gpsSerial.available()) {
      char c = gpsSerial.read();
      // Serial.write(c); //apague o comentario para mostrar os dados crus
      if (gps.encode(c))  // Atribui true para newData caso novos dados sejam recebidos
        newData = true;
    }
  }

  if (newData) {
    SoltarBalao();
    ImprimirTela();

    if (GravaCartao == 0) {
      //     float flat, flon;
      //     unsigned long age;
      DHT.read11(pinoDHT11);  //LÊ AS INFORMAÇÕES DO SENSOR
      DateTime now = rtc.now();
      Serial.println(Contador);
      if (IniciaArquivo == true) {
        myFile = SD.open("gps.txt", FILE_WRITE);  //Cria e abre o arquivo
        delay(100);
        myFile.print(" ;  ");
        myFile.print("Hora");
        myFile.print(" ;  ");
        myFile.print("  Temp");
        myFile.print(" ;  ");
        myFile.print("  Alt");
        myFile.print(" ;  ");
        myFile.print("  Pre");
        myFile.print(" ;  ");
        myFile.print("  Umi");
        myFile.print(" ;  ");
        myFile.print("  Lat");
        myFile.print(" ;  ");
        myFile.print("  Long");
        myFile.print(" ;  ");
        myFile.print("  N_Sat");
        myFile.print(" ;  ");
        myFile.println("  Prec");
        myFile.print(" ;  ");
        IniciaArquivo = false;
        Contador = 0;
      }
      if (now.hour() > 10) {
        myFile.print(now.hour(), DEC);
      } else {
        myFile.print("0");
        myFile.print(now.hour(), DEC);
      }
      if (now.minute() < 10) {
        myFile.print("0");
        myFile.print(now.minute(), DEC);
      } else {
        myFile.print(now.minute(), DEC);
      }
      if (now.second() < 10) {
        myFile.print("0");
        myFile.print(now.second(), DEC);
      } else {
        myFile.print(now.second(), DEC);
      }
      myFile.print(" ;  ");
      // Temperatura
      if (bmp180.readTemperature() < 10) {
        myFile.print(bmp180.readTemperature(), 2);
        myFile.print(" ;  ");
      } else {
        myFile.print(bmp180.readTemperature(), 2);
        myFile.print(" ;  ");
      }
      //Altitude
      myFile.print(bmp180.readAltitude(), 2);
      myFile.print(" ;  ");
      // Pressao
      myFile.print(bmp180.readPressure());
      myFile.print(" ;  ");
      // Umidade
      myFile.print(DHT.humidity, 2);
      myFile.print(" ;  ");

      gps.f_get_position(&flat, &flon, &age);
      myFile.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);  //LAT
      myFile.print(" ;  ");
      myFile.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);  //LON
      myFile.print(" ;  ");
      myFile.print(gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites());  //SAT
      myFile.print(" ;  ");
      myFile.println(gps.hdop() == TinyGPS::GPS_INVALID_HDOP ? 0 : gps.hdop());  //PREC
      myFile.print(" ;  ");


      Contador = Contador + 1;
      if (Contador == 100) {
        myFile.close();
        delay(500);
        IniciaArquivo = true;
        GravaCartao = 1;  // NAO GRAVA NO CARTAO
        Contador = 0;
        digitalWrite(ledPin, LOW);
      }
    }
  }
}
