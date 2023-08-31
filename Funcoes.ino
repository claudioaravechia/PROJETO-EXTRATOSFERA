// abre o dispositivo para soltar o balao do satelite
void SoltarBalao() {
  // abertura do servo
  AltitudeAbertura = AltitudeAbertura + 10.0;
  Serial.print(AltitudeAbertura);
  if (AltitudeAbertura > 650.0) {
    //if (AltitudeAbertura > bmp180.readAltitude()) {
    writeServos(0, 0);
    AltitudeAbertura = 520;
    delay(1000);
    writeServos(0, 90);
  }
}

// imprime os valores dos sensores na tela
void ImprimirTela() {
  // le os outros sensores
  DHT.read11(pinoDHT11); //LÊ AS INFORMAÇÕES DO SENSOR
  DateTime now = rtc.now();
  Serial.print("Data: "); //IMPRIME O TEXTO NO MONITOR SERIAL
  Serial.print(now.day(), DEC); //IMPRIME NO MONITOR SERIAL O DIA
  Serial.print('/'); //IMPRIME O CARACTERE NO MONITOR SERIAL
  Serial.print(now.month(), DEC); //IMPRIME NO MONITOR SERIAL O MÊS
  Serial.print('/'); //IMPRIME O CARACTERE NO MONITOR SERIAL
  Serial.print(now.year(), DEC); //IMPRIME NO MONITOR SERIAL O ANO
  Serial.print('-');
  if (now.hour() > 10) {
    Serial.print(now.hour(), DEC); //IMPRIME NO MONITOR SERIAL A HORA
  }
  else
  {
    Serial.print("0"); //IMPRIME NO MONITOR SERIAL A HORA
    Serial.print(now.hour(), DEC); //IMPRIME NO MONITOR SERIAL A HORA
  }
  Serial.print(':'); //IMPRIME O CARACTERE NO MONITOR SERIAL
  if (now.minute() < 10) {
    Serial.print("0"); //IMPRIME NO MONITOR SERIAL OS MINUTOS
    Serial.print(now.minute(), DEC); //IMPRIME NO MONITOR SERIAL OS MINUTOS
  }
  else
  {
    Serial.print(now.minute(), DEC); //IMPRIME NO MONITOR SERIAL OS MINUTOS
  }
  Serial.print(':'); //IMPRIME O CARACTERE NO MONITOR SERIAL
  if (now.second() < 10) {
    Serial.print("0"); //IMPRIME NO MONITOR SERIAL OS SEGUNDOS
    Serial.print(now.second(), DEC); //IMPRIME NO MONITOR SERIAL OS SEGUNDOS
  }
  else {
    Serial.print(now.second(), DEC); //IMPRIME NO MONITOR SERIAL OS SEGUNDOS
  }
  Serial.println(); //QUEBRA DE LINHA NA SERIAL
  Serial.print("Temperatura : ");
  if ( bmp180.readTemperature() < 10)
  {
    Serial.print(bmp180.readTemperature());
    Serial.println(" C");
  }
  else
  {
    Serial.print(bmp180.readTemperature(), 1);
    Serial.println(" C");

  }
  Serial.print("Altitude : ");
  Serial.print(bmp180.readAltitude());
  Serial.println(" m");
  Serial.print("Pressao : ");
  Serial.print(bmp180.readPressure());
  Serial.println(" Pa");
  Serial.print("UMI. : ");
  Serial.print(DHT.humidity);
  Serial.println(" %");

  float flat, flon;
  unsigned long age;
  gps.f_get_position(&flat, &flon, &age);
  Serial.print("LAT=");
  Serial.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
  Serial.print(" LON=");
  Serial.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
  Serial.print(" SAT=");
  Serial.print(gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites());
  Serial.print(" PREC=");
  Serial.print(gps.hdop() == TinyGPS::GPS_INVALID_HDOP ? 0 : gps.hdop());
  Serial.println();
  Serial.println();
}

// inicializaco dos servos
void beginServos() {

#define Frequencia 50  // VALOR DA FREQUENCIA DO SERVO

  pwm.begin();                 // INICIA O OBJETO PWM
  pwm.setPWMFreq(Frequencia);  // DEFINE A FREQUENCIA DE TRABALHO DO SERVO
}


void writeServos(int nServo, int posicao) {
#define SERVOMIN 125  // VALOR PARA UM PULSO MAIOR QUE 1 mS
#define SERVOMAX 525  // VALOR PARA UM PULSO MENOR QUE 2 mS

  int pos = map(posicao, 0, 180, SERVOMIN, SERVOMAX);
  Serial.print("ANGULO: ");
  Serial.println(pos);
  pwm.setPWM(nServo, 0, pos);
}
