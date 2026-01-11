
#define LED_IR 2
#define LED 13
#define BTN_CAL 8
#define BTN_START 9

#define pwmi  3   //PWM LEFT MOTOR
#define izq1  5   //LEFT1 MOTOR
#define izq2  4   //LEFT2 MOTOR
#define pwmd  11  //PWM RIGHT MOTOR
#define der1  6   //RIGHT1 MOTOR
#define der2  7   //RIGHT2 MOTOR

int sensores[8];
int digital[8];
int lectura_fondo[8];
int lectura_linea[8];
int umbral[8];
long int sumap, suma, pos, poslast, position;

//////////////////////PID//////////////////////
float KP = 0.18;
float KD = 4;
float KI = 0.002;
int vel = 140;
int veladelante = 200;
int velatras = 100;
///////////////////////////////////////////////

///////////////////////Datos para la integral////////////////////////
int error1 = 0;
int error2 = 0;
int error3 = 0;
int error4 = 0;
int error5 = 0;
int error6 = 0;
/////////////////////////////////////////////////////////////////////

//////////////////////variables PID//////////////////////
int proporcional = 0;
int integral = 0;
int derivativo = 0;
int diferencial = 0;
int last_prop;
int set_point = 350;
/////////////////////////////////////////////////////////


void setup() {
  // Serial.begin(9600);

  //set timer 2 divisor to    32 for PWM frequency of   980.39 Hz
  TCCR2B = TCCR2B & B11111000 | B00000011;   

  pinMode(izq1,OUTPUT);
  pinMode(izq2,OUTPUT);
  pinMode(der1,OUTPUT);
  pinMode(der2,OUTPUT);

  pinMode(LED_IR, OUTPUT);
  pinMode(LED, OUTPUT);
  pinMode(BTN_CAL, INPUT_PULLUP);
  pinMode(BTN_START, INPUT_PULLUP);

  digitalWrite(LED_IR, HIGH);
  digitalWrite(LED, HIGH);
  while(digitalRead(BTN_CAL));
  for(int i=0; i<50; i++){
    fondos();
    digitalWrite(LED, LOW);
    delay(20);
    digitalWrite(LED, HIGH);
    delay(20);
  }
  while(digitalRead(BTN_CAL));
  for(int i=0; i<50; i++){
    lineas();
    digitalWrite(LED, LOW);
    delay(20);
    digitalWrite(LED, HIGH);
    delay(20);
  }
  promedio();
  while(digitalRead(BTN_START));
  delay(500);
  digitalWrite(LED, LOW);

}

void loop() {
  int go = digitalRead(BTN_START);
  while(true){
    int go = digitalRead(BTN_START);
    frenos();
    lectura();
    PID();

    if(go == 0){
      motores(-20, -20);
      motores(0, 0);
      delay(1000);
      while(digitalRead(BTN_START));
      delay(1000);
      // break;
    }
  }

  // while(true){
  //   motores(0, 0);
  // }
}

void fondos(){
  for(int i=0; i<8; i++){
    lectura_fondo[i] = analogRead(A7-i);

    // Serial.print(lectura_fondo[i]);
    // Serial.print("\t");
  }

  // Serial.println("");
}

void lineas(){
  for(int i=0; i<8; i++){
    lectura_linea[i] = analogRead(A7-i);

    // Serial.print(lectura_linea[i]);
    // Serial.print("\t");
  }

  // Serial.println("");
}

void promedio(){
  for(int i=0; i<8; i++){
    umbral[i] = (lectura_fondo[i] + lectura_linea[i])/2;
    // Serial.print(umbral[i]);
    // Serial.print("\t");
  }

  // Serial.println("");
}

int lectura(void){
  for(int i=0; i<8; i++){
    sensores[i] = analogRead(A7-i);

    if(sensores[i] <= umbral[i]){
      digital[i] = 0;
    }else{
      digital[i] = 1;
    }

    // Serial.print(digital[i]);
    // Serial.print("\t");
  }

  // Serial.println("");

  sumap = (700*digital[0] + 600*digital[1] + 500*digital[2] + 400*digital[3] + 300*digital[4] + 200*digital[5] + 100*digital[6] + 0*digital[7]);
  suma = (digital[0] + digital[1] + digital[2] + digital[3] + digital[4] + digital[5] + digital[6] + digital[7]);
  pos = (sumap/suma);
  if(poslast <= 100 && pos == -1){
    pos = 0;
  }
  if(poslast >= 600 && pos == -1){
    pos = 700;
  }
  poslast = pos;
  return pos;
}

void PID(){
  proporcional = pos - set_point;
  derivativo = proporcional - last_prop;
  integral = error1 + error2 + error3 + error4 + error5 + error6;
  last_prop = proporcional;
  error6 = error5;
  error5 = error4;
  error4 = error3;
  error3 = error2;
  error2 = error1;
  error1 = proporcional;

  int diferencial = (proporcional * KP) + (derivativo * KD) + (integral * KI);

  if(diferencial > vel){
    diferencial = vel;
  }else if(diferencial < -vel){
    diferencial = -vel;
  }

  (diferencial < 0) ?
  motores(vel, vel + diferencial) : motores(vel - diferencial, vel);
}

void frenos(){
  if(pos <= 0){
    motores(veladelante, -velatras);
  }
  if(pos >= 700){
    motores(-velatras, veladelante);
  }
}

void motores(int izq, int der){//0 hasta 255    0 hasta -255
  // if (izq > M1_max_speed ) izq = M1_max_speed;
  // if (der > M2_max_speed ) der = M2_max_speed;
  // if (izq < 0) izq = 0; 
  // if (der < 0) der = 0; 
  ////////////////motor LEFT "IZQUIERDO" ////////////////////////
  if(izq>=0){
    digitalWrite(izq1,HIGH);
    digitalWrite(izq2,LOW);
  }
  else{
    digitalWrite(izq1,LOW);
    digitalWrite(izq2,HIGH);
    izq=izq*(-1);
  }
  analogWrite(pwmi,izq);
  
  ////////////////motor RIGHT "DERECHO" ////////////////////////
  if(der>=0){
    digitalWrite(der1,HIGH);
    digitalWrite(der2,LOW);
  }
  else{
    digitalWrite(der1,LOW);
    digitalWrite(der2,HIGH);
    der=der*(-1);
  }
  analogWrite(pwmd,der);
}
