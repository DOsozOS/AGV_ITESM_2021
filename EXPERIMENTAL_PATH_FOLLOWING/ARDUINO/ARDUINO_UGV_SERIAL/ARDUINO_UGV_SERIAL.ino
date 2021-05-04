//%%%%Conexiones Eje
#define reversa A8
#define pwmEje 46
#define velA A9
#define velB A10
#define frenoE A13
//%%%%Conexiones Freno
#define potFre A11
#define dirFre 49
#define pwmFre 45
#define resetFre 43
#define f1Fre 41
#define f2Fre 39
#define setpoint_frenado 7
//%%%%Conexiones Dirección
#define potDir A12
#define dirDir 37
#define pwmDir 44
#define resetDir 29
#define f1Dir 27
#define f2Dir 25
//%%%%Conexiones Encoders
#define encDD 21
#define encDI 20
#define encTD 19
#define encTI 18
//%%%%%%%%%%%%%%%%%Fin definición de conexiones%%%%%%%%%%%%%%
// 9 ES EL CENTRO!!!!
//variables para la comunicación serial
char datos[10];  //almacena los datos que llegan como instrucciones por el puerto serial
int contador=0;  //contador de caracteres recibidos por el puerto serial

//variable para general pwm a la salida
char imprimir=0;
int Setpoint_VF=0;
int Setpoint_V=0;
int Setpoint_D=4, Error_D, Output_D, OutputAnterior_D=0;
int Setpoint_F=0;
int Error_F, Output_F, OutputAnterior_F=0;
int contador_marcha=1;

int SensorFreno;           //contiene el valor de el potenciometro de la dirección (0 -1023)
int SensorFrenoAjustada;  //se establece en un rango de 0 a 8

//Comunicación serial para velocidad
char datos_velocidad[4];
int posicion_velocidad = 0;
char *resultado_velocidad = NULL;
char separador_velocidad[] = "V#";
double valores_velocidad[] = {0,0,0};
int Periodo_Encoder_Izquierdo = 0;
int Periodo_Encoder_Derecho = 0;
double Velocidad = 0;
unsigned long Tiempo_Tx1 = 0;
int decena = 0;
int unidad = 0;
int decimo = 0;
int centesimo = 0;

//Comunicación con control de xbox
char datos_control[11];
int posicion_control = 0;
char *resultado_control = NULL;
char separador_control[] = "FAD#";
int valores_control[] = {0,0,0};
byte Comandos_xbox_aux = 0b1000000;
int Comandos_xbox[8] = {0,0,0,0,0,0,0,0};
int salir = 0;
int cambiar_marcha=1;
int cambiar_paro=0;
int SensorDireccion;     //contiene el valor de el potenciometro de la dirección (0 -1023)
float velocidad_ros=0;
float direccion_ros=0;
float velocidad_setpoint = 0;
float direccion_setpoint = 0;
float direccion_xbox = 0;
float velocidad_xbox = 0;
float SensorDireccionAjustada=0.0;

int contador_control=0;
unsigned long Tiempo_Tx2 = 0;
unsigned long Tiempo_Lectura_XBOX = 0;
int Seguridad = 0;
int Seguridad_aux = 0;
int Paro = 0;

//controller variables
float KP = 15;
float KI = 2;
float KD = 1;
float offset_kp = 10;
float ErrorIntegral = 0.0001;
float ErrorIntegral2 = 0.0001;
float prev_error = 0;
float outputpower = 0.0;
unsigned long tiempo_ros = 0;
unsigned long time_controller = 0;
unsigned long time_ki_reset = 0;
unsigned long timeout_disconnection = 0;
unsigned long timeout_disconnection_xbox = 0;
bool new_speed_data = false;
bool CONNECTION_LOST = false;
float timeout_cmd = 0.0;
float timeout_cmd_past = 0.0;
bool freno_velc = false;

unsigned long disconnection_timeout_th = 1000;
unsigned long actualtime = 0;
int counter_watchdog = 0;
boolean blinkled = true;
boolean xbox_connected = false;
boolean warning = false;
//if false, then commands will be received from pc
//true if commands are gonna be send by xbox control

boolean xbox_pc = false;
void setup() {
  //Setup inputs
  pinMode(f1Fre,INPUT);
  pinMode(f2Fre,INPUT);
  pinMode(f1Dir,INPUT);
  pinMode(f2Dir,INPUT);
  pinMode(encDD,INPUT);
  pinMode(encDI,INPUT);
  pinMode(encTD,INPUT);
  pinMode(encTI,INPUT);
  //Setup outputs
  pinMode(reversa,OUTPUT);
  pinMode(velA,OUTPUT);
  pinMode(velB,OUTPUT);
  pinMode(frenoE,OUTPUT);
  pinMode(dirFre,OUTPUT);
  pinMode(resetFre,OUTPUT);
  pinMode(dirDir,OUTPUT);
  pinMode(resetDir,OUTPUT);
  pinMode(8,OUTPUT);
  Serial.begin(115200);
  //Serial1.begin(115200);
  //Serial2.begin(115200);
  Serial3.begin(115200); // Encoder serial

  digitalWrite(resetFre, HIGH);//Habilita drivers de motores
  digitalWrite(resetDir, HIGH);
 
  digitalWrite(frenoE, HIGH); //Desactiva el paro de emergencia
      
  digitalWrite(velA, LOW); //Activa la velocidad 1
  digitalWrite(velB, HIGH);
  //Serial.println("Marcha 1"); 
  
  digitalWrite(reversa, HIGH); //desactiva la reversa
  //Serial.println("Forward");

  //Serial.print("Vel="); 
  //Serial.println(Setpoint_V);

  Tiempo_Tx1 = millis();
  Tiempo_Tx2 = millis();
  Tiempo_Lectura_XBOX = millis();
  timeout_disconnection = millis();

  pinMode(13,OUTPUT);
  pinMode(7,OUTPUT);
  pinMode(6,OUTPUT);
  pinMode(5,OUTPUT);
  pinMode(pwmEje, OUTPUT);
  pinMode(pwmFre, OUTPUT);
  pinMode(pwmDir, OUTPUT);
}


// Control motor dirección. Speed should be a number from -255 to 255
void setMotorSpeed_D(int speedM){
  if (speedM < 0){
    digitalWrite(dirDir,LOW);
    speedM = -speedM; // make speed positive
  }
  else{
    digitalWrite(dirDir,HIGH);; // motor forward command
  }
  analogWrite(pwmDir, speedM);
}


// Control motor freno. Speed should be a number from -255 to 255
void setMotorSpeed_F(int speedM){
  if (speedM < 0){
    digitalWrite(dirFre,LOW);
    speedM = -speedM; // make speed positive
  }
  else{
    digitalWrite(dirFre,HIGH);; // motor forward command
  }
  analogWrite(pwmFre, speedM);
}

void controlDireccion (void){
  ;  //se establece en un rango de 0 a 8
  SensorDireccion=analogRead(potDir); //Conversion ADC pot dirección
  SensorDireccionAjustada=0.08712 * SensorDireccion - 50.15249;//map(SensorDireccion,418,810,0,15); //Cambia el rango de lectura a uno de 0 a 8
  
  Error_D=direccion_setpoint-SensorDireccionAjustada;
  
  if (abs(Error_D)>1){
  if (Error_D>0)  Output_D=-255;
  if (Error_D<0)  Output_D=255;
  }
  else
  {
    Output_D=0;
  }
  
  setMotorSpeed_D(Output_D); 
  if (Output_D!=OutputAnterior_D){ //Si el valor de salida es nuevo lo escribe, de lo contrario no lo modifica
    OutputAnterior_D=Output_D; 
    setMotorSpeed_D(Output_D); 
  }
  }

void controlFreno (void){
//  
//  if(Setpoint_F==9){digitalWrite(6,HIGH);}
//  else{digitalWrite(6,LOW);}

  //if(Setpoint_F > 2)analogWrite(pwmEje,50);
  
  
  SensorFreno=analogRead(potFre); //Conversion ADC pot dirección
  SensorFrenoAjustada=map(SensorFreno,710,875,0,9); //Cambia el rango de lectura a uno de 0 a 8
  if(SensorFrenoAjustada >=setpoint_frenado-1)
  {
    //Si elfreno está a tope o cerca del tope, la velocidad
    //que se publicará a ROS deberá ser 0.
    Velocidad = 0;
  }
  Error_F=Setpoint_F-SensorFrenoAjustada;
  if (Error_F==0) Output_F=0;
  if (Error_F>0)  Output_F=-255;
  if (Error_F<0)  Output_F=255;
  setMotorSpeed_F(Output_F); 
  if (Output_F!=OutputAnterior_F){ //Si el valor de salida es nuevo lo escribe, de lo contrario no lo modifica
    OutputAnterior_F=Output_F; 
    setMotorSpeed_F(Output_F); 
  }
 }

void paroEmergencia(void){ //Define las condiciones de paro de emergencia
    velocidad_setpoint = 0;
    Setpoint_F = setpoint_frenado;
    }

String inString = "";
void LecturaVelocidad(){
  
  if (Serial3.available()) {
    inString = Serial3.readStringUntil('\n');
    int firstcommaIndex = inString.indexOf(',');
    int secondCommaIndex = inString.indexOf(',', firstcommaIndex + 1);
    int thirdCommaIndex = inString.indexOf(',', secondCommaIndex + 1);
    int fourthCommaIndex = inString.indexOf(',', thirdCommaIndex + 1);
    float velocidad = 0.0;
    String simbolo1 = inString.substring(firstcommaIndex+1,secondCommaIndex);
    String simbolo2 = inString.substring(thirdCommaIndex+1,fourthCommaIndex);
    if(simbolo1 == "V" and simbolo2 == "#"){
    String secondValue = inString.substring(secondCommaIndex+1, thirdCommaIndex);
      if(secondValue.indexOf('.')>-1){      
        Velocidad = secondValue.toFloat();
        new_speed_data = true;
      }
    }
  }
}
void VelocidadCrucero()
{
    float velocidadEjeAjustada = 0.0;
    
    float Error = velocidad_setpoint - Velocidad;
    double delta_time_controller = (millis()-time_controller)*0.001;
    float rate_error = (Error-prev_error)/(delta_time_controller);
    
    prev_error = Error;
    if(velocidad_setpoint==0)
    {
      ErrorIntegral=0;
      ErrorIntegral2=0;
      outputpower=0;
      rate_error = 0;
    }
    if (SensorFrenoAjustada<setpoint_frenado){

    if (velocidad_setpoint>0){
      if(Error>-0.2){
        Setpoint_F = 0;
        ErrorIntegral = ErrorIntegral + Error*delta_time_controller;
        ErrorIntegral2 = ErrorIntegral;
        velocidadEjeAjustada=map(offset_kp+Error*KP + ErrorIntegral*KI + rate_error*KD,0,122,50,185);
        //Serial.println(Error);
        analogWrite(pwmEje,velocidadEjeAjustada);
      }
      else if (Error<=-0.2 and Error >-0.4)
      {
        //Serial.println(0);
        Setpoint_F = 0;
        ErrorIntegral = 0.5*ErrorIntegral2;
        analogWrite(pwmEje,50);
      }
      else if (Error<=-0.4 and Error >-0.6)
      {
        //Serial.println(0);
        ErrorIntegral = 0;
        Setpoint_F = setpoint_frenado-2;
        analogWrite(pwmEje,50);
      }
      else if (Error<=-0.6)// and Error >-1)
      {
        //Serial.println(0);
        ErrorIntegral = 0;
        Setpoint_F = setpoint_frenado-1;
        analogWrite(pwmEje,50);
      }
    }
    else
    {
      ErrorIntegral = 0;
      analogWrite(pwmEje,50);
    }
    }
    time_controller = millis();
}
void VelocidadCrucero2()
{
    if (SensorFrenoAjustada<2 and velocidad_setpoint>0){
        float nocontrol = map(KP*velocidad_setpoint,0,122,50,185);
        analogWrite(pwmEje,nocontrol);
        Setpoint_F=0;
    }
    else if(velocidad_setpoint==0)
    {
      analogWrite(pwmEje,50);
    }
    else if(velocidad_setpoint<0)
    {
      analogWrite(pwmEje,50);
      Setpoint_F = setpoint_frenado-3 -1*velocidad_setpoint;
    }
    else
    {
      analogWrite(pwmEje,50);
    }
    time_controller = millis();
}

void decodificar_dir_vel()
{
  if (Serial.available()) {
    digitalWrite(5,HIGH);
    String inString = Serial.readStringUntil('\n');
    int firstcommaIndex = inString.indexOf(',');
    int secondCommaIndex = inString.indexOf(',', firstcommaIndex + 1);//simbolo1
    int thirdCommaIndex = inString.indexOf(',', secondCommaIndex + 1);//velocidad
    int fourthCommaIndex = inString.indexOf(',', thirdCommaIndex + 1);//direccion
    int fifthCommaIndex = inString.indexOf(',', fourthCommaIndex + 1);//freno
    int sixCommaIndex = inString.indexOf(',', fifthCommaIndex + 1);
    
    float velocidad = 0.0;
    String simbolo1 = inString.substring(firstcommaIndex+1,secondCommaIndex);
    String simbolo2 = inString.substring(fifthCommaIndex+1,sixCommaIndex);
    //Serial.println(simbolo1);
    //Serial.println(simbolo2);
    if(simbolo1 == "#" and simbolo2 == "$"){
    //Serial.println("Received");
    String speed_str = inString.substring(secondCommaIndex+1, thirdCommaIndex);
    String steer_str = inString.substring(thirdCommaIndex+1, fourthCommaIndex);
    String brake_str = inString.substring(fourthCommaIndex+1, fifthCommaIndex);
      if(speed_str.indexOf('.')>-1 and steer_str.indexOf('.')>-1 ){ 
        velocidad_ros = speed_str.toFloat();
        direccion_ros = steer_str.toFloat();
        timeout_disconnection = actualtime;
      }
      Setpoint_F = brake_str.toInt();
      //Serial.println("speed:" + speed_str + " steer: " + steer_str + " brake_str: "+brake_str);
    }
  }
}

void imprimir_datos_ros()
{
  if(actualtime-tiempo_ros>100){
  String velocidadstr =  String(Velocidad, DEC);//Velocidad
  String rotacionstr =  String(SensorDireccionAjustada, DEC);//SensorDireccionAjustada
  String powerstr = String(outputpower,DEC);
  String frenostr = String(SensorFrenoAjustada,DEC);
  String setvelstr = String(velocidad_setpoint,DEC);
  Serial.println(",#,"+velocidadstr+","+rotacionstr+",$,");
  //Serial.println(Setpoint_F);
  //Serial.println(inString);
  tiempo_ros = actualtime;
  }
} 
//,#,12.2,3.4,$,
void loop() {
  
  actualtime = millis();
  
  decodificar_dir_vel();
      
    direccion_setpoint = direccion_ros;
    velocidad_setpoint = velocidad_ros;
    
    //IF NO ROS RESPONES WAS RECEIVED
    if((actualtime-timeout_disconnection)>disconnection_timeout_th)
    {CONNECTION_LOST = true;}
    else{CONNECTION_LOST = false;}  
  
  digitalWrite(13,CONNECTION_LOST);
  
  LecturaVelocidad();
  if(velocidad_setpoint == 0)
  {
    Velocidad = 0;
  }
  if( SensorFrenoAjustada >=setpoint_frenado){
    velocidad_xbox = 0;
   velocidad_ros = 0;
   velocidad_setpoint = 0;
   outputpower = 0;
   ErrorIntegral = 0;
   analogWrite(pwmEje,50);
  }
  if(CONNECTION_LOST){
   //Serial.println("BRAKING");
   paroEmergencia();
   velocidad_xbox = 0;
   velocidad_ros = 0;
   velocidad_setpoint = 0;
   outputpower = 0;
   ErrorIntegral = 0;
   analogWrite(pwmEje,50);
  }
  else{   
    VelocidadCrucero();
  }
  
  controlDireccion();
  controlFreno();
  imprimir_datos_ros();
  delay(50);
}
