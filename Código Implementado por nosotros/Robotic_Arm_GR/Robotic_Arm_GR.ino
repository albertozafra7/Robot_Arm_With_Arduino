#include "MeMegaPi.h"
#include <Servo.h>

MePort limitSwitch(PORT_7); // Metemos el switch de los finales de carrera
Servo svs[1] = {Servo()}; // Se mete el servo de la pinza
MeStepperOnBoard steppers[3] = {MeStepperOnBoard(PORT_1),MeStepperOnBoard(PORT_2),MeStepperOnBoard(PORT_3)};  // Se crean tres objetos de los steppers

// Limites de las articulaciones
float qlimit_0[2] = {-90.0,90.0}; //ToDo
float qlimit_1[2] = {0.0,0.0}; //ToDo
float qlimit_2[2] = {0.0,0.0}; //ToDo

// Se genera la estructura para un vector 3D
struct Vector3
{
  double x;
  double y; 
  double z;
};

float lastPositions[3] = {0,0,0}; // Vector para guardar la posición anterior del robot
const double RADS = PI / 180.0; // Conversión de grados sexadecimales a radianes
const double DEGS = 180.0 / PI; // Conversión de radianes a grados sexadecimales
const int STEPS = 2;  // No cambiar, son los pasos que realiza cada motor en cada interacción
const int GEAR_1 = 9; // Relación entre los dos tipos de engranajes 72/8
const int GEAR_2 = 7; // 


// longitudes de los eslabones
const double L1 = 150.0;  // Eslabón vertical que une S1 y S2
const double L2 = 155.0;  // Eslabón que une S2 y S3
const double L3 = 200.0;  // Eslabón que une S3 y el extremo

// Distancia a la que se encuentra el eslabon conforme al extremo en mm
const double Tz = -45.0;  // Como el extremo se encuentra por encima del eslabon es negativo
const double Tx = 65.0;

String buffer = ""; // Designamos una variable que recogerá todo lo que introduzcamos por medio del puerto serie hasta el \n
bool endMoving = true;  // 
int sensor1, sensor2; // Definimos los Bumpers

Vector3 vectorToAngles(float x,float y,float z);  // Función que le adjudica a un vector 3D devuelto los valores pasados por parámetro
void setSpeed(float speed); // Función que settea la velocidad actual a la pasada por parámetro

void move_steps(int motor_index, int steps);

// Parámetros, NO CAMBIAR
int testSpeed = 200;  // Velocidad de prueba
int testAcceleration = 300; // Aceleración de prueba
int currentSpeed = 350; // Velocidad actualmente usada
int maxSpeed = 500; // Máxima velocidad
int currentAcceleration = 1000; // Aceleración actualmente usada

// Definimos los pines que se van a usar para los bumpers
int pin1, pin2;

//************************************* Aquí comienza el código principal *************************************
void setup() {
  
  // Comenzamos el puerto serie
  Serial.begin(115200);

  // Configuración motores paso a paso
  setSpeedConfiguration(currentSpeed,maxSpeed,currentAcceleration); // Establezco la configuración de los motores con la velocidad actual, la máxima y la acleración
  
  for(int i=0;i<3;i++){
    steppers[i].setMicroStep(STEPS);  // Establecemos el paso de cada motor a 2
    steppers[i].enableOutputs();  // Habilita las salidas para poder meterle la corriente y de esta manera mover el robot
  }

  //Sensores finales de carrera
  pin1 = limitSwitch.pin1();  // Creo los pines de los bumpers
  pin2 = limitSwitch.pin2();
  pinMode(pin1,INPUT_PULLUP); // Entrada digital
  pinMode(pin2,INPUT_PULLUP);
  
  //Pinza. Configuración servo 
  svs[0].attach(A8);  // Servo se encuentra en el puerto A8, que es el 6
  open_grip();  // Abrir la pinza
  delay(1000);  // Esperamos
  close_grip(); // Cerrar la pinza
  
}


void loop() {

  //Lectura del puerto serie y filtrado del comando
  if (Serial.available()) { // Miro si hay algo en el buffer
     char c = Serial.read();  // Leo lo que hay
     if (c == '\n') { // Si lo que leo es un intro
      parseBuffer();  // Filtro el mensaje
    } else {  // Si no
      buffer += c;  // Introduzco en la variable buffer los datos
    }
  }

  //Visualización finales de carrera
  sensor1 = digitalRead(pin1);  // Lee la entrada del bumper 1
  Serial.println(sensor1);  // La imprime
  sensor2 = digitalRead(pin2);  // Aquí igual
  Serial.println(sensor2);


  // Lo MAS IMPORTANTE
  //Permito corriente a los motores en un movimiento
  long isMoving = 0;  // Generamos una variable que guarda la distancia que queda hasta llegar al destino
  for(int i=0;i<3;i++){
      isMoving += abs(steppers[i].distanceToGo());  // Se calcula la distancia absoluta entre mi target y mi current position
      steppers[i].run();  // Dándole pasos al motor
  }
  if(isMoving>0){ // Cuando ha llegado a su target posicion se sale
      endMoving = true;
  }
  else{
      if(endMoving){  // Si no, actualiza las posiciones del robot
         endMoving = false;
         steppers[0].setCurrentPosition(lastPositions[0]);
         steppers[1].setCurrentPosition(lastPositions[1]);
         steppers[2].setCurrentPosition(lastPositions[2]);
      }
  }
}

void parseBuffer() {  // Coje la cadena, le quita los espacios y la filtra
  
  buffer =" "+buffer+" ";
  buffer.toLowerCase();

  int count = 0;
  int startIndex = 0;
  int endIndex = 0;
  int len = buffer.length();
  if (len < 1) {
    return;
  }
  String tmp;
  String values[6];

  bool openEnable = false;
  bool closeEnable = false;
  
  //Filtrado del mensaje por el puerto serie
  while (true) {
    startIndex = buffer.indexOf(" ", endIndex);
    endIndex = buffer.indexOf(" ", startIndex + 1);
    tmp = buffer.substring(startIndex + 1, endIndex);
   
    if(tmp.indexOf("q1",0)>-1){ // Lee q1, detecta lo siguiente como parámetros y lo imprime
      values[0] = tmp.substring(2, tmp.length());
      Serial.println(values[0]);
      move_q1(stringToFloat(values[0]));
    }
    else if(tmp.indexOf("open",0)>-1){  // Abre o cierra
       openEnable = true; // Se puede sustituir por la llamada a la función
    }
    else if(tmp.indexOf("close",0)>-1){
      closeEnable = true;
    }
    count++;
    
    if (endIndex == len - 1) break; // Fin del While
  }

// Esto es lo que se puede sustituir por la funcion
  //Acciones a realizar tras el filtrado del puerto serie
  if(closeEnable){
     close_grip();
  }else if(openEnable){
     open_grip();
  }

  Serial.println("OK");
  buffer = "";
}


// Pongo los steppers a la velocidad que le digo, maxima y aceleración
//Establecer velocidad de los motores
void setSpeedConfiguration(float c_speed, float max_speed, float accel){
    for(int i=0;i<3;i++){
        steppers[i].setSpeed(c_speed);
        steppers[i].setMaxSpeed(max_speed);
        steppers[i].setAcceleration(accel);
    }
}


//Apertura y cierre de la pinza
void runServo(int index,int angle){
  svs[index].write(angle);
}

// Le doy 120 grados para que cierre
void close_grip(){
 runServo(0,120);
 delay(100);
}

void open_grip(){
 runServo(0,0);
 delay(100);
}

//Transformación de String a float
float stringToFloat(String s){
  float f = atof(s.c_str());
  return f;
}


//******FUNCIONES A IMPLEMENTAR POR EL GRUPO DE ALUMNOS****//

void move_steps(int motor_index, int steps){
  for (int i = 0; i<abs(steps); i++){
    steppers[motor_index].step();
    delay(10);
  }
}


//Busqueda de límites del robot

//Límites eje 1 - establecerlo en -90,90

//90/1.8 = 50, 180/1.8 = 100
void reset_stepper0(float qlim[2]){  
  move_steps(0,qlim[0]*GEAR_2*STEPS/1.8);

  steppers[0].setSpeed(-testSpeed);
  delay(2000);

  move_steps(0,qlim[0]*GEAR_2*STEPS/1.8);
  move_steps(0,qlim[1]*GEAR_2*STEPS/1.8);
  
  steppers[0].setSpeed(testSpeed);
  delay(2000);
}

//Límites eje 2 - ejemplo
void reset_stepper1(){
  
  int count_steps1 = 0; // Cuenta el número de pasos girando hacia la derecha
  int count_steps2 = 0; // Cuenta el número de pasos girando hacia la izquierda
  steppers[1].setSpeed(testSpeed);  // Situamos la velocidad de dicho motor positiva, para que gire hacia la derecha
  bool exit1=true;
  bool exit2=true;
  
  while(exit1){
    if(digitalRead(pin1)==LOW){ // Si el bumper no se encuentra pulsado
      steppers[1].step(); // Movemos un paso el motor
      delay(10);
      count_steps1++; // Aumentamos el número de pasos que se han dado para llegar
    }
    else{ // Si el bumper se encuentra pulsado
      qlimit_1[0] = count_steps1*1.8/(GEAR_2*STEPS);  // Convertimos el número de pasos dados en el ángulo límite positivo
      Serial.println(qlimit_1[0]);  // Guardamos el límite
      exit1=false;  // Nos salimoss del bucle
    }
  }
  
  steppers[1].setSpeed(-testSpeed); // Ponemos la velocidad negativa, para que gire hacia la izquierda
  delay(2000);
  
  // Se repite el mismo bucle
  while(exit2){
    if(digitalRead(pin1)==LOW){
      steppers[1].step();
      delay(10);
      count_steps2++;
    }
    else{
      qlimit_1[1] = -(count_steps2-count_steps1)*1.8/(GEAR_2*STEPS);  // ----------> Atento con el signo negativo
      Serial.println(qlimit_1[1]);
      exit2=false;
    }
  }

  steppers[1].setCurrentPosition(-(count_steps2-count_steps1));
  delay(1000);
  move_q2(0.0);
  delay(1000);
}

//Límites eje 3
void reset_stepper2(){
      
  int count_steps1 = 0;
  int count_steps2 = 0;
  steppers[2].setSpeed(testSpeed);
  bool exit1=true;
  bool exit2=true;
  
  while(exit1){
    if(digitalRead(pin2)==LOW){
      steppers[2].step();
      delay(10);
      count_steps1++;
    }
    else{
      qlimit_2[0] = count_steps1*1.8/(GEAR_2*STEPS);
      Serial.println(qlimit_2[0]);
      exit1=false;
    }
  }
  
  steppers[2].setSpeed(-testSpeed);
  delay(2000);
  
  while(exit2){
    if(digitalRead(pin2)==LOW){
      steppers[2].step();
      delay(10);
      count_steps2++;
    }
    else{
      qlimit_2[1] = -(count_steps2-count_steps1)*1.8/(GEAR_2*STEPS);
      Serial.println(qlimit_2[1]);
      exit2=false;
    }
  }

  steppers[2].setCurrentPosition(-(count_steps2-count_steps1));
  delay(1000);
  move_q3(0.0);
  delay(1000);
}


//Punto inicial y vuelta a la posición de home
void setHome(double x,double y,double z){
  home.x = x;
  home.y = y;
  home.z = z;
}

void setHome(Vector3 h){
  home = h;
}

void goHome(){
  moveToPoint(home);
}


//Cinemática directa. Movimiento en q1,q2,q3, en grados
void move_q1(float q1){

}

void move_q2(float q2){
  
}

void move_q3(float q3){
  
}

void moveToAngles(float q1, float q2, float q3){ // mover los 3 a una posicion
  
}

Vector3 forwardKinematics (float q1, float q2, float q3){ // Cinematica directa
  
}


//Cinemática inversa. Movimiento en x,y,z
void moveToPoint(Vector3 point){
  
}

Vector3 inverseKinematics(float x,float y,float z){
 
}


//Trayectoria y tarea p&p
void trajectory (float q1, float q2, float q3, float t){
  
}

void pick_and_place (){ // Tarea

}
