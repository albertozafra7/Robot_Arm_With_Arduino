#include "MeMegaPi.h"
#include <Servo.h>

//************************************* Declaración de las variables a utilizar *************************************

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

// Definimos la variable home
Vector3 home = {0.0, 0.0, 0.0};

// Definimos la variable que guarda si la tarea a realizar es la predeterminada o no
bool pick = false;

// Definimos la variable que guarda el modo de introducción de los datos
bool cart = false;

// Definimos la variable que guarda si el robot se encuentra preguntando por la introducción de una posición intermedia
bool StepAsk = false;

// Definimos la variable que guarda si el robot debe pasar por una posición intermedia
bool MustPass = false;


//************************************* Declaración de los prototipos de las funciones *************************************

//----------- Funciones preestablecidas -----------
void parseBuffer(); // Filtra los mensajes recibidos por el monitor serie
void setSpeedConfiguration(float c_speed, float max_speed, float accel); // Configuración de la velocidad de los motores
void runServo(int index,int angle); // Movimiento de un servo un determinado ángulo
void close_grip();  // Cierre de la pinza situada en la muñeca del robot
void open_grip();   // Apertura de la pinza situada en el actuador final
float stringToFloat(String s);  // Conversión de una cadena de carácteres a una variable tipo float


//----------- Funciones desarrolladas -----------
void move_steps(int motor_index, int steps);  // Movimiento de un motor paso a paso un número determinado de pasos
void reset_stepper0(float qlim[2]);   // Establecemos los límites del motor situado en el primer eje
void reset_stepper1();  // Establecemos los límites del motor situado en el segundo eje
void reset_stepper2();  // Establecemos los límites del motor situado en el tercer eje
void setHome(double x,double y,double z); // Establecemos nuestro sistema de referencia
void setHome(Vector3 h);  // Sobrecarga del setHome
void goHome();  // Posicionamiento del robot en el origen del sistema de referencia previamente establecido
void move_q1(float q1); // Movimiento del primer eje a la posición angular q1
void move_q2(float q2); // Movimiento del segundo eje a la posición angular q2
void move_q3(float q3); // Movimiento del tercer eje a la posición angular q3
void moveToAngles(float q1, float q2, float q3);  // Movimiento de los tres ejes a unas posiciones angulares preestablecidas
Vector3 forwardKinematics (float q1, float q2, float q3); // Cálculo de la cinemática directa del robot
void moveToPoint(Vector3 point);  // Movimiento de los tres ejes del robot para que el actuador final se encuentre en unas coordenadas específicas
Vector3 inverseKinematics(float x,float y,float z); // Cálculo de la cinemática inversa del robot
void trajectory (float q1, float q2, float q3, float t);  // Función que mueve el robot a una posición angular síncronamente
void pick_and_place (); // Función que realiza una tarea específica

//----------- Funciones desarrolladas como apoyo -----------
void Read();  // Lectura de los mensajes pasados por el monitor serie
void defaultPick(); // Realización de la tarea predeterminada
void designedPick(); // Realización de una tarea definida por el usuario
float dataRead(); // Lectura de los datos pasados a pick&place utilizados en designedPick()
void AddPos(Vector3*, size_t*, size_t*);  // Añade una posición intermedia a la tarea específica, en designedPick()
void AddPosAng(Vector3*, size_t *capacity, size_t *count);  // Añade una posición intermedia a la tarea específica, pero con posiciones angulares
void resize(size_t, Vector3*, size_t*, size_t); // Redimensiona el array de posiciones intermedias utilizado en designedPick()
void Remove(Vector3*,size_t *); // Eliminar el último elemento que contiene el array
void Trim(size_t, Vector3*, size_t*); // Reducción del array que contiene las posiciones intermedias de la tarea específica



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
    
    //******* Edición del parse buffer para el pick&place *******
    
    else if(tmp.indexOf("y",0)>-1 && !StepAsk) // El usuario desea realizar la tarea predeterminada
      pick = true;
    else if(tmp.indexOf("n",0)>-1 && !StepAsk) // El usuario desea realizar una tarea específica
      pick = false;
    else if((tmp.indexOf("coord",0)>-1) || tmp.indexOf("cart",0)>-1) || tmp.indexOf("cord",0)>-1))  // El usuario debe introducir las coordenadas cartesianas en la tarea específica
      cart = true;
    else if((tmp.indexOf("ang",0)>-1) || tmp.indexOf("pos",0)>-1))  // El usuario debe introducir las posiciones articulares en la tarea específica
      cart = false;
    else if(tmp.indexOf("y",0)>-1 && StepAsk) // El usuario desea introducir un paso intermedio en la tarea específica
      MustPass = true;
    else if(tmp.indexOf("n",0)>-1 && StepAsk) // El usuario no desea introducir un paso intermedio en la tarea específica
      MustPass = false;
    
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
  move_steps(0,qlim[0]*GEAR_1*STEPS/1.8);

  steppers[0].setSpeed(-testSpeed);
  delay(2000);

  move_steps(0,qlim[0]*GEAR_1*STEPS/1.8);
  move_steps(0,qlim[1]*GEAR_1*STEPS/1.8);
  
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
  
  if ( (q1 <= qlim[1]) && (q1 >= qlim[0])){   //Límite qlim = [-90,90]
    
    if(q1<0.0)
      steppers[0].setSpeed(-currentSpeed);
    else
      steppers[0].setSpeed(currentSpeed);
   
    move_steps(0,q1*GEAR_1*STEPS/1.8);
    //delay(2000); No se si hay que ponerlo
  } 
}

void move_q2(float q2){

    if ( (q2 <= qlimit_1[0]) && (q2 >= qlimit_1[1])){   //Límite qlimit = [+,-]
    
    if(q2<0.0)
      steppers[0].setSpeed(-currentSpeed);
    else
      steppers[0].setSpeed(currentSpeed);
   
    move_steps(0,q2*GEAR_2*STEPS/1.8);
    //delay(2000); No se si hay que ponerlo
  }   
}

void move_q3(float q3){
     
    if ( (q3 <= qlimit_2[0]) && (q3 >= qlimit_2[1])){   //Límite qlimit = [+,-]
    
    if(q3<0.0)
      steppers[0].setSpeed(-currentSpeed);
    else
      steppers[0].setSpeed(currentSpeed);
   
    move_steps(0,q2*GEAR_2*STEPS/1.8);
    //delay(2000);
  }  
}

void moveToAngles(float q1, float q2, float q3){ // mover los 3 a una posicion
  move_q1(q1);
  move_q2(q2);
  move_q3(q3);
}

Vector3 forwardKinematics (float q1, float q2, float q3){ // Cinematica directa
  Vector3 effector;
  float r=L2*sin(q2)+(268.7936)*sin(q3+q2+0.1682);
  effector.x=r*cos(q1);
  effector.y=r*sin(q1);
  effector.z=L1+L2*cos(q2)+(268.7936)*cos(q3+q2+0.1682);
  return effector;
}


//Cinemática inversa. Movimiento en x,y,z
void moveToPoint(Vector3 point){
  // Básicamente se calcula la cinemática inversa del punto
  Vector3 q = inverseKinematics(point.x,point.y,point.z);

  // Y movemos a dichos ángulos
  moveToAngles(q.x,q.y,q.z);
}

// Calculamos la cinemática inversa
Vector3 inverseKinematics(float x,float y,float z){
  // Generamos el vector que se va a devolver
  Vector3 q;

  // Calculamos la posición angular del primer eje
  q.x = atan(y/x);  // q1 = arcotangente(y/x)

  // Calculamos el coseno de q3, mediante el teorema del coseno y el teorema de pitágoras
  // El teorema del coseno es el siguiente: c^2 = a^2 + b^2 - a*b*cos(a^b)
  float cosq3 = (x^2 + y^2 + z^2 - l2^2 - l3^2)/(2*l2*l3);

  // Calculamos la posición angular del tercer eje, mediante el coseno y el seno de dicha posición
  // tan(q3) = sen(q3)/cos(q3), habiendo calculado anteriormente el coseno de q3 y obteniendo el seno a partir de la identidad trigonométrica sen^2 + cos^2 = 1
  q.z = atan(sqrt(1-cosq3^2)/cosq3);  // q3 = arcotangente(seno(q3)/coseno(q3))

  // Finalmente obtenemos la posición angular del segundo eje
  // Por medio de la resta de 2 ángulos, alfa y beta

  // Siendo alfa = atan(z/(sqrt(x^2+y^2)))
  // Beta = atan((l3*sin(atan(sqrt(1-cosq3^2)/cosq3)))/(l2+l3*cosq3))
  // q2 = alfa - beta
  q.y = atan(z/(sqrt(x^2+y^2)))-atan((l3*(sqrt(1-cosq3^2)/cosq3))/(l2+l3*cosq3)); // Dependiendo de si el valor de la raíz cuadrada contenida en alfa, se puede obtener la posición de codo arriba o codo abajo

  return q; 
}

//Trayectoria y tarea p&p
void trajectory (float q1, float q2, float q3, float t){
  
}

void pick_and_place (){ // Tarea

  // Enviamos un mensaje inicial
  Serial.println("¿Desea realizar la operacion por defecto? (Y/N): ");
  // Leemos la respuesta
  Read();

  if(pick)
    defaultPick();
  else
    designedPick();
}


//********************************************** Funciones extras **********************************************

// Función que lee los mensajes de pick&place
void Read(){
  char c; // Variable donde se va a ir evaluando carácter por carácter los mensajes enviados por Matlab
  
  while(Serial.available() <= 0){}
  while(Serial.available() > 0){
    c = Serial.read();  // Leo lo que hay
    if (c == '\n')  // Si lo que leo es un intro
      parseBuffer();  // Filtro el mensaje
    else  // Si no
      buffer += c;  // Introduzco en la variable buffer los datos
  }
}

// Función que realiza la tarea predeterminada
void defaultPick(){
  // Generamos el vector que va a contener el punto donde se debe coger el objeto
  Vector3 q_origin;
  q_origin.x = 67.2;
  q_origin.y = 73.5;
  q_origin.z = 53.2;

  // Generamos el vector que va a contener el punto intermedio
  Vector3 q_step;
  q_step.x = 31.7;
  q_step.y = 22.8;
  q_step.z = -17.7;

  // Generamos el vector que va a contener el punto final, donde se debe colocar el objeto
  Vector3 q_final;
  q_final.x = -52;
  q_final.y = 134;
  q_final.z = -33;


  // Seguidamente se posiciona el robot en el home
  goHome();

  // Se abre la pinza
  open_grip();

  // Se posiciona el robot en la posición donde tiene que coger el objeto, tiene que llegar en 5s
  trayectory(q_origin.x,q_origin.y,q_origin.z,5);

  // Se coge el objeto con la pinza
  delay(10);
  close_grip();

  // Se posiciona el robot en el punto intermedio
  trayectory(q_step.x,q_step.y,q_step.z,5);

  // Se posiciona el robot en el punto final
  trayectory(q_final.x,q_final.y,q_final.z,5);

  // Se coloca el objeto
  open_grip();
  delay(10);

  // Se vuelve al home
  goHome();
  
}

// Función que realiza una tarea específica
void designedPick(){
  // Se generan los vectores que van a contener las posiciones de la tarea
  Vector3 origin; // La posición donde se debe coger el objeto
  Vector3 finalpos; // La posición donde se debe de dejar el objeto

  // Se pregunta cómo se desean introducir las coordenadas del punto
  Serial.println("¿Desea trabajar con posiciones angulares o con coordenadas cartesianas? (ang/coord): ");

  // Se lee la respuesta
  Read();

  if(cart) {  // Si se trabaja con coordenadas cartesianas

    // Se pregunta por dichas coordenadas

    // Coordenadas donde se va a coger el objeto
    Serial.println("Introduzca las coordenadas cartesianas donde se va a coger el objeto: ");
    
    // Posición X
    Serial.println("X = ");
    origin.x = dataRead();
    

    // Posición Y
    Serial.println("Y = ");
    origin.y = dataRead();

    // Posición Z
    Serial.println("Z = ");
    origin.z = dataRead();


    // Coordenadas donde se va a colocar el objeto
    Serial.println("Introduzca las coordenadas cartesianas donde se va a colocar el objeto: ");

    // Coordenada X
    Serial.println("X = ");
    finalpos.x = dataRead();

    // Coordenada Y
    Serial.println("Y = ");
    finalpos.y = dataRead();

    // Coordenada Z
    Serial.println("Z = ");
    finalpos.z = dataRead();


    // Preguntamos si desea pasar por puntos intermedios
    Serial.println("¿Desea introducir algún punto intermedio? (y/n):");

    // Se establece el modo de introducción de una posición intermedia
    StepAsk = true;
    // Se lee la respuesta
    Read();

    if(MustPass){
      // Creamos un array dinámico de posiciones por si el usuario desea introducir más de una posición intermedia
      size_t capacity = 1; // Guarda la capacidad del array
      size_t count = 0; // Guarda la posición del array en la que nos encontramos
      Vector3* Passes = new Vector3[1];
      
      do {

        // Añadimos una posición en el array
        AddPos(Passes,&capacity,&count);
        // Preguntamos si se desea introducir una nueva posición
        Serial.println("¿Desea introducir otra posición intermedia?")

      } while(MustPass);

      // Reproducimos la tarea
      goHome(); // Nos situamos en el home
      open_grip();  // Abrimos la pinza
      // Vamos a la posición donde se debe coger el objeto
      trayectory(inverseKinematics(origin.x,origin.y,origin.z),5);
      delay(10);
      close_grip(); // Cerramos la pinza
  
      for(size_t i = 0; i < count; i++) // Pasamos por las posiciones intermedias
        trayectory(inverseKinematics(Passes[i].x,Passes[i].y,Passes[i].z),5);
  
      // Nos situamos en la posición final
      trayectory(inverseKinematics(finalpos.x,finalpos.y,finalpos.z),5);
  
      open_grip();  // Se coloca el objeto
      delay(10);
      goHome(); // Se vuelve al home

      StepAsk = false;

      size_t j = count; // Guardamos una copia del count
      for(size_t i = 0; i < j; i++) // Borramos el array entero
        Remove(Passes,&count,&capacity);

    } else {  // Si no se poseen posiciones intermedias
      // Reproducimos la tarea
      goHome(); // Nos situamos en el home
      open_grip();  // Abrimos la pinza
      // Vamos a la posición donde se debe coger el objeto
      trayectory(inverseKinematics(origin.x,origin.y,origin.z),5);
      delay(10);
      close_grip(); // Cerramos la pinza
 
      // Nos situamos en la posición final
      trayectory(inverseKinematics(finalpos.x,finalpos.y,finalpos.z),5);
  
      open_grip();  // Se coloca el objeto
      delay(10);
      goHome(); // Se vuelve al home

    }

  } else {  // Si se realiza con posiciones angulares
    
    // Se pregunta por dichas posiciones angulares

    // Coordenadas donde se va a coger el objeto
    Serial.println("Introduzca las posiciones angulares que va a tener el robot para poder coger el objeto: ");
    
    // Posición del eje 1
    Serial.println("q1 = ");
    origin.x = dataRead();
    

    // Posición del eje 2
    Serial.println("q2 = ");
    origin.y = dataRead();

    // Posición del eje 3
    Serial.println("q3 = ");
    origin.z = dataRead();


    // Coordenadas donde se va a colocar el objeto
    Serial.println("Introduzca las posiciones angulares que va a tener el robot para poder colocar el objeto: ");

    // Posición del eje 1
    Serial.println("q1 = ");
    finalpos.x = dataRead();

    // Posición del eje 2
    Serial.println("q2 = ");
    finalpos.y = dataRead();

    // Posición del eje 3
    Serial.println("q3 = ");
    finalpos.z = dataRead();


    // Preguntamos si desea pasar por puntos intermedios
    Serial.println("¿Desea introducir algún punto intermedio? (y/n):");

    // Se establece el modo de introducción de una posición intermedia
    StepAsk = true;
    // Se lee la respuesta
    Read();

    if(MustPass){
      // Creamos un array dinámico de posiciones por si el usuario desea introducir más de una posición intermedia
      size_t capacity = 1; // Guarda la capacidad del array
      size_t count = 0; // Guarda la posición del array en la que nos encontramos
      Vector3* Passes = new Vector3[1];
      
      do {

        // Añadimos una posición en el array
        AddPosAng(Passes,&capacity,&count);
        // Preguntamos si se desea introducir una nueva posición
        Serial.println("¿Desea introducir otra posición intermedia?")

      } while(MustPass);

      // Reproducimos la tarea
      goHome(); // Nos situamos en el home
      open_grip();  // Abrimos la pinza
      // Vamos a la posición donde se debe coger el objeto
      trayectory(origin.x,origin.y,origin.z,5);
      delay(10);
      close_grip(); // Cerramos la pinza
  
      for(size_t i = 0; i < count; i++) // Pasamos por las posiciones intermedias
        trayectory(Passes[i].x,Passes[i].y,Passes[i].z,5);
  
      // Nos situamos en la posición final
      trayectory(finalpos.x,finalpos.y,finalpos.z,5);
  
      open_grip();  // Se coloca el objeto
      delay(10);
      goHome(); // Se vuelve al home

      StepAsk = false;  // Dejamos de preguntar por las posiciones intermedias

      size_t j = count; // Guardamos una copia del count
      for(size_t i = 0; i < j; i++) // Borramos el array entero
        Remove(Passes,&count,&capacity);

    } else {  // Si no se poseen posiciones intermedias
      // Reproducimos la tarea
      goHome(); // Nos situamos en el home
      open_grip();  // Abrimos la pinza
      // Vamos a la posición donde se debe coger el objeto
      trayectory(origin.x,origin.y,origin.z,5);
      delay(10);
      close_grip(); // Cerramos la pinza
 
      // Nos situamos en la posición final
      trayectory(finalpos.x,finalpos.y,finalpos.z,5);
  
      open_grip();  // Se coloca el objeto
      delay(10);
      goHome(); // Se vuelve al home

    }
  }
  
}

// Función que lee los valores de pick&place
float dataRead(){
  float data = 0; // Variable que guarda el valor introducido
  char c; // Variable donde se va a ir evaluando carácter por carácter los mensajes enviados por Matlab
  
  while(Serial.available() <= 0){}
  while(Serial.available() > 0){
    c = Serial.read();  // Leo lo que hay
    if (c == '\n'){  // Si lo que leo es un intro
      data = stringToFloat(buffer);
      buffer = "";
    }else  // Si no
      buffer += c;  // Introduzco en la variable buffer los datos
  }

  return data;
}

// Añade una posición intermedia
void AddPos(Vector3* Passes, size_t *capacity, size_t *count){
  // Incrementamos el contador
  *count = *count+1;

  // Si el contador es mayor que la capacidad
  if (count > capacity){
    // Redimensionamos el array
    size_t newSize = (*capacity) * 2;
    resize(newSize,Passes,capacity,*count);
  } 
 
 // Preguntamos por sus coordenadas
 Serial.println("Introduzca sus coordenadas cartesianas: ");
  
 // Coordenada X
 Serial.println("X = ");
 Passes[*count-1].x = dataRead();
  
 // Coordenada Y
 Serial.println("Y = ");
 Passes[*count-1].y = dataRead();
  
 // Coordenada Z
 Serial.println("Z = ");
 Passes[*count-1].z = dataRead();
}

// Añade una posición intermedia
void AddPosAng(Vector3* Passes, size_t *capacity, size_t *count){
  // Incrementamos el contador
  *count = *count+1;

  // Si el contador es mayor que la capacidad
  if (count > capacity){
    // Redimensionamos el array
    size_t newSize = (*capacity) * 2;
    resize(newSize,Passes,capacity,*count);
  } 
 
 // Preguntamos por sus coordenadas
 Serial.println("Introduzca sus posiciones angulares: ");
  
 // Posición angular del eje 1
 Serial.println("q1 = ");
 Passes[*count-1].x = dataRead();
  
 // Posición angular del eje 2
 Serial.println("q2 = ");
 Passes[*count-1].y = dataRead();
  
 // Posición angular del eje 3
 Serial.println("Z = ");
 Passes[*count-1].z = dataRead();
}

// Redimensionamiento del array de posiciones
void resize(size_t newCapacity, Vector3* Passes, size_t* capacity, size_t count){

  // Generamos un array de posiciones con la nueva capacidad
  Vector3* newArray = new Vector3[newCapacity];
  // Mocemos todo lo contenido en el array anterior de posiciones al nuevo
  memmove(newArray, Passes, count  * sizeof(Vector3));
  // Borramos todo lo contenido en el array anterior
  delete[] Passes;
  // Guardamos la nueva capacidad del array
  *capacity = newCapacity;
  // Guardamos en la dirección de memoria del anterior array el nuevo array
  Passes = newArray;
}

// Eliminar el primer elemento del array
void Remove(Vector3* Passes,size_t *count, size_t* capacity){
  *count = *count - 1;  // Reducimos el contador

  // Redimensionamos el array
  Trim(*count,Passes,capacity);

}
 
// Reducción de la capacidad del array de posiciones
void Trim(size_t count, Vector3* Passes, size_t* capacity){
  resize(count,Passes,capacity,count);
}
