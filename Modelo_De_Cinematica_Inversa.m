% Modelo de cinemática inversa

% Generamos el robot

%% Tabla generada por mí

% Articulación 1
L1 = Link([0,150,0,deg2rad(90)]);
L1.qlim = [deg2rad(-90),deg2rad(90)]; % Introducimos el qlim por separado

% Articulación 2
L2 = Link([0,0,155,0]);
L2.offset = deg2rad(90);

% Articulación 3
L3 = Link([0,0,200,0]);

% Unión de articulaciones
L = [L1 L2 L3];

%% Tabla generada por David

% Articulación 1
L1 = Link([0,150,0,deg2rad(-90)]);
L1.qlim = [deg2rad(-90),deg2rad(90)]; % Introducimos el qlim por separado

% Articulación 2
L2 = Link([0,0,155,0]);
L2.offset = deg2rad(-90);

% Articulación 3
L3 = Link([deg2rad(90),0,200,0]);

% Unión de articulaciones
L = [L1 L2 L3];

%% Tabla generada por Carles

% Articulación 1
L1 = Link([0,150,0,deg2rad(-90)]);
L1.qlim = [deg2rad(-90),deg2rad(90)]; % Introducimos el qlim por separado

% Articulación 2
L2 = Link([0,0,155,0]);
L2.offset = deg2rad(-90);

% Articulación 3
L3 = Link([0,0,200,deg2rad(-90)]);
L3.offset = deg2rad(90);

% Unión de articulaciones
L = [L1 L2 L3];

%% Generación del robot

% Creamos el robot mediante la unión en serie de las articulaciones
% contenidas en el vector.
% Además, le otorgamos un nombre al robot
robot = SerialLink(L,'name','PAPI');

% Definimos el vector de q con tantas q como grados de libertad
q = [0 0 0];
robot.tool = transl(65,45,0);

% Ploteamos el robot mediante el vector de q
%robot.plot(q);

%% Generamos la cinemática directa

% Definimos las posiciones articulares para el PAPI
qe = [0, deg2rad(30), deg2rad(90)];
% Obtenemos la matriz de transformación
T = robot.fkine(qe)

% Calculamos las coordenadas mediante el método inverso
qe_inversa = robot.ikunc(T);  % Este método es el exacto

%% Matrices de transformación homogénea

q1 = 0;
q2 = deg2rad(30);
q3 = deg2rad(90);
l1 = 150;
l2 = 155;
l3 = 200;

r = l2*sin(q2)+(268.7936)*sin(q3+q2+0.1682);
effector =[r*cos(q1) r*sin(q1) l1+l2*cos(q2)+(268.7936)*cos(q3+q2+0.1682)];
cosq3 = (effector(1)^2 + effector(2)^2 + effector(3)^2 - l2^2 - l3^2)/(2*l2*l3);

qe_inv = [atan(effector(2)/effector(1)) atan(effector(3)/(sqrt(effector(1)^2+effector(2)^2)))-atan((l3*(sqrt(1-cosq3^2)/cosq3))/(l2+l3*cosq3)) atan(sqrt(1-cosq3^2)/cosq3)];
 
 T_inv = robot.fkine(qe_inv)
 
% %effector =[l3*cos(q1)*cos(q2)*cos(q3)-l3*cos(q1)*sin(q2)*sin(q3)+l2*sin(q1)*cos(q2) l3*sin(q1)*cos(q2)*cos(q3)-l3*sin(q1)*sin(q2)*sin(q3)+l2*cos(q1)*cos(q2) l3*sin(q2)*cos(q3)+l3*cos(q2)*sin(q3)+l1+l2*sin(q2)];
% 
% effector = [(l2*cos(q2)+l3*cos(q2+q3))*cos(q1) (l2*cos(q2)+l3*cos(q2+q3))*sin(q1) l1-sin(q2)*l2-l3*sin(q2+q3)];
% 
% qe_inv = [atan(effector(2)/effector(1)) atan(effector(3)/(sqrt(effector(1)^2+effector(2)^2)))-atan((l3*sin(atan(sqrt(1-cosq3^2)/cosq3)))/(l2+l3*cosq3)) atan(sqrt(1-cosq3^2)/cosq3)];
% 
% T_inv = robot.fkine(qe_inv)
