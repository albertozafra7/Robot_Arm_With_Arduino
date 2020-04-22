%% Setup

L1=Link([0 150 0 pi/2]);L1.qlim = [-pi/2 pi/2];
L2=Link([0 0 155 0]);L2.offset=pi/2;
L3=Link([0 0 200 0]);

L=[L1 L2 L3];
robot=SerialLink(L, 'name', 'P2');
robot.tool=transl(65, 45, 0)

q=[0 0 0];
robot.plot(q);
%% Forward kinematics
T=robot.fkine(q);

%% Inverse kinematics
Q=robot.ikunc(T);

%% Trajectories
q_1 = [deg2rad(67.2) deg2rad(73.5) deg2rad(53.2)];

q_2 = [deg2rad(31.7) deg2rad(22.8) deg2rad(-17.7)];

q_3=[deg2rad(-52) deg2rad(134) deg2rad(-33)];

qt = mtraj(@lspb, q_1, q_2,100);

qt_2= mtraj(@lspb, q_2, q_3,100);
robot.plot(qt)
robot.plot(qt_2)



