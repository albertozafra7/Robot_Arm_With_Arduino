L1=Link([0 150 0 -pi/2]);
L2=Link([0 0 155 0]);L2.offset=-pi/2;
L3=Link([0 0 200 0]);

L=[L1 L2 L3];
robot=SerialLink(L, 'name', 'P2');
robot.tool=transl(65, 45, 0)

q=[pi/6 pi/5 -pi/6];
T=robot.fkine(q);
Q=robot.ikunc(T);
robot.plot(Q);



