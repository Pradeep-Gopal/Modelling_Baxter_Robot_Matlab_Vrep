clc;
clear all;

%%Baxter 6 DOF Forward and Inverse Kinematics Validation%%

syms q1 q2 q3 q4 q5 q6 q7 alpha1 alpha2 alpha3 alpha4 alpha5 
syms alpha6 alpha7 L0 L1 L2 L3 L4 L5 L6 L7 Lh a1 a2 a3 a4 a5 d1 d2 d3 d4 
syms d5 d6 d7 

%%Input Joint Variables

disp('Input Joint Angles to compute the Transformation matrix')

q1=45;
q2=10;
q4=45;
q5=22;
q6=45;
q7=10;
 
Q=[q1,q2,q4,q5,q6,q7]
  
%%Link Lengths
L0=270;
L1=69;
L2=364;
L3=69;
L4=375;
L5=0;
L6=368;
Lh=370.48;

q1=q1;
q2=q2;
q4=q4+90;
q5=q5;
q6=q6;
q7=q7;

alpha1=-90;
alpha2=0;
alpha4=90;
alpha5=-90;
alpha6=90;
alpha7=0;

a1=L1;
a2=Lh;
a4=0;
a5=L5;
a6=0;
a7=0;

d1=L0;
d2=0;
d4=0;
d5=L4;
d6=0;
d7=L6;


%%0-1 matrix 
Rz_theta1=[cosd(q1) -sind(q1) 0 0;sind(q1) cosd(q1) 0 0;0 0 1 0;0 0 0 1];
Rx_alpha1=[1 0 0 0;0 cosd(alpha1) -sind(alpha1) 0;0 sind(alpha1) cosd(alpha1) 0;0 0 0 1];
Tz_d1=[1 0 0 0;0 1 0 0;0 0 1 d1;0 0 0 1];
Tx_a1=[1 0 0 a1;0 1 0 0;0 0 1 0;0 0 0 1];

%%1-2 matrix 
Rz_theta2=[cosd(q2) -sind(q2) 0 0;sind(q2) cosd(q2) 0 0;0 0 1 0;0 0 0 1];
Rx_alpha2=[1 0 0 0;0 cosd(alpha2) -sind(alpha2) 0;0 sind(alpha2) cosd(alpha2) 0;0 0 0 1];
Tz_d2=[1 0 0 0;0 1 0 0;0 0 1 d2;0 0 0 1];
Tx_a2=[1 0 0 a2;0 1 0 0;0 0 1 0;0 0 0 1];

%%2-4 matrix 
Rz_theta4=[cosd(q4) -sind(q4) 0 0;sind(q4) cosd(q4) 0 0;0 0 1 0;0 0 0 1];
Rx_alpha4=[1 0 0 0;0 cosd(alpha4) -sind(alpha4) 0;0 sind(alpha4) cosd(alpha4) 0;0 0 0 1];
Tz_d4=[1 0 0 0;0 1 0 0;0 0 1 d4;0 0 0 1];
Tx_a4=[1 0 0 a4;0 1 0 0;0 0 1 0;0 0 0 1];

%%4-5 matrix 
Rz_theta5=[cosd(q5) -sind(q5) 0 0;sind(q5) cosd(q5) 0 0;0 0 1 0;0 0 0 1];
Rx_alpha5=[1 0 0 0;0 cosd(alpha5) -sind(alpha5) 0;0 sind(alpha5) cosd(alpha5) 0;0 0 0 1];
Tz_d5=[1 0 0 0;0 1 0 0;0 0 1 d5;0 0 0 1];
Tx_a5=[1 0 0 a5;0 1 0 0;0 0 1 0;0 0 0 1];

%%5-6 matrix 
Rz_theta6=[cosd(q6) -sind(q6) 0 0;sind(q6) cosd(q6) 0 0;0 0 1 0;0 0 0 1];
Rx_alpha6=[1 0 0 0;0 cosd(alpha6) -sind(alpha6) 0;0 sind(alpha6) cosd(alpha6) 0;0 0 0 1];
Tz_d6=[1 0 0 0;0 1 0 0;0 0 1 d6;0 0 0 1];
Tx_a6=[1 0 0 a6;0 1 0 0;0 0 1 0;0 0 0 1];

%%6-7 matrix 
Rz_theta7=[cosd(q7) -sind(q7) 0 0;sind(q7) cosd(q7) 0 0;0 0 1 0;0 0 0 1];
Rx_alpha7=[1 0 0 0;0 cosd(alpha7) -sind(alpha7) 0;0 sind(alpha7) cosd(alpha7) 0;0 0 0 1];
Tz_d7=[1 0 0 0;0 1 0 0;0 0 1 d7;0 0 0 1];
Tx_a7=[1 0 0 a7;0 1 0 0;0 0 1 0;0 0 0 1];



%%multiplying to get homogeneous matrices
A_1 = (Rz_theta1*Tz_d1*Tx_a1*Rx_alpha1);
A_2 =(Rz_theta2*Tz_d2*Tx_a2*Rx_alpha2);
A_4 =(Rz_theta4*Tz_d4*Tx_a4*Rx_alpha4);
A_5 =(Rz_theta5*Tz_d5*Tx_a5*Rx_alpha5);
A_6 =(Rz_theta6*Tz_d6*Tx_a6*Rx_alpha6);
A_7 =(Rz_theta7*Tz_d7*Tx_a7*Rx_alpha7);


%Transformation matrices 
T01=A_1;
T02=A_1*A_2;
T04=A_1*A_2*A_4;
%T04=simplify(T04)
T05=A_1*A_2*A_4*A_5;
%T05=simplify(T05)
T06=A_1*A_2*A_4*A_5*A_6;
disp('Final Transformation matrix for the input joint angles')
T07=A_1*A_2*A_4*A_5*A_6*A_7

%Computing Theta5, Theta6, Theta7
R04=T04(1:3,1:3);
D04=T04(1:3,4);
R04T=transpose(R04);
R04TD04= R04T*D04;
T04inv=[R04T -R04T*D04;0 0 0 1];
%T04inv=simplify(T04inv);
T47=T04inv*T07;
%T47=simplify(T04inv*T07) 

%Computing theta5
r23=T47(2,3);
r13=T47(1,3);
theta5=(atan2(r23,r13))*180/3.14;

%Computing theta 7
r32=T47(3,2);
r31=T47(3,1);
theta7=(atan2(r32,-r31))*180/3.14;

%Computing theta6
r33=T47(3,3);
theta6=(atan2((-r31/cosd(q7)),r33))*180/3.14;

%Computing theta1
x05=T05(1,4);
%x05=simplify(x05)
y05=T05(2,4);
%y05=simplify(y05)
z05=T05(3,4);

%z05=simplify(z05)
theta1=(atan2(y05,x05))*180/3.14;

%Computing theta2
E = 2*Lh*(L1-(x05/cosd(q1)));
F = 2*Lh*(z05-L0);
G = (x05/cosd(q1))^2+(L1^2)+(Lh^2)+(z05^2)+(L0^2)-(L4^2)-((2*x05*L1)/cosd(q1))-2*z05*L0;
t = (-F-sqrt((E^2)+(F^2)-(G^2)))/(G-E);
theta2 = 2*atan(t);
teta2=theta2*180/3.14;


%Computing theta4
theta4 = (atan2(L0-z05-Lh*sind(q2),(x05/cosd(q1))-L1-(Lh*cosd(q2)))-theta2)*180/3.14;


%Final joint variables
j1=theta1;
j2=teta2;
j4=theta4;
j5=theta5;
j6=theta6;
j7=theta7;

disp('Joint angles calculated from the Transformation matrix above')
Joint_Angles = [j1,j2,j4,j5,j6,j7]






