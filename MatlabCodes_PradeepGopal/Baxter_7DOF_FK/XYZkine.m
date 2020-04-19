function [Q] = XYZkine(FK)
%DRAWKINE Summary of this function goes here

Q1=[0 FK(1,4) FK(1,8) FK(1,12) FK(1,16) FK(1,20) FK(1,24) FK(1,28) FK(1,32) FK(1,36) FK(1,40)];
Q2=[0 FK(2,4) FK(2,8) FK(2,12) FK(2,16) FK(2,20) FK(2,24) FK(2,28) FK(2,32) FK(2,36) FK(2,40)];
Q3=[0 FK(3,4) FK(3,8) FK(3,12) FK(3,16) FK(3,20) FK(3,24) FK(3,28) FK(3,32) FK(3,36) FK(3,40)];
Q=[Q1;Q2;Q3];
end

