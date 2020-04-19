function [Q] = XYZkine(FK)
%DRAWKINE Summary of this function goes here
%   Detailed explanation goes here
%P=[FK(1:3,4) FK(1:3,8) FK(1:3,12) FK(1:3,16)];

Q1=[0 FK(1,4) FK(1,8) FK(1,12) FK(1,16) FK(1,20) FK(1,24)];
Q2=[0 FK(2,4) FK(2,8) FK(2,12) FK(2,16) FK(2,20) FK(2,24)];
Q3=[0 FK(3,4) FK(3,8) FK(3,12) FK(3,16) FK(3,20) FK(3,24)];
Q=[Q1;Q2;Q3];
end

