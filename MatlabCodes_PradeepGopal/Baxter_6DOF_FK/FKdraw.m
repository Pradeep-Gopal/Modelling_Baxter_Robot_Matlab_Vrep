function [ FK ] = FKdraw( j1,j2,j4,j5,j6,j7)

%initial parameter
%[j0 j1 j2 j3;d0 d1 d2 d3;a0 a1 a2 a3;t0 t1 t2 t3]

% D-H Parameters
a1 = 69; % length of first arm
a2 = 0; % length of second arm
a3 = 69; % length of third arm
a4 = 0; % length of fourth arm
a5 = 10; % length of fifth arm
a6 = 0; % length of sixth arm
a7 = 0; % length of seventh arm
d1 = 270; % offset of first arm
d2 = 0; % offset of second arm
d3 = 364; % offset of third arm
d4 = 0; % offset of fourth arm
d5 = 375; % offset of fifth arm
d6 = 0; % offset of sixth arm
d7 = 368; % offset of seventh arm
Lh=370.48
j=[j1 j2 j4+90 j5 j6 j7;d1 0 0 d5 0 d7;a1 Lh 0 a5 0 0;-90 0 90 -90 90 0];

FK=DHkine(j);
Q=XYZkine(FK);

h=plot3(Q(1,:),Q(2,:),Q(3,:),'-o','LineWidth',2,'MarkerSize',6,'MarkerFaceColor',[0.5,0.5,0.5]);



grid on;
text(Q(1,7),Q(2,7),Q(3,7),['  (', num2str(Q(1,7),3), ', ', num2str(Q(2,7),3),', ', num2str(Q(3,7),3), ')']);
title('(Baxter Arm)')
xlabel('X Axis');
ylabel('Y Axis');
zlabel('Z Axis');
axis([-1250 1250 -1500 4000 -750 1000]);
h = rotate3d;
h.Enable = 'on';
h.ActionPostCallback = @mypostcallback;
assignin('base','FK',FK);

end

function mypostcallback(obj,evd)
%disp('A rotation is about to occur.');
ax_properties = get(gca);
assignin('base','pov',ax_properties.View);
end

%use evalin('base',a) to get variable a from workspace 
%assignin('base','a_rms',a_rms) to write variable a_rms to workspace