function [ FK ] = DHkine(j)
%collum 1=joint angle, 2=joint offset, 3=link lenght, 4=twist angle
T01=DHmatrix(j(1,1),j(2,1),j(3,1),j(4,1));
T12=DHmatrix(j(1,2),j(2,2),j(3,2),j(4,2));
T24=DHmatrix(j(1,3),j(2,3),j(3,3),j(4,3));
T45=DHmatrix(j(1,4),j(2,4),j(3,4),j(4,4));
T56=DHmatrix(j(1,5),j(2,5),j(3,5),j(4,5));
T67=DHmatrix(j(1,6),j(2,6),j(3,6),j(4,6));


T02=T01*T12;
T04=T02*T24;
T05=T04*T45;
T06=T05*T56;
T07=T06*T67

FK=[T01 T02 T04 T05 T06 T07];
end

