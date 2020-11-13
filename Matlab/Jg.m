function J=Jg(q)
 T=FK(q) ;
% Geometrical Jacobian
disp('Geometrical Method')

O=T(1:3,4);
T1=Tz(0.4)*Rz(q(1))*Tx(0.025);
T2=T1*Ry(q(2))*Tz(0.56);
T3=T2*Ry(q(3))*Tz(0.025)*Tx(0.515);
T4=T3*Rx(q(4));
T5=T4*Ry(q(5));
R1=T1(1:3,1:3);
R2=T2(1:3,1:3);
R3=T3(1:3,1:3);
R4=T4(1:3,1:3);
R5=T5(1:3,1:3);

% The first joint (revolute)
O0 = [0 0 0]';
z0 = [0 0 1]';
Jv1 = cross(z0, O - O0);
Jw1 = z0;

% The second joint (revolute)
O1=T1(1:3,4);
z1 = R1*[0 1 0]';
Jv2 = cross(z1, O - O1);
Jw2 = z1;

% The third joint (revolute)
O2=T2(1:3,4);
z2 = R2*[0 1 0]';
Jv3 = cross(z2, O - O2);
Jw3 = z2;

% The fourth joint (revolute)
O3=T3(1:3,4);
z3 = R3*[1 0 0]';
Jv4 = cross(z3, O - O3);
Jw4 = z3;

% The fifth joint (revolute)
O4=T4(1:3,4);
z4 = R4*[0 1 0]';
Jv5 = cross(z4, O - O4);
Jw5 = z4;

% The sixth joint (revolute)
O5=T5(1:3,4);
z5 = R5*[1 0 0]';
Jv6 = cross(z5, O - O5);
Jw6 = z5;


Jv = [Jv1, Jv2, Jv3,Jv4, Jv5, Jv6];
Jw = [Jw1, Jw2, Jw3,Jw4, Jw5, Jw6];
J = [Jv; Jw];
end
