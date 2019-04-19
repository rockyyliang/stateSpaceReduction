%3 mass controllability/observability probblem
clear; close all;
m1 = 1; m2 = 1; m3 = 2;
k1 = 10; k2 = 10; k3 = 10;
b1 = 5; b2 = 3; b3 = 1;

A = [0 1 0 0 0 0;
    -k1/m1 -b1/m1 k1/m1 b1/m1 0 0;
    0 0 0 1 0 0;
    k1/m2 b1/m2 -(k1+k2)/m2 -(b1+b2)/m2 k2/m2 b2/m2;
    0 0 0 0 0 1;
    0 0 k2/m3 b2/m3 -(k2+k3)/m3 -(b2+b3)/m3];
B = [0 1/m1 0 0 0 0]';
C = [1 0 0 0 0 0];

%confirm controllability/observability
P = ctrb(A,B);
detp = det(P);
Q = obsv(A,C);
detq = det(Q);

%find grammian, can also use lyap()
%lyap(A,BB') lyap(A',C'C)
sys = ss(A,B,C,0);
wc = gram(sys,'c');
hc = gram(sys,'o');

%solve balanced Wb and Hb
R = chol(hc);
[u,s,v] = svd(R*wc*(R'));
%S = epsilon^2
Tb = (s^(-.25))*u'*R;
Wb = Tb*wc*Tb';
Hb = inv(Tb')*hc*inv(Tb);
%check Wb and Hb, their diag should be the same as g
[sysb,g] = balreal(sys);

%plot system response
tvec = 0:0.001:5;
impulse(sys,tvec);

%find transform T for system reduction
%Abar = inv(T)AT
%since P is full rank, T = P
T = Tb;
Abar = T*A*inv(T);
Bbar = T*B;
Cbar = C*inv(T);

for i = 0:1
    %partition system matrices
    %n is the amount of dimensions to keep
    %loop thru different n values to check model validity
    n = 4-i;
    Arr = Abar(1:n,1:n);
    Br = Bbar(1:n);
    Cr = Cbar(1:n);
    Add = Abar(n+1:end,n+1:end);
    Bd = Bbar(n+1:end);
    Cd = Cbar(n+1:end);
    Ard = Abar(1:n,n+1:end);
    Adr = Abar(n+1:end,1:n);
    %reduced system matrices
    Ared = Arr - Ard*inv(Add)*Adr;
    Bred = Br - Ard*inv(Add)*Bd;
    Cred = Cr - Cd*inv(Add)*Adr;
    sysr = ss(Ared,Bred,Cred,0);
    hold on;
    impulse(sysr,tvec);
end
legend('original','discard 2 states','discard 3 states');
