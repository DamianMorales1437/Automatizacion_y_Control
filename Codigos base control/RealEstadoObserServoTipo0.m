% REALIMENTACION DE ESTADO SERVOSISTEMA TIPO 0 OBSERVADOR COMPLETO

% si me dan la fdt de la planta (ej)
s = tf('s');
G = (13*(s^2 + 2.5*s +2.5)) / ((s+0.88)*(s+4.5)*(s+13.2));

% para conocer los valores del ov% y ts del sistema realimentado sin
% controlador
step(feedback(G,1))

% Definicion de las matrices
sys = ss(G);
A = sys.A;
B = sys.B;
C = sys.C;
D = sys.D;

% Definicion de las matrices transformadas (tipo 0 a tipo 1)
[~,n] = size(A);
Ahat = [A zeros(n,1); -C 0];
Bhat = [B;0];

% Definicion vector depolos deseados (para cumplir con el overshoot)
J = [-2.66+3i -2.66-3i -2 -2];

% Calculo de las ganancias de realimentacion
Khat = acker(Ahat,Bhat,J);
[~,m] = size(Khat);

% K de realimentacion
K = Khat(1:m-1);

% Valor de Ki (SIMULINK)
Ki = -Khat(m);

% Matrices del nuevo sistema realimentado tipo0 a tipo 1
AA = [A-B*K B*Ki;-C 0];
BB = [0;0;0;1];
CC = [C 0];
DD = 0;

% para ver la respuesta al escalon del sistema realimentado
step(AA,BB,CC,DD)

% Calculo de ganancias de realimentacion del observador (Ke)
Ke = acker(A',C',[-30 -30 -30])';

% Matrices del Sistema Observador
A_ = A-Ke*C-B*K;
B_ = Ke;
C_ = K;
D_ = 0;

% espacio de estados del observador
sys = ss(A_,B_,C_,D_);

% Hallando el numerador y denominador fdt 
[num,den] = ss2tf(A_,B_,C_,D_);

% FDT del controlador:
fdt_sys = tf(num,den);


