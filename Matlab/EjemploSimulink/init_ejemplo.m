% Parametros del ejemplo (se corre como InitFcn del modelo)
global Kp Ki Kd Tm Imax umax idle vmax uIant yDant

%% Controlador (rama SCD, unidades SI)
Tm   = 0.5e-3;                    % PID a 2 kHz
Kp   = 0.032029*40/1000*180/pi;   % 0.0734
Ki   = 0.244381*60/1000*180/pi;   % 0.840
Kd   = 0.000529*20/1000*180/pi;   % 6.06e-4
Imax = 0.05;                      % limite de la I
umax = 0.08;                      % saturacion de tau
idle = 0.04;  vmax = 0.20;        % ventana de cada motor
Tbase = 0.10;                     % throttle base
Kth  = 4;                         % PID 1, 1/s
wmax = deg2rad(120);              % limite de omega_ref

uIant = 0;  yDant = 0;

%% Planta (SI)
m = 0.0336;  M = 0.120;  L = 0.34;  l = L/2;
J  = 2*m*l^2 + M*L^2/12;          % kg m^2
Ap = [0 1; 0 0];
Bp = [0; l/J];
Cp = eye(2);
Dp = [0; 0];

%% Motor + helice (identificado en banco, tf_matlab_50.m)
% entrada: throttle en % (0..100), salida: empuje del banco (se asume N)
% 1 polo (50%):  Gm_num = 0.2633;  Gm_den = [1 1.603];
% 2 polos:       Gm_num = 1.535;   Gm_den = [1 4.941 9.804];
% Gs2:           Gm_num = 2.337;   Gm_den = [1 6.807 23.65];
% Gs1:           Gm_num = 0.3173;  Gm_den = [1 3.153];
% Tramo 0-1555 (escalon 80 %), G2: Gm_num = 0.9874;  Gm_den = [1 4.818 10.11];
Gm_num = 8630;
Gm_den = [1 268.5 33190];

%% Resintonia del lazo de omega para un motor de 1 polo (asignacion de polos)
% tau -> omega: b / (s (s + a)), con D sobre la medicion
% polinomio de lazo cerrado: s^3 + (a + b Kd) s^2 + b Kp s + b Ki
resintonizar = false;
if resintonizar && numel(Gm_den) == 2
    a  = Gm_den(2);
    b  = 200 * Gm_num * l / J;       % 2 motores, fraccion -> %
    wn = 20;  z = 0.8;  p3 = 5;      % polos deseados: wn, z y uno real en -p3
    Kd = (2*z*wn + p3 - a) / b;
    Kp = (wn^2 + 2*z*wn*p3) / b;
    Ki = wn^2 * p3 / b;
end

%% Referencia
ref = deg2rad(10);
