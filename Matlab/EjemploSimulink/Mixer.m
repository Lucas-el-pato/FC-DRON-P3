function res = Mixer(k)

global idle vmax umax

T   = k(1);                          % throttle base
tau = min(max(k(2), -umax), umax);   % salida de la saturacion

a   = abs(tau);
thr = min(max(T, idle), vmax);
if thr + a > vmax, thr = vmax - a; end
if thr - a < idle, thr = idle + a; end

vL = min(max(thr - tau, idle), vmax);   % M1, izquierda
vR = min(max(thr + tau, idle), vmax);   % M3, derecha
res = [vL; vR];
