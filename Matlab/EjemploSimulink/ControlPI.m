function res = ControlPI(e)

global Kp Ki Tm Imax uIant

uI = min(max(uIant + Tm * Ki * e, -Imax), Imax);
uIant = uI;
res = Kp * e + uI;
