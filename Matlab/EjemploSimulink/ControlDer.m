function res = ControlDer(y)

global Kd Tm yDant

res = -(Kd / Tm) * (y - yDant);
yDant = y;
