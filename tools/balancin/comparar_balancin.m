function res = comparar_balancin(csvFile, carpetaModelo)
% COMPARAR_BALANCIN  Superpone un test grabado en la SD con el modelo teorico.
%
%   comparar_balancin('E:\BAL\T001P10.CSV', 'C:\Users\gatog\Downloads\EjemploSimulink')
%
%   Lee el CSV del firmware (bal_log.c), corre ejemplo_balancin.slx con la
%   misma referencia y el mismo throttle base y grafica:
%     1) theta real vs simulada (+ referencia)
%     2) motores vL (M1) / vR (M3) reales vs simulados
%     3) error theta_real - theta_sim
%   Imprime sobreimpulso, tiempo de establecimiento (2 %) y error en regimen.
%   Guarda la figura junto al CSV (<nombre>_comparacion.png).
%
%   El modelo arranca en theta = 0 con un escalon en t = 0.5 s. Si el test
%   empezo en otra referencia (ref_prev), se simula el escalon ref - ref_prev
%   y se suma ref_prev a la salida (la planta es lineal en theta; las
%   saturaciones hacen que sea una aproximacion).
%
%   No modifica los archivos del modelo: la referencia y Tbase se pasan con
%   Simulink.SimulationInput, que tienen prioridad sobre lo que deja InitFcn.

if nargin < 2 || isempty(carpetaModelo)
    carpetaModelo = pwd;
end
mdl = 'ejemplo_balancin';

%% Datos reales
hdr = leer_encabezado(csvFile);
opts = detectImportOptions(csvFile, 'CommentStyle', '#', ...
                           'VariableNamingRule', 'preserve');
T = readtable(csvFile, opts);

stepS   = campo(hdr, 'step_s', 0.5);
ref     = campo(hdr, 'ref_deg', NaN);
refPrev = campo(hdr, 'ref_prev_deg', 0);
thr     = campo(hdr, 'thr', 0.10);

t = T.t_s - stepS + 0.5;          % escalon en t = 0.5 s, como el modelo
tEnd = t(end);

%% Simulacion con los mismos parametros
addpath(carpetaModelo);
load_system(fullfile(carpetaModelo, [mdl '.slx']));
in = Simulink.SimulationInput(mdl);
in = in.setVariable('ref', deg2rad(ref - refPrev));
in = in.setVariable('Tbase', thr);
in = in.setModelParameter('StopTime', num2str(tEnd));
out = sim(in);

th  = out.logsout.get('theta').Values;
mot = out.logsout.get('motores').Values;
tSim  = th.Time;
thSim = rad2deg(squeeze(th.Data)) + refPrev;
m = squeeze(mot.Data);
if size(m, 1) == 2 && size(m, 2) ~= 2
    m = m.';
end
tMot = mot.Time;

thReal = T.theta_deg;
thSimI = interp1(tSim, thSim, t, 'linear', 'extrap');

%% Graficas
[carpeta, nombre] = fileparts(csvFile);
f = figure('Name', ['Balancin ' nombre], 'Position', [100 100 900 800]);

subplot(3, 1, 1);
plot(t, thReal, 'LineWidth', 1.2); hold on;
plot(tSim, thSim, '--', 'LineWidth', 1.2);
stairs(t, T.ref_deg, 'k:');
grid on; ylabel('\theta [deg]');
legend('real (SD)', 'modelo', 'referencia', 'Location', 'best');
title(sprintf('%s: ref %.2f deg (desde %.2f), Tbase %.3f', ...
              nombre, ref, refPrev, thr), 'Interpreter', 'none');

subplot(3, 1, 2);
plot(t, T.vL * 100, t, T.vR * 100, 'LineWidth', 1.1); hold on;
set(gca, 'ColorOrderIndex', 1);
plot(tMot, m(:, 1) * 100, '--', tMot, m(:, 2) * 100, '--', 'LineWidth', 1.1);
grid on; ylabel('Motor [%]');
legend('M1 real', 'M3 real', 'M1 modelo', 'M3 modelo', 'Location', 'best');

subplot(3, 1, 3);
plot(t, thReal - thSimI, 'LineWidth', 1.1);
grid on; ylabel('\theta_{real} - \theta_{modelo} [deg]'); xlabel('Tiempo [s]');

png = fullfile(carpeta, [nombre '_comparacion.png']);
exportgraphics(f, png, 'Resolution', 150);

%% Metricas
mReal = metricas(t, thReal, refPrev, ref);
mSim  = metricas(tSim, thSim, refPrev, ref);
res = struct('real', mReal, 'modelo', mSim, 'rmse_deg', ...
             sqrt(mean((thReal(t >= 0.5) - thSimI(t >= 0.5)).^2)), 'figura', png);

fprintf('\n%s  (ref %.2f deg, desde %.2f deg)\n', nombre, ref, refPrev);
fprintf('                       real     modelo\n');
fprintf('sobreimpulso [%%]   %8.2f   %8.2f\n', mReal.overshoot_pct, mSim.overshoot_pct);
fprintf('t establec. 2%% [s] %8.3f   %8.3f\n', mReal.settling_s, mSim.settling_s);
fprintf('error regimen [deg]%8.3f   %8.3f\n', mReal.ess_deg, mSim.ess_deg);
fprintf('RMSE real-modelo  %8.3f deg\n', res.rmse_deg);
fprintf('Figura: %s\n', png);
end

%% ------------------------------------------------------------------------
function hdr = leer_encabezado(csvFile)
% Lineas "# clave,valor" del principio del archivo.
hdr = struct();
fid = fopen(csvFile, 'r');
if fid < 0
    error('No se pudo abrir %s', csvFile);
end
c = onCleanup(@() fclose(fid));
while true
    ln = fgetl(fid);
    if ~ischar(ln) || ~startsWith(ln, '#')
        break;
    end
    kv = strsplit(strtrim(ln(2:end)), ',');
    if numel(kv) < 2
        continue;
    end
    clave = matlab.lang.makeValidName(strtrim(kv{1}));
    val = str2double(kv{2});
    if isnan(val)
        hdr.(clave) = strtrim(kv{2});
    else
        hdr.(clave) = val;
    end
end
end

function v = campo(s, nombre, def)
if isfield(s, nombre)
    v = s.(nombre);
else
    v = def;
end
end

function m = metricas(t, y, y0, yRef)
% Respuesta al escalon que empieza en t = 0.5 s.
m = struct('overshoot_pct', NaN, 'settling_s', NaN, 'ess_deg', NaN);
paso = yRef - y0;
k = t >= 0.5;
t = t(k); y = y(k);
if isempty(t)
    return;
end
yFin = mean(y(t >= t(end) - 1));
m.ess_deg = yRef - yFin;
if abs(paso) < 1e-6
    return;                       % sin escalon: solo error en regimen
end
s = sign(paso);
m.overshoot_pct = max(0, max(s * (y - yRef))) / abs(paso) * 100;
banda = 0.02 * abs(paso);
fuera = find(abs(y - yRef) > banda, 1, 'last');
if isempty(fuera)
    m.settling_s = 0;
elseif fuera < numel(t)
    m.settling_s = t(fuera + 1) - 0.5;
end
end
