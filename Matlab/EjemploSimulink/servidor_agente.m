function servidor_agente()
% Deja MATLAB escuchando tareas: ejecuta tarea_agente_in.m cuando aparece,
% guarda la salida en out_agente.txt y crea fin_agente.txt al terminar.
% Para detenerlo: stop(timerfind('Tag','agente')); delete(timerfind('Tag','agente'))
dir = fileparts(mfilename('fullpath'));
t = timerfind('Tag', 'agente');
if ~isempty(t), stop(t); delete(t); end
t = timer('Tag', 'agente', 'ExecutionMode', 'fixedSpacing', 'Period', 0.5, ...
    'BusyMode', 'drop', 'TimerFcn', @(~, ~) revisar(dir));
start(t);
fprintf('Servidor del agente activo en %s\n', dir);
end

function revisar(dir)
entrada = fullfile(dir, 'tarea_agente_in.m');
if ~isfile(entrada), return; end
tarea  = fullfile(dir, 'tarea_agente.m');
salida = fullfile(dir, 'out_agente.txt');
fin    = fullfile(dir, 'fin_agente.txt');
if isfile(fin), delete(fin); end
movefile(entrada, tarea, 'f');
clear tarea_agente
cd(dir);
try
    texto = evalc('evalin(''base'', ''tarea_agente'')');
catch ME
    texto = sprintf('ERROR:\n%s', getReport(ME, 'extended', 'hyperlinks', 'off'));
end
fid = fopen(salida, 'w'); fwrite(fid, texto); fclose(fid);
fid = fopen(fin, 'w'); fclose(fid);
fprintf('[agente] tarea terminada\n');
end
