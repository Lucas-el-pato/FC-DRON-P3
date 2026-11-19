% Construye ejemplo_balancin.slx, exporta la imagen del diagrama y simula.
here = fileparts(mfilename('fullpath'));
cd(here);
evalin('base', 'init_ejemplo');

mdl = 'ejemplo_balancin';
if bdIsLoaded(mdl), bdclose(mdl); end
if exist([mdl '.slx'], 'file'), delete([mdl '.slx']); end
new_system(mdl);
open_system(mdl);

lib.step  = 'simulink/Sources/Step';
lib.sum   = 'simulink/Math Operations/Sum';
lib.gain  = 'simulink/Math Operations/Gain';
lib.sat   = 'simulink/Discontinuities/Saturation';
lib.fcn   = 'simulink/User-Defined Functions/Interpreted MATLAB Function';
lib.const = 'simulink/Sources/Constant';
lib.mux   = 'simulink/Signal Routing/Mux';
lib.demux = 'simulink/Signal Routing/Demux';
lib.ss    = 'simulink/Continuous/State-Space';
lib.zoh   = 'simulink/Discrete/Zero-Order Hold';
lib.scope = 'simulink/Sinks/Scope';

b = @(name) [mdl '/' name];

add_block(lib.step, b('Referencia'), 'Position', [20 85 50 115], ...
    'Time', '0.5', 'Before', '0', 'After', 'ref');
add_block(lib.sum, b('Sum1'), 'Position', [100 90 120 110], ...
    'IconShape', 'round', 'Inputs', '|+-', 'ShowName', 'off');
add_block(lib.gain, b('PID 1'), 'Position', [160 85 200 115], 'Gain', 'Kth');
add_block(lib.sat, b('Limite w_ref'), 'Position', [240 85 270 115], ...
    'UpperLimit', 'wmax', 'LowerLimit', '-wmax');
add_block(lib.sum, b('Sum2'), 'Position', [320 90 340 110], ...
    'IconShape', 'round', 'Inputs', '|+-', 'ShowName', 'off');
add_block(lib.fcn, b('P + I'), 'Position', [380 75 480 125], ...
    'MATLABFcn', 'ControlPI(u)', 'OutputDimensions', '1', 'SampleTime', 'Tm');
add_block(lib.sum, b('Sum3'), 'Position', [520 90 540 110], ...
    'IconShape', 'round', 'Inputs', '|++', 'ShowName', 'off');
add_block(lib.fcn, b('D'), 'Position', [480 165 580 215], ...
    'MATLABFcn', 'ControlDer(u)', 'OutputDimensions', '1', 'SampleTime', 'Tm', ...
    'Orientation', 'up');
add_block(lib.sat, b('Saturacion'), 'Position', [580 85 610 115], ...
    'UpperLimit', 'umax', 'LowerLimit', '-umax');
add_block(lib.const, b('T'), 'Position', [590 20 640 40], 'Value', 'Tbase');
add_block(lib.mux, b('Mux'), 'Position', [670 45 675 125], 'Inputs', '2', ...
    'ShowName', 'off');
add_block(lib.fcn, b('Mixer'), 'Position', [710 60 810 110], ...
    'MATLABFcn', 'Mixer(u)', 'OutputDimensions', '2', 'SampleTime', '-1');
add_block(lib.gain, b('A porcentaje'), 'Position', [850 70 890 100], 'Gain', '100');
add_block(lib.demux, b('DemuxM'), 'Position', [930 45 935 125], 'Outputs', '2', ...
    'ShowName', 'off');
add_block('simulink/Continuous/Transfer Fcn', b('Motor M1'), ...
    'Position', [980 20 1080 60], 'Numerator', 'Gm_num', 'Denominator', 'Gm_den');
add_block('simulink/Continuous/Transfer Fcn', b('Motor M3'), ...
    'Position', [980 110 1080 150], 'Numerator', 'Gm_num', 'Denominator', 'Gm_den');
add_block(lib.sum, b('SumF'), 'Position', [1130 60 1160 110], ...
    'IconShape', 'rectangular', 'Inputs', '-+', 'ShowName', 'off');
add_block(lib.ss, b('Planta'), 'Position', [1210 50 1330 120], ...
    'A', 'Ap', 'B', 'Bp', 'C', 'Cp', 'D', 'Dp', 'X0', '[0;0]');
add_block(lib.demux, b('Demux'), 'Position', [1380 55 1385 115], 'Outputs', '2', ...
    'ShowName', 'off');
add_block(lib.zoh, b('ZOH 10 ms'), 'Position', [600 300 640 330], ...
    'SampleTime', '0.01', 'Orientation', 'left');
add_block(lib.scope, b('theta'), 'Position', [1440 0 1470 30]);
add_block(lib.scope, b('Motores'), 'Position', [860 0 890 30]);

ln = @(src, dst) add_line(mdl, src, dst, 'autorouting', 'smart');
name = @(h, s) set_param(h, 'Name', s);
TH = char(952);  W = char(969);  TAU = char(964);

ln('Referencia/1', 'Sum1/1');
name(ln('Sum1/1', 'PID 1/1'), ['e_' TH]);
ln('PID 1/1', 'Limite w_ref/1');
name(ln('Limite w_ref/1', 'Sum2/1'), [W '_ref']);
name(ln('Sum2/1', 'P + I/1'), ['e_' W]);
ln('P + I/1', 'Sum3/1');
ln('D/1', 'Sum3/2');
ln('Sum3/1', 'Saturacion/1');
ln('T/1', 'Mux/1');
name(ln('Saturacion/1', 'Mux/2'), TAU);
ln('Mux/1', 'Mixer/1');
hMix = ln('Mixer/1', 'A porcentaje/1');
name(hMix, '[vL; vR]');
ln('Mixer/1', 'Motores/1');
name(ln('A porcentaje/1', 'DemuxM/1'), '[%L; %R]');
ln('DemuxM/1', 'Motor M1/1');
ln('DemuxM/2', 'Motor M3/1');
hF1 = ln('Motor M1/1', 'SumF/1');
name(hF1, 'F1');
hF2 = ln('Motor M3/1', 'SumF/2');
name(hF2, 'F2');
name(ln('SumF/1', 'Planta/1'), 'u = F2 - F1');
ln('Planta/1', 'Demux/1');
hTh = ln('Demux/1', 'ZOH 10 ms/1');
name(hTh, TH);
ln('Demux/1', 'theta/1');
ln('ZOH 10 ms/1', 'Sum1/2');
hW = ln('Demux/2', 'Sum2/2');
name(hW, W);
ln('Demux/2', 'D/1');

set_param(get_param(hTh, 'SrcPortHandle'), 'DataLogging', 'on', ...
    'DataLoggingNameMode', 'Custom', 'DataLoggingName', 'theta');
set_param(get_param(hMix, 'SrcPortHandle'), 'DataLogging', 'on', ...
    'DataLoggingNameMode', 'Custom', 'DataLoggingName', 'motores');
set_param(get_param(hF1, 'SrcPortHandle'), 'DataLogging', 'on', ...
    'DataLoggingNameMode', 'Custom', 'DataLoggingName', 'F1');
set_param(get_param(hF2, 'SrcPortHandle'), 'DataLogging', 'on', ...
    'DataLoggingNameMode', 'Custom', 'DataLoggingName', 'F2');

tEnd = 10;
set_param(mdl, 'InitFcn', 'init_ejemplo', ...
    'SolverType', 'Fixed-step', 'Solver', 'ode4', 'FixedStep', 'Tm/4', ...
    'StopTime', num2str(tEnd), 'SignalLogging', 'on', 'SignalLoggingName', 'logsout');

save_system(mdl);
print(['-s' mdl], '-dpng', '-r150', fullfile(here, 'ejemplo_balancin_diagrama.png'));

out = sim(mdl);
th  = out.logsout.get('theta').Values;
mot = out.logsout.get('motores').Values;
F1  = out.logsout.get('F1').Values;
F2  = out.logsout.get('F2').Values;

f = figure('Visible', 'off', 'Position', [100 100 800 700]);
subplot(3,1,1);
plot(th.Time, rad2deg(th.Data), 'LineWidth', 1.2); hold on;
stairs([0 0.5 tEnd], rad2deg([0 ref ref]), 'k--');
grid on; ylabel('\theta [°]'); legend('\theta', 'referencia', 'Location', 'southeast');
title('Escalón de 10° en la referencia, con el modelo del motor');
subplot(3,1,2);
plot(mot.Time, squeeze(mot.Data)' * 100, 'LineWidth', 1.2);
grid on; ylabel('Motor [%]');
legend('M1 (v_L)', 'M3 (v_R)', 'Location', 'northeast');
subplot(3,1,3);
plot(F1.Time, squeeze(F1.Data), F2.Time, squeeze(F2.Data), 'LineWidth', 1.2);
grid on; ylabel('Empuje'); xlabel('Tiempo [s]');
legend('F1 (M1)', 'F2 (M3)', 'Location', 'northeast');
if ~exist('archivoResp', 'var'), archivoResp = 'ejemplo_balancin_respuesta.png'; end
exportgraphics(f, fullfile(here, archivoResp), 'Resolution', 150);
fprintf('theta final = %.2f deg\n', rad2deg(th.Data(end)));
