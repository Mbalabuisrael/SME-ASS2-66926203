%% SME3701 ASSIGNMENT 2 (2026) - QUESTION 1.1
% build_geared_shaft_model.m
%
% Builds the Simulink model of the geared shaft torsional system and saves
% it as geared_shaft_model.slx.
%
% Governing equation:   J0*thetaddot + kt*theta = Mt(t)
% Rearranged for the integrator chain:
%                        thetaddot = ( Mt(t) - kt*theta ) / J0
%
% Torque disturbance model:
%   The driving gear carries 16 teeth and turns at 1000 rev/min, so one
%   revolution takes 0.06 s and each tooth is in mesh for 0.06/16 = 3.75 ms.
%   The broken tooth cannot transmit load, so once per revolution the
%   transmitted torque collapses from 1000 N.m to zero for 3.75 ms. This is
%   built as a Constant of 1000 N.m minus a Pulse Generator of amplitude
%   1000 N.m, period 0.06 s and pulse width 6.25 percent.
%
% USAGE:  run sme3701_parameters.m first, then run this file.
% ------------------------------------------------------------------

if ~exist('kt','var')
    run('sme3701_parameters.m');
end

mdl = 'geared_shaft_model';

% Close and delete any earlier copy so the script can be re run cleanly
if bdIsLoaded(mdl); close_system(mdl, 0); end
if exist([mdl '.slx'], 'file'); delete([mdl '.slx']); end

new_system(mdl);
open_system(mdl);

%% ------------------------ Add the blocks ------------------------------
% Block positions are [left top right bottom] in pixels.

add_block('simulink/Sources/Constant', [mdl '/Steady torque M0'], ...
    'Value','M0', 'Position',[40 60 110 100]);

add_block('simulink/Sources/Pulse Generator', [mdl '/Broken tooth pulse'], ...
    'PulseType','Time based', ...
    'Amplitude','M0', ...
    'Period','T_rev', ...
    'PulseWidth','duty', ...
    'PhaseDelay','0', ...
    'Position',[40 175 110 235]);

add_block('simulink/Math Operations/Sum', [mdl '/Transmitted torque Mt'], ...
    'Inputs','+-', 'IconShape','round', 'Position',[175 125 205 155]);

add_block('simulink/Math Operations/Sum', [mdl '/Net torque'], ...
    'Inputs','+-', 'IconShape','round', 'Position',[275 125 305 155]);

add_block('simulink/Math Operations/Gain', [mdl '/Divide by J0'], ...
    'Gain','1/J0', 'Position',[350 120 410 160]);

add_block('simulink/Continuous/Integrator', [mdl '/Integrate to thetadot'], ...
    'InitialCondition','thetadot0', 'Position',[455 120 495 160]);

add_block('simulink/Continuous/Integrator', [mdl '/Integrate to theta'], ...
    'InitialCondition','theta0', 'Position',[545 120 585 160]);

add_block('simulink/Math Operations/Gain', [mdl '/Stiffness kt'], ...
    'Gain','kt', 'Orientation','left', 'Position',[380 270 440 310]);

%% ------------------------ Sinks and logging ---------------------------
add_block('simulink/Sinks/Scope', [mdl '/Scope Mt'], ...
    'Position',[240 30 275 65]);
add_block('simulink/Sinks/To Workspace', [mdl '/Log Mt'], ...
    'VariableName','Mt_log', 'SaveFormat','Structure With Time', ...
    'Position',[240 -35 310 5]);

add_block('simulink/Sinks/Scope', [mdl '/Scope thetadot'], ...
    'Position',[520 30 555 65]);
add_block('simulink/Sinks/To Workspace', [mdl '/Log thetadot'], ...
    'VariableName','thetadot_log', 'SaveFormat','Structure With Time', ...
    'Position',[520 -35 590 5]);

add_block('simulink/Sinks/Scope', [mdl '/Scope theta'], ...
    'Position',[665 120 700 155]);
add_block('simulink/Sinks/To Workspace', [mdl '/Log theta'], ...
    'VariableName','theta_log', 'SaveFormat','Structure With Time', ...
    'Position',[665 195 735 235]);

%% ------------------------ Connect the blocks --------------------------
add_line(mdl, 'Steady torque M0/1',      'Transmitted torque Mt/1', 'autorouting','on');
add_line(mdl, 'Broken tooth pulse/1',    'Transmitted torque Mt/2', 'autorouting','on');
add_line(mdl, 'Transmitted torque Mt/1', 'Net torque/1',            'autorouting','on');
add_line(mdl, 'Transmitted torque Mt/1', 'Scope Mt/1',              'autorouting','on');
add_line(mdl, 'Transmitted torque Mt/1', 'Log Mt/1',                'autorouting','on');

add_line(mdl, 'Net torque/1',            'Divide by J0/1',          'autorouting','on');
add_line(mdl, 'Divide by J0/1',          'Integrate to thetadot/1', 'autorouting','on');

add_line(mdl, 'Integrate to thetadot/1', 'Integrate to theta/1',    'autorouting','on');
add_line(mdl, 'Integrate to thetadot/1', 'Scope thetadot/1',        'autorouting','on');
add_line(mdl, 'Integrate to thetadot/1', 'Log thetadot/1',          'autorouting','on');

add_line(mdl, 'Integrate to theta/1',    'Scope theta/1',           'autorouting','on');
add_line(mdl, 'Integrate to theta/1',    'Log theta/1',             'autorouting','on');
add_line(mdl, 'Integrate to theta/1',    'Stiffness kt/1',          'autorouting','on');
add_line(mdl, 'Stiffness kt/1',          'Net torque/2',            'autorouting','on');

%% ------------------------ Signal labels -------------------------------
% Naming the signals makes the diagram self explanatory in the recording.
% Wrapped in try so a labelling quirk on any release never stops the build.
try
    set_param(get_param([mdl '/Transmitted torque Mt'],'PortHandles').Outport(1), ...
        'Name','Mt_Nm');
    set_param(get_param([mdl '/Divide by J0'],'PortHandles').Outport(1), ...
        'Name','thetaddot_rad_s2');
    set_param(get_param([mdl '/Integrate to thetadot'],'PortHandles').Outport(1), ...
        'Name','thetadot_rad_s');
    set_param(get_param([mdl '/Integrate to theta'],'PortHandles').Outport(1), ...
        'Name','theta_rad');
catch
    warning('Signal labelling skipped on this release. The model still runs.');
end

%% ------------------------ Solver configuration ------------------------
set_param(mdl, 'SolverType','Variable-step');
set_param(mdl, 'Solver','ode45');
set_param(mdl, 'StopTime', num2str(Tstop));
set_param(mdl, 'MaxStep',  num2str(MaxStep));
set_param(mdl, 'RelTol',   num2str(RelTol));
set_param(mdl, 'AbsTol',   '1e-10');
set_param(mdl, 'ZeroCrossControl','UseLocalSettings');
set_param(mdl, 'ReturnWorkspaceOutputs','on');
set_param(mdl, 'SaveOutput','on');

%% ------------------------ Annotation on the canvas --------------------
try
    note = Simulink.Annotation([mdl '/annotation']);
    note.Text = sprintf(['SME3701 Assignment 2, Question 1.1\n' ...
        'Geared shaft torsional vibration:  J0*thetaddot + kt*theta = Mt(t)\n\n' ...
        'J0 = %.2f N.m.s^2     kt = %.2f N.m/rad     fn = %.2f Hz\n' ...
        'Broken tooth: torque drops to 0 N.m for %.2f ms once every %.0f ms'], ...
        J0, kt, fn, tau*1000, T_rev*1000);
    note.Position = [40 350 640 430];
catch
    warning('Canvas annotation skipped on this release. The model still runs.');
end

%% ------------------------ Save ----------------------------------------
save_system(mdl, [mdl '.slx']);
fprintf('Simulink model saved as %s.slx in %s\n', mdl, pwd);
fprintf('Open it, press Run, then run run_and_plot.m to produce the figures.\n');
