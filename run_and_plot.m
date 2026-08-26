%% SME3701 ASSIGNMENT 2 (2026) - QUESTIONS 1.2, 1.3 AND 1.4
% run_and_plot.m
%
% Runs the Simulink model, plots theta(t) and thetadot(t), extracts the
% maximum angular displacement and angular velocity, and identifies the
% dominant torsional vibration frequency from an FFT of the response.
%
% USAGE:  run sme3701_parameters.m, then run build_geared_shaft_model.m,
%         then this file.
% ------------------------------------------------------------------

if ~exist('kt','var'); run('sme3701_parameters.m'); end

mdl = 'geared_shaft_model';
if ~bdIsLoaded(mdl)
    if exist([mdl '.slx'],'file')
        load_system(mdl);
    else
        run('build_geared_shaft_model.m');
    end
end

%% ------------------------ Run the simulation --------------------------
out = sim(mdl);

% unpackLog is defined at the end of this file. It copes with either of the
% two To Workspace logging formats used across MATLAB releases.
[t,    theta]    = unpackLog(out.theta_log);
[~,    thetadot] = unpackLog(out.thetadot_log);
[tM,   Mt]       = unpackLog(out.Mt_log);

%% ============ QUESTION 1.2 : plot theta(t) and thetadot(t) =============
figure('Name','Q1.2 Simulated response','Color','w','Position',[80 80 900 720]);

subplot(3,1,1);
plot(tM*1000, Mt, 'LineWidth', 1.1, 'Color', [0.12 0.31 0.47]); grid on;
xlim([0 180]); ylim([-100 1150]);
xlabel('Time   t   (ms)');
ylabel('M_t   (N\cdotm)');
title('Transmitted torque M_t(t): three revolutions of the driving gear');

subplot(3,1,2);
plot(t, theta, 'LineWidth', 0.6, 'Color', [0.12 0.31 0.47]); grid on; hold on;
yline(theta0, '--r', 'LineWidth', 1.0);
yline(M0*(z-1)/z/kt, ':', 'Color', [0.33 0.51 0.21], 'LineWidth', 1.4);
xlabel('Time   t   (s)');
ylabel('\theta   (rad)');
title('Angular displacement of the driven gear, \theta(t)');
legend('\theta(t)','Static twist before the break','Mean twist after the break', ...
    'Location','southwest');

subplot(3,1,3);
plot(t, thetadot, 'LineWidth', 0.6, 'Color', [0.44 0.19 0.63]); grid on;
xlabel('Time   t   (s)');
ylabel('$\dot{\theta}$   (rad/s)','Interpreter','latex');
title('Angular velocity of the driven gear');

%% ============ QUESTION 1.3 : maximum displacement and velocity =========
[theta_max,    i1] = max(abs(theta));
[thetadot_max, i2] = max(abs(thetadot));

fprintf('\n=============== QUESTION 1.3 : PEAK RESPONSE ===================\n');
fprintf('Simulation window                 : 0 to %.2f s\n', t(end));
fprintf('Maximum angular displacement      : %.5f rad  (%.4f deg) at t = %.4f s\n', ...
    theta_max, rad2deg(theta_max), t(i1));
fprintf('Maximum angular velocity          : %.3f rad/s   at t = %.4f s\n', ...
    thetadot_max, t(i2));
fprintf('Static twist before the break     : %.5f rad\n', theta0);
fprintf('Dynamic magnification, theta_max/theta_static = %.3f\n', theta_max/theta0);
fprintf('===============================================================\n');

%% ============ QUESTION 1.4 : dominant frequency and resonance =========
% The ode45 solver returns unevenly spaced time points, so the signal is
% first resampled onto a uniform grid before the FFT is taken.
Fs   = 20000;                        % uniform sampling rate [Hz]
tu   = (0 : 1/Fs : t(end))';
thu  = interp1(t, theta, tu, 'pchip');
thu  = thu - mean(thu);              % remove the static offset
N    = numel(thu);
Nfft = 2^nextpow2(4*N);              % zero pad for finer frequency spacing
win  = 0.5 - 0.5*cos(2*pi*(0:N-1)'/(N-1));   % Hann window, no toolbox needed
Y    = fft(thu .* win, Nfft);
f    = (0:Nfft/2)' * Fs/Nfft;
A    = abs(Y(1:Nfft/2+1));
A    = A/max(A);

figure('Name','Q1.4 Frequency content','Color','w','Position',[120 120 900 380]);
plot(f, A, 'LineWidth', 1.0, 'Color', [0.12 0.31 0.47]); hold on;
xline(fn,      '--r', 'LineWidth', 1.2);
xline(6*f_rev, ':', 'Color', [0.33 0.51 0.21], 'LineWidth', 1.2);
xline(7*f_rev, ':', 'Color', [0.33 0.51 0.21], 'LineWidth', 1.2);
xlim([0 300]); ylim([0 1.1]);
xlabel('Frequency   f   (Hz)');
ylabel('Normalised amplitude of \theta   ( - )');
title('Single sided amplitude spectrum of \theta(t)');
legend('Spectrum of \theta(t)', ...
    sprintf('Natural frequency f_n = %.2f Hz', fn), ...
    sprintf('6th harmonic = %.2f Hz', 6*f_rev), ...
    sprintf('7th harmonic = %.2f Hz', 7*f_rev), 'Location','northeast');

[~, ipk]    = max(A);
f_dominant  = f(ipk);

n_near   = round(fn/f_rev);
f_near   = n_near*f_rev;
r_near   = f_near/fn;
detuning = 100*(f_near - fn)/fn;
magnif   = 1/abs(1 - r_near^2);

fprintf('\n========== QUESTION 1.4 : DOMINANT FREQUENCY AND RESONANCE =====\n');
fprintf('Dominant peak in the response spectrum   : %.2f Hz\n', f_dominant);
fprintf('Undamped natural frequency, fn           : %.2f Hz\n', fn);
fprintf('Disturbance fundamental (1 per rev)      : %.2f Hz\n', f_rev);
fprintf('Closest harmonic: n = %d at %.2f Hz\n', n_near, f_near);
fprintf('Frequency ratio r = f_%d/fn               : %.4f\n', n_near, r_near);
fprintf('Detuning from resonance                  : %.2f %%\n', detuning);
fprintf('Dynamic magnification 1/|1 - r^2|         : %.2f\n', magnif);
if abs(detuning) < 10
    fprintf('ASSESSMENT: the %dth harmonic lies within %.1f %% of fn.\n', n_near, abs(detuning));
    fprintf('            Exact resonance does not occur, but the system runs in\n');
    fprintf('            a severe near resonant condition and must be detuned.\n');
else
    fprintf('ASSESSMENT: no harmonic lies close to fn. Resonance does not occur.\n');
end
fprintf('===============================================================\n\n');

%% ------------------------ Save the figures ----------------------------
figs = findobj('Type','figure');
for k = 1:numel(figs)
    exportgraphics(figs(k), sprintf('figure_%d.png', k), 'Resolution', 300);
end
fprintf('Figures exported to %s\n', pwd);

%% ------------------------ Local function ------------------------------
function [tv, yv] = unpackLog(s)
% Extracts time and data from a To Workspace log, whichever format was used.
    if isa(s,'timeseries')
        tv = s.Time;  yv = squeeze(s.Data);
    elseif isstruct(s) && isfield(s,'signals')
        tv = s.time;  yv = squeeze(s.signals.values);
    else
        error('run_and_plot:badLog','Unrecognised To Workspace logging format.');
    end
end
