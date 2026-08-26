%% SME3701 ASSIGNMENT 2 (2026) - QUESTION 1
% sme3701_parameters.m
% System parameters for the geared shaft torsional vibration model.
% Governing equation:  J0*thetaddot + kt*theta = Mt(t)
%
% Run this FIRST. It populates the base workspace used by the Simulink model.
% -----------------------------------------------------------------

clear; clc;

%% --------------- Given data (from the assignment brief) ---------------
J0    = 0.10;      % Mass moment of inertia of driven gear   [N.m.s^2]
d     = 0.050;     % Diameter of solid steel driven shaft    [m]
L     = 1.00;      % Length of driven shaft                  [m]
G     = 79.3e9;    % Shear modulus of steel                  [Pa]
M0    = 1000;      % Steady transmitted torque               [N.m]
N_rpm = 1000;      % Rotational speed of the driving gear    [rev/min]
z     = 16;        % Number of teeth on the driving gear     [ - ]
n_brk = 1;         % Number of broken teeth                  [ - ]

%% --------------- Derived shaft and system properties ------------------
Ip    = pi*d^4/32;      % Polar second moment of area of the shaft   [m^4]
kt    = G*Ip/L;         % Torsional stiffness of the driven shaft    [N.m/rad]
wn    = sqrt(kt/J0);    % Undamped natural circular frequency        [rad/s]
fn    = wn/(2*pi);      % Undamped natural frequency                 [Hz]
Tn    = 1/fn;           % Natural period                             [s]

%% --------------- Excitation properties --------------------------------
f_rev  = N_rpm/60;      % Rotational frequency of driving gear          [Hz]
T_rev  = 1/f_rev;       % Time for one revolution of driving gear       [s]
w0     = 2*pi*f_rev;    % Fundamental circular frequency of disturbance [rad/s]
tau    = n_brk*T_rev/z; % Duration of the missing tooth engagement      [s]
duty   = 100*tau/T_rev; % Pulse width as a percentage of the period     [%]
f_mesh = z*f_rev;       % Tooth meshing frequency                       [Hz]

%% --------------- Initial conditions -----------------------------------
% The system runs under steady conditions before the tooth breaks, so the
% shaft is already twisted by the static amount and is not moving.
theta0    = M0/kt;   % Static twist before the break   [rad]
thetadot0 = 0;       % Initial angular velocity        [rad/s]

%% --------------- Simulation settings ----------------------------------
Tstop   = 0.5;   % Simulation stop time     [s]
MaxStep = 1e-5;  % Maximum solver step      [s]
RelTol  = 1e-8;  % Relative tolerance       [ - ]

%% --------------- Report to the Command Window -------------------------
fprintf('\n============ SME3701 ASSIGNMENT 2 : SYSTEM PARAMETERS ============\n');
fprintf('Polar second moment of area, Ip = %.6e m^4\n', Ip);
fprintf('Torsional stiffness,         kt = %.2f N.m/rad\n', kt);
fprintf('Natural circular frequency,  wn = %.2f rad/s\n', wn);
fprintf('Natural frequency,           fn = %.2f Hz\n', fn);
fprintf('Natural period,              Tn = %.5f s\n', Tn);
fprintf('-----------------------------------------------------------------\n');
fprintf('Driving gear speed,       f_rev = %.4f Hz  (%.0f rev/min)\n', f_rev, N_rpm);
fprintf('Revolution period,        T_rev = %.4f s\n', T_rev);
fprintf('Missing tooth duration,     tau = %.5f s  (%.2f %% duty)\n', tau, duty);
fprintf('Tooth meshing frequency, f_mesh = %.2f Hz\n', f_mesh);
fprintf('-----------------------------------------------------------------\n');
fprintf('Static twist before break, theta0 = %.6f rad (%.4f deg)\n', theta0, rad2deg(theta0));
n_near = round(fn/f_rev);
fprintf('Nearest harmonic of the disturbance is n = %d at %.2f Hz\n', n_near, n_near*f_rev);
fprintf('Separation from fn = %.2f %%\n', 100*abs(n_near*f_rev - fn)/fn);
fprintf('=================================================================\n\n');
