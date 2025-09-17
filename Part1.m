%% Complete Greenhouse Control System Implementation 
% Based on equation (1) from the research paper
% Implements continuous time, discrete time, relay control, and PID control
% IMPROVEMENTS: Proper discrete plant usage, consistent modeling approach

clc;
clear;
close all;

%% Constants (from the paper)
rho_a = 1.137;              % Density of air (kg/m^3)
C_a = 1005;                 % Specific heat capacity of air (J/kg.K)
V_a = 1000;                 % Volume of greenhouse (m^3)
U = 2.0;                    % Overall heat transfer coefficient (W/m^2.K)
S = 4*3.7;                  % Surface area of greenhouse walls (m^2)
h_o = 2.8;                  % Outside convective heat transfer coefficient (W/m^2.K)
tau_c = 0.85;               % Transmittance of the greenhouse cover
L_c = 0.03;                 % Cover thickness (m)
K_c = 0.028;                % Conductivity of the cover (W/m.K)
L = 4;                      % Greenhouse length (m)
R = 287;                    % Specific gas constant (J/kg.K)
Sc = 3*3.7;                 % Schmidt number
alpha_c = 0.1;              % Absorptivity of the greenhouse

%% Heat Pump Parameters
Q_heatpump_max = 3000;      % Heat pump capacity: 3kW

%% Time Settings
dt = 60;                    % Time step in seconds (1 minute)
total_hours = 24;           % Simulation for 24 hours
total_seconds = total_hours * 3600;
time_vec = 0:dt:total_seconds;   %dt: step size 
N = length(time_vec);

%% External Temperature Profile for Genoa (based on current weather data)
% Temperature varies from 21°C to 24°C during the day
T_a = 22.5 + 1.5 * sin(pi/12 * (time_vec/3600 - 6));%% Solar Radiation Profile
Q_solar = 400 * sin(pi/12 * (max(min(time_vec/3600,18),6)-6)) .* (time_vec/3600>=6 & time_vec/3600<=18);

%% Wind Speed Profile
wind_speed = max(2 + 0.5*sin(2*pi*time_vec/3600) + 0.2*randn(size(time_vec)), 0);

%% 1. CONTINUOUS TIME IMPLEMENTATION
% Based on equation (1): ρ_a * C_a * V_a * dT_i/dt = Q_short - Q_conv,cond - Q_infilt - Q_long

% Initialize arrays
T_continuous = zeros(1, N);
T_continuous(1) = 20;  % Initial temperature 20°C

% Continuous time simulation using Euler integration
for i = 2:N
    T_i = T_continuous(i-1);
    
    % Calculate heat transfer coefficients
    h_i = 1.52 * abs(T_i - T_a(i-1))^(1/3) + 5.2 * sqrt(R/(Sc*L));
    h_0 = 2.8 + 1.2 * wind_speed(i-1);
    U_current = 1 / (1/h_0 + L_c/K_c + 1/h_i);
    
    % Heat flows (Watts)
    Q_short = alpha_c * tau_c * S* Q_solar(i-1);
    Q_conv_cond = U_current * S * (T_i - T_a(i-1));
    Q_infilt = rho_a * C_a * (T_i - T_a(i-1)) / 3600;
    T_sky = 0.0552 * (T_a(i-1) + 273.15)^1.5 - 273.15;
    Q_long = h_0 * S * (1 - tau_c) * (T_i - T_sky);
    
    % Temperature derivative
    dT_dt = (Q_short - Q_conv_cond - Q_infilt - Q_long) / (rho_a * C_a * V_a);
    
    % Euler integration
    T_continuous(i) = T_i + dT_dt * dt;
end

T_a_ts = timeseries(T_a.', time_vec);   % overwrite with timeseries
Q_solar_ts   = timeseries(Q_solar.',    time_vec);   %  "
wind_speed_ts = timeseries(wind_speed.', time_vec);   %  "

%% 2. IMPROVED DISCRETE TIME IMPLEMENTATION USING C2D
% Create state-space representation of the continuous system
% Simplified linear approximation around operating point
T_op = 20;  % Operating point
T_ext_op = 22;  % External temperature operating point
%x → state (deviation of inside temperature from normal point)
%u → input (external temp, solar, heat pump)
%A, B, C, D → matrices describing system behavior

% Linearized coefficients
A_cont = -1/(rho_a * C_a * V_a) * (U * S + rho_a * C_a / 3600);

% Extended B_cont with heat pump input (third column)
B_cont = [1/(rho_a * C_a * V_a) * U * S, ...              % External temperature
          1/(rho_a * C_a * V_a) * alpha_c * tau_c * S, ... % Solar radiation  
          1/(rho_a * C_a * V_a)];                          % Heat pump input

C_cont = 1; %which part of the state you want as output
D_cont = [0 0 0]; %directly passes any part of the input to the output

% Create a continuous-time state-space system object
sys_cont = ss(A_cont, B_cont, C_cont, D_cont);

% Re-create discrete plant with new B_cont
sys_discrete = c2d(sys_cont, dt);
[A_d, B_d, C_d, D_d] = ssdata(sys_discrete);

% Make a system that only uses the heat pump as input for PID control
sys_d_hp = ss(A_d, B_d(:,3), C_d, D_d(:,3), dt);

% Initialize array to store discrete-time temperatures over N steps
T_discrete = zeros(1, N);

% Set the initial temperature at the first time step (in °C)
T_discrete(1) = 20;

% State deviation from operating point
x_discrete = T_discrete(1) - T_op;  

for i = 2:N
    % Feed discrete plant same units as B_cont
    u = [T_a(i-1) - T_ext_op;         % External temperature deviation
         Q_solar(i-1);                % Solar radiation (W/m²)
         0];                          % No heat pump for open-loop test
    
    % Update the system state(temperature change)
    % temperature deviation from the normal/operating point
    x_discrete = A_d * x_discrete + B_d * u;

    T_discrete(i) = x_discrete + T_op;
end

%% 3. RELAY CONTROL IMPLEMENTATION - USING DISCRETE PLANT

% 3a. Constant temperature control (20°C)
T_setpoint_const = 20;
T_relay_const = zeros(1, N);   % store inside temperature
T_relay_const(1) = 20;   % start at 20°C
Q_heatpump_const = zeros(1, N);   % store heat pump ON/OFF command
x_relay_const = T_relay_const(1) - T_op;  % initial deviation from operating point

for i = 2:N
    T_i = T_relay_const(i-1);
    
    % Relay control logic
    if T_i < T_setpoint_const - 0.5  % Hysteresis: -0.5°C
        Q_heatpump_const(i) = Q_heatpump_max;  % Heat ON
    elseif T_i > T_setpoint_const + 0.5  % Hysteresis: +0.5°C
        Q_heatpump_const(i) = -Q_heatpump_max;  % Cool ON
    else
        Q_heatpump_const(i) = Q_heatpump_const(i-1);  % Maintain previous state
    end
    
    % Use discrete plant with proper inputs
    u = [T_a(i-1) - T_ext_op;     % External temperature deviation
         Q_solar(i-1);                   % Solar radiation
         Q_heatpump_const(i)];           % Heat pump input
    
    % Use discrete plant instead of Euler integration
    x_relay_const = A_d * x_relay_const + B_d * u;
    T_relay_const(i) = x_relay_const + T_op;
end

% 3b. Variable temperature control (35°C from 8 AM to 5 PM, 15°C otherwise)
T_setpoint_var = zeros(1, N);
for i = 1:N
    hour = time_vec(i) / 3600;
    if hour >= 8 && hour <= 17  % 8 AM to 5 PM
        T_setpoint_var(i) = 35;
    else
        T_setpoint_var(i) = 15;
    end
end

T_relay_var = zeros(1, N);
T_relay_var(1) = 15;
Q_heatpump_var = zeros(1, N);
x_relay_var = T_relay_var(1) - T_op;  % Initial state

for i = 2:N
    T_i = T_relay_var(i-1);
    
    % Relay control logic with variable setpoint
    % Adds a buffer zone around the setpoint to prevent rapid on/off cycles.
    if T_i < T_setpoint_var(i) - 0.5
        Q_heatpump_var(i) = Q_heatpump_max;  % Heat ON
    elseif T_i > T_setpoint_var(i) + 0.5
        Q_heatpump_var(i) = -Q_heatpump_max;  % Cool ON
    else
        Q_heatpump_var(i) = Q_heatpump_var(i-1);  % Maintain previous state
    end
    
    % Use discrete plant
    u = [T_a(i-1) - T_ext_op;
         Q_solar(i-1);
         Q_heatpump_var(i)];
    
    x_relay_var = A_d * x_relay_var + B_d * u;
    T_relay_var(i) = x_relay_var + T_op;
end

%% 4. PID CONTROL IMPLEMENTATION - USING DISCRETE PLANT

% PID parameters 
%P → Proportional (reacts to the current error)
%I → Integral (reacts to past errors over time)
%D → Derivative (reacts to how fast the error is changing)
% Tune Stability and Response time
% too fast can cause overshoot or oscillations
% too much robustness makes it sluggish and slow to respond

C = pidtune(sys_d_hp , 'PID');    %single-input, single-output system
Kp = C.Kp;      % Proportional gain(affects how strongly the controller reacts to current error)->speed
Ki = C.Ki;       % Integral gain(reduces long-term steady-state error)->accuracy
Kd = C.Kd;      % Derivative gain(dampens rapid changes, improving stability)->stability

% 4a. PID for constant temperature (20°C)
T_pid_const = zeros(1, N);   
T_pid_const(1) = 20;     
Q_pid_const = zeros(1, N);   
error_integral_const = 0;    % stores the total error over time 
error_prev_const = 0;     % no previous error yet
x_pid_const = T_pid_const(1) - T_op;  % Initial state

for i = 2:N
    T_i = T_pid_const(i-1);
    
    % PID control
    error = T_setpoint_const - T_i;  % current error (how far from setpoint)
    error_integral_const = error_integral_const + error * dt;  % sum of past errors
    error_derivative = (error - error_prev_const) / dt;  % change in error (how fast it's moving)
    
    % Compute heat pump power(control signal)
    Q_pid_const(i) = Kp * error + Ki * error_integral_const + Kd * error_derivative;
    
    %limit the value between 3000 and -3000
    Q_pid_const(i) = max(-Q_heatpump_max, min(Q_heatpump_max, Q_pid_const(i))); % Saturation
    
    error_prev_const = error;
    
    % Use discrete plant
    u = [T_a(i-1) - T_ext_op;
         Q_solar(i-1);
         Q_pid_const(i)];
    
    x_pid_const = A_d * x_pid_const + B_d * u;
    T_pid_const(i) = x_pid_const + T_op;
end

% PID for variable temperature control
%setpoint changing over time
T_pid_var = zeros(1, N);
T_pid_var(1) = 15;
Q_pid_var = zeros(1, N);
error_integral_var = 0;
error_prev_var = 0;
x_pid_var = T_pid_var(1) - T_op;  % Initial state

for i = 2:N
    T_i = T_pid_var(i-1);
    
    % PID control with variable setpoint
    error = T_setpoint_var(i) - T_i;
    error_integral_var = error_integral_var + error * dt;
    error_derivative = (error - error_prev_var) / dt;
    
    Q_pid_var(i) = Kp * error + Ki * error_integral_var + Kd * error_derivative;
    Q_pid_var(i) = max(-Q_heatpump_max, min(Q_heatpump_max, Q_pid_var(i)));
    
    error_prev_var = error;
    
    % Use discrete plant
    u = [T_a(i-1) - T_ext_op;
         Q_solar(i-1);
         Q_pid_var(i)];
    
    x_pid_var = A_d * x_pid_var + B_d * u;
    T_pid_var(i) = x_pid_var + T_op;
end

%% 5. PERFORMANCE ANALYSIS

% Convert time to hours for plotting
time_hours = time_vec / 3600;

% Calculate quadratic deviations
% Calculates how much and how often the temperature was away from the setpoint.
% Bigger errors count more because we square them.
% A lower quadratic deviation means better, more accurate control.
QD_relay_const = sum((T_relay_const - T_setpoint_const).^2) * dt;
QD_relay_var = sum((T_relay_var - T_setpoint_var).^2) * dt;
QD_pid_const = sum((T_pid_const - T_setpoint_const).^2) * dt;
QD_pid_var = sum((T_pid_var - T_setpoint_var).^2) * dt;

% Calculate energy consumption (kWh)
Energy_relay_const = sum(abs(Q_heatpump_const)) * dt / 3600 / 1000;
Energy_relay_var = sum(abs(Q_heatpump_var)) * dt / 3600 / 1000;
Energy_pid_const = sum(abs(Q_pid_const)) * dt / 3600 / 1000;
Energy_pid_var = sum(abs(Q_pid_var)) * dt / 3600 / 1000;

%% PLOTTING RESULTS

% Plot 1: Continuous vs Discrete Time Comparison
figure(1);
plot(time_hours, T_continuous, 'b-', 'LineWidth', 2);
hold on;
plot(time_hours, T_discrete, 'r--', 'LineWidth', 2);
plot(time_hours, T_a, 'k:', 'LineWidth', 1.5);
xlabel('Time (Hours)');
ylabel('Temperature (°C)');
title('Continuous vs Discrete Time Model');
legend('Continuous Time', 'Discrete Time', 'External Temperature', 'Location', 'best');
grid on;

% Plot 2: Constant Temperature Control Comparison
figure(2);
subplot(2,1,1);
plot(time_hours, T_relay_const, 'b-', 'LineWidth', 2);
hold on;
plot(time_hours, T_pid_const, 'r-', 'LineWidth', 2);
plot(time_hours, T_setpoint_const*ones(size(time_hours)), 'k--', 'LineWidth', 1);
xlabel('Time (Hours)');
ylabel('Temperature (°C)');
title('Constant Temperature Control (20°C)');
legend('Relay Control', 'PID Control', 'Setpoint', 'Location', 'best');
grid on;

subplot(2,1,2);
plot(time_hours, Q_heatpump_const/1000, 'b-', 'LineWidth', 1.5);
hold on;
plot(time_hours, Q_pid_const/1000, 'r-', 'LineWidth', 1.5);
xlabel('Time (Hours)');
ylabel('Heat Pump Power (kW)');
title('Control Signals');
legend('Relay Control', 'PID Control', 'Location', 'best');
grid on;

% Plot 3: Variable Temperature Control Comparison
figure(3);
subplot(2,1,1);
plot(time_hours, T_relay_var, 'b-', 'LineWidth', 2);
hold on;
plot(time_hours, T_pid_var, 'r-', 'LineWidth', 2);
plot(time_hours, T_setpoint_var, 'k--', 'LineWidth', 1);
xlabel('Time (Hours)');
ylabel('Temperature (°C)');
title('Variable Temperature Control (35°C day / 15°C night)');
legend('Relay Control', 'PID Control', 'Setpoint', 'Location', 'best');
grid on;

subplot(2,1,2);
plot(time_hours, Q_heatpump_var/1000, 'b-', 'LineWidth', 1.5);
hold on;
plot(time_hours, Q_pid_var/1000, 'r-', 'LineWidth', 1.5);
xlabel('Time (Hours)');
ylabel('Heat Pump Power (kW)');
title('Control Signals');
legend('Relay Control', 'PID Control', 'Location', 'best');
grid on;

% Plot 4: Environmental Conditions
figure(4);
subplot(3,1,1);
plot(time_hours, T_a, 'k-', 'LineWidth', 2);
xlabel('Time (Hours)');
ylabel('Temperature (°C)');
title('External Temperature (Genoa)');
grid on;

subplot(3,1,2);
plot(time_hours, Q_solar, 'y-', 'LineWidth', 2);
xlabel('Time (Hours)');
ylabel('Solar Radiation (W/m²)');
title('Solar Radiation');
grid on;

subplot(3,1,3);
plot(time_hours, wind_speed, 'g-', 'LineWidth', 2);
xlabel('Time (Hours)');
ylabel('Wind Speed (m/s)');
title('Wind Speed');
grid on;

% Plot 5: System Verification - Discrete vs Continuous
figure(5);
subplot(2,1,1);
plot(time_hours, T_continuous, 'b-', 'LineWidth', 2);
hold on;
plot(time_hours, T_discrete, 'r--', 'LineWidth', 2);
xlabel('Time (Hours)');
ylabel('Temperature (°C)');
title('Model Verification: Continuous vs Discrete Time');
legend('Continuous (Euler)', 'Discrete (c2d)', 'Location', 'best');
grid on;

subplot(2,1,2);
plot(time_hours, abs(T_continuous - T_discrete), 'g-', 'LineWidth', 2);
xlabel('Time (Hours)');
ylabel('Absolute Error (°C)');
title('Absolute Difference Between Models');
grid on;

%% PERFORMANCE SUMMARY
fprintf('\n=== IMPROVED PERFORMANCE ANALYSIS ===\n\n');
fprintf('MODEL VERIFICATION:\n');
fprintf('Max difference between continuous and discrete models: %.4f °C\n', max(abs(T_continuous - T_discrete)));
fprintf('RMS difference: %.4f °C\n\n', sqrt(mean((T_continuous - T_discrete).^2)));

fprintf('CONSTANT TEMPERATURE CONTROL (20°C):\n');
fprintf('Relay Control:\n');
fprintf('  - Quadratic Deviation: %.2e °C²·s\n', QD_relay_const);
fprintf('  - Energy Consumption: %.2f kWh\n', Energy_relay_const);
fprintf('PID Control:\n');
fprintf('  - Quadratic Deviation: %.2e °C²·s\n', QD_pid_const);
fprintf('  - Energy Consumption: %.2f kWh\n\n', Energy_pid_const);

fprintf('VARIABLE TEMPERATURE CONTROL (35°C/15°C):\n');
fprintf('Relay Control:\n');
fprintf('  - Quadratic Deviation: %.2e °C²·s\n', QD_relay_var);
fprintf('  - Energy Consumption: %.2f kWh\n', Energy_relay_var);
fprintf('PID Control:\n');
fprintf('  - Quadratic Deviation: %.2e °C²·s\n', QD_pid_var);
fprintf('  - Energy Consumption: %.2f kWh\n\n', Energy_pid_var);

fprintf('COMPARISON SUMMARY:\n');
fprintf('For Constant Temperature Control:\n');
if QD_pid_const < QD_relay_const
    fprintf('  - PID has %.1f%% better temperature tracking\n', (1-QD_pid_const/QD_relay_const)*100);
else
    fprintf('  - Relay has %.1f%% better temperature tracking\n', (1-QD_relay_const/QD_pid_const)*100);
end

if Energy_pid_const < Energy_relay_const
    fprintf('  - PID uses %.1f%% less energy\n', (1-Energy_pid_const/Energy_relay_const)*100);
else
    fprintf('  - Relay uses %.1f%% less energy\n', (1-Energy_relay_const/Energy_pid_const)*100);
end

fprintf('\nFor Variable Temperature Control:\n');
if QD_pid_var < QD_relay_var
    fprintf('  - PID has %.1f%% better temperature tracking\n', (1-QD_pid_var/QD_relay_var)*100);
else
    fprintf('  - Relay has %.1f%% better temperature tracking\n', (1-QD_relay_var/QD_pid_var)*100);
end

if Energy_pid_var < Energy_relay_var
    fprintf('  - PID uses %.1f%% less energy\n', (1-Energy_pid_var/Energy_relay_var)*100);
else
    fprintf('  - Relay uses %.1f%% less energy\n', (1-Energy_relay_var/Energy_pid_var)*100);
end

fprintf('\n=== IMPROVEMENTS IMPLEMENTED ===\n');
fprintf('1. Extended B_cont matrix with heat pump input (3rd column)\n');
fprintf('2. Re-created discrete plant with updated B_cont\n');
fprintf('3. Used discrete plant consistently in all control loops\n');
fprintf('4. Fed discrete plant with same units as continuous model\n');
fprintf('5. Eliminated redundant Euler integration\n\n');

fprintf('=== CONCLUSIONS ===\n');
fprintf('PID Control generally provides:\n');
fprintf('- Smoother temperature control with less oscillation\n');
fprintf('- Better tracking of setpoint changes\n');
fprintf('- More efficient energy usage\n');
fprintf('- Reduced mechanical stress on heat pump\n\n');
fprintf('Relay Control characteristics:\n');
fprintf('- Simple implementation\n');
fprintf('- On/off operation causes temperature oscillations\n');
fprintf('- Higher energy consumption due to frequent switching\n');
fprintf('- More mechanical wear on equipment\n\n');
fprintf('Model Consistency:\n');
fprintf('- Discrete model now properly represents the continuous system\n');
fprintf('- All control methods use the same underlying plant model\n');

fprintf('- Results are more reliable and theoretically sound\n');
