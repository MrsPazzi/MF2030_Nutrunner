%% 036BX4 motor specs

rpm_to_rs =  2 * pi / 60;
rs_to_rpm = 60 / (2 * pi);

U_N = 36;
I0 = 0.124;
J = 6.3E-6;
R = 3.23;
K_M = 0.0644;
K_E = 0.006741 / rpm_to_rs;
C_V = 1.1E-6;
L = 0.000238;
b_m = C_V / rpm_to_rs;

%% State space matrix's (Task 1-3)
A = [(-R/L), (-K_E/L); K_M/J, (-b_m/J)];  % State matrix
B = [(1/L), 0]';                          % Input to state vector
C = [0, 1];                               % State to output vector
D = 0;                                    % Feed through vector (No disturbance for now)

%% Simplified State space matrix's (Task 4)
A4 = -((K_M*K_E + (b_m*R))/(J*R)); % State matrix simplified
B4 = K_M/ (J*R);                   % Input to state vector
C4 = 1;                            % State to output vector
D4 = 0;                            % Feed through vector (No disturbance for now)

%% Technical specifications cordless nutrunner

maximumTorque = 55;     % [Nm] (T_max)
minimumTorque = 10;     % [Nm] (T_min)

%% Damping

d_t = 1.5; % [Nm*s/rad]
d_j = 1.5; % [Nm*s/rad]

%% Stiffness

k_t = 739; % [Nm/rad]

%% Inertias

J_gs1 = 6.5E-8; % [kgm^2]
J_gs2 = 5.3E-8; % [kgm^2]

%% Material

densitySteel = 7580; % [kg/m^3]

%% M8 screw

A_M8 = 3.66E-5;
E_steel = 210E9;
L_free = 0.1;
L_p = 1.25E-3;