clc, clear, close all;

%% Symbolical representation (State-space and Transfer function)
clc, clear, close all;
syms R L K_E K_M J C_V s


A = [(-R/L), (-K_E/L); K_M/J, (-C_V/J)]; % State matrix
B = [(1/L), 0]'; % Input to state vector
C = [0, 1]; % State to output vector
D = 0; % Feed through vector (No disturbance for now)



I = eye(size(A));
G = simplify(C * ((s*I - A)\B) + D); % Transfer function theta_dot(s)/u(s)
pretty(G)


%% Symbolical representation (State-space and Transfer function) Simplified
clc, clear, close all;
syms R L K_E K_M J C_V s


A = -((K_M*K_E + (C_V*R))/(J*R)); % State matrix
B = K_M/ (J * R); % Input to state vector
C = 1; % State to output vector
D = 0; % Feed through vector (No disturbance for now)



I = eye(size(A));
G = simplify(C * ((s*I - A)\B) + D); % Transfer function theta_dot(s)/u(s)
pretty(G)

%% Numeric representation (State-space and Transfer function) + step response (angular velocity) + tau
clc, clear, close all;
run 'motor_specs.m' % Get motor parameters

ss3 = ss(A,B,C,D); % Creates a state-space object

% [symG_num, symG_den] = ss2tf(A,B,C,D); % TF from SS, works with numeric

G = tf(ss3)               % speed/voltage transfer function (uses ss object)
dcg = dcgain(G);           % steady-state speed per volt (rad/s per V)

opt = RespConfig;
opt.Amplitude = 36;

% Step responses
[y,t] = step(ss3,opt);
no_load_speed = dcg * U_N % Calculated no-load speed Task2

tau = 0; 
index = 1;

while y(index) <= (0.632 * no_load_speed)
    tau = t(index);
    index = index + 1;

end
pzmap(G)
grid on

tau % Calculated tau Task2

% Simplified TF
A = -((K_M*K_E + (C_V*R))/(J*R)); % State matrix
B = K_M/ (J * R); % Input to state vector
C = 1; % State to output vector
D = 0; % Feed through vector (No disturbance for now)

ss3 = ss(A,B,C,D); % Creates a state-space object
G = tf(ss3)      
pzmap(G)
grid on


%step(ss3, opt)                 % speed response to 36 V step
%grid on
%figure, bode(ss3), grid on  % frequency response

%% Numeric representation (State-space and Transfer function) + step response (current)
clc, clear, close all;
run 'motor_specs.m' % Get motor parameters


C = [1, 0; 0, 1]; % State to output vector

ss3 = ss(A,B,C,D); % Creates a state-space object

% [symG_num, symG_den] = ss2tf(A,B,C,D); % TF from SS, works with numeric

G = tf(ss3)               % speed/voltage transfer function (uses ss object)

opt = RespConfig;
opt.Amplitude = 36;

% Step responses
[y,t] = step(ss3,opt);

i_t = y(:,1);   % current [A]
w_t = y(:,2);   % angular velocity [rad/s]

figure('Name','Step response to 36 V');
subplot(2,1,1);
plot(t,i_t,'LineWidth',1.2);
grid on;
xlabel('Time [s]');
ylabel('Current [A]');
title('Current Response to 36 V Step');

subplot(2,1,2);
plot(t,w_t,'LineWidth',1.2);
grid on;
xlabel('Time [s]');
ylabel('Speed [rpm]');
title('Angular Velocity Response to 36 V Step');

% Frequency response (Bode)
figure('Name','Bode Plot');
bode(ss3);
grid on;
title('Frequency Response of Motor Model');


%% From no load speed to stall torque
clear, clc, close all
run 'motor_specs.m' % Get motor parameters

B = [(1/L), 0; 0, (-1/J)];  % Input to state vector
C = [1, 0; 0, 1];           % State to output vector
D = zeros(2,2);             % Feed through vector (No disturbance for now)

ss4 = ss(A,B,C,D);         % M, U --> [omega, i]
G = tf(ss4)                % [omega, i]
dcg = dcgain(G);   
Tstall = K_M * (36/R);     % stall torque
Tend = 7;                  % [s] (plot time)
N    = 3000;
offset = 500;
t    = linspace(0, Tend, N+offset).';

U_in   = U_N * ones(N+offset,1);   % constant 36 V
tauL = linspace(0, Tstall, N).';   % Creates a column vector with N evenly spaced values from 0-Tstall
tauL = [zeros(offset,1); tauL];    % Created a column vector with offset zeros followed by tauL
Umat = [U_in, tauL];

[x, ~, ~] = lsim(G, Umat, t, [0;0]);

i_t = x(:,1);   % Current as a function of time
w_t = x(:,2);   % Angular velocity as a function of time


figure('Name','Steady-state maps (U=36V)');
plot(1000*tauL, w_t,'LineWidth',1.2);
hold on;

% Horizontal line at no-load velocity 
no_load_vel_vector= dcg * [36,0;0,0 ]; 
no_load_vel = no_load_vel_vector(2)
yline(no_load_vel,'r--','No-load velocity');

% Vertical line at the final applied torque
xline(1000*tauL(end),'b--','Final torque');

hold off;

xlabel('Applied Torque [mN·m]');
ylabel('Angular Velocity [rad/sec]');
title('Motor Angular Velocity Response');
grid on;

%slope = (w_t(1500) - w_t(500)) / (tauL(1500) - tauL(500)); % Slope

%Prints out stall torque calculated from data sheet and our calculated
Tstall*1000
calculated_stall = K_M * i_t(end)

%% Calculated gear ratio (Task5)

clc
run 'motor_specs.m' 

totalEfficiency = 1;    % [-] (eta_gs)

torqueSection1 = calculated_stall;
torqueSection2 = maximumTorque;
%torqueSection2 = minimumTorque;

totalGearRatio = torqueSection2 / (torqueSection1 * totalEfficiency) % n

gearRatioSection1 = sqrt(totalGearRatio) % n_s1

%% Task6

clc
run 'motor_specs.m' 

% Angle gear

radiusHorisontalAngle = ([10, 11, 14, 19.6] ./ 2) .* 1E-3; % [m]
radiusVerticalAngle = ([27.7, 9.55] ./ 2) .* 1E-3; % [m]

volumeHorisontalAngle = [(24.5 * (((10/2)^2)*pi)) , (19.6 * (((11/2)^2)*pi)) , (13.9 * (((14/2)^2)*pi)) , (13.1 * (((19.6/2)^2)*pi))] .* 1E-9; % [m]
volumeVerticalAngle = [(7.042 * (((27.7/2)^2)*pi)) , ((9.55^2) + (9.55^2))] .* 1E-9; % [m]
m_horisontalAngle = densitySteel .* volumeHorisontalAngle; % [kg]
m_verticalAngle = densitySteel .* volumeVerticalAngle;     % [kg]

J_horisontalAngle = (1/2) .* (m_horisontalAngle .* (radiusHorisontalAngle.^2));     % [kgm^2]
J_verticalAngle = [(1/2), (1/12)] .* (m_verticalAngle .* (radiusVerticalAngle.^2)); % [kgm^2]

J_angle = sum(J_horisontalAngle) + sum(J_verticalAngle); % [kgm^2]

% Gear inertia

J_gear = J_gs2 + (J_gs1 / totalGearRatio); % [kgm^2]

% Middle axis inertia

radiusSquaredMiddleAxis = [(5.9/2)^2 , ((9/2)^2 + (11/2)^2)] .* 1E-6;
volumeMiddleAxis = [((5.9/2)^2 * (40.3 - 16.2)), (((11/2)^2 * 16.2) - ((9/2)^2 * 14))] .* 1E-9;
m_middleAxis = densitySteel .* volumeMiddleAxis;
J_middleAxis = sum(((1/2) * m_middleAxis) .* radiusSquaredMiddleAxis);

% Matrixes

d1 = d_j; % [Nm*s/rad]
d2 = d_t; % [Nm*s/rad]
d3 = b; % [Nm*s/rad] motor damping

dampingMatrix = [(d1 + d2) -d2; d2 (d2 + d3)]; % [Nm*s/rad]
inertiaMatrix = [J_angle 0; 0 J_gear + J]; % [kgm^2]

% Stiffness matrix

k1 = 0;   % [Nm/rad] k_j assumed 0 (no screw stiffness)
k2 = k_t; % [Nm/rad]
k3 = 0;   % [Nm/rad] Rigid motor axis 

stiffnessMatrix = [(k1 + k2) -k2; -k2 (k2 + k3)]; % [Nm/rad]

% State space model

% E = eye(2);
% A6 = [zeros(2) E; ((-inv(inertiaMatrix)) * stiffnessMatrix), ((-inv(inertiaMatrix)) * dampingMatrix)];
% B6 = [zeros(2) inv(inertiaMatrix)]'
% C6 = [0 1 0 0]
% %C6 = [1 0 0 0; 0 1 0 0; 0 0 0 0; 0 0 0 0]
% D6 = [0 0] %zeros(1,4)

J_T = J_angle
n = totalGearRatio
J_gm = J_gear + J + J_middleAxis

A6 = [ 0, 0, 1, 0;
      0, 0, 0, 1;
     -k_t/(J_gm*n^2), k_t/(J_gm*n), (((-K_E*K_M)/R) - b - (d_t/(n^2)))/J_gm, d_t/(J_gm*n);
      k_t/(J_T*n), -k_t/J_T, -d_t/(J_T*n), -(d_t+d_j)/J_T ];

B6 = [0, 0, K_M/(J_gm*R), 0]';

C6 = eye(4);

D6 = 0;%zeros(4,2);

stateSpaceFouthOrder = ss(A6,B6,C6,D6)

t6TransferFunction = tf(stateSpaceFouthOrder)

% Symbolic values

syms phi_dotdot1 phi_dotdot2 phi_dot1 phi_dot2 phi1 phi2

angularAcceleration = [phi_dotdot1, phi_dotdot2];
angularVelocity = [phi_dot1, phi_dot2];
angles = [phi1, phi2];

%% Task 8

clc
run 'motor_specs.m' 

k_j = (A_M8*E_steel) / L_free;

k_theta = k_j * (L_p / (2 * pi))^2

A8 = [ 0, 0, 1, 0;
      0, 0, 0, 1;
     -k_t/(J_gm*n^2), k_t/(J_gm*n), (((-K_E*K_M)/R) - b - (d_t/(n^2)))/J_gm, d_t/(J_gm*n);
      k_t/(J_T*n), -(k_t + k_theta)/J_T, -d_t/(J_T*n), -(d_t+d_j)/J_T ]