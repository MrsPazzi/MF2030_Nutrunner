clc, clear, close all;

%% Symbolical representation (State-space and Transfer function)
syms R L k_e k_m J b s


A = [(-R/L), (-k_e/L); k_m/J, (-b/J)]; % State matrix
B = [(1/L), 0]'; % Input to state vector
C = [0, 1]; % State to output vector
D = 0; % Feed through vector (No disturbance for now)



I = eye(size(A));
G = simplify(C * ((s*I - A)\B) + D); % Transfer function theta_dot(s)/u(s)
pretty(G)


%% Numeric representation (State-space and Transfer function) + step response (angular velocity)
clc, clear, close all;
run 'motor_specs.m' % Get motor parameters

A = [(-R/L), (-k_e/L); k_m/J, (-b/J)]; % State matrix
B = [(1/L), 0]'; % Input to state vector
C = [0, 1]; % State to output vector
D = 0; % Feed through vector (No disturbance for now)

ss3 = ss(A,B,C,D); % Creates a state-space object

% [symG_num, symG_den] = ss2tf(A,B,C,D); % TF from SS, works with numeric

G = tf(ss3)               % speed/voltage transfer function (uses ss object)
dcg = dcgain(G);           % steady-state speed per volt (rad/s per V)

opt = RespConfig;
opt.Amplitude = 36;


%step(ss3, opt)                 % speed response to 36 V step
%grid on
%figure, bode(ss3), grid on  % frequency response

no_load_speed = dcg * U_0
%% Numeric representation (State-space and Transfer function) + step response (current)
clc, clear, close all;
run 'motor_specs.m' % Get motor parameters

A = [(-R/L), (-k_e/L); k_m/J, (-b/J)]; % State matrix
B = [(1/L), 0]'; % Input to state vector
C = [1, 0; 0, 1]; % State to output vector
D = 0; % Feed through vector (No disturbance for now)


ss3 = ss(A,B,C,D); % Creates a state-space object

% [symG_num, symG_den] = ss2tf(A,B,C,D); % TF from SS, works with numeric

G = tf(ss3)               % speed/voltage transfer function (uses ss object)
dcg = dcgain(G);           % steady-state speed per volt (rad/s per V)

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

A = [(-R/L), (-k_e/L); k_m/J, (-b/J)]; % State matrix
B = [(1/L), 0; 0, (-1/J)]; % Input to state vector
C = [1, 0; 0, 1]; % State to output vector
D = zeros(2,2); % Feed through vector (No disturbance for now)



ss4 = ss(A,B,C,D);
G = tf(ss4)              
dcg = dcgain(G); 
Tstall = k_m * (36/R);
Tend = 7;                  % seconds
N    = 3000;
offset = 500;
t    = linspace(0, Tend, N+offset).';

V_in   = 36 * ones(N+offset,1);                  % constant 36 V
tauL = linspace(0, Tstall, N).';   % N×1 column
tauL = [zeros(offset,1); tauL];  
Umat = [V_in, tauL]; 

[x, ~, ~] = lsim(G, Umat, t, [0;0]);
i_t = x(:,1);
w_t = x(:,2);


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
ylabel('Angular Velocity [rad/min]');
title('Motor Angular Velocity Response');
grid on;

slope = (w_t(1500) - w_t(500)) / (tauL(1500) - tauL(500)); % Slope
