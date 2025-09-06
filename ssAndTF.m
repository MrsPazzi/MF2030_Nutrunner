clc, clear;

%% Symbolical representation (State-space and Transfer function)
syms R L k_e k_m J b s


A = [(-R/L), (-k_e/L); k_m/J, (-b/J)]; % State matrix
B = [(1/L), 0]'; % Input to state vector
C = [0, 1]; % State to output vector
D = 0; % Feed through vector (No disturbance for now)



I = eye(size(A));
G = simplify(C * ((s*I - A)\B) + D); % Transfer function theta_dot(s)/u(s)
pretty(G)


%% Numeric representation (State-space and Transfer function)
run 'motor_specs.m' % Get motor parameters

A = [(-R/L), (-k_e/L); k_m/J, (-b/J)]; % State matrix
B = [(1/L), 0]'; % Input to state vector
C = [0, 1]; % State to output vector
D = 0; % Feed through vector (No disturbance for now)

ss3 = ss(A,B,C,D); % Creates a state-space object

% [symG_num, symG_den] = ss2tf(A,B,C,D); % TF from SS, works with numeric

G = tf(ss3)               % speed/voltage transfer function (uses ss object)
dcg = dcgain(G);           % steady-state speed per volt (rad/s per V)
step(ss3)                 % speed response to 1 V step
grid on
figure, bode(ss3), grid on  % frequency response

no_load_speed = dcg * U_0