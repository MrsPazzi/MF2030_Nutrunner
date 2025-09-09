% 036BX4 motor specs
U_N = 36;
I0 = 0.124;
J = 6.3E-6;
R = 3.23;
K_M = 0.0644;
K_E = 0.006741;
C_V = 1.1E-6;
L = 0.000238;

A = [(-R/L), (-K_E/L); K_M/J, (-C_V/J)];  % State matrix
B = [(1/L), 0]';                          % Input to state vector
C = [0, 1];                               % State to output vector
D = 0;                                    % Feed through vector (No disturbance for now)