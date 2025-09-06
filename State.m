J = 1;
R = 1;
K = 1;
k = 1;
b = 1;
L = 1;


A = [k/J, (-b/J); (-R/L), (-K/L)]; %State matrix
B = [0, (1/L)]'; %input vector
C = [1, 0]; % 
D = 0; %No disturbance

ss3 = ss(A,B,C,D);





