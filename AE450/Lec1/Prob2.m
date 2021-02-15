clc;
clear all; close all;

%% Prob 2a. State-space Equation
disp('>> Prob 2a. State-space Equation');

M = 2.0;
m = 0.1;
l = 0.5;
g = 9.81;

A = [0 1 0 0; ...
    (M+m)*g/M/l 0 0 0; ...
    0 0 0 1; ...
    -m*g/M 0 0 0]
B = [0; -1/M/l; 0; 1/M]

disp(repmat('=',1,80));

%% Prob 2b. Stability Analysis
disp('>> Prob 2b. Stability Analysis');

eig(A)

disp(repmat('=',1,80));

%% Prob 2c. Controllability Analysis
disp('>> Prob 2c. Controllability Analysis');

rank(ctrb(A,B))

disp(repmat('=',1,80));

%% Prob 2d. Full-state Feedback Controller
disp('>> Prob 2d. Full-state Feedback Controller');

P = [-5 -6 -7 -8];
K = acker(A,B,P)

A_cl = A-B*K;
% eig(A_cl)

C = eye(4);
D = 0;
% sys = ss(A,B,C,D);
sys_cl = ss(A_cl,B,C,D);

figure(1)
step(sys_cl); grid on;

disp(repmat('=',1,80));