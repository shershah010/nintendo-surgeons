clear all
close all
clc

%% Load CSV
%[comp(1:5, :); comp(6:10, :); comp(11:15, :); comp(16:20, :); comp(21: 25, :)]
%q = a * flex^2 + b*flex + c
%a = .0004757;
%b = -.629484;
%c = 206.86;

%q = b_0 + pwmFileb_1* left_flex + b_2 * exp(-left_flex/b3)
%b_0 = -53034.36
%b_1 = 6.71203
%b_2 = 53243.43
%b_3 = 7237.643
a = -.986365423;
b = .00088301426;
c = 274.448006079;
f
pwmFile = readtable('..\data\longer\longer_ss080_2.csv');
flex = table2array(pwmFile(:, 'left_flex'));
y_dif = table2array(pwmFile(:, 'tip_pos_y')) - table2array(pwmFile(:, 'base_pos_y'));
x_dif = table2array(pwmFile(:, 'tip_pos_x')) - table2array(pwmFile(:, 'base_pos_x'));


%% Generate q(t)

%q = b_0 + b_1 * flex + b_2 * exp(-flex * 1/b_3)
q = a * flex + b * flex.*flex + c;
q_measured = asin(x_dif ./ sqrt(x_dif.^2 + y_dif.^2)) * 180/pi;
q = pi * q/90;
q_measured = pi* q_measured/90;
u = table2array(pwmFile(:, 'left_pwm'));
time = table2array(pwmFile(:, 'time'));
%% Package paths

cur = pwd;
addpath( genpath( [cur, '/gen/' ] ));

%% testing with generated data
% Note the values used are totally made up and shouldn't be used as
% starting points for your actual analysis

alpha = 0.0680;
gamma = 0.2518;
% for pwm in pwms_clean:
tau0 = 0;
dtau0 = 0;

K = 8.7143;
D = 0.0378;
q0 = 0;
dq0 = 0;
% 
% 
x = [K D alpha gamma];
ub = zeros(4);
lb = zeros(4);
% 
 cost = @(x) cost_function(x(1),x(2),x(3),x(4), time, u, q_measured, q0, dq0, tau0, dtau0);
% 
% ,
%[X, resnorm] = lsqnonlin(cost, [2.6, 0.5, 0.31, 0.6])
disp('wait..............starting non linear least square fit.......Have faith')
options = optimoptions(@fminunc,'Display','iter','Algorithm','quasi-newton');
[X, resnorm] = lsqnonlin(cost, x,0.0,Inf,options)
pwm = max(table2array(pwmFile(:,'left_pwm')));
dlmwrite('../data/longer/nlin.csv',[X,pwm],'-append');
% 

% clear all
% close all
% clc

%alternate regression
%% Linear regression to find K and alpha
% pwmtotheta = readtable('..\data\pwm_to_theta.csv');
% u_ss = table2array(pwmtotheta(:, 'pwm'));
% q_ss = table2array(pwmtotheta(:, 'theta'));
% q_ss = 2 * q_ss;
% G = G_gen(q_ss);
% 
% comp = [u_ss q_ss G];
% 
% comp1 = comp(1:5, :)
% comp2 = comp(6:10, :)
% comp3 = comp(11:15, :)
% comp4 = comp(16:20, :)
% comp5 = comp(21:25, :)
% comp6 = comp(26:30, :)
% comp7 = comp(31:35, :)
% comp8 = comp(36: 40, :)
% comp9 = comp(41:45, :)
% comp10 = comp(46: 50, :)
% 
% 
% s = 10;
% H = zeros(s, 2);
% for i = 1:s
%     i_1 = 1 + 5 *(i - 1)
%     i_2 = 5*i, 1:2
%     lsq_A = [-1 * comp(i_1:i_2, 1), comp(i_1:i_2, 2)]
%     lsq_B = -1 * comp(i_1:i_2, 3)
%     x = lsq_A \ lsq_B
%     H(i, :) = transpose(x)
% end
% 
% 
% 
