%clear all
%close all
%clc

%% Package paths

cur = pwd;
addpath( genpath( [cur, '/gen/' ] ));

%% testing with generated data
% Note the values used are totally made up and shouldn't be used as
% starting points for your actual analysis

alpha = .1;
gamma = 0.5;
tau0 = 0;
dtau0 = 0;

K = 3;
D = 0.4;
q0 = 0;
dq0 = 0;
% 
% 
% x = K, D, alpha, gamma;
% 
 cost = @(x) cost_function(x(1), x(2), x(3), x(4), time, u, q_measured, q0, dq0, tau0, dtau0);
% 
% 
%[X, resnorm] = lsqnonlin(cost, [2.6, 0.5, 0.31, 0.6])
[X, resnorm] = lsqnonlin(cost, [K, D, alpha, gamma])


%% 

%clear all
%close all
%clc

%% Load CSV

%q = a * flex^2 + b*flex + c
a = .0004757;
b = -.629484;
c = 206.86;

%q = b_0 + b_1* left_flex + b_2 * exp(-left_flex/b3)
b_0 = -53034.36
b_1 = 6.71203
b_2 = 53243.43
b_3 = 7237.643


flex = table2array(pwmFile(:, 'left_flex'));
y_dif = table2array(pwmFile(:, 'tip_pos_y')) - table2array(pwmFile(:, 'base_pos_y'));
x_dif = table2array(pwmFile(:, 'tip_pos_x')) - table2array(pwmFile(:, 'base_pos_x'));
time = table2array(pwmFile(:, 'time'));

%% Generate q(t)

q = b_0 + b_1 * flex + b_2 * exp(-flex * 1/b_3)
q_measured = asin(x_dif ./ sqrt(x_dif.^2 + y_dif.^2)) * 180/pi

%alternate regression
q_2 = 0.0004 * flex.^2 + -0.5643 * flex + 196.6876

[q_filter, locs] = lowpass(q_measured, .001)

%% do your regression

peaks, locs = findpeaks(q_filter, 'MinPeakHeight', 20, 'MinPeakDistance', 20)
