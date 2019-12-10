clear all
close all
clc

data_dir = '../data/longer/';
files = dir(data_dir);
all_data{10, 10} = {};


for i = 4:103
    file_name = strcat(data_dir, files(i).name);
    % Thanks Tom https://www.mathworks.com/matlabcentral/answers/43244-selecting-only-the-numbers-from-a-string-variable
    id = regexp(file_name,'\d*','Match');
    pwm = str2double(id(1))/20;
    trial = str2double(id(2)) + 1;
    % Read in first ~1700 lines from CSV file
    data_cur = importdata(strcat(data_dir, files(i).name), 2, 1700);
    all_data{pwm, trial} = data_cur;
end
