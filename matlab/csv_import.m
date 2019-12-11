clear all
close all
clc

data_dir = '../data/doomed/';
files = dir(strcat(data_dir, '*.csv'));
all_data{10, 10} = {};
dim = size(files);
len = dim(1);

for i = 1:len
    file_name = strcat(data_dir, files(i).name);
    % Thanks Tom https://www.mathworks.com/matlabcentral/answers/43244-selecting-only-the-numbers-from-a-string-variable
    id = regexp(file_name,'\d*','Match');
    pwm = (str2double(id(1)) / 10) + 1;
    trial = str2double(id(2)) + 1;
    % Read in first ~1700 lines from CSV file
    data_cur = importdata(strcat(data_dir, files(i).name), 2, 4800);
    all_data{pwm, trial} = data_cur;
end
