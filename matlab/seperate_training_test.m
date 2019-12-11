function data = seperate_training_test(dataset, training_trials)
%Assume dataset is a 2D cell array, each containing one trial.
%Rows of dataset should corrospond to data taken at different PWMs.
%Columns of dataset should corrospond to different trials of data at the same PWM.
%Each cell is a table of the form outputed by importdata function
%Function will take the first training_trials columns and concatenate into
%training data, remaining columns will be concatenated into test data.
%Will also generate array of steady state data for training data.%

dim = size(dataset);
rows = dim(1);
cols = dim(2);

ss_size = (training_trials + 1) * rows;
ss_split{ss_size, 1} = {};
ss_flex = zeros(ss_size, 1);
ss_angle = zeros(ss_size, 1);
ss_input = zeros(ss_size, 1);


all_time = zeros();
all_angle = zeros();
all_flex = zeros();
all_input = zeros();

%Compile training

%Make training data for 0pwm (kinda hacky?) by taking ~First 2 sec of each
%of first training data trials
for c = 1:training_trials
    data_cur = dataset{1, c};
    pump_active = data_cur.left_pwm > 0;
    time_active = data_cur.time(pump_active);
    low_time = min(time_active);
    ss_i = data_cur.time < low_time;
    ss_data = data_cur(ss_i, :);
    ss_flex(c) = mean(ss_data.left_flex);
    ss_input(c) = mean(ss_data.left_pwm);
    ss_angle(c) = mean(calc_angle(ss_data));
end

for r = 1:rows
    for c = 1:training_trials
        data_cur = dataset{r, c};
        pump_active  = data_cur.left_pwm > 0;
        filter_active = data_cur(pump_active, :);
        data_dim = size(filter_active);
        last_i = data_dim(1);
        last_t = filter_active.time(last_i);
        start_t = last_t - 5;
        time_c = filter_active.time;
        ss_i = time_c <= last_t & time_c >= start_t;
        ss_data = filter_active(ss_i, :);
        dim = size(ss_data);
        store_index = (r*training_trials + c);
%         ss_flex(store_index) = mean(ss_data.left_flex);
%         ss_input(store_index) = ss_data.left_pwm(1);
%         ss_angle(store_index) = mean(calc_angle(ss_data));
        ang = mean(calc_angle(ss_data));
        ss_angle = ones(dim(1), 1) * ang;
        ss_flex = ss_data.left_flex;
        ss_time =  ss_data.time;
        ss_input = ss_data.left_pwm;
        sample = table;
        sample.flex = ss_flex;
        sample.angle = ss_angle;
        sample.input = ss_input;
        sample.time = ss_time;
        ss_split{store_index, 1} = sample;
        
        all_angle = [all_angle; calc_angle(data_cur)];
        all_flex = [all_flex; data_cur.left_flex];
        all_input = [all_input; data_cur.left_pwm];
        last_i = size(all_time);
        last_time = all_time(last_i(1));
        all_time = [all_time; last_time + data_cur.time];
    end
end

% ss_table = table;
% ss_table.flex = ss_flex;
% ss_table.angle = ss_angle;
% ss_table.input = ss_input;
% data = ss_table
% data = ss_split

all_table = table;
all_table.time = all_time;
all_table.flex = all_flex;
all_table.input = all_input;
all_table.angle = all_angle;
data = all_table;
%Compile test
% for c = (training_trials+1):cols
%     for r = 1:rows
%         data_cur = datatset{r, c}
%     end
% end