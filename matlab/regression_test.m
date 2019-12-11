data = my_data;

data_dim = size(data);
rows = data_dim(1);
cols = data_dim(2);

combined = zeros(1, 4);

window_size = 20;
h = ones(1, window_size)/window_size;


for r = 1:rows
    for c = 1:cols
        data_cur = data{r, c};
        der_data = der_filt(data_cur, 1, 1 , 1);
        last_dim = size(der_data);
        index_lim = der_data(1:last_dim(1), 1);
        ang = data_cur.angle(index_lim);
        ang = conv(ang, h, 'same');
        combined_cur = [ang, der_data(1:last_dim(1), 2:4)];
        combined = [combined; combined_cur];
    end
end

% angle = combined(:, 1)
% angle = conv(angle, h, 'same');
% last_dim = size(combined);
% index_lim = combined(75:last_dim(1), 2);
% index_lim = der_data(:, 1);
% ang_act = angle(index_lim);
% dim = size(ang_act);
B = combined(:, 1);
A = zeros(dim(1), 4);
A(:, 1:3) = combined(:, 2:4);
% A(:, 1:3) = der_data(:, 2:4);
A(:, 4) = ones(dim(1), 1);

% angle_test = my_test.angle;
% flex_test = my_test.flex;
% time_test = my_test.time

X = A\B;
predicted = A * X;
window_size = 20;

%plot(time_test, angle_test);
h = ones(1, window_size)/window_size;
predicted = conv(predicted, h, 'same');
plot(data.time(index_lim), predicted)
hold on
plot(data.time, angle)
hold off