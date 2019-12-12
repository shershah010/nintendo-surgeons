function plot_all_flex_angle(dataset, pwm_step, trial_step)
%Simply displays flex/angle measurements over time for the given
%dataset as returned by importdata
hold on
dim = size(dataset);
rows = dim(1);
cols = dim(2);

for r = 1:pwm_step:rows
    for c = 1:trial_step:cols
        data_cur = dataset{r, c};
        time_vals = data_cur.time;
        flex_vals = data_cur.left_flex;
        pressure_vals = data_cur.left_pressure;
        angle_vals = calc_angle(data_cur);
        plot(time_vals, flex_vals);
        plot(time_vals, angle_vals);
        plot(time_vals, pressure_vals);
    end
end