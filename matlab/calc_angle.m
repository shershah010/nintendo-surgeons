function angle = calc_angle(dataset)
%Given a dataset table as returned by importdata, return an array of angle
%measurements
y_dif = table2array(dataset(:, 'tip_pos_y')) - table2array(dataset(:, 'base_pos_y'));
x_dif = table2array(dataset(:, 'tip_pos_x')) - table2array(dataset(:, 'base_pos_x'));
angle = asin(x_dif ./ sqrt(x_dif.^2 + y_dif.^2)) * 180/pi;