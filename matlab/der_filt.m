function clean = der_filt(dataset, ker_size_filt, ker_size_der, der_window)
%Given a dataset in the form as returned by seperate_training_test, return the flex
%measurement after a moving average LPF over ker_size points as well as a
%numerical approximation of the derivative
flex = dataset.flex;
time = dataset.time;
h_filt = ones(ker_size_filt, 1)/ker_size_filt;
h_filt_der = ones(ker_size_der, 1)/ker_size_der;
f_filt = conv(flex, h_filt, 'same');
% b_filt = (1/ker_size_filt)*ones(1,ker_size_filt);
% b_der = (1/ker_size_der)*ones(1,ker_size_der);
% a = 1;
% f_filt = filter(b_filt, a, flex);
i_keep = ker_size_filt+1:size(flex) - ker_size_filt;
f_filt = f_filt(i_keep);
h_der = [1, zeros(1, der_window - 1), -1];
% f_diff = diff(f_filt);
% t_diff = diff(time(i_keep));
f_diff = conv(f_filt, h_der, 'same');
t_diff = conv(time(i_keep), h_der, 'same');
f_der = f_diff ./ t_diff;
% f_der = [0; f_der];
% f_der_filt = filter(b_der, a, f_der);
f_der_filt = conv(f_der, h_filt_der, 'same');
f_diff_2 = conv(f_der_filt, h_filt_der, 'same');
f_der_2 = f_diff_2 ./ t_diff;
% f_der_2 = [0; f_der_2];
clean = [i_keep', f_der_2, f_der_filt, f_filt];