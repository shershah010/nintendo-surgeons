function u = u_model(K, D, alpha, gamma, traj_desired)
%Assume traj_desired is a 2d trajectory array [t_desired, q_desired].
%Return feed forward PWM based on generated dynamics model.
t_desired  = traj_desired(1, :);
q_desired = traj_desired(2, :) * pi/90;

q_diff = diff(q_desired);
t_diff = diff(t_desired);
q_desired_dot = [0, q_diff ./ t_diff];
q_double_diff = diff(q_desired_dot);
q_desired_double_dot = [0, q_double_diff ./ t_diff];
tau_desired = M_gen(q_desired) .* q_desired_double_dot + (C_gen(q_desired, q_desired_dot) + D) .* q_desired_dot + G_gen(q_desired) + K .* q_desired;
tau_diff = diff(tau_desired);
tau_dot = [0, tau_diff ./ t_diff];
tau_double_diff = diff(tau_dot);
tau_double_dot = [0, tau_double_diff ./ t_diff];
u = ((gamma*gamma) * tau_double_dot + (2*gamma)*tau_dot + tau_desired)./alpha;