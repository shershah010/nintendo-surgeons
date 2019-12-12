function u = u_model(traj_desired)
%Assume traj_desired is a 2d trajectory array [t_desired, q_desired].
%Return feed forward PWM based on generated dynamics model.

%load [INSERT DYNAMICS MODEL .MAT FILE HERE]
K = 8.8487;
D = 0.014824;
alpha = 0.097663;
gamma = 0.44373;

t_desired  = traj_desired(1);
q_desired = traj_desired(2);

q_diff = [0, diff(q_desired)];
t_diff = [0, diff(t_desired)];
q_desired_dot = q_diff ./ t_diff;
q_double_diff = [0, diff(q_desired_d)];
q_desired_double_dot = q_double_diff ./ t_diff
tau = M_gen(q_desired) * q_desired_double_dot + (C_gen(q_desired, q_desired_dot) + D)*q_desired(dot) + G_gen(q_desired) + K(q_desired)*q_desired;
tau_diff = [0, diff(tau_desired)];
tau_dot = tau_diff ./ t_diff;
tau_double_diff = [0, diff(tau_dot)];
tau_double_dot = tau_double_diff ./ t_diff
u = (gamma*gamma*tau_double_dot + 2*gamma*tau_dot + tau)./alpha