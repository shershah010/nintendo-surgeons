function dynamics2 = dynamics2_gen(q,dq,C1,C2,D,Tau)
%DYNAMICS2_GEN
%    DYNAMICS2 = DYNAMICS2_GEN(Q,DQ,C1,C2,D,TAU)

%    This function was generated by the Symbolic Math Toolbox version 8.4.
%    09-Dec-2019 15:26:10

t2 = sin(q);
t3 = q./2.0;
t4 = q.*1.0e+4;
t6 = q+1.0e-4;
t5 = t4+1.0;
t7 = cos(t6);
t8 = sin(t6);
t12 = t3+5.0e-5;
t9 = 1.0./t5;
t13 = t7./2.0;
t14 = cos(t12);
t15 = sin(t12);
t10 = t9.^2;
t11 = t9.^3;
t16 = t13-1.0./2.0;
t17 = t15.*(2.1e+1./1.0e+3);
t18 = t14.*(2.1e+1./1.0e+3);
t19 = t7.*t9.*4.2e+2;
t20 = t8.*t9.*4.2e+2;
t21 = -t19;
t22 = t8.*t10.*4.2e+6;
t23 = t10.*t16.*8.4e+6;
t24 = t17+t21+t22;
t27 = t18+t20+t23;
t25 = t24.^2;
t28 = t27.^2;
t26 = t25.*(7.0./2.0e+2);
t29 = t28.*(7.0./2.0e+2);
t30 = t26+t29;
t31 = 1.0./t30;
dynamics2 = [dq;Tau.*t31+t31.*(t15.*7.21035e-3-t7.*t9.*1.44207e+2+t8.*t10.*1.44207e+6-1.0./q.^3.*t2.^4.*(q.^4.*1.0./t2.^4-1.0).*(C1.*2.0+C2.*(q./t2-t2./q).^2.*4.0))-dq.*t31.*(D-dq.*(t27.*(t15.*1.05e-2+t21+t8.*t10.*8.4e+6+t11.*t16.*1.68e+11).*(7.0./2.0e+2)-t24.*(t14.*1.05e-2+t20+t7.*t10.*8.4e+6-t8.*t11.*8.4e+10).*(7.0./2.0e+2)))];
