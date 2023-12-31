function H = GlobalEEJacobianFSAuto(in1)
%GLOBALEEJACOBIANFSAUTO
%    H = GLOBALEEJACOBIANFSAUTO(IN1)

%    This function was generated by the Symbolic Math Toolbox version 8.2.
%    27-Oct-2023 13:30:01

q3 = in1(3,:);
q6 = in1(6,:);
q7 = in1(7,:);
t2 = sin(q3);
t3 = cos(q3);
t4 = sin(q6);
t5 = cos(q6);
t6 = cos(q7);
t7 = t3.*t4;
t8 = t2.*t5;
t9 = t7+t8;
t10 = sin(q7);
t11 = t2.*t4;
t13 = t3.*t5;
t14 = t11-t13;
t12 = t10.*t14.*(1.7e1./5.0e1);
t15 = t3.*t5.*(1.7e1./5.0e1);
H = reshape([1.0,0.0,0.0,1.0,t2.*(-3.0./1.0e1)+t12-t2.*t5.*(1.7e1./5.0e1)-t3.*t4.*(1.7e1./5.0e1)-t6.*t9.*(1.7e1./5.0e1),t3.*(3.0./1.0e1)+t15-t2.*t4.*(1.7e1./5.0e1)-t9.*t10.*(1.7e1./5.0e1)-t6.*t14.*(1.7e1./5.0e1),0.0,0.0,0.0,0.0,t12-t2.*t5.*(1.7e1./5.0e1)-t3.*t4.*(1.7e1./5.0e1)-t6.*t9.*(1.7e1./5.0e1),t15-t2.*t4.*(1.7e1./5.0e1)-t9.*t10.*(1.7e1./5.0e1)-t6.*t14.*(1.7e1./5.0e1),t12-t6.*t9.*(1.7e1./5.0e1),t9.*t10.*(-1.7e1./5.0e1)-t6.*t14.*(1.7e1./5.0e1)],[2,7]);
