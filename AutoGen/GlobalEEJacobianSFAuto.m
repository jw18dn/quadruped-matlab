function H = GlobalEEJacobianSFAuto(in1)
%GLOBALEEJACOBIANSFAUTO
%    H = GLOBALEEJACOBIANSFAUTO(IN1)

%    This function was generated by the Symbolic Math Toolbox version 8.2.
%    27-Oct-2023 13:29:57

q3 = in1(3,:);
q4 = in1(4,:);
q5 = in1(5,:);
t2 = sin(q3);
t3 = cos(q3);
t4 = sin(q4);
t5 = cos(q4);
t6 = cos(q5);
t7 = t3.*t4;
t8 = t2.*t5;
t9 = t7+t8;
t10 = sin(q5);
t11 = t2.*t4;
t13 = t3.*t5;
t14 = t11-t13;
t12 = t10.*t14.*(1.7e1./5.0e1);
t15 = t3.*t5.*(1.7e1./5.0e1);
H = reshape([1.0,0.0,0.0,1.0,t2.*(3.0./1.0e1)+t12-t2.*t5.*(1.7e1./5.0e1)-t3.*t4.*(1.7e1./5.0e1)-t6.*t9.*(1.7e1./5.0e1),t3.*(-3.0./1.0e1)+t15-t2.*t4.*(1.7e1./5.0e1)-t9.*t10.*(1.7e1./5.0e1)-t6.*t14.*(1.7e1./5.0e1),t12-t2.*t5.*(1.7e1./5.0e1)-t3.*t4.*(1.7e1./5.0e1)-t6.*t9.*(1.7e1./5.0e1),t15-t2.*t4.*(1.7e1./5.0e1)-t9.*t10.*(1.7e1./5.0e1)-t6.*t14.*(1.7e1./5.0e1),t12-t6.*t9.*(1.7e1./5.0e1),t9.*t10.*(-1.7e1./5.0e1)-t6.*t14.*(1.7e1./5.0e1),0.0,0.0,0.0,0.0],[2,7]);
