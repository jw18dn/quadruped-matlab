function [M,B,GC] = DynamicsSSAuto(in1,in2)
%DYNAMICSSSAUTO
%    [M,B,GC] = DYNAMICSSSAUTO(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.2.
%    27-Oct-2023 13:29:54

dq3 = in1(10,:);
dq4 = in1(11,:);
dq5 = in1(12,:);
dq6 = in1(13,:);
dq7 = in1(14,:);
q3 = in1(3,:);
q4 = in1(4,:);
q5 = in1(5,:);
q6 = in1(6,:);
q7 = in1(7,:);
t2 = q3+q4+q5;
t3 = sin(t2);
t4 = q3+q4;
t5 = sin(t4);
t6 = q3+q6+q7;
t7 = sin(t6);
t8 = q3+q6;
t9 = sin(t8);
t10 = cos(t2);
t11 = t10.*1.156e-1;
t12 = cos(t4);
t13 = t12.*3.468e-1;
t14 = cos(t6);
t15 = t14.*1.156e-1;
t16 = cos(t8);
t17 = t16.*3.468e-1;
t34 = t3.*1.156e-1;
t35 = t5.*3.468e-1;
t43 = t7.*1.156e-1;
t44 = t9.*3.468e-1;
t18 = -t34-t35-t43-t44;
t19 = t11+t13+t15+t17;
t20 = q4+q5;
t21 = cos(t20);
t22 = cos(q4);
t23 = cos(q5);
t24 = t23.*7.8608e-2;
t25 = q6+q7;
t26 = cos(t25);
t27 = cos(q6);
t28 = cos(q7);
t29 = t28.*7.8608e-2;
t30 = t26.*3.468e-2;
t31 = sin(q3);
t32 = cos(q3);
t33 = t32.*(3.0./1.0e1);
t36 = t11+t13;
t41 = t21.*3.468e-2;
t37 = t22.*(-1.0404e-1)+t24-t41+1.310133333333333e-1;
t38 = t23.*3.9304e-2;
t39 = t3.*(1.7e1./5.0e1);
t40 = t5.*(1.7e1./5.0e1);
t42 = t38+2.620266666666667e-2;
t45 = t15+t17;
t46 = t27.*1.0404e-1;
t47 = t29+t30+t46+1.310133333333333e-1;
t48 = t28.*3.9304e-2;
t49 = t7.*(1.7e1./5.0e1);
t50 = t9.*(1.7e1./5.0e1);
t51 = t30+t48+2.620266666666667e-2;
t52 = t48+2.620266666666667e-2;
t53 = t31.*(3.0./1.0e1);
t54 = t10.*(1.7e1./5.0e1);
t55 = t12.*(1.7e1./5.0e1);
t56 = t14.*(1.7e1./5.0e1);
t57 = t16.*(1.7e1./5.0e1);
M = reshape([4.772e1,0.0,t18,-t34-t35,-t34,-t43-t44,-t43,1.0,0.0,1.0,0.0,0.0,4.772e1,t19,t36,t11,t45,t15,0.0,1.0,0.0,1.0,t18,t19,t21.*(-6.936e-2)-t22.*2.0808e-1+t24+t26.*6.936e-2+t27.*2.0808e-1+t29+2.606826666666667,t37,t38-t41+2.620266666666667e-2,t47,t51,-t39-t40+t53,-t33+t54+t55,-t49-t50-t53,t33+t56+t57,t3.*(-1.156e-1)-t5.*3.468e-1,t36,t37,t24+1.310133333333333e-1,t42,0.0,0.0,-t39-t40,t54+t55,0.0,0.0,t3.*(-1.156e-1),t11,t21.*(-3.468e-2)+t38+2.620266666666667e-2,t42,2.620266666666667e-2,0.0,0.0,-t39,t54,0.0,0.0,t7.*(-1.156e-1)-t9.*3.468e-1,t45,t47,0.0,0.0,t29+1.310133333333333e-1,t52,0.0,0.0,-t49-t50,t56+t57,t7.*(-1.156e-1),t15,t51,0.0,0.0,t52,2.620266666666667e-2,0.0,0.0,-t49,t56,-1.0,0.0,t31.*(-3.0./1.0e1)+t39+t40,t39+t40,t39,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-1.0,t10.*(-1.7e1./5.0e1)-t12.*(1.7e1./5.0e1)+t33,t10.*(-1.7e1./5.0e1)-t12.*(1.7e1./5.0e1),t10.*(-1.7e1./5.0e1),0.0,0.0,0.0,0.0,0.0,0.0,-1.0,0.0,t49+t50+t53,0.0,0.0,t49+t50,t49,0.0,0.0,0.0,0.0,0.0,-1.0,t14.*(-1.7e1./5.0e1)-t16.*(1.7e1./5.0e1)-t33,0.0,0.0,t14.*(-1.7e1./5.0e1)-t16.*(1.7e1./5.0e1),t14.*(-1.7e1./5.0e1),0.0,0.0,0.0,0.0],[11,11]);
if nargout > 1
    B = reshape([0.0,0.0,0.0,-1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-1.0,0.0,0.0,0.0,0.0],[11,4]);
end
if nargout > 2
    t58 = dq3.^2;
    t59 = dq4.^2;
    t60 = dq6.^2;
    t61 = dq5.^2;
    t62 = dq7.^2;
    t63 = sin(t20);
    t64 = sin(t25);
    t65 = sin(q4);
    t66 = sin(q5);
    t67 = sin(q6);
    t68 = sin(q7);
    t69 = t10.*1.134036;
    t70 = t12.*3.402108;
    t71 = t14.*1.134036;
    t72 = t16.*3.402108;
    t73 = t58.*t64.*3.468e-2;
    t74 = t32.*t58.*(3.0./1.0e1);
    t75 = t31.*t58.*(3.0./1.0e1);
    GC = [t10.*t58.*(-1.156e-1)-t10.*t59.*1.156e-1-t12.*t58.*3.468e-1-t10.*t61.*1.156e-1-t12.*t59.*3.468e-1-t14.*t58.*1.156e-1-t14.*t60.*1.156e-1-t16.*t58.*3.468e-1-t14.*t62.*1.156e-1-t16.*t60.*3.468e-1-dq3.*dq4.*t10.*2.312e-1-dq3.*dq5.*t10.*2.312e-1-dq3.*dq4.*t12.*6.936e-1-dq4.*dq5.*t10.*2.312e-1-dq3.*dq6.*t14.*2.312e-1-dq3.*dq7.*t14.*2.312e-1-dq3.*dq6.*t16.*6.936e-1-dq6.*dq7.*t14.*2.312e-1;t3.*t58.*(-1.156e-1)-t3.*t59.*1.156e-1-t5.*t58.*3.468e-1-t3.*t61.*1.156e-1-t5.*t59.*3.468e-1-t7.*t58.*1.156e-1-t7.*t60.*1.156e-1-t9.*t58.*3.468e-1-t7.*t62.*1.156e-1-t9.*t60.*3.468e-1-dq3.*dq4.*t3.*2.312e-1-dq3.*dq5.*t3.*2.312e-1-dq3.*dq4.*t5.*6.936e-1-dq4.*dq5.*t3.*2.312e-1-dq3.*dq6.*t7.*2.312e-1-dq3.*dq7.*t7.*2.312e-1-dq3.*dq6.*t9.*6.936e-1-dq6.*dq7.*t7.*2.312e-1+4.681332e2;t69+t70+t71+t72+t59.*t63.*3.468e-2+t59.*t65.*1.0404e-1-t60.*t64.*3.468e-2+t61.*t63.*3.468e-2-t62.*t64.*3.468e-2-t60.*t67.*1.0404e-1-t61.*t66.*3.9304e-2-t62.*t68.*3.9304e-2+dq3.*dq4.*t63.*6.936e-2+dq3.*dq5.*t63.*6.936e-2+dq3.*dq4.*t65.*2.0808e-1+dq4.*dq5.*t63.*6.936e-2-dq3.*dq6.*t64.*6.936e-2-dq3.*dq5.*t66.*7.8608e-2-dq3.*dq7.*t64.*6.936e-2-dq4.*dq5.*t66.*7.8608e-2-dq3.*dq6.*t67.*2.0808e-1-dq6.*dq7.*t64.*6.936e-2-dq3.*dq7.*t68.*7.8608e-2-dq6.*dq7.*t68.*7.8608e-2;t69+t70-t58.*t63.*3.468e-2-t58.*t65.*1.0404e-1-t61.*t66.*3.9304e-2-dq3.*dq5.*t66.*7.8608e-2-dq4.*dq5.*t66.*7.8608e-2;t69-t58.*t63.*3.468e-2+t58.*t66.*3.9304e-2+t59.*t66.*3.9304e-2+dq3.*dq4.*t66.*7.8608e-2;t71+t72+t73+t58.*t67.*1.0404e-1-t62.*t68.*3.9304e-2-dq3.*dq7.*t68.*7.8608e-2-dq6.*dq7.*t68.*7.8608e-2;t71+t73+t58.*t68.*3.9304e-2+t60.*t68.*3.9304e-2+dq3.*dq6.*t68.*7.8608e-2;t74-t10.*t58.*(1.7e1./5.0e1)-t10.*t59.*(1.7e1./5.0e1)-t12.*t58.*(1.7e1./5.0e1)-t10.*t61.*(1.7e1./5.0e1)-t12.*t59.*(1.7e1./5.0e1)-dq3.*dq4.*t10.*(1.7e1./2.5e1)-dq3.*dq5.*t10.*(1.7e1./2.5e1)-dq3.*dq4.*t12.*(1.7e1./2.5e1)-dq4.*dq5.*t10.*(1.7e1./2.5e1);t75-t3.*t58.*(1.7e1./5.0e1)-t3.*t59.*(1.7e1./5.0e1)-t5.*t58.*(1.7e1./5.0e1)-t3.*t61.*(1.7e1./5.0e1)-t5.*t59.*(1.7e1./5.0e1)-dq3.*dq4.*t3.*(1.7e1./2.5e1)-dq3.*dq5.*t3.*(1.7e1./2.5e1)-dq3.*dq4.*t5.*(1.7e1./2.5e1)-dq4.*dq5.*t3.*(1.7e1./2.5e1);-t74-t14.*t58.*(1.7e1./5.0e1)-t14.*t60.*(1.7e1./5.0e1)-t16.*t58.*(1.7e1./5.0e1)-t14.*t62.*(1.7e1./5.0e1)-t16.*t60.*(1.7e1./5.0e1)-dq3.*dq6.*t14.*(1.7e1./2.5e1)-dq3.*dq7.*t14.*(1.7e1./2.5e1)-dq3.*dq6.*t16.*(1.7e1./2.5e1)-dq6.*dq7.*t14.*(1.7e1./2.5e1);-t75-t7.*t58.*(1.7e1./5.0e1)-t7.*t60.*(1.7e1./5.0e1)-t9.*t58.*(1.7e1./5.0e1)-t7.*t62.*(1.7e1./5.0e1)-t9.*t60.*(1.7e1./5.0e1)-dq3.*dq6.*t7.*(1.7e1./2.5e1)-dq3.*dq7.*t7.*(1.7e1./2.5e1)-dq3.*dq6.*t9.*(1.7e1./2.5e1)-dq6.*dq7.*t7.*(1.7e1./2.5e1)];
end