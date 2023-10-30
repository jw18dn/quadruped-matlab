function [] = LegDynamics(params)
% Dynamics for the leg in the local (or body) coordinates. We made some
% minor changes and may need to compare this to older functions

% Independent Variables
g = params.g; 
l = params.leg.l;
m = params.leg.m;
Ixx = params.leg.Ixx;
Iyy = params.leg.Iyy;
Izz = params.leg.Izz;
Z = zeros(3);
syms t 'real'

% Dependent Variables 
q = sym('q',[2,1],'real');              % [theta_1 theta_2]
dq = sym('dq',[2,1],'real');            % velocity 
d2q = sym('d2q',[2,1],'real');          % acceleration

% Control Inputs
u = sym('u',[2,1],'real');              % control inputs

% Operational Space Controller Values
qT_des = sym('qT_des',[2,1],'real');    % Desired end effector position
dqT_des = sym('dqT_des',[2,1],'real');  % Desired end effector velocity
ddqT_des = sym('dqT_des',[2,1],'real'); % Desired end effector acceleration
Kp = sym('Kp',[2,1],'real');
Kd = sym('Kd',[2,1],'real');

% Variable Vectors for Equations
states = {q(1); q(2); dq(1); dq(2)};

%% Transformation Maricies 
% Pivot point centered at [0,0]
% [Rotation, Translation
%      0    ...    1]

% Joints - Kinematic Chain
T_joint(:,:,1) = TranslationMatrix(q(1),l(1));                  % q(1) = theta_1     
T_joint(:,:,2) = T_joint(:,:,1)*TranslationMatrix(q(2),l(2));   % q(2) = theta_2     

% Center of mass
T_com(:,:,1) = TranslationMatrix(q(1),l(1)/2); 
T_com(:,:,2) = T_joint(:,:,1)*TranslationMatrix(q(2),l(2)/2); 

%% Cartesian Positions
for i = 1:size(T_joint,3)
    % Joint/Member End Position
    j(:,i) = T_joint(1:3,end,i);       

    % Center of Mass Equations
    n(:,i) = T_com(1:3,end,i);   
end

% Rotational - about the y axis
w(:,1) = [0; q(1); 0];
w(:,2) = [0; q(1) + q(2); 0];

% Joint Kinematic Function Development
% matlabFunction(j,'File',[pwd '\AutoGen\LocalLegKinematicsAuto'],'Vars',{q});
matlabFunction(j(:,end),'File',[pwd '\AutoGen\LocalEEKinematicsAuto'],'Vars',{q});

%% Cartesian Velocities
% Joint Translational
dj = simplify(fulldiff(j,states));

% COM Translational
dn = simplify(fulldiff(n,states));

% Rotational
dw(:,1) = [0; dq(1); 0];
dw(:,2) = [0; dq(1) + dq(2); 0];

% Joint Velocity Function
matlabFunction(dj(:,end),'File',[pwd '\AutoGen\LocalEEVelocityAuto'],'Vars',{q,dq});

%% Inertial Force Matrix
for i = 1:size(T_joint,3)
    % Translational Inertia Matrix
    It = m(i).*eye(3);

    % Rotational Inertia Matrix
    Ir = [Ixx(i); Iyy(i); Izz(i)].*eye(3);

    % Assemble the complete Inertial Matrix in cartesian space
    Mc(:,:,i) = [It,     Z;
                 Z,     Ir];
end

%% Gravitational Force Vector
for i = 1:size(T_joint,3)
    Gc(:,i) = m(i).* g .* [n(:,i); w(:,i)];
end

%% Joint Space Conversion
for i = 1:size(T_joint,3)
    % Center of Mass Jacobians
    J_com = jacobian([n(:,i); w(:,i)],q);

    % Inertial Matrix
    Mj(:,:,i) = J_com.' * Mc(:,:,i) * J_com;

    % Gravitational Forces
    % Because the gravitational force vector is only a function of
    % position, we cant use the jacobian for it and instead can use the
    % regular equations itself 'x(q) so no translation necessary'
    Gj(:,i) = Gc(:,i);
end

%% Energy Equations - In Cartesian Space
for i = 1:size(T_joint,3)
    % Kinetic Energy: T = 1/2 * [x; w]' * M * [x; w]
    T(:,i) = 1/2*[dn(:,i); dw(:,i)].'*Mc(:,:,i)*[dn(:,i); dw(:,i)];

    % Potential Energy: V = m * g * q 
    V(:,i) = Gc(:,i);
end
Tc_total = sum(T,'all');
Vc_total = sum(V,'all'); 

%% Energy Equations - In Joint Space
for i = 1:size(T_joint,3)
    % Kinetic Energy: T = 1/2 * dq' * M * dq
    T(:,i) = 1/2*dq.'*Mj(:,:,i)*dq;

    % Potential Energy: V = m * g * q - Translation from Cartesian to Joint Causes Issue
    V(:,i) = Gj(:,i);
end
Tj_total = sum(T,'all');
Vj_total = sum(V,'all'); 

%% Lagrange Formuation
% Lagrangian: L = T - V
% Lag = Tc_total - Vc_total; 
Lag = Tj_total - Vj_total; 

% L_partial/q_partial - d/dt * L_partial/dq_partial - R_partial/dq_partial = 0
L_q_partial = simplify(jacobian(Lag,q).');
L_dq_partial = simplify(jacobian(Lag,dq).');
dt_L_dq_partial = simplify(fulldiff(L_dq_partial,states));

% Non Conservative Forces
Q_nc = u(:);
  
% Combine Everything
Dynamics = simplify(dt_L_dq_partial - L_q_partial - Q_nc,3);

%% Acceleration Equations
% Seperate the equation into its parts: 0 = M(q)*ddq + B*u + C(q,dq) + G(q) 
M = simplify(jacobian(Dynamics,d2q));
B = simplify(jacobian(Dynamics,u));
GC = simplify(Dynamics - M*d2q - B*u);

% Solve for the Accelerations
Acc = -simplify(M\(B*u + GC));

% Function Development
% matlabFunction(Acc,'File',[pwd '\AutoGen\LocalLegDynamicsAuto'],'Vars',{t,[q; dq],u});

% Check the Energy Balance
% matlabFunction(Tj_total,Vj_total,'File',[pwd '\AutoGen\EnergyCheckAuto'],'Vars',{q,dq});
% 
%% Operational Space Controller
% End Effector Jacobian and its derivative
J_ee = jacobian([j(:,end); w(:,end)],q);
dJ_ee = fulldiff(J_ee,states);

% Jee is only inverable if it is square, our system is fully controllable
% so we can remove everything exceot for the x and z components
J_ee = [J_ee(1,:); J_ee(3,:)];
dJ_ee = [dJ_ee(1,:); dJ_ee(3,:)];

% Feed Forward Torque - Develop an expression for ddq using ddqT = dJ*dq + J*ddq. 
% If J_ee is not ivertable we would have to use a QP to solve this
ddq = J_ee\(ddqT_des - dJ_ee*dq);
torque_ff = -simplify(B\(M*ddq + GC));

% Set the desired accelerations equal to a PD controller
torque_adt = Kp.*(qT_des - j([1,3],end)) + Kd.*(dqT_des - dj([1,3],end));

% Combine the Torques
torque = torque_adt + torque_ff;

% Function Development
matlabFunction(J_ee,'File',[pwd '\AutoGen\LocalEEJacobianAuto'],'Vars',{q});
matlabFunction(torque,'File',[pwd '\AutoGen\LocalEEOSCAuto'],'Vars',{q,dq,qT_des,dqT_des,ddqT_des,Kp,Kd})
end
