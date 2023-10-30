function [] = FullSystemDynamics(params)
% Dynamics for the leg in the local (or body) coordinates. We made some
% minor changes and may need to compare this to older functions

% Independent Variables
g = params.g; 
l = [params.body.l; params.leg.l; params.leg.l];
m = [params.body.m; params.leg.m; params.leg.m];
Ixx = [params.body.Ixx; params.leg.Ixx; params.leg.Ixx];
Iyy = [params.body.Iyy; params.leg.Iyy; params.leg.Iyy];
Izz = [params.body.Izz; params.leg.Izz; params.leg.Izz];
Z = zeros(3);
syms t 'real'

% Dependent Variables 
q = sym('q',[7,1],'real');              % [x; y; B_theta; L1_theta1; L1_theta2; L2_theta1; L2_theta2]
dq = sym('dq',[7,1],'real');            % velocity 
d2q = sym('d2q',[7,1],'real');          % acceleration

% Control Inputs
u = sym('u',[4,1],'real');              

% Lagrangian Multipliers
lam = sym('lam',[4,1],'real');             

% Variable Vectors for Equations
states = {q(1); q(2); q(3); q(4); q(5); q(6); q(7); dq(1); dq(2); dq(3); dq(4); dq(5); dq(6); dq(7); lam(1); lam(2); lam(3); lam(4)};

%% Transformation Maricies 
% Pivot point centered at [0,0]
% [Rotation, Translation
%      0    ...    1]

% Global Position Matrix
pos_vec = [q(1); 0; q(2)];
last_row = [0, 0, 0, 1];
% Body COM
T_com(:,:,1) = [eye(3), pos_vec; last_row];

% Joints - Kinematic Chain
% Leg 1 (Back Leg)
T_joint(:,:,1) = T_com(:,:,1)*TranslationMatrix(q(3),-l(1)/2);                      
T_joint(:,:,2) = T_joint(:,:,1)*TranslationMatrix(q(4),l(2));      
T_end_effector(:,:,1) = T_joint(:,:,2)*TranslationMatrix(q(5),l(3));      
% Leg 2 (Front Leg)
T_joint(:,:,3) = T_com(:,:,1)*TranslationMatrix(q(3),l(1)/2);                      
T_joint(:,:,4) = T_joint(:,:,3)*TranslationMatrix(q(6),l(4));      
T_end_effector(:,:,2) = T_joint(:,:,4)*TranslationMatrix(q(7),l(5));  

% Center of Mass
% Leg 1 (Back Leg)
T_com(:,:,2) = T_joint(:,:,1)*TranslationMatrix(q(4),l(2)/2); 
T_com(:,:,3) = T_joint(:,:,2)*TranslationMatrix(q(5),l(3)/2); 
% Leg 2 (Front Leg)
T_com(:,:,4) = T_joint(:,:,3)*TranslationMatrix(q(6),l(4)/2); 
T_com(:,:,5) = T_joint(:,:,4)*TranslationMatrix(q(7),l(5)/2); 

%% Cartesian Positions
% Joint Position
for i = 1:size(T_joint,3)
    j(:,i) = T_joint(1:3,end,i);       
end

% End Effector Positions
for i = 1:size(T_end_effector,3)
    e(:,i) = T_end_effector(1:3,end,i); 
end

% Center of Mass Equations
for i = 1:size(T_com,3)
    n(:,i) = T_com(1:3,end,i);   
end

% Rotational - about the y axis
% Body
w(:,1) = [0; q(3); 0];
% Leg 1
w(:,2) = [0; q(3) + q(4); 0];
w(:,3) = [0; q(3) + q(4) + q(5); 0];
% Leg 2
w(:,4) = [0; q(3) + q(6); 0];
w(:,5) = [0; q(3) + q(6) + q(7); 0];

% Body Kinematic Function
matlabFunction([j(:,1), j(:,3)],'File',[pwd '\AutoGen\GlobalBodyKinematicsAuto'],'Vars',{q});

% Joint Kinematic Function Development
matlabFunction(j,'File',[pwd '\AutoGen\GlobalKinematicsAuto'],'Vars',{q});

% End Effector Kinematics
matlabFunction(e,'File',[pwd '\AutoGen\GlobalEEKinematicsAuto'],'Vars',{q});

%% Cartesian Velocities
% Joint Translational
dj = simplify(fulldiff(j,states));

% End Effector Translational
de = simplify(fulldiff(e,states));

% COM Translational
dn = simplify(fulldiff(n,states));

% Rotational
dw = simplify(fulldiff(w,states));

%% Inertial Force Matrix - The Translational may be wrong
for i = 1:size(T_com,3)
    % Translational Inertia Matrix
    It = m(i).*eye(3);

    % Rotational Inertia Matrix
    Ir = [Ixx(i); Iyy(i); Izz(i)].*eye(3);

    % Assemble the complete Inertial Matrix in cartesian space
    Mc(:,:,i) = [It,     Z;
                 Z,     Ir];
end

%% Gravitational Force Vector - For each ridgid body
for i = 1:size(T_com,3)
    Gc(:,i) = m(i).* g .* [n(:,i); w(:,i)];
end

%% Energy Equations
for i = 1:size(T_com,3)
    % Kinetic Energy: T = 1/2 * [x; w]' * M * [x; w]
    T(:,i) = 1/2*[dn(:,i); dw(:,i)].'*Mc(:,:,i)*[dn(:,i); dw(:,i)];

    % Potential Energy: V = m * g * q 
    V(:,i) = Gc(:,i);
end
T_total = sum(T,'all');
V_total = sum(V,'all'); 

% Check the Energy Balance
% matlabFunction(T_total,V_total,'File',[pwd '\AutoGen\EnergyCheckAuto'],'Vars',{q,dq});

%% Lagrange Formuation
% Lagrangian: L = T - V
Lag = T_total - V_total; 

% L_partial/q_partial - d/dt * L_partial/dq_partial = 0
L_q_partial = simplify(jacobian(Lag,q).');
L_dq_partial = simplify(jacobian(Lag,dq).');
dt_L_dq_partial = simplify(fulldiff(L_dq_partial,states));

%% Double Stance Dynamics
% Lagrangian Holonomic Constraints
lambda = lam;

% End Effector Positions, Velocity, and Acceleration
h = [e(1,1); e(3,1); e(1,2); e(3,2)];         
dh = simplify(fulldiff(h,states));
ddh = simplify(fulldiff(dh,states));

% End Effector Jacobian & Constraint Force
H = jacobian(h,q);
Q_const = H.'*lambda;

% Non Conservative Forces
Q_nc = [0; 0; 0; u(:)];

% Dynamics Equations, each one equals zero
f = simplify(dt_L_dq_partial - L_q_partial - Q_const - Q_nc,3);      % f(q,dq,ddq,u,lam) = 0
g = ddh;                                                             % g(q,dq,ddq,u) = 0
dynamics = [f; g];

% Solve for ddq: 0 = M(q)*ddq + B*u + C(q,dq) + G(q)
M = simplify(jacobian(dynamics,[d2q; lambda]));
B = simplify(jacobian(dynamics,u));
GC = simplify(dynamics - M*[d2q; lambda] - B*u);        

% End Effector Jacobian
matlabFunction(H,'File',[pwd '\AutoGen\GlobalEEJacobianSSAuto'],'Vars',{q});

% Matrix Version of Dynamics
matlabFunction(M,B,GC,'File',[pwd '\AutoGen\DynamicsSSAuto'],'Vars',{[q; dq], u})

%% Back Leg Stance Dynamics
% Lagrangian Holonomic Constraints
lambda = lam(1:2);

% End Effector Positions, Velocity, and Acceleration
h = [e(1,1); e(3,1)];         
dh = simplify(fulldiff(h,states));
ddh = simplify(fulldiff(dh,states));

% End Effector Jacobian & Constraint Force
H = jacobian(h,q);
Q_const = H.'*lambda;

% Non Conservative Forces
Q_nc = [0; 0; 0; u(:)];

% Dynamics Equations - each function equals zero
f = simplify(dt_L_dq_partial - L_q_partial - Q_const - Q_nc,3);      % f(q,dq,ddq,u,lam) = 0
g = ddh;                                                                % g(q,dq,ddq,u) = 0
dynamics = [f; g];

% Solve for ddq: 0 = M(q)*ddq + B*u + C(q,dq) + G(q)
M = simplify(jacobian(dynamics,[d2q; lambda]));
B = simplify(jacobian(dynamics,u));
GC = simplify(dynamics - M*[d2q; lambda] - B*u);        

% End Effector Jacobian
matlabFunction(H,'File',[pwd '\AutoGen\GlobalEEJacobianSFAuto'],'Vars',{q});

% Matrix Version of Dynamics
matlabFunction(M,B,GC,'File',[pwd '\AutoGen\DynamicsSFAuto'],'Vars',{[q; dq], u})

%% Front Leg Stance Dynamics
% Lagrangian Holonomic Constraints
lambda = lam(3:4);

% End Effector Positions, Velocity, and Acceleration
h = [e(1,2); e(3,2)];         
dh = simplify(fulldiff(h,states));
ddh = simplify(fulldiff(dh,states));

% End Effector Jacobian & Constraint Force
H = jacobian(h,q);
Q_const = H.'*lambda;

% Non Conservative Forces
Q_nc = [0; 0; 0; u(:)];

% Dynamics Equations - each function equals zero
f = simplify(dt_L_dq_partial - L_q_partial - Q_const - Q_nc,3);      % f(q,dq,ddq,u,lam) = 0
g = ddh;                                                                % g(q,dq,ddq,u) = 0
dynamics = [f; g];

% Solve for ddq: 0 = M(q)*ddq + B*u + C(q,dq) + G(q)
M = simplify(jacobian(dynamics,[d2q; lambda]));
B = simplify(jacobian(dynamics,u));
GC = simplify(dynamics - M*[d2q; lambda] - B*u);        

% End Effector Jacobian
matlabFunction(H,'File',[pwd '\AutoGen\GlobalEEJacobianFSAuto'],'Vars',{q});

% Matrix Version of Dynamics
matlabFunction(M,B,GC,'File',[pwd '\AutoGen\DynamicsFSAuto'],'Vars',{[q; dq], u})

%% Flight Dynamics - No Constraints
% Non Conservative Forces
Q_nc = [0; 0; 0; u(:)];

% Dynamics Equations, each one equals zero
Energy = simplify(dt_L_dq_partial - L_q_partial - Q_nc,3);    % f(q,dq,ddq,u,lam)
dynamics = Energy;

% Solve for ddq: 0 = M(q)*ddq + B*u + C(q,dq) + G(q)
M = simplify(jacobian(dynamics,d2q));
B = simplify(jacobian(dynamics,u));
GC = simplify(dynamics - M*d2q - B*u);        

% Matrix Version of Dynamics
matlabFunction(M,B,GC,'File',[pwd '\AutoGen\DynamicsFFAuto'],'Vars',{[q; dq], u})

end
