% Copyright (c) 2023, Jason White
% 
% Redistribution and use in source and binary forms, with or without
% modification, are permitted provided that the following conditions are met:
% 
% 1. Redistributions of source code must retain the above copyright notice, this
%    list of conditions and the following disclaimer.
% 
% 2. Redistributions in binary form must reproduce the above copyright notice,
%    this list of conditions and the following disclaimer in the documentation
%    and/or other materials provided with the distribution.
% 
% 3. Neither the name of the copyright holder nor the names of its
%    contributors may be used to endorse or promote products derived from
%    this software without specific prior written permission.
% 
% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
% AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
% IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
% DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
% FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
% DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
% SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
% CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
% OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
% OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
%------------------------------------------------------------------------------


clear
clc
close all

% Notes:
%   Need to add force limit in optimization
%   Need to update the foot reference location after each step
%   Need to modify the desired foot location to acheive forward motion

%% User Defined Conditions - Column Vectors
% Robot State - [1, 2, 3, 4] - [SS, SF, FS, FF]
robot_state = 4;

% Body Initial Conditions
qB_ic = [0; 0.5; 0];                % [body x; body z; body angle]
dqB_ic = [0; 0; 0];
uB_ic = [0; 0; 0; 0];               % [Fx1; Fz1; Fx2; Fz2]

% Leg 1 Initial Conditions (Back Leg)
qL_ic(:,1) = [-3*pi/4; pi/2];       % [Upper angle 'theta1'; Lower Angle 'theta2']
dqL_ic(:,1) = [0; 0];
uL_ic(:,1) = [0; 0];                % [Torque 1; Torque 2]

% Leg 2 Initial Conditions (Front Leg)
qL_ic(:,2) = [-3*pi/4; pi/2];       % [Upper angle 'theta1'; Lower Angle 'theta2']
dqL_ic(:,2) = [0; 0];
uL_ic(:,2) = [0; 0];                % [Torque 1; Torque 2]

% Body Desired States
qB_des = [0; 0.49; 0];
dqB_des = [0; 0; 0];

% Time Parameters
Horizon_Time = 1;                   % Time Horizon for Optimization
Gait_Time = 0.3;                    % Clock Based Gait Time
DS_Percent = 0.1;                  % Double Stance Time (negetive if we want flight phase)

% Optimization Parameters
Nodes = 51;                         % Optimization Nodes
Control_Rate = 0.01;  
Sim_Time = 5;
Integration = 'Euler';        % Integration Scheme
% Trapezoidal fails because of the first node??

% Cost Function Weights 
Q = [10; 10000; 10000; 10000; 10000; 1000];    % [Positions; Velocities]
R = [.01; .01; .01; .01];                        % [L1 Forces; L2 Forces]

% Toe Operational Space Gaits
Kp = [20; 40];
Kd = [4; 6];

%% Calculated Values
dt = Horizon_Time/(Nodes-1);                    % Time intervals
t_vec = linspace(0,Horizon_Time,Nodes);         % Time Vector
numb_legs = size(qL_ic,2);                      % Number of legs
body_state_len = size([qB_ic; dqB_ic],1);       % Num of body position/velocity states          
body_input_len = size(uB_ic,1);                 % Num of body control inputs
pos_len = size([qB_ic; qL_ic(:)],1);            % Num of position states

%% Variable Initialization
% Simulation Parameters
iter = 0;
t_gait_seq = 0;
Exec_Time = 0;

qB_total = []; 
qT_ref_total = [];
qB_traj_total = [];
qL_total = [];
dqB_total = []; 
dqL_total = [];
lam_total = [];
u_total = [];
gait_total = [];

% ODE Options
options1 = odeset('events',@(t,y) Event1(t,y),'RelTol',1e-13,'AbsTol',1e-15);
options2 = odeset('events',@(t,y) Event2(t,y),'RelTol',1e-13,'AbsTol',1e-15);
options3 = odeset('events',@(t,y) Event3(t,y),'RelTol',1e-13,'AbsTol',1e-15);
options4 = odeset('events',@(t,y) Event4(t,y),'RelTol',1e-13,'AbsTol',1e-15);

% Initialize the trajectory & forces
qB_traj = [qB_ic; dqB_ic].*ones(body_state_len,Nodes);
qB_force = uB_ic.*ones(body_input_len,Nodes);
q_ic = [qB_ic; qL_ic(:,1); qL_ic(:,2)];
dq_ic = [dqB_ic; dqL_ic(:,1); dqL_ic(:,2)];

% Initialized the state space matricies
A = zeros(body_state_len, body_state_len, Nodes);
B = zeros(body_state_len, body_input_len, Nodes);     
f = zeros(body_state_len, Nodes);

%% Develop the Kinematics and Dynamic Functions
robot_params = RobotProperties_cheetah3();
% BodyDynamics(Robot_params);
% LegDynamics(Robot_params);
% FullSystemDynamics(Robot_params);

%% Gait Development
% Create the Gait Sequence: 0 - Contact, 1 - Flight
[Gait_Time_vec, Gait_Matrix] = GaitDefinition(Nodes, Gait_Time, DS_Percent);
Swing_Time = Gait_Time_vec(2) - Gait_Time_vec(1);
Gait_Seq_Prev = [0,0];
qTstance_ic_global = GlobalEEKinematicsAuto(q_ic);

%% Optimization Function Development
% Optimization Options
options = optimset('Display','off');

% Develop the Constraint Matrix Functions for Body Control
DynamicContinuity(dt, body_state_len, body_input_len, Integration);

% Develop the Objective Functions for Body Control
[H_body,c_body] = ObjectiveFunction(body_state_len,body_input_len,Nodes,qB_des,dqB_des,Q,R);

while Exec_Time < Sim_Time   
    % Update the Gait Sequence Along the Time Horizon
    
    if Exec_Time > Gait_Time*2
        Gait_Seq = GaitSequence(t_gait_seq, t_vec, Gait_Time_vec, Gait_Matrix, Nodes);
    else 
        Gait_Seq = zeros([Nodes,2]);    
    end
    Gait_Seq_Curr = Gait_Seq(1,:);

    % Vectors from COM to toe positions
    [COM_to_qT_vec, qTstance_ic_global] = ToePositionVectors(Nodes, Swing_Time, Gait_Time, Gait_Seq, dqB_des, q_ic, qTstance_ic_global, qB_traj);
    
    %% Body Control & Optimization
    % Linearize the Body Dynamics 
    for i = 1:Nodes
        [A(:,:,i),B(:,:,i),f(:,i)] = GlobalLinearBodyDynamicsAuto(COM_to_qT_vec(:,:,i)); 
    end
    
    % Assemble the Dynamic Continuity Constraints
    [Aeq_body_1,beq_body_1] = AssembleEqualityConstraint(Nodes,A,B,f,[qB_ic; dqB_ic]);
    
    % Constrain legs to produce no force when in flight
    [Aeq_body_2,beq_body_2] = FlightForceConstraints(body_state_len, Nodes, Gait_Seq);  

    % Find the Body Forces and Trajectory
    body_dv = quadprog(H_body,c_body,[],[],[Aeq_body_1;Aeq_body_2],[beq_body_1;beq_body_2],[],[],[],options);

    % Seperate the Results
    qB_traj = reshape(body_dv(1:Nodes*body_state_len),body_state_len,Nodes);          % Center of Mass Trajectory
    qB_force = reshape(body_dv(Nodes*body_state_len+1:end),body_input_len,Nodes);     % Forces on Body from Legs
    
    %% Swing Leg Desired States
    % Reset the desired pos, velocity, motor inputs, and lagrangian multipliers
    u = [NaN; NaN; NaN; NaN]; 
    qT_global_ref = [0; 0; 0];
    
    %% Individual Motor Control
    % Find the motor torque depending on the leg being in contact or swing
    % i = 1 means back leg?, i = 2 means front leg
    for i = 1:numb_legs
      % Reference the rows
      ref = (i-1)*2 + (1:2);
        
      % Swing Leg Control
        if Gait_Seq_Curr(i) == 1   
            % Find the Toe Position in the Local Frame
            if Gait_Seq_Curr(i) ~= Gait_Seq_Prev(i)
                qT_ic_local = LocalEEKinematicsAuto(qL_ic(:,i));
            end
            
            % Develop the Local End Effector Trajectory
            [qT_ref, dqT_ref, ddqT_ref] = EndEffectorTrajectory(robot_params,t_osc,qT_ic_local,qB_ic,dqB_des,Gait_Time_vec);

            % Global EE desired position - NOT ROTATED (still need to do this)
            qj = GlobalBodyKinematicsAuto(q_ic);
            qT_global_ref = qT_ref + qj(:,i);
            
            % Find the Control Inputs to Track the desired states
            u(ref) = LocalEEOSCAuto(qL_ic(:,i), dqL_ic(:,i), qT_ref([1,3],:), dqT_ref([1,3],:), ddqT_ref([1,3],:), Kp, Kd);
            if max(abs(u(ref))) > 20
                keyboard
            end
            
      % Stance Leg Control - add something for if we are in flight (push
      % down on the leg more)
        else                                                
            % Find the Jacobians  
            J = LocalEEJacobianAuto(qL_ic(:,i));
            
            % 2D Rotation Matrix
            R = RotationMatrix2D(qB_ic(3));
            
            % First Force from QP - change sign to apply to ground
            F = reshape(qB_force(:,1),2,2);
            
            % Modify force if leg is actually in flight but we expect stance
            if i == 1 && robot_state == 3
                F(:,i) = [0; F(2,i)/10];
            elseif i == 2 && robot_state == 2
                F(:,i) = [0; F(2,i)/10];                
            elseif robot_state == 4 
                F(:,i) = [0; F(2,i)/10];
            end
            
            % Translation to motor Torques
            u(ref) = -J.'*R.'*F(:,i);
        end
    end
    
    %% Simulation - we need full system dynamics with event functions to simulate
    % Different ode45 cases
    lam = 1e-13.*[1; 1; 1; 1];
    if robot_state == 1                 % Double Stance
        [t_sim, sim_results] = ode45(@(t,y) DynamicsSS(t, y, u), [0 Control_Rate], [q_ic; dq_ic; lam], options1);
    elseif robot_state == 2             % Back Leg Stance
        [t_sim, sim_results] = ode45(@(t,y) DynamicsSF(t, y, u), [0 Control_Rate], [q_ic; dq_ic; lam], options2);   
    elseif robot_state == 3             % Front Leg Stance
        [t_sim, sim_results] = ode45(@(t,y) DynamicsFS(t, y, u), [0 Control_Rate], [q_ic; dq_ic; lam], options3);   
    elseif robot_state == 4             % Flight
        [t_sim, sim_results] = ode45(@(t,y) DynamicsFF(t, y, u), [0 Control_Rate], [q_ic; dq_ic; lam], options4);   
    end
    
    if t_sim < Control_Rate 
        robot_state = DetermineGaitChange(robot_state, sim_results(end,:).');
        sim_results(end,:) = UpdateSimResults(sim_results(end,:), u, robot_state);
    end     
       
    
    %% Update the Initial Conditions
    Exec_Time = round(Exec_Time + Control_Rate,3,'decimals');    
    q_ic = sim_results(end,1:7).';              % [qB; qT_1; qT_2]
    dq_ic = sim_results(end,8:14).';            % [dqB; dqT_1; dqT_2]
    
    % Position States
    qB_ic = q_ic(1:3);
    qL_ic = [q_ic(4:5), q_ic(6:7)];
    
    % Velocity States
    dqB_ic = dq_ic(1:3);
    dqL_ic = [dq_ic(4:5), dq_ic(6:7)];
    
    % Gait Seq
    Gait_Seq_Prev = Gait_Seq_Curr;
    
    % Update the Gait Sequence Time & OSC time
    t_gait_seq = t_gait_seq + t_sim(end);
    if t_gait_seq > Gait_Time_vec(end)
        t_gait_seq = t_gait_seq - Gait_Time_vec(end);
    end
    if t_gait_seq > Gait_Time_vec(3) % This can be re-done to be more clean
        t_osc = t_gait_seq - Gait_Time_vec(3);
    else
        t_osc = t_gait_seq - Gait_Time_vec(1);
    end 
    
    
    % Save the Results
    iter = iter + 1;
    qB_total = [qB_total, qB_ic(:)];
    qB_traj_total(:,:,iter) = qB_traj;
    qT_ref_total = [qT_ref_total, qT_global_ref];
    qL_total = [qL_total, qL_ic(:)];
    dqB_total = [dqB_total, dqB_ic(:)];
    dqL_total = [dqL_total, dqL_ic(:)];    
    lam_total = [lam_total, lam(:)];    
    u_total = [u_total, u(:)];
    gait_total = [gait_total; Gait_Seq_Curr];
end

%% Plotting
% Figure Information  
h1 = figure(1); 
i = 1;
Anim_Time = 0;   
fps = 40;
filename = 'Anim.gif';
figure(h1)

while Anim_Time < Sim_Time
    Animation(i, h1, fps, [pwd '\Animation\Videos' filename], t_vec, qB_total(:,i), qL_total(:,i), dqB_total(:,i), dqL_total(:,i), qB_traj_total(:,:,i), qT_ref_total(:,i), u_total(:,i), robot_params)
    i = i+1;
    Anim_Time = round(Anim_Time + Control_Rate,3,'decimals');
end


%% Gait Determination & Results Update Functions
function robot_state = DetermineGaitChange(robot_state, results)
    % Parse Solution
    q = results(1:7);
    dq = results(8:14);
    lam = results(15:18);

    % Seperate the Positions and Velocities
    qee = GlobalEEKinematicsAuto(q);
    
    % Adjust qee values to be zero        
    if qee(3,1) < 1e-12
        qee(3,1) = 0;
    end
    if qee(3,2) < 1e-12
        qee(3,2) = 0;
    end
    
    % Determine New Phase
    if qee(3,1) <= 0 && qee(3,2) <= 0 && robot_state ~= 1     % Transition to Double Stance
        robot_state = 1;
    elseif qee(3,1) <= 0 && lam(4) <= 0 && robot_state ~= 2   % Transition to Back Leg Stance
        robot_state = 2;
    elseif qee(3,2) <= 0 && lam(2) <= 0 && robot_state ~= 3   % Transition to Front Leg Stance
        robot_state = 3;
    else                                                      % Transition to Flight
        robot_state = 4;
    end
end

function sim_results = UpdateSimResults(sim_results, u, robot_state)
    % Seperate the Positions and Velocities
    q = sim_results(end,1:7).';
    dq = sim_results(end,8:14).';
    
    % Update the Velocities & Robot State
    if robot_state == 1
        dq_new = ImpactVelocitySS(q,dq,u).'; 
    elseif robot_state == 2
        dq_new = ImpactVelocitySF(q,dq,u).';     
    elseif robot_state == 3
        dq_new = ImpactVelocityFS(q,dq,u).';
    else
        dq_new = dq.';
    end
    
    % Update the Velocities
    sim_results(8:14) = dq_new;
end

%% Dynamics Functions
function dy = DynamicsFF(t, y, u)   
    % Seperate Positions and Velocities
    q = y(1:7);
    dq = y(8:14);
    
    % Find the Acceleration Matricies
    [M,B,GC] = DynamicsFFAuto(y, u);                
    Acc = -M\(B*u + GC);
        
    % Seperate the Acceleration and Lagrangian Multipliers
    ddq = Acc(1:7);
    lam = [0; 0; 0; 0];
    
    % Combine to match ODE45 form
    dy = [dq; ddq; lam];
end

function dy = DynamicsSF(t, y, u)   
    % Seperate Positions and Velocities
    q = y(1:7);
    dq = y(8:14);

    % Find the Acceleration Matricies
    [M,B,GC] = DynamicsSFAuto(y, u);                
    Acc = -M\(B*u + GC);
        
    % Seperate the Acceleration and Lagrangian Multipliers
    ddq = Acc(1:7);
    lam = [Acc(8:9); 0; 0];
    
    % Combine to match ODE45 form
    dy = [dq; ddq; lam];
end

function dy = DynamicsFS(t, y, u)   
    % Seperate Positions and Velocities
    q = y(1:7);
    dq = y(8:14);

    % Find the Acceleration Matricies
    [M,B,GC] = DynamicsFSAuto(y, u);                
    Acc = -M\(B*u + GC);
    
    % Seperate the Acceleration and Lagrangian Multipliers
    ddq = Acc(1:7);
    lam = [0; 0; Acc(8:9)];
    
    % Combine to match ODE45 form
    dy = [dq; ddq; lam];
end

function dy = DynamicsSS(t, y, u)   
    % Seperate Positions and Velocities
    q = y(1:7);
    dq = y(8:14);
    
    % Find the Acceleration Matricies
    [M,B,GC] = DynamicsSSAuto(y, u);                
    Acc = -M\(B*u + GC);
        
    % Seperate the Acceleration and Lagrangian Multipliers
    ddq = Acc(1:7);
    lam = Acc(8:11);
    
    % Combine to match ODE45 form
    dy = [dq; ddq; lam];
end

%% Impulse Functions
function [dq_plus] = ImpactVelocitySS(q,dq,u)
    % Find the States Right Before Impact 
    M = DynamicsSSAuto([q; dq], u);
    Je = GlobalEEJacobianSSAuto(q);
    rows = size(Je,1);  cols = size(Je,2);

    % Find the Impulse and Velocity after Impact
    LHS = [M(1:cols,1:cols),   -Je(:,:,1)';
              Je(:,:,1),    zeros(rows,rows)];
    RHS = [M(1:cols,1:cols)*dq; zeros(rows,1)];
    temp = LHS\RHS;
    dq_plus = temp(1:cols);
end

function [dq_plus] = ImpactVelocitySF(q,dq,u)
    % Find the States Right Before Impact 
    M = DynamicsSFAuto([q; dq], u);
    Je = GlobalEEJacobianSFAuto(q);
    rows = size(Je,1);  cols = size(Je,2);

    % Find the Impulse and Velocity after Impact
    LHS = [M(1:cols,1:cols),   -Je(:,:,1)';
              Je(:,:,1),    zeros(rows,rows)];
    RHS = [M(1:cols,1:cols)*dq; zeros(rows,1)];
    temp = LHS\RHS;
    dq_plus = temp(1:cols);
end

function [dq_plus] = ImpactVelocityFS(q,dq,u)
    % Find the States Right Before Impact 
    M = DynamicsFSAuto([q; dq], u);
    Je = GlobalEEJacobianFSAuto(q);
    rows = size(Je,1);  cols = size(Je,2);

    % Find the Impulse and Velocity after Impact
    LHS = [M(1:cols,1:cols),   -Je(:,:,1)';
              Je(:,:,1),    zeros(rows,rows)];
    RHS = [M(1:cols,1:cols)*dq; zeros(rows,1)];
    temp = LHS\RHS;
    dq_plus = temp(1:cols);
end

%% Event Function
% need to sperate feet event functions

function [position,isterminal,direction] = Event1(t,y)
    % Both feet in stance     
    % End Effector Force
    lam = y(15:18);

    % Function
    position = [lam(2), lam(4)];                % The value that we want to be zero - Vertical Force
    isterminal = [1, 1];                        % Halt integration 
    direction = [-1, -1];                       % Value Decreasing
end

function [position,isterminal,direction] = Event2(t,y)
    % Back foot in stance, front in flight
    % End Effector Position
    q = y(1:7);
    q_ee = GlobalEEKinematicsAuto(q);
    
    % End Effector Force
    lam = y(15:18);

    % Function
    position = [lam(2), q_ee(3,2)];             % The value that we want to be zero
    isterminal = [1, 1];                        % Halt integration 
    direction = [-1, -1];                       
end

function [position,isterminal,direction] = Event3(t,y)
    % Front foot in stance, back in flight
    % End Effector Position
    q = y(1:7);
    q_ee = GlobalEEKinematicsAuto(q);
    
    % End Effector Force
    lam = y(15:18);

    % Function
    position = [lam(4), q_ee(3,1)];             % The value that we want to be zero
    isterminal = [1, 1];                        % Halt integration 
    direction = [-1, -1];                       
end

function [position,isterminal,direction] = Event4(t,y)
    % Both feet in flight before event
    % End Effector Position
    q = y(1:7);
    q_ee = GlobalEEKinematicsAuto(q);

    % Function
    position = [q_ee(3,1), q_ee(3,2)];          % The value that we want to be zero
    isterminal = [1, 1];                        % Halt integration 
    direction = [-1, -1];                       % Value Increasing or Decreasing
end
