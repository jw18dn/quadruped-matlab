function [] = DynamicContinuity(dt, A_len, B_len, integration)
% Makes the functions necessary for developing the overall equality
% constraint matrix. We can further modify this to remove the velocity
% equlaity constraints.

% State Space Symbolic Matricies
A = sym('A',[A_len,A_len,2],'real');  
B = sym('B',[A_len,B_len,2],'real'); 
f = sym('f',[A_len,2],'real'); 

% System State Symbolic Vectors
x = sym('x',[A_len,2],'real');  
u = sym('u',[B_len,2],'real');
dv_ic = [x(:,1); u(:,1)];
dv = [x(:); u(:)];

% State Symbolic vectors
xic = sym('qic',[A_len,1],'real'); 

% State Space Representation
dx(:,1) = A(:,:,1)*x(:,1) + B(:,:,1)*u(:,1) + f(:,1);
dx(:,2) = A(:,:,2)*x(:,2) + B(:,:,2)*u(:,2) + f(:,2);  

% Integration Style
if strcmp(integration,'Euler')
    ceq_defect = x(:,2) - x(:,1) - dx(:,1).*dt;
elseif strcmp(integration,'Trapezoidal')
    ceq_defect = x(:,2) - x(:,1) - (dx(:,1) + dx(:,2)).*dt./2;
elseif strcmp(integration,'Simpson')
    disp('Need Additional Design Variables')
    keyboard
else
    disp('Invalid Integration Choice')
    keyboard
end

% Initial Condition Continuity
ceq_initial = x(:,1) - xic;
Aeq_ic = jacobian(ceq_initial,dv_ic);
beq_ic = -subs(ceq_initial,dv_ic,[zeros(A_len,1); zeros(B_len,1)]);

% Dynamic Continuity - Take the Jacobian
Aeq = jacobian(ceq_defect,dv);
beq = -subs(ceq_defect,dv,[zeros(A_len*2,1); zeros(B_len*2,1)]);

% Develop both Functions
matlabFunction(Aeq_ic,beq_ic,'File',[pwd '\AutoGen\InitialConditionMatrixAuto'],'Outputs',{'Aeq_ic','beq_ic'},'Vars',{xic}); 
matlabFunction(Aeq,beq,'File',[pwd '\AutoGen\ContDynConstMatrixAuto'],'Outputs',{'Aeq','beq'},'Var',{A, B, f}); 
end

