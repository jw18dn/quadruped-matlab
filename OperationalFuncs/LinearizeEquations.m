function [] = LinearizeEquations(q,u,ddq)
% Inputs - symbolic vector (state variables), symbolic vector (control
% inputs), symbolic velocity and accelerations variables/equations)

% Symbolic Variable Development
qo = sym('qo',size(q),'real');
uo = sym('uo',size(u),'real');

% Sub in Operating Point Variables
dqo = subs(ddq, [q; u], [qo; uo]); 

% Find the Operating Point Gradient
Jqo = jacobian(dqo,qo);
Juo = jacobian(dqo,uo);

% Develop the Linear Dynamics using 1st order Taylor series
dq_linear = dqo + Jqo * (q - qo) + Juo * (u - uo); 

% State Space Matricies
A = jacobian(dq_linear,q);
B = jacobian(dq_linear,u);

% Group Additional terms - remove the state variable & control input terms
f = simplify(dq_linear - (A*q + B*u));

% Function Generation
matlabFunction(A, B, f, 'File',[pwd '\AutoGen\LinearizedDynamicsAuto'],'Vars',{qo,uo});
end

