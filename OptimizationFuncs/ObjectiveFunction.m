function [H,f] = ObjectiveFunction(A_len,B_len,Nodes,q_des,dq_des,Q,R)
     % System States  
     states = sym('q',[A_len Nodes],'real');                 % System State Matrix
     inputs = sym('u',[B_len Nodes],'real');                 % Control Input Matrix  

    %% Objective Function Forumlation
    % Design Vector and matrix sizing
    dv = [states(:); inputs(:)];

    % Find the error (x_d is a vector, x is a matrix)
    q_e = [q_des; dq_des] - states;

    % We need to grab the diagonal
    J = sum(Q.*q_e.^2,'all') + sum(R.*inputs.^2,'all');

    % Find the Matrix components  
    H = double(hessian(J,dv));
    f_obj = jacobian(J,dv);
    f = double(subs(f_obj,dv,zeros(size(dv))));

end