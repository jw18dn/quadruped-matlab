function [Aeq,beq] = AssembleEqualityConstraint(Nodes,A,B,f,qic)
    % A & B depth is the number of nodes
    % Takes the state space matrix for one node in and produces part of the
    % large equality constraint matrix

    % Matrix Sizes
    A_width = size(A,2);
    B_width = size(B,2);

    % Preallocate the matrix as sparse - could use spalloc?
    Aeq_x = zeros(A_width*Nodes,A_width*Nodes);   
    Aeq_u = zeros(A_width*Nodes,B_width*Nodes);
    beq = zeros(A_width*Nodes,1);

    % Reference Location
    rows = A_width+1:A_width*2; 
    x_cols = 1:A_width*2;
    u_cols = 1:B_width*2;

    for i = 1:Nodes-1      
        % Tie the current state to the first node
        if i == 1
            % Develop Initial Condition Constraint
            [Aeq_ic,beq_ic] = InitialConditionMatrixAuto(qic(:,1));

            % Replace First Block with only the Right Half of the Matrix Developed
            Aeq_x(1:A_width,1:A_width) = Aeq_ic(1:A_width,1:A_width);
            Aeq_u(1:A_width,1:B_width) = Aeq_ic(1:A_width,A_width+1:end);    
            beq(1:A_width,1) = beq_ic;
        end

        % Tie the preceding nodes to the system dynamics
        [Aeq_temp,beq_temp] = ContDynConstMatrixAuto(A(:,:,i:i+1),B(:,:,i:i+1),f(:,i:i+1));

        % Replace the Preallocated "Zero" Matrix Values
        Aeq_x(rows,x_cols) = Aeq_temp(1:A_width,1:A_width*2);
        Aeq_u(rows,u_cols) = Aeq_temp(1:A_width,A_width*2+1:end); 
        beq(rows,1) = beq_temp;

        % Find the new ref location
        rows = rows + A_width; 
        x_cols = x_cols + A_width;
        u_cols = u_cols + B_width;
    end
    
    % Stack the two equality constraint matricies
    Aeq = [Aeq_x, Aeq_u];
end

