function [Aeq_f,beq_f] = FlightForceConstraints(state_len, Nodes, New_Gait_sequence)
    % Use the Previously Developed funciton
    u = [New_Gait_sequence(:,1).'; New_Gait_sequence(:,1).'; New_Gait_sequence(:,2).'; New_Gait_sequence(:,2).'];
    Aeq_f1 = GaitInputConstraintsAuto(u);
    
    % Stack the Matricies
    Aeq_f2 = zeros(size(Aeq_f1,1),state_len*Nodes);
    Aeq_f = [Aeq_f2, Aeq_f1];
    beq_f = zeros(size(Aeq_f,1),1);
end

