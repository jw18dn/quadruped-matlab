function [New_Gait_sequence] = GaitSequence(t_gait_seq, t_vec, Gait_Time, Gait_Matrix, Nodes)
    % Find the new gait sequence time vec
    t_vec_gait = t_vec + t_gait_seq;    
    
    % Extend the Gait Sequence to Beyond Time Hoizon Length
    Gait_Time_copy = Gait_Time;
    Gait_Matrix_copy = Gait_Matrix;
    
    while Gait_Time(end) <= t_vec_gait(end)
        Gait_Time = [Gait_Time(1:end-1); Gait_Time_copy + Gait_Time(end)];
        Gait_Matrix = [Gait_Matrix(1:end-1,:); Gait_Matrix_copy];
    end

    New_Gait_sequence = zeros(Nodes,2);
    i = 1;
    j = 1;    
    while i < Nodes
        if t_vec_gait(i) < Gait_Time(j)
            New_Gait_sequence(i,:) = Gait_Matrix(j,:);
            i=i+1;
        else
            j=j+1;
        end
    end
end

