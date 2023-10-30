function [COM_qT, qTstance_ic] = ToePositionVectors(Nodes, Swing_Time, Gait_Time, Gait_Seq, dqB_des, q_ic, qTstance_ic, qB_traj)
    % Future foot positions
    dist = dqB_des./(Swing_Time/Gait_Time);
    
    % Find the last double stance positions
    if Gait_Seq(1,1) == 0 && Gait_Seq(1,2) == 0
        qTstance_ic = GlobalEEKinematicsAuto(q_ic);
    elseif Gait_Seq(1,1) == 0
        temp = GlobalEEKinematicsAuto(q_ic);
        qTstance_ic = [temp(:,1), qTstance_ic(:,2)];
    elseif Gait_Seq(1,2) == 0
        temp = GlobalEEKinematicsAuto(q_ic);
        qTstance_ic = [qTstance_ic(:,1), temp(:,2)];
    else
        keyboard
    end
        
    for i = 1:Nodes
        qB_cartesian(:,1,i) = [qB_traj(1,i); 0; qB_traj(2,i)];
        if i == 1
            qT_future(:,:,i) = qTstance_ic;
        elseif Gait_Seq(i,1) == 0 && Gait_Seq(i-1,1) == 1
            qT_future(:,:,i) = [dist + qT_future(:,1,i-1), qT_future(:,2,i-1)];
        elseif Gait_Seq(i,2) == 0 && Gait_Seq(i-1,2) == 1
            qT_future(:,:,i) = [qT_future(:,1,i-1), dist + qT_future(:,2,i-1)];
        else
            qT_future(:,:,i) = qT_future(:,:,i-1);
        end
    end        
    COM_qT = qB_cartesian - qT_future;
end

