function [COM_qT] = BodyPositionVectors(Nodes, qB_traj)
    % Robot Properties
    params = RobotProperties();
    pos_vec = [params.body.l/2; 0; 0];
        
    % Find the Future Position Ends
    for i = 1:Nodes
        % Body Center
        qB_cart(:,1,i) = [qB_traj(1,i); 0; qB_traj(2,i)];
        
        % Body Ends
        R = RotationMatrix3D(qB_traj(3,i));
        qB_cart_ends(:,:,i) = qB_cart(:,1,i) + [R*-pos_vec, R*pos_vec];
    end    
    
    % Vector from the COM to the edges
    COM_qT = qB_cart - qB_cart_ends;
end

