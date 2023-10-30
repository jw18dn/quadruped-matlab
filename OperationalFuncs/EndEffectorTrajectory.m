function [ee_ref,dee_ref,ddee_ref] = EndEffectorTrajectory(params,t,qT_ic,qB_ic,dqB_des,Gait_t_vec)
    %% Assumes there is always a double stance time programed in
    Flight_Time = Gait_t_vec(2) - Gait_t_vec(1);
    Stance_Time = Gait_t_vec(5) - Flight_Time;

    % Parameters
    radius = params.step_height;
    Gait_Time = Stance_Time + Flight_Time;

    % Reference Trajectory
    dist = dqB_des(1)/(Flight_Time/Gait_Time);
    xT_ref = qT_ic(1) + dist*(t/Flight_Time);
    xT_ref = dist*(t/Flight_Time);
    dxT_ref = dist/Flight_Time;
    ddxT_ref = 0;
    
    psi = pi*t/Flight_Time;    
    zT_ref = qT_ic(3) + radius*sin(psi);
    dzT_ref = (pi/Flight_Time)*radius*cos(psi);
    ddzT_ref = -(pi^2/Flight_Time^2)*radius*sin(psi);
    
    % Combined
    ee_ref = [xT_ref; 0; zT_ref];
    dee_ref = [dxT_ref; 0; dzT_ref];
    ddee_ref = [ddxT_ref; 0; ddzT_ref];
end

