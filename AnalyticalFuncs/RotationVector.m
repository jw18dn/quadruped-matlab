function [q_star] = RotationVector(position,translation,rotation)
    % Position
    q = position; % Global Position
    T = translation; % Local Frame Translation
    theta = rotation; % Local Rotation

    % 2D Rotation Matrix
    R = [cos(theta), -sin(theta);
         sin(theta),  cos(theta)];

    % New Position
    q_star = q + R*T;
end

