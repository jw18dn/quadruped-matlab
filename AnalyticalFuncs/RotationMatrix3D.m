function [R] = RotationMatrix3D(theta)
    % 2D Rotation Matrix
    R = [cos(theta), 0, -sin(theta);
            0,       1,     0;
         sin(theta), 0, cos(theta)];
end

