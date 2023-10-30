function [R] = RotationMatrix2D(theta)
    % 2D Rotation Matrix
    R = [cos(theta), -sin(theta);
         sin(theta),  cos(theta)];
end

