function [T] = TranslationMatrix(q,l)
% q is the angle
% l is the length

    T = [cos(q)    0     -sin(q) l*cos(q(1));
           0       1        0        0
         sin(q)    0      cos(q) l*sin(q(1));
           0       0        0        1];   
end

