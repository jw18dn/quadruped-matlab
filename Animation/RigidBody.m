function [] = RigidBody(radius,length,rotation,translation,color)
    % Nodes along helf circle
    N = 1000;
    
    % half circle
    th = linspace(-pi/2,pi/2,N);

    % Plot points
    x_points = [length+radius*cos(th) 0-radius*(cos(th(end:-1:1)))];
    y_points = [radius*sin(th) radius*sin(th(end:-1:1))];

    % Rotation Matrix
    ROTMAT = [cos(rotation) -sin(rotation); sin(rotation) cos(rotation)];

    % Points
    p = translation + ROTMAT*[x_points; y_points];

    fill(p(1,:), p(2,:),color)
end

