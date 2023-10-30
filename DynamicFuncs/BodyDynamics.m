function [] = BodyDynamics(params)
    % Reassign params Values
    L = params.body.l;                      % Body Length
    M = params.body.m + 2*sum(params.leg.m);     % Mass 
    g = params.g(1:3);
    Ixx = params.body.Ixx;
    Iyy = params.body.Iyy;
    Izz = params.body.Izz;
    numbLegs = params.numbLegs;
      
    % Body Dependent Variables
    q = sym('q',[3,1],'real');              % [x y z]
    w = sym('w',[3,1],'real');              % [theta phi psi]
    dq = sym('dq',[3,1],'real');            % [dx dy dz] 
    dw = sym('dw',[3,1],'real');            % [dtheta dphi dpsi]    
    
    % vector from COM to Gound Reaction Force
    r = sym('r',[3,numbLegs],'real');
        
    % Ground Reaction Forces
    f = sym('f', [3,numbLegs], 'real');     % [xyz by numb legs]
    
    % Inertial Matrix - body frame
    I_b = [Ixx,  0,   0; 
            0,  Iyy,  0; 
            0,   0,  Izz];

    % Inertial Matrix - world frame
    R = RotationMatrix3D(w(2));
    I = R*I_b*R.';        
    
    % Linear Acceleration
    ddq = sum(f,2)./M - g;
    
    % Angular Acceleration
    for i = 1:numbLegs
        rf_cross(:,i) = cross(r(:,i),f(:,i));
    end
    ddw = I\sum(rf_cross,2);
        
    %% State Space Representation
    % Vectors
%     x = [q; w; dq; dw];
%     dx = [dq; dw; ddq; ddw];
%     u = f(:);

    % 2D Vectors
    x = [q(1); q(3); w(2); dq(1); dq(3); dw(2)];
    dx = [dq(1); dq(3); dw(2); ddq(1); ddq(3); ddw(2)];
    u = [f(1,1); f(3,1); f(1,2); f(3,2)];
    
    % State Space Matricies
    A = jacobian(dx,x);
    B = jacobian(dx,u);
    C = simplify(dx - (A*x + B*u));
    
    % Function Generation
    matlabFunction(A, B, C, 'File',[pwd '\AutoGen\GlobalLinearBodyDynamicsAuto'],'Vars',{r})
end

