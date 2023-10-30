function [params] = RobotProperties_cheetah3()
% General Physics Parameters
params.g = [0; 0; 9.81; 0; 0; 0]; 
params.numbLegs = 2;

% Body Parameters - Inependent Variables
params.body.l = 0.6;            % length - meters
params.body.w = 0.15;            % width - meters
params.body.m = 45;             % kg
params.body.Ixx = 0.41;         % kg*m^2 - 0.35
params.body.Iyy = 2.1;          % kg*m^2
params.body.Izz = 2.1;          % kg*m^2 - 2.1

% Leg Parameters - Independent Variables (same for all 4 legs)
params.leg.l = [0.34; 0.34];
params.leg.w = [0.06; 0.06];           
params.leg.m = [0.68; 0.68];
params.leg.Ixx = [0; 0];
params.leg.Iyy = [1/12*params.leg.m(1)*params.leg.l(1)^2; 1/12*params.leg.m(2)*params.leg.l(2)^2];
params.leg.Izz = [0; 0];

% Other Parameters
params.step_height = 0.1;
end

