function [Time_Vec, Gait_Matrix] = GaitDefinition(Nodes, GaitTime, DoubleStancePercent)
    % Develops Clock based gait sequence (0 for contact, 1 for flight)
    
    % Always start with both legs on the ground....
    
    % Develop the Gait Sequence for the given Time Horizon
    if  DoubleStancePercent > 0                            % There is a double stance phase   
        DS_Time = GaitTime*DoubleStancePercent;         % Double Stance Time
        SS_Time = GaitTime - DS_Time;
        Gait_Matrix = [0, 0;                            % Contact Sequence never changes, only the times
                       0, 1;
                       0, 0;
                       1, 0;
                       0, 0];
        Time_Vec = [1/4*DS_Time, 1/4*DS_Time + 1/2*SS_Time, 3/4*DS_Time + 1/2*SS_Time, 3/4*DS_Time + SS_Time, DS_Time + SS_Time].';
        
    elseif DoubleStancePercent < 0                         % There is no double stance phase
        NC_Time = -GaitTime*DoubleStancePercent;
        SS_Time = GaitTime - NC_Time;
        Gait_Matrix = [1, 0;                            % Contact Sequence never changes, only the times
                       1, 1;
                       0, 1;
                       1, 1;
                       1, 0];
        Time_Vec = [1/4*SS_Time, 1/4*SS_Time + 1/2*NC_Time, 3/4*SS_Time + 1/2*NC_Time, 3/4*SS_Time + NC_Time, SS_Time + NC_Time].';
        
    else                                                % Theres an instantenous double stance phase
        Gait_Matrix = [1, 0;
                       0, 1;
                       1, 0];
        Time_Vec = [1/4*GaitTime, 3/4*GaitTime, GaitTime].';
    end
    
    % Create a Function to sub values into 
    u = sym('u',[4,Nodes],'real');
    Mat = eye(4*Nodes).*u(:);
    matlabFunction(Mat,'File',[pwd '\AutoGen\GaitInputConstraintsAuto'],'Vars',{u});    
end



