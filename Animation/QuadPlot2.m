function [] = QuadPlot2(qB_ic, qL_ic, qB_traj, qT_ref, u, robot_params)
   % Plot Parameters
   subplot(2,1,1)
   cla()
   window = robot_params.body.l + 1;
   xlim([-window window]);
   axis equal
   hold on
   set(gcf,'color','white')
   set(gca,'box','off','ycolor','w')
   set(gca,'box','off','xcolor','w')
   set(gca,'fontname','georgia','fontangle','italic') 
   title("Animation",'Interpreter','latex')  
   
   % Color Conversion
   light_blue = sscanf('aed0ec','%2x%2x%2x',[1 3])/255;
   dark_blue = sscanf('5693c4','%2x%2x%2x',[1 3])/255;
   grey = sscanf('d0d0d0','%2x%2x%2x',[1 3])/255;
   
   % Find the Joint Positions
   j = GlobalKinematicsAuto([qB_ic; qL_ic(:)]);     % Joints
   e = GlobalEEKinematicsAuto([qB_ic; qL_ic(:)]);   % End Effectors
   
   % Plotting the system
   % Body
   RigidBody(robot_params.body.w/2,robot_params.body.l,qB_ic(3),[j(1,1); j(3,1)],light_blue)
   % Legs
   RigidBody(robot_params.leg.w(1)/2,robot_params.leg.l(1),qL_ic(1),[j(1,1); j(3,1)],light_blue)
   RigidBody(robot_params.leg.w(2)/2,robot_params.leg.l(2)-robot_params.leg.w(2)/2,qL_ic(1) + qL_ic(2),[j(1,2); j(3,2)],light_blue)
   RigidBody(robot_params.leg.w(1)/2,robot_params.leg.l(1),qL_ic(3),[j(1,3); j(3,3)],light_blue)
   RigidBody(robot_params.leg.w(2)/2,robot_params.leg.l(2)-robot_params.leg.w(2)/2,qL_ic(3) + qL_ic(4),[j(1,4); j(3,4)],light_blue)
   
   % Ground Plot
   line([-3 3],[0 0],'LineWidth',1,'Color','k')                                        % Ground
   
   % Desired Trajectory and Leg Positions
   if nargin > 2
        plot(qB_traj(1,:), qB_traj(2,:), 'k--')
   end
   
  % Toe Reference Plotting
   if qT_ref(3) ~= 0
        plot(qT_ref(1), qT_ref(3), 'k*')
   end
   
   % Motor Torque Plots
   subplot(2,1,2)
   cla()
   hold on
   xlim([0,150]);
   set(gca,'fontname','georgia','fontangle','italic') 
   xlabel("$\tau (Nm)$",'Interpreter','latex')   
   set(gca,'box','off','ycolor','w')
%    xticks([0, 10, 20, 30, 40, 50]);
   xticks([0, 50, 100, 150]);
   title("Motor Torques (100Hz)",'Interpreter','latex')
   barh(abs(u).','EdgeColor',light_blue,'FaceColor',light_blue)
end