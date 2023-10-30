function [] = QuadPlot(qB_ic, qL_ic, qB_traj, qT_ref,u,robot_params)
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
   j = GlobalKinematicsAuto([qB_ic; qL_ic(:)]);
   e = GlobalEEKinematicsAuto([qB_ic; qL_ic(:)]);
   
   % Find the Member Center Points
   c = [(j(:,1) + j(:,2))./2, (j(:,2) + e(:,1))./2, (j(:,3) + j(:,4))./2, (j(:,4) + e(:,2))./2];
   
   % Quadruped Body Plotting
   body_length = norm(j(:,1) - j(:,3));
   rectangle2([qB_ic(1), qB_ic(2), body_length, robot_params.body_height],'Rotation',rad2deg(qB_ic(3)),'Curvature',[0.2, 0.7],'EdgeColor','k','FaceColor',light_blue);
   
   % Leg Plotting
   % Back Leg
   rectangle2([c(1,1), c(3,1), robot_params.leg.l(1), robot_params.leg_height],'Curvature',[0.2,0.7],'Rotation',rad2deg(qB_ic(3)+qL_ic(1)),'FaceColor',light_blue)
   rectangle2([c(1,2), c(3,2), robot_params.leg.l(2), robot_params.leg_height],'Curvature',[0.2,0.7],'Rotation',rad2deg(qB_ic(3)+qL_ic(1)+qL_ic(2)),'FaceColor',light_blue)
   % Front Legs
   rectangle2([c(1,3), c(3,3), robot_params.leg.l(1), robot_params.leg_height],'Curvature',[0.2,0.7],'Rotation',rad2deg(qB_ic(3)+qL_ic(3)),'FaceColor',light_blue)       
   rectangle2([c(1,4), c(3,4), robot_params.leg.l(2), robot_params.leg_height],'Curvature',[0.2,0.7],'Rotation',rad2deg(qB_ic(3)+qL_ic(3)+qL_ic(4)),'FaceColor',light_blue)
     
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