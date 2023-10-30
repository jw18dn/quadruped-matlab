function [] = Animation(j, h1, fps, filename, t_vec, qB_ic, qL_ic, dqB_ic, dqL_ic, qB_traj, qT_ref, u,robot_params)
   % Plot Quadruped
   QuadPlot2(qB_ic, qL_ic, qB_traj, qT_ref, u,robot_params)

   % Write to temp file
   drawnow;
    
   % GIF:
   frame = getframe(h1);
   im = frame2im(frame);
   [imind,cm] = rgb2ind(im,256);
   if j == 1
       imwrite(imind,cm,filename,'gif','DelayTime',1/fps,'Loopcount',inf);
   elseif j == numel(j)
       imwrite(imind,cm,filename,'gif','DelayTime',1/fps,'WriteMode','append');
   else
       imwrite(imind,cm,filename,'gif','DelayTime',1/fps,'WriteMode','append');
   end  
end