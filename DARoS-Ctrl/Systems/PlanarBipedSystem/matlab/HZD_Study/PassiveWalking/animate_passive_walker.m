function animate_passive_walker(tspan, z_trj, params, stance_pos, fig)
    figure(fig)
    plot([-1 3],[0 0],'k-','LineWidth',3); % ground
    hold on
    
    len = size(z_trj,2);
    h_leg_right    = plot([0],[0],'r-o',...
                'LineWidth',3,...
                'MarkerEdgeColor','r',...
                'MarkerFaceColor','r',...
                'MarkerSize',6); 
            
    h_leg_left    = plot([0],[0],'g-o',...
    'LineWidth',3,...
    'MarkerEdgeColor','g',...
    'MarkerFaceColor','g',...
    'MarkerSize',6); 
    
    h_body    = plot([0],[0],'-',...
                'LineWidth',3,...
                'MarkerEdgeColor','y',...
                'MarkerFaceColor','y',...
                'MarkerSize',6); 
    
            
    theta=linspace(0,2*pi);
%     ellipse = [0.06*cos(theta); 0.28*sin(theta)];
    circle = [0.05*cos(theta); 0.05*sin(theta)];
    
    hip_idx = 1;
    
    
%     if(opt_check)
%         skip_step = 1;
%         pause_time = 0.1;
%     else
%         skip_step = 20;
%         pause_time = 0.00001;
%     end

    skip_step = 20;
    pause_time = 0.00001;
    video_recording = false;
    if(video_recording)
        v = VideoWriter('HZD_1.avi');
        open(v);
    end
    
    for k =1:skip_step:len        
        drawnow 
        zint = z_trj(:,k);     
        keypoints = keypoints_passive(zint, params);

        circle_body = circle + keypoints(:,hip_idx) + [stance_pos(k);0];
        h_body.XData = circle_body(1,:);
        h_body.YData = circle_body(2,:);

        
        stance_foot_offset = [stance_pos(k);0];
        % Stance leg
        draw_lines([ (keypoints(:,hip_idx)+stance_foot_offset) stance_foot_offset ], h_leg_left);
        
        % Right leg
        draw_lines([(keypoints(:,hip_idx)+stance_foot_offset) (keypoints(:,2)+stance_foot_offset)], h_leg_right);
            
        drawnow
        sim_time = sprintf('%f (sec)', tspan(k));

        title(sim_time)

        axis equal % sets the X:Y aspect ratio 1:1; otherwise things will look stretched
        axis([-0.3 1.2 -0.1 1.4])
        
        if(video_recording)
            frame = getframe(gcf);
            writeVideo(v,frame);
        end
        
        pause(pause_time);
    end
%     draw_lines(z(:,end),kinematics,h_leg);
    if(video_recording)
        close(v);
    end
end

function draw_lines(leg_keypoints,h_leg)
    h_leg.XData = leg_keypoints(1,:);
    h_leg.YData = leg_keypoints(2,:);
    drawnow
end

function draw_leg(hip, foot, h_leg, l_thigh, l_shank)
    d = norm(hip - foot);
    foot_wrt_hip = foot - hip;
    th1 = atan(foot_wrt_hip(1)/-foot_wrt_hip(2));
    th2 = acos( (l_thigh^2 + d^2 - l_shank^2)/(2*l_thigh*d) );
    
    knee = [hip(1) + l_thigh*sin(th1 + th2); hip(2) - l_thigh*cos(th1 + th2)];
    
    h_leg.XData = [hip(1), knee(1), foot(1)];
    h_leg.YData = [hip(2), knee(2), foot(2)];
end

