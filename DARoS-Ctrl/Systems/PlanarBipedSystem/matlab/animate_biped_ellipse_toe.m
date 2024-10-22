function animate_biped_ellipse_toe(tspan, z_trj, fig)
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
    ellipse = [0.06*cos(theta); 0.28*sin(theta)];
%     circle = [0.05*cos(theta); 0.05*sin(theta)];
    
    for k =1:10:len-1        
        drawnow 
        zint = z_trj(:,k);     
        keypoints = keypoints_biped_toe(zint);

        body_center = (keypoints(:,1) + keypoints(:,12))/2;
        th = zint(3);
        ellipse_body = [cos(th) -sin(th); sin(th), cos(th)] * ellipse + body_center(1:2);
        h_body.XData = ellipse_body(1,:);
        h_body.YData = ellipse_body(2,:);

      
        % Left leg
        draw_lines([keypoints(:,1) keypoints(:,7:11)], h_leg_left);
        
        % Right leg
        draw_lines(keypoints(:,1:6), h_leg_right);
            
        drawnow
        sim_time = sprintf('%f (sec)', tspan(k));

        title(sim_time)

        axis equal % sets the X:Y aspect ratio 1:1; otherwise things will look stretched
        axis([-0.3 1.7 -0.1 1.4])
        pause(0.00001);
    end
%     draw_lines(z(:,end),kinematics,h_leg);
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

