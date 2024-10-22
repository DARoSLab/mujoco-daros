function animate_SingleLeg(tspan, z_trj, fig)
color_list{1} = [0 0.4470 0.7410];
color_list{2} = [0.8500 0.3250 0.0980];
color_list{3} = [0.9290 0.6940 0.1250]; % yellow
color_list{4} = [0.4940 0.1840 0.5560];
color_list{5} = [0.4660 0.6740 0.1880];
color_list{6} = [0.3010 0.7450 0.9330];
color_list{7} = [0.6350 0.0780 0.1840];

    figure(fig)
     plot([-2 2.5],[0 0],'k-','LineWidth',3); % ground

    hold on
    
    left_leg_color_idx = 2;
    line_width = 3;
    len = size(z_trj,2);
        
    h_body    = plot([0],[0],'-',...
                'LineWidth',4,...
                'Color', color_list{4}, ...
                'MarkerEdgeColor','y',...
                'MarkerFaceColor','y',...
                'MarkerSize',6); 
    
    
    h_leg    = plot([0],[0],'-o',...
    'LineWidth', line_width,...
    'Color', color_list{left_leg_color_idx}, ...
    'MarkerEdgeColor',color_list{left_leg_color_idx},...
    'MarkerFaceColor',color_list{left_leg_color_idx},...
    'MarkerSize',2); 


   
            
    theta=linspace(0,2*pi);
    circle = [0.11*cos(theta); 0.11*sin(theta)];
    
    video_recording = false;
    if(video_recording)
        v = VideoWriter('Running.avi');
        open(v);
    end
    
    for k =1:len        
        drawnow 
        zint = z_trj(:,k);     
        keypoints = keypoints_biped(zint);
        
        circle_body = circle + keypoints(:,1);
        h_body.XData = circle_body(1,:);
        h_body.YData = circle_body(2,:);

      
        % Leg
        draw_lines([keypoints(:,1:end), keypoints(:,3)], h_leg);
%         draw_lines([keypoints(:,end), ], h_leg);
            
        drawnow
        sim_time = sprintf('%f (sec)', tspan(k));

        title(sim_time)

        axis equal % sets the X:Y aspect ratio 1:1; otherwise things will look stretched
        axis([-1.0 1.5 -0.1 1.1])
        
        if(video_recording)
            frame = getframe(gcf);
            writeVideo(v,frame);
        end
        
        pause(0.1);
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

