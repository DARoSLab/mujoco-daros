function animate_biped_toe(tspan, z_trj, fig, opt_check)

color_list{1} = [0 0.4470 0.7410];
color_list{2} = [0.8500 0.3250 0.0980];
color_list{3} = [0.9290 0.6940 0.1250]; % yellow
color_list{4} = [0.4940 0.1840 0.5560];
color_list{5} = [0.4660 0.6740 0.1880];
color_list{6} = [0.3010 0.7450 0.9330];
color_list{7} = [0.6350 0.0780 0.1840];

  if nargin < 4
    opt_check = false;
  end
    figure(fig)
    plot([-1 5],[0 0],'k-','LineWidth',1); % ground
    hold on
    
    right_leg_color_idx = 2;
    left_leg_color_idx = 1;
    line_width = 3;
    len = size(z_trj,2);
    
    h_leg_left    = plot([0],[0],'-o',...
    'LineWidth', line_width,...
    'Color', color_list{left_leg_color_idx}, ...
    'MarkerEdgeColor',color_list{left_leg_color_idx},...
    'MarkerFaceColor',color_list{left_leg_color_idx},...
    'MarkerSize',2); 

    h_heel_left    = plot([0],[0],'-o',...
    'LineWidth', line_width,...
    'Color', color_list{left_leg_color_idx}, ...
    'MarkerEdgeColor',color_list{left_leg_color_idx},...
    'MarkerFaceColor',color_list{left_leg_color_idx},...
    'MarkerSize',5); 
    
    h_body    = plot([0],[0],'-',...
                'LineWidth',4,...
                'Color', color_list{4}, ...
                'MarkerEdgeColor','y',...
                'MarkerFaceColor','y',...
                'MarkerSize',6); 
    
            

    
    h_leg_right    = plot([0],[0],'-o',...
                'LineWidth', line_width,...
                'Color', color_list{right_leg_color_idx}, ...
                'MarkerEdgeColor',color_list{right_leg_color_idx},...
                'MarkerFaceColor',color_list{right_leg_color_idx},...
                'MarkerSize',2); 
    h_heel_right    = plot([0],[0],'-o',...
                'LineWidth', line_width,...
                'Color', color_list{right_leg_color_idx}, ...
                'MarkerEdgeColor',color_list{right_leg_color_idx},...
                'MarkerFaceColor',color_list{right_leg_color_idx},...
                'MarkerSize',5); 
    
    
            
    theta=linspace(0,2*pi);
    ellipse = [0.08*cos(theta); 0.36*sin(theta)];
%     circle = [0.05*cos(theta); 0.05*sin(theta)];
    

    if(opt_check)
        skip_step = 1;
        pause_time = 0.1;
    else
        skip_step = 30;
        pause_time = 0.00001;
    end
    
    video_recording = false;
    if(video_recording)
        v = VideoWriter('walking_test.avi');
        open(v);
    end
    
    
%     keypoints: [pelvis 
%         knee_r ankle_r(3) heel_r(4) toe_joint_r(5) toe_tip_r toe_back_r(7)
%         knee_l ankle_l(9) heel_l(10) toe_joint_l(11) toe_tip_l toe_back_l (13)
%         shoulder (14)]

    hip_idx = 1;
    shoulder_idx = 14;
    
    for k =1:skip_step:len        
        drawnow 
        zint = z_trj(:,k);     
        keypoints = keypoints_biped(zint);

        body_center = (keypoints(:,hip_idx) + keypoints(:,shoulder_idx))/2;
        th = zint(3);
        ellipse_body = [cos(th) -sin(th); sin(th), cos(th)] * ellipse + body_center(1:2);
        h_body.XData = ellipse_body(1,:);
        h_body.YData = ellipse_body(2,:);

        % Right leg
        draw_lines([keypoints(:,hip_idx:3) keypoints(:,5:7) keypoints(:,5)], h_leg_right);
        draw_lines(keypoints(:,3:4), h_heel_right);

        % Left leg
        draw_lines([keypoints(:,hip_idx) keypoints(:,8:9) keypoints(:, 11:13) keypoints(:,11) ], h_leg_left);
        draw_lines(keypoints(:,9:10), h_heel_left);
    

            
        drawnow
        sim_time = sprintf('%f (sec)', tspan(k));

        title(sim_time)

        axis equal % sets the X:Y aspect ratio 1:1; otherwise things will look stretched
        axis([-0.3 3.7 -0.1 1.6])
%         axis([-0.3 0.7 -0.1 0.5])
        
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

