classdef controller_HZD < handle
properties
    
%     alpha =[
%     0.2311    0.1519    0.1662   -0.0708    0.1591   -0.2721    0.0003   -0.1852   -0.0889
%     0.0001    0.0312   -0.1506    0.2633   -0.3961    0.3849   -0.2632    0.1332   -0.0763
%    -0.0005    0.2071   -1.2895    1.4780   -1.7255    1.1838   -0.4762   -0.0049   -0.1183
%    -0.0000   -0.6192    1.4427   -1.8189    1.9595   -1.3755    0.6822    0.0418    0.3043
%    -0.0889   -0.1452   -0.3395    0.5456   -0.4148    0.9417   -0.0240    0.3555    0.2322
%    -0.0763   -0.0977   -0.5191    0.0413   -1.2444    0.2563   -0.4255    0.0489    0.0004
%    -0.1183    0.5377   -0.3078    0.6858   -0.0032   -0.6502    0.5269   -0.4462   -0.0010
%     0.3043    0.6722    0.2209    0.7660    0.3819    0.1518    0.2745    0.0969   -0.0000 ];
%    stride_time = 0.33;

   alpha1 = [    0.2311    0.1958    0.3677   -0.4150    0.7327   -0.6455    0.1117   -0.2076   -0.1066
    0.0001    0.0260   -0.1610    0.3200   -0.4974    0.5223   -0.3628    0.1537   -0.0696
   -0.0005    0.1783   -1.8413    3.0163   -4.1990    3.7379   -2.1319    0.3700   -0.1486
    0.0000   -0.6227    1.9935   -3.3555    4.3760   -3.9144    2.3360   -0.3336    0.3380
   -0.1066   -0.1417   -1.0869    2.4376   -2.9470    3.1578   -1.0135    0.5722    0.2322
   -0.0696   -0.1225   -1.0530    1.5499   -3.2965    1.9368   -1.1884    0.2536    0.0004
   -0.1486    0.4836   -0.8758    2.0931   -1.7561    0.7030   -0.0929   -0.2340   -0.0010
    0.3380    0.6555    0.0980    1.1439   -0.0220    0.4684    0.1504    0.1245    0.0000];

% v = 0.7 m/s
   alpha2 = [    0.2311    0.1095    0.2158   -0.1858    0.6999   -0.5314    0.4555   -0.1853   -0.1067
    0.0001    0.0107   -0.0433    0.0867   -0.1461    0.2287   -0.2921    0.2130   -0.1277
   -0.0005    0.3251   -2.3579    3.9533   -5.5388    4.9909   -2.9726    0.6090   -0.2211
    0.0000   -0.7521    2.4292   -4.0995    5.3864   -4.8805    3.0208   -0.5777    0.4607
   -0.1067   -0.1264   -0.1945    0.3561    0.2162    0.5524    0.3117    0.3910    0.2322
   -0.1277   -0.2606   -0.1309   -0.8401    0.1632   -1.0573    0.2331   -0.0327    0.0004
   -0.2211    0.4110    0.0950   -0.3770    1.1642   -1.5389    0.8745   -0.6125   -0.0010
    0.4607    0.8276    0.4913    0.5995    0.7028    0.2559    0.2671    0.2004   -0.0000];
%    stride_time = 0.495;
      
    alpha3 = [ 0.1708    0.0589    0.1884   -0.1706    0.4820   -0.3581    0.2834   -0.1141    0.0074
    0.0000    0.0785   -0.3489    0.7043   -0.9911    0.9705   -0.6695    0.3188   -0.1398
   -0.0007    0.4105   -2.0852    3.3763   -4.5324    3.8793   -2.0497   -0.0040   -0.3185
   -0.0000   -0.7371    2.2935   -3.8443    4.9236   -4.2884    2.3553   -0.1152    0.4689
    0.0074    0.1052   -0.1661    0.7844   -0.4093    1.1818   -0.0940    0.4386    0.1716
   -0.1398   -0.5031   -0.4008   -0.7533    0.0594   -1.0646    0.0937   -0.2386    0.0000
   -0.3185    0.6625   -0.4640    0.4404   -0.0216   -0.4261    0.3136   -0.5075   -0.0014
    0.4689    1.0968    0.2100    1.2323   -0.1500    1.0588   -0.0694    0.3013   -0.0000];
   
 alpha4 = [ 
    0.1708    0.0875    0.1845   -0.1721    0.6025   -0.4520    0.4020   -0.1342   -0.0297
    0.0000    0.0107   -0.0421    0.0838   -0.1464    0.2353   -0.2967    0.2152   -0.1285
   -0.0007    0.4919   -2.4650    4.0932   -5.4980    4.7321   -2.5108    0.1175   -0.3932
    0.0000   -0.7854    2.4810   -4.1501    5.3331   -4.6409    2.5695   -0.1485    0.5635
   -0.0297   -0.0052    0.0574    0.3020    0.2104    0.6189    0.2705    0.3837    0.1716
   -0.1285   -0.2700   -0.6332   -0.4902    0.0367   -1.2479    0.1504   -0.2818    0.0000
   -0.3932    0.6516   -0.5306    0.4930   -0.1726   -0.1155    0.1447   -0.4643   -0.0014
    0.5635    1.1432    0.4266    1.1761   -0.0382    1.2619   -0.1024    0.4008    0.0000];   

stride_time = 0.48;
    
   alpha;
   right_stance = true;
   step_timeoffset = 0;
   num_leg_joint = 4;
   
   heel_contact_time = 0.1;
   full_contact_time = 0.2;
   
%    pelvis(1) knee_r ankle_r heel_r(4) toe_joint_r toe_tip_r(6) toe_back_r(7) ...
%                     knee_l ankle_l heel_l(10) toe_joint_l toe_tip_l(12) toe_back_l ...
%                     shoulder
   cidx_heel_r = [ 4 ];
   cidx_foot_r = [ 4, 6, 7 ];
   cidx_toe_r = [ 6, 7 ];
   
   cidx_heel_l = [ 10 ];
   cidx_foot_l = [ 10, 12, 13 ];
   cidx_toe_l = [ 12, 13 ];
end

methods
    function [tau, jpos_des, jvel_des] = getJointCMD(obj, state, curr_time)
        obj.alpha = obj.alpha4;
        
        seq_time = (curr_time - obj.step_timeoffset);
        s = seq_time/obj.stride_time;
        sdot = 1/obj.stride_time;
        
        if (s > 1)
            s = 1;
            sdot = 0;
        end
        
        
        num_joint = length(state)/2-3;
        jpos_des = zeros(num_joint,1);
        jvel_des = zeros(num_joint,1);
        
        for i=1:obj.num_leg_joint
            if(obj.right_stance)
                [jpos_des(i), jvel_des(i)] = fn_bezier_pt(obj.alpha(i,:), s, sdot);
                [jpos_des(i+obj.num_leg_joint), jvel_des(i+obj.num_leg_joint)] = fn_bezier_pt(obj.alpha(i+4,:), s, sdot);
            else
                [jpos_des(i), jvel_des(i)] = fn_bezier_pt(obj.alpha(i+4,:), s, sdot);
                [jpos_des(i+obj.num_leg_joint), jvel_des(i+obj.num_leg_joint)] = fn_bezier_pt(obj.alpha(i,:), s, sdot);
            end
        end
        
%         % Contact Set
%         if (seq_time < obj.heel_contact_time)
%             if(obj.right_stance)
%                 contact_idx = obj.cidx_heel_r;
%             else
%                 contact_idx = obj.cidx_heel_l;
%             end
%         % Full Contact
%         elseif(seq_time < obj.full_contact_time)
%             if(obj.right_stance)
%                 contact_idx = obj.cidx_foot_r;
%             else
%                 contact_idx = obj.cidx_foot_l;
%             end
%         % Toe Contact
%         else
%             if(obj.right_stance)
%                 contact_idx = obj.cidx_toe_r;
%             else
%                 contact_idx = obj.cidx_toe_l;
%             end
%         end

        
        % Contact check   
        key_pts = keypoints_biped(state);
        if(obj.right_stance)            
            check_pt_idx = [4, 6, 7, 10, 12, 13];
            num_cp = 0;
            contact_idx = [];
            for i = 1:length(check_pt_idx)
                idx = check_pt_idx(i);
                if(key_pts(2,idx) < 0)
                num_cp = num_cp + 1;
                contact_idx(num_cp) = idx;
                end
            end
        else
            check_pt_idx = [4, 6, 7, 10, 12, 13];
            num_cp = 0;
            contact_idx = [];
            for i = 1:length(check_pt_idx)
                idx = check_pt_idx(i);
                if(key_pts(2,idx) < 0)
                num_cp = num_cp + 1;
                contact_idx(num_cp) = idx;
                end
            end

        end
        tau = wbc_jpos_ctrl(state, jpos_des, jvel_des, contact_idx);

        heel_contact = false;
        key_pts = keypoints_biped(state);
        if(obj.right_stance)
            if(key_pts(2, 10) < 0)
            heel_contact = true;
            end
        else
            if(key_pts(2, 4) < 0)
            heel_contact = true;
            end
        end
%         
%         if( seq_time >= obj.stride_time && heel_contact)
        if( seq_time >= obj.stride_time)
            % switching stance_leg 
            obj.right_stance = ~obj.right_stance;
            obj.step_timeoffset = curr_time;            
        end
        
    end % End of getJointCMD   
    
end
    
end    