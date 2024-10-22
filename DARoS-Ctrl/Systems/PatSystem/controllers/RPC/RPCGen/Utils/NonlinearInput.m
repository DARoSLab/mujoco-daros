%% Converts the GRF to Momentum rate of change
function hDot = NonlinearInput(x, u, s, NUM_FEET)

% Rotation Matrix to transform torques
rpy_on = [1;1;1];  % activate roll, pitch, or yaw
R = simplify(RPYToR(rpy_on.*x(4:6)));%QuaternionMatrixSym(RPYToQuaternion(rpy_on.*x(4:6))'));
%R = simplify(QuaternionMatrixSym(RPYToQuaternion(rpy_on.*x(4:6))'));

% Set up symbolic CoM forces and torque vectors
f_com = sym(zeros(3, 1));
tau_com = sym(zeros(3, 1));

% Iterate through all of the robot legs
for foot = 1:NUM_FEET
    
    % Leg state number
    i_u = 6*(foot - 1);
    
    % Foot position vector from CoM (CURRENTLY ONLY FOR FLAT GROUND)
    r_foot = s(foot)*u(1+i_u:3+i_u);
    
    % Foot ground reaction forces
    f_foot = s(foot)*u(4+i_u:6+i_u);
    
    % Summation of CoM forces
    f_com = f_com + f_foot;
    
    % Summation of CoM torques
    tau_com = tau_com + CrossProd(r_foot)*f_foot;
end

% Net wrench on the CoM
hDot = [f_com; R.'*tau_com];
end