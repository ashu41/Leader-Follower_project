%% This function takes current state of leader and follower robots and old error
%  returns desired speed and turn rate
function [vel_alpha,vel_f,err] = pose_controller(start,target,err_old)

r = sqrt((start.x-target.x)^2+(start.y-target.y)^2); % euclidean distance between two robots
theta = atan2(start.y-target.y,start.x-start.y);    % angle formed by line joining two robots
d = 2;      % optimal distance to maintain from the leader

% PID controller implementation for follower speed
err.R = r-d;
err_R_inte = err.R + err_old.R;
R_dot = Kp_r*err.R+Ki_r*err_R_inte;
vel_f = R_dot/cos(target.alpha-theta);

% PID controller implementation for follower turn rate (R*alpha_dot)
err.delta = target.alpha-theta;
err_a_inte = err.delta + err_old.delta;
vel_alpha = Kp_a*err.delta + Ki_a*err_a_inte;

end