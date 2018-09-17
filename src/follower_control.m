%% This function takes current state of leader and follower robots and old error
%  returns desired speed and turn rate
function [vel_alpha,vel_f,err] = follower_control(s_l,s_f,err_old)

r = sqrt((s_l.x-s_f.x)^2+(s_l.y-s_f.y)^2); % euclidean distance between two robots
theta = atan2(s_l.y-s_f.y,s_l.x-s_l.y);    % angle formed by line joining two robots
d = 2;      % optimal distance to maintain from the leader

% PID controller implementation for follower speed
err.R = r-d;
err_R_inte = err.R + err_old.R;
R_dot = Kp_r*err.R+Ki_r*err_R_inte;
vel_f = R_dot/cos(s_f.alpha-theta);

% PID controller implementation for follower turn rate (R*alpha_dot)
err.delta = s_f.alpha-theta;
err_a_inte = err.delta + err_old.delta;
vel_alpha = Kp_a*err.delta + Ki_a*err_a_inte;

end