%% runs all the function calling scripts here

%initialize variables
err_old = 0;
key = uicontrol('Style', 'PushButton', ...
                    'String', 'Break', ...
                    'Callback', 'delete(gcbf)');

while (ishandle(key))
% getting robot states from top mounted camera
    [s_l,s_f] = camera_feedback();
    
    [vel_alpha,vel_f,err] = follower_control(s_l,s_f,err_old);
    err_old = err;
%     pause(0.5);
end