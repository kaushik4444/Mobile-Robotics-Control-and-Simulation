                    size_of_array = round(1.3);
% Feel free to add code anywhere you find necessary to solve the task.
% as a guide, we have provide recommendation where to add code

%% DO NOT CHANGE ANYTHING HERE - Setting up the remote api
vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

% connection status
connected = false; 

% motor handles
leftMotor = 0;
rightMotor = 0;

% sensor reading buffers
detectionState = zeros(1, 6);   % Detection status
detectedPoint = zeros(6, 3);    % Point (vector)
range = zeros(1, 6);                % Euclidean distance

% driving velocity
driveVel = 2;


% position and orientation reference values 
min_distance = 0.26;   % 1 meter equivalent
max_deflection = 0.1;

% sensor reference values
ref_row = 0.5; 
ref_col = 0.5;
current_row = 0.5;
current_col = 0.5;

% robot position vector
leader_x = [];
leader_y = [];
follower_x = [];
follower_y = [];

% time-sequence vector
time = [];

% time-variables initialization
step_time = 0;
current_time = 0;
total_time = 0;

% Initialization of PID
depth_error = 0; orientation_error= 0;
depth_vel = 0; orientation_vel = 0;
previous_error_depth = 0; previous_error_orientation = 0;
total_error_depth = 0; total_error_orientation = 0;
p_depth = 0; i_depth = 0; d_depth = 0;
p_orientation = 0; i_orientation = 0; d_orientation = 0;
t = 0.05;

% PID gain values
proportional_depth = 40;
proportional_orientation = 0.8;
integral = 0.01;
derivative = 0;

if (clientID>-1)
    
    connected = true;
    disp('Connected to remote API server');
    
    % get object handles
    [~,leader]=vrep.simxGetObjectHandle(clientID,'Pioneer_leader',vrep.simx_opmode_blocking);
    [~,follower]=vrep.simxGetObjectHandle(clientID,'Pioneer_follower',vrep.simx_opmode_blocking);
    [~,leftMotor]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor',vrep.simx_opmode_blocking);
    [~,rightMotor]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_rightMotor',vrep.simx_opmode_blocking);
    
    % get camera system handles
    [~,rgbdCam]=vrep.simxGetObjectHandle(clientID,'rgbd_sensor',vrep.simx_opmode_blocking);    
    
    % initialize camera
    [~, resolution, rgb_image]=vrep.simxGetVisionSensorImage2(clientID,rgbdCam,1,vrep.simx_opmode_streaming);
    [~, d_resolution, depth_image]=vrep.simxGetVisionSensorDepthBuffer2(clientID,rgbdCam,vrep.simx_opmode_streaming);
    
end

 if (connected)   
    while true  
        
        % start time
        tic
        
        % read leader position
        [record, leader_pos] = vrep.simxGetObjectPosition(clientID,leader,-1,vrep.simx_opmode_blocking);
        leader_x = [leader_x,leader_pos(1)];
        leader_y = [leader_y,leader_pos(2)];
        
        % read follower position
        [record, follower_pos] = vrep.simxGetObjectPosition(clientID,follower,-1,vrep.simx_opmode_blocking);
        follower_x = [follower_x,follower_pos(1)];
        follower_y = [follower_y,follower_pos(2)];
        
        % get image
        [~, d_resolution, depth_image]=vrep.simxGetVisionSensorDepthBuffer2(clientID,rgbdCam,vrep.simx_opmode_buffer );
        [~, resolution, rgb_image]=vrep.simxGetVisionSensorImage2(clientID,rgbdCam,1,vrep.simx_opmode_buffer );
       
        %imshow(rgb_image); % uncomment to see rgb image
        %imshow(depth_image); % uncomment to see depth image
       
        % make sure there is a valid image
        if(size(rgb_image, 1) > 0 && size(depth_image, 1) > 0)
            maxvalue = max(rgb_image, [], 'all');                        % check for max intensity value
            if maxvalue ~= 0                                             % Leader detected
                [current_row,current_col] = find(rgb_image==maxvalue);   % check for the index of max intensity value in the image vector
                size_of_array = size(current_col);                       % multiple indices having max intensity value in the image vector
                centre_value = round(size_of_array(1)/2);
                x = (ref_row - current_row(centre_value)*(1/64));        % distance of max intensity value from centre             
                y = (ref_col - current_col(centre_value)*(1/64));

                depth_error = (depth_image(current_row(centre_value),current_col(centre_value)) - min_distance);  % calculation of depth_error
                orientation_error = abs(y) - max_deflection;              % calculation of orientation error        
                direction = y/abs(y);                                     % calculation of pitch error
                
                
                % PID controller implementation
                p_depth = proportional_depth * depth_error;
                i_depth = integral * t * (depth_error + total_error_depth);
                d_depth = derivative * (depth_error - previous_error_depth) / t;

                p_orientation = proportional_orientation * orientation_error;
                i_orientation = integral * t * (orientation_error + total_error_orientation);
                d_orientation = derivative * (orientation_error - previous_error_orientation) / t;

                total_error_depth = total_error_depth + depth_error;
                total_error_orientation = total_error_orientation + orientation_error;

                previous_error_depth = depth_error;
                previous_error_orientation = orientation_error;

                depth_vel = (p_depth + i_depth + d_depth) * driveVel;
                orientation_vel = (p_orientation + i_orientation + d_orientation) * direction * driveVel;
                
                if (depth_error >= 0 && orientation_error > 0)            % control for position and orientation
                    [~]=vrep.simxSetJointTargetVelocity(clientID, leftMotor, (depth_vel-orientation_vel), vrep.simx_opmode_blocking);
                    [~]=vrep.simxSetJointTargetVelocity(clientID, rightMotor, (depth_vel+orientation_vel), vrep.simx_opmode_blocking);
                elseif (depth_error >= 0)                                 % control for position
                    [~]=vrep.simxSetJointTargetVelocity(clientID, leftMotor, depth_vel, vrep.simx_opmode_blocking);
                    [~]=vrep.simxSetJointTargetVelocity(clientID, rightMotor, depth_vel, vrep.simx_opmode_blocking);
                elseif (orientation_error > 0)                            % control for orientation
                    [~]=vrep.simxSetJointTargetVelocity(clientID, leftMotor, -orientation_vel, vrep.simx_opmode_blocking);
                    [~]=vrep.simxSetJointTargetVelocity(clientID, rightMotor, orientation_vel, vrep.simx_opmode_blocking);
                else                                                      % stop motors      
                    [~]=vrep.simxSetJointTargetVelocity(clientID, leftMotor, 0, vrep.simx_opmode_blocking);
                    [~]=vrep.simxSetJointTargetVelocity(clientID, rightMotor, 0, vrep.simx_opmode_blocking);
                end
            else                                                          % search for the leader
                [~]=vrep.simxSetJointTargetVelocity(clientID, leftMotor, 0.4*driveVel, vrep.simx_opmode_blocking);
                [~]=vrep.simxSetJointTargetVelocity(clientID, rightMotor, -0.4*driveVel, vrep.simx_opmode_blocking);
            end
            
        end
        
        % read time
        step_time = toc;
        current_time = total_time + step_time;
        time = [time, current_time];
        total_time = current_time;
    end
    % stop wheels
    [~]=vrep.simxSetJointTargetVelocity(clientID, leftMotor, 0, vrep.simx_opmode_blocking);
    [~]=vrep.simxSetJointTargetVelocity(clientID, rightMotor, 0, vrep.simx_opmode_blocking);
    
    % Now close the connection to V-REP:    
    vrep.simxFinish(clientID);
else
    disp('Failed connecting to remote API server');
end
vrep.delete(); % call the destructor!