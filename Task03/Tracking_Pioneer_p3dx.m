
% Feel free to add code anywhere you find necessary to solve the task.
% as a guide, we have provide recommendation where to add code

%% DO NOT CHANGE ANYTHING HERE - Setting up the remote api
vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

% connection status
connected = false;

% ultrasonic sensor handles
rangefinder = zeros(1, 6); 

% object handles
yawMotor = 0;
pitchMotor = 0;
camera = 0;

% sensor reference values
ref_row = 0.5; 
ref_col = 0.5;
current_row = 0.5;
current_col = 0.5;

% robot position vector
robot_x = [];
robot_y = [];

% time-sequence vector
time    = [];

% time-variables initialization
step_time = 0;
current_time = 0;
total_time = 0;

% Initialization of PID
pitch_error = 0; yaw_error = 0;
pitch_vel = 0; yaw_vel = 0;
previous_error_pitch = 0; previous_error_yaw = 0;
total_error_pitch = 0; total_error_yaw = 0;
p_pitch = 0; i_pitch = 0; d_pitch = 0;
p_yaw = 0; i_yaw = 0; d_yaw = 0;
t = 0.05;

% PID gain values
proportional = 1;
integral = 0;
derivative = 0;

if (clientID>-1)
    
    connected = true;
    disp('Connected to remote API server');
    
    % get camera system handles
    [~,camera]=vrep.simxGetObjectHandle(clientID,'VisionSensor',vrep.simx_opmode_blocking);
    [~,pitchMotor]=vrep.simxGetObjectHandle(clientID,'Pitch',vrep.simx_opmode_blocking);
    [~,yawMotor]=vrep.simxGetObjectHandle(clientID,'Yaw',vrep.simx_opmode_blocking);
    
    % get robot object handle
    [~,robot]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx',vrep.simx_opmode_blocking);
    
    % initialize camera
    [~, resolution, image]=vrep.simxGetVisionSensorImage2(clientID,camera,1,vrep.simx_opmode_streaming);    
end

 if (connected)   
    
    while true                   % true
        
        % start time
        tic
        
        % read object position
        [record, robot_position] = vrep.simxGetObjectPosition(clientID,robot,-1,vrep.simx_opmode_blocking);
        robot_x = [robot_x,robot_position(1)];
        robot_y = [robot_y,robot_position(2)];
        
        
        % get image
        [~, resolution, image]=vrep.simxGetVisionSensorImage2(clientID,camera,1,vrep.simx_opmode_buffer);
        
        
        % make sure there is a valid image
        if(size(image, 1) > 0)
            
            maxvalue = max(image, [], 'all');   % check for max intensity value
            [current_row,current_col] = find(image==maxvalue);  % check for the index of max intensity value in the image vector
            size_of_array = size(current_col);                  % multiple indices having max intensity value in the image vector
            centre_value = round(size_of_array(1)/2);
            pitch_error = -(ref_row - current_row(centre_value)*(1/64));   % calculation of pitch error
            yaw_error = (ref_col - current_col(centre_value)*(1/64));     % calculation of yaw error

            % PID controller implementation 
            p_pitch = proportional * pitch_error;
            i_pitch = integral * t * (pitch_error + total_error_pitch);
            d_pitch = derivative * (pitch_error - previous_error_pitch) / t;

            p_yaw = proportional * yaw_error;
            i_yaw = integral * t * (yaw_error + total_error_yaw);
            d_yaw = derivative * (yaw_error - previous_error_yaw) / t;

            total_error_pitch = total_error_pitch + pitch_error;
            total_error_yaw = total_error_yaw + yaw_error;

            previous_error_pitch = pitch_error;
            previous_error_yaw = yaw_error;

            pitch_vel = p_pitch + i_pitch + d_pitch;
            yaw_vel = p_yaw + i_yaw + d_yaw;

            vrep.simxSetJointTargetVelocity(clientID, pitchMotor, pitch_vel, vrep.simx_opmode_blocking);
            vrep.simxSetJointTargetVelocity(clientID, yawMotor, yaw_vel, vrep.simx_opmode_blocking);
        
        end
        
        % read time
        step_time = toc;
        current_time = total_time + step_time;
        time = [time, current_time];
        total_time = current_time;
    end
    
    % stop camera system motors
    [~]=vrep.simxSetJointTargetVelocity(clientID, pitchMotor, 0, vrep.simx_opmode_blocking);
    [~]=vrep.simxSetJointTargetVelocity(clientID, yawMotor, 0, vrep.simx_opmode_blocking);
    
    % now close the connection to V-REP:    
    vrep.simxFinish(clientID);
else
    disp('Failed connecting to remote API server');
end
vrep.delete(); % call the destructor!