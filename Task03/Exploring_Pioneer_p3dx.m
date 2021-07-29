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

% motor handles
leftMotor = 0;
rightMotor = 0;

% sensor reading buffers
detectionState = zeros(1, 6);   % Detection status
detectedPoint = zeros(6, 3);    % Point (vector)
range = zeros(1, 6);            % Euclidean distance

% robot position vector
robot_x = [];
robot_y = [];

% time-sequence vector
time = [];

% time-variables initialization
step_time = 0;
current_time = 0;
total_time = 0;

% driving velocity
driveVel = 2;

if (clientID>-1)
    
    connected = true;
    disp('Connected to remote API server');
    
    % get object handles
    [~,robot]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx',vrep.simx_opmode_blocking);
    [~,leftMotor]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor',vrep.simx_opmode_blocking);
    [~,rightMotor]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_rightMotor',vrep.simx_opmode_blocking);
    
    % proximity sensor handles
    [~,rangefinder(1)]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor1',vrep.simx_opmode_blocking);
    [~,rangefinder(2)]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor2',vrep.simx_opmode_blocking);
    [~,rangefinder(3)]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor3',vrep.simx_opmode_blocking);
    [~,rangefinder(4)]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor4',vrep.simx_opmode_blocking);
    [~,rangefinder(5)]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor5',vrep.simx_opmode_blocking);
    [~,rangefinder(6)]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor6',vrep.simx_opmode_blocking);
    
    % initialize proximity sensor reading
    for i = 1:6
    [~,detectionState(i),detectedPoint(i, :),~,~]=vrep.simxReadProximitySensor(clientID,rangefinder(i),vrep.simx_opmode_streaming );
    end
  
end

 if (connected)   
    while true
        
        % start time
        tic
        
        % read object position
        [record, robot_position] = vrep.simxGetObjectPosition(clientID, robot, -1,vrep.simx_opmode_blocking);
        robot_x = [robot_x, robot_position(1)];
        robot_y = [robot_y, robot_position(2)];
        
        % run motors
        [~]=vrep.simxSetJointTargetVelocity(clientID, leftMotor, driveVel, vrep.simx_opmode_blocking);
        [~]=vrep.simxSetJointTargetVelocity(clientID, rightMotor, driveVel, vrep.simx_opmode_blocking);
        
        % read rangefinders sensors
        for n = 1:6
            [~,detectionState(n),detectedPoint(n, :),~,~]=vrep.simxReadProximitySensor(clientID,rangefinder(n),vrep.simx_opmode_buffer);
            range(n) = norm(detectedPoint(n, :)); % Euclidean distance to obstacles 
        end
        
        % left rangefinder sensors obstacle detection 
        if (range(1) < 0.4 || range(2) < 0.4 || range(3) < 0.4)
            [~]=vrep.simxSetJointTargetVelocity(clientID, leftMotor, 0, vrep.simx_opmode_blocking);
            [~]=vrep.simxSetJointTargetVelocity(clientID, rightMotor, 0, vrep.simx_opmode_blocking);
            [~]=vrep.simxSetJointTargetVelocity(clientID, leftMotor, driveVel, vrep.simx_opmode_blocking);
            [~]=vrep.simxSetJointTargetVelocity(clientID, rightMotor, -driveVel, vrep.simx_opmode_blocking);
            
        % right rangefinder sensors obstacle detection
        elseif (range(4) < 0.4 || range(5) < 0.4 || range(6) < 0.4)
            [~]=vrep.simxSetJointTargetVelocity(clientID, leftMotor, 0, vrep.simx_opmode_blocking);
            [~]=vrep.simxSetJointTargetVelocity(clientID, rightMotor, 0, vrep.simx_opmode_blocking);
            [~]=vrep.simxSetJointTargetVelocity(clientID, leftMotor, -driveVel, vrep.simx_opmode_blocking);
            [~]=vrep.simxSetJointTargetVelocity(clientID, rightMotor, driveVel, vrep.simx_opmode_blocking);
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