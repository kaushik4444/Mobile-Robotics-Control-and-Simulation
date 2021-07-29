% Feel free to add code anywhere you find necessary to solve the task.
% as a guide, we have provide recommendation where to add code
clear all, clc
%% DO NOT CHANGE ANYTHING HERE - Setting up the remote api
vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections
%clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
clientID=vrep.simxStart('127.0.0.1',19997,true,true,5000,1);

% connection status
connected = false;

% robot parameters
d = 0.1950; % wheel radius
r = d/2; % wheel radius
T = 0.3310; % wheel track
del_theta = 0;
time = 0;
curr_wz = 0;
accel_x = 0;
curr_ax = 0;
gyro_z = 0;
vx = 0;
theta = -pi;
curr_x = 5;
curr_y = 2;
del_time = 0;
x = 5;
y = 2;
p = 0;
q = 0;

if (clientID>-1)
    
    connected = true;
    disp('Connected to remote API server');
    
    % start simulation
    e = vrep.simxStartSimulation(clientID,vrep.simx_opmode_blocking);
    
    % IMU signals
    [gyro_err,gyro_signal]=vrep.simxReadStringStream(clientID,'gyro_data',vrep.simx_opmode_streaming);
    [accel_err,accel_signal]=vrep.simxReadStringStream(clientID,'accel_data',vrep.simx_opmode_streaming);
    
end
% call figure
%--------------------------------------------------------------------
environment(); %uncomment this line to plot in the map
%--------------------------------------------------------------------
line = line(nan, nan, 'color', 'red');
pause(0.1);
if (connected)   
    while(true) % CHANGE THIS LINE TO 'while loop'
        
        % read gyroscope and accelerometer data
        [gyro_err,gyro_signal]=vrep.simxReadStringStream(clientID,'gyro_data',vrep.simx_opmode_buffer);
        [accel_err,accel_signal]=vrep.simxReadStringStream(clientID,'accel_data',vrep.simx_opmode_buffer);
        
        % Gyroscope
        % average angular velocity in Z direction
        gyro_z = 0;
        accel_x = 0;
        
        if gyro_err == vrep.simx_return_ok
            [gyro_buffer]= vrep.simxUnpackFloats(gyro_signal);
            for i = 3:3:length(gyro_buffer)
                gyro_z = gyro_z + gyro_buffer(i);
            end
            curr_wz = 3*gyro_z/(length(gyro_buffer));
        end
        
        % Accelerometer
        % average translational acceleration 
        if accel_err == vrep.simx_return_ok
            [accel_buffer]= vrep.simxUnpackFloats(accel_signal);
            for i = 1:3:length(accel_buffer)
                accel_x = accel_x + accel_buffer(i);
            end
            accel_x = (3*accel_x/(length(accel_buffer)));
            curr_ax = abs(accel_x);
        end
        p = [p, curr_wz];   
        q = [q, curr_ax];
        
        % Retrieves the simulation time of the last fetched command 
        [cmdTime] = vrep.simxGetLastCmdTime(clientID); % TIME IF YOU NEED IT
        del_time = (cmdTime - time)/1000; % time step
        
        del_s = vx * del_time + (curr_ax * del_time^2)/2 + 0.022; % distance travelled in a step
        vx = curr_ax*del_time; % velocity 
        
        del_theta = curr_wz * del_time; %current orientation of pioneer
        
        time = cmdTime;
        theta = theta + del_theta;
        
        curr_x = curr_x + del_s*cos(theta);
        curr_y = curr_y + del_s*sin(theta);     
        
        % plot in real-time
        x = get(line, 'xData');
        y = get(line, 'yData');
        
        %---------------------------------------------------------------------------------
        x = [x, curr_x];   % change n to any variable you want plotted
        y = [y, curr_y]; % change m to any variable you want plotted
        %---------------------------------------------------------------------------------
        
        set(line, 'xData', x, 'yData', y);
        pause(0.1);
    end
    
    % stop simulation
    [~]=vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot);
    pause(5);
    
    % Now close the connection to V-REP:    
    vrep.simxFinish(clientID);
else
    disp('Failed connecting to remote API server');
end
vrep.delete(); % call the destructor!