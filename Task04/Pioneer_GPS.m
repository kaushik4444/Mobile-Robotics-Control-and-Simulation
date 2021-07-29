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
T = 0.3310;% wheel track
ref_x = 5; 
ref_y = 2;
error_x = [];
error_y = [];
curr_x = 0;
curr_y = 0;

if (clientID>-1)
    
    connected = true;
    disp('Connected to remote API server');
    
    % start simulation
    e = vrep.simxStartSimulation(clientID,vrep.simx_opmode_blocking);
    
    % GPS signal
    [gps_err,gps_signal]=vrep.simxReadStringStream(clientID,'gps_data',vrep.simx_opmode_streaming);
    
end
% call figure
%--------------------------------------------------------------------
environment(); %uncomment this line to plot in the map
%--------------------------------------------------------------------
line = line(nan, nan, 'color', 'red');
pause(0.1);
 if (connected)        
      while(true)
        
        % read GPS position
        [gps_err,gps_signal]=vrep.simxReadStringStream(clientID,'gps_data',vrep.simx_opmode_buffer);

        if gps_err == vrep.simx_return_ok
            [gps_buffer]= vrep.simxUnpackFloats(gps_signal);
            %disp(gps_buffer) % sequence on (x, y, z,) position
        end

        % Retrieves the simulation time of the last fetched command 
        [cmdTime] = vrep.simxGetLastCmdTime(clientID); % TIME IF YOU NEED IT

        error_x = [error_x,ref_x - gps_buffer(1)];   % initial error in x
        error_y = [error_y,ref_y - gps_buffer(2)];   % initial error in y

        % Error correction to obtain current co-ordinates
        curr_x = gps_buffer(length(gps_buffer)-2) + error_x(1);
        curr_y = gps_buffer(length(gps_buffer)-1) + error_y(1);

        % plot in real-time
        x = get(line, 'xData');
        y = get(line, 'yData');

        %---------------------------------------------------------------------------------
         x = [x, curr_x];   % change n to any variable you want plotted
         y = [y, curr_y];   % change m to any variable you want plotted
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