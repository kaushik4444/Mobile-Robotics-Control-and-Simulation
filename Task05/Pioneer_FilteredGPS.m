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
% starting values
ref_x = 5; 
ref_y = 2;

error_x = [];
error_y = [];
currgps_x = 0;
currgps_y = 0;
currodo_x = 5; 
currodo_y = 2; 
theta = -pi;  
prev_rotLM = 0;
prev_rotRM = 0;


if (clientID>-1)
    
    connected = true;
    disp('Connected to remote API server');
    
    % start simulation
    e = vrep.simxStartSimulation(clientID,vrep.simx_opmode_blocking);
    
    % get object handles
    [~,leftMotor]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor',vrep.simx_opmode_blocking);
    [~,rightMotor]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_rightMotor',vrep.simx_opmode_blocking);
    
    % initialize motor position reading (encoder)
    [~, rotLM]=vrep.simxGetJointPosition(clientID, leftMotor, vrep.simx_opmode_streaming);
    [~, rotRM]=vrep.simxGetJointPosition(clientID, rightMotor, vrep.simx_opmode_streaming);
    
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
        
        % get motor angular position (encoder emulator)
        [~, rotLM]=vrep.simxGetJointPosition(clientID, leftMotor, vrep.simx_opmode_buffer );
        [~, rotRM]=vrep.simxGetJointPosition(clientID, rightMotor, vrep.simx_opmode_buffer );
         
        % correction of rotLM/rotRM at -pi/pi boundary condition 
        if (prev_rotLM > 0 && rotLM < 0) 
            prev_rotLM = prev_rotLM - 2*pi;
        end
        if (prev_rotRM > 0 && rotRM < 0) 
             prev_rotRM = prev_rotRM - 2*pi;
        end
        % change of wheels angular position in a step
        del_rotLM = rotLM - prev_rotLM;
        del_rotRM = rotRM - prev_rotRM;
                
        prev_rotLM = rotLM;
        prev_rotRM = rotRM;
        
        % distance travelled by right and left wheels
        del_sLM = r*del_rotLM;
        del_sRM = r*del_rotRM;
        
        % average distance travelled by pioneer in a step
        del_s = (del_sRM + del_sLM)/2;
        
        % change of orientataion of pioneer in a step
        del_theta = (del_sRM - del_sLM)/T;
        
        n = cos(theta + del_theta/2);
        m = sin(theta + del_theta/2);
        
        theta = theta + del_theta;
        
        % minimizing the error amplification
        if abs(n) < 0.15
            n = 0;
            m = abs(sin(theta))/sin(theta);
        elseif abs(m) < 0.15
            m = 0;
            n = abs(cos(theta))/cos(theta);
        end
        
        % transformation into global coordinate system
        currodo_x = currodo_x + del_s*n;
        currodo_y = currodo_y + del_s*m;         
          
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
        currgps_x = gps_buffer(length(gps_buffer)-2) + error_x(1);
        currgps_y = gps_buffer(length(gps_buffer)-1) + error_y(1);
        
        % Residue calculation
        res_x = currgps_x - currodo_x;
        res_y = currgps_y - currodo_y;
        
        % Estimated values calculation
        est_x = currodo_x + 0.5 * res_x;
        est_y = currodo_y + 0.5 * res_y;
          
        %plot in real-time
        x = get(line, 'xData');
        y = get(line, 'yData');

        %---------------------------------------------------------------------------------
         x = [x, est_x];   % change n to any variable you want plotted
         y = [y, est_y];   % change m to any variable you want plotted
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