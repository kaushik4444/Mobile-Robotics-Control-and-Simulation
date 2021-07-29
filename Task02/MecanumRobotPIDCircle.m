% Use this script as a template for your code
% You might have to make several copies of the script for the different subtasks
% Communication to v-rep is already incorporated
% To see the robot in action, please ensure that v-rep is running before you execute this script

% Handles:
%  - The handle called bodyFrame will return the robot's position in the inertial frame 
%  - The motor handles give you access to the motor in v-rep
%  - No need for sensors at this point. Only robot position should serve as feedback

% For errors or questions send me an e-mail: nasser.gyagenda@uni-siegen.de

%% DO NOT CHANGE ANYTHING HERE - Setting up the remote api
vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
connected = false;

if (clientID>-1)
    connected = true;
    disp('Connected to remote API server');
end

% Vehicle parameters
a = 0.158516;       % COM to any front/back wheels [m]
b = 0.228020;       % COM to any right/left wheels [m]
R = 0.050369;     % Wheel radius [m]
r = 2.1;
count = 1;
error_threshold = 0.01;

if(connected)
    % handles to wheel motors
    [returnCode,wheelFL]=vrep.simxGetObjectHandle(clientID,'rollingJoint_fl',vrep.simx_opmode_blocking);
    [returnCode,wheelFR]=vrep.simxGetObjectHandle(clientID,'rollingJoint_fr',vrep.simx_opmode_blocking);
    [returnCode,wheelRR]=vrep.simxGetObjectHandle(clientID,'rollingJoint_rr',vrep.simx_opmode_blocking);
    [returnCode,wheelRL]=vrep.simxGetObjectHandle(clientID,'rollingJoint_rl',vrep.simx_opmode_blocking);
    
    % handle to robot body reference frame
    [returnCode,bodyFrame]=vrep.simxGetObjectHandle(clientID,'body_frame',vrep.simx_opmode_blocking);
end


%% To Do
if(connected)
    %---------------------------------------------------------- 
    while (count < 3)

        for i = -pi/18:-pi/18:-2*pi
            
            x = r *cos(i);
            y = r *sin(i);
            theta = i + pi/2;
            
            if theta >= 2*pi
                theta = theta - 2*pi;
            end
            
            errorx = 0; errory = 0; errorgamma = 0;
            previous_errorx = 0; previous_errory = 0; previous_errorgamma = 0;
            total_errorx = 0; total_errory = 0; total_errorgamma = 0;
            
            px = 0; ix = 0; dx = 0;
            py = 0; iy = 0; dy = 0;
            pgamma = 0; igamma = 0; dgamma = 0;
            
            t = 0.05;
            proportional = 3;
            integral = 0.01;
            derivative = 0.2;

            while true

                [record, current_pos] = vrep.simxGetObjectPosition(clientID,bodyFrame,-1,vrep.simx_opmode_blocking);
                [record, current_ori] = vrep.simxGetObjectOrientation(clientID,bodyFrame,-1,vrep.simx_opmode_blocking);
                
                if theta > pi && current_ori(3) <= 0
                    current_ori(3) = current_ori(3) + 2*pi;
                elseif theta < -pi && current_ori(3) >= 0
                    current_ori(3) = current_ori(3) - 2*pi;
                end
                
                errorx = x - current_pos(1);
                errory = y - current_pos(2);
                errorgamma = theta - current_ori(3);
                
                if (abs(errorx) < error_threshold && abs(errory) < error_threshold && abs(errorgamma) < error_threshold)
                    break
                end
                
                px = proportional * errorx;
                ix = integral * t * (errorx + total_errorx);
                dx = derivative * (errorx - previous_errorx) / t;

                py = proportional * errory;
                iy = integral * t * (errory + total_errory);
                dy = derivative * (errory - previous_errory) / t;

                pgamma = proportional * errorgamma;
                igamma = integral * t * (errorgamma + total_errorgamma);
                dgamma = derivative * (errorgamma - previous_errorgamma) / t;

                total_errorx = total_errorx + errorx;
                total_errory = total_errory + errory;
                total_errorgamma = total_errorgamma + errorgamma;

                previous_errorx = errorx;
                previous_errory = errory;
                previous_errorgamma = errorgamma;

                vx1 = px + ix + dx;
                vy1 = py + iy + dy;
                w1 = (pgamma + igamma + dgamma);

                A = [cos(theta) sin(theta) 0; -sin(theta) cos(theta) 0 ; 0 0 1]*[vx1; vy1; w1];
                vx = A(1,1);
                vy = A(2,1);
                w = A(3,1);

                v1 = (vx - vy - (a+b) * w);
                v2 = (vx + vy + (a+b) * w);
                v3 = (vx + vy - (a+b) * w);
                v4 = (vx - vy + (a+b) * w);

                vrep.simxSetJointTargetVelocity(clientID,wheelFL,v1,vrep.simx_opmode_blocking);
                vrep.simxSetJointTargetVelocity(clientID,wheelFR,v2,vrep.simx_opmode_blocking);
                vrep.simxSetJointTargetVelocity(clientID,wheelRR,v4,vrep.simx_opmode_blocking);
                vrep.simxSetJointTargetVelocity(clientID,wheelRL,v3,vrep.simx_opmode_blocking);
            end 

            vrep.simxSetJointTargetVelocity(clientID,wheelFL,0,vrep.simx_opmode_blocking);
            vrep.simxSetJointTargetVelocity(clientID,wheelFR,0,vrep.simx_opmode_blocking);
            vrep.simxSetJointTargetVelocity(clientID,wheelRR,0,vrep.simx_opmode_blocking);
            vrep.simxSetJointTargetVelocity(clientID,wheelRL,0,vrep.simx_opmode_blocking);
            [record, current_pos] = vrep.simxGetObjectPosition(clientID,bodyFrame,-1,vrep.simx_opmode_blocking);
            [record, current_ori] = vrep.simxGetObjectOrientation(clientID,bodyFrame,-1,vrep.simx_opmode_blocking);
            final_pos = [current_pos(1),current_pos(2),current_ori(3)];
            disp(final_pos);
        end
        count = count + 1;
    end
    
    while (count < 5)
        
        for i = pi/18:pi/18:2*pi

            x = r *cos(i);
            y = r *sin(i);
            theta = i + pi/2;
            
            if theta >= 2*pi
                theta = theta - 2*pi;
            end
            
            errorx = 0; errory = 0; errorgamma = 0;
            previous_errorx = 0; previous_errory = 0; previous_errorgamma = 0;
            total_errorx = 0; total_errory = 0; total_errorgamma = 0;
            
            px = 0; ix = 0; dx = 0;
            py = 0; iy = 0; dy = 0;
            pgamma = 0; igamma = 0; dgamma = 0;
            
            t = 0.05;
            proportional = 3;
            integral = 0.01;
            derivative = 0.2;

            while true

                [record, current_pos] = vrep.simxGetObjectPosition(clientID,bodyFrame,-1,vrep.simx_opmode_blocking);
                [record, current_ori] = vrep.simxGetObjectOrientation(clientID,bodyFrame,-1,vrep.simx_opmode_blocking);
                
                if theta > pi && current_ori(3) <= 0
                    current_ori(3) = current_ori(3) + 2*pi;
                elseif theta < -pi && current_ori(3) >= 0
                    current_ori(3) = current_ori(3) - 2*pi;
                end
                
                errorx = x - current_pos(1);
                errory = y - current_pos(2);
                errorgamma = theta - current_ori(3);
                
                if (abs(errorx) < error_threshold && abs(errory) < error_threshold && abs(errorgamma) < error_threshold)
                    break
                end
                
                px = proportional * errorx;
                ix = integral * t * (errorx + total_errorx);
                dx = derivative * (errorx - previous_errorx) / t;

                py = proportional * errory;
                iy = integral * t * (errory + total_errory);
                dy = derivative * (errory - previous_errory) / t;

                pgamma = proportional * errorgamma;
                igamma = integral * t * (errorgamma + total_errorgamma);
                dgamma = derivative * (errorgamma - previous_errorgamma) / t;

                total_errorx = total_errorx + errorx;
                total_errory = total_errory + errory;
                total_errorgamma = total_errorgamma + errorgamma;

                previous_errorx = errorx;
                previous_errory = errory;
                previous_errorgamma = errorgamma;

                vx1 = px + ix + dx;
                vy1 = py + iy + dy;
                w1 = (pgamma + igamma + dgamma);

                A = [cos(theta) sin(theta) 0; -sin(theta) cos(theta) 0 ; 0 0 1]*[vx1; vy1; w1];
                vx = A(1,1);
                vy = A(2,1);
                w = A(3,1);

                v1 = (vx - vy - (a+b) * w);
                v2 = (vx + vy + (a+b) * w);
                v3 = (vx + vy - (a+b) * w);
                v4 = (vx - vy + (a+b) * w);

                vrep.simxSetJointTargetVelocity(clientID,wheelFL,v1,vrep.simx_opmode_blocking);
                vrep.simxSetJointTargetVelocity(clientID,wheelFR,v2,vrep.simx_opmode_blocking);
                vrep.simxSetJointTargetVelocity(clientID,wheelRR,v4,vrep.simx_opmode_blocking);
                vrep.simxSetJointTargetVelocity(clientID,wheelRL,v3,vrep.simx_opmode_blocking);
            end 

            vrep.simxSetJointTargetVelocity(clientID,wheelFL,0,vrep.simx_opmode_blocking);
            vrep.simxSetJointTargetVelocity(clientID,wheelFR,0,vrep.simx_opmode_blocking);
            vrep.simxSetJointTargetVelocity(clientID,wheelRR,0,vrep.simx_opmode_blocking);
            vrep.simxSetJointTargetVelocity(clientID,wheelRL,0,vrep.simx_opmode_blocking);
            
            [record, current_pos] = vrep.simxGetObjectPosition(clientID,bodyFrame,-1,vrep.simx_opmode_blocking);
            [record, current_ori] = vrep.simxGetObjectOrientation(clientID,bodyFrame,-1,vrep.simx_opmode_blocking);
            final_pos = [current_pos(1),current_pos(2),current_ori(3)];
            disp(final_pos);
        end
        count = count + 1;
    end
    %add your code here
    
    %----------------------------------------------------------
    
    % Now close the connection to V-REP:    
    vrep.simxFinish(clientID);
else
    disp('Failed connecting to remote API server');
end
vrep.delete(); % call the destructor!