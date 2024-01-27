disp('Program started');
% sim=remApi('remoteApi','extApi.h'); % using the header (requires a compiler)
sim=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
sim.simxFinish(-1); % just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19999,true,true,5000,5);

phi=40;
theta=30;
psi=0;
sigma=0:1:360;
[theta_traj]=Infinite_Rotational_Motion2(phi,theta,psi,45,90,90,0,120,240,0,120,240,sigma);

if (clientID>-1)
    disp('Connected to remote API server');
    [returnCode,joint_A1]=sim.simxGetObjectHandle(clientID,'joint_A1',sim.simx_opmode_blocking);
    [returnCode,joint_A2]=sim.simxGetObjectHandle(clientID,'joint_A2',sim.simx_opmode_blocking);
    [returnCode,joint_A3]=sim.simxGetObjectHandle(clientID,'joint_A3',sim.simx_opmode_blocking);
    [returnCode,position_u3]=sim.simxGetObjectHandle(clientID,'position_u3',sim.simx_opmode_blocking);
    for k=1:length(sigma)
        sim.simxSetJointPosition(clientID,joint_A1,deg2rad(theta_traj(1,k)),sim.simx_opmode_blocking);
        sim.simxSetJointPosition(clientID,joint_A2,deg2rad(theta_traj(2,k)),sim.simx_opmode_blocking);
        sim.simxSetJointPosition(clientID,joint_A3,-deg2rad(theta_traj(3,k)),sim.simx_opmode_blocking);
        %disp(k)
        [returnCode,position]=sim.simxGetObjectPosition(clientID,joint_A1,-1,sim.simx_opmode_blocking);
        disp(position)
    end
end
sim.delete();