function CoppeliaSim_Test()
    disp('Program started');
    % sim=remApi('remoteApi','extApi.h'); % using the header (requires a compiler)
    sim=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
    sim.simxFinish(-1); % just in case, close all opened connections
    clientID=sim.simxStart('127.0.0.1',19999,true,true,5000,5);

    if (clientID>-1)
        disp('Connected to remote API server');
            
        % Now try to retrieve data in a blocking fashion (i.e. a service call):
%         [res,objs]=sim.simxGetObjects(clientID,sim.sim_handle_all,sim.simx_opmode_blocking);
%         if (res==sim.simx_return_ok)
%             fprintf('Number of objects in the scene: %d\n',length(objs));
%         else
%             fprintf('Remote API function call returned with error code: %d\n',res);
%         end
        [returnCode,joint_A1]=sim.simxGetObjectHandle(clientID,'joint_A1',sim.simx_opmode_blocking);
        [returnCode,joint_A2]=sim.simxGetObjectHandle(clientID,'joint_A2',sim.simx_opmode_blocking);
        [returnCode,joint_A3]=sim.simxGetObjectHandle(clientID,'joint_A3',sim.simx_opmode_blocking);
        sim.simxSetJointPosition(clientID,joint_A1,100*pi/180,sim.simx_opmode_blocking);
        sim.simxSetJointPosition(clientID,joint_A2,50*pi/180,sim.simx_opmode_blocking);
        sim.simxSetJointPosition(clientID,joint_A3,-80*pi/180,sim.simx_opmode_blocking);
    end
    sim.delete();
end