clear,clc
set(groot,'defaultAxesTickLabelInterpreter','latex');
set(groot,'defaulttextinterpreter','latex');
set(groot,'defaultLegendInterpreter','latex');

%%
clear,clc
v1=[0;-1;0];
v2=[0.8660;0.5;0];
v3=[-0.8660;0.5;0];
sigma=0:1:720;
[theta_traj]=Infinite_Rotational_Motion(v1,v2,v3,45,90,90,0,120,240,sigma);
%%
k=1:length(sigma);
figure
plot(theta_traj(2,:))
% plot3(theta_traj(1,:),theta_traj(2,:),theta_traj(3,:))
% xlabel("\theta_1")
% ylabel("\theta_2")
% zlabel("\theta_3")
grid on
set(gcf,'color','w')
% phi=zeros(1,length(k));
% theta=zeros(1,length(k));
% psi=zeros(1,length(k));
% for i=1:length(k)
%     [phi(i),theta(i),psi(i)]=forward_kinematics2(45,0,120,240,90,120,90,theta_traj(1,i),theta_traj(2,i),theta_traj(3,i));
% end
%%
figure
plot(k,phi,k,theta,k,psi)
grid on
set(gcf,'color','w')


%%
clear,clc
theta1=0:10:360;
theta2=0:10:360;
theta3=0:10:360;
V=Configuration_Space(45,90,120,0,120,240,theta1,theta2,theta3,0.2);
%%
[V1,V2,V3]=meshgrid(V(1,:),V(2,:),V(3,:));

%%
figure
plot3(V(1,:),V(2,:),V(3,:),'.')
% hold on
% plot3(theta_traj(1,:),theta_traj(2,:),theta_traj(3,:))
xlabel("$\theta_{1}$")
ylabel("$\theta_{2}$")
zlabel("$\theta_{3}$")
title("Configuration space of the coaxial SPM")
view([360,360,360])
grid on
grid minor
axis equal
set(gcf,'color','w')
%%
lineVector = [360, 360, 360];
% Normalize the line vector
lineVector = lineVector / norm(lineVector);

% Create a plane equation: ax + by + cz = d
a = lineVector(1);
b = lineVector(2);
c = lineVector(3);

% Choose a value for d (adjust as needed)
d = 0;

% Create the plane
plane = @(x, y) (d-a*x-b*y)/c;
planepoints=plane(V(1,:),V(2,:));

figure
plot3(V(1,:),V(2,:),planepoints,'.')
grid on



%%
clear, clc
[X,Y,Z]=sphere(60);
x=X(11:end,:);
y=Y(11:end,:);
z=Z;
%%
figure
scatter3(X,Y,Z,"blue",".")
xlabel("X")
ylabel("Y")
zlabel("Z")
axis equal
grid on
set(gcf,'color','w')

%%
clear
%[v1,v2,v3]=forward_kinematics(45,0,120,240,90,120,0,0,0)
thetai=inverse_kinematics(45,0,120,240,90,90,40,30,20);
%%
[phi,theta,psi,v1,v2,v3]=forward_kinematics2(45,0,120,240,90,120,90,thetai(1),thetai(2),thetai(3));

%%
clear
phi=0:5:360;
theta=0:5:360;
psi=0:5:360;
[W,N,nn]=Workspace1(45,0,120,240,90,90,120,phi,theta,psi,0.2);


%%
figure
plot3(W(1,:),W(2,:),W(3,:),'.')
xlabel("$\phi$")
ylabel("$\theta$")
zlabel("$\psi$")
view(90,0)
grid on
grid minor
set(gcf,'color','w')

%%
figure
plot3(N(1,:),N(2,:),N(3,:),'.')
hold on
plot3(nn(1,:),nn(2,:),nn(3,:),'.')
hold on
xlabel("$x$")
ylabel("$y$")
zlabel("$z$")
zlim([0,1])
%view(0,90)
grid on
grid minor
set(gcf,'color','w')

%

%%

a=serialport("COM5",115200);
disp(a.Status)

%%
steps1=theta_traj(1,:)*((16*200)/360);
steps2=theta_traj(2,:)*((16*200)/360);
steps3=theta_traj(3,:)*((16*200)/360);
steps=[round(steps1,4);round(steps2,4);round(steps3,4)];
tstring='I';
%strings1=cell(length(steps1),1);
for i=1:length(steps1)
    tstring=[tstring,',','M',' A',num2str((steps1(i))),' B',num2str((steps2(i))),' C',num2str((steps3(i)))];
    %strings1{i}=tstring;
    %write(a,tstring,"string");
    %disp(i)
    %pause(1);
    
end
%%
nano=serialport("COM6",115200);
configureCallback(nano,"terminator",@Measure_Orientation);
startTime=datetime('now');
i=0;
phi=nan;
theta=nan;
psi=nan;
h=animatedline;
while true
    if numel(nano.UserData) >= 3
        % Read data from UserData of the serialport object
        phi = nano.UserData(1);
        theta = nano.UserData(2);
        psi = nano.UserData(3);
    end
    
    t=datetime('now')-startTime;
    addpoints(h,datenum(t),phi);
    drawnow
end
%disp(nano.UserData)

%%
clear,clc
phi=0;
theta=30;
psi=10;
sigma=0:1:360;
[theta_traj]=Infinite_Rotational_Motion2(phi,theta,psi,45,90,90,0,120,240,0,120,240,sigma);

%%
theta1=theta_traj(1,:);
theta2=theta_traj(2,:);
theta3=theta_traj(3,:);
figure
subplot(3,1,1)
plot(theta1)
xlabel("\textit{Sample}")
ylabel("$\theta_{1}(deg)$")
grid on 
grid minor
set(gcf,'color','w')
subplot(3,1,2)
plot(theta2)
xlabel("\textit{Sample}")
ylabel("$\theta_{2}(deg)$")
grid on 
grid minor
set(gcf,'color','w')
subplot(3,1,3)
plot(theta1)
xlabel("\textit{Sample}")
ylabel("$\theta_{3}(deg)$")
grid on 
grid minor
set(gcf,'color','w')

%%
Z=zeros(1,length(theta1));
for i=1:length(theta1)
    w1=define_wi(45,0,120,240,theta1(i),theta2(i),theta3(i),1);
    w2=define_wi(45,0,120,240,theta1(i),theta2(i),theta3(i),2);
    w3=define_wi(45,0,120,240,theta1(i),theta2(i),theta3(i),3);
    [v1,v2,v3]=forward_kinematics(45,0,120,240,90,120,theta1(i),theta2(i),theta3(i));
    J=calculate_Jacobian(w1,w2,w3,v1,v2,v3);
    Z(i)=conditioning_index(J);
end
figure
plot(Z)
xlabel("\textit{Sample}")
ylabel("$\zeta(J)$")
% yticks([0,0.2,0.4,0.6,0.8,1])
% yticklabels(["$0$","$0.2$","$0.4$","$0.6$","$0.8$","$1$"])
ylim([0.8,1])
grid on
grid minor
set(gcf,'color','w')
