%%
clear, clc

clear, clc, close all
set(groot,'defaultAxesTickLabelInterpreter','latex');
set(groot,'defaulttextinterpreter','latex');
set(groot,'defaultLegendInterpreter','latex');

theta1=linspace(0,360,100);
theta2=linspace(0,360,100);
theta3=linspace(0,360,100);

alpha1=linspace(45,135,100);
alpha2=linspace(45,135,100);
[A1,A2]=meshgrid(alpha1,alpha2);
Z=zeros(length(alpha1),length(alpha1));
%z=2*A1-A2;

for i=1:length(alpha1)
    for j=1:length(alpha2)
        w1=define_wi(alpha1(i),0,120,240,theta1(j),theta2(j),theta3(j),1);
        w2=define_wi(alpha1(i),0,120,240,theta1(j),theta2(j),theta3(j),2);
        w3=define_wi(alpha1(i),0,120,240,theta1(j),theta2(j),theta3(j),3);
        [v1,v2,v3]=forward_kinematics(alpha1(i),0,120,240,alpha2(j),120,theta1(j),theta2(j),theta3(j));
        J=calculate_Jacobian(w1,w2,w3,v1,v2,v3);
        Z(j,i)=conditioning_index(J);
    end
end
%%
figure
surf(A1,A2,Z)
xlabel("$\alpha_{1}$")
ylabel("$\alpha_{2}$")
zlabel("$\zeta(J)$")
title("Local Dexterity as a function of $\alpha_{1}$ and $\alpha_{2}$ with $\beta_{1}=0$ y $\beta_{2}=90$")
colorbar
grid on
set(gcf,'color','w')

%%
clear,clc
theta1=linspace(0,360,100);
theta2=linspace(0,360,100);
theta3=linspace(0,360,100);

beta2=linspace(30,120,100);
Z2=zeros(1,length(beta2));

for i=1:length(beta2)
    w1=define_wi(45,0,120,240,theta1(i),theta2(i),theta3(i),1);
    w2=define_wi(45,0,120,240,theta1(i),theta2(i),theta3(i),2);
    w3=define_wi(45,0,120,240,theta1(i),theta2(i),theta3(i),3);
    [v1,v2,v3]=forward_kinematics(45,0,120,240,90,rad2deg(2*asin(sin(deg2rad(beta2(i))*cos(pi/6)))),theta1(i),theta2(i),theta3(i));
    J=calculate_Jacobian(w1,w2,w3,v1,v2,v3);
    Z2(i)=conditioning_index(J);
    
end
figure
plot(beta2,Z2)
xlabel("$\beta_{2}$")
ylabel("$\zeta(J)$")
title("Local dexterity as a function of $\beta_{2}$ with $\alpha_{1}=45$ y $\alpha_{2}=90$")
grid on
set(gcf,'color','w')

%%
clear, clc
[v1,v2,v3]=forward_kinematics(45,0,120,240,90,120,30,60,70)

%%
clear, clc
% v1=[-0.8967;0.4427;0];
% v2=[0.1471;-0.8314;-0.5358];
% v3=[0.7495;0.3887;0.5358];
theta_i=inverse_kinematics(45,0,120,240,90,90,30,30,30)

%%
figure
surf(A1,A2,Z)
xlabel("A1")
ylabel("A2")
zlabel("Z")
grid on
colorbar
set(gcf,'color','w')

%%
n=define_Q(-30,-30,-30)
v1=[-0.4330;-0.7500;0.4999];
v2=[-0.5413;0.5625;-0.6251];
n2=cross(v1,v2)/norm(cross(v1,v2))

%%
theta=-45:1:45;
psi=-45:1:45;
Z=zeros(length(theta),length(psi));
for i=1:length(theta)
    for j=1:length(psi)
        T=Inverse_Kin
        w1=define_wi(alpha1(i),0,120,240,theta1(j),theta2(j),theta3(j),1);
        w2=define_wi(alpha1(i),0,120,240,theta1(j),theta2(j),theta3(j),2);
        w3=define_wi(alpha1(i),0,120,240,theta1(j),theta2(j),theta3(j),3);
        [v1,v2,v3]=forward_kinematics(alpha1(i),0,120,240,alpha2(j),120,theta1(j),theta2(j),theta3(j));
        J=calculate_Jacobian(w1,w2,w3,v1,v2,v3);
        Z(j,i)=conditioning_index(J);
    end
end