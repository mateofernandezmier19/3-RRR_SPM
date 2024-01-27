function [theta_traj,v11,v22,v33,n] = Infinite_Rotational_Motion2(phi,theta,psi,alpha1,alpha2,beta2,eta11,eta12,eta13,eta21,eta22,eta23,sigma)
% This function computes the trajectory (given in a sequence of input
% joint angles) for an infinite rotation of the mobile platform around a
% normal vector n (for a given orientation of the platform determined by
% the angles phi, theta, and psi)
v1=calculate_vi(phi,theta,psi,beta2,eta21,eta22,eta23,1);
v2=calculate_vi(phi,theta,psi,beta2,eta21,eta22,eta23,2);
v3=calculate_vi(phi,theta,psi,beta2,eta21,eta22,eta23,3);
if beta2==90
    n=cross(v1,v2)/norm(cross(v1,v2));
else
    n=(v1+v2+v3)/norm(v1+v2+v3);
end
S=length(sigma);
v11=zeros(3,S);
v22=zeros(3,S);
v33=zeros(3,S);
theta_traj=zeros(3,S);
for j=1:S
    v11(:,j)=v1*cos(deg2rad(sigma(j)))+cross(v1,n)*sin(deg2rad(sigma(j)))+n*(dot(n,v1))*(1-cos(deg2rad(sigma(j))));
    v22(:,j)=v2*cos(deg2rad(sigma(j)))+cross(v2,n)*sin(deg2rad(sigma(j)))+n*(dot(n,v2))*(1-cos(deg2rad(sigma(j))));
    v33(:,j)=v3*cos(deg2rad(sigma(j)))+cross(v3,n)*sin(deg2rad(sigma(j)))+n*(dot(n,v3))*(1-cos(deg2rad(sigma(j))));
    theta_traj(:,j)=inverse_kinematics2(alpha1,eta11,eta12,eta13,alpha2,v11(:,j),v22(:,j),v33(:,j));
end
for k=1:S-1
    if (theta_traj(1,k)-theta_traj(1,k+1))>0
        theta_traj(1,k+1)=theta_traj(1,k+1)+360;
    end
    if (theta_traj(2,k)-theta_traj(2,k+1))>0
        theta_traj(2,k+1)=theta_traj(2,k+1)+360;
    end
    if (theta_traj(3,k)-theta_traj(3,k+1))>0
        theta_traj(3,k+1)=theta_traj(3,k+1)+360;
    end
end
if (theta_traj(3,1)-theta_traj(2,1))>120
    theta_traj(2,:)=theta_traj(2,:)+360;
end
if (theta_traj(2,1)-theta_traj(1,1))>120
    theta_traj(1,:)=theta_traj(1,:)+360;
end
if (theta_traj(1,1)-theta_traj(3,1))>120
    theta_traj(3,:)=theta_traj(3,:)+360;
end
end