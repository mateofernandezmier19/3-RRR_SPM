function [theta] = inverse_kinematics2(alpha1,eta11,eta12,eta13,alpha2,v1,v2,v3)
% Implementation of the inverse kinematics algorithm for the SPM with
% coaxial input axes
v=[v1,v2,v3];
theta=zeros(3,1);
for i=1:3
    [A,B,C]=define_coefficients(alpha1,eta11,eta12,eta13,alpha2,v(:,i),i);
    t1=(-(2*B)+sqrt(((2*B)^2)-4*A*C))/(2*A);
%     if t(1)<0
%         theta(i)=rad2deg(2*atan2(t(1),1));
%     else
%         theta(i)=rad2deg(2*atan2(t(2),1));
%     end
    theta(i)=rad2deg(2*atan2(t1,1));
end
end