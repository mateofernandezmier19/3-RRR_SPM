function [theta_i] = inverse_kinematics(alpha1,eta11,eta12,eta13,alpha2,beta2,phi,theta,psi)
% Implementation of the inverse kinematics algorithm for the SPM with
% coaxial input axes
theta_i=zeros(3,2);
for i=1:3
    v=calculate_vi(phi,theta,psi,beta2,eta11,eta12,eta13,i);
    [A,B,C]=define_coefficients(alpha1,eta11,eta12,eta13,alpha2,v,i);
    t1=(-(2*B)+sqrt(((2*B)^2)-4*A*C))/(2*A);
%     t2=(-(2*B)-sqrt(((2*B)^2)-4*A*C))/(2*A);
%     theta_i(i,:)=[rad2deg(2*atan2(t1,1)),rad2deg(2*atan2(t2,1))];
%     if t(1)<0
%         theta(i)=rad2deg(2*atan2(t(1),1));
%     else
%         theta(i)=rad2deg(2*atan2(t(2),1));
%     end
if isreal(t1)
    theta_i(i)=rad2deg(2*atan2(t1,1));
else
    theta_i(i)=nan;
end
end