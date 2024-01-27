function [wi] = define_wi(alpha1,eta11,eta12,eta13,theta_1,theta_2,theta_3,i)
% Computes the vector wi give a set of input joint angles theta1, theta2,
% and theta3
theta=[theta_1,theta_2,theta_3];
eta1=[eta11,eta12,eta13];
wi=[cos(deg2rad(eta1(i)-theta(i)))*sin(deg2rad(alpha1));
    sin(deg2rad(eta1(i)-theta(i)))*sin(deg2rad(alpha1));
    -cos(deg2rad(alpha1))];
end