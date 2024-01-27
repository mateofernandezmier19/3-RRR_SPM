function [phi,theta,psi,v1,v2,v3,w1,w2,w3] = forward_kinematics2(alpha1,eta11,eta12,eta13,alpha2,gamma2,beta2,theta_1,theta_2,theta_3)
% Implements the forward kinematics algorithm for the SPM with coaxial
% input axes
w1=define_wi(alpha1,eta11,eta12,eta13,theta_1,theta_2,theta_3,1);
w2=define_wi(alpha1,eta11,eta12,eta13,theta_1,theta_2,theta_3,2);
w3=define_wi(alpha1,eta11,eta12,eta13,theta_1,theta_2,theta_3,3);
F=@(v) [w1(1)*v(1)+w1(2)*v(2)+w1(3)*v(3)-cos(deg2rad(alpha2));
        w2(1)*v(4)+w2(2)*v(5)+w2(3)*v(6)-cos(deg2rad(alpha2));
        w3(1)*v(7)+w3(2)*v(8)+w3(3)*v(9)-cos(deg2rad(alpha2));
        v(1)*v(4)+v(2)*v(5)+v(3)*v(6)-cos(deg2rad(gamma2));
        v(1)*v(7)+v(2)*v(8)+v(3)*v(9)-cos(deg2rad(gamma2));
        v(4)*v(7)+v(5)*v(8)+v(6)*v(9)-cos(deg2rad(gamma2));
        (v(1)^2)+(v(2)^2)+(v(3)^2)-1;
        (v(4)^2)+(v(5)^2)+(v(6)^2)-1;
        (v(7)^2)+(v(8)^2)+(v(9)^2)-1];
w1_rot=w1*cos(deg2rad(10))+cross(w1,[0;0;1])*sin(deg2rad(10))+[0;0;1]*(dot([0;0;1],w1))*(1+cos(deg2rad(10)));
w2_rot=w2*cos(deg2rad(10))+cross(w2,[0;0;1])*sin(deg2rad(10))+[0;0;1]*(dot([0;0;1],w2))*(1+cos(deg2rad(10)));
w3_rot=w3*cos(deg2rad(10))+cross(w3,[0;0;1])*sin(deg2rad(10))+[0;0;1]*(dot([0;0;1],w3))*(1+cos(deg2rad(10)));
v0=[w1_rot(1);w1_rot(2);-w1_rot(3);w2_rot(1);w2_rot(2);-w2_rot(3);w3_rot(1);w3_rot(2);-w3_rot(3)];
options=[optimoptions('fsolve','Algorithm','trust-region-dogleg','Display','off')];
v=fsolve(F,v0,options);
v1=v(1:3);
v2=v(4:6);
v3=v(7:9);
if beta2==90
    n=cross(v1,v2)/norm(cross(v1,v2));
else
    n=(v1+v2+v3)/norm(v1+v2+v3);
end
G=@(T) [cos(T(1))*sin(T(3))+sin(T(1))*sin(T(2))*cos(T(3))-n(1);
        sin(T(1))*sin(T(3))-cos(T(1))*sin(T(2))*cos(T(3))-n(2);
        cos(T(2))*cos(T(3))-n(3)];
options2=optimoptions("fsolve","Algorithm","trust-region",'Display','off');
T0=[0;0;0];
T=fsolve(G,T0,options2);
phi=rad2deg(T(1));
theta=rad2deg(T(2));
psi=rad2deg(T(3));
end