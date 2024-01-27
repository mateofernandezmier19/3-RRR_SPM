function [Ai,Bi,Ci] = define_coefficients(alpha1,eta11,eta12,eta13,alpha2,vi,i)
% Computes the coefficients for the quadratic equation used in the inverse
% kinematics algorithm for the SPM with coaxial inputs axes
if i==1
    Ai=-cos(deg2rad(eta11))*sin(deg2rad(alpha1))*vi(1)-sin(deg2rad(eta11))*sin(deg2rad(alpha1))*vi(2)-cos(deg2rad(alpha1))*vi(3)-cos(deg2rad(alpha2));
    Bi=sin(deg2rad(eta11))*sin(deg2rad(alpha1))*vi(1)-cos(deg2rad(eta11))*sin(deg2rad(alpha1))*vi(2);
    Ci=cos(deg2rad(eta11))*sin(deg2rad(alpha1))*vi(1)+sin(deg2rad(eta11))*sin(deg2rad(alpha1))*vi(2)-cos(deg2rad(alpha1))*vi(3)-cos(deg2rad(alpha2));
elseif i==2
    Ai=-cos(deg2rad(eta12))*sin(deg2rad(alpha1))*vi(1)-sin(deg2rad(eta12))*sin(deg2rad(alpha1))*vi(2)-cos(deg2rad(alpha1))*vi(3)-cos(deg2rad(alpha2));
    Bi=sin(deg2rad(eta12))*sin(deg2rad(alpha1))*vi(1)-cos(deg2rad(eta12))*sin(deg2rad(alpha1))*vi(2);
    Ci=cos(deg2rad(eta12))*sin(deg2rad(alpha1))*vi(1)+sin(deg2rad(eta12))*sin(deg2rad(alpha1))*vi(2)-cos(deg2rad(alpha1))*vi(3)-cos(deg2rad(alpha2));
elseif i==3
    Ai=-cos(deg2rad(eta13))*sin(deg2rad(alpha1))*vi(1)-sin(deg2rad(eta13))*sin(deg2rad(alpha1))*vi(2)-cos(deg2rad(alpha1))*vi(3)-cos(deg2rad(alpha2));
    Bi=sin(deg2rad(eta13))*sin(deg2rad(alpha1))*vi(1)-cos(deg2rad(eta13))*sin(deg2rad(alpha1))*vi(2);
    Ci=cos(deg2rad(eta13))*sin(deg2rad(alpha1))*vi(1)+sin(deg2rad(eta13))*sin(deg2rad(alpha1))*vi(2)-cos(deg2rad(alpha1))*vi(3)-cos(deg2rad(alpha2));
else
    Ai=0;
    Bi=0;
    Ci=0;
end
end