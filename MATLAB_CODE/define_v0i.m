function [v0i] = define_v0i(beta2,eta21,eta22,eta23,i)
% This function calculates the vector v0i (i=1,2,3) from the geometric
% parameters of the SPM
if i==1
    v0i=[sin(deg2rad(eta21))*sin(deg2rad(beta2));-cos(deg2rad(eta21))*sin(deg2rad(beta2));-cos(deg2rad(beta2))];
elseif i==2
    v0i=[sin(deg2rad(eta22))*sin(deg2rad(beta2));-cos(deg2rad(eta22))*sin(deg2rad(beta2));-cos(deg2rad(beta2))];
elseif i==3
    v0i=[sin(deg2rad(eta23))*sin(deg2rad(beta2));-cos(deg2rad(eta23))*sin(deg2rad(beta2));-cos(deg2rad(beta2))];
end
end