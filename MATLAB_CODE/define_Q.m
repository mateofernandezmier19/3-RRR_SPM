function [Q] = define_Q(phi,theta,psi)
% This function calculates a rotation matrix given angles phi, theta, and
% psi
%   Returns the rotation matrix given by a rotation of phi around axis Z, a
%   rotation of angle theta around axis X', and a rotation of angle psi
%   around axis Y''
Rz_phi=[cos(deg2rad(phi)),sin(-deg2rad(phi)),0;sin(deg2rad(phi)),cos(deg2rad(phi)),0;0,0,1];
Rx_theta=[1,0,0;0,cos(deg2rad(theta)),-sin(deg2rad(theta));0,sin(deg2rad(theta)),cos(deg2rad(theta))];
Ry_psi=[cos(deg2rad(psi)),0,sin(deg2rad(psi));0,1,0;-sin(deg2rad(psi)),0,cos(-deg2rad(psi))];
Q=Rz_phi*Rx_theta*Ry_psi;
end