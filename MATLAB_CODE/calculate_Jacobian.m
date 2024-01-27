function [J] = calculate_Jacobian(w1,w2,w3,v1,v2,v3)
% Computes the Jacobian matrix of the manipulator for a given configuration
u=[0;0;-1];
A=[transpose(cross(w1,v1));
   transpose(cross(w2,v2));
   transpose(cross(w3,v3))];
B=diag([dot(cross(u,w1),v1),dot(cross(u,w2),v2),dot(cross(u,w3),v3)]);
J=B\A;
end