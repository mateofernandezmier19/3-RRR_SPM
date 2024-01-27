function [vi] = calculate_vi(phi,theta,psi,beta2,eta21,eta22,eta23,i)
% Calculates the vector vi given a orientation of the mobile platform
Q=define_Q(phi,theta,psi);
v0i=define_v0i(beta2,eta21,eta22,eta23,i);
vi=Q*v0i;
end