function [cond_index] = conditioning_index(J)
% Calculates the local dexterity of the SPM at a given configuration as the
% conditioning index of the Jacobian matrix of the SPM at that
% configuration
k=Euclidean_norm(J)*Euclidean_norm(inv(J));
cond_index=1/k;
end