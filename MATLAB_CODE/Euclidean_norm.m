function [norm] = Euclidean_norm(A)
% Computes the Euclidean norm of a matrix A
W=(1/3)*eye(3);
TR=transpose(A)*W*A;
norm=sqrt(trace(TR));
end