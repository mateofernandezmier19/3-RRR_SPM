function [W,N,nn] = Workspace1(alpha1,eta11,eta12,eta13,alpha2,beta2,gamma2,phi,theta,psi,zeta_min)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
W=[];
N=[];
nn=[];
for i=1:length(phi)
    for j=1:length(theta)
        for k=1:length(psi)
            singularity=0;
            T=inverse_kinematics(alpha1,eta11,eta12,eta13,alpha2,beta2,phi(i),theta(j),psi(k));
            if (isnan(T(1)) || isnan(T(2)) || isnan(T(3)))
                continue
            else
                w=[phi(i);theta(j);psi(k)];
                Q=define_Q(phi(i),theta(j),psi(k));
                n=Q(:,3);
                W=[W,w];
                N=[N,n];
                [v1,v2,v3,w1,w2,w3]=forward_kinematics(alpha1,eta11,eta12,eta13,alpha2,gamma2,T(1),T(2),T(3));
                J=calculate_Jacobian(w1,w2,w3,v1,v2,v3);
                zeta=conditioning_index(J);
                if zeta<zeta_min
                    singularity=1;
                end
                if singularity==1
                    nn=[nn,n];
                end
            end
        end
    end
end
end