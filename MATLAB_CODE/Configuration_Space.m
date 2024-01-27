function [V] = Configuration_Space(alpha1,alpha2,gamma2,eta11,eta12,eta13,T1,T2,T3,zeta_min)
% This function numerically computes the configuration space of the coaxial
% SPM taking into account link surpass, singularity and colision detection
M1=length(T1);
M2=length(T2);
M3=length(T3);
V=[];
for a1=1:M1
    for a2=1:M2
        for a3=1:M3
            singularity=0;
            collision=0;
            if ((T3(a3)-T2(a2))>120) || ((T2(a2)-T1(a1))>120) || ((T1(a1)-T3(a3))>120)
                collision=1;
            end
            theta1=T1(a1);
            theta2=T2(a2);
            theta3=T3(a3);
            [v1,v2,v3,w1,w2,w3]=forward_kinematics(alpha1,eta11,eta12,eta13,alpha2,gamma2,theta1,theta2,theta3);
            J=calculate_Jacobian(w1,w2,w3,v1,v2,v3);
            zeta=conditioning_index(J);
            if zeta<zeta_min
                singularity=1;
            end
            if (singularity==0)&&(collision==0)
                theta=[theta1;theta2;theta3];
                V=[V,theta];
            end                
        end
    end
end
end