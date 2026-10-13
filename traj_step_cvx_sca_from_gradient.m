function [q_next,solved] = traj_step_cvx_sca_from_gradient(q0,qPrev,qNext,...
    gradF,L0,gradL,stepMax,area,radius)
% Affine numerical first-order models of objective and normalized leakage.
% Fixed neighboring positions preserve the entire trajectory's mobility.
q_next=q0; solved=false;
if exist('cvx_begin','file')~=2
    error('CVX is required.');
end
try
    cvx_begin quiet
        variable q(2)
        maximize(gradF(:).'*(q-q0(:)))
        subject to
            norm(q-q0(:),2) <= radius;
            norm(q-qPrev(:),2) <= stepMax;
            norm(q-qNext(:),2) <= stepMax;
            q>=0;
            q<=area;
            L0(:)+gradL*(q-q0(:))<=1;
    cvx_end
    solved=contains(lower(cvx_status),'solved') && all(isfinite(q));
    if solved
        q_next=q(:).';
    end
catch ME
    warning('CVX trajectory subproblem failed: %s',ME.message);
    try,cvx_clear;catch,end
end
end
%% Old unused helper retained below; main trajectory update now uses CVX/SCA.
