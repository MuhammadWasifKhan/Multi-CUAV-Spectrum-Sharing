function [v,gSig,gLeak] = mm_unit_beam_unprotected(hSU,denom,MM_iter,alphaFallback)
global AIIBPC_BEAM_CALLS;
if ~isempty(AIIBPC_BEAM_CALLS), AIIBPC_BEAM_CALLS=AIIBPC_BEAM_CALLS+1; end
% Initial CBF-MM surrogate WITHOUT PU/WU interference penalties.
% The rank-one desired-signal surrogate converges to MRT.
Nt=numel(hSU); B=(hSU*hSU')/max(denom,1e-30)+alphaFallback*eye(Nt);
v=ones(Nt,1)/sqrt(Nt);
for it=1:max(1,MM_iter)
    w=B*v; vNew=w/max(norm(w),1e-30);
    if norm(vNew-v)<1e-8, v=vNew; break; end
    v=vNew;
end
gSig=abs(hSU'*v)^2;
gLeak=0; % Protection-channel leakage is measured only after allocation.
end


