function rhoL = assign_licensed_original(hu2L,hp2,h_up2,P_PBS_j,sigma2_lic,GammaLic,Pmax_lic,Bsc_lic,K,J,allocMode)
if nargin < 11 || isempty(allocMode)
    allocMode = 'hungarian';
end
RmatL = zeros(K,J);
for k = 1:K
    for j = 1:J
        p_app = min(Pmax_lic/max(J,1),GammaLic(j)/max(h_up2(j),1e-30));
        denom = max(hp2(k)*P_PBS_j(j) + sigma2_lic,1e-30);
        SINR  = (hu2L(k)*p_app)/denom;
        RmatL(k,j) = Bsc_lic*log2(1 + max(SINR,0));
    end
end
if strcmpi(allocMode,'greedy')
    pairsL = assign_greedy_max(RmatL);
else
    pairsL = assign_hungarian_max(RmatL);
end
rhoL = zeros(K,J);
for ii = 1:size(pairsL,1)
    rhoL(pairsL(ii,1),pairsL(ii,2)) = 1;
end
end
