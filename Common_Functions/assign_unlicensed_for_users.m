function rhoU = assign_unlicensed_for_users(hu2U,hw2,h_uw2,P_WIFI_m,sigma2_un,GammaUn,Pmax_un,Bsc_un,K,M,needUn,allocMode)
if nargin < 12 || isempty(allocMode)
    allocMode = 'hungarian';
end
rhoU = zeros(K,M);
activeUsers = find(needUn(:));
if isempty(activeUsers) || Pmax_un <= 0
    return;
end
Ka = numel(activeUsers);
RmatU = zeros(Ka,M);
for aa = 1:Ka
    k = activeUsers(aa);
    for m = 1:M
        p_app = min(Pmax_un/max(M,1),GammaUn(m)/max(h_uw2(m),1e-30));
        denom = max(hw2(k)*P_WIFI_m(m) + sigma2_un,1e-30);
        SINR  = (hu2U(k)*p_app)/denom;
        RmatU(aa,m) = Bsc_un*log2(1 + max(SINR,0));
    end
end
if strcmpi(allocMode,'greedy')
    pairsLocal = assign_greedy_max(RmatU);
else
    pairsLocal = assign_hungarian_max(RmatU);
end
for ii = 1:size(pairsLocal,1)
    aa = pairsLocal(ii,1);
    m  = pairsLocal(ii,2);
    rhoU(activeUsers(aa),m) = 1;
end
end
