function [cap,leakage] = leakage_caps_vec(tx,u,Hprot,GammaVec,bandName)
% Backward-compatible helper.
% In the corrected aggregate version this function is not used for final
% protection,because final protection must consider all UAVs together.
% It is kept only in case another helper calls it.
if strcmpi(bandName,'lic')
    idx = tx(u).idxLic;
    p   = tx(u).pLic(:);
    vList = tx(u).vLic;
else
    idx = tx(u).idxUn;
    p   = tx(u).pUn(:);
    vList = tx(u).vUn;
end
L = size(idx,1);
cap = zeros(L,1);
leakage = zeros(L,1);
for ell = 1:L
    sc = idx(ell,2);
    v  = vList{ell};
    gLeak = abs(Hprot{u,sc}' * v)^2;
    cap(ell) = max(GammaVec(sc) / max(gLeak,1e-30),0);
    leakage(ell) = p(ell) * gLeak;
end
end
