function p = kkt_waterfill_capped_original(a,Ptot,pcap)
L = numel(a);
p = zeros(L,1);
if L==0 || Ptot<=0,return; end
a = max(a(:),1e-30);
pcap = max(pcap(:),0);
if isfinite(sum(pcap)) && sum(pcap) <= Ptot
    p = pcap;
    return;
end
lam_lo = 1e-12;
lam_hi = 1e12;
for it = 1:100
    lam = 0.5*(lam_lo + lam_hi);
    p_try = max(0,1./lam - 1./a);
    p_try = min(p_try,pcap);
    if sum(p_try) > Ptot
        lam_lo = lam;
    else
        lam_hi = lam;
    end
end
p = max(0,1./lam_hi - 1./a);
p = min(p,pcap);
s = sum(p);
if s > Ptot && s > 0
    p = p * (Ptot/s);
end
end
