function I=leakage(tx,H,G,band)
I=zeros(numel(G),1);
for u=1:numel(tx)
    [idx,p,vs]=band_data(tx,u,band);
    for e=1:numel(p)
        sc=idx(e,2);I(sc)=I(sc)+p(e)*abs(H{u,sc}'*vs{e})^2;
    end
end
end
