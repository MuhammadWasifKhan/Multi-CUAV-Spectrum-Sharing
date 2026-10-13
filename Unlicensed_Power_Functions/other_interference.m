function v=other_interference(tx,H,u,k,sc,band)
v=0;
for t=1:numel(tx)
    if t==u,continue;end
    [idx,p,vs]=band_data(tx,t,band);
    for e=1:numel(p)
        if idx(e,2)==sc,v=v+p(e)*abs(H{t,k}'*vs{e})^2;end
    end
end
end
