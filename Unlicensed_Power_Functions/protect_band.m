function tx=protect_band(tx,H,G,band,rateFn)
% Rate-aware candidate power reductions on each violating protected channel.
for sc=1:numel(G)
    I=leakage(tx,H,G,band);
    if I(sc)<=G(sc)*(1+1e-10),continue;end
    entries=zeros(0,3);
    for u=1:numel(tx)
        [idx,p,vs]=band_data(tx,u,band);
        for e=1:numel(p)
            if idx(e,2)==sc && p(e)>0
                entries(end+1,:)=[u,e,abs(H{u,sc}'*vs{e})^2]; %#ok<AGROW>
            end
        end
    end
    best=tx;scale=G(sc)/max(I(sc),realmin);
    for z=1:size(entries,1)
        u=entries(z,1);e=entries(z,2);[~,p]=band_data(best,u,band);
        p(e)=p(e)*scale;best=set_p(best,u,band,p);
    end
    bestR=rateFn(best);
    for first=1:size(entries,1)
        trial=tx;order=[first,setdiff(1:size(entries,1),first,'stable')];
        for z=order
            I=leakage(trial,H,G,band);
            if I(sc)<=G(sc)*(1+1e-10),break;end
            u=entries(z,1);e=entries(z,2);g=entries(z,3);
            [~,p]=band_data(trial,u,band);
            p(e)=max(0,p(e)-min(p(e),(I(sc)-G(sc))/max(g,realmin)));
            trial=set_p(trial,u,band,p);
        end
        if leakage(trial,H,G,band)<=G*(1+1e-7)
            r=rateFn(trial);
            if r>bestR,best=trial;bestR=r;end
        end
    end
    tx=best;
end
end
