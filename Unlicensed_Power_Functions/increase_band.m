function tx=increase_band(tx,H,G,Pmax,band,rateFn)
for pass=1:2
    changed=false;
    for u=1:numel(tx)
        [idx,p,vs]=band_data(tx,u,band);
        for e=1:numel(p)
            sc=idx(e,2);I=leakage(tx,H,G,band);g=abs(H{u,sc}'*vs{e})^2;
            extra=min(max(0,Pmax-sum(p)),max(0,G(sc)-I(sc))/max(g,realmin));
            if extra<=1e-14,continue;end
            base=rateFn(tx);bestP=p(e);
            for x=[0,0.25,0.5,0.75,1]*extra
                cand=tx;pp=p;pp(e)=pp(e)+x;cand=set_p(cand,u,band,pp);
                r=rateFn(cand);
                if r>base+1e-7,base=r;bestP=pp(e);end
            end
            if bestP>p(e),p(e)=bestP;tx=set_p(tx,u,band,p);changed=true;end
        end
    end
    if ~changed,break;end
end
end

