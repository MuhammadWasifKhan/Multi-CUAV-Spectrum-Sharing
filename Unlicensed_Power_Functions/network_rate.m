function r=network_rate(tx,HL,HU,hp,hw,PP,PW,nL,nU,BL,BU)
r=0;
for u=1:numel(tx)
    for b=1:2
        if b==1,H=HL;hh=hp;Pi=PP;noise=nL;B=BL;band='lic';
        else,H=HU;hh=hw;Pi=PW;noise=nU;B=BU;band='un';end
        [idx,p,vs]=band_data(tx,u,band);
        for e=1:numel(p)
            k=tx(u).users(idx(e,1));sc=idx(e,2);
            sig=p(e)*abs(H{u,k}'*vs{e})^2;
            den=noise+hh(k)*Pi(sc)+other_interference(tx,H,u,k,sc,band);
            r=r+B*log2(1+sig/max(den,realmin));
        end
    end
end
end
