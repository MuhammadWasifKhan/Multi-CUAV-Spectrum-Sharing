function r=slot_solver(Q,mode,W,P,V,sh,bs,ap,H,fl,fu,c,A, ...
    PP,PW,nL,nU,GL,GU,PL,PU,BL,BU,Nt,mmIter,AO,active,reserve,seed)
[HL,HU,HP,HW,hp,hw]=make_channels(Q,W,P,V,sh,bs,ap,H,fl,fu,c,A,Nt,seed);
U=size(Q,1);K=size(W,1);
score=zeros(U,K);
for u=1:U
    for k=1:K,score(u,k)=norm(HL{u,k})^2;end
end
[~,assoc]=max(score,[],1);
tx=repmat(struct('users',[],'idxLic',zeros(0,2),'pLic',[], ...
    'vLic',{{}},'idxUn',zeros(0,2),'pUn',[],'vUn',{{}}),U,1);
for u=1:U,tx(u).users=find(assoc==u);end
best=tx;bestRate=0;previous=tx;
for it=1:max(1,AO)
    tx=previous;
    for u=1:U
        us=tx(u).users;
        if isempty(us),continue;end
        for b=1:2
            if b==1,HS=HL;HI=HP;hh=hp;pi=PP;noise=nL;power=PL;band='lic';bw=BL;
            else,HS=HU;HI=HW;hh=hw;pi=PW;noise=nU;power=PU;band='un';bw=BU;end
            C=zeros(numel(us),numel(active));
            for k=1:numel(us)
                for s=1:numel(active)
                    sc=active(s);v=mrt(HS{u,us(k)});
                    den=noise+hh(us(k))*pi(sc)+other_interference(previous,HS,u,us(k),sc,band);
                    C(k,s)=bw*log2(1+(power/max(numel(us),1))*abs(HS{u,us(k)}'*v)^2/max(den,realmin));
                end
            end
            pairs=hungarian_max(C);idx=zeros(size(pairs,1),2); beams=cell(size(pairs,1),1);g=zeros(size(pairs,1),1);
            for e=1:size(pairs,1)
                k=pairs(e,1);sc=active(pairs(e,2));idx(e,:)=[k,sc];
                % Initial unprotected MM surrogate converges to MRT.
                beams{e}=mm_unprotected(HS{u,us(k)},mmIter);
                den=noise+hh(us(k))*pi(sc)+other_interference(previous,HS,u,us(k),sc,band);
                g(e)=abs(HS{u,us(k)}'*beams{e})^2/max(den,realmin);
            end
            pow=waterfill(g,power,power*ones(numel(g),1));
            if b==1,tx(u).idxLic=idx;tx(u).vLic=beams;tx(u).pLic=pow;
            else,tx(u).idxUn=idx;tx(u).vUn=beams;tx(u).pUn=pow;end
        end
    end
    rateFn=@(z)network_rate(z,HL,HU,hp,hw,PP,PW,nL,nU,BL,BU);
    if strcmp(mode,'AI')
        controlled=ai_fourcase(tx,HP,HW,GL,GU,PL,PU,rateFn);
    else
        controlled=rss_fourcase(tx,HL,HU,HP,HW,GL,GU,PL,PU, ...
            hp,hw,PP,PW,nL,nU,reserve,mmIter,rateFn);
    end
    rNew=rateFn(controlled);
    if ~isfinite(rNew) || rNew<bestRate-1e-8*max(1,bestRate),break;end
    best=controlled;previous=controlled;bestRate=rNew;
end
r=bestRate;
end

