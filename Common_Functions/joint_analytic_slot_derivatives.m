function [gradRate,L0,gradLeak]=joint_analytic_slot_derivatives(Q,tx,WS,WP,WW,w_b,w_wifi,H,...
    fL,fU,fP,fW,c,A2G,nP,nW,Shadow,Ppbs,Pwifi,sigmaL,sigmaU,GammaL,GammaU,...
    seed,useRayleigh,useShadow,useShadowA2G,Nt,BscL,BscU)
% Exact derivative of frozen-resource, fixed-fading SINR w.r.t. positions.
% h(q)=h(q0)*sqrt(g(q)/g(q0)) under common small-scale fading.
[HL,HU,HP,HW,hp2,hw2]=gen_multi_channels_vec(Q,WS,WP,WW,w_b,w_wifi,H,...
    fL,fU,fP,fW,c,A2G,nP,nW,Shadow,seed,useRayleigh,useShadow,useShadowA2G,Nt);
U=size(Q,1); S=numel(GammaL)+numel(GammaU);
gradRate=zeros(U,2); L0=zeros(S,1); gradLeak=zeros(S,U,2);
for band=1:2
    if band==1
        HS=HL; HT=HP; W=WP; f=fL; hI=hp2; PI=Ppbs; noise=sigmaL;
        gamma=GammaL; offset=0; bw='lic'; Bsc=BscL;
    else
        HS=HU; HT=HW; W=WW; f=fU; hI=hw2; PI=Pwifi; noise=sigmaU;
        gamma=GammaU; offset=numel(GammaL); bw='un'; Bsc=BscU;
    end
    K=size(WS,1); gainGrad=zeros(U,K,2);
    for u=1:U
        for k=1:K
            [~,dg]=a2g_gain_and_gradient(Q(u,:),WS(k,:),H,f,c,A2G,...
                useShadowA2G,Shadow.a2g_su_z(k));
            gainGrad(u,k,:)=dg;
        end
        [idx,p,vList]=get_band_fields(tx,u,bw);
        for ell=1:size(idx,1)
            sc=idx(ell,2);
            if sc<1 || sc>numel(gamma),continue;end
            [~,dg]=a2g_gain_and_gradient(Q(u,:),W(sc,:),H,f,c,A2G,...
                useShadowA2G,ternary_shadow_z(Shadow,sc,band));
            term=p(ell)*abs(HT{u,sc}'*vList{ell})^2;
            L0(offset+sc)=L0(offset+sc)+term/max(gamma(sc),realmin);
            gradLeak(offset+sc,u,:)=reshape(squeeze(gradLeak(offset+sc,u,:)).' +...
                term/max(gamma(sc),realmin)*dg,[1,1,2]);
        end
    end
    for u=1:U
        [idx,p,vList]=get_band_fields(tx,u,bw);
        for ell=1:size(idx,1)
            k=tx(u).users(idx(ell,1)); sc=idx(ell,2);
            if sc<1 || sc>numel(PI),continue;end
            sig=p(ell)*abs(HS{u,k}'*vList{ell})^2;
            den=noise+hI(k)*PI(sc);
            denGrad=zeros(U,2);
            for v=1:U
                if v==u,continue;end
                [idxV,pV,vV]=get_band_fields(tx,v,bw);
                for ee=1:size(idxV,1)
                    if idxV(ee,2)==sc
                        inter=pV(ee)*abs(HS{v,k}'*vV{ee})^2;
                        den=den+inter;
                        denGrad(v,:)=denGrad(v,:)+inter*squeeze(gainGrad(v,k,:)).';
                    end
                end
            end
            den=max(den,1e-30);
            coeff=1/(log(2)*(den+sig));
            gradRate(u,:)=gradRate(u,:)+sig*coeff*squeeze(gainGrad(u,k,:)).'*Bsc;
            for v=1:U
                if v~=u
                    gradRate(v,:)=gradRate(v,:)-sig/den*coeff*denGrad(v,:)*Bsc;
                end
            end
        end
    end
end
end
