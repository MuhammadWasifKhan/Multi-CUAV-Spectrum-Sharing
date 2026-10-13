function [HL,HU,HP,HW,hp,hw]=make_channels(Q,W,P,V,sh,bs,ap,H,fl,fu,c,A,Nt,seed)
U=size(Q,1);K=size(W,1);J=size(P,1);M=size(V,1);
HL=cell(U,K);HU=cell(U,K);HP=cell(U,J);HW=cell(U,M);
rng(seed+555,'twister');hp=zeros(K,1);hw=zeros(K,1);
for k=1:K
    d=max(norm(W(k,:)-bs),1);pl=20*log10(4*pi*fl/c)+37*log10(d)+sh.pbs(k);
    hp(k)=10^(-pl/10)*abs((randn+1i*randn)/sqrt(2))^2;
    d=max(norm(W(k,:)-ap),1);pl=20*log10(4*pi*fu/c)+30*log10(d)+sh.wifi(k);
    hw(k)=10^(-pl/10)*abs((randn+1i*randn)/sqrt(2))^2;
end
for u=1:U
    rng(seed+10000*u,'twister');q=Q(u,:);
    for k=1:K
        HL{u,k}=channel(gain(q,W(k,:),H,fl,c,A,sh.su(k)),Nt);
        HU{u,k}=channel(gain(q,W(k,:),H,fu,c,A,sh.su(k)),Nt);
    end
    for j=1:J,HP{u,j}=channel(gain(q,P(j,:),H,fl,c,A,sh.pu(j)),Nt);end
    for m=1:M,HW{u,m}=channel(gain(q,V(m,:),H,fu,c,A,sh.wu(m)),Nt);end
end
end

