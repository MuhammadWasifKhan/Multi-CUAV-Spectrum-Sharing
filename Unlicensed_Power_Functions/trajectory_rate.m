function r=trajectory_rate(Q,mode,W,P,V,sh,bs,ap,H,fl,fu,c,A, ...
    PP,PW,nL,nU,GL,GU,PL,PU,BL,BU,Nt,mmIter,AO,active,reserve,seed)
N=size(Q,2);r=0;
for n=1:N
    r=r+slot_solver(squeeze(Q(:,n,:)),mode,W,P,V,sh,bs,ap,H,fl,fu,c,A, ...
        PP,PW,nL,nU,GL,GU,PL,PU,BL,BU,Nt,mmIter,AO,active,reserve,seed+n)/N;
end
end

