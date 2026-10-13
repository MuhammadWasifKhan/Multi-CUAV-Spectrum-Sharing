function Q=trajectory_ao(Q,mode,W,P,V,sh,bs,ap,H,fl,fu,c,A, ...
    PP,PW,nL,nU,GL,GU,PL,PU,BL,BU,Nt,mmIter,AO, ...
    active,reserve,starts,step,Dmin,area,maxIter,seed)
% Joint all-waypoint CVX trust-region SCA; common random numbers for trials.
[U,N,~]=size(Q);
f=@(Z)trajectory_rate(Z,mode,W,P,V,sh,bs,ap,H,fl,fu,c,A, ...
    PP,PW,nL,nU,GL,GU,PL,PU,BL,BU,Nt,mmIter,AO,active,reserve,seed);
current=f(Q);
for it=1:maxIter
    grad=zeros(U,N,2); epsFD=1;
    % Local finite-difference surrogate of REOPTIMIZED sum-rate.
    % This replaces unused analytical leakage derivatives from the old file.
    for n=2:N-1
        for u=1:U
            for xy=1:2
                qa=Q; qb=Q;
                qa(u,n,xy)=qa(u,n,xy)+epsFD;
                qb(u,n,xy)=qb(u,n,xy)-epsFD;
                sa=slot_solver(squeeze(qa(:,n,:)),mode,W,P,V,sh,bs,ap, ...
                    H,fl,fu,c,A,PP,PW,nL,nU,GL,GU,PL,PU,BL,BU,Nt,mmIter,AO,active,reserve,seed+n);
                sb=slot_solver(squeeze(qb(:,n,:)),mode,W,P,V,sh,bs,ap, ...
                    H,fl,fu,c,A,PP,PW,nL,nU,GL,GU,PL,PU,BL,BU,Nt,mmIter,AO,active,reserve,seed+n);
                grad(u,n,xy)=(sa-sb)/(2*epsFD);
            end
        end
    end
    accepted=false; radius=7.5;
    for bt=1:3
        [candidate,solved]=sca_path(Q,grad,starts,step,Dmin,area,radius);
        if solved && path_ok(candidate,starts,step,Dmin,area)
            trial=f(candidate);
            if isfinite(trial) && trial>=current-1e-8*max(1,abs(current))
                if abs(trial-current)<=1e-5*max(1,abs(current))
                    return;
                end
                Q=candidate;current=trial;accepted=true;break;
            end
        end
        radius=radius/2;
    end
    if ~accepted,break;end
end
end

