function ok=check_multi_traj_feasibility(Q,qStart,qEnd,stepMax,area,Dmin_UAV)
ok=true;
for u=1:size(Q,1)
    X=squeeze(Q(u,:,:));
    tolFeas=1e-3; % Numerical CVX feasibility tolerance only (meters).
    ok=ok && norm(X(1,:)-qStart(u,:))<tolFeas && ...
        norm(X(end,:)-qEnd(u,:))<tolFeas && ...
        all(X(:)>=-tolFeas) && all(X(:)<=area+tolFeas) && ...
        all(sqrt(sum(diff(X,1,1).^2,2))<=stepMax+tolFeas);
end
if nargin>=6 && Dmin_UAV>0
    for n=1:size(Q,2)
        for u=1:size(Q,1)-1
            for v=u+1:size(Q,1)
                duv=squeeze(Q(u,n,:)-Q(v,n,:));
                ok=ok && norm(duv,2)>=Dmin_UAV-tolFeas;
            end
        end
    end
end
end
