function ok=path_ok(Q,starts,step,Dmin,area)
[U,N,~]=size(Q); tol=1e-3;ok=all(isfinite(Q(:))) && all(Q(:)>=-tol) && all(Q(:)<=area+tol);
for u=1:U
    p=squeeze(Q(u,:,:));
    ok=ok && norm(p(1,:)-starts(u,:))<tol && norm(p(end,:)-starts(u,:))<tol;
    ok=ok && all(sqrt(sum(diff(p).^2,2))<=step+tol);
end
for n=1:N
    for u=1:U-1
        for v=u+1:U
            ok=ok && norm(squeeze(Q(u,n,:)-Q(v,n,:)))>=Dmin-tol;
        end
    end
end
end

