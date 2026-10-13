function [Qout,ok]=sca_path(Q,grad,starts,step,Dmin,area,radius)
[U,N,~]=size(Q);X0=reshape(permute(Q,[2 3 1]),N,2*U);
G=reshape(permute(grad,[2 3 1]),N,2*U);scale=max(1,max(abs(G(:))));
Qout=Q;ok=false;
try
    cvx_begin quiet
        variable X(N,2*U)
        maximize(sum(sum((G/scale).*(X-X0))) - ...
            1e-4/max(radius^2,1e-9)*sum_square(reshape(X-X0,[],1)))
        subject to
            X(:)>=0;X(:)<=area;
            for u=1:U
                cc=2*u-1:2*u;
                X(1,cc)==starts(u,:);X(N,cc)==starts(u,:);
                for n=1:N-1
                    norm(X(n+1,cc)-X(n,cc),2)<=step;
                end
                for n=2:N-1
                    norm(X(n,cc)-X0(n,cc),2)<=radius;
                end
            end
            for n=1:N
                for u=1:U-1
                    for v=u+1:U
                        cu=2*u-1:2*u;cv=2*v-1:2*v;
                        d=X0(n,cu)-X0(n,cv);
                        2*d*(X(n,cu)-X(n,cv)).'-sum(d.^2)>=Dmin^2;
                    end
                end
            end
    cvx_end
    ok=contains(lower(cvx_status),'solved') && all(isfinite(X(:)));
    if ok,Qout=permute(reshape(X,N,2,U),[3 1 2]);end
catch ME
    warning('Trajectory SCA: %s',ME.message);
    try,cvx_clear;catch,end
end
end

