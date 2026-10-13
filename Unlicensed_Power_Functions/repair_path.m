function Q=repair_path(target,starts,step,Dmin,area)
[U,N,~]=size(target); Q=zeros(size(target));
for u=1:U
    for n=1:N,Q(u,n,:)=starts(u,:);end
end
if ~path_ok(Q,starts,step,Dmin,area)
    error('The specified stationary starting positions violate UAV separation.');
end
Xraw=reshape(permute(target,[2 3 1]),N,2*U);
X0=reshape(permute(Q,[2 3 1]),N,2*U);
try
    cvx_begin quiet
        variable X(N,2*U)
        minimize(sum_square(reshape(X-Xraw,[],1)))
        subject to
            X(:)>=0; X(:)<=area;
            for u=1:U
                cc=2*u-1:2*u;
                X(1,cc)==starts(u,:);X(N,cc)==starts(u,:);
                for n=1:N-1
                    norm(X(n+1,cc)-X(n,cc),2)<=step;
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
    if contains(lower(cvx_status),'solved')
        proposed=permute(reshape(X,N,2,U),[3 1 2]);
        if path_ok(proposed,starts,step,Dmin,area),Q=proposed;end
    end
catch ME
    warning('Initial trajectory repair: %s',ME.message);
    try,cvx_clear;catch,end
end
end

