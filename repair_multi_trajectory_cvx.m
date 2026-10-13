function Qfixed=repair_multi_trajectory_cvx(Qraw,area,stepMax,qStart,qEnd,Dmin_UAV)
% Joint convex projection with a safe collision-free straight-line anchor.
% Collision halfspaces linearized at the anchor are an INNER approximation;
% the safe anchor is a fallback, never an unchecked per-UAV projection.
U=size(Qraw,1); N=size(Qraw,2);
anchor=zeros(U,N,2);
for u=1:U
    if norm(qEnd(u,:)-qStart(u,:))>(N-1)*stepMax+1e-8
        error('UAV %d: endpoints unreachable.',u);
    end
    for n=1:N
        t=(n-1)/(N-1);
        anchor(u,n,:)=(1-t)*qStart(u,:)+t*qEnd(u,:);
    end
end
if ~check_multi_traj_feasibility(anchor,qStart,qEnd,stepMax,area,Dmin_UAV)
    error(['Straight-line collision-free anchor unavailable. Supply a ' ...
           'collision-free initial trajectory for this geometry.']);
end
if check_multi_traj_feasibility(Qraw,qStart,qEnd,stepMax,area,Dmin_UAV)
    Qfixed=Qraw; return;
end
Xraw=reshape(permute(Qraw,[2 3 1]),N,2*U);
X0=reshape(permute(anchor,[2 3 1]),N,2*U);
Qfixed=anchor;
try
    cvx_begin quiet
        variable X(N,2*U)
        minimize(sum_square(reshape(X-Xraw,N*2*U,1)))
        subject to
            X(:)>=0; X(:)<=area;
            for u=1:U
                cols=2*u-1:2*u;
                X(1,cols)==qStart(u,:);
                X(N,cols)==qEnd(u,:);
                for n=1:N-1
                    norm(X(n+1,cols)-X(n,cols),2)<=stepMax;
                end
            end
            for n=1:N
                for u=1:U-1
                    for v=u+1:U
                        cu=2*u-1:2*u; cv=2*v-1:2*v;
                        d0=X0(n,cu)-X0(n,cv);
                        2*d0*(X(n,cu)-X(n,cv)).'-sum(d0.^2)>=Dmin_UAV^2;
                    end
                end
            end
    cvx_end
    if contains(lower(cvx_status),'solved') && all(isfinite(X(:)))
        candidate=permute(reshape(X,N,2,U),[3 1 2]);
        if check_multi_traj_feasibility(candidate,qStart,qEnd,stepMax,area,Dmin_UAV)
            Qfixed=candidate;
        else
            warning('Joint collision-safe repair failed feasibility check; using safe anchor.');
        end
    else
        warning('Joint collision-safe repair status %s; using safe anchor.',cvx_status);
    end
catch ME
    warning('Joint collision-safe repair: %s; using safe anchor.',ME.message);
    try,cvx_clear;catch,end
end
assert(check_multi_traj_feasibility(Qfixed,qStart,qEnd,stepMax,area,Dmin_UAV), ...
    'Unable to construct a collision-free feasible path.');
end
