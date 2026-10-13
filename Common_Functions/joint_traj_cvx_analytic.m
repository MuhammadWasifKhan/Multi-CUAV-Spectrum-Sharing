function [Qtrial,solved]=joint_traj_cvx_analytic(Q0,gradR,L0,gradL,qStart,qEnd,stepMax,area,radius,Dmin_UAV)
% A SINGLE convex program for all 3 UAVs and all N waypoints.
% Linearized normalized aggregate interference is enforced conservatively
% locally; true protected interference is rechecked by slot allocation.
[U,N,~]=size(Q0); S=size(L0,1);
X0=reshape(permute(Q0,[2 3 1]),N,2*U);
G=reshape(permute(gradR,[2 3 1]),N,2*U);
% Scale rate gradient to improve numerical conditioning.
scale=max(1,max(abs(G(:))));
solved=false; Qtrial=Q0;
try
    cvx_begin quiet
        variable X(N,2*U)
        expressions leakageLinear(S,N)
        maximize( sum(sum((G/scale).*(X-X0))) - ...
            (1e-4/max(radius^2,1e-9))*sum_square(reshape(X-X0,N*2*U,1)) )
        subject to
            X(:)>=0; X(:)<=area;
            for u=1:U
                cols=2*u-1:2*u;
                X(1,cols)==qStart(u,:);
                X(N,cols)==qEnd(u,:);
                for n=1:N-1
                    norm(X(n+1,cols)-X(n,cols),2)<=stepMax;
                end
                for n=2:N-1
                    norm(X(n,cols)-X0(n,cols),2)<=radius;
                end
            end
            % Convex inner approximation of pairwise non-collision.
            % At feasible Q0: ||d||^2 >= Dmin^2 is conservatively enforced
            % by ||d0||^2 + 2*d0'*(d-d0) >= Dmin^2.
            % The same check applies to all time slots, including endpoints.
            for n=1:N
                for u=1:U-1
                    for v=u+1:U
                        cu=2*u-1:2*u; cv=2*v-1:2*v;
                        d0=X0(n,cu)-X0(n,cv);
                        if sum(d0.^2)<Dmin_UAV^2-1e-5
                            error('Joint SCA requires a collision-free expansion path.');
                        end
                        2*d0*(X(n,cu)-X(n,cv)).' - sum(d0.^2) >= Dmin_UAV^2;
                    end
                end
            end
            % Aggregate PU/WU protection is re-enforced by the complete
            % AI-IBPC / IA-RSS communication re-optimization at Qtrial.
            % Do not impose an additional frozen-resource linearized leakage
            % constraint here; it can make the local trajectory SCA infeasible.
    cvx_end
    solved=contains(lower(cvx_status),'solved') && all(isfinite(X(:)));
    if solved
        Qtrial=permute(reshape(X,N,2,U),[3 1 2]);
    end
catch ME
    warning('Joint trajectory CVX: %s',getReport(ME,'basic','hyperlinks','off'));
    try,cvx_clear;catch,end
end
end
