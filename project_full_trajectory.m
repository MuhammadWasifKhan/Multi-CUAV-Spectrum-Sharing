function Q = project_full_trajectory(Q,area,stepMax,q_start,q_end,projIter)
N = size(Q,1);
Q(:,1) = min(max(Q(:,1),0),area);
Q(:,2) = min(max(Q(:,2),0),area);
Q(1,:) = q_start;
Q(N,:) = q_end;
for it = 1:projIter
    Q(1,:) = q_start;
    for n = 2:N
        d = Q(n,:) - Q(n-1,:);
        nd = norm(d);
        if nd > stepMax
            Q(n,:) = Q(n-1,:) + stepMax * d / max(nd,1e-12);
        end
        Q(n,1) = min(max(Q(n,1),0),area);
        Q(n,2) = min(max(Q(n,2),0),area);
    end
    Q(N,:) = q_end;
    for n = N-1:-1:1
        d = Q(n,:) - Q(n+1,:);
        nd = norm(d);
        if nd > stepMax
            Q(n,:) = Q(n+1,:) + stepMax * d / max(nd,1e-12);
        end
        Q(n,1) = min(max(Q(n,1),0),area);
        Q(n,2) = min(max(Q(n,2),0),area);
    end
    Q(1,:) = q_start;
    Q(N,:) = q_end;
end
end
%% ============================================================
%                  A2G GAIN
%% ============================================================
