function Q = build_multi_fly_hover_trajectory(q_start_all,q_hover_all,q_end_all,N,stepMax,area)
% Closed-loop/non-stationary initialization.
% If q_end_all = q_start_all,this creates q0 -> hover/service point -> q0.
% If the hover point is too far for the available N and stepMax,it uses the
% farthest feasible point along the q0-to-hover direction and still returns.
U = size(q_start_all,1);
Q = zeros(U,N,2);
for u = 1:U
    qs = q_start_all(u,:);
    qh = q_hover_all(u,:);
    qe = q_end_all(u,:);
    d1 = norm(qh - qs);
    d2 = norm(qe - qh);
    totalDist = d1 + d2;
    maxDist = stepMax*(N-1);
    % If the selected hover point is infeasible,pull it back along the
    % start-hover direction so that a return path remains feasible.
    if totalDist > maxDist && d1 > 1e-12
        % For closed-loop qs=qe,the feasible hover distance is maxDist/2.
        % For general qs~=qe this scale is a safe conservative fallback.
        scale = min(1,0.95*maxDist/max(totalDist,1e-12));
        qh = qs + scale*(qh - qs);
        qh = min(max(qh,0),area);
        d1 = norm(qh - qs);
        d2 = norm(qe - qh);
    end
    n1 = ceil(d1/max(stepMax,1e-12));
    n2 = ceil(d2/max(stepMax,1e-12));
    totalMove = n1 + n2;
    pts = zeros(N,2);
    idx = 1;
    pts(idx,:) = qs;
    if totalMove <= N-1 && totalMove > 0
        hoverSteps = (N-1) - totalMove;
        for ii = 1:n1
            idx = idx + 1;
            a = ii/max(n1,1);
            pts(idx,:) = (1-a)*qs + a*qh;
        end
        for ii = 1:hoverSteps
            idx = idx + 1;
            pts(idx,:) = qh;
        end
        for ii = 1:n2
            idx = idx + 1;
            a = ii/max(n2,1);
            pts(idx,:) = (1-a)*qh + a*qe;
        end
        if idx < N
            pts(idx+1:N,:) = repmat(qe,N-idx,1);
        end
    else
        % Final fallback: direct path. This only becomes stationary when
        % qs=qe and no feasible hover displacement exists.
        for n = 1:N
            a = (n-1)/(N-1);
            pts(n,:) = (1-a)*qs + a*qe;
        end
    end
    Q(u,:,:) = min(max(pts(1:N,:),0),area);
end
Q = project_multi_trajectory(Q,area,stepMax,q_start_all,q_end_all,30);
% Collision avoidance is checked/repaired by the caller after initialization.
end
