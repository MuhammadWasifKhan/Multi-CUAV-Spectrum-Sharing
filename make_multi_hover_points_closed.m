function q_hover_all = make_multi_hover_points_closed(q_start_all,WS,area)
% Closed-loop compatible hover/service point selection.
% Each SU is assigned to its nearest UAV initial point. Each UAV then uses
% the centroid of its assigned SUs as the hover/service point.
U = size(q_start_all,1);
K = size(WS,1);
assign = zeros(K,1);
q_hover_all = zeros(U,2);
for k = 1:K
    distToUAV = vecnorm(q_start_all - WS(k,:),2,2);
    [~,uBest] = min(distToUAV);
    assign(k) = uBest;
end
for u = 1:U
    ids = find(assign == u);
    if isempty(ids)
        q_hover_all(u,:) = q_start_all(u,:);
    else
        q_hover_all(u,:) = mean(WS(ids,:),1);
    end
    q_hover_all(u,:) = min(max(q_hover_all(u,:),0),area);
end
end
