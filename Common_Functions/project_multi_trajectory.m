function Q = project_multi_trajectory(Q,area,stepMax,q_start_all,q_end_all,projIter)
U = size(Q,1);
for u = 1:U
    Qu = squeeze(Q(u,:,:));
    Qu = project_full_trajectory(Qu,area,stepMax,q_start_all(u,:),q_end_all(u,:),projIter);
    Q(u,:,:) = Qu;
end
end
