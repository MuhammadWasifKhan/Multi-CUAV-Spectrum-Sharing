function Q = build_multi_straight_trajectory(q_start_all,q_end_all,N)
U = size(q_start_all,1);
Q = zeros(U,N,2);
for u = 1:U
    for n = 1:N
        a = (n-1)/(N-1);
        Q(u,n,:) = (1-a)*q_start_all(u,:) + a*q_end_all(u,:);
    end
end
end
