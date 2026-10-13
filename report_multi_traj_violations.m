function report_multi_traj_violations(Q,qStart,qEnd,stepMax,area)
for u=1:size(Q,1)
    X=squeeze(Q(u,:,:));
    steps=sqrt(sum(diff(X,1,1).^2,2));
    fprintf('  UAV %d: start error %.3g m; end error %.3g m; max step %.9f / %.9f m; coordinate range [%.5f, %.5f].\n', ...
        u,norm(X(1,:)-qStart(u,:)),norm(X(end,:)-qEnd(u,:)), ...
        max(steps),stepMax,min(X(:)),max(X(:)));
end
end
