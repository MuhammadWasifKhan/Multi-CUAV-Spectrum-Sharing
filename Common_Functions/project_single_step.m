function q_projected = project_single_step(q,q_prev,q_end,n,N,stepMax,area)
q_projected = min(max(q,[0,0]),[area,area]);
d = q_projected - q_prev;
nd = norm(d);
if nd > stepMax
    q_projected = q_prev + stepMax * d / max(nd,1e-12);
end
q_projected = min(max(q_projected,[0,0]),[area,area]);
remSteps = N - n;
rReach = remSteps * stepMax;
v = q_projected - q_end;
nv = norm(v);
if nv > rReach
    q_projected = q_end + rReach * v / max(nv,1e-12);
end
q_projected = min(max(q_projected,[0,0]),[area,area]);
d = q_projected - q_prev;
nd = norm(d);
if nd > stepMax
    q_projected = q_prev + stepMax * d / max(nd,1e-12);
end
q_projected = min(max(q_projected,[0,0]),[area,area]);
end
