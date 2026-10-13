function v = unit_vec(x)
nx = norm(x);
if nx < 1e-12
    v = [0,0];
else
    v = x/nx;
end
end
%% ============================================================
%                  VECTOR CHANNEL GENERATION,Nt = 8
%% ============================================================
