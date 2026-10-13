function v = leakage_rzf_unit(hSU,hProt,alpha)
Nt = numel(hSU);
A = hProt*hProt' + alpha*eye(Nt);
v = A \ hSU;
if norm(v) < 1e-30
    v = mrt_unit(hSU);
else
    v = v/norm(v);
end
end
