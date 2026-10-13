function q_next = traj_step_projected_gradient_mean(q0,q_prev,q_end,n,N,WS,H,f_lic,f_un,c,A2G,...
        cLic_k,Blic_k,cUn_k,Bun_k,stepMax,area,epsFD)
Kloc = size(WS,1);
grad = [0,0];
for k = 1:Kloc
    wk = WS(k,:);
    if cLic_k(k) > 0 && Blic_k(k) > 0
        [dgdx,dgdy,g0] = fd_gain_gradient(q0,wk,H,f_lic,c,A2G,epsFD);
        coeff = (Blic_k(k)/log(2)) * cLic_k(k) / max(1 + cLic_k(k)*g0,1e-12);
        grad = grad + coeff * [dgdx,dgdy];
    end
    if cUn_k(k) > 0 && Bun_k(k) > 0
        [dgdx,dgdy,g0] = fd_gain_gradient(q0,wk,H,f_un,c,A2G,epsFD);
        coeff = (Bun_k(k)/log(2)) * cUn_k(k) / max(1 + cUn_k(k)*g0,1e-12);
        grad = grad + coeff * [dgdx,dgdy];
    end
end
if norm(grad) < 1e-12
    q_candidate = q0;
else
    q_candidate = q_prev + stepMax * grad/norm(grad);
end
q_next = project_single_step(q_candidate,q_prev,q_end,n,N,stepMax,area);
end
