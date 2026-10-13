function h = gen_miso_vector(gLarge,Nt,useRayleigh)
% MISO channel vector Nt x 1. The expected norm^2 is approximately Nt*gLarge.
% This gives array gain to both MRT and MM while retaining spatial DoF.
if useRayleigh
    h = sqrt(max(gLarge,0)) * (randn(Nt,1) + 1j*randn(Nt,1))/sqrt(2);
else
    h = sqrt(max(gLarge,0)) * ones(Nt,1);
end
end
