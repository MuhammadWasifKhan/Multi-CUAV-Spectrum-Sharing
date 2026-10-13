function Qnew = resample_multi_trajectory_local(Qold,Nnew)
U = size(Qold,1);
Nold = size(Qold,2);
Qnew = zeros(U,Nnew,2);
xOld = linspace(0,1,Nold);
xNew = linspace(0,1,Nnew);
for u = 1:U
    Qnew(u,:,1) = interp1(xOld,squeeze(Qold(u,:,1)),xNew,'linear','extrap');
    Qnew(u,:,2) = interp1(xOld,squeeze(Qold(u,:,2)),xNew,'linear','extrap');
end
end
%% ============================================================
%                  MULTI-UAV TRAJECTORY FUNCTIONS
%% ============================================================
