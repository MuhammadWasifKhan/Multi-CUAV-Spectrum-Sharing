function extraNoise = previous_interuav_noise_matrix(txPrev,HsuFull,servingUAV,users_u,S,bandName)
% Previous-iterate inter-UAV co-channel interference used in the AO
% assignment metric. Size: (#local users) x (#subchannels).
extraNoise = zeros(numel(users_u),S);
if isempty(txPrev)
    return;
end
for kk = 1:numel(users_u)
    kglob = users_u(kk);
    for s = 1:S
        extraNoise(kk,s) = inter_uav_power_same_sc(txPrev,HsuFull,servingUAV,kglob,s,bandName);
    end
end
end
