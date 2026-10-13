function Ptot = total_band_power(tx,bandName)
Ptot = 0;
for u = 1:numel(tx)
    [~,p,~] = get_band_fields(tx,u,bandName);
    if ~isempty(p)
        Ptot = Ptot + sum(p);
    end
end
end
