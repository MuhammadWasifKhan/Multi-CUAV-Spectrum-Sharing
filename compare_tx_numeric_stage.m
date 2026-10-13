function same=compare_tx_numeric_stage(a,b,tol)
same=numel(a)==numel(b);
if ~same,return;end
for uu=1:numel(a)
    fields={'users','idxLic','idxUn','pLic','pUn'};
    for ff=1:numel(fields)
        x=a(uu).(fields{ff}); y=b(uu).(fields{ff});
        if ~isequal(size(x),size(y)) || numel(x)~=numel(y)
            same=false;return;
        end
        if isempty(x),continue;end
        if max(abs(double(x(:))-double(y(:))))>tol
            same=false;return;
        end
    end
    for field={'vLic','vUn'}
        x=a(uu).(field{1});y=b(uu).(field{1});
        if numel(x)~=numel(y),same=false;return;end
        for ii=1:numel(x)
            if ~isequal(size(x{ii}),size(y{ii})) || ...
                    max(abs(x{ii}(:)-y{ii}(:)))>tol
                same=false;return;
            end
        end
    end
end
end
