function reserved_verify(tx,raw,resL,resU,allowReplacement)
for u=1:numel(tx)
    assert(isequal(tx(u).users,raw(u).users), ...
        'User association changed unexpectedly');
    for b=1:2
        if b==1,idx=tx(u).idxLic;original=raw(u).idxLic;reserve=resL;
        else,idx=tx(u).idxUn;original=raw(u).idxUn;reserve=resU;end
        assert(numel(unique(idx(:,2)))==size(idx,1), ...
            'One UAV has two streams on one subchannel');
        for ell=1:size(idx,1)
            assert(any(original(:,1)==idx(ell,1)), ...
                'An unassociated user received a stream');
        end
        if ~allowReplacement
            assert(isequal(idx(1:size(original,1),:),original), ...
                'Initial assignment changed during addition-only test');
            assert(all(ismember(idx(size(original,1)+1:end,2),reserve)), ...
                'Added stream is not on a reserved subchannel');
        else
            assert(isequal(idx(:,1),original(:,1))&& ...
                size(idx,1)==size(original,1), ...
                'Replacement changed the number of original streams');
            changed=idx(:,2)~=original(:,2);
            assert(all(ismember(idx(changed,2),reserve)), ...
                'Replacement used a non-reserved channel');
        end
    end
end
for b=1:2
    if b==1,reserve=resL;else,reserve=resU;end
    for sc=reserve
        n=0;
        for u=1:numel(tx)
            if b==1,idx=tx(u).idxLic;else,idx=tx(u).idxUn;end
            n=n+sum(idx(:,2)==sc);
        end
        assert(n<=1,'Reserved SC assigned to multiple UAVs');
    end
end
end
