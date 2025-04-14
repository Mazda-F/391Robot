function [W, Nf_new, len] = getFIR(beta)
%FINDWLEN gets FIR W normalized vec
%   input beta
    idx = 0;
    while true
        if (beta^idx <= exp(-4))
            break;
        else
            idx = idx + 1;
        end
    end
    len = idx; % this is vector len
    
    wsum = 0;
    for idx = 1:len
        W(idx) = beta^(idx-1);
        wsum = wsum + W(idx);
    end
    W = W/wsum;

    Nf_new = 0;
    for idx = 1:len
        Nf_new = Nf_new + (idx-1)*W(idx);
    end
end