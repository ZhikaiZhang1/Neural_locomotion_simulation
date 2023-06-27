function stance_swing = detect_phase(flx_out, ext_out)
% 1: not stance phase, 0: stance phase, 2: swing phase (with contact)

    if flx_out <= ext_out
        stance_swing = 1;
    else 
        stance_swing = 0;
    end

end