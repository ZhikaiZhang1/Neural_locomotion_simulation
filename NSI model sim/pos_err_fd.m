function [fd, pos1_err_all_out,fd_all_out, influence] = pos_err_fd(pos_goal, curr_pos, pos_err_all, max_min_fd, divider, w_fd, min_max, fd_all)
    err = pos_goal - curr_pos;
    pos1_err_all_out = [pos_err_all,err];
    if min_max == 'min'
        fd_prelim = divider/abs(err);
        if (err>=0)
            influence = min(max_min_fd,fd_prelim);
            fd = min(max_min_fd,fd_prelim)*w_fd;
        else
            influence = max_min_fd;
            fd = max_min_fd*w_fd;
        end

    else
        influence =  max(-max_min_fd, min(max_min_fd,divider/(err)));
        fd = max(-max_min_fd, min(max_min_fd,divider/(err)))*w_fd;
    end
    fd_all_out = [fd_all,fd];
end