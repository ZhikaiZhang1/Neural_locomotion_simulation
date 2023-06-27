function [new_pos, pos_all_out] = get_vel_pos(curr_pos, Tau, neuron1_out, neuron2_out,pos_all)
    new_pos = curr_pos + neuron1_out*Tau;
    new_pos = new_pos - neuron2_out*Tau;
    pos_all_out = [pos_all,new_pos];
end