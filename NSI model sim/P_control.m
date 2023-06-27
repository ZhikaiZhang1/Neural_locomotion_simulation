function output = P_control(desired_pos, pos, weight)
    output = (desired_pos-pos)*weight;
    min(output, 3);
end