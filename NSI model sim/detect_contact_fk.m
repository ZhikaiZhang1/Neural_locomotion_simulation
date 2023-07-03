function [contact,contact_all_out,z_all_out] = detect_contact_fk(T,q_1, q_2, q_3, position1_c,position1_tr,position1, thresh, contact_all,z_all,robot_height)
   Transformation = subs(T, [q_1, q_2, q_3], [position1_c,position1_tr,position1]);
   cartesian_pos = vpa(Transformation)*[0;0;robot_height;1];
   z_all_out = [z_all,cartesian_pos(3)];
    
   %    contact_threshold
   if cartesian_pos(3) > thresh
       contact = 0;
   else
       contact = 1;
   end
   contact_all_out = [contact_all,contact];
end