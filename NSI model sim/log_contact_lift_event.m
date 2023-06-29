function [contact_all_timeo,lift_all_timeo] = log_contact_lift_event(contact_all,contact_all_time,lift_all_time,i,contact)
    if length(contact_all)-1>0
                if contact_all(length(contact_all)-1) == 0 && contact == 1
                    contact_all_timeo = [contact_all_time,i];
                else
                    contact_all_timeo = contact_all_time;
                end
    else
       contact_all_timeo = contact_all_time;
    end
    
       if length(contact_all)-1>0
                if contact_all(length(contact_all)-1) == 1 && contact == 0
                    lift_all_timeo = [lift_all_time,i];
                
                else
                    lift_all_timeo = lift_all_time;
                end
       else
           lift_all_timeo = lift_all_time;
       end
       
end