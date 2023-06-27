function [T02,T03,T04,T05, q_1,q_2,q_3] = get_fk_sym()
    q_1=sym('q_1');q_2=sym('q_2');q_3=sym('q_3');
    BENDY_OUTPUT_DIS = 0.036;
    BENDY_INPUT_DIS = 0.049;
    ELBOW_INPUT_DIS = 0.037;
    ELBOW_OUTPUT_DIS = 0.046;
    FOOT_INPUT_DIS = 0.08665;
    starting_offset = 0;
    %alpha = [0,0,0,0,0];
    alpha = [starting_offset,-pi/2,0,0,0];
    a = [-BENDY_INPUT_DIS,-BENDY_OUTPUT_DIS-BENDY_INPUT_DIS,-BENDY_OUTPUT_DIS-BENDY_INPUT_DIS, -ELBOW_INPUT_DIS-BENDY_OUTPUT_DIS, -ELBOW_OUTPUT_DIS-FOOT_INPUT_DIS];
    d = [0,0,0,0,0];
    theta = [starting_offset,q_1,q_2,q_3,pi/2];
    
    alpha_0=alpha(1);alpha_1=alpha(2);alpha_2=alpha(3);alpha_3=alpha(4);alpha_4=alpha(5);
    a_0=a(1);a_1=a(2);a_2=a(3);a_3=a(4);a_4=a(5);
    d_1=d(1);d_2=d(2);d_3=d(3);d_4=d(4);d_5=d(5);
    
    R_1 = Rotaion_Matrix(0, alpha(1));
    R_2 = Rotaion_Matrix(q_1, alpha(2));
    R_3 = Rotaion_Matrix(q_2, alpha(3));
    R_4 = Rotaion_Matrix(pi/2, alpha(4));
    R_5 = Rotaion_Matrix(q_3, alpha(5));
    
    p_1=[cos(theta(1))*a_0;sin(theta(1))*a_0;d_1];
    p_2=[cos(theta(2))*a_1;sin(theta(2))*a_1;d_2];
    p_3=[cos(theta(3))*a_2;sin(theta(3))*a_2;d_3];
    p_4=[cos(theta(4))*a_3;sin(theta(4))*a_3;d_4];
    p_5=[cos(theta(5))*a_4;sin(theta(5))*a_4;d_5];
    
    T_1 = [R_1,p_1;zeros(1,3),1];
    T_2 = [R_2,p_2;zeros(1,3),1];
    T_3 = [R_3,p_3;zeros(1,3),1];
    T_4 = [R_4,p_4;zeros(1,3),1];
    T_5 = [R_5,p_5;zeros(1,3),1];
    T = T_1*T_2*T_3*T_4*T_5;
    T02 = T_1*T_2; T03 = T_1*T_2*T_3; T04 = T_1*T_2*T_3*T_4; T05 = T_1*T_2*T_3*T_4*T_5;
    
    
    
    
%     % FK test case [0,0,0]
%     Transformation = subs(T, [q_1, q_2, q_3], [0,0,0]);
%     default_config = vpa(Transformation)*[0;0;0;1]
%     x = 2*BENDY_OUTPUT_DIS+2*BENDY_INPUT_DIS+ELBOW_INPUT_DIS
%     y = 0
%     z = BENDY_OUTPUT_DIS+BENDY_INPUT_DIS+ELBOW_OUTPUT_DIS+FOOT_INPUT_DIS
%     
%     % test case [0,-pi/2,0]
%     Transformation = subs(T, [q_1, q_2, q_3], [0,-pi/2,0]);
%     default_config = vpa(Transformation)*[0;0;0;1]
%     x = 2*BENDY_OUTPUT_DIS+2*BENDY_INPUT_DIS+BENDY_INPUT_DIS+ELBOW_OUTPUT_DIS+FOOT_INPUT_DIS
%     y = 0
%     z = BENDY_OUTPUT_DIS+ELBOW_INPUT_DIS
%     
%     % test case [0,0,pi/2]
%     %test_angles = [0,0,pi/2];
%     %test_angles = [0.3,0.15,0.2];
%     %test_angles = [0,-pi/2,0];
%     test_angles = [0.16,1.3,0.4];
%     %test_angles = [-0.3,-1.1,1.3];
%     %test_angles = [-0.02,0.1,0.2];
%     %Transformation = subs(T, [q_1, q_2, q_3], [0,0,pi/2]);
%     Transformation = subs(T, [q_1, q_2, q_3], test_angles);
%     default_config = vpa(Transformation)*[0;0;0;1]
%     x = 2*BENDY_OUTPUT_DIS+2*BENDY_INPUT_DIS+ELBOW_INPUT_DIS-BENDY_OUTPUT_DIS-FOOT_INPUT_DIS
%     y = 0
%     z = BENDY_INPUT_DIS+ELBOW_OUTPUT_DIS
%     euler_angles = rotm2eul(double(Transformation(1:3,1:3)))

end