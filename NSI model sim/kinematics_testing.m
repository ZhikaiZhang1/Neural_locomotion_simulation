%% FK
clear all
close all
q_1=sym('q_1');q_2=sym('q_2');q_3=sym('q_3');q_4=sym('q_4');q_5=sym('q_5');q_6=sym('q_6');
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




% FK test case [0,0,0]
Transformation = subs(T, [q_1, q_2, q_3], [0,0,0]);
default_config = vpa(Transformation)*[0;0;0;1]
x = 2*BENDY_OUTPUT_DIS+2*BENDY_INPUT_DIS+ELBOW_INPUT_DIS
y = 0
z = BENDY_OUTPUT_DIS+BENDY_INPUT_DIS+ELBOW_OUTPUT_DIS+FOOT_INPUT_DIS

% test case [0,-pi/2,0]
Transformation = subs(T, [q_1, q_2, q_3], [0,-pi/2,0]);
default_config = vpa(Transformation)*[0;0;0;1]
x = 2*BENDY_OUTPUT_DIS+2*BENDY_INPUT_DIS+BENDY_INPUT_DIS+ELBOW_OUTPUT_DIS+FOOT_INPUT_DIS
y = 0
z = BENDY_OUTPUT_DIS+ELBOW_INPUT_DIS

% test case [0,0,pi/2]
%test_angles = [0,0,pi/2];
%test_angles = [0.3,0.15,0.2];
%test_angles = [0,-pi/2,0];
test_angles = [0.16,1.3,0.4];
%test_angles = [-0.3,-1.1,1.3];
%test_angles = [-0.02,0.1,0.2];
%Transformation = subs(T, [q_1, q_2, q_3], [0,0,pi/2]);
Transformation = subs(T, [q_1, q_2, q_3], test_angles);
default_config = vpa(Transformation)*[0;0;0;1]
x = 2*BENDY_OUTPUT_DIS+2*BENDY_INPUT_DIS+ELBOW_INPUT_DIS-BENDY_OUTPUT_DIS-FOOT_INPUT_DIS
y = 0
z = BENDY_INPUT_DIS+ELBOW_OUTPUT_DIS
euler_angles = rotm2eul(double(Transformation(1:3,1:3)))

%% IK ith orientation

% % jacobian/IK test case
% Z0=[0;0;1];
% Z1 = T_1(1:3,3);%use
% Z2 = T02(1:3,3); %use
% Z3 = T03(1:3,3);
% Z4 = T04(1:3,3);%use
% Z5 = T05(1:3,3);
% Transformation = subs(T, [q_1, q_2, q_3], [0,0,0]);
% curr_eff_pos = vpa(Transformation)*[0;0;0;1];
% %get jacobian
% Odiff3 = curr_eff_pos - T04*[0;0;0;1];
% Odiff2 = curr_eff_pos - T02*[0;0;0;1];
% Odiff1 = curr_eff_pos - T_1*[0;0;0;1];
% 
% Jv3 = cross(Z4,Odiff3(1:3));
% Jv2 = cross(Z2,Odiff2(1:3));
% Jv1 = cross(Z1,Odiff1(1:3));
% Jo3 = Z4; Jo2 = Z2; Jo1 = Z1;
% Jacobian = [Jv1 Jv2 Jv3;Jo1 Jo2 Jo3];
% Jacobian_num = vpa(subs(Jacobian, [q_1, q_2, q_3], [0,0,0]))
% pos_err = 100; ori_err = 100;
% 
% joint_angles = [0; 0; 0]; 
% desired_pos = [double(default_config(1:3));flip(transpose(euler_angles))];
% %desired_pos = transpose([-0.145334,-0.029654,0.220021,1.572637,1.220164,-2.839864])
% delta_angles = [0;0;0];
% 
% %-----------------logging---------------------
% pos_err_all = []; ori_err_all = [];
% angles_all = [];
% %-----------------logging---------------------
% while (abs(pos_err) >= 0.02 || abs(ori_err) >= 0.035)
%     joint_angles(1) = wrapToPi(joint_angles(1)+delta_angles(1));
%     joint_angles(2) = wrapToPi(joint_angles(2)+delta_angles(2));
%     joint_angles(3) = wrapToPi(joint_angles(3)+delta_angles(3));
%     angles_all = [angles_all,joint_angles];
% 
%     Transformation = subs(T, [q_1, q_2, q_3], [joint_angles(1),joint_angles(2),joint_angles(3)]);
%     pos_result = vpa(Transformation)*[0;0;0;1];
%     ori_result = flip(transpose(rotm2eul(double(Transformation(1:3,1:3)))));
%     
%     pos_err = max(abs(desired_pos(1:3) - pos_result(1:3)))
%     pos_err_all = [pos_err_all, pos_err];
%     ori_err = (norm(desired_pos(4:6)) - norm(ori_result(1:3)))
%     ori_err_all = [ori_err_all, ori_err];
%     %get the jacobian
%     Jacobian_num = vpa(subs(Jacobian, [q_1, q_2, q_3], [joint_angles(1),joint_angles(2),joint_angles(3)]));
%     inv_Jacobian_num = pinv(Jacobian_num);
%     deltaE = [desired_pos(1:3)-pos_result(1:3);desired_pos(4:6)-ori_result];
% 
%     delta_angles = inv_Jacobian_num*deltaE;
% end
% 
% 
% 
% final_angles = transpose(wrapToPi(joint_angles))
% original_angles = test_angles
% 
% 
% %original_pos = double(desired_pos)
% %final_pos = vpa([pos_result(1:3);ori_result])
% original_pos = double(desired_pos)
% Transformation = subs(T, [q_1, q_2, q_3], final_angles);
% final_ori = flip(transpose(rotm2eul(double(Transformation(1:3,1:3)))));
% final_pos = [vpa(Transformation)*[0;0;0;1];vpa(final_ori)]
% inv_Jacobian_num
% deltaE
% Jacobian_num1
% original_Jacobian_num = vpa(subs(Jacobian, [q_1, q_2, q_3], [0,0,0]))
% 
% %previous error combos
% %pos_err = norm(desired_pos(1:3)) - norm(pos_result(1:3));
% %ori_err = norm(desired_pos(4:6)) - norm(ori_result(1:3));
% 
% %pos_err = max(abs(norm(desired_pos(1:3))-norm(pos_result(1:3))))
% %ori_err = max(abs(norm(desired_pos(4:6))-norm(ori_result(1:3))))
%     
% %pos_err = max(abs(desired_pos(1:3) - pos_result(1:3)))%max(abs(norm(desired_pos(1:3))-norm(pos_result(1:3))))
% %ori_err = max(abs(desired_pos(4:6) - ori_result(1:3)))%max(abs(norm(desired_pos(4:6))-norm(ori_result(1:3))))
% steps = linspace(0,length(pos_err_all)-1,length(pos_err_all));
% figure()
% plot(steps, pos_err_all)
% title('position error')
% xlabel('steps')
% ylabel('error (m)')
% 
% figure()
% plot(steps, ori_err_all)
% title('orientation error')
% xlabel('steps')
% ylabel('error (rad)')
% figure()
% plot(steps, angles_all)

%% IK without orientation



% jacobian/IK test case
Z0=[0;0;1];
Z1 = T_1(1:3,3);%use
Z2 = T02(1:3,3); %use
Z3 = T03(1:3,3);
Z4 = T04(1:3,3);%use
Z5 = T05(1:3,3);
Transformation = subs(T, [q_1, q_2, q_3], [0,0,0]);
curr_eff_pos = vpa(Transformation)*[0;0;0;1];
%get jacobian
Odiff3 = curr_eff_pos - T04*[0;0;0;1];
Odiff2 = curr_eff_pos - T02*[0;0;0;1];
Odiff1 = curr_eff_pos - T_1*[0;0;0;1];

Jv3 = cross(Z4,Odiff3(1:3));
Jv2 = cross(Z2,Odiff2(1:3));
Jv1 = cross(Z1,Odiff1(1:3));
Jo3 = Z4; Jo2 = Z2; Jo1 = Z1;
Jacobian = [Jv1 Jv2 Jv3];
Jacobian_num = vpa(subs(Jacobian, [q_1, q_2, q_3], [0,0,0]))
pos_err = 100; ori_err = 100;

joint_angles = [0; 0; 0]; 
desired_pos = [double(default_config(1:3));flip(transpose(euler_angles))];
%desired_pos = transpose([-0.145334,-0.029654,0.220021,1.572637,1.220164,-2.839864])
delta_angles = [0;0;0];

%-----------------logging---------------------
pos_err_all = []; ori_err_all = [];
angles_all = [];
%-----------------logging---------------------
while (abs(pos_err) >= 0.02)
    joint_angles(1) = wrapToPi(joint_angles(1)+delta_angles(1));
    joint_angles(2) = wrapToPi(joint_angles(2)+delta_angles(2));
    joint_angles(3) = wrapToPi(joint_angles(3)+delta_angles(3));
    angles_all = [angles_all,joint_angles];

    Transformation = subs(T, [q_1, q_2, q_3], [joint_angles(1),joint_angles(2),joint_angles(3)]);
    pos_result = vpa(Transformation)*[0;0;0;1];
    ori_result = flip(transpose(rotm2eul(double(Transformation(1:3,1:3)))));
    
    pos_err = max(abs(desired_pos(1:3) - pos_result(1:3)))
    pos_err_all = [pos_err_all, pos_err];
    %ori_err = (norm(desired_pos(4:6)) - norm(ori_result(1:3)))
    %ori_err_all = [ori_err_all, ori_err];
    %get the jacobian
    Jacobian_num = vpa(subs(Jacobian, [q_1, q_2, q_3], [joint_angles(1),joint_angles(2),joint_angles(3)]));
    inv_Jacobian_num = pinv(Jacobian_num);
    deltaE = [desired_pos(1:3)-pos_result(1:3)];

    delta_angles = inv_Jacobian_num*deltaE;
end



final_angles = transpose(wrapToPi(joint_angles))
original_angles = test_angles


%original_pos = double(desired_pos)
%final_pos = vpa([pos_result(1:3);ori_result])
original_pos = double(desired_pos)
Transformation = subs(T, [q_1, q_2, q_3], final_angles);
final_ori = flip(transpose(rotm2eul(double(Transformation(1:3,1:3)))));
final_pos = [vpa(Transformation)*[0;0;0;1];vpa(final_ori)]
inv_Jacobian_num
deltaE
Jacobian_num
original_Jacobian_num = vpa(subs(Jacobian, [q_1, q_2, q_3], [0,0,0]))

%previous error combos
%pos_err = norm(desired_pos(1:3)) - norm(pos_result(1:3));
%ori_err = norm(desired_pos(4:6)) - norm(ori_result(1:3));

%pos_err = max(abs(norm(desired_pos(1:3))-norm(pos_result(1:3))))
%ori_err = max(abs(norm(desired_pos(4:6))-norm(ori_result(1:3))))
    
%pos_err = max(abs(desired_pos(1:3) - pos_result(1:3)))%max(abs(norm(desired_pos(1:3))-norm(pos_result(1:3))))
%ori_err = max(abs(desired_pos(4:6) - ori_result(1:3)))%max(abs(norm(desired_pos(4:6))-norm(ori_result(1:3))))
steps = linspace(0,length(pos_err_all)-1,length(pos_err_all));
figure()
plot(steps, pos_err_all)
title('position error')
xlabel('steps')
ylabel('error (m)')

figure()
plot(steps, angles_all)

%% testing current sin leg trajectory
close all

 amplitudes = [pi/8 pi/8 pi/8];
 const_offsets = [
            [-1,1,-1,1,-1,1];
            [0,0,0,0,0,0];
            [0,0,0,0,0,0]]*amplitudes(1);
        phase_offsets = [
            [pi/2,-pi/2,pi/2,-pi/2,pi/2,-pi/2];
            [0,pi,0,pi,0,pi];
            [0,pi,0,pi,0,pi]];
 t = 0;
 freq = 106;
 dt = 1.0/105*pi;
 pos_3d = [];
 forward_cmd = 1;
 turn_cmd = 0;
 %joint_min = -pi/16;
 joint_min = -Inf;
 all_t = [];

 cycle = 4;
 leg_idx = 2;
 for k = 1:cycle*freq
     joint_angle = [];
     for i = 1:3
        joint_angle = [joint_angle, amplitudes(i)*sin(t + phase_offsets(i,1))];
     end
     joint_angle(2) = max([joint_angle(2), 0]);
     joint_angle(3) = max([joint_angle(3), 0]);
%      joint_angle(1) = joint_angle(1) * (0.5*sign(forward_cmd) + (0.5*(sign(turn_cmd)))) + const_offsets(1,1);
%      joint_angle(2) = max([joint_angle(2), joint_min]) + const_offsets(2,1);
%      joint_angle(2) = -joint_angle(2);
%      joint_angle(3) = max([joint_angle(3), joint_min]) + const_offsets(3,1);
     Transformation = subs(T, [q_1, q_2, q_3], [joint_angle(1),joint_angle(2),joint_angle(3)]);
     pos_3d = [pos_3d,vpa(Transformation)*[0;0;ELBOW_OUTPUT_DIS+FOOT_INPUT_DIS;1]];
     all_t = [all_t, t];
     t = t + dt;
%      if t >= 2*pi
%         t = 0;
%      end
 end
 x = pos_3d(1,:);
 y = pos_3d(2,:);
 z = pos_3d(3,:);
 figure();
 plot3(x,y,z)
%  zlim([0,1])
%  xlim([-0.3,0.1])
%  ylim([-0.2,0.2]



% hold on
% for i = 1:length(x)
%     h = plot3(x(i),y(i),z(i),'o');            % draw something on the trajectory
%     pause(0.01);                                % wait a minute
%     delete(h);                                 % delete it
% end
% hold off
 figure();
 plot(x,y)

figure();
plot(all_t, z)

figure();
z = z-(max(z)+min(z))/2;
lengthz = length(z)
for i = 1:length(z)
     z(i) = -z(i);
end
% z = z+(0.241+0.215)/2
% z = z-(0.241+0.215)/2
indices = find(z<=0);
z(indices) = [];

time_z = [0]
for i = 1:length(z)-1
    time_z = [time_z,time_z(length(time_z))+1/200];
end
z = z+0.195



plot(time_z, z)
csvwrite('z_desired.csv', z(1:200))
