%% ����ʽ���,��������������ȳ���������ȳ�
%function [WheelFactor_mK, JointFactor_mK] = Polynomial_Fitting(paras)
%% 6*nά����������,�ֱ�������Ť�ػ�ؽ�Ť�ظ���״̬������n������ֵ
Sample_nums = 20; %������Ŀ
Fitting_nums = 3; %��ϵ���ߴ���,������Ĵ�����0

%����n������ֵ
Wheel_mK = zeros(6, Sample_nums); 
Joint_mK = zeros(6, Sample_nums); 

%����L������
L_min = 0.13*0.25; %��̵���������10cm
L_max = 0.40*0.25; %�����������30cm
L_bais = (L_max - L_min) / (Sample_nums-1); %Sample_nums-1 ���ܱ�֤L������1*Sample_numsά��
L = L_min:L_bais:L_max;

%�������ʽ��Ϻ��ϵ������1����Ϊ��һ�г�����
WheelFactor_mK = zeros(6, Fitting_nums+1);
JointFactor_mK = zeros(6, Fitting_nums+1);

%% ��ȡn��lqr�������k���󲢱�����������������
for i = 1:Sample_nums
    K_temp = Cal_mK_In_Zhi_Tui_Mode( L(i) ); %��ʱ�������lqr���ȡ��k����
    
    %���õ�i��lqr������������������
    Wheel_mK(1, i) = K_temp(1,1);
    Wheel_mK(2, i) = K_temp(1,2);
    Wheel_mK(3, i) = K_temp(1,3);
    Wheel_mK(4, i) = K_temp(1,4);
    Wheel_mK(5, i) = K_temp(1,5);
    Wheel_mK(6, i) = K_temp(1,6);
    
    Joint_mK(1, i) = K_temp(2,1);
    Joint_mK(2, i) = K_temp(2,2);
    Joint_mK(3, i) = K_temp(2,3);
    Joint_mK(4, i) = K_temp(2,4);
    Joint_mK(5, i) = K_temp(2,5);
    Joint_mK(6, i) = K_temp(2,6);
    
end

%% ����ʽ��ϻ�ȡFitting_nums�׵�ϵ����������ϵ���Ӹߵ�������
for i = 1:6
    WheelFactor_mK(i, :) = polyfit( L, Wheel_mK(i, :), Fitting_nums );
    
    JointFactor_mK(i, :) = polyfit( L, Joint_mK(i, :), Fitting_nums );
end

%% �������ӽ��в���,��ͼ
%figure;
%plot(L,Wheel_mK(1, :),'red-o') 
%title('����Ť�ص�����������������ȳ����������')
%grid on;

%L0 = 0.15;
%K1_1 = polyval(WheelFactor_mK(1, :), L0); %����Ť�ص����������
fprintf('float wheel_fitting_factor[6][4] = {\n');
fprintf('{%f,%f,%f,%f},\n',WheelFactor_mK(1,1),WheelFactor_mK(1,2),WheelFactor_mK(1,3),WheelFactor_mK(1,4));
fprintf('{%f,%f,%f,%f},\n\n',WheelFactor_mK(2,1),WheelFactor_mK(2,2),WheelFactor_mK(2,3),WheelFactor_mK(2,4));
fprintf('{%f,%f,%f,%f},\n',WheelFactor_mK(3,1),WheelFactor_mK(3,2),WheelFactor_mK(3,3),WheelFactor_mK(3,4));
fprintf('{%f,%f,%f,%f},\n\n',WheelFactor_mK(4,1),WheelFactor_mK(4,2),WheelFactor_mK(4,3),WheelFactor_mK(4,4));
fprintf('{%f,%f,%f,%f},\n',WheelFactor_mK(5,1),WheelFactor_mK(5,2),WheelFactor_mK(5,3),WheelFactor_mK(5,4));
fprintf('{%f,%f,%f,%f}\n',WheelFactor_mK(6,1),WheelFactor_mK(6,2),WheelFactor_mK(6,3),WheelFactor_mK(6,4));
fprintf('};\n float joint_fitting_factor[6][4] = {\n');
fprintf('{%f,%f,%f,%f},\n',JointFactor_mK(1,1),JointFactor_mK(1,2),JointFactor_mK(1,3),JointFactor_mK(1,4));
fprintf('{%f,%f,%f,%f},\n\n',JointFactor_mK(2,1),JointFactor_mK(2,2),JointFactor_mK(2,3),JointFactor_mK(2,4));
fprintf('{%f,%f,%f,%f},\n',JointFactor_mK(3,1),JointFactor_mK(3,2),JointFactor_mK(3,3),JointFactor_mK(3,4));
fprintf('{%f,%f,%f,%f},\n\n',JointFactor_mK(4,1),JointFactor_mK(4,2),JointFactor_mK(4,3),JointFactor_mK(4,4));
fprintf('{%f,%f,%f,%f},\n',JointFactor_mK(5,1),JointFactor_mK(5,2),JointFactor_mK(5,3),JointFactor_mK(5,4));
fprintf('{%f,%f,%f,%f}\n',JointFactor_mK(6,1),JointFactor_mK(6,2),JointFactor_mK(6,3),JointFactor_mK(6,4));
fprintf('};');
fprintf('\n');
fprintf('/////////////////////////////////////////////////////////////////////////////////////////////////');

