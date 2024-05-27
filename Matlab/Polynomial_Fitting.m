%% 多项式拟合,参数是最短虚拟腿长和最长虚拟腿长
%function [WheelFactor_mK, JointFactor_mK] = Polynomial_Fitting(paras)
%% 6*n维的两个矩阵,分别存放轮子扭矩或关节扭矩各自状态变量的n次增益值
Sample_nums = 20; %采样数目
Fitting_nums = 3; %拟合的最高次数,常数项的次数是0

%保存n次增益值
Wheel_mK = zeros(6, Sample_nums); 
Joint_mK = zeros(6, Sample_nums); 

%保存L的样本
L_min = 0.13*0.25; %最短的虚拟腿是10cm
L_max = 0.40*0.25; %最长的虚拟腿是30cm
L_bais = (L_max - L_min) / (Sample_nums-1); %Sample_nums-1 才能保证L矩阵是1*Sample_nums维度
L = L_min:L_bais:L_max;

%保存多项式拟合后的系数，加1是因为有一列常数项
WheelFactor_mK = zeros(6, Fitting_nums+1);
JointFactor_mK = zeros(6, Fitting_nums+1);

%% 获取n次lqr函数后的k矩阵并保存在两个大数组中
for i = 1:Sample_nums
    K_temp = Cal_mK_In_Zhi_Tui_Mode( L(i) ); %暂时保存调用lqr后获取的k矩阵
    
    %调用第i次lqr并保存在两个大数组
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

%% 多项式拟合获取Fitting_nums阶的系数，阶数的系数从高到低排序
for i = 1:6
    WheelFactor_mK(i, :) = polyfit( L, Wheel_mK(i, :), Fitting_nums );
    
    JointFactor_mK(i, :) = polyfit( L, Joint_mK(i, :), Fitting_nums );
end

%% 给个例子进行测试,画图
%figure;
%plot(L,Wheel_mK(1, :),'red-o') 
%title('轮子扭矩的腿倾角增益与虚拟腿长的拟合曲线')
%grid on;

%L0 = 0.15;
%K1_1 = polyval(WheelFactor_mK(1, :), L0); %轮子扭矩的腿倾角增益
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

