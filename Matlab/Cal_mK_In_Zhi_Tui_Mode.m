 %% ��ȡֱ��ģ�͵�K����
   %��������Ϊ1���������L
function [K] = Cal_mK_In_Zhi_Tui_Mode(paras)
%% ���ú�������ȡA,B����
[A_ballance, B_ballance] = Cal_mA_mB_In_Zhi_Tui_Mode(paras(1));
%[A_ballance, B_ballance] = Cal_mA_mB_In_Zhi_Tui_Mode(0.13);
%% ����lqr����
T = 0.005;%������������
%[G,H]=c2d(A_ballance,B_ballance,T);

Q = [150   0    0    0    0     0;   %�����
     0   880   0    0    0     0;   %������ٶ�
     0    0    5800   0    0    0;   %λ��
     0    0    0    250    0   0;   %�ٶ�
     0    0    0    0   80000    0;   %����Ƕ�
     0    0    0    0    0     500];   %������ٶ�

     
R = [60      0 ;  %����
     0        40];   %�ؽ�

%K = dlqr(G, H, Q, R);
K = lqr(A_ballance, B_ballance, Q, R);

%fprintf('{%f,%f,%f,%f,%f,%f},\n',K(1,1),K(1,2),K(1,3),K(1,4),K(1,5),K(1,6));
%fprintf('{%f,%f,%f,%f,%f,%f},\n\n',K(2,1),K(2,2),K(2,3),K(2,4),K(2,5),K(2,6));
end