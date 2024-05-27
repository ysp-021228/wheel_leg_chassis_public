%% 
   %��ȡֱ��ģ�͵�A��B���󣬲�������������ľ���ֵ
   %��������Ϊ1���������L
function [A_ballance, B_ballance] = Cal_mA_mB_In_Zhi_Tui_Mode(paras)
%% �������� g R L Lm l Mw Mp M Iw Ip Im ... %�������
syms theta  thetad1     thetad2 ...  %������ֱ����ļн�
     x      xdot1       xdot2 ...    %λ��
     phi    phidot1     phidot2 ...  %�������
     T   Tp   N   P  Nm  Pm  Nf   %Ť������
 
L = paras(1); %0.15
%L = 0.17;
g = 9.8;
R = 0.075;
Lm =(3/4)*L;     %�ڷ������棬Ӧ�õõ���L
l = 0.045;   %��Сһ��
Mw = 0.320;
Mp = 1.99;   %���еĴ�С��
M = 6.715;
Iw = 0.000120085216;
Ip = 0.04797207231;
Im = 0.20703226313;

%% ���鷽��
% �������ֺ�������ת�ط���
f1 = [Mw*xdot2 - (Nf -N) == 0;
      Iw*xdot2/R - (T - Nf*R) == 0];
x1 = [xdot2 N];
[sovxdot2, ~] = solve(f1,x1);

% �԰ڸ˺�������ת�ط���
f2 = [N - Nm - (Mp*(xdot2 + L*thetad2*cos(theta) - L*(thetad1)^2*sin(theta))) == 0;
      P - Pm - Mp*g - (Mp*(-L*thetad2*sin(theta) - L*(thetad1)^2*cos(theta))) == 0;
      Ip*thetad2 - ((P*L + Pm*Lm)*sin(theta) - (N*L + Nm*Lm)*cos(theta) - T + Tp) == 0];

% �Ի����������ת�ط���
f3 = [Nm - ( M* (xdot2 + (L + Lm)*thetad2*cos(theta) - (L + Lm)*(thetad1)^2*sin(theta) - l*phidot2*cos(phi) + l*(phidot1)^2*sin(phi)) ) == 0;
      Pm - M*g - (M* (  -(L + Lm)*thetad2*sin(theta) - (L + Lm)*(thetad1)^2*cos(theta) - l*phidot2*sin(phi) - l*(phidot1)^2*cos(phi)  ) ) == 0;
      Im*phidot2 - ( Tp + Nm*l*cos(phi) + Pm*l*sin(phi) ) == 0];

f = [f1;f2;f3];

%% ��ȥ�м����
Nm_c = solve(f(6),Nm);
N_c = solve(subs(f(3), Nm, Nm_c), N);
Pm_c = solve(f(7), Pm);
P_c = solve(subs(f(4), Pm, Pm_c), P);
Nf_c = solve(subs(f(1), N, N_c), Nf);

%% �г������Ի����̣���ȥ�м������
xdot2_f = subs((sovxdot2 - xdot2), Nf, Nf_c);
thetad2_f = subs(f(5), [Nm, Pm, P], [Nm_c, Pm_c, P_c]);
phidot2_f = subs(f(8), [Nm, Pm, P], [Nm_c, Pm_c, P_c]);
f_nl = [xdot2_f == 0;thetad2_f;phidot2_f];
[xdot2_c, thetad2_c, phidot2_c] = solve(f_nl, [xdot2, thetad2, phidot2]); 

%% ���״̬�ռ���ʽ
% ״̬�������������
X = [theta; thetad1; x; xdot1; phi; phidot1];
U = [T; Tp];
Xdot = [thetad1; thetad2_c; xdot1; xdot2_c; phidot1; phidot2_c];

% ͨ��jacobian�����õ�A��B����
A = jacobian(Xdot, X);
B = jacobian(Xdot, U);

% ����ƽ����ֵ����ƽ��㴦�������Ի�
A_ballance = subs(A, [theta, thetad1, x, xdot1, phi, phidot1, T, Tp], [0, 0, 0, 0, 0, 0, 0, 0]);
vpa(A_ballance);
A_ballance = double(A_ballance);

B_ballance = subs(B, [theta, thetad1, x, xdot1, phi, phidot1, T, Tp], [0, 0, 0, 0, 0, 0, 0, 0]);
vpa(B_ballance);
B_ballance = double(B_ballance);

end