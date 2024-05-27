tic
fitfun = @GA_LQRD;%适应度函数
dim=4 ;                 %维度变量
Max_iteration=50;     %迭代次数
SearchAgents_no=20;
% SearchAgents_no=50; %个体数量
lb=1;                %下限变量
ub=50;                 %上限变量
% tlt='Chung Reynolds';
% i=1;
[Xfood, Xvalue,CNVG] = SO(SearchAgents_no,Max_iteration,fitfun, dim,lb,ub);
figure,
plot(CNVG,"Color","r")
% xlim([1 1000]);%设置坐标轴范围
toc



function f=GA_LQRD(x)
%%%%%%%%%% 模型参数  %%%%%%%%%%%%%%%%
 m = 1270+71*2;%整车质量
 a = 1.015;    %质心到前轴的距离
 b = 1.895;    %质心到后轴的距离
 L = a+b;      %轴距
 Iz = 1536.7;  %绕z轴的转动惯量
 k1 = -112600; %前轴侧偏刚度
 k2 = -89500;  %后轴侧偏刚度
 vx = 50/3.6;  %车速
 A = [0,vx,1,0;
      0,0,0,1;
      0,0,(k1+k2)/m/vx,(a*k1-b*k2)/m/vx-vx;
      0,0,(a*k1-b*k2)/Iz/vx,(a^2*k1+b^2*k2)/Iz/vx];
 B = [0;0;-k1/m;-a*k1/Iz];
%  Q = [x(1),0,0,0;
%       0,x(2),0,0;
%       0,0,x(3),0;
%       0,0,0,x(4)];
  Q = [x(1),0,0,0;
      0,x(2),0,0;
      0,0,x(3),0;
      0,0,0,x(4)];
 R = 5;
 K=lqr(A,B,Q,R);

 %%%%%运行 simulink 模型%%%%%%
 assignin("base","K",K);
 [t_time,x_state,y1,y2,y3]=sim("test1",[0,20]);
%  out= sim('test1.slx',[0,20]);

 %%%%%%%%% 计算优化目标适应度函数%%%%%%%%%
% ED_RMS = sqrt(sum(y1.*y1)/size(y1,1));%size(a,n)n=1返回n的行数，n=2返回列数
% Ephi_RMS = sqrt(sum(y2.*y2)/size(y2,1));
% delt_RMS = sqrt(sum(y3.*y3)/size(y3,1));
%%% 标准差%%%
std_ed = std(y1);
std_Ephi = std(y2);
std_delt = std(y3);
%%%均值%%%
ED_U = sum(y1)/size(y1,1);
Ephi_U = sum(y1)/size(y1,1);
delt_U = sum(y1)/size(y1,1);

f = sum(abs((y1 -ED_U)/std_ed))+sum(abs(y2 -Ephi_U)/std_Ephi)+sum(abs((y3 -delt_U)/std_delt));
end