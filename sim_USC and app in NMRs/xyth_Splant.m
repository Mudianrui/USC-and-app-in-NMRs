function [sys,x0,str,ts] = xyth_Splant(t,x,u,flag)
switch flag
case 0
    [sys,x0,str,ts]=mdlInitializeSizes;
case 1
    sys=mdlDerivatives(t,x,u);
case 3
    sys=mdlOutputs(t,x,u);
case {2,4,9}
    sys=[];
otherwise
    error(['Unhandled flag = ',num2str(flag)]);
end

function [sys,x0,str,ts]=mdlInitializeSizes
sizes = simsizes;
sizes.NumContStates  = 3;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 3;
sizes.NumInputs      = 2;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;
sys = simsizes(sizes);
x0  = [-2.5,-2.50,2*pi/4];
str = [];
ts  = [0 0];

function sys=mdlDerivatives(~,x,u)
v = u(1);
w = u(2);
th = x(3);
sys(1) = v*cos(th);
sys(2) = v*sin(th);
sys(3) = w;

function sys=mdlOutputs(~,x,~)
sys(1) = x(1);
sys(2) = x(2);
sys(3) = thNormalization(x(3));
