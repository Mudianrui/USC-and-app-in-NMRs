function [sys,x0,str,ts] = Scontroller(t,x,u,flag)
%% Scontroller
switch flag
case 0
    [sys,x0,str,ts]=mdlInitializeSizes;
case 3
    sys=mdlOutputs(t,x,u);
case {1,2,4,9}
    sys=[];
otherwise
    error(['Unhandled flag = ',num2str(flag)]);
end

function [sys,x0,str,ts]=mdlInitializeSizes
%% mdlInitializeSizes
sizes = simsizes;
sizes.NumContStates  = 0;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 28;
sizes.NumInputs      = 12;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 0;
sys = simsizes(sizes);
x0  = [];
str = [];
ts  = [];

function sys=mdlOutputs(~,~,u)
%% input
t = u(1);
xr = u(2);
yr = u(3);
thr = u(4);
vr = u(5);
wr = u(6);
ep = u(7);
x = u(8);
y = u(9);
th = u(10);
v = u(11);
w = u(12);

%% controller
AAtype = 1;%0 automatic control/1 shared control
T = 5;
Lw = 0.05;%Lane constraint: half of lane width

%% ACC
kv = 100;
vlow = 0.5-0.6*exp(-t);
vup = 1.6+0.2*sin(t);
if AAtype==0
    kappav = 0.8;
    vc = (1-kappav)*vlow+kappav*vup;
else
    vc = 1.2-1*cos(0.7*t);% vr;
end

zetav = constraint_transformation(v,vlow,vup,0,0);
zetavd = constraint_transformation(vc,vlow,vup,0.1,1);
uv = -kv*(zetav-zetavd);
% uv = -kv*(v-vc);

%% LK
kth = 10;
kpi = 0.2;
rho_th0 = pi;
epsilon_th = kpi*pi;
rhoth = (rho_th0-epsilon_th)*smoothTfun1((T-t)/T)+epsilon_th;
ths = thr+4*kpi*atan(ep/Lw);

kbwmax = 0.3;
kbwmin = 0.1;
the = thNormalization(th-thr);
vp = v*sin(the);
vpmin = 0.1;
kbw = kbwmin+(kbwmax-kbwmin)*trns101(vp/vpmin,1);

if ths-th>=pi
    ths = ths-2*pi;
elseif ths-th<=-pi
    ths = ths+2*pi;
end

if AAtype==0
    thc = ths;
else
    thc = ths+0.3*pi*sin(t);
end
zetath = constraint_transformation(th,ths-rhoth,ths+rhoth,0,0);
zetathd = constraint_transformation(thc,ths-rhoth,ths+rhoth,kbw*rhoth,1);
wc = -kth*(zetath-zetathd);

%% uw
kw = 10;
rho_w0 = 100;
epsilon_w = 10;
rhow = (rho_w0-epsilon_w)*smoothTfun1((T-t)/T)+epsilon_w;
ST = 1-smoothTfun1((T-t)/T);
zw = ST*(w-wc);
zetaw = constraint_transformation(zw,-rhow,rhow,0,0);
uw = -kw*zetaw;


%% output
D = [1 1;1 -1];
url = D\[uv;uw];
sys(1) = url(1);
sys(2) = url(2);
sys(3) = t;
sys(4) = xr;
sys(5) = yr;
sys(6) = thr;
sys(7) = vr;
sys(8) = wr;
sys(9) = ep;
sys(10) = x;
sys(11) = y;
sys(12) = th;
sys(13) = v;
sys(14) = w;
sys(15) = -Lw;
sys(16) = Lw;
sys(17) = vc;
sys(18) = vlow;
sys(19) = vup;
sys(20) = ths;
sys(21) = thc;
sys(22) = ths-rhoth;
sys(23) = ths+rhoth;
sys(24) = zw;
sys(25) = -rhow;
sys(26) = rhow;
sys(27) = uv;
sys(28) = uw;
