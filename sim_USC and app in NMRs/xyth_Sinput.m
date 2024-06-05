function [sys,x0,str,ts] = xyth_Sinput(t,x,u,flag)
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
sizes = simsizes;
sizes.NumContStates  = 0;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 7;
sizes.NumInputs      = 4;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 0;
sys = simsizes(sizes);
x0  = [];
str = [];
ts  = [];

function sys=mdlOutputs(~,~,u)
t = u(1);

vd = 0.6+0.5*cos(0.7*t);
isClockwise = 1;%1 clockwise, -1 counterclockwise
xe0 = 0;
ye0 = 0;
re = 2;
Le = 2;
flag = 1;
switch floor(flag)
    case 0%Time independent circular runway - lateral
        xt = u(2);
        yt = u(3);
        %tht = x(4);
        if xt<xe0-Le/2
            ep = re-sqrt((xt-(xe0-Le/2))^2+(yt-ye0)^2);
            th = atan2(yt-ye0,xt-(xe0-Le/2));
            thd = th-pi/2;
            xd = re*cos(th);
            yd = re*sin(th);
            if xd>0
                xd = xe0-Le/2-xd;
                yd = ye0-yd;
            else
                xd = xe0-Le/2+xd;
                yd = ye0+yd;
            end
            wd = re*vd;
        elseif xt>xe0+Le/2
            ep = re-sqrt((xt-(xe0+Le/2))^2+(yt-ye0)^2);
            th = atan2(yt-ye0,xt-(xe0+Le/2));
            thd = th-pi/2;
            xd = re*cos(th);
            yd = re*sin(th);
            if xd<0
                xd = xe0+Le/2-xd;
                yd = ye0-yd;
            else
                xd = xe0+Le/2+xd;
                yd = ye0+yd;
            end
            wd = re*vd;
        else
            if yt>ye0
                ep = (ye0+re)-yt;
                thd = 0;
                xd = xt;
                yd = re;
                wd = 0;
            else
                ep = yt-(ye0-re);
                thd = -pi;
                xd = xt;
                yd = -re;
                wd = 0;
            end
        end

    case 1%Time independent circular runway - longitudinal
        xt = u(2);
        yt = u(3);
        %tht = x(4);
        if yt<ye0-Le/2
            ep = re-sqrt((xt-xe0)^2+(yt-(ye0-Le/2))^2);
            th = atan2(yt-(ye0-Le/2),xt-xe0);
            thd = th-pi/2;
            xd = re*cos(th);
            yd = re*sin(th);
            if yd>0
                xd = xe0-xd;
                yd = ye0-Le/2-yd;
            else
                xd = xe0+xd;
                yd = ye0-Le/2+yd;
            end
            wd = re*vd;
        elseif yt>ye0+Le/2
            ep = re-sqrt((xt-xe0)^2+(yt-(ye0+Le/2))^2);
            th = atan2(yt-(ye0+Le/2),xt-xe0);
            thd = th-pi/2;
            xd = re*cos(th);
            yd = re*sin(th);
            if yd<0
                xd = xe0-xd;
                yd = ye0+Le/2-yd;
            else
                xd = xe0+xd;
                yd = ye0+Le/2+yd;
            end
            wd = re*vd;
        else
            if xt>xe0
                ep = (xe0+re)-xt;
                thd = -pi/2;
                xd = xe0+re;
                yd = yt;
                wd = 0;
            else
                ep = xt-(xe0-re);
                thd = pi/2;
                xd = xe0-re;
                yd = yt;
                wd = 0;
            end
        end
    otherwise
        error(['Unhandled flag = ',num2str(flag)]);
end

if isClockwise==1
    thd = thNormalization(thd);
else
    thd = thNormalization(thd+pi);
end
sys(1)=t;
sys(2)=xd;
sys(3)=yd;
sys(4)=thd;
sys(5)=vd;
sys(6)=wd;
sys(7)=isClockwise*ep;


