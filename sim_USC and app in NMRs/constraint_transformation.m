function vir = constraint_transformation(x,cLow,cUp,bw,flag)
if flag/2==0
    if x<=cLow
        lflag = flag;
        flag = -1;
    elseif x>=cUp
        lflag = flag;
        flag = -2;
    end
elseif cLow>=cUp
    lflag = flag;
    flag = -3;
elseif bw<0
    lflag = flag;
    flag = -4;
end

switch flag
    case 0
        kap = (cLow-cUp)^2/4;
        bate = -(x-cLow)*(x-cUp);
        zeta = kap*(x-(cLow+cUp)/2)/bate;
        vir = zeta;
        
    case 1
        kap = (cLow-cUp)^2/4;
        if x<cLow+bw
            xdc = cLow+bw;
            bated = -(xdc-cLow)*(xdc-cUp);
            muid = kap*((xdc-(cLow+cUp)/2)^2+(cLow-cUp)^2/4)/(bated^2);
            arf0 = muid*(x-(cLow+bw))+kap*(xdc-(cLow+cUp)/2)/bated;
        elseif x>cUp-bw
            xdc = cUp-bw;
            bated = -(xdc-cLow)*(xdc-cUp);
            muid = kap*((xdc-(cLow+cUp)/2)^2+(cLow-cUp)^2/4)/(bated^2);
            arf0 = muid*(x-(cUp-bw))+kap*(xdc-(cLow+cUp)/2)/bated;
        else
            bated = -(x-cLow)*(x-cUp);
            arf0 = kap*(x-(cLow+cUp)/2)/bated;
        end
        vir = arf0;
        
    otherwise
        error(['Unhandled flag from constraint_transformation: errorflag =',num2str(flag),',flag=',num2str(lflag),',x=',num2str(x),',cLow=',num2str(cLow),',cUp=',num2str(cUp),'.']);
end
