
%------------- Inverse Kinematics of 'pushbutton' ----------------

% input: the center position of the top panel 
% ouput: 

function [phi1,phi2,phi3]=Inverse_Kinematics_pushbutton(x,y,z)
    
    [delta,psi,r0]=cart2pol(x,y,z);

    % r the radius of the hexagon's inscribed circle
    r=0.06;%m
    % the length of the leg
    l=0.06; %m
    % the center location of the base
    Ob=[0,0,0];
    
    % the location of the waterbomb center bi
    theta=[0,2*pi/3,4*pi/3];
    
    a=[0,0,0];
    d=[0,0,0];
    e=[0,0,0];
    % derive the second order polynomial equation for each leg
    
    for i=1:3
        a(i)=(r-l)*(sin(psi)*cos(delta-theta(i)))-(r0/2);
        d(i)=2*l*cos(psi);
        e(i)=sin(psi)*cos(delta-theta(i))*l+r*sin(psi)*cos(delta-theta(i))-r0/2;
        syms t1
        f=a(i)*t1^2+d(i)*t1+e(i);
        A(i,:)=vpa(solve(f==0))
    end
   

    % calculate the leg angle
    phi1=(2*atan(A(1,1)))/pi*180
    phi2=(2*atan(A(2,1)))/pi*180
    phi3=(2*atan(A(3,1)))/pi*180

    % second solution
    phi1b=(2*atan(A(1,2)))/pi*180
    phi2b=(2*atan(A(2,2)))/pi*180
    phi3b=(2*atan(A(3,2)))/pi*180
end
