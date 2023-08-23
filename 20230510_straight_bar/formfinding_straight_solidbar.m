clear;close all;clc;
% consider a solid bar here
% each bar is a P = [x y z], A = [p, q, r], instead of 2 nodes
% each bar is described with x = [px;py;pz], q = quaternion
% difference: consider the bar as a solid ==> k_bar = inf
%% parameter setting
l_bar = 0.3;      % bar length 30cm
k_spring = 200;    % 200 newton/m
l_spring = 0.1;    % spring rest lenght 10cm
m_bar = 0.02;      % bar mass 20g
m_motor = 0.09;    % motor mass 90g, let's suppose motor in the center
steps = 2000;      % max iteration steps
dt = 0.001;       % time steps
%% initialization connection matrix
connection = zeros(12,12);
%                  1  2  3  4  5  6  7  8  9 10 11 12   1 cable   -1 strut
connection(1,:) = [0  1  1  1  0  0  -1 0  1  0  0  0];
connection(2,:) = [1  0  1  0  1  0  1  -1 0  0  0  0];
connection(3,:) = [1  1  0  0  0  1  0  1 -1  0  0  0];
connection(4,:) = [1  0  0  0  0  0  1  0  1 -1  1  0];
connection(5,:) = [0  1  0  0  0  0  1  1  0  0 -1  1];
connection(6,:) = [0  0  1  0  0  0  0  1  1  1  0 -1];
connection(7,:) = [-1 1  0  1  1  0  0  0  0  0  1  0];
connection(8,:) = [0 -1  1  0  1  1  0  0  0  0  0  1];
connection(9,:) = [1  0 -1  1  0  1  0  0  0  1  0  0];
connection(10,:)= [0  0  0 -1  0  1  0  0  1  0  1  1];
connection(11,:)= [0  0  0  1 -1  0  1  0  0  1  0  1];
connection(12,:)= [0  0  0  0  1 -1  0  1  0  1  1  0];
%% initialization of x1-x6, q1-q6
xs_init =[0.0144    0.0683    0.1928   -0.0492    0.2296    0.0952    0.0882    0.2345   -0.0472    0.1136   -0.0081    0.1701;
          -0.0188    0.1560    0.0219    0.1306    0.1364   -0.1079    0.2157   -0.0251   -0.0314   -0.0499    0.0857    0.1233;
          0         0         0    0.0799    0.0799    0.0799    0.1648    0.1648    0.1648    0.2489    0.2489    0.2489];
xs_init = xs_init+rand(3,12)*0.5;
[x1,x2,x3,x4,x5,x6] = init_x(xs_init);
[q1,q2,q3,q4,q5,q6] = init_q(xs_init);
%% iterations
xs = zeros(3,12,steps+1);
xs(:,:,1) = get_xs(l_bar,x1,x2,x3,x4,x5,x6,q1,q2,q3,q4,q5,q6);

for i=1:steps
    xc = xs(:,:,i);
    f = zeros(3,12);
    for j = 1:12
        for k = 1:12
            if connection(j,k) == 1 % cable connection
                f(1:3,j) = cable(xc(1:3,j), xc(1:3,k), l_spring, k_spring) + f(1:3,j);
            end
        end
    end
    [x1,x2,x3,x4,x5,x6] = update_x(x1,x2,x3,x4,x5,x6,f,dt);
    [q1,q2,q3,q4,q5,q6] = update_q(q1,q2,q3,q4,q5,q6,f,dt,l_bar);
    xs(:,:,i+1) = get_xs(l_bar,x1,x2,x3,x4,x5,x6,q1,q2,q3,q4,q5,q6);
end
%% visualization
figure(2);
set(figure(2),'Position',[500,100,800,700]);
p = plot3(1,1,1,'o');
for i = 1:10:steps
    axis([-5 15 -5 15 -5 15]/30);
    axis equal;
    hold on;
    x_plot =  [xs(1,1,i), xs(1,2,i), xs(1,3,i), xs(1,4,i), xs(1,5,i), xs(1,6,i), xs(1,7,i), xs(1,8,i), xs(1,9,i), xs(1,10,i), xs(1,11,i), xs(1,12,i)];
    y_plot =  [xs(2,1,i), xs(2,2,i), xs(2,3,i), xs(2,4,i), xs(2,5,i), xs(2,6,i), xs(2,7,i), xs(2,8,i), xs(2,9,i), xs(2,10,i), xs(2,11,i), xs(2,12,i)];
    z_plot =  [xs(3,1,i), xs(3,2,i), xs(3,3,i), xs(3,4,i), xs(3,5,i), xs(3,6,i), xs(3,7,i), xs(3,8,i), xs(3,9,i), xs(3,10,i), xs(3,11,i), xs(3,12,i)];
    set (p, 'XData', x_plot, 'YData', y_plot,'ZData', z_plot );
    
    bar_17 = line([xs(1,1,i) xs(1,7,i)],[xs(2,1,i) xs(2,7,i)],[xs(3,1,i) xs(3,7,i)],'linestyle','-','color','r','LineWidth',3);
    bar_28 = line([xs(1,2,i) xs(1,8,i)],[xs(2,2,i) xs(2,8,i)],[xs(3,2,i) xs(3,8,i)],'linestyle','-','color','r','LineWidth',3);
    bar_39 = line([xs(1,3,i) xs(1,9,i)],[xs(2,3,i) xs(2,9,i)],[xs(3,3,i) xs(3,9,i)],'linestyle','-','color','r','LineWidth',3);
    bar_410 = line([xs(1,4,i) xs(1,10,i)],[xs(2,4,i) xs(2,10,i)],[xs(3,4,i) xs(3,10,i)],'linestyle','-','color','r','LineWidth',3);
    bar_511 = line([xs(1,5,i) xs(1,11,i)],[xs(2,5,i) xs(2,11,i)],[xs(3,5,i) xs(3,11,i)],'linestyle','-','color','r','LineWidth',3);
    bar_612 = line([xs(1,6,i) xs(1,12,i)],[xs(2,6,i) xs(2,12,i)],[xs(3,6,i) xs(3,12,i)],'linestyle','-','color','r','LineWidth',3);
    
    str_12 = line([xs(1,1,i) xs(1,2,i)],[xs(2,1,i) xs(2,2,i)],[xs(3,1,i) xs(3,2,i)]);
    str_13 = line([xs(1,1,i) xs(1,3,i)],[xs(2,1,i) xs(2,3,i)],[xs(3,1,i) xs(3,3,i)]);
    str_14 = line([xs(1,1,i) xs(1,4,i)],[xs(2,1,i) xs(2,4,i)],[xs(3,1,i) xs(3,4,i)]);
    str_19 = line([xs(1,1,i) xs(1,9,i)],[xs(2,1,i) xs(2,9,i)],[xs(3,1,i) xs(3,9,i)]);
    
    str_23 = line([xs(1,2,i) xs(1,3,i)],[xs(2,2,i) xs(2,3,i)],[xs(3,2,i) xs(3,3,i)]);
    str_25 = line([xs(1,2,i) xs(1,5,i)],[xs(2,2,i) xs(2,5,i)],[xs(3,2,i) xs(3,5,i)]);
    str_27 = line([xs(1,2,i) xs(1,7,i)],[xs(2,2,i) xs(2,7,i)],[xs(3,2,i) xs(3,7,i)]);
    
    str_36 = line([xs(1,3,i) xs(1,6,i)],[xs(2,3,i) xs(2,6,i)],[xs(3,3,i) xs(3,6,i)]);
    str_38 = line([xs(1,3,i) xs(1,8,i)],[xs(2,3,i) xs(2,8,i)],[xs(3,3,i) xs(3,8,i)]);
    
    str_47 = line([xs(1,4,i) xs(1,7,i)],[xs(2,4,i) xs(2,7,i)],[xs(3,4,i) xs(3,7,i)]);
    str_49 = line([xs(1,4,i) xs(1,9,i)],[xs(2,4,i) xs(2,9,i)],[xs(3,4,i) xs(3,9,i)]);
    str_411 = line([xs(1,4,i) xs(1,11,i)],[xs(2,4,i) xs(2,11,i)],[xs(3,4,i) xs(3,11,i)]);
    
    str_57 = line([xs(1,5,i) xs(1,7,i)],[xs(2,5,i) xs(2,7,i)],[xs(3,5,i) xs(3,7,i)]);
    str_58 = line([xs(1,5,i) xs(1,8,i)],[xs(2,5,i) xs(2,8,i)],[xs(3,5,i) xs(3,8,i)]);
    str_512 = line([xs(1,5,i) xs(1,12,i)],[xs(2,5,i) xs(2,12,i)],[xs(3,5,i) xs(3,12,i)]);
    
    str_68 = line([xs(1,6,i) xs(1,8,i)],[xs(2,6,i) xs(2,8,i)],[xs(3,6,i) xs(3,8,i)]);
    str_69 = line([xs(1,6,i) xs(1,9,i)],[xs(2,6,i) xs(2,9,i)],[xs(3,6,i) xs(3,9,i)]);
    str_610 = line([xs(1,6,i) xs(1,10,i)],[xs(2,6,i) xs(2,10,i)],[xs(3,6,i) xs(3,10,i)]);
    
    str_711 = line([xs(1,7,i) xs(1,11,i)],[xs(2,7,i) xs(2,11,i)],[xs(3,7,i) xs(3,11,i)]);
    
    str_812 = line([xs(1,8,i) xs(1,12,i)],[xs(2,8,i) xs(2,12,i)],[xs(3,8,i) xs(3,12,i)]);
    
    str_910 = line([xs(1,9,i) xs(1,10,i)],[xs(2,9,i) xs(2,10,i)],[xs(3,9,i) xs(3,10,i)]);
    
    str_1011 = line([xs(1,10,i) xs(1,11,i)],[xs(2,10,i) xs(2,11,i)],[xs(3,10,i) xs(3,11,i)]);
    str_1012 = line([xs(1,10,i) xs(1,12,i)],[xs(2,10,i) xs(2,12,i)],[xs(3,10,i) xs(3,12,i)]);
    
    str_1112 = line([xs(1,11,i) xs(1,12,i)],[xs(2,11,i) xs(2,12,i)],[xs(3,11,i) xs(3,12,i)]);
       
    drawnow;
    if i == steps-9
         pause(5);
    end

    delete(bar_17);delete(bar_28);delete(bar_39);delete(bar_410);delete(bar_511);delete(bar_612);
    
    delete(str_12);delete(str_13);delete(str_14);delete(str_19);delete(str_23);delete(str_25);
    delete(str_27);delete(str_36);delete(str_38);delete(str_47);delete(str_49);delete(str_411);
    delete(str_57);delete(str_58);delete(str_512);delete(str_68);delete(str_69);delete(str_610);
    delete(str_711);delete(str_812);delete(str_910);delete(str_1011);delete(str_1012);delete(str_1112);
    
    hold off;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [x1,x2,x3,x4,x5,x6] = init_x(xs)
x1 = 0.5* (xs(:,1) + xs(:,7));
x2 = 0.5* (xs(:,2) + xs(:,8));
x3 = 0.5* (xs(:,3) + xs(:,9));
x4 = 0.5* (xs(:,4) + xs(:,10));
x5 = 0.5* (xs(:,5) + xs(:,11));
x6 = 0.5* (xs(:,6) + xs(:,12));
end
function [q1,q2,q3,q4,q5,q6] = init_q(xs)
a1 = xs(:,7) - xs(:,1);
a2 = xs(:,8) - xs(:,2);
a3 = xs(:,9) - xs(:,3);
a4 = xs(:,10) - xs(:,4);
a5 = xs(:,11) - xs(:,5);
a6 = xs(:,12) - xs(:,6);
q1 = quaternion([cos(norm(a1)/2) a1'/norm(a1)*sin(norm(a1)/2)]);
q2 = quaternion([cos(norm(a2)/2) a2'/norm(a2)*sin(norm(a2)/2)]);
q3 = quaternion([cos(norm(a3)/2) a3'/norm(a3)*sin(norm(a3)/2)]);
q4 = quaternion([cos(norm(a4)/2) a4'/norm(a4)*sin(norm(a4)/2)]);
q5 = quaternion([cos(norm(a5)/2) a5'/norm(a5)*sin(norm(a5)/2)]);
q6 = quaternion([cos(norm(a6)/2) a6'/norm(a6)*sin(norm(a6)/2)]);
end

function [xs] = get_xs(l_bar,x1,x2,x3,x4,x5,x6,q1,q2,q3,q4,q5,q6)
vec1 = rotvec(q1)/norm(rotvec(q1));
vec2 = rotvec(q2)/norm(rotvec(q2));
vec3 = rotvec(q3)/norm(rotvec(q3));
vec4 = rotvec(q4)/norm(rotvec(q4));
vec5 = rotvec(q5)/norm(rotvec(q5));
vec6 = rotvec(q6)/norm(rotvec(q6));

p1 = x1 - 0.5*l_bar*vec1';p7 = x1 + 0.5*l_bar*vec1';
p2 = x2 - 0.5*l_bar*vec2';p8 = x2 + 0.5*l_bar*vec2';
p3 = x3 - 0.5*l_bar*vec3';p9 = x3 + 0.5*l_bar*vec3';
p4 = x4 - 0.5*l_bar*vec4';p10 = x4 + 0.5*l_bar*vec4';
p5 = x5 - 0.5*l_bar*vec5';p11 = x5 + 0.5*l_bar*vec5';
p6 = x6 - 0.5*l_bar*vec6';p12 = x6 + 0.5*l_bar*vec6';

xs = [p1,p2,p3,p4,p5,p6,p7,p8,p9,p10,p11,p12];
end

function [f] = cable(p1,p2,l, k)
f =- k*(l-norm(p1-p2))*((p2-p1)/norm(p2-p1));
end

function [x1_,x2_,x3_,x4_,x5_,x6_] = update_x(x1,x2,x3,x4,x5,x6,f,dt)
F = zeros(3,6);
for b = 1:6
    F(:,b) = f(1:3,b) + f(1:3,b+6);
end
F = F*dt*0.1;
x1_ = x1 + F(:,1);
x2_ = x2 + F(:,2);
x3_ = x3 + F(:,3);
x4_ = x4 + F(:,4);
x5_ = x5 + F(:,5);
x6_ = x6 + F(:,6);
end

function [q1_,q2_,q3_,q4_,q5_,q6_] = update_q(q1,q2,q3,q4,q5,q6,f,dt,l_bar)
T = zeros(3,6);
qs = [q1,q2,q3,q4,q5,q6];
for b = 1:6
    T(:,b) = cross(-0.5*l_bar*rotvec(qs(b))/norm(rotvec(qs(b))),f(:,b))+ cross(0.5*l_bar*rotvec(qs(b))/norm(rotvec(qs(b))),f(:,b+6));
end
T = T*dt;
q1_ = quaternion([cos(norm(T(:,1))/2) T(:,1)'/norm(T(:,1))*sin(norm(T(:,1))/2)])*q1;
q2_ = quaternion([cos(norm(T(:,2))/2) T(:,2)'/norm(T(:,2))*sin(norm(T(:,2))/2)])*q2;
q3_ = quaternion([cos(norm(T(:,3))/2) T(:,3)'/norm(T(:,3))*sin(norm(T(:,3))/2)])*q3;
q4_ = quaternion([cos(norm(T(:,4))/2) T(:,4)'/norm(T(:,4))*sin(norm(T(:,4))/2)])*q4;
q5_ = quaternion([cos(norm(T(:,5))/2) T(:,5)'/norm(T(:,5))*sin(norm(T(:,5))/2)])*q5;
q6_ = quaternion([cos(norm(T(:,6))/2) T(:,6)'/norm(T(:,6))*sin(norm(T(:,6))/2)])*q6;
end
