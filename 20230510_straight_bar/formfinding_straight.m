clear;close all;clc;
%% parameter setting
l_bar = 0.15;      % half bar 15cm
k_bar = 10000;   % solid bar
k_spring = 200;    % 200 newton/m
l_spring = 0.1;    % spring rest lenght 10cm
m_bar = 0.02;      % bar mass 20g
m_motor = 0.09;    % motor mass 90g

steps = 1000;      % max iteration steps
dt = 0.00001;       % time steps
threshold = 0.00001; % threshold for quitting iteration
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
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% pre-initialization of xs_ini
% initialization x1 - x12 (for straight bars)
cable_butt_length_ini = sqrt(6)/4*(l_bar*2);
bar_length = 2*l_bar; % bar lengths are fixed and decided

x1_ini = [0;0;0]; % points 1 2 3 on the ground
x2_ini = [cable_butt_length_ini/2;cable_butt_length_ini*sqrt(3)/2;0];
x3_ini = [cable_butt_length_ini;0;0];

alpha_z = 40; % alphas are random
alpha_17 = 80;
alpha_28 = -40;
alpha_39 = -160;

x7_ini = the_other_node(x1_ini, bar_length, alpha_17, alpha_z);
x8_ini = the_other_node(x2_ini, bar_length, alpha_28, alpha_z);
x9_ini = the_other_node(x3_ini, bar_length, alpha_39, alpha_z);

x4_ini = [-cable_butt_length_ini/2;cable_butt_length_ini*sqrt(3)/2;bar_length*sin(alpha_z)];
x5_ini = [3*cable_butt_length_ini/2;cable_butt_length_ini*sqrt(3)/2;bar_length*sin(alpha_z)];
x6_ini = [cable_butt_length_ini/2;-cable_butt_length_ini*sqrt(3)/2;bar_length*sin(alpha_z)];

alpha_410 = alpha_28;
alpha_511 = alpha_39;
alpha_612 = alpha_17;

x10_ini = the_other_node(x4_ini, bar_length, alpha_410, alpha_z);
x11_ini = the_other_node(x5_ini, bar_length, alpha_511, alpha_z);
x12_ini = the_other_node(x6_ini, bar_length, alpha_612, alpha_z);

% figure()
% Xs = [x1_ini,x2_ini,x3_ini,x4_ini,x5_ini,x6_ini,x7_ini,x8_ini,x9_ini,x10_ini,x11_ini,x12_ini];
% plot3(Xs(1,:),Xs(2,:),Xs(3,:),'o')
% axis equal

% iteration
steps_pre = 50000;
xs = zeros(3,12,steps_pre+1);
xs(:,:,1) = [x1_ini,x2_ini,x3_ini,x4_ini,x5_ini,x6_ini,x7_ini,x8_ini,x9_ini,x10_ini,x11_ini,x12_ini];
for i=1:steps_pre
    xc = xs(:,:,i);
    dx = zeros(3,12);
    for j = 1:12
        for k = 1:12
            if connection(j,k) == 1 % cable connection
                dx(1:3,j) = cable(xc(1:3,j), xc(1:3,k), l_spring, k_spring) + dx(1:3,j);
            elseif connection(j,k) == -1 % strut connection
                dx(1:3,j) = bar(xc(1:3,j), xc(1:3,k), l_bar*2, k_bar) + dx(1:3,j);
            end
        end
        dx(3,j) = -0.5*(m_bar + m_motor) * 9.98 + dx(3,j); % add gravity
    end
    dx(3,1:3) = [0,0,0];% points 1 2 3 on the ground
    xc = xc + dx*dt;
    xs(:,:,i+1) = xc;
    
    if (norm(xs(:,:,i+1) - xs(:,:,i))) < threshold
        break
    end
end
xs_ini = xs(:,:,i+1);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% iterations
xs = zeros(3,12,steps+1);
xs(:,:,1) = xs_ini;
for i=1:steps
   xc = xs(:,:,i);
    dx = zeros(3,12);
    for j = 1:12
        for k = 1:12
            if connection(j,k) == 1 % cable connection
                dx(1:3,j) = cable(xc(1:3,j), xc(1:3,k), l_spring, k_spring) + dx(1:3,j);
            elseif connection(j,k) == -1 % strut connection
                dx(1:3,j) = bar(xc(1:3,j), xc(1:3,k), l_bar*2, k_bar) + dx(1:3,j);
            end
        end
        dx(3,j) = -0.5*(m_bar + m_motor) * 9.98 + dx(3,j); % add gravity
    end
    dx(3,1:3) = [0,0,0];% points 1 2 3 on the ground
    xc = xc + dx*dt;
    xs(:,:,i+1) = xc;
end
% %% (plot difference)
% figure(1);
% diffs = zeros(1,steps);
% for i = 2:steps+1
%     diffs(1,i-1) = norm(xs(:,:,i) - xs(:,:,i-1));
% end
% plot(diffs);

%% for future use
x1 = 0.5* (xs(:,1,steps) + xs(:,7,steps));
x2 = 0.5* (xs(:,2,steps) + xs(:,8,steps));
x3 = 0.5* (xs(:,3,steps) + xs(:,9,steps));
x4 = 0.5* (xs(:,4,steps) + xs(:,10,steps));
x5 = 0.5* (xs(:,5,steps) + xs(:,11,steps));
x6 = 0.5* (xs(:,6,steps) + xs(:,12,steps));

xs_init = xs(:,:,steps);
%% visualization
figure(2);
set(figure(2),'Position',[500,100,800,700]);
p = plot3(1,1,1,'o');
for i = 1:steps
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
     if i == steps
         pause(100);
     end

    delete(bar_17);delete(bar_28);delete(bar_39);delete(bar_410);delete(bar_511);delete(bar_612);
    
    delete(str_12);delete(str_13);delete(str_14);delete(str_19);delete(str_23);delete(str_25);
    delete(str_27);delete(str_36);delete(str_38);delete(str_47);delete(str_49);delete(str_411);
    delete(str_57);delete(str_58);delete(str_512);delete(str_68);delete(str_69);delete(str_610);
    delete(str_711);delete(str_812);delete(str_910);delete(str_1011);delete(str_1012);delete(str_1112);
    
    hold off;
end



%% functions
function [p] = the_other_node(node, bar, theta1, theta2)

bar_0 = [bar;0;0];
rot_1 = [cos(theta1/180*pi) -sin(theta1/180*pi) 0;sin(theta1/180*pi) cos(theta1/180*pi) 0;0 0 1];
bar_1 = rot_1*bar_0;

p2(1,1) = bar_1(1)*cos(theta2/180*pi);
p2(2,1) = bar_1(2)*cos(theta2/180*pi);
p2(3,1) = bar*sin(theta2/180*pi);

p = node+p2;
end

function [f] = cable(p1,p2,l, k)
f =- k*(l-norm(p1-p2))*((p2-p1)/norm(p2-p1));
end

function [f] = bar(p1,p2,l, k)
f = -k*(l-norm(p1-p2))*((p2-p1)/norm(p2-p1));
end

