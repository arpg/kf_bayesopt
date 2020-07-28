function upload2Dplot(input_f,output_f,it,ini)
%upload2Dplot('data/2Ddata3_ekfslam_NIS','plot/2Dplot3_ekfslam_NIS',20,20)
%it number of iterations-1
%ini number of initial sample
%input_add input folder such as T1 folder
%default in each csv file the first 1000 line is from gaussian process
inname_first     = input_f;
inname_middle = '/iteration';
inname_last     = '.csv';
ouname_first    = output_f;
ouname_middle = '/iteration';
ouname_last     = '.jpg';%'.eps', eps is big, black and white

samx = 50;
samy = 50;
%xm = linspace(0,0.98,samx);
%ym = xm;

%  vedio_out = [ouname_first,'/vedio.avi'];
%  myObj = VideoWriter(vedio_out);
%  myObj.FrameRate = 5;
%  open(myObj);
i = 99;
inname = [inname_first,inname_middle,num2str(i),inname_last];
ouname= [ouname_first,ouname_middle,num2str(i),ouname_last];
M = csvread(inname);
le = length(M);

G = samx*samy;

x_range_min = min(M(1:G,1));
y_range_min = min(M(1:G,2));
x_range_max = max(M(1:G,1));
y_range_max = max(M(1:G,2));

xm = linspace(x_range_min,x_range_max,samx);
ym = linspace(y_range_min,y_range_max,samy);
[X,Y] = meshgrid(xm,ym);
%My = zeros(samx,samy);
%Mc = zeros(samx,samy);
%for i = 0:it

%    x = M(1:G,1);
    y   = M(1:G,3);
    miny = min(y);
    My  = reshape(y,samx,samy);
%     su  = M(1:G,4);
%     Msu = reshape(su,samx,samy);
%     sl  = M(1:G,5);
%     Msl = reshape(sl,samx,samy); 
%     cita1 = su - y;%also  y - sl, standard deviation is too small to show, so I multiply it by 100 times
%     cita2 = y-sl; %cita 1 and cita 2 should be the same
%     cita1 = 100*cita1; cita2 = 100*cita2;
%     su = y+100*cita1; sl = y-100*cita2;
    c = M(1:G,6);
    Mc = reshape(c,samx,samy);
    xl1 = M((G+1):le,1);
    xl2 = M((G+1):le,2);
    yl  = M((G+1):le,3);
    [minY,L] = min(yl(ini+1:length(yl)));
    L = L+ini;% we start searching from ini+ to length(yl), the sequence returned needs to add ini
%    leS = length(xl);
    im = figure;
%    set(im,'Visible','off');%do not show the image , just save it.
   %subplot(2,1,1);
    surf(X,Y,My);
%    colorbar;
%    hold on
%     vsu = surf(X,Y,Msu,'FaceAlpha',0.3);
%     vsu.EdgeColor = 'none';
%     vsl = surf(X,Y,Msl,'FaceAlpha',0.3);
%     vsl.EdgeColor = 'none';   
%    hold off
    hold on
    plot3(xl1(1:ini),xl2(1:ini),yl(1:ini),'og','LineWidth',2,'MarkerSize',20, 'MarkerFaceColor','g')
    plot3(xl1((ini+1):length(yl)),xl2((ini+1):length(yl)),yl((ini+1):length(yl)),'+r','LineWidth',2,'MarkerSize',20); %plot with initial sample
    hold off
    %zlim([0 7]);%limit the range of Z axis
%    title('QPSD and RPSD Tuning by Bayesopt','fontname','Times New Roman','fontsize',20);
    
    handlex = xlabel('noise $\ddot{\xi}$','fontname','Times New Roman', 'fontsize', 80,'fontweight','bold');
    set(handlex,'Interpreter','latex');
    handley = ylabel('noise $\ddot{\theta}$','fontname','Times New Roman', 'fontsize', 80,'fontweight','bold');
    set(handley,'Interpreter','latex');
    handlez = zlabel('JNIS','fontname','Times New Roman', 'fontsize', 80,'fontweight','bold');
    set(handlez,'Interpreter','latex');
    %    legend('prediction','upun','loun','samplePoint');
    legend({'surrogate fn','Ini Sample Pts','It Sample Pts'},'FontSize',60,'FontName','Times');
    set(gca,'fontsize',60,'fontname','Times');
    %subplot(2,1,2);
    %surf(X,Y,Mc);
    %legend({'acq fun'},'FontSize',18,'FontName','Times');
    %set(gca,'fontsize',16,'fontname','Times');
%    saveas(gcf,ouname);
%    frame = imread(ouname);
%    writeVideo(myObj,frame);
end
%close(myObj);

