function upload1Dplot(input_f,output_f,it,ini)
% upload1Dplot('/home/zhaozhong/matlab_code/data/1Ddata_ekfslam_NIS','/home/zhaozhong/matlab_code/plot/1Dplot_ekfslam_NIS', 10, 10);
% upload1Dplot('/home/zhaozhong/matlab_code/data/1Ddata_ekfslam_NIS','/home/zhaozhong/matlab_code/plot/1Dplot1_skycrane_NIS', 10, 10);
%it number of iterations-1
%ini number of initial sample
%input_add input folder such as T1 folder
%default in each csv file the first 1000 line is from gaussian process
inname_first     = input_f;
inname_middle = '/iteration';
inname_last     = '.csv';
ouname_first    = output_f;
ouname_middle = '/iteration';
ouname_last     = '.jpg';

% vedio_out = [ouname_first,'/vedio.avi'];
% myObj = VideoWriter(vedio_out);
% myObj.FrameRate = 5;
% open(myObj);

G = 1000;
i=48;
%for i = 0:it
    inname = [inname_first,inname_middle,num2str(i),inname_last];
    ouname= [ouname_first,ouname_middle,num2str(i),ouname_last];
    M = csvread(inname);
    le = length(M);
    x = M(1:G,1);
    y = M(1:G,2);
    su = M(1:G,3);
    sl = M(1:G,4);
%     cita1 = su - y;%also  y - sl, standard deviation is too small to show, so I multiply it by 100 times
%     cita2 = y-sl; %cita 1 and cita 2 should be the same
%     cita1 = 100*cita1; cita2 = 100*cita2;
%     su = y+100*cita1; sl = y-100*cita2;
    c = M(1:G,5);
    xl = M((G+1):le,1);
    yl = M((G+1):le,2);
    [minY, L] = min(yl((ini+1):length(yl)));
    leS = length(xl);
    im = figure;
    %x = x*10; xl = xl*10;%unnormalized
%    set(im,'Visible','off');%do not show the image , just save it.
    set(gca,'Fontname','Times','FontSize',14);
    %subplot(2,1,1);
    plot(x,y,'b','linewidth',8);
    hold on
    plot(x,su,'--k','linewidth',8);
    plot(x,sl,'--k','linewidth',8);
    %plot(xl,yl,'pr');%p means star, r means red. Pretty powerful. Plot with initial sample
    plot(xl((ini+1):leS),yl((ini+1):leS),'o','MarkerSize',20, 'MarkerFaceColor',[1 0 0]); %plot without initial sample
    plot(xl(1:ini),yl(1:ini),'o','MarkerSize',20, 'MarkerFaceColor',[0 1 0]);
    %plot(xl(L),minY,'+r','LineWidth',2,'MarkerSize',15); %plot with initial sample
    %plot(xl(length(yl)),yl(length(yl)),'+r','LineWidth',2,'MarkerSize',15);
    hold off
    %title('QPSD Tuning by Bayesopt','fontname','Times New Roman','fontsize',20);
    h = legend({'Surrogate fn','Upper uncert','Lower uncert','It Sample pts','Ini Sample pts'},'FontSize',60,'FontName','Times');
    rect = [0.65, 0.18, .25, .25];%set legend position in a plot
    set(h, 'Position', rect);
    xlabel('x','fontname','Times', 'fontsize', 80,'fontweight','bold');
    ylabel('JNIS','fontname','Times','fontsize',80,'fontweight','bold');
    set(gca,'fontsize',60,'fontname','Times');
    %set(gca,'fontsize',10);
    %subplot(2,1,2);
    %plot(x,c);
    %legend('acq fun');
    %set(gca,'fontsize',16,'fontname','Times');
%    saveas(gca,ouname);
%    frame = imread(ouname);
%    writeVideo(myObj,frame);
%end
%close(myObj);