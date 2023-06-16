function Q2

    azimuth    = 36.795;
    elevation  = 78.169;
    delta      = .43;
    h_param    = 300.4 + delta;
    f_param    = h_param-300*(1-0.466);%-0.024;
    paraboloid_Z = @(x,y) (x.^2+y.^2)./4./f_param-h_param;
    numPtIn    = 0;
    peakCords  = [];
    rayLength  = [150 250];
    randomWalkLimit = 0.07/1.6/100;
    
    plotFlag = false;
    elseFlag = false;
    interval   = 1;
    inPtArray = [];
    
    azimuth = azimuth/180*pi;
    elevation  = elevation/180*pi;
    
    theta = elevation - pi/2;   
    Transformer = [ cos(theta),     0,      sin(theta)  ;...
                    0,              1,      0           ;...
                    -sin(theta),    0,      cos(theta)  ] *...
                  [ cos(azimuth),   sin(azimuth),   0   ;...
                    -sin(azimuth),  cos(azimuth),   0   ;...
                    0,              0,              1   ];
    
    
    data1 = csvread('C:\Users\qeng1\Documents\Code\Modelling\data\A\appendix1.csv');
    data2 = csvread('C:\Users\qeng1\Documents\Code\Modelling\data\A\appendix2.csv');

    % 坐标初始化
    checkPoint(false);
    % 坐标变换
    transformData;
    % 按照均方差优化理想抛物面解析式
    MSDtraverse(.01);
    
    plotFlag = false;
    % 计算具体节点坐标等数据
    processPointString;
    % 计算均方差
    loss = MSDloss;
    %坐标逆变换
    revTransformData;
    
    % 寻找最大修正幅度及其点位编号
    maxDist = 0;
    index   = [];
    for ii=1:interval:size(data1,1)
        if data1(ii,8) == 1
            if abs(data1(ii,7)) > maxDist && ii ~= 369 && ii ~= 612 && ii ~= 752 && ii ~= 821
                maxDist = data1(ii,7);
                index = ii;
            end
        end
    end

    plotFlag = true;
    % 画出拟合好的节点及下拉锁
    plotPointString(data1,data2)
    % 画出300米开口范围
    plot300perimeter;
    
    % 计算抛物面顶点坐标
    [x_,y_,z_] = revTransform(0,0,-h_param);
    peakCords = [x_,y_,z_];

    % 输出运行结果
    fprintf('Max Correction = %.2f.\n',maxDist);
    fprintf('Max Correction Index = %d.\n',index);
    fprintf('Mean Square Deviation = %.2f\n',loss);
    fprintf('Paraboloid Peak = [ %.3f, %.3f, %.3f ]\n',peakCords(1),peakCords(2),peakCords(3));
    plot3(peakCords(1),peakCords(2),peakCords(3),'-o','Color','b','MarkerSize',15,'MarkerFaceColor','#D9FFFF')
    fprintf('Verify: [820]%.2f->[821]%.2f->[822]%.2f.\n',data1(820,7),data1(821,7),data1(822,7));
    % 画出理想抛物面
    plotParaboloid;

    xlabel('X-axis (m)')
    ylabel('Y-axis (m)')
    zlabel('Z-axis (m)')
    
    % 输出到excel中
    write2excel

end