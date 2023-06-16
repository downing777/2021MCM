function Q3
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
    translatedConnection = [];
    
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
    
    [~,name,~] = xlsread('C:\Users\qeng1\Documents\Code\Modelling\data\A\1.csv');
    surface = xlsread('C:\Users\qeng1\Documents\Code\Modelling\data\A\3v2.csv');
    
    % 蒙特卡洛模拟
    MonteCarloSimulation(10000);
    % 优化反射率
    ReflecRateTraverse(.001);
    % 初始化节点数据
    checkPoint(false);
    % 坐标线性变换
    transformData;        
    plotFlag = false;
    % 数据计算
    processPointString;
    % 坐标逆变换
    revTransformData;
    plotFlag = true;
    % 画出300米范围
    plot300perimeter;
    
    % 按照0.07%的伸缩误差随机调整各节点参数
    randomizeCords;
    % 画出需要调整的节点及相邻节点间的绳索
    plotMesh;
    % 计算反射面
    calcComNorm;
    % 建模入射电磁波并计算反射电磁波方向
    generateRay;
    % 画出馈源舱
    plotFeed;
    % 计算信号接收比例
    calcRecvRate;

end