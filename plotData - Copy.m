function plotData
fprintf('\n');
%===================================
%   data1 column 4-6 are intersect oordinates
%   data1 column 7 is distance
%   data1 column 8 is bool, in = 1, out = 0
%   data1 column 9 is bool, positive = 1, negative = 0;
%===================================

    azimuth    = 36.795;
    elevation  = 78.169;
    delta      = .43;
    h_param    = 300.4 + delta;
    f_param    = h_param-300*(1-0.466);
    paraboloid_Z = @(x,y) (x.^2+y.^2)./4./f_param-h_param;
    numPtIn    = 0;
    peakCords  = [];
    
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
    %fprintf('data 2 processed, size = %d x %d\n',size(data2,1),size(data2,2));
    %fprintf('data 1 processed, size = %d x %d\n',size(data1,1),size(data1,2));
    
    [~,name,~] = xlsread('C:\Users\qeng1\Documents\Code\Modelling\data\A\1.csv');
    connection = xlsread('C:\Users\qeng1\Documents\Code\Modelling\data\A\4v2.csv');
    %[~,connection,~] = xlsread('C:\Users\qeng1\Documents\Code\Modelling\data\A\3.csv');
    %translatedConnection = zeros(length(connection)-1,3);
    
    
%% Main
    Q2  
    function Q3
        
       
        

    end
    
    
    function Q2
        
        data1_ = data1;
        data2_ = data2;
        
        checkPoint(false);
        transformData;
        
        %MSDtraverse(.01);
        %return
        
        plotFlag = false;
        processPointString;
        loss = MSDloss;
        
        revTransformData;
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
        
        %return
        plotFlag = true;
        plotPointString(data1,data2)
        plot300perimeter;
        
        [x_,y_,z_] = revTransform(0,0,-h_param);
        peakCords = [x_,y_,z_];
        
        fprintf('Max Correction = %.2f.\n',maxDist);
        fprintf('Max Correction Index = %d.\n',index);
        fprintf('Mean Square Deviation = %.2f\n',loss);
        fprintf('Paraboloid Peak = [ %.3f, %.3f, %.3f ]\n',peakCords(1),peakCords(2),peakCords(3));
        plot3(peakCords(1),peakCords(2),peakCords(3),'-o','Color','b','MarkerSize',15,'MarkerFaceColor','#D9FFFF')
        fprintf('Verify: [820]%.2f->[821]%.2f->[822]%.2f.\n',data1(820,7),data1(821,7),data1(822,7));
        %plotParaboloid;
        
        
        xlabel('X-axis (m)')
        ylabel('Y-axis (m)')
        zlabel('Z-axis (m)')
        
        write2excel

    end
    
%% Functions

    function translation
        for i = 1:size(data1,1)
            nodeName = name{i+1,1};
            for j = 2:length(connection)
                for k =1:3
                    if isequal(connection{j,k},nodeName)
                        translatedConnection(j-1,k) = i;
                    end
                end
            end
            i
        end
        xlswrite('4.xlsx',translatedConnection);
    end
   

    function calculateCOM
        for i = 1:size(connection,1)
               [x,y,z] = findCords(i);
        end
    end

    function generateRay
        
    end

    function write2excel
            results = cell(length(inPtArray)+1,8);
            for j = 2:length(inPtArray)+1
                results{j,1} = name{inPtArray(j-1)+1,1};
                if data1(inPtArray(j-1),9) == 1
                    results{j,2} = ['+' num2str(round(data1(inPtArray(j-1),7),4))];
                else
                    results{j,2} = ['-' num2str(round(data1(inPtArray(j-1),7),4))];
                end
                results{j,3} = round(data1(inPtArray(j-1),4),4);
                results{j,4} = round(data1(inPtArray(j-1),5),4);
                results{j,5} = round(data1(inPtArray(j-1),6),4);
            end
            results{1,1} = '对应主索节点编号';
            results{1,2} = '伸缩量（米）';
            results{1,3} = '调整后的X坐标（米）';
            results{1,4} = '调整后的Y坐标（米）';
            results{1,5} = '调整后的Z坐标（米）';
            results{1,6} = '顶点坐标X（米）';
            results{2,6} = round(peakCords(1),4);
            results{1,7} = '顶点坐标Y（米）';
            results{2,7} = round(peakCords(2),4);
            results{1,8} = '顶点坐标Z（米）';
            results{2,8} = round(peakCords(3),4);

            xlswrite('results.xlsx',results);
        end

    function checkPoint(tfFlag)
       for i=1:interval:size(data1,1)
            m = [data1(i,1),data1(i,2),data1(i,3)];
            if ptInOrOut(m,tfFlag)
                numPtIn = numPtIn + 1;
                data1(i,8) = 1;
                inPtArray = [inPtArray,i];
            else
                data1(i,8) = 0;
            end
       end
    end

    function transformData
        for k=1:size(data1,1)
            [xp,yp,zp] = transform(data1(k,1),data1(k,2),data1(k,3));
            data1(k,1:3) = [xp,yp,zp];
        end
        for k=1:size(data2,1)
            [xp,yp,zp] = transform(data2(k,1),data2(k,2),data2(k,3));
            data2(k,1:3) = [xp,yp,zp];
        end
       for k=1:size(data2,1)
            [xp,yp,zp] = transform(data2(k,4),data2(k,5),data2(k,6));
            data2(k,4:6) = [xp,yp,zp];
       end
    end

    function revTransformData
        for k=1:size(data1,1)
            [xp,yp,zp] = revTransform(data1(k,1),data1(k,2),data1(k,3));
            data1(k,1:3) = [xp,yp,zp];
        end
        for k=1:size(data1,1)
            [xp,yp,zp] = revTransform(data1(k,4),data1(k,5),data1(k,6));
            data1(k,4:6) = [xp,yp,zp];
        end
        for k=1:size(data2,1)
            [xp,yp,zp] = revTransform(data2(k,1),data2(k,2),data2(k,3));
            data2(k,1:3) = [xp,yp,zp];
        end
           for k=1:size(data2,1)
                [xp,yp,zp] = revTransform(data2(k,4),data2(k,5),data2(k,6));
                data2(k,4:6) = [xp,yp,zp];
           end
    end

    function [x_,y_,z_] = transform(x,y,z)
       out = Transformer * [x,y,z]';
       x_ = out(1);y_ = out(2); z_ = out(3);
    end

    function [x_,y_,z_] = revTransform(x,y,z)
       out = inv(Transformer) * [x,y,z]';
       x_ = out(1);y_ = out(2); z_ = out(3);
    end

    function verify
        fprintf('Verify: [368]%.2f->[369]%.2f->[370]%.2f.\n',data1(368,7),data1(369,7),data1(370,7));
        fprintf('Verify: [611]%.2f->[612]%.2f->[613]%.2f.\n',data1(611,7),data1(612,7),data1(613,7));
    end

    function MSDtraverse(interval_)
        plotFlag = false;
        lossArray = [];
        distArray = [];
        
        for deltaV = -.6:interval_:.6
            fprintf('Current delta = %.2f\n',deltaV);
            delta = deltaV;
            h_param = 300.4 + delta;
            f_param = h_param-300*(1-0.466);
            
            processPointString;
            
            maxDist = 0;
            index   = [];
            for k=1:interval:size(data1,1)
                if data1(k,8) == 1
                    if abs(data1(k,7)) > maxDist && k ~= 369 && k ~= 612 && k ~= 752 && k ~= 821
                        maxDist = data1(k,7);
                        index = k;
                    end
                end
            end
            fprintf('Max Correction = %.2f.\n',maxDist);
            %fprintf('Max Correction Index = %d.\n',index);
            loss = MSDloss;
            lossArray = [lossArray,loss];
            distArray = [distArray,maxDist];
        end
        [AX,~,~] = plotyy(-.6:interval_:.6,lossArray,-.6:interval_:.6,distArray);
        title('Correction offset on Z-axis') 
        set(get(AX(1),'Ylabel'),'String','Mean Squre Deviation') 
        set(get(AX(2),'Ylabel'),'String','Max Correction (m)') 
        grid on
    end

    function loss = MSDloss
        loss = 0;
        delta_c = [];
        for i=1:interval:size(data1,1)
        % 这里有问题的两条数据用近邻代替
            if i == 369 || i == 612 || i == 752 || i == 821
                dist = data1(i+1,7);
                else
                dist = data1(i,7);
            end
            delta_c = [delta_c,dist];
        end
        loss = sqrt((delta_c*delta_c')/numPtIn);
        %fprintf('Mean Square Deviation = %.2f.\n',loss);
    end

    function calcIntersectDist(index)
        % solve for the foot of the perpendicular point
        function e = equations(x)
            e(1) = (x(1)-data2(index,1))*(data2(index,2)-data2(index,5)) - (data2(index,1)-data2(index,4))*(x(2)-data2(index,2));
            e(2) = (x(2)-data2(index,2))*(data2(index,3)-data2(index,6)) - (data2(index,2)-data2(index,5))*(x(3)-data2(index,3));
            e(3) = x(1)^2+x(2)^2-4*f_param*(x(3)+h_param);
        end
        options = optimoptions('fsolve','Display','none');
        intersect = fsolve(@equations,data1(index,1:3),options);
        data1(index,4:6) = intersect;
        data1(index,7) = norm(data1(index,1:3)-intersect);
        if norm(data1(index,1:3)) > norm(data1(index,4:6))
            data1(index,9) = 1; % positive correction
        else
            data1(index,9) = 0; % negative correction
        end
        if plotFlag
            plot3(intersect(1),intersect(2),intersect(3),'*k');
            plot3([data1(index,1),data1(index,4)],[data1(index,2),data1(index,5)],[data1(index,3),data1(index,6)],'b');
        end
    end 
 
    function processPointString
        t = tic;
        plot3(0,0,-200);
        hold on
        shading interp
        axis equal
        grid on
        for i=1:interval:size(data1,1)%*~testMode+100*testMode
            m = [data1(i,1),data1(i,2),data1(i,3)];
            if data1(i,8) == 1
                if plotFlag
                    % point
                    plot3(data1(i,1),data1(i,2),data1(i,3),'o--r');
                    % string
                    plot3([data2(i,1),data2(i,4)+(data2(i,4)-data2(i,1))*0],[data2(i,2),data2(i,5)+(data2(i,5)-data2(i,2))*0],[data2(i,3),data2(i,6)+(data2(i,6)-data2(i,3))*0],'--b');
                end
                % intersection point
                calcIntersectDist(i);
            else
                if elseFlag
                    if plotFlag
                        % point
                        plot3(data1(i,1),data1(i,2),data1(i,3),'o--b');
                        % string
                        plot3([data2(i,1),data2(i,4)+(data2(i,4)-data2(i,1))*0],[data2(i,2),data2(i,5)+(data2(i,5)-data2(i,2))*0],[data2(i,3),data2(i,6)+(data2(i,6)-data2(i,3))*0],'--b');
                    end
                    % intersection point
                    calcIntersectDist(i);
                end
            end
            %drawnow
            %pause(.01)
        end
        toc(t)
    end

    function plotPointString(data1_,data2_)
        t = tic;
        optionFlag = true;
        for i=1:interval:size(data1_,1)
            m = [data1_(i,1),data1_(i,2),data1_(i,3)];
            if data1_(i,8) == 1
                if plotFlag
                    % point
                    plot3(data1_(i,1),data1_(i,2),data1_(i,3),'o--r');
                    % string
                    if i == 0
                        plot3([data2_(i,1),data2_(i,4)+(data2_(i,4)-data2_(i,1))*100],[data2_(i,2),data2_(i,5)+(data2_(i,5)-data2_(i,2))*100],[data2_(i,3),data2_(i,6)+(data2_(i,6)-data2_(i,3))*100],'--b');
                    else
                        plot3([data2_(i,1),data2_(i,4)+(data2_(i,4)-data2_(i,1))*0],[data2_(i,2),data2_(i,5)+(data2_(i,5)-data2_(i,2))*0],[data2_(i,3),data2_(i,6)+(data2_(i,6)-data2_(i,3))*0],'--b');
                    end
                    % intersection point
                    plot3(data1_(i,4),data1_(i,5),data1_(i,6),'*k');
                    plot3([data1_(i,1),data1_(i,4)],[data1_(i,2),data1_(i,5)],[data1_(i,3),data1_(i,6)],'m');
                end
            else
                % point
                if elseFlag
                    if plotFlag
                        plot3(data1_(i,1),data1_(i,2),data1_(i,3),'o--b');
                        % string
                        plot3([data2_(i,1),data2_(i,4)+(data2_(i,4)-data2_(i,1))*0],[data2_(i,2),data2_(i,5)+(data2_(i,5)-data2_(i,2))*0],[data2_(i,3),data2_(i,6)+(data2_(i,6)-data2_(i,3))*0],'--b');
                        % intersection point
                        plot3(data1_(i,4),data1_(i,5),data1_(i,6),'*k');
                        plot3([data1_(i,1),data1_(i,4)],[data1_(i,2),data1_(i,5)],[data1_(i,3),data1_(i,6)],'m');
                    end
                end
            end
            if optionFlag
                if plotFlag
                    hold on
                    shading interp
                    axis equal
                    grid on
                end
                optionFlag = false;
            end
        end
        toc(t)
    end
    
    function plotParaboloid
        if plotFlag
            arrayX  = -300:1:300;
            arrayY  = -300:1:300;
            [x,y]   = meshgrid(arrayX,arrayY);
            z = paraboloid_Z(x,y);
            meshHdl = mesh(x,y,z);
            alpha(meshHdl,.2);
        end
    end
    
    function output=rid369(input)
        output=input;
        output(369,:)=output(370,:);
    end
    
    function plot300perimeter
        % https://blog.csdn.net/weixin_44986426/article/details/114868368
        if plotFlag
            r = 150;
            valueX = @(z) z*cos(azimuth)/tan(elevation);
            valueY = @(z) z*sin(azimuth)/tan(elevation);
            valueZ = -195;
            X1 = [valueX(valueZ) valueY(valueZ) valueZ];
            valueZ = -300;
            X2 = [valueX(valueZ) valueY(valueZ) valueZ];
            length_cyl=norm(X2-X1);
            [x,y,z]=cylinder(r,100);
            z=z*length_cyl;
            %绘制两个底面
            hold on;
            cylinderHdl=mesh(x,y,z);
            %计算圆柱体旋转的角度
            unit_V=[0 0 1];
            angle_X1X2=acos(dot( unit_V,(X2-X1) )/( norm(unit_V)*norm(X2-X1)) )*180/pi;
            %计算旋转轴
            axis_rot=cross(unit_V,(X2-X1));
            %将圆柱体旋转到期望方向
            if angle_X1X2~=0 % Rotation is not needed if required direction is along X
                rotate(cylinderHdl,axis_rot,angle_X1X2,[0 0 0])
            end
            %将圆柱体和平面挪到期望的位置
            set(cylinderHdl,'XData',get(cylinderHdl,'XData')+X1(1))
            set(cylinderHdl,'YData',get(cylinderHdl,'YData')+X1(2))
            set(cylinderHdl,'ZData',get(cylinderHdl,'ZData')+X1(3))
            % 设置圆柱体的颜色
            set(cylinderHdl,'FaceColor','k')
            set(cylinderHdl,'EdgeAlpha',0)
            alpha(cylinderHdl,0.3);
        end
    end

    function res = ptInOrOut(pt,tfFlag)
        if tfFlag == 0
            v = [0 0 1];
        else
            v = [cos(azimuth) sin(azimuth) tan(elevation)];
        end
        if norm(cross(pt,v))/norm(v)>150
            res = false;
        else
            res = true;
        end
    end
    
end





%#ok<*DEFNU>
%#ok<*NASGU>
%#ok<*AGROW>
%#ok<*PLOTYY>
%#ok<*SETNU>
%#ok<*ASGLU>