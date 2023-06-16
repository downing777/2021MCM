function test
    function solvee
        x=sym('x');
        y=sym('y');
        z=sym('z');
        index = 1;
        delta      = .43;
        h_param    = 300.4 + delta;
        f_param    = h_param-300*(1-0.466);
        data2 = [-78.1895   77.9275 -284.8028  -77.6827   77.4223 -282.9567];

        eqns =[(x-data2(index,1))*(data2(index,2)-data2(index,5)) == (data2(index,1)-data2(index,4))*(y-data2(index,2)),...
        (y-data2(index,2))*(data2(index,3)-data2(index,6)) == (data2(index,2)-data2(index,5))*(z-data2(index,3)),...
        x^2+y^2==4*f_param*(z+h_param)];
        S=solve(eqns,[x y z]);

        fprintf('%.4f %.4f %.4f\n',round(S.x(1),4),round(S.y(1),4),round(S.z(1),4))

    end
    connection = xlsread('C:\Users\qeng1\Documents\Code\Modelling\data\A\3v2.csv');
    connection(1,:)
    
end

%#ok<*DEFNU>
%#ok<*NASGU>
%#ok<*AGROW>
%#ok<*PLOTYY>
%#ok<*SETNU>
%#ok<*ASGLU>