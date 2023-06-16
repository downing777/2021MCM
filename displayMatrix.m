function displayMatrix(matrix,expression,lines,columns)
% displayMatrix(matrix,[2 3],[4 6]);
    if nargin == 4
        line = checkArray(lines);
        column = checkArray(columns);
    elseif nargin == 3
        line = checkArray(lines);
        column = checkArray(size(matrix,2));
    elseif nargin == 2
        line = checkArray(size(matrix,1));
        column = checkArray(size(matrix,2));
    else
        line = checkArray(size(matrix,1));
        column = checkArray(size(matrix,2));
        expression = '.0f';
    end

    for i=line
        s='';
        mat =[];
        for j=column
            s = [s,'%',expression,'  '];
            mat = [mat,matrix(i,j)];
        end
        fprintf('%s%g%s','Index',i,': ');
        fprintf('%s','|');
        fprintf(s,mat);
        fprintf('%s\n','|');
    end
    fprintf('\n');
%%
    function old
        if nargin == 3
            line = checkArray(lines);
            column = checkArray(columns);
        elseif nargin == 2
            line = checkArray(lines);
            column = checkArray(size(matrix,2));
        else
            line = checkArray(size(matrix,1));
            column = checkArray(size(matrix,2));
        end

    %     c = '';
    %     s = '';
    %     for j=column
    %         c=[c,'C  ',j,':'];
    %         s=[s,'%s%g%s'];
    %     end
    %     s=[s,'\n\n'];
    %     fprintf(s,c);

        for i=line
            s='';
            mat =[];
            for j=column
                s = [s,'%4.4f  '];
                mat = [mat,matrix(i,j)];
            end
            fprintf('%s%g%s','Line',i,': ');
            fprintf('%s','|');
            fprintf(s,mat);
            fprintf('%s\n','|');
        end
    end
    
    function array = checkArray(input)
        array = [];
        if size(input,2) == 1
            for ii=1:input
                array = [array,ii];
            end
        else
            for ii=input(1):input(2)
                array = [array,ii];
            end
        end
    end

end