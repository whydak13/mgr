function [  ] = matric2Carray( in_matrix )
    [n,m,~]=size(in_matrix);
    line_format='';
    for i=1:m
        line_format=strcat(line_format, '%f, ');
    end
    %line_format=strcat(line_format);
    disp(sprintf('={'))
    for i=1:n
        disp(sprintf((line_format),in_matrix(i,:)))
    end
    disp(strcat('};'))

end

