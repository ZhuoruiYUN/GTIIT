function print_matrix(matrix_name, matrix)
    [nRows, nCols] = size(matrix);
    fprintf('%s = [', matrix_name);
    for i = 1:nRows
        for j = 1:nCols
            elem_str = char(matrix(i, j));
            fprintf('%s', elem_str);
            if j < nCols
                fprintf(' ');
            end
        end
        if i < nRows
            fprintf('; ');
        end
    end
    fprintf('];\n');
end