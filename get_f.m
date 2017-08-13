function [ f ] =get_f( Np, r, F, Xf, Phi )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
     r2 = repmat(r, Np , 1);
    y_sp = reshape(r2', size(r2, 1) * size(r2, 2), 1);
    y2 = F*Xf;
    f= (-(y_sp - y2)'*Phi)';
end

