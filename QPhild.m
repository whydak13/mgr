function eta=QPhild(H,f,A_cons,b,max_it)
    % E=H;
    % F=f;
    % M=A_cons;
    % gamma=b;
    % eta =x

    %eta = H \ f;
    eta = linsolve(H,-f);

%    kk = 0;
%    for i = 1:n1
%        if (A_cons(i,:) * eta > b(i))
%            kk = kk + 1;
%        end
%    end
    kk = sum(A_cons * eta > b); % replace above code
    
    if (kk == 0)
        return;
    end
    
    %P = A_cons * (H \ A_cons');
    HA = linsolve(H, A_cons');
    P = A_cons * HA;
    d = b - A_cons * eta;
    
    [n,m] = size(d);
    lambda = zeros(n,m);
    
    for km = 1:max_it
        %find the elements in the solution vector one by one
        % km could be larger if the Lagranger multiplier has a slow
        % convergence rate.
        lambda_p = lambda;

        for i = 1:n
            la = lambda(i,1) - (P(i,:) * lambda + d(i,1)) / P(i,i);
            lambda(i,1) = max(0,la);
        end

        al = (lambda - lambda_p)' * (lambda - lambda_p);

        if (al < 10e-8); 

            break; 
        end
    end
%             global g_k
%             g_k=[g_k km];
    eta = eta - HA * lambda;