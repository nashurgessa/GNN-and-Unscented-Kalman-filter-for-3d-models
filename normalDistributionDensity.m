function likelihood = normalDistributionDensity(con, mu, x, Dimension)

    if Dimension==1
        fprintf("Dimension M: %d\n", Dimension);
        d = mu - x
        fprintf("d: %d\n", d);
        e = -0.5 * power(d, 2. ) / con
        likelihood = (1.0 / sqrt(2*pi8conv)) * exp(e);
    else
        % fprintf("Dimension M: %d\n", Dimension);
        d = mu - x;  % predicted - measured
        e = -0.5 * transpose(d) * inv(con) * d;
        denominator = power(2*pi, (Dimension* 0.5)) * sqrt(det(con));
        likelihood = exp(e) * (1 / denominator);
        
%         fprintf("d value: %d\n", d);
%         fprintf("e: %d\n", e);
%         fprintf("denominator: %d\n", denominator);
        % fprintf("likelihood: %d\n", likelihood);
    end
    
end