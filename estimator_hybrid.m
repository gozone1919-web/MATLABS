function [dHx, dHy, valid] = estimator_hybrid(sig, alpha, half_min, SUM_min)
%   δH = α·δ1 + (1-α)·δ2     δ1: 차분합, δ2: ln(우/좌)
%   (k는 estimator_ranges에서 중심 감도로 구함)

    if nargin < 3, half_min = []; end
    if nargin < 4, SUM_min  = []; end
    [~, ~, d1x, d1y, v1] = estimator_dos(sig, 1, SUM_min);
    [~, ~, d2x, d2y, v2] = estimator_logratio(sig, 1, half_min);
    dHx = alpha*d1x + (1 - alpha)*d2x;
    dHy = alpha*d1y + (1 - alpha)*d2y;
    valid = v1 & v2;
end