function [Xh, V] = estimate_all(sig, alpha, gain, half_min, SUM_min)
%ESTIMATE_ALL  세 추정기 X_hat = k·δ [mm]  (행: DoS, LogRatio, Hybrid)
%   gain = [k_DoS k_Log k_Hyb] [mm]  (estimator_ranges 출력)
%   V    = valid 마스크

    if nargin < 4 || isempty(half_min), half_min = 0; end
    if nargin < 5 || isempty(SUM_min),  SUM_min  = 0; end
    [x1, ~, ~, ~, v1] = estimator_dos(sig, gain(1), SUM_min);
    [x2, ~, ~, ~, v2] = estimator_logratio(sig, gain(2), half_min);
    [d3, ~, v3]       = estimator_hybrid(sig, alpha, half_min, SUM_min);
    x3 = gain(3)*d3;
    Xh = [x1(:).'; x2(:).'; x3(:).'];
    V  = [v1(:).'; v2(:).'; v3(:).'];
end