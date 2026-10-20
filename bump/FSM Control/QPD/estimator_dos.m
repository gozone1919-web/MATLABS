function [Xh, Yh, dx, dy, valid] = estimator_dos(sig, k, SUM_min)
% ESTIMATOR_DOS  차분합(Difference-over-Sum) 위치 추정
%
%   [Xh, Yh, dx, dy, valid] = estimator_dos(sig, k, SUM_min)
%
%   sig      XDiff, YDiff, SUM 필드를 가진 구조체 (qpd_model 출력 또는 KPA101 입력)
%   k        스케일 [mm]. 스칼라 또는 [kx ky]. 중심 근처 calibration으로 얻음.
%            생략하면 1 (무차원 δ 그대로 반환)
%   SUM_min  유효 판정 하한 [V]. SUM <= SUM_min 이면 invalid.
%            호출하는 쪽에서 nSigma × σ_SUM 으로 계산해서 넣음
%            (실험: dark 데이터의 SUM 표준편차 사용). 생략하면 0.
%
%   δx = -XDiff/SUM   (Thorlabs XDiff = 좌-우 이므로 부호 반전 -> +X = 오른쪽)
%   δy =  YDiff/SUM
%   X̂ = kx·δx,  Ŷ = ky·δy
%   valid    SUM > SUM_min 인 샘플. invalid 샘플의 출력은 NaN
%            (폐루프에서는 NaN을 그대로 쓰지 말고 이전 값 유지 등으로 처리)

    if nargin < 2 || isempty(k), k = 1; end
    if nargin < 3 || isempty(SUM_min), SUM_min = 0; end
    kx = k(1); ky = k(end);

    valid = sig.SUM > SUM_min;
    S = sig.SUM;
    S(~valid) = NaN;

    dx = -sig.XDiff ./ S;
    dy =  sig.YDiff ./ S;
    Xh = kx*dx;
    Yh = ky*dy;
end
