function [Xh, Yh, dx, dy, valid] = estimator_logratio(sig, k, half_min)
%ESTIMATOR_LOGRATIO  Log-ratio 위치 추정 (자연로그)
%
%   [Xh, Yh, dx, dy, valid] = estimator_logratio(sig, k, half_min)
%
%   sig       XDiff, YDiff, SUM 필드를 가진 구조체
%   k         스케일 [mm]. 스칼라 또는 [kx ky]. 중심 근처 calibration으로 얻음.
%             생략하면 1 (무차원 δ 그대로 반환)
%   half_min  유효 판정 하한 [V]. 네 반쪽 합 중 하나라도 이 값 미만이면 invalid.
%             호출하는 쪽에서 nSigma × σ_half 로 계산해서 넣음
%             (실험: dark 데이터에서 반쪽 합을 직접 복원해 표준편차 계산). 생략하면 0.
%
%   KPA101은 개별 사분면을 주지 않으므로 XDiff, YDiff, SUM에서 반쪽 합을 복원:
%     좌 = (SUM + XDiff)/2,  우 = (SUM - XDiff)/2
%     상 = (SUM + YDiff)/2,  하 = (SUM - YDiff)/2
%   δx = ln(우/좌),  δy = ln(상/하)
%   X̂ = kx·δx,  Ŷ = ky·δy
%
%   valid    네 반쪽 합이 모두 half_min 이상인 샘플. invalid 샘플의 출력은 NaN
%   (log 인자는 별도로 1e-9 V 이하로 내려가지 않게 수치 보호. 판정과는 무관)

    if nargin < 2 || isempty(k), k = 1; end
    if nargin < 3 || isempty(half_min), half_min = 0; end
    kx = k(1); ky = k(end);
    epsV = 1e-9;   % log 수치 보호값 (물리 파라미터 아님)

    L  = (sig.SUM + sig.XDiff)/2;
    Rt = (sig.SUM - sig.XDiff)/2;
    T  = (sig.SUM + sig.YDiff)/2;
    B  = (sig.SUM - sig.YDiff)/2;

    valid = (L >= half_min) & (Rt >= half_min) & (T >= half_min) & (B >= half_min) & ...
            (L > epsV) & (Rt > epsV) & (T > epsV) & (B > epsV);

    dx = log(max(Rt, epsV) ./ max(L, epsV));
    dy = log(max(T,  epsV) ./ max(B, epsV));
    dx(~valid) = NaN;
    dy(~valid) = NaN;
    Xh = kx*dx;
    Yh = ky*dy;
end
