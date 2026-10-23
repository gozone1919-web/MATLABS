function out = estimator_ranges(r0_list, det, alpha, opts)
%ESTIMATOR_RANGES  세 추정기 공통 calibration + 범위 판정 (S2, S3 공용)
%   절차 (세 방법 동일): δ 계산 -> 중심 감도 dδ/dX -> k = 1/감도 -> X_hat = k·δ
%   출력 열 순서: [DoS LogRatio Hybrid]
%     gain_raw    중심 감도 dδ/dX [1/mm]
%     gain        k [mm]
%     lin_range   상대오차 <= tol_rel 유지 최대 |X| [mm]
%     slope_range 국부 감도 >= slope_frac 유지 최대 |X| [mm]
%     slope_at_end  true면 스캔 끝까지 유지 (실제 범위는 그 이상)
%     rmse_r0     |X| <= r0 RMSE [mm]
%     Xs, Xh      스캔 위치, X_hat (그림용)
%   잡음 없음, Y = 0

    if nargin < 4 || isempty(opts), opts = struct(); end
    o.Xs         = getf(opts, 'Xs', linspace(-3.5, 3.5, 701));
    o.cal_frac   = getf(opts, 'cal_frac', 0.1);     % calibration 구간 |X| <= 0.1·r0
    o.tol_rel    = getf(opts, 'tol_rel', 0.01);     % 선형 범위 기준 5%
    o.slope_frac = getf(opts, 'slope_frac', 0.1);   % 감도 유지 기준 10%

    P  = 1/(det.resp*det.G);     % 잡음 없는 비율 계산이라 광량 무관 (SUM = 1 V)
    Xs = o.Xs;

    for ir = 1:numel(r0_list)
        beam.r0 = r0_list(ir); beam.P = P;

        % 중심 감도 -> k
        xc = linspace(-o.cal_frac*beam.r0, o.cal_frac*beam.r0, 21);
        sc = qpd_model(xc, zeros(size(xc)), beam, det);
        [~, ~, d1] = estimator_dos(sc, 1);
        [~, ~, d2] = estimator_logratio(sc, 1);
        dH         = estimator_hybrid(sc, alpha);
        p1 = polyfit(xc, d1, 1); p2 = polyfit(xc, d2, 1); p3 = polyfit(xc, dH, 1);
        gain_raw = [p1(1) p2(1) p3(1)];
        gain     = 1 ./ gain_raw;

        % 스캔 -> 범위 판정
        sig = qpd_model(Xs, zeros(size(Xs)), beam, det);
        Xh  = estimate_all(sig, alpha, gain);

        lin = zeros(1,3); slp = zeros(1,3); atend = false(1,3); rmse = zeros(1,3);
        for m = 1:3
            lin(m)   = linear_range(Xs, Xh(m,:), o.tol_rel);
            slp(m)   = slope_range(Xs, Xh(m,:), o.slope_frac);
            atend(m) = slp(m) >= max(abs(Xs)) - 2*mean(diff(Xs));
            in1 = abs(Xs) <= beam.r0;
            rmse(m)  = sqrt(mean((Xh(m,in1) - Xs(in1)).^2));
        end

        out(ir).r0 = beam.r0;
        out(ir).gain_raw = gain_raw;   out(ir).gain = gain;
        out(ir).lin_range = lin;       out(ir).slope_range = slp;
        out(ir).slope_at_end = atend;  out(ir).rmse_r0 = rmse;
        out(ir).Xs = Xs;               out(ir).Xh = Xh;
        out(ir).opts = o;
    end
end

function r = linear_range(x, xh, tol_rel)
    ok = abs(xh - x) <= tol_rel*abs(x) | abs(x) < 1e-12;
    r = min(edge_extent(x, ok, +1), edge_extent(x, ok, -1));
end

function r = slope_range(x, xh, slope_frac)
    ok = gradient(xh, x) >= slope_frac;
    r = min(edge_extent(x, ok, +1), edge_extent(x, ok, -1));
end

function r = edge_extent(x, ok, dirn)
% 중심에서 dirn 방향으로 ok가 연속인 마지막 점의 |x|
    [~, idx] = min(abs(x));
    while idx + dirn >= 1 && idx + dirn <= numel(x) && ok(idx + dirn)
        idx = idx + dirn;
    end
    r = abs(x(idx));
end

function v = getf(s, name, default)
    if isfield(s, name) && ~isempty(s.(name)), v = s.(name); else, v = default; end
end