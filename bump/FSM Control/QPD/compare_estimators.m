%% compare_estimators.m  (S2)
% 세 추정기 비교. 모두 같은 절차를 거침:
%   무차원 δ 계산 -> 중심 기울기로 k [mm] 구함 -> X_hat = k·δ
%   DoS     : δ1 = -XDiff/SUM
%   LogRatio: δ2 = ln(우/좌)
%   Hybrid  : δH = α·δ1 + (1-α)·δ2   (논문 원식은 r0·δH, 여기선 다른 둘과 같이 k·δH)
% 판정 기준(cal_frac, tol_rel, slope_frac, 스캔 범위)은 estimator_ranges.m에서 관리

clear; clc; close all;
rng(1);

%% 1. 파라미터
det.R    = 3.9;    % [mm] 센서 반경
det.gap  = 0;      % [mm] 미확인 -> 0 (이상적 조건)
det.resp = 0.4;    % [A/W] @633 nm
det.G    = 1e4;    % [V/A]

% 광원: Mirrorcle Dev Kit red diode laser, 635 nm, 5 mW (nominal), NE13A-A 사용
% 실제 QPD 도달 파워는 E1에서 PM100D로 측정해 교체
P_laser  = 5e-3;                          % [W] nominal
T_ND     = 10^(-1.3);                     % NE13A-A, OD 1.3 -> 투과율 약 5%
R_mirror = 1;                             % 미러 반사 손실 미반영 (값 미확인)
P_det      = P_laser*T_ND*R_mirror;       % [W] 약 0.25 mW
SUM_target = P_det*det.resp*det.G;        % [V] 약 1.0 V (SNR 정의 기준으로도 사용)

r0_list = [0.5 1.0 1.5 2.0];              % [mm] 1/e^2 빔 반경
alpha   = 0.9;                            % Hybrid 가중치 (논문값)
methods = {'DoS', 'LogRatio', 'Hybrid'};
nm = numel(methods); nr = numel(r0_list);

%% 2~3. calibration + 1D 오차 곡선
rng_out = estimator_ranges(r0_list, det, alpha);
Xs = rng_out(1).Xs;
o  = rng_out(1).opts;

figure('Name', '1D estimation error', 'Position', [100 100 1100 750]);
for ir = 1:nr
    Xh = rng_out(ir).Xh;
    subplot(2, 2, ir);
    plot(Xs, Xh - Xs, 'LineWidth', 1.2);  grid on;
    xlabel('X true [mm]'); ylabel('X_hat - X [mm]');
    title(sprintf('r_0 = %.1f mm', r0_list(ir)));
    ylim([-1 1]*r0_list(ir)); xline(det.R, 'k:'); xline(-det.R, 'k:');
    if ir == 1, legend(methods, 'Location', 'best'); end
end

fprintf('\n=== 원신호 중심 감도 dδ/dX [1/mm] ===\n');
print_table(r0_list, methods, vertcat(rng_out.gain_raw), '%10.3f');
fprintf('\n=== 선형 범위 |X| [mm] (상대오차 <= %.0f%%) ===\n', o.tol_rel*100);
print_table(r0_list, methods, vertcat(rng_out.lin_range), '%10.3f');
fprintf('\n=== 감도 유지 범위 |X| [mm] (중심 감도의 %.0f%% 이상, %.2f = 스캔 끝) ===\n', o.slope_frac*100, max(Xs));
print_table(r0_list, methods, vertcat(rng_out.slope_range), '%10.3f');
fprintf('\n=== RMSE [mm], |X| <= r0 ===\n');
print_table(r0_list, methods, vertcat(rng_out.rmse_r0), '%10.4f');

%% 4. Y 오프셋에 따른 X 추정 영향 (r0 = 1 mm)
beam.r0 = 1.0; beam.P = P_det;
gain = estimator_ranges(beam.r0, det, alpha).gain;
Yoff = [0 0.5 1.0 2.0]*beam.r0;

figure('Name', 'X error vs Y offset', 'Position', [150 150 1100 350]);
for m = 1:nm
    subplot(1, nm, m); hold on;
    for iy = 1:numel(Yoff)
        sig = qpd_model(Xs, Yoff(iy)*ones(size(Xs)), beam, det);
        Xh  = estimate_all(sig, alpha, gain);
        plot(Xs, Xh(m,:) - Xs, 'LineWidth', 1.1);
    end
    grid on; ylim([-1 1]);
    xlabel('X true [mm]'); ylabel('X_hat - X [mm]');
    title(sprintf('%s, r_0 = %.1f mm', methods{m}, beam.r0));
    legend(compose('Y = %.1f mm', Yoff), 'Location', 'best');
end

%% 5. noise sweep
% 잡음: sigma_quad (사분면별 독립 백색잡음). 실측 KPA101 잡음은 sigma_out으로 따로 씀.
% SNR_dB = 20·log10(SUM_target / sigma_quad)   <- 자체 정의, 논문 SNR과 다름
% 유효 하한: SUM_min = nSigma·2·sigma_quad, half_min = nSigma·√2·sigma_quad
%           (실험에선 dark 데이터 표준편차로 교체)
SNR_dB      = [20 30 40 50 60];
nSigma_list = [3 5];              % 후보값
r0_noise    = 1.0;
Xn     = linspace(-r0_noise, r0_noise, 21);
Ntrial = 200;

beam.r0 = r0_noise; beam.P = P_det;
gain = estimator_ranges(beam.r0, det, alpha).gain;
Xrep = repmat(Xn, Ntrial, 1);
Xh_clean = estimate_all(qpd_model(Xrep, zeros(size(Xrep)), beam, det), alpha, gain);

nS = numel(SNR_dB); nN = numel(nSigma_list);
rms_valid = NaN(nS, nm, nN);
inv_ratio = zeros(nS, nm, nN);

for in = 1:nN
    for is = 1:nS
        noise.sigma_quad = SUM_target/10^(SNR_dB(is)/20);
        SUM_min  = nSigma_list(in)*2*noise.sigma_quad;
        half_min = nSigma_list(in)*sqrt(2)*noise.sigma_quad;

        sig = qpd_model(Xrep, zeros(size(Xrep)), beam, det, noise);
        [Xh, V] = estimate_all(sig, alpha, gain, half_min, SUM_min);

        for m = 1:nm
            v = V(m,:);
            inv_ratio(is,m,in) = mean(~v);
            if any(v)
                e = Xh(m,v) - Xh_clean(m,v);
                rms_valid(is,m,in) = sqrt(mean(e.^2));
            end
        end
    end
end

figure('Name', 'Noise sweep', 'Position', [200 200 1000 400]);
for in = 1:nN
    subplot(1, nN, in);
    semilogy(SNR_dB, rms_valid(:,:,in)*1e3, '-o', 'LineWidth', 1.2); grid on;
    xlabel('SNR [dB]'); ylabel('valid RMS error [um]');
    title(sprintf('nSigma = %g, r_0 = %.1f mm', nSigma_list(in), r0_noise));
    legend(methods, 'Location', 'best');
end

for in = 1:nN
    fprintf('\n=== noise sweep, nSigma = %g, r0 = %.1f mm ===\n', nSigma_list(in), r0_noise);
    fprintf('%8s | %-30s | %s\n', 'SNR dB', 'valid RMS [um]', 'invalid [%]');
    fprintf('%8s | ', ''); fprintf('%10s', methods{:}); fprintf(' | '); fprintf('%10s', methods{:}); fprintf('\n');
    for is = 1:nS
        fprintf('%8d | ', SNR_dB(is)); fprintf('%10.2f', rms_valid(is,:,in)*1e3);
        fprintf(' | ');                fprintf('%10.1f', inv_ratio(is,:,in)*100); fprintf('\n');
    end
end

%% local function
function print_table(r0_list, methods, M, fmt)
    fprintf('%8s', 'r0 mm'); fprintf('%10s', methods{:}); fprintf('\n');
    for i = 1:numel(r0_list)
        fprintf('%8.1f', r0_list(i)); fprintf(fmt, M(i,:)); fprintf('\n');
    end
end