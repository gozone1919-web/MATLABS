%% optical_sweep.m  (S3)
% 질문: S46642를 입사각 22.5° 또는 45°로 배치하고 FSM-QPD 거리 L = 10~30 cm일 때,
%       QPD에서 S2가 요구하는 위치 범위를 커버하려면 Mirrorcle 명령 u를 얼마나 써야 하는가?
%
% 계산 흐름
%   u (normalized, -1~1)
%   -> Vdiff = VdiffMax·u                   (Mirrorcle API: u -> Vdifference 선형)
%   -> θ_mech = kθ·Vdiff                     (S46642 중심부 선형 근사)
%   -> 미러 법선 회전 -> 벡터 반사 r' = r - 2(r·n)n
%   -> QPD 평면과 교점 -> QPD 좌표 X, Y [mm]
%
% 출력
%   1. 중심 2x2 Jacobian  dX,Y/du  [mm / command]
%   2. 추정기별 S2 linear range 커버에 필요한 |u| (X축, Y축 따로)
%   3. 추정기별 S2 slope range 커버에 필요한 |u| (X축, Y축 따로)
%   4. 입사각별 광학계 평면도 (점선 = 실제 필요한 최대 수평 명령에서의 빔 편향)
%   5. 필요한 |u| >= u_lin_limit 이면 경고
%
% 가정
%   - 입사면은 수평(테이블 면). 미러 X 명령 -> 수평 이동(QPD X), Y 명령 -> 수직 이동(QPD Y)
%     (실제 마운트에서 축이 바뀌면 결과 표의 X/Y만 바꿔 읽으면 됨)
%   - QPD 평면: u = (0,0)일 때의 반사광에 수직, 미러에서 L 떨어진 곳.
%     QPD +X = 빔을 따라 QPD를 바라볼 때 오른쪽, +Y = 위쪽 (qpd_model 좌표와 동일)
%   - 명령 부호(+u -> +X 인지)는 임의. E3 calibration에서 확인.
%   - r0는 L과 무관 (divergence 미반영). S2와 같은 r0 목록 사용.
%   - 미러 자체의 X↔Y 결합, 마운트 정렬 오차, datasheet 곡선 비선형은 모델링하지 않음 (E3에서 측정).
%   - 2θ, 2θ·cos(i) 근사는 검산용으로만 출력하고 결과 계산에는 쓰지 않음.

clear; clc; close all;

%% 1. 파라미터 -------------------------------------------------------------
% Mirrorcle S46642
mir.VdiffMax = 173;          % [V] mtidevice - S46642.ini 의 VdifferenceMax
% 기계각/전압 기울기 [deg/V]
%   S46642 datasheet 그래프 중심부 판독 기반 임시값 (p.2, 50 V -> 약 1.6°, 100 V -> 약 3.1°)
%   E3 실측 calibration으로 교체 예정
%   X/Y 곡선이 그래프에서 겹쳐 구분되지 않아 현재 같은 값 사용
%   참고(sanity check용, 계산에 사용 안 함): 할선 4.97° / 174.94 V ≈ 0.0284 °/V
mir.k_theta_x = 0.032;       % [deg/V]
mir.k_theta_y = 0.032;       % [deg/V]
% 중심 선형 근사 유효 영역: datasheet 곡선이 |Vdiff| > 약 100 V에서 휘기 시작
u_lin_limit = 100/mir.VdiffMax;   % ≈ 0.58

% 기하
inc_list = [22.5 45];        % [deg] 입사각 (미러 법선과 입사광 사이 각). 22.5° = 제조사 권장, 45° = 비교용
L_list   = [100 150 200 250 300];   % [mm] FSM-QPD 거리

% QPD / 추정기 (S2와 동일)
det.R    = 3.9;   det.gap  = 0;   det.resp = 0.4;   det.G = 1e4;
r0_list  = [0.5 1.0 1.5 2.0];      % [mm]
alpha    = 0.9;
methods  = {'DoS', 'LogRatio', 'Hybrid'};

% 평면도 전용 (계산에 사용 안 함)
draw.L_laser = 100;          % [mm] 레이저 -> FSM 거리. 미확인이라 그림 표시용 임의값

%% 2. S2 기준 범위 가져오기 -------------------------------------------------
rng_out = estimator_ranges(r0_list, det, alpha);   % S2와 같은 기준

%% 3. 기하 계산: Jacobian + 필요 명령 범위 -----------------------------------
na = numel(inc_list); nL = numel(L_list); nr = numel(r0_list); nm = numel(methods);
J      = zeros(2, 2, na, nL);
J_chk  = zeros(2, na, nL);     % [수평 근사, 수직 근사]
u_lin  = NaN(nr, nm, 2, na, nL);   % (r0, method, 축[X Y], 입사각, L)
u_slp  = NaN(nr, nm, 2, na, nL);
du = 1e-4;
ugrid = linspace(0, 1, 20001);

for ia = 1:na
    geo.inc = inc_list(ia);
    for iL = 1:nL
        geo.L = L_list(iL);

        % 1) 중심 Jacobian (중앙 차분)
        f = @(ux, uy) fsm_to_qpd(ux, uy, mir, geo);
        [xp, yp] = f( du, 0); [xm, ym] = f(-du, 0);
        J(:,1,ia,iL) = [xp - xm; yp - ym]/(2*du);
        [xp, yp] = f(0,  du); [xm, ym] = f(0, -du);
        J(:,2,ia,iL) = [xp - xm; yp - ym]/(2*du);

        % 검산용 소각 근사: 수평 2θL, 수직 2θ·cos(i)·L  (θ = kθ·VdiffMax per command)
        J_chk(:,ia,iL) = [2*deg2rad(mir.k_theta_x*mir.VdiffMax)*geo.L; ...
                          2*cosd(geo.inc)*deg2rad(mir.k_theta_y*mir.VdiffMax)*geo.L];

        % 2), 3) 필요 명령: 한 축만 움직였을 때 해당 축 변위가 목표에 처음 도달하는 |u|
        [Xg_p, ~] = f( ugrid, zeros(size(ugrid)));
        [Xg_m, ~] = f(-ugrid, zeros(size(ugrid)));
        [~, Yg_p] = f(zeros(size(ugrid)),  ugrid);
        [~, Yg_m] = f(zeros(size(ugrid)), -ugrid);
        for ir = 1:nr
            for m = 1:nm
                Rl = rng_out(ir).lin_range(m);
                Rs = rng_out(ir).slope_range(m);
                u_lin(ir,m,1,ia,iL) = req_u(ugrid, Xg_p, Xg_m, Rl);
                u_lin(ir,m,2,ia,iL) = req_u(ugrid, Yg_p, Yg_m, Rl);
                u_slp(ir,m,1,ia,iL) = req_u(ugrid, Xg_p, Xg_m, Rs);
                u_slp(ir,m,2,ia,iL) = req_u(ugrid, Yg_p, Yg_m, Rs);
            end
        end
    end
end

%% 4. 결과 출력 --------------------------------------------------------------
fprintf('=== 1. 중심 Jacobian [mm/command] ===\n');
fprintf('%6s %6s | %9s %9s %9s %9s | %18s\n', '입사각', 'L mm', 'dX/dux', 'dX/duy', 'dY/dux', 'dY/duy', '검산 (벡터/근사)');
for ia = 1:na
    for iL = 1:nL
        Jm = J(:,:,ia,iL);
        hv = [abs(Jm(1,1)) abs(Jm(2,2))];
        fprintf('%6.1f %6d | %9.3f %9.3f %9.3f %9.3f | X %.4f  Y %.4f\n', inc_list(ia), L_list(iL), ...
            Jm(1,1), Jm(1,2), Jm(2,1), Jm(2,2), hv(1)/J_chk(1,ia,iL), hv(2)/J_chk(2,ia,iL));
    end
end

warn = {};
print_req('2. linear range 커버에 필요한 |u|  (X축 / Y축)', u_lin, rng_out, 'lin_range', ...
          r0_list, methods, inc_list, L_list, false);
print_req('3. slope range 커버에 필요한 |u|  (X축 / Y축, * = S2 스캔 끝까지 기울기 유지 -> 실제 범위는 이 이상)', ...
          u_slp, rng_out, 'slope_range', r0_list, methods, inc_list, L_list, true);

% 5. 경고
allu = {u_lin, u_slp}; names = {'linear', 'slope'};
for k = 1:2
    U = allu{k};
    idx = find(U >= u_lin_limit | isnan(U));
    for q = idx(:).'
        [ir, m, ax, ia, iL] = ind2sub(size(U), q);
        axn = 'XY';
        if isnan(U(q))
            msg = sprintf('[경고] %s range: 입사각 %.1f°, L %d mm, r0 %.1f, %s, %s축 -> |u| <= 1 에서 도달 불가', ...
                names{k}, inc_list(ia), L_list(iL), r0_list(ir), methods{m}, axn(ax));
        else
            msg = sprintf('[경고] %s range: 입사각 %.1f°, L %d mm, r0 %.1f, %s, %s축 -> |u| = %.3f >= %.2f (datasheet 비선형 영역)', ...
                names{k}, inc_list(ia), L_list(iL), r0_list(ir), methods{m}, axn(ax), U(q), u_lin_limit);
        end
        warn{end+1} = msg; %#ok<SAGROW>
    end
end
fprintf('\n=== 5. 경고 ===\n');
if isempty(warn), fprintf('없음 (모든 경우 |u| < %.2f)\n', u_lin_limit);
else, fprintf('%s\n', warn{:}); end

%% 5. 평면도 -----------------------------------------------------------------
figure('Name', '광학계 평면도', 'Position', [100 100 1200 520]);
for ia = 1:na
    subplot(1, na, ia); hold on; axis equal; grid on;
    g = geo; g.inc = inc_list(ia);
    [n0, f0] = nominal_frame(g.inc);

    % 입사광 (레이저 -> FSM)
    plot([0 0], [-draw.L_laser 0], 'r-', 'LineWidth', 2);
    plot(0, -draw.L_laser, 'ks', 'MarkerFaceColor', [0.3 0.3 0.3], 'MarkerSize', 9);
    text(4, -draw.L_laser, '레이저 (거리는 그림용 임의값)', 'FontSize', 8);

    % 미러 (실제 Ø4.6 mm, 보이게 20 mm로 과장)
    t = cross(n0, [0 0 1]); t = t/norm(t);
    plot([-10 10]*t(1), [-10 10]*t(2), 'b-', 'LineWidth', 3);
    text(5, 8, sprintf('FSM S46642 (입사각 %.1f°, 크기 과장)', g.inc), 'FontSize', 8);

    % 반사광 (최대 L까지) + 실제 필요한 최대 수평 명령에서의 편향 (점선)
    Lmax = max(L_list);
    plot([0 Lmax*f0(1)], [0 Lmax*f0(2)], 'r-', 'LineWidth', 1.2);
    u_fan = max(max(max(u_slp(:,:,1,ia,:))));     % 이 입사각에서 X축 필요 |u| 최댓값
    for s = [-1 1]
        rr = reflect_dir(s*u_fan, 0, mir, g);
        plot([0 Lmax*rr(1)], [0 Lmax*rr(2)], 'r:', 'LineWidth', 0.8);
    end

    % QPD 후보 위치 (센서 Ø7.8 mm 실제 크기 + 마커)
    xq = cross(f0, [0 0 1]); xq = xq/norm(xq);
    for iL = 1:nL
        P = L_list(iL)*f0;
        plot(P(1) + [-1 1]*det.R*xq(1), P(2) + [-1 1]*det.R*xq(2), 'k-', 'LineWidth', 2);
        plot(P(1), P(2), 'ko', 'MarkerSize', 4, 'MarkerFaceColor', 'k');
        off = (-1)^iL * 14 * xq(1:2);     % 라벨을 빔 양옆으로 번갈아 배치
        text(P(1) + off(1), P(2) + off(2), sprintf('L=%d (입사광과 %.0f mm)', L_list(iL), abs(P(1))), ...
             'FontSize', 7);
    end
    xlabel('x [mm] (테이블 면)'); ylabel('y [mm]');
    title(sprintf('입사각 %.1f° (점선: X축 필요 최대 |u_x| = %.3f)', ...
              g.inc, u_fan));
end

%% ======================= local functions ==================================
function [n0, f0] = nominal_frame(inc)
% 테이블 좌표: z 위, 입사광 d = +y. 미러는 원점.
    d  = [0 1 0];
    n0 = [sind(inc), -cosd(inc), 0];          % 입사각 inc가 되도록 수평면에서 회전
    f0 = d - 2*dot(d, n0)*n0;                 % u = 0 반사 방향 = QPD 평면 법선
end

function r = reflect_dir(ux, uy, mir, geo)
% 명령 -> 기계각 -> 법선 회전 -> 반사 방향. ux, uy 같은 크기 배열 -> r: N x 3
    [n0, ~] = nominal_frame(geo.inc);
    d  = [0 1 0];
    ev = [0 0 1];                             % 미러면 안의 수직 축 (이 축 회전 -> 수평 이동)
    eh = cross(n0, ev); eh = eh/norm(eh);     % 미러면 안의 수평 축 (이 축 회전 -> 수직 이동)
    ux = ux(:); uy = uy(:);
    th_v = mir.k_theta_x*mir.VdiffMax*ux;     % X 명령 -> ev 축 회전 [deg]
    th_h = mir.k_theta_y*mir.VdiffMax*uy;     % Y 명령 -> eh 축 회전 [deg]
    N = numel(ux);
    n = rodrigues(repmat(n0, N, 1), eh, th_h);    % eh 축 회전 먼저
    n = rodrigues(n, ev, th_v);                   % 그다음 ev 축 회전
    r = repmat(d, N, 1) - 2*(n*d.').*n;
end

function [X, Y] = fsm_to_qpd(ux, uy, mir, geo)
% 명령 배열 -> QPD 좌표 [mm]. QPD 평면은 f0에 수직, 중심 L·f0.
    [~, f0] = nominal_frame(geo.inc);
    xq = cross(f0, [0 0 1]); xq = xq/norm(xq);    % 빔 방향으로 볼 때 오른쪽
    yq = cross(xq, f0);                           % 위쪽 (= +z)
    sz = size(ux);
    r = reflect_dir(ux, uy, mir, geo);
    t = geo.L ./ (r*f0.');                        % ray와 QPD 평면 교점
    p = t.*r - geo.L*f0;
    X = reshape(p*xq.', sz);
    Y = reshape(p*yq.', sz);
end

function v = rodrigues(v, k, ang_deg)
% 행벡터 v (N x 3) 를 단위축 k (1 x 3) 기준으로 ang_deg (N x 1) 만큼 회전
    a = deg2rad(ang_deg(:));
    K = repmat(k, size(v,1), 1);
    v = v.*cos(a) + cross(K, v, 2).*sin(a) + K.*(v*k.').*(1 - cos(a));
end

function u = req_u(ugrid, Zp, Zm, R)
% +u, -u 방향 각각 |Z| >= R 에 처음 도달하는 u 중 큰 값. 도달 못 하면 NaN.
    ip = find(abs(Zp) >= R, 1); im = find(abs(Zm) >= R, 1);
    if isempty(ip) || isempty(im), u = NaN; return; end
    u = max(ugrid(ip), ugrid(im));
end

function print_req(ttl, U, rng_out, field, r0_list, methods, inc_list, L_list, mark_end)
    fprintf('\n=== %s ===\n', ttl);
    for ia = 1:numel(inc_list)
        fprintf('\n-- 입사각 %.1f° --\n', inc_list(ia));
        for ir = 1:numel(r0_list)
            Rv = rng_out(ir).(field);
            fprintf('r0 = %.1f mm   (S2 범위 [mm]: ', r0_list(ir));
            for m = 1:numel(methods)
                star = '';
                if mark_end && rng_out(ir).slope_at_end(m), star = '*'; end
                fprintf('%s %.2f%s  ', methods{m}, Rv(m), star);
            end
            fprintf(')\n%8s', 'L mm'); fprintf('%18s', methods{:}); fprintf('\n');
            for iL = 1:numel(L_list)
                fprintf('%8d', L_list(iL));
                for m = 1:numel(methods)
                    fprintf('%18s', sprintf('%.4f / %.4f', U(ir,m,1,ia,iL), U(ir,m,2,ia,iL)));
                end
                fprintf('\n');
            end
        end
    end
end