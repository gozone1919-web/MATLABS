%% fsm_dynamics_model.m  (S4)
% FSM 동특성 모델 (S46642 데이터시트 기반, 축별)
%   G(s) = H_LPF(s) · H_res(s)        DC 이득 = 1 (정규화)
%   H_LPF : 6차 Bessel 저역통과 필터 (데이터시트 권장, ini HardwareFilterBw = 120 Hz)
%           주파수 스케일은 데이터시트 4쪽 step 응답(120 Hz 필터)의 50% 도달 시간에 맞춤
%   H_res : 2차 기계 공진 (데이터시트 fres, Q)
%   각도 변환 이득 없음 -> 뒤의 J [mm/u]에 포함 (이득 중복 방지)
%   통신 지연 없음     -> Simulink Plant의 Delay 블록(delay)에서 처리 (중복 방지)
% 단독 실행 가능. init_pid_sim.m에서 불러도 됨 (Ts가 있으면 그 값을 씀)
% 필요: Control System Toolbox (tf, zpk, step, stepinfo, bode, c2d)

%% 1. 파라미터
fsm.f_lpf  = 120;           % [Hz] HardwareFilterBw (S46642 ini, 데이터시트 권장값)
fsm.t50_ds = 6.0e-3;        % [s]  데이터시트 4쪽 step 응답(120 Hz 필터) 50% 도달 시간, 그래프 판독값
fsm.f_res  = [359 358];     % [Hz] 공진 주파수 [X Y] (데이터시트)
fsm.Q      = [34.8 35.0];   % [-]  공진 Q [X Y] (데이터시트)

if ~exist('Ts', 'var'),         Ts = 1e-3;         end   % [s] 이산화 주기 (임시, E2 후 교체)
if ~exist('show_plots', 'var'), show_plots = true; end

%% 2. 6차 Bessel 정규화 극점 (DC 군지연 = 1 s 기준)
b6     = [1 21 210 1260 4725 10395 10395];
p_norm = roots(b6);
w3     = fzero(@(w) abs(10395/polyval(b6, 1i*w)) - 1/sqrt(2), [0.1 10]);  % 정규화 -3 dB [rad/s]

%% 3. 축별 모델
ax_name = {'X', 'Y'};
t_eval  = linspace(0, 0.04, 40001);     % step 응답 계산용 시간 [s]
G = cell(1,2); Gd = cell(1,2); H_lpf = cell(1,2); H_res = cell(1,2);
a = zeros(1,2);

for i = 1:2
    wn       = 2*pi*fsm.f_res(i);
    H_res{i} = tf(wn^2, [1 wn/fsm.Q(i) wn^2]);

    % 50% 도달 시간이 데이터시트 값과 같아지는 Bessel 주파수 스케일 a 찾기
    t50_err = @(s) t50_of(make_lpf(p_norm, s) * H_res{i}, t_eval) - fsm.t50_ds;
    a(i)    = fzero(t50_err, [50 500]);

    H_lpf{i} = make_lpf(p_norm, a(i));
    G{i}     = minreal(H_lpf{i} * H_res{i});    % 연속 모델
    Gd{i}    = c2d(G{i}, Ts, 'zoh');            % 이산 모델 (ZOH)
end

% Simulink / init_pid_sim.m 용 출력
Gx_fsm = G{1};   Gy_fsm = G{2};
[Gdx_num, Gdx_den] = tfdata(tf(Gd{1}), 'v');    % Discrete Transfer Fcn 블록 계수 (X)
[Gdy_num, Gdy_den] = tfdata(tf(Gd{2}), 'v');    % Discrete Transfer Fcn 블록 계수 (Y)

%% 4. 확인 출력
fprintf('\n=== FSM 동특성 모델 (S46642 데이터시트 기반) ===\n');
fprintf('Mirrorcle 예제 코드 필터 지연 추정식 100/(158·BW) = %.2f ms (참고)\n', 100/(158*fsm.f_lpf)*1e3);
for i = 1:2
    si = stepinfo(G{i});
    fprintf('\n[%s축] 공진 %.0f Hz, Q = %.1f\n', ax_name{i}, fsm.f_res(i), fsm.Q(i));
    fprintf('  Bessel 군지연     : %.2f ms (등가 -3 dB %.1f Hz)\n', 1/a(i)*1e3, w3*a(i)/(2*pi));
    fprintf('  50%% 도달 시간     : %.2f ms (데이터시트 판독 %.1f ms)\n', t50_of(G{i}, t_eval)*1e3, fsm.t50_ds*1e3);
    fprintf('  상승 시간 10-90%%  : %.2f ms\n', si.RiseTime*1e3);
    fprintf('  오버슈트          : %.2f %%\n', si.Overshoot);
    fprintf('  정착 시간 2%%      : %.2f ms\n', si.SettlingTime*1e3);
    fprintf('  DC 이득           : %.4f (연속), %.4f (이산)\n', dcgain(G{i}), dcgain(Gd{i}));
end
fprintf('\n이산 모델: Ts = %.4g s\n', Ts);
if max(fsm.f_res) > 0.25/Ts
    fprintf('참고: 공진(%.0f Hz)이 Nyquist(%.0f Hz)에 가까워 이산 모델 정확도가 낮음\n', ...
        max(fsm.f_res), 0.5/Ts);
end

%% 5. 그림
if show_plots
    opts = bodeoptions;
    opts.FreqUnits = 'Hz';
    opts.Grid = 'on';

    figure('Name', 'FSM Bode', 'Position', [100 100 900 650]);
    bode(H_lpf{1}, H_res{1}, G{1}, G{2}, Gd{1}, opts);
    legend({'6차 Bessel LPF', '기계 공진 (X)', '전체 X (연속)', '전체 Y (연속)', ...
        sprintf('전체 X (이산, Ts = %.3g s)', Ts)}, 'Location', 'southwest');
    title('FSM 동특성 모델 Bode 선도');

    figure('Name', 'FSM Step', 'Position', [150 150 900 450]);
    step(G{1}, G{2}, Gd{1}, 0.04); grid on; hold on;
    xline(fsm.t50_ds, 'k:', '데이터시트 50%', 'HandleVisibility', 'off');
    legend({'X (연속)', 'Y (연속)', sprintf('X (이산, Ts = %.3g s)', Ts)}, 'Location', 'southeast');
    title('FSM 동특성 모델 Step 응답 (정규화)');
end

%% local functions
function H = make_lpf(p_norm, s)
% 정규화 Bessel 극점을 s배 스케일한 저역통과 필터 (DC 이득 1)
p = p_norm * s;
H = zpk([], p, real(prod(-p)));
end

function t50 = t50_of(sys, t)
% step 응답이 처음 0.5에 도달하는 시간 (선형 보간)
y = step(sys, t);
k = find(y >= 0.5, 1);
t50 = interp1(y(k-1:k), t(k-1:k), 0.5);
end