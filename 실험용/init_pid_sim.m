%% init_pid_sim.m
% PID_control.slx 실행 전에 먼저 실행. Simulink가 쓰는 변수를 전부 Base Workspace에 만든다.
% 순서: FSM Control 폴더에서 addpath(genpath(pwd)) -> init_pid_sim -> PID_control 실행
% 1~4: 동기가 Model Workspace에 넣어둔 임시값 그대로 (mws_backup.mat)

%% 1. 시뮬레이션 / 제어 주기  (임시, E2 실측으로 교체)
Ts    = 1e-3;     % [s] 제어 주기
delay = 0;        % [s] 루프 지연
Tstop = 2;        % [s] 시뮬레이션 길이

%% 2. 플랜트  (임시)
Gx = 1;           % FSM X DC 이득 (정규화)  -> FSM 블록 교체 후 사용 안 함
Gy = 1;           % FSM Y DC 이득 (정규화)  -> FSM 블록 교체 후 사용 안 함
J  = diag([38.6 35.7]);   % [mm/u] S3 (L = 200 mm, 22.5°) 크기만, 부호는 E3에서 확인 후 반영
u_max = 0.58;     % [u] 명령 제한 = 100 V / 173 V (데이터시트 전압-각도 선형 구간, 그래프 판독)


%% 2-1. FSM 동특성 모델 (S4, 데이터시트 기반)
% Gdx_num, Gdx_den, Gdy_num, Gdy_den 생성 (위의 Ts로 이산화)
show_plots = false;
fsm_dynamics_model;

%% 3. PID  (임시, 튜닝 전)
Kp_x = 0.1/38.6;  Ki_x = 0;  Kd_x = 0;  N_x = 100;
Kp_y = 0.1/35.7;  Ki_y = 0;  Kd_y = 0;  N_y = 100;

%% 4. 기준값 / 외란
Xref = 0;  Xref_step = 0;     % [mm]
Yref = 0;  Yref_step = 0;     % [mm]
dx   = 0;  dx_step   = 0;     % [mm]
dy   = 0;  dy_step   = 0;     % [mm]
step_time = 0.5;              % [s]

%% 5. Sensor (QPD + 위치 추정기) Lookup Table용 데이터
det.R    = 3.9;    % [mm]  compare_estimators.m과 같은 값
det.gap  = 0;
det.resp = 0.4;    % [A/W]
det.G    = 1e4;    % [V/A]

r0_sensor = 1.0;   % [mm] 임시, E3에서 실측 r0로 교체
alpha     = 0.9;   % Hybrid 가중치
est_id    = 3;     % 1 = DoS, 2 = LogRatio, 3 = Hybrid

rs = estimator_ranges(r0_sensor, det, alpha);
ok = all(isfinite(rs.Xh), 1);          % LogRatio/Hybrid NaN 구간 제외 (세 추정기 공통 범위)
Xs_sensor   = rs.Xs(ok);               % [mm] Lookup Table breakpoints
Xhat_DoS    = rs.Xh(1, ok);            % [mm]
Xhat_Log    = rs.Xh(2, ok);            % [mm]
Xhat_Hybrid = rs.Xh(3, ok);            % [mm]

Xhat_list   = {Xhat_DoS, Xhat_Log, Xhat_Hybrid};
Xhat_sensor = Xhat_list{est_id};       % Sensor Lookup Table 데이터 (est_id로 선택)

Ys_sensor   = Xs_sensor;               % [mm] Y축 절점 (지금은 X와 같음, E3 후 Y 실측으로 교체)
Yhat_sensor = Xhat_sensor;             % [mm] Y축 테이블 (지금은 X와 같음, E3 후 Y 실측으로 교체)

fprintf('init_pid_sim 완료: Ts = %.4g s, est_id = %d, Sensor 범위 ±%.2f mm\n', ...
    Ts, est_id, max(Xs_sensor));