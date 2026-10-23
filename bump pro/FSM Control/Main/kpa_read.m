%% kpa_read.m
% KPA101 (Monitor 모드)에서 XDiff, YDiff, SUM 읽기 테스트
% 기존 시뮬레이션 코드(qpd_model, estimator 등)와 무관한 단독 스크립트
% API 이름 출처: Thorlabs Motion_Control_Examples, Python/Kinesis/KCube/KPA101/KPA101_pythonnet.py
%               Kinesis DotNet_API.chm으로 확인

clearvars -except mMTIDevice
clc;

%% 설정
use_sim  = false;   % true: Kinesis Simulator의 가상 KPA101 사용
serialNo = '69253238';
poll_ms  = 50;      % Kinesis 상태 갱신 주기 [ms]. Status 값은 이 주기로만 바뀜
N_read   = 20;      % 읽을 횟수
dt_read  = 0.1;     % 읽기 간격 [s]

kin    = 'C:\Program Files\Thorlabs\Kinesis\';
devAsm = NET.addAssembly([kin 'Thorlabs.MotionControl.DeviceManagerCLI.dll']);
kpaAsm = NET.addAssembly([kin 'Thorlabs.MotionControl.KCube.PositionAlignerCLI.dll']);
import Thorlabs.MotionControl.DeviceManagerCLI.*
import Thorlabs.MotionControl.KCube.PositionAlignerCLI.*

if use_sim, SimulationManager.Instance.InitializeSimulations(); end

%% 1. 장치 검색
DeviceManagerCLI.BuildDeviceList();
list = DeviceManagerCLI.GetDeviceList(KCubePositionAligner.DevicePrefix);
fprintf('찾은 KPA101: %d개\n', list.Count);
for i = 0:list.Count-1
    fprintf('  %s\n', char(list.Item(i)));
end
if isempty(serialNo)
    if list.Count == 0
        error('KPA101을 찾지 못함 (USB 연결, Kinesis 프로그램이 꺼져 있는지 확인)');
    end
    serialNo = char(list.Item(0));
end

%% 2. 연결
dev = KCubePositionAligner.CreateKCubePositionAligner(serialNo);
dev.Connect(serialNo);

try
    dev.WaitForSettingsInitialized(10000);
    dev.StartPolling(poll_ms);
    pause(0.5);
    dev.EnableDevice();
    pause(0.5);
    info = dev.GetDeviceInfo();
    fprintf('연결됨: %s (%s)\n', char(info.Description), serialNo);

    %% 3. Monitor 모드 설정 (KPA101 자체 PID 사용 안 함)
    modeType = kpaAsm.AssemblyHandle.GetType( ...
        'Thorlabs.MotionControl.KCube.PositionAlignerCLI.PositionAlignerStatus+OperatingModes');
    monitorMode = System.Enum.Parse(modeType, 'Monitor');
    dev.SetOperatingMode(monitorMode, false);
    pause(0.5);

    %% 4. 읽기
    data = zeros(N_read, 4);                        % [t XDiff YDiff SUM]
    t0 = tic;
    for k = 1:N_read
        st = dev.Status;
        xd   = System.Convert.ToDouble(st.PositionDifference.X);
        yd   = System.Convert.ToDouble(st.PositionDifference.Y);
        sumV = System.Convert.ToDouble(st.Sum);
        data(k,:) = [toc(t0), xd, yd, sumV];
        fprintf('%6.2f s   XDiff %8.4f V   YDiff %8.4f V   SUM %8.4f V\n', data(k,:));
        pause(dt_read);
    end
catch ME
    kpa_close(dev, use_sim);    % 에러 나면 연결 해제 후 에러 표시
    rethrow(ME);
end
kpa_close(dev, use_sim);        % 정상 종료 시 연결 해제

% 추정기에 바로 넣을 수 있는 형식 (qpd_model 출력과 같은 필드 이름)
sig = struct('XDiff', data(:,2), 'YDiff', data(:,3), 'SUM', data(:,4));

%% local functions
function kpa_close(dev, use_sim)
try, dev.StopPolling(); end
try, dev.Disconnect(true); end
if use_sim
    Thorlabs.MotionControl.DeviceManagerCLI.SimulationManager.Instance.UninitializeSimulations();
end
fprintf('연결 해제\n');
end