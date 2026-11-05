%% clear_mws.m
% PID_control.slx의 Model Workspace 변수를 모두 지우고 저장
% 이후 Simulink는 Base Workspace (init_pid_sim.m이 만든 변수)만 사용
% 실행 전 원본을 PID_control_orig.slx로 복사해 둠

mdl = 'PID_control';
slx = which([mdl '.slx']);
bak = fullfile(fileparts(slx), [mdl '_orig.slx']);

if ~isfile(bak)
    copyfile(slx, bak);                      % 원본 백업 (한 번만)
    fprintf('원본 백업: %s\n', bak);
end

load_system(mdl);
mws = get_param(mdl, 'ModelWorkspace');
fprintf('정리 전 Model Workspace 변수 %d개\n', numel(whos(mws)));

clear(mws);                                  % Model Workspace 변수 전부 삭제
save_system(mdl);

fprintf('정리 후 Model Workspace 변수 %d개 -> 저장 완료\n', numel(whos(mws)));