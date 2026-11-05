%% backup.m
% PID_control.slx의 Model Workspace 변수를 mws_backup.mat로 저장
% (init_pid_sim.m 만들기 전에 동기가 넣어둔 임시값 백업용)

mdl = 'PID_control';
load_system(mdl);                                   % 창 없이 모델 불러오기
mws = get_param(mdl, 'ModelWorkspace');

out_file = fullfile(fileparts(which(mdl)), 'mws_backup.mat');   % slx와 같은 폴더에 저장
save(mws, out_file);

fprintf('저장 완료: %s\n', out_file);
disp(whos('-file', out_file));                      % 저장된 변수 목록 확인