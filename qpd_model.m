function sig = qpd_model(X, Y, beam, det, noise)
%QPD_MODEL  PDQ80A 출력 모델 (가우시안 빔 + 유한 원형 센서 + gap 파라미터)
%
%   sig = qpd_model(X, Y, beam, det)
%   sig = qpd_model(X, Y, beam, det, noise)
%
%   입력
%     X, Y        검출면 위 광점 중심 [mm], 같은 크기의 배열
%                 +X = 오른쪽, +Y = 위쪽 (빔이 들어오는 방향에서 PDQ80A를 정면으로 본 기준)
%     beam.r0     1/e^2 빔 반경 [mm]
%     beam.P      검출면 입사 광파워 [W]
%     det.R       센서 반경 [mm]                  (PDQ80A: 7.8 mm 직경 -> 3.9)
%     det.gap     사분면 gap 폭 [mm]              (필수. 값 미확인이면 0 = 이상적 조건으로 명시)
%     det.resp    응답도 [A/W]
%     det.G       트랜스임피던스 이득 [V/A]
%     noise       (선택, 모두 기본 0) -- 첫 비교용 단순 모델이며 실측 잡음 모델이 아님
%       .sigma_quad   사분면별 독립 백색 가우시안 잡음 표준편차 [V]
%                     (이론적 QPD 사분면 잡음 연구용)
%       .sigma_out    출력단 백색 가우시안 잡음 표준편차 [V], 1x3, [XDiff YDiff SUM] 순
%                     (실제 KPA101에서 측정한 출력 잡음 재현용, 채널 간 독립 가정)
%       sigma_quad와 sigma_out은 동시에 넣지 말 것 (잡음 중복). 둘 다 0보다 크면 에러.
%       .offset_quad  사분면별 DC offset [V], 1x4, A B C D 순
%                     (합성 offset / 사분면 불일치 시험용. 데이터시트 offset과 다름)
%       .offset_out   출력단 DC offset [V], 1x3, [XDiff YDiff SUM] 순
%                     (KPA101 출력에 직접 더해짐. 실제 dark 측정값은 여기에 넣을 것.
%                      데이터시트 'Signal Output Offset 100 mV max'가 어느 단 기준인지는 미확인)
%
%   출력 sig (구조체, 각 필드는 X와 같은 크기)
%     .A .B .C .D           사분면 전압 [V]
%     .XDiff .YDiff .SUM    [V]
%
%   사분면 정의 (Thorlabs 기준)
%     A = Q1 우상,  B = Q2 좌상,  C = Q3 좌하,  D = Q4 우하
%     XDiff = (B+C) - (A+D)   -> 광점이 오른쪽으로 가면 음수
%     YDiff = (A+B) - (C+D)   -> 광점이 위로 가면 양수
%     SUM   = A+B+C+D
%
%   계산 방법
%     x 방향은 가우시안을 erf로 해석적으로 적분하고, y 방향은 수치 적분(trapz).
%     원형 센서 경계(x = ±sqrt(R^2-y^2))와 gap(|x|<gap/2, |y|<gap/2 영역 제외)을 그대로 반영.
%
%   이 함수의 출력 형식(XDiff, YDiff, SUM)은 실제 KPA101 읽기 함수와 동일하게 맞춘다.
%   추정기는 sig만 입력으로 받으므로, 실험 단계에서는 이 함수만 KPA101 입력으로 교체하면 된다.

    if nargin < 5 || isempty(noise), noise = struct(); end
    if ~isfield(det, 'gap') || isempty(det.gap) || isnan(det.gap)
        error('qpd_model:gap', 'det.gap 값을 명시해야 함 (미확인이면 0 = 이상적 조건).');
    end
    if isfield(noise, 'sigma')
        error('qpd_model:noise', 'noise.sigma는 sigma_quad 또는 sigma_out으로 바뀜.');
    end
    sigma    = getf(noise, 'sigma_quad', 0);
    sig_out  = getf(noise, 'sigma_out', [0 0 0]);
    if sigma > 0 && any(sig_out > 0)
        error('qpd_model:noise', 'sigma_quad와 sigma_out을 동시에 넣으면 잡음이 중복됨. 하나만 사용할 것.');
    end
    off_quad = getf(noise, 'offset_quad', [0 0 0 0]);
    off_out  = getf(noise, 'offset_out',  [0 0 0]);

    sz = size(X);
    X = X(:); Y = Y(:);                    % N x 1
    w = beam.r0; R = det.R; g = det.gap; h = g/2;

    Ny = 2001;
    yu = linspace(h, R, Ny);               % 위쪽 반평면 적분 구간 (1 x Ny)
    yl = -yu;                              % 아래쪽 반평면
    xu = sqrt(max(R^2 - yu.^2, 0));        % 해당 y에서 센서 x 경계

    I0 = 2*beam.P/(pi*w^2);                % 피크 세기 [W/mm^2]
    c  = w*sqrt(pi/8);                     % ∫exp(-2(x-X)^2/w^2)dx 계수
    s  = sqrt(2)/w;

    % x 방향 적분 (N x Ny), xu <= h 인 행은 0
    valid = double(xu > h);
    Ixr = c*(erf(s*(xu - X)) - erf(s*(h - X))) .* valid;     % 오른쪽
    Ixl = c*(erf(s*(-h - X)) - erf(s*(-xu - X))) .* valid;   % 왼쪽

    Gu = exp(-2*(yu - Y).^2/w^2);
    Gl = exp(-2*(yl - Y).^2/w^2);

    % 사분면 광파워 [W]
    PA = I0*trapz(yu, Gu.*Ixr, 2);         % 우상
    PB = I0*trapz(yu, Gu.*Ixl, 2);         % 좌상
    PC = I0*trapz(yu, Gl.*Ixl, 2);         % 좌하 (yl은 yu의 부호 반전, 구간 길이 동일)
    PD = I0*trapz(yu, Gl.*Ixr, 2);         % 우하

    k = det.resp*det.G;                    % W -> V
    N = numel(X);
    V = k*[PA PB PC PD] + repmat(off_quad(:).', N, 1);
    if sigma > 0
        V = V + sigma*randn(N, 4);
    end

    sig.A = reshape(V(:,1), sz);
    sig.B = reshape(V(:,2), sz);
    sig.C = reshape(V(:,3), sz);
    sig.D = reshape(V(:,4), sz);
    sig.XDiff = (sig.B + sig.C) - (sig.A + sig.D);
    sig.YDiff = (sig.A + sig.B) - (sig.C + sig.D);
    sig.SUM   =  sig.A + sig.B + sig.C + sig.D;

        % 출력단 offset + 출력단 잡음 (KPA101 출력 기준)
    sig.XDiff = sig.XDiff + off_out(1);
    sig.YDiff = sig.YDiff + off_out(2);
    sig.SUM   = sig.SUM   + off_out(3);
    if any(sig_out > 0)
        sig.XDiff = sig.XDiff + sig_out(1)*randn(sz);
        sig.YDiff = sig.YDiff + sig_out(2)*randn(sz);
        sig.SUM   = sig.SUM   + sig_out(3)*randn(sz);
    end
end

function v = getf(s, name, default)
    if isfield(s, name) && ~isempty(s.(name)), v = s.(name); else, v = default; end
end
