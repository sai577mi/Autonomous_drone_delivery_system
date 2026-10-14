function R = compare_pid_lqr_errors(varargin)
%COMPARE_PID_LQR_ERRORS  Error graphs (PID - LQR) for all 18 signals.
%
%   SINGLE FILE: contains the PID simulation (pidSim) and the LQR simulation
%   (lqrSim) as local functions. Just run:
%       compare_pid_lqr_errors
%       compare_pid_lqr_errors('seed',7)
%       compare_pid_lqr_errors('drone',1)      % which LQR drone to compare
%
%   18 signals = 6 groups x 3 axes:
%     Position X/Y/Z, Velocity Vx/Vy/Vz, Acceleration ax/ay/az,
%     Angle roll/pitch/yaw, Angular rate, Angular acceleration.
%
%   error(t) = PID signal(t) - LQR signal(t), on a common time grid.
%
%   NOTE: the two programs use different scenes (PID: customer at [210 20],
%   fixed obstacles; LQR: customer at [210 0] for 1 drone, random obstacles),
%   so the error is a controller + scenario difference, not a pure
%   controller-only difference. See the comments at the bottom for how to
%   make it a fair comparison.

opt = struct('seed',7,'drone',1);
for ii = 1:2:numel(varargin), opt.(varargin{ii}) = varargin{ii+1}; end

%% ---- run both simulations (no animation) ---------------------------------
Lp = pidSim('seed',opt.seed);        % PID
Rl = lqrSim('nDrones',1,'seed',opt.seed);    % LQR

tP = Lp.t(1:Lp.nR);
tL = Rl.G.t(1:Rl.nR);
dr = min(opt.drone, numel(Rl.L));

%% ---- build the 18 signals for each controller ------------------------------
[SP, names, units] = sig18(tP, Lp.p(:,1:Lp.nR), Lp.v(:,1:Lp.nR), Lp.ang(:,1:Lp.nR));
[SL, ~,     ~    ] = sig18(tL, Rl.L(dr).p(:,1:Rl.nR), Rl.L(dr).v(:,1:Rl.nR), Rl.L(dr).ang(:,1:Rl.nR));

%% ---- common time grid and error -----------------------------------------------
dtc = 0.025;
tc  = 0:dtc:min(tP(end), tL(end));
E   = zeros(18, numel(tc));
for k = 1:18
    E(k,:) = interp1(tP, SP(k,:), tc, 'linear') - interp1(tL, SL(k,:), tc, 'linear');
end

%% ---- figure: 18 error graphs ----------------------------------------------------
col = {[0.85 0.1 0.1],[0.1 0.6 0.1],[0.1 0.3 0.9]};
f = figure('Color','w','Position',[30 30 1500 1100],'Name','PID - LQR error graphs','NumberTitle','off');
tl = tiledlayout(f,6,3,'TileSpacing','compact','Padding','compact');
title(tl,'Error graphs: PID - LQR (18 signals)','FontWeight','bold');
rmsE = sqrt(mean(E.^2,2));
for k = 1:18
    ax = nexttile(tl); hold(ax,'on'); grid(ax,'on');
    plot(ax, tc, E(k,:), 'Color', col{mod(k-1,3)+1}, 'LineWidth', 1.2);
    yline(ax, 0, 'k-', 'HandleVisibility','off');
    title(ax, sprintf('%s   (RMS %.3g)', names{k}, rmsE(k)), 'FontSize', 9);
    ylabel(ax, ['\Delta ' units{k}]);
    if k > 15, xlabel(ax,'Time [s]'); end
    xlim(ax,[tc(1) tc(end)]);
end

%% ---- figure: overlay PID vs LQR (to see where the error comes from) -------------------
f2 = figure('Color','w','Position',[70 70 1500 1100],'Name','PID vs LQR overlay','NumberTitle','off');
t2 = tiledlayout(f2,6,3,'TileSpacing','compact','Padding','compact');
title(t2,'PID (blue) vs LQR (red)','FontWeight','bold');
for k = 1:18
    ax = nexttile(t2); hold(ax,'on'); grid(ax,'on');
    plot(ax, tP, SP(k,:), 'b', 'LineWidth', 1.0);
    plot(ax, tL, SL(k,:), 'r', 'LineWidth', 1.0);
    title(ax, names{k}, 'FontSize', 9); ylabel(ax, units{k});
    if k > 15, xlabel(ax,'Time [s]'); end
    if k == 1, legend(ax,'PID','LQR','Location','best'); end
end

%% ---- table of RMS / max error -------------------------------------------------------------
fprintf('\n%-26s %12s %12s   %s\n','Signal','RMS error','Max |error|','unit');
for k = 1:18
    fprintf('%-26s %12.4g %12.4g   %s\n', names{k}, rmsE(k), max(abs(E(k,:))), units{k});
end

R.t = tc; R.error = E; R.names = names; R.units = units;
R.rms = rmsE; R.pid = SP; R.lqr = SL;
end

%% ======================================================================
function [S, names, units] = sig18(t, pos, vel, angRad)
% Same processing as plotSixDOF in both programs.
n   = numel(t);
ang = rad2deg(angRad);
acc = zeros(3,n); rate = zeros(3,n); aacc = zeros(3,n);
for i = 1:3
    acc(i,:)  = movmean(gradient(vel(i,:), t), 9);
    rate(i,:) = movmean(rad2deg(gradient(unwrap(deg2rad(ang(i,:))), t)), 9);
    aacc(i,:) = movmean(gradient(rate(i,:), t), 15);
end
S = [pos; vel; acc; ang; rate; aacc];
ax3 = {'X','Y','Z'; 'Vx','Vy','Vz'; 'ax','ay','az'; ...
       'Roll','Pitch','Yaw'; 'Roll rate','Pitch rate','Yaw rate'; ...
       'Roll accel','Pitch accel','Yaw accel'};
grp = {'Position','Velocity','Acceleration','Angle','Rate','Angular accel'};
un  = {'m','m/s','m/s^2','deg','deg/s','deg/s^2'};
names = cell(18,1); units = cell(18,1);
for g = 1:6
    for i = 1:3
        names{(g-1)*3+i} = sprintf('%s - %s', grp{g}, ax3{g,i});
        units{(g-1)*3+i} = un{g};
    end
end
end

%% ----------------------------------------------------------------------
% FAIR COMPARISON (optional): to isolate the controller, both programs must fly
% the same mission and obstacles. Easiest way: in lqr_defaultConfig use
% nDrones = 1 and set cfg.routeLength / waypoint fractions so cust = [210;20;3]
% matches the PID mission, and set cfg.nObstacles = 0 (and remove obstacles(1:6)
% from the PID file, or set its avoidance radius to 0). Then the error reflects
% PID vs LQR only.

%% ======================================================================
%%  PID SIMULATION
%% ======================================================================
function out = pidSim(varargin)
% PID delivery drone with dynamic 3D obstacle avoidance (simulation core only).
out = [];
opts = struct('seed',7,'tMax',240);
for ii = 1:2:numel(varargin), opts.(varargin{ii}) = varargin{ii+1}; end
try, rng(opts.seed); catch, randn('state',opts.seed); rand('state',opts.seed); end %#ok<NASGU>

P = pid_defaultParams(opts);
[bld, trees] = pid_makeScene(); %#ok<ASGLU>
segs = pid_makeMission();
obs  = pid_makeObstacles();
nO   = numel(obs);

zeroOff = struct('name','NONE','z',NaN,'lat',0,'along',0,'speedScale',1);
A = struct('lastKey','','active',false,'off',zeroOff,'clearTimer',0,'det',[], ...
           'seen',false(1,nO),'threat',false(1,nO));

%% ------------------------------------------------------------------ state
dt = P.dt;
d = P.d; c = P.c;
Amix = [1 1 1 1; -d d d -d; -d -d d d; -c c -c c];     % [T;tx;ty;tz] = Amix*F
invA = inv(Amix);

p = [0;0;0]; v = zeros(3,1); ang = zeros(3,1); om = zeros(3,1);
wm = P.wMin*ones(4,1);
payload = P.mPayload; released = false;
pkgP = [0;0;0]; pkgV = zeros(3,1);
pref = p; vref = zeros(3,1);
integ = zeros(3,1); eprev = zeros(3,1);
psiCmd = 0; gust = zeros(3,1);
k = 1; tSeg = 0; sg = segs(k);
landed = false;
minSep = inf; minSepT = 0; nAvoid = 0; lastName = 'NONE';
nReplan = round(P.replanDt/dt);

nSteps = round(P.tMax/dt);
nRec = floor(nSteps/P.recStride) + 2;
Lg.t = zeros(1,nRec);  Lg.p = zeros(3,nRec);  Lg.v = zeros(3,nRec);
Lg.ang = zeros(3,nRec); Lg.wm = zeros(4,nRec); Lg.T = zeros(1,nRec); Lg.W = zeros(1,nRec);
Lg.acmd = zeros(3,nRec); Lg.pref = zeros(3,nRec); Lg.carrot = zeros(3,nRec);
Lg.zMis = zeros(1,nRec); Lg.wind = zeros(3,nRec); Lg.seg = zeros(1,nRec);
Lg.mode = cell(1,nRec); Lg.pay = zeros(1,nRec);
r = 0;
Ttot = 0; a_cmd = zeros(3,1); carrot = p;

fprintf('\n=== PID drone: dynamic obstacle-avoidance delivery ===\n');
fprintf('[t=%6.2f s] MISSION: %s\n', 0, segs(1).name);

%% ------------------------------------------------------------------ main loop
for it = 1:nSteps
    t = (it-1)*dt;
    mTot = P.mFrame + payload;

    obs = pid_stepObstacles(obs, dt, t);
    [wind, gust] = pid_windModel(t, gust, P);

    if mod(it-1, nReplan) == 0
        A = pid_avoidanceUpdate(A, t, p, v, sg, obs, P, bld, zeroOff);
        if A.active && ~strcmp(A.off.name, lastName), nAvoid = nAvoid + 1; end
        if A.active, lastName = A.off.name; else, lastName = 'NONE'; end
    end

    sg = segs(k);
    done = false;
    switch k
        case 1, done = p(3) > 9.5;
        case {2,3}, done = norm(p(1:2)-sg.T(1:2)) < 4;
        case 4, done = norm(p(1:2)-sg.T(1:2)) < 1.5 && p(3) < sg.T(3)+0.6 && norm(v) < 0.7;
        case 5
            if ~released && (t - tSeg) > 2.0
                released = true; payload = 0; pkgP = p - [0;0;0.4]; pkgV = v;
                fprintf('[t=%6.2f s] DELIVERY: package released (mass %.1f -> %.1f kg)\n', t, mTot, P.mFrame);
            end
            done = (t - tSeg) > 4.5;
        case 6, done = p(3) > sg.T(3) - 0.5;
        case 7, done = norm(p(1:2)-sg.T(1:2)) < 3;
        case 8, done = p(3) < 0.12;
    end
    if done
        if k == 8, landed = true; break; end
        k = k + 1; tSeg = t; sg = segs(k);
        fprintf('[t=%6.2f s] MISSION: %s\n', t, sg.name);
    end

    off = A.off;
    carrot = pid_carrotTarget(p, sg, off, P);
    [pref, vref] = pid_refStep(pref, vref, carrot, off.speedScale, P, dt);
    lead = pref - p; nl = norm(lead);
    if nl > P.maxLead, pref = p + lead*P.maxLead/nl; end

    % ---- position PID (separate X, Y, Z)
    e = pref - p;
    integ = integ + e*dt;
    integ = max(min(integ, P.IntLim), -P.IntLim);
    deriv = (e - eprev)/dt;  eprev = e;
    a_cmd = P.Kp.*e + P.Ki.*integ + P.Kd.*deriv;
    ah = norm(a_cmd(1:2));
    if ah > P.accH, a_cmd(1:2) = a_cmd(1:2)*P.accH/ah; end
    a_cmd(3) = min(max(a_cmd(3), P.accDown), P.accUp);

    Fw = mTot*(a_cmd + [0;0;P.g]);
    R  = eul2R(ang);
    Tcmd = min(max(dot(Fw, R(:,3)), 0), P.Tmax);

    if norm(vref(1:2)) > 1.5
        dpsi = wrapPi(atan2(vref(2), vref(1)) - psiCmd);
        psiCmd = psiCmd + max(min(dpsi, P.yawRate*dt), -P.yawRate*dt);
    end
    an = a_cmd + [0;0;P.g];
    axb =  cos(psiCmd)*an(1) + sin(psiCmd)*an(2);
    ayb = -sin(psiCmd)*an(1) + cos(psiCmd)*an(2);
    th_d  = max(min(atan2(axb, an(3)),  P.maxTilt), -P.maxTilt);
    phi_d = max(min(atan2(-ayb, an(3)), P.maxTilt), -P.maxTilt);
    eang = [phi_d; th_d; psiCmd] - ang;  eang(3) = wrapPi(eang(3));

    alpha = P.KattP.*eang - P.KattD.*om;
    tau   = P.J*alpha + cross(om, P.J*om);

    Fm   = invA*[Tcmd; tau];
    Fm   = min(max(Fm, P.kf*P.wMin^2), P.kf*P.wMax^2);
    wcmd = sqrt(Fm/P.kf);
    wm   = wm + (wcmd - wm)/P.tauM*dt;
    Fa   = P.kf*wm.^2;
    Ttot = sum(Fa);
    tauA = Amix(2:4,:)*Fa;

    omdot = P.J \ (tauA - cross(om, P.J*om));
    om = om + omdot*dt;
    sp = sin(ang(1)); cp = cos(ang(1)); tt = tan(ang(2)); ct = cos(ang(2));
    angdot = [om(1) + (om(2)*sp + om(3)*cp)*tt;
              om(2)*cp - om(3)*sp;
              (om(2)*sp + om(3)*cp)/ct];
    ang = ang + angdot*dt;  ang(3) = wrapPi(ang(3));

    Fth = R*[0;0;Ttot];
    Fg  = [0;0;-mTot*P.g];
    Fdr = -P.cd*norm(v)*v;
    Fwd =  P.cd*norm(wind)*wind;
    a   = (Fth + Fg + Fdr + Fwd)/mTot;
    v = v + a*dt;  p = p + v*dt;
    if p(3) <= 0
        p(3) = 0; if v(3) < 0, v(3) = 0; end
        if Ttot < mTot*P.g
            v(1:2) = 0; om = zeros(3,1); ang(1:2) = 0;
        end
    end
    if released
        pkgV = pkgV + ([0;0;-P.g] - 0.3*norm(pkgV)*pkgV)*dt;
        pkgP = pkgP + pkgV*dt;
        if pkgP(3) < 0.1, pkgP(3) = 0.1; pkgV = zeros(3,1); end
    end

    for j = 1:nO
        sep = norm(obs(j).pos - p) - obs(j).r;
        if sep < minSep, minSep = sep; minSepT = t; end
    end

    if mod(it-1, P.recStride) == 0
        r = r + 1;
        Lg.t(r) = t; Lg.p(:,r) = p; Lg.v(:,r) = v; Lg.ang(:,r) = ang; Lg.wm(:,r) = wm;
        Lg.T(r) = Ttot; Lg.W(r) = mTot*P.g; Lg.acmd(:,r) = a_cmd; Lg.pref(:,r) = pref;
        Lg.carrot(:,r) = carrot; Lg.wind(:,r) = wind; Lg.seg(r) = k; Lg.pay(r) = payload;
        c0 = pid_carrotTarget(p, sg, zeroOff, P); Lg.zMis(r) = c0(3);
        if A.active, Lg.mode{r} = A.off.name; else, Lg.mode{r} = 'NONE'; end
    end
end

r = r + 1;
Lg.t(r) = (it-1)*dt; Lg.p(:,r) = p; Lg.v(:,r) = v; Lg.ang(:,r) = ang; Lg.wm(:,r) = wm;
Lg.T(r) = Ttot; Lg.W(r) = mTot*P.g; Lg.acmd(:,r) = a_cmd; Lg.pref(:,r) = pref;
Lg.carrot(:,r) = carrot; Lg.wind(:,r) = wind; Lg.seg(r) = k; Lg.pay(r) = payload;
Lg.zMis(r) = 0; Lg.mode{r} = 'NONE';
nR = r;

fprintf('\n=== PID Summary ===\n');
if landed, fprintf('Mission complete: landed at t = %.1f s\n', Lg.t(nR));
else,      fprintf('Mission NOT finished within %.0f s (last phase: %s)\n', P.tMax, segs(k).name); end
fprintf('Avoidance maneuvers : %d\n', nAvoid);
fprintf('Min separation      : %.2f m at t = %.1f s\n\n', minSep, minSepT);

out = Lg; out.nR = nR;
end

%% ======================================================================
function P = pid_defaultParams(opts)
P.g = 9.81;  P.mFrame = 1.5;  P.mPayload = 0.5;
P.J = diag([0.030 0.030 0.055]);
P.L = 0.25;  P.d = P.L/sqrt(2);
P.kf = 1e-5; P.km = 1.6e-7; P.c = P.km/P.kf;
P.wMax = 1100; P.wMin = 100; P.tauM = 0.03;
P.Tmax = 4*P.kf*P.wMax^2;
P.cd = 0.10;
P.dt = 0.005; P.tMax = opts.tMax; P.recStride = 5; P.replanDt = 0.25;
P.Kp = [2.2;2.2;4.5];  P.Ki = [0.4;0.4;0.8];  P.Kd = [2.6;2.6;3.6];  P.IntLim = [5;5;3];
P.KattP = [100;100;8]; P.KattD = [16;16;4];
P.vMaxH = 6;  P.aMaxH = 3;  P.vUp = 3;  P.vDown = 2;  P.aMaxV = 2;
P.kRef = 0.7; P.kRefZ = 0.9; P.maxLead = 6; P.look = 12;
P.accH = 4;   P.accUp = 6;  P.accDown = -5;
P.maxTilt = deg2rad(25);  P.yawRate = deg2rad(60);
P.radarRange = 40; P.Th = 8; P.dtPred = 0.1; P.zMin = 2.0; P.zMax = 45;
P.rDrone = 0.5; P.margin = 1.5; P.speedGain = 0.15; P.buf = 0.5; P.releaseTime = 1.5;
P.windMean = 3.5; P.windVar = 1.5; P.windDir = 0.5; P.gustTau = 2.0; P.gustSigma = 1.0;
P.visScale = 3.5;
end

function segs = pid_makeMission()
cust = [210;20;3];
names = {'TAKEOFF','WAYPOINT 1','WAYPOINT 2','CUSTOMER APPROACH','DELIVERY', ...
         'RETURN HOME (CLIMB)','RETURN HOME','LAND'};
S = {[0;0;0],  [0;0;10], [70;25;10], [140;-15;10], cust, cust, [210;20;10], [0;0;10]};
T = {[0;0;10], [70;25;10],[140;-15;10], cust,       cust, [210;20;10], [0;0;10], [0;0;0]};
vertOK = [false true true true true true true false];
for i = 1:numel(names)
    segs(i).name = names{i}; segs(i).S = S{i}; segs(i).T = T{i}; segs(i).vertOK = vertOK(i); %#ok<AGROW>
end
end

function [bld, trees] = pid_makeScene()
bld = [ 40 -22 14 12 18;
        55  42 16 14 24;
        30  22 10  8 20;
       118  40 14 12 16;
       160 -30 20 14 26;
       225  30 12 10  7;
       -12 -14  8  8  6;
       190 -45 16 16 22;
        90  60 12 12 15];
hash = @(k) mod(sin(k*12.9898 + 78.233)*43758.5453, 1);
kk = 0;
trees = zeros(0,3);
while size(trees,1) < 55
    kk = kk + 3; x = -30 + 280*hash(kk); y = -80 + 180*hash(kk+1);
    if abs(y) < 7 || abs(x-105) < 7, continue; end
    if hypot(x-210, y-20) < 25 || hypot(x, y) < 12, continue; end
    if pid_distPointToRoute([x;y]) < 14, continue; end
    if pid_inBuilding([x;y;0], bld), continue; end
    trees(end+1,:) = [x, y, 4 + 4*hash(kk+2)]; %#ok<AGROW>
end
end

function dmin = pid_distPointToRoute(q)
W = [0 0; 70 25; 140 -15; 210 20; 0 0]';
dmin = inf;
for i = 1:size(W,2)-1
    a = W(:,i); b = W(:,i+1); ab = b - a;
    s = min(max(dot(q-a, ab)/dot(ab,ab), 0), 1);
    dmin = min(dmin, norm(q - (a + s*ab)));
end
end

function obs = pid_makeObstacles()
obs(1) = pid_mk('bird',  [34;60;11.5], [0;-4;0],    0.5, [-20 -60 6], [260 140 18], 0.015, 0.3);
obs(2) = pid_mk('kite',  [105;6;12.0], [0.6;0.3;0], 1.0, [ 90  -8 10], [125  18 15], 0.0,   0.0);
obs(3) = pid_mk('drone', [283;56.5;7], [-3;-1.5;0], 0.8, [-20 -90  3], [300  90 25], 0.01,  0.0);
obs(4) = pid_mk('bird',  [150;150;9.5],[0;-2;0],    0.5, [ 0 -60 6],   [260 200 16], 0.02,  0.3);
obs(5) = pid_mk('balloon',[52;18.6;10.3],[0;0;0],   1.3, [-1e3 -1e3 0],[1e3 1e3 60], 0.0,   0.0);
obs(6) = pid_mk('bird',  [212;30;4.5], [0;-0.8;0],  0.5, [200 10 2.5], [220 40 8],   0.02,  0.2);
end

function o = pid_mk(type, pos, vel, r, lo, hi, turn, vzAmp)
o.type = type; o.pos = pos(:); o.vel = vel(:); o.v0 = vel(:); o.speed = norm(vel(1:2));
o.r = r; o.lo = lo(:); o.hi = hi(:); o.turn = turn; o.vzAmp = vzAmp;
o.phase = 6.28*rand;
end

function obs = pid_stepObstacles(obs, dt, t)
for i = 1:numel(obs)
    o = obs(i);
    switch o.type
        case 'bird'
            ang = o.turn*randn*sqrt(dt);
            cs = cos(ang); sn = sin(ang);
            o.vel(1:2) = [cs -sn; sn cs]*o.vel(1:2);
            o.vel(1:2) = o.vel(1:2)/max(norm(o.vel(1:2)),1e-6)*o.speed;
            o.vel(3) = o.vzAmp*cos(0.9*t + o.phase);
        case 'kite'
            o.vel = o.v0 + 0.4*[sin(0.5*t+o.phase); cos(0.4*t); 0.4*sin(0.7*t)];
        case 'drone'
            ang = o.turn*randn*sqrt(dt);
            cs = cos(ang); sn = sin(ang);
            o.vel(1:2) = [cs -sn; sn cs]*o.vel(1:2);
        otherwise
            o.vel = zeros(3,1);
    end
    o.pos = o.pos + o.vel*dt;
    for ax = 1:3
        if o.pos(ax) < o.lo(ax), o.pos(ax) = o.lo(ax); o.vel(ax) = abs(o.vel(ax)); o.v0(ax) = abs(o.v0(ax));
        elseif o.pos(ax) > o.hi(ax), o.pos(ax) = o.hi(ax); o.vel(ax) = -abs(o.vel(ax)); o.v0(ax) = -abs(o.v0(ax)); end
    end
    obs(i) = o;
end
end

function [wind, gust] = pid_windModel(t, gust, P)
Wm = P.windMean + P.windVar*sin(0.07*t);
th = P.windDir + 0.6*sin(0.045*t + 1);
a  = exp(-P.dt/P.gustTau);
gust = a*gust + sqrt(1-a^2)*P.gustSigma*[randn; randn; 0.3*randn];
wind = Wm*[cos(th); sin(th); 0] + gust;
end

function det = pid_radarScan(p, v, obs, P)
det = [];
for i = 1:numel(obs)
    rel = obs(i).pos - p;  dist = norm(rel);
    if dist > P.radarRange, continue; end
    relv = obs(i).vel - v;
    Rs = obs(i).r + P.rDrone + P.margin + P.speedGain*norm(relv);
    s.id = i; s.type = obs(i).type; s.dist = dist; s.rel = rel; s.relv = relv;
    s.height = obs(i).pos(3);
    s.dir = obs(i).vel/max(norm(obs(i).vel),1e-6);
    s.heading = atan2d(obs(i).vel(2), obs(i).vel(1));
    s.closing = -dot(rel, relv)/max(dist,1e-6);
    s.Rs = Rs;
    s.ttc = pid_timeToCollision(rel, relv, Rs);
    if isempty(det), det = s; else, det(end+1) = s; end %#ok<AGROW>
end
end

function ttc = pid_timeToCollision(rel, relv, Rs)
a = dot(relv,relv); b = 2*dot(rel,relv); cc = dot(rel,rel) - Rs^2;
if cc < 0, ttc = 0; return; end
if a < 1e-9, ttc = inf; return; end
disc = b^2 - 4*a*cc;
if disc < 0, ttc = inf; return; end
tt = (-b - sqrt(disc))/(2*a);
if tt < 0, ttc = inf; else, ttc = tt; end
end

function A = pid_avoidanceUpdate(A, t, p, v, sg, obs, P, bld, zeroOff)
det = pid_radarScan(p, v, obs, P);
A.det = det;
nO = numel(obs);
inRange = false(1,nO);
if ~isempty(det), inRange([det.id]) = true; end
for i = find(inRange & ~A.seen)
    s = det([det.id] == i);
    fprintf('[t=%6.2f s] RADAR   : %-7s #%d  dist=%5.1f m  TTC=%s  safety=%.1f m\n', ...
        t, s.type, i, s.dist, ttcStr(s.ttc), s.Rs);
end
A.seen = inRange;
A.threat = false(1,nO);

if isempty(det)
    A = pid_tryRelease(A, t, P, zeroOff, true);
    return
end

idx = [det.id];  Rs = [det.Rs];
ro0 = pid_rolloutRisk(p, v, sg, zeroOff, obs, idx, Rs, P, bld);
if ro0.margin >= P.buf
    A = pid_tryRelease(A, t, P, zeroOff, all([det.dist] > 1.5*Rs));
    return
end

A.clearTimer = 0;
A.threat(idx(ro0.per < P.buf)) = true;
needNew = true;
if A.active
    roc = pid_rolloutRisk(p, v, sg, A.off, obs, idx, Rs, P, bld);
    if roc.margin >= P.buf, needNew = false; end
end
if ~needNew, return; end

cands = pid_buildCandidates(p, sg, ro0, Rs, A, P, zeroOff);
c0 = pid_carrotTarget(p, sg, zeroOff, P);  z0 = c0(3);
bestCost = inf; best = []; bestM = -inf; fallback = [];
for q = 1:numel(cands)
    cand = cands{q};
    roq = pid_rolloutRisk(p, v, sg, cand, obs, idx, Rs, P, bld);
    cost = 1.8*abs(cand.lat) + 1.2*abs(cand.along);
    if ~isnan(cand.z)
        if cand.z >= z0, cost = cost + (cand.z - z0); else, cost = cost + 2.5*(z0 - cand.z); end
    end
    if strncmp(cand.name,'EMERG',5), cost = cost + 6; end
    if A.active && strcmp(cand.name, A.off.name), cost = 0.7*cost; end
    if roq.margin >= P.buf && cost < bestCost
        bestCost = cost; best = cand; best.margin = roq.margin;
    end
    if roq.margin > bestM, bestM = roq.margin; fallback = cand; fallback.margin = roq.margin; end
end
emergency = false;
if isempty(best), best = fallback; emergency = true; end

A.off = rmfield(best,'margin'); A.active = true;
if isnan(best.z), zs = 'mission alt'; else, zs = sprintf('%.1f m', best.z); end
key = sprintf('%s|%.0f|%.0f', best.name, best.lat, ternary(isnan(best.z), -1, best.z));
if strcmp(key, A.lastKey), return; end
A.lastKey = key;
fprintf('[t=%6.2f s] DECIDE  : %s%s -> target Z = %s, lateral = %+.1f m, along = %+.1f m  (predicted clearance %.1f m)\n', ...
    t, best.name, ternary(emergency,' (best effort)',''), zs, best.lat, best.along, best.margin);
end

function A = pid_tryRelease(A, t, P, zeroOff, farEnough)
if ~A.active, return; end
if farEnough, A.clearTimer = A.clearTimer + P.replanDt; else, A.clearTimer = 0; end
if A.clearTimer >= P.releaseTime
    A.active = false; A.off = zeroOff; A.clearTimer = 0; A.lastKey = '';
    fprintf('[t=%6.2f s] AVOID   : obstacle cleared -> avoidance OFF\n', t);
end
end

function cands = pid_buildCandidates(p, sg, ro, Rs, A, P, zeroOff) %#ok<INUSL>
thr = find(ro.per < P.buf);
S = sg.S; T = sg.T; dxy = T(1:2) - S(1:2); Ls = norm(dxy);
if Ls < 1, u = [1;0]; ref0 = T(1:2); else, u = dxy/Ls; ref0 = S(1:2); end
perp = [-u(2); u(1)];
zo = ro.tpos(3,thr);  Rt = Rs(thr);
lat_o = perp'*(ro.tpos(1:2,thr) - ref0);
extra = P.buf + 0.7;
zc = max(zo + Rt + extra);
zd = min(zo - Rt - extra);
latL = max(max(lat_o + Rt + extra), 1.0);
latR = min(min(lat_o - Rt - extra), -1.0);
mkc = @(n,z,l,a,s) struct('name',n,'z',z,'lat',l,'along',a,'speedScale',s);
cands = {};
canUp = sg.vertOK && zc <= P.zMax;
canDn = sg.vertOK && zd >= P.zMin;
if canUp, cands{end+1} = mkc('CLIMB', zc, 0, 0, 1); end
if canDn, cands{end+1} = mkc('DESCEND', zd, 0, 0, 1); end
cands{end+1} = mkc('LEFT',  NaN, latL, 0, 1);
cands{end+1} = mkc('RIGHT', NaN, latR, 0, 1);
if Ls >= 1
    cands{end+1} = mkc('FORWARD',  NaN, 0,  12, 1.35);
    cands{end+1} = mkc('BACKWARD', NaN, 0,  -8, 1);
end
if canUp
    zm = p(3) + 0.75*(zc - p(3));
    cands{end+1} = mkc('CLIMB+LEFT',  zm, 0.75*latL, 0, 1);
    cands{end+1} = mkc('CLIMB+RIGHT', zm, 0.75*latR, 0, 1);
    cands{end+1} = mkc('EMERGENCY 3D', min(zc+3,P.zMax), 0.5*latL, -4, 1);
end
if canDn
    zm = p(3) + 0.75*(zd - p(3));
    cands{end+1} = mkc('DESCEND+LEFT',  zm, 0.75*latL, 0, 1);
    cands{end+1} = mkc('DESCEND+RIGHT', zm, 0.75*latR, 0, 1);
end
end

function ro = pid_rolloutRisk(p, v, sg, off, obs, idx, Rs, P, bld)
N = round(P.Th/P.dtPred); nO = numel(idx);
ro.per = inf(1,nO); ro.tpos = zeros(3,nO); ro.tt = zeros(1,nO); ro.hit = false;
pq = p; vq = v;
for kk = 1:N
    tq = kk*P.dtPred;
    cq = pid_carrotTarget(pq, sg, off, P);
    [pq, vq] = pid_refStep(pq, vq, cq, off.speedScale, P, P.dtPred);
    for j = 1:nO
        o = obs(idx(j));
        po = o.pos + o.vel*tq;
        m = norm(pq - po) - Rs(j);
        if m < ro.per(j), ro.per(j) = m; ro.tpos(:,j) = po; ro.tt(j) = tq; end
    end
    if pid_inBuilding(pq, bld), ro.hit = true; end
end
ro.margin = min(ro.per);
if ro.hit, ro.margin = min(ro.margin, -5); end
end

function tf = pid_inBuilding(q, bld)
tf = false;
for b = 1:size(bld,1)
    if abs(q(1)-bld(b,1)) < bld(b,3)/2 + 3 && abs(q(2)-bld(b,2)) < bld(b,4)/2 + 3 && ...
       (numel(q) < 3 || q(3) < bld(b,5) + 2)
        tf = true; return;
    end
end
end

function c = pid_carrotTarget(p, sg, off, P)
S = sg.S; T = sg.T; dxy = T(1:2) - S(1:2); Ls = norm(dxy);
if Ls < 1
    perp = [0;1];
    cxy = T(1:2) + perp*off.lat;
    cz  = T(3);
else
    u = dxy/Ls; perp = [-u(2); u(1)];
    s  = min(max(dot(p(1:2)-S(1:2), u), 0), Ls);
    st = min(s + P.look, Ls) + off.along;
    st = min(max(st, -5), Ls);
    cxy = S(1:2) + u*st + perp*off.lat;
    cz  = S(3) + (T(3)-S(3))*min(max(st/Ls,0),1);
end
if ~isnan(off.z), cz = off.z; end
c = [cxy; cz];
end

function [pr, vr] = pid_refStep(pr, vr, carrot, vScale, P, dt)
e = carrot - pr; exy = e(1:2); dxy = norm(exy);
if dxy > 1e-6, vh = exy/dxy*min(P.kRef*dxy, P.vMaxH*vScale); else, vh = [0;0]; end
vdn = min(P.vDown, 0.5 + 0.6*pr(3));
vz  = min(max(P.kRefZ*e(3), -vdn), P.vUp);
dv  = [vh - vr(1:2); vz - vr(3)];
lim = P.aMaxH*dt; dvh = norm(dv(1:2));
if dvh > lim, dv(1:2) = dv(1:2)/dvh*lim; end
dv(3) = min(max(dv(3), -P.aMaxV*dt), P.aMaxV*dt);
vr = vr + dv;  pr = pr + vr*dt;
end


%% ===== shared small helpers (used by both PID and LQR parts) =====
function R = eul2R(a)
cph = cos(a(1)); sph = sin(a(1)); cth = cos(a(2)); sth = sin(a(2)); cps = cos(a(3)); sps = sin(a(3));
R = [cps*cth, cps*sth*sph - sps*cph, cps*sth*cph + sps*sph;
     sps*cth, sps*sth*sph + cps*cph, sps*sth*cph - cps*sph;
     -sth,    cth*sph,               cth*cph];
end

function x = wrapPi(x), x = atan2(sin(x), cos(x)); end

function s = ttcStr(x)
if isinf(x), s = 'inf'; else, s = sprintf('%.1f s', x); end
end

function out = ternary(cond, a, b)
if cond, out = a; else, out = b; end
end

%% ======================================================================
%%  LQR SIMULATION
%% ======================================================================
function out = lqrSim(varargin)
% Multi-drone LQR delivery swarm with dynamic 3D obstacle avoidance (simulation core only).
cfg = lqr_defaultConfig();
if mod(numel(varargin),2) ~= 0, error('Options must be name/value pairs.'); end
for ii = 1:2:numel(varargin)
    if ~isfield(cfg, varargin{ii}), continue; end
    cfg.(varargin{ii}) = varargin{ii+1};
end
rng(cfg.seed);
N = cfg.nDrones; dt = cfg.dt;

%% ------------------------------------------------------------ build everything
P = lqr_droneParams(1, N, cfg);  M = lqr_makeMission(1, N, cfg);
for i = 2:N, P(i) = lqr_droneParams(i, N, cfg); M(i) = lqr_makeMission(i, N, cfg); end
world = lqr_makeWorld(cfg, M);
[bld, trees] = lqr_makeScene(cfg, M, world); %#ok<ASGLU>
env  = lqr_makeObstacles(cfg, M, world);
nEnv = numel(env);
zeroOff = struct('name','NONE','z',NaN,'lat',0,'along',0,'speedScale',1);
lqr_printParams(P);

D = lqr_initDrone(P(1), M(1), nEnv+N-1, zeroOff);
for i = 2:N, D(i) = lqr_initDrone(P(i), M(i), nEnv+N-1, zeroOff); end

nSteps = round(cfg.tMax/dt);
nReplan = max(1, round(cfg.replanDt/dt));
recStride = max(1, round(cfg.recDt/dt));
nRec = floor(nSteps/recStride) + 2;
for i = 1:N
    L(i).p = zeros(3,nRec); L(i).v = zeros(3,nRec); L(i).ang = zeros(3,nRec); %#ok<AGROW>
    L(i).wm = zeros(4,nRec); L(i).T = zeros(1,nRec); L(i).W = zeros(1,nRec);
    L(i).pref = zeros(3,nRec); L(i).carrot = zeros(3,nRec); L(i).wind = zeros(3,nRec);
    L(i).seg = ones(1,nRec); L(i).pay = zeros(1,nRec); L(i).pkg = zeros(3,nRec);
    L(i).pkgOn = false(1,nRec); L(i).mode = cell(1,nRec);
end
G.t = zeros(1,nRec); G.minDD = nan(1,nRec); G.minDO = nan(1,nRec);
r = 0; minDD = inf; minDDt = 0; minDO = inf; minDOt = 0;

fprintf('\n=== LQR swarm of %d delivery drones ===\n', N);
for i = 1:N, fprintf('[t=%6.2f s] UAV%d MISSION: %s\n', 0, i, D(i).segs(1).name); end

%% ------------------------------------------------------------ main loop
for it = 1:nSteps
    t = (it-1)*dt;
    env = lqr_stepObstacles(env, dt, t);
    for i = 1:N
        if D(i).landed, continue; end
        if mod(it-1, nReplan) == 0
            obsI = [env, lqr_swarmObstacles(D, P, i)];
            D(i) = lqr_replan(D(i), t, obsI, P(i), bld, zeroOff);
        end
        [wnd, D(i).gust] = lqr_windModel(t, D(i).gust, cfg, dt);
        D(i).wind = wnd;
        D(i) = lqr_stepDrone(D(i), P(i), t, wnd);
    end

    dd = inf; dobs = inf;
    for i = 1:N
        for j = i+1:N
            dd = min(dd, norm(D(i).p - D(j).p) - P(i).rBody - P(j).rBody);
        end
        for j = 1:nEnv
            dobs = min(dobs, norm(env(j).pos - D(i).p) - env(j).r - P(i).rBody);
        end
    end
    if dd   < minDD, minDD = dd;   minDDt = t; end
    if dobs < minDO, minDO = dobs; minDOt = t; end

    isLast = all([D.landed]) || it == nSteps;
    if mod(it-1, recStride) == 0 || isLast
        r = r + 1;
        G.t(r) = t; G.minDD(r) = dd; G.minDO(r) = dobs;
        for i = 1:N
            L(i).p(:,r) = D(i).p; L(i).v(:,r) = D(i).v; L(i).ang(:,r) = D(i).ang;
            L(i).wm(:,r) = D(i).wm; L(i).T(r) = D(i).Ttot;
            L(i).W(r) = (P(i).mFrame + D(i).payload)*P(i).g;
            L(i).pref(:,r) = D(i).pref; L(i).carrot(:,r) = D(i).carrot;
            L(i).wind(:,r) = D(i).wind; L(i).seg(r) = D(i).k; L(i).pay(r) = D(i).payload;
            L(i).pkg(:,r) = D(i).pkgP; L(i).pkgOn(r) = D(i).released;
            if D(i).A.active, L(i).mode{r} = D(i).A.off.name; else, L(i).mode{r} = 'NONE'; end
        end
    end
    if isLast, break; end
end
nR = r;

fprintf('\n=== LQR Summary ===\n');
for i = 1:N
    if D(i).landed, fprintf('UAV%d: mission complete, landed.  avoid maneuvers %d\n', i, D(i).nAvoid);
    else, fprintf('UAV%d: NOT finished (last phase: %s).\n', i, D(i).segs(D(i).k).name); end
end
fprintf('Total flight time        : %.1f s\n', G.t(nR));
fprintf('Min drone-drone clearance: %.2f m at t = %.1f s\n', minDD, minDDt);
fprintf('Min drone-obstacle clear.: %.2f m at t = %.1f s\n\n', minDO, minDOt);

out.L = L; out.G = G; out.P = P; out.M = M; out.cfg = cfg; out.nR = nR;
end

%% ======================================================================
function cfg = lqr_defaultConfig()
cfg.nDrones = 3;  cfg.seed = 7;  cfg.tMax = 300;  cfg.dt = 0.005;
cfg.recDt = 0.025;  cfg.replanDt = 0.25;
cfg.g = 9.81;  cfg.rho = 1.225;
cfg.windMean = 3.5;  cfg.windVar = 1.5;  cfg.windVarFreq = 0.07;
cfg.windDir = 0.5;   cfg.windDirVar = 0.6;  cfg.windDirFreq = 0.045;  cfg.windDirPhase = 1;
cfg.gustTau = 2.0;   cfg.gustSigma = 1.0;   cfg.gustVertFrac = 0.3;
cfg.mFrame = 1.5;  cfg.mPayload = 0.5;  cfg.motorMassFrac = 0.08;
cfg.armLength = 0.25;  cfg.coreRadius = 0.12;  cfg.rotorRadius = 0.17;
cfg.kf = 1e-5;  cfg.km = 1.6e-7;  cfg.twr = 2.5;  cfg.wMinFrac = 0.15;  cfg.tauM = 0.03;
cfg.dragCoef = 1.1;  cfg.frontalArea = 0.15;  cfg.spread = 0.05;
cfg.pkgSize = [0.22 0.22 0.14];
cfg.maxTiltDeg = 25;  cfg.yawRateDeg = 60;
cfg.tiltUse = 0.9;  cfg.thrustUse = 0.4;  cfg.descentUse = 0.5;
cfg.vMaxH = 6;  cfg.vUp = 3;  cfg.vDown = 2;  cfg.aMaxH = 3;  cfg.aMaxV = 2;
cfg.kRef = 0.7;  cfg.kRefZ = 0.9;  cfg.maxLead = 6;  cfg.look = 12;
cfg.touchV0 = 0.5;  cfg.touchGain = 0.6;  cfg.yawMinSpeed = 1.5;
cfg.lqr.posTolXY = 1.5;  cfg.lqr.posTolZ = 1.0;  cfg.lqr.velTol = 2.0;
cfg.lqr.intTol = 10;     cfg.lqr.intLimFrac = 0.5;
cfg.lqr.angTolDeg = 5;   cfg.lqr.rateTol = 1.5;   cfg.lqr.alphaFrac = 0.08;
cfg.lqr.yawTolDeg = 15;  cfg.lqr.yawRateTol = 1.0; cfg.lqr.alphaFracYaw = 0.3;
cfg.routeLength = 210;  cfg.homeSpacing = 10;  cfg.fanDeg = 12;
cfg.cruiseAlt = 10;  cfg.layerGap = 4;  cfg.deliveryAlt = 3;
cfg.wp1Frac = 0.33;  cfg.wp1Lat = 0.09;  cfg.wp2Frac = 0.66;  cfg.wp2Lat = -0.13;
cfg.wpRadius = 4;  cfg.custRadius = 1.5;  cfg.custSpeed = 0.7;  cfg.custZTol = 0.6;
cfg.altTol = 0.5;  cfg.homeRadius = 3;  cfg.landZ = 0.12;
cfg.releaseDelay = 2.0;  cfg.deliveryDwell = 4.5;  cfg.releaseDrop = 0.4;  cfg.pkgDrag = 0.3;
cfg.radarRange = 40;  cfg.Th = 8;  cfg.dtPred = 0.1;  cfg.zMin = 2;  cfg.zMax = 45;
cfg.margin = 1.5;  cfg.speedGain = 0.15;  cfg.buf = 0.5;  cfg.releaseTime = 1.5;
cfg.cost = struct('wLat',1.8,'wAlong',1.2,'wUp',1.0,'wDown',2.5,'emerg',6,'hyst',0.7);
cfg.cand = struct('reserve',0.7,'minLat',1.0,'mixFrac',0.75,'fwdAlong',12,'fwdSpeed',1.35, ...
                  'backAlong',-8,'emergUp',3,'emergLat',0.5,'emergAlong',-4);
cfg.nBuildings = 8;  cfg.nTrees = 40;  cfg.bldWidth = [8 20];  cfg.bldHeight = [6 26];
cfg.treeHeight = [4 8];  cfg.routeClear = 14;  cfg.bldMargin = 3;  cfg.bldZMargin = 2;  cfg.worldPad = 40;
cfg.nObstacles = 6;  cfg.obsTypes = {'bird','kite','drone','balloon'};
cfg.obsAlt = [4 16];  cfg.obsLateral = 40;
cfg.obsSpec.bird    = struct('r',0.5,'speed',4.0,'turn',0.015,'vz',0.3);
cfg.obsSpec.kite    = struct('r',1.0,'speed',0.7,'turn',0.0,  'vz',0.0);
cfg.obsSpec.drone   = struct('r',0.8,'speed',3.4,'turn',0.01, 'vz',0.0);
cfg.obsSpec.balloon = struct('r',1.3,'speed',0.0,'turn',0.0,  'vz',0.0);
end

function P = lqr_droneParams(i, N, cfg) %#ok<INUSD>
rs = RandStream('mt19937ar','Seed',cfg.seed*1000 + i);
sp = @() 1 + cfg.spread*(2*rand(rs) - 1);
P.id = i;  P.g = cfg.g;
P.mFrame = cfg.mFrame*sp();  P.mPayload = cfg.mPayload*sp();
P.mMotor = cfg.motorMassFrac*P.mFrame;
P.L = cfg.armLength*sp();  P.d = P.L/sqrt(2);  P.rCore = cfg.coreRadius*sp();
P.rBody = P.L + cfg.rotorRadius;  P.rDrone = P.rBody;
mc = P.mFrame - 4*P.mMotor;  Jc = 0.4*mc*P.rCore^2;
P.J = diag([Jc + 4*P.mMotor*P.d^2, Jc + 4*P.mMotor*P.d^2, Jc + 4*P.mMotor*P.L^2]);
P.kf = cfg.kf*sp();  P.km = cfg.km*sp();  P.c = P.km/P.kf;
P.mDesign = P.mFrame + P.mPayload;
P.wHover = sqrt(P.mDesign*P.g/(4*P.kf));
P.twr = cfg.twr*sp();
P.wMax = sqrt(P.twr)*P.wHover;  P.wMin = cfg.wMinFrac*P.wHover;
P.Tmax = 4*P.kf*P.wMax^2;  P.tauM = cfg.tauM*sp();
d = P.d;  c = P.c;
P.Amix = [1 1 1 1; -d d d -d; -d -d d d; -c c -c c];
P.invA = inv(P.Amix);
P.mp = d*[1 -1; 1 1; -1 1; -1 -1];
P.cd = 0.5*cfg.rho*cfg.dragCoef*cfg.frontalArea;
P.maxTilt = deg2rad(cfg.maxTiltDeg);  P.yawRate = deg2rad(cfg.yawRateDeg);
P.accH = cfg.tiltUse*P.g*tan(P.maxTilt);
P.accUp = cfg.thrustUse*(P.twr - 1)*P.g;
P.accDown = -cfg.descentUse*P.g;
f = {'vMaxH','vUp','vDown','aMaxH','aMaxV','kRef','kRefZ','maxLead','look','touchV0','touchGain', ...
     'yawMinSpeed','radarRange','Th','dtPred','zMin','zMax','margin','speedGain','buf','releaseTime', ...
     'replanDt','wpRadius','custRadius','custSpeed','custZTol','altTol','homeRadius','landZ', ...
     'releaseDelay','deliveryDwell','releaseDrop','pkgDrag','cost','cand','bldMargin','bldZMargin','dt'};
for q = 1:numel(f), P.(f{q}) = cfg.(f{q}); end
Fh = P.mDesign*P.g/4;  Fmax = P.kf*P.wMax^2;  Fmin = P.kf*P.wMin^2;
dF = min(Fmax - Fh, Fh - Fmin);
P.tauMax = [4*P.d*dF; 4*P.d*dF; 4*P.c*dF];
P.alphaMax = P.tauMax./diag(P.J);
q = cfg.lqr;
aXY = P.accH;  aZ = min(P.accUp, -P.accDown);
kxy = lqrPos(q.posTolXY, q.velTol, q.intTol, aXY);
kz  = lqrPos(q.posTolZ,  q.velTol, q.intTol, aZ);
P.Kpos = [kxy; kxy; kz];
P.IntLim = q.intLimFrac*q.intTol*ones(3,1);
angTol  = [deg2rad(q.angTolDeg); deg2rad(q.angTolDeg); deg2rad(q.yawTolDeg)];
rateTol = [q.rateTol; q.rateTol; q.yawRateTol];
frac    = [q.alphaFrac; q.alphaFrac; q.alphaFracYaw];
P.Katt = zeros(3,2);
for a = 1:3
    Q = diag([1/angTol(a)^2, 1/rateTol(a)^2]);  R = 1/(frac(a)*P.alphaMax(a))^2;
    P.Katt(a,:) = lqrGain([0 1; 0 0], [0; 1], Q, R);
end
end

function K = lqrPos(posTol, velTol, intTol, aMax)
A = [0 1 0; 0 0 0; 1 0 0];  B = [0; 1; 0];
Q = diag(1./[posTol velTol intTol].^2);  R = 1/aMax^2;
K = lqrGain(A, B, Q, R);
end

function K = lqrGain(A, B, Q, R)
n = size(A,1);
H = [A, -B*(R\B'); -Q, -A'];
[U, T] = schur(H, 'real');
[U, ~] = ordschur(U, T, 'lhp');
U1 = U(1:n,1:n);  U2 = U(n+1:end,1:n);
Pm = U2/U1;  Pm = (Pm + Pm')/2;
K = R\(B'*Pm);
if any(real(eig(A - B*K)) >= 0), error('LQR design failed: closed loop not stable (check tolerances).'); end
end

function lqr_printParams(P)
rows = {
 'frame mass [kg]',            @(p) p.mFrame
 'payload mass [kg]',          @(p) p.mPayload
 'collision radius [m]',       @(p) p.rBody
 'max motor speed [rad/s]',    @(p) p.wMax
 'thrust/weight (loaded)',     @(p) p.twr
 'LQR pos XY  Kp',             @(p) p.Kpos(1,1)
 'LQR pos XY  Kv',             @(p) p.Kpos(1,2)
 'LQR pos XY  Ki',             @(p) p.Kpos(1,3)
 'LQR pos Z   Kp',             @(p) p.Kpos(3,1)
 'LQR pos Z   Kv',             @(p) p.Kpos(3,2)
 'LQR pos Z   Ki',             @(p) p.Kpos(3,3)
 'LQR roll/pitch K_angle',     @(p) p.Katt(1,1)
 'LQR roll/pitch K_rate',      @(p) p.Katt(1,2)
 'LQR yaw K_angle',            @(p) p.Katt(3,1)
 'LQR yaw K_rate',             @(p) p.Katt(3,2) };
fprintf('\n=== LQR drone parameters ===\n');
fprintf('%-30s', 'parameter');  for i = 1:numel(P), fprintf('%12s', sprintf('UAV%d', i)); end;  fprintf('\n');
for q = 1:size(rows,1)
    fprintf('%-30s', rows{q,1});
    for i = 1:numel(P), fprintf('%12.4g', rows{q,2}(P(i))); end
    fprintf('\n');
end
end

function M = lqr_makeMission(i, N, cfg)
slot = i - (N+1)/2;
home = [0; slot*cfg.homeSpacing; 0];
ang  = deg2rad(slot*cfg.fanDeg);
cruise = cfg.cruiseAlt + (i-1)*cfg.layerGap;
cust = [home(1:2) + cfg.routeLength*[cos(ang); sin(ang)]; cfg.deliveryAlt];
dxy = cust(1:2) - home(1:2);  Lr = norm(dxy);  u = dxy/Lr;  perp = [-u(2); u(1)];
W1 = [home(1:2) + cfg.wp1Frac*Lr*u + cfg.wp1Lat*Lr*perp; cruise];
W2 = [home(1:2) + cfg.wp2Frac*Lr*u + cfg.wp2Lat*Lr*perp; cruise];
hc = [home(1:2); cruise];  cc = [cust(1:2); cruise];
names = {'TAKEOFF','WAYPOINT 1','WAYPOINT 2','CUSTOMER APPROACH','DELIVERY', ...
         'RETURN HOME (CLIMB)','RETURN HOME','LAND'};
S = {home, hc, W1, W2, cust, cust, cc, hc};
T = {hc,   W1, W2, cust, cust, cc,  hc, home};
vertOK = [false true true true true true true false];
for k = 1:numel(names)
    segs(k).name = names{k}; segs(k).S = S{k}; segs(k).T = T{k}; segs(k).vertOK = vertOK(k); %#ok<AGROW>
end
M.home = home; M.cust = cust; M.cruise = cruise; M.segs = segs;
M.poly = [home(1:2), W1(1:2), W2(1:2), cust(1:2), home(1:2)];
M.W3 = [segs(1).S, segs(1).T, segs(2).T, segs(3).T, segs(4).T, segs(6).T, segs(7).T, segs(8).T];
end

function world = lqr_makeWorld(cfg, M)
pts = [M.poly];
world.lo = min(pts,[],2) - cfg.worldPad;  world.hi = max(pts,[],2) + cfg.worldPad;
end

function dmin = lqr_distToRoutes(q, M)
dmin = inf;
for m = 1:numel(M)
    W = M(m).poly;
    for i = 1:size(W,2)-1
        a = W(:,i); b = W(:,i+1); ab = b - a;
        s = min(max(dot(q-a, ab)/dot(ab,ab), 0), 1);
        dmin = min(dmin, norm(q - (a + s*ab)));
    end
end
end

function [bld, trees] = lqr_makeScene(cfg, M, world)
span = world.hi - world.lo;
bld = zeros(0,5);  tries = 0;
while size(bld,1) < cfg.nBuildings && tries < 5000
    tries = tries + 1;
    w = cfg.bldWidth(1) + diff(cfg.bldWidth)*rand;  dp = cfg.bldWidth(1) + diff(cfg.bldWidth)*rand;
    h = cfg.bldHeight(1) + diff(cfg.bldHeight)*rand;
    cxy = world.lo + span.*rand(2,1);
    if lqr_distToRoutes(cxy, M) < hypot(w,dp)/2 + cfg.routeClear, continue; end
    bld(end+1,:) = [cxy' w dp h]; %#ok<AGROW>
end
trees = zeros(0,3);  tries = 0;
while size(trees,1) < cfg.nTrees && tries < 5000
    tries = tries + 1;
    cxy = world.lo + span.*rand(2,1);
    if lqr_distToRoutes(cxy, M) < cfg.routeClear, continue; end
    if lqr_inBuilding([cxy;0], bld, cfg.bldMargin, cfg.bldZMargin), continue; end
    trees(end+1,:) = [cxy' cfg.treeHeight(1) + diff(cfg.treeHeight)*rand]; %#ok<AGROW>
end
end

function obs = lqr_makeObstacles(cfg, M, world)
nO = cfg.nObstacles;  nT = numel(cfg.obsTypes);
for n = 1:nO
    type = cfg.obsTypes{mod(n-1, nT) + 1};  sp = cfg.obsSpec.(type);
    m = M(randi(numel(M)));
    dxy = m.cust(1:2) - m.home(1:2);  u = dxy/norm(dxy);  perp = [-u(2); u(1)];
    pxy = m.home(1:2) + rand*dxy + perp*(rand-0.5)*cfg.obsLateral;
    z = cfg.obsAlt(1) + diff(cfg.obsAlt)*rand;
    hdg = 2*pi*rand;  vel = sp.speed*[cos(hdg); sin(hdg); 0];
    lo = [world.lo; cfg.zMin + 0.5];  hi = [world.hi; cfg.zMax - 5];
    obs(n) = lqr_mk(type, [pxy; z], vel, sp.r, lo, hi, sp.turn, sp.vz, type); %#ok<AGROW>
end
end

function o = lqr_mk(type, pos, vel, r, lo, hi, turn, vzAmp, label)
o.type = type; o.label = label; o.pos = pos(:); o.vel = vel(:); o.v0 = vel(:);
o.speed = norm(vel(1:2)); o.r = r; o.lo = lo(:); o.hi = hi(:);
o.turn = turn; o.vzAmp = vzAmp; o.phase = 2*pi*rand;
end

function lst = lqr_swarmObstacles(D, P, i)
lst = [];
for j = 1:numel(D)
    if j == i, continue; end
    o = lqr_mk('swarm', D(j).p, D(j).v, P(j).rBody, zeros(3,1), zeros(3,1), 0, 0, sprintf('UAV%d', j));
    lst = [lst, o]; %#ok<AGROW>
end
end

function obs = lqr_stepObstacles(obs, dt, t)
for i = 1:numel(obs)
    o = obs(i);
    switch o.type
        case 'bird'
            ang = o.turn*randn*sqrt(dt);
            cs = cos(ang); sn = sin(ang);
            o.vel(1:2) = [cs -sn; sn cs]*o.vel(1:2);
            o.vel(1:2) = o.vel(1:2)/max(norm(o.vel(1:2)),1e-6)*o.speed;
            o.vel(3) = o.vzAmp*cos(0.9*t + o.phase);
        case 'kite'
            o.vel = o.v0 + 0.4*[sin(0.5*t+o.phase); cos(0.4*t); 0.4*sin(0.7*t)];
        case 'drone'
            ang = o.turn*randn*sqrt(dt);
            cs = cos(ang); sn = sin(ang);
            o.vel(1:2) = [cs -sn; sn cs]*o.vel(1:2);
        otherwise
            o.vel = zeros(3,1);
    end
    o.pos = o.pos + o.vel*dt;
    for ax = 1:3
        if o.pos(ax) < o.lo(ax), o.pos(ax) = o.lo(ax); o.vel(ax) = abs(o.vel(ax)); o.v0(ax) = abs(o.v0(ax));
        elseif o.pos(ax) > o.hi(ax), o.pos(ax) = o.hi(ax); o.vel(ax) = -abs(o.vel(ax)); o.v0(ax) = -abs(o.v0(ax)); end
    end
    obs(i) = o;
end
end

function [wind, gust] = lqr_windModel(t, gust, cfg, dt)
Wm = cfg.windMean + cfg.windVar*sin(cfg.windVarFreq*t);
th = cfg.windDir + cfg.windDirVar*sin(cfg.windDirFreq*t + cfg.windDirPhase);
a  = exp(-dt/cfg.gustTau);
gust = a*gust + sqrt(1-a^2)*cfg.gustSigma*[randn; randn; cfg.gustVertFrac*randn];
wind = Wm*[cos(th); sin(th); 0] + gust;
end

function d = lqr_initDrone(P, M, nO, zeroOff)
d.p = M.home; d.v = zeros(3,1); d.ang = zeros(3,1); d.om = zeros(3,1);
d.wm = P.wMin*ones(4,1);
d.payload = P.mPayload; d.released = false; d.pkgP = M.home; d.pkgV = zeros(3,1);
d.pref = M.home; d.vref = zeros(3,1); d.integ = zeros(3,1);
d.psiCmd = 0; d.gust = zeros(3,1); d.wind = zeros(3,1);
d.segs = M.segs; d.k = 1; d.tSeg = 0; d.landed = false;
d.A = struct('lastKey','','active',false,'off',zeroOff,'clearTimer',0,'det',[], ...
             'seen',false(1,nO),'threat',false(1,nO));
d.carrot = M.home; d.a_cmd = zeros(3,1); d.Ttot = 0;
d.nAvoid = 0; d.lastName = 'NONE';
end

function d = lqr_replan(d, t, obs, P, bld, zeroOff)
d.A = lqr_avoidanceUpdate(d.A, t, d.p, d.v, d.segs(d.k), obs, P, bld, zeroOff);
if d.A.active && ~strcmp(d.A.off.name, d.lastName), d.nAvoid = d.nAvoid + 1; end
if d.A.active, d.lastName = d.A.off.name; else, d.lastName = 'NONE'; end
end

function d = lqr_stepDrone(d, P, t, wind)
dt = P.dt;  tag = sprintf('UAV%d', P.id);
sg = d.segs(d.k);
mTot = P.mFrame + d.payload;

done = false;
switch d.k
    case 1, done = d.p(3) > sg.T(3) - P.altTol;
    case {2,3}, done = norm(d.p(1:2) - sg.T(1:2)) < P.wpRadius;
    case 4, done = norm(d.p(1:2) - sg.T(1:2)) < P.custRadius && d.p(3) < sg.T(3) + P.custZTol && norm(d.v) < P.custSpeed;
    case 5
        if ~d.released && (t - d.tSeg) > P.releaseDelay
            fprintf('[t=%6.2f s] %s DELIVERY: package released (mass %.2f -> %.2f kg)\n', t, tag, mTot, P.mFrame);
            d.released = true; d.payload = 0; mTot = P.mFrame;
            d.pkgP = d.p - [0;0;P.releaseDrop]; d.pkgV = d.v;
        end
        done = (t - d.tSeg) > P.deliveryDwell;
    case 6, done = d.p(3) > sg.T(3) - P.altTol;
    case 7, done = norm(d.p(1:2) - sg.T(1:2)) < P.homeRadius;
    case 8, done = d.p(3) < P.landZ;
end
if done
    if d.k == 8
        d.landed = true; d.v = zeros(3,1); d.om = zeros(3,1); d.Ttot = 0;
        fprintf('[t=%6.2f s] %s MISSION COMPLETE: landed\n', t, tag);
        return
    end
    d.k = d.k + 1; d.tSeg = t; sg = d.segs(d.k);
    fprintf('[t=%6.2f s] %s MISSION: %s\n', t, tag, sg.name);
end

off = d.A.off;
d.carrot = lqr_carrotTarget(d.p, sg, off, P);
[d.pref, d.vref] = lqr_refStep(d.pref, d.vref, d.carrot, off.speedScale, P, dt);
lead = d.pref - d.p;  nl = norm(lead);
if nl > P.maxLead, d.pref = d.p + lead*P.maxLead/nl; end

% ---- position LQR
ep = d.p - d.pref;  evl = d.v - d.vref;
d.integ = max(min(d.integ + ep*dt, P.IntLim), -P.IntLim);
a_cmd = -(P.Kpos(:,1).*ep + P.Kpos(:,2).*evl + P.Kpos(:,3).*d.integ);
ah = norm(a_cmd(1:2));
if ah > P.accH, a_cmd(1:2) = a_cmd(1:2)*P.accH/ah; end
a_cmd(3) = min(max(a_cmd(3), P.accDown), P.accUp);
d.a_cmd = a_cmd;

Fw = mTot*(a_cmd + [0;0;P.g]);
R  = eul2R(d.ang);
Tcmd = min(max(dot(Fw, R(:,3)), 0), P.Tmax);

if norm(d.vref(1:2)) > P.yawMinSpeed
    dpsi = wrapPi(atan2(d.vref(2), d.vref(1)) - d.psiCmd);
    d.psiCmd = d.psiCmd + max(min(dpsi, P.yawRate*dt), -P.yawRate*dt);
end
an = a_cmd + [0;0;P.g];
axb =  cos(d.psiCmd)*an(1) + sin(d.psiCmd)*an(2);
ayb = -sin(d.psiCmd)*an(1) + cos(d.psiCmd)*an(2);
th_d  = max(min(atan2(axb, an(3)),  P.maxTilt), -P.maxTilt);
phi_d = max(min(atan2(-ayb, an(3)), P.maxTilt), -P.maxTilt);
eang = [phi_d; th_d; d.psiCmd] - d.ang;  eang(3) = wrapPi(eang(3));

% ---- attitude LQR
alpha = P.Katt(:,1).*eang - P.Katt(:,2).*d.om;
tau   = P.J*alpha + cross(d.om, P.J*d.om);

Fm   = P.invA*[Tcmd; tau];
Fm   = min(max(Fm, P.kf*P.wMin^2), P.kf*P.wMax^2);
wcmd = sqrt(Fm/P.kf);
d.wm = d.wm + (wcmd - d.wm)/P.tauM*dt;
Fa   = P.kf*d.wm.^2;
Ttot = sum(Fa);
tauA = P.Amix(2:4,:)*Fa;

omdot = P.J \ (tauA - cross(d.om, P.J*d.om));
d.om = d.om + omdot*dt;
sp = sin(d.ang(1)); cp = cos(d.ang(1)); tt = tan(d.ang(2)); ct = cos(d.ang(2));
angdot = [d.om(1) + (d.om(2)*sp + d.om(3)*cp)*tt;
          d.om(2)*cp - d.om(3)*sp;
          (d.om(2)*sp + d.om(3)*cp)/ct];
d.ang = d.ang + angdot*dt;  d.ang(3) = wrapPi(d.ang(3));

Fth = R*[0;0;Ttot];
Fg  = [0;0;-mTot*P.g];
Fdr = -P.cd*norm(d.v)*d.v;
Fwd =  P.cd*norm(wind)*wind;
acc = (Fth + Fg + Fdr + Fwd)/mTot;
d.v = d.v + acc*dt;  d.p = d.p + d.v*dt;
if d.p(3) <= 0
    d.p(3) = 0; if d.v(3) < 0, d.v(3) = 0; end
    if Ttot < mTot*P.g
        d.v(1:2) = 0; d.om = zeros(3,1); d.ang(1:2) = 0;
    end
end
d.Ttot = Ttot;
if d.released
    d.pkgV = d.pkgV + ([0;0;-P.g] - P.pkgDrag*norm(d.pkgV)*d.pkgV)*dt;
    d.pkgP = d.pkgP + d.pkgV*dt;
    if d.pkgP(3) < 0.1, d.pkgP(3) = 0.1; d.pkgV = zeros(3,1); end
end
end

function det = lqr_radarScan(p, v, obs, P)
det = [];
for i = 1:numel(obs)
    rel = obs(i).pos - p;  dist = norm(rel);
    if dist > P.radarRange, continue; end
    relv = obs(i).vel - v;
    Rs = obs(i).r + P.rDrone + P.margin + P.speedGain*norm(relv);
    s.id = i; s.type = obs(i).label; s.dist = dist; s.rel = rel; s.relv = relv;
    s.height = obs(i).pos(3);
    s.heading = atan2d(obs(i).vel(2), obs(i).vel(1));
    s.Rs = Rs;
    s.ttc = lqr_timeToCollision(rel, relv, Rs);
    if isempty(det), det = s; else, det(end+1) = s; end %#ok<AGROW>
end
end

function ttc = lqr_timeToCollision(rel, relv, Rs)
a = dot(relv,relv); b = 2*dot(rel,relv); cc = dot(rel,rel) - Rs^2;
if cc < 0, ttc = 0; return; end
if a < 1e-9, ttc = inf; return; end
disc = b^2 - 4*a*cc;
if disc < 0, ttc = inf; return; end
tt = (-b - sqrt(disc))/(2*a);
if tt < 0, ttc = inf; else, ttc = tt; end
end

function A = lqr_avoidanceUpdate(A, t, p, v, sg, obs, P, bld, zeroOff)
tag = sprintf('UAV%d', P.id);
det = lqr_radarScan(p, v, obs, P);
A.det = det;
nO = numel(obs);
inRange = false(1,nO);
if ~isempty(det), inRange([det.id]) = true; end
for i = find(inRange & ~A.seen)
    s = det([det.id] == i);
    fprintf('[t=%6.2f s] %s RADAR  : %-7s #%d dist=%5.1f m TTC=%s safety=%.1f m\n', ...
        t, tag, s.type, i, s.dist, ttcStr(s.ttc), s.Rs);
end
A.seen = inRange;
A.threat = false(1,nO);

if isempty(det)
    A = lqr_tryRelease(A, t, P, zeroOff, true);
    return
end

idx = [det.id];  Rs = [det.Rs];
ro0 = lqr_rolloutRisk(p, v, sg, zeroOff, obs, idx, Rs, P, bld);
if ro0.margin >= P.buf
    A = lqr_tryRelease(A, t, P, zeroOff, all([det.dist] > 1.5*Rs));
    return
end

A.clearTimer = 0;
A.threat(idx(ro0.per < P.buf)) = true;
needNew = true;
if A.active
    roc = lqr_rolloutRisk(p, v, sg, A.off, obs, idx, Rs, P, bld);
    if roc.margin >= P.buf, needNew = false; end
end
if ~needNew, return; end

cands = lqr_buildCandidates(p, sg, ro0, Rs, P);
c0 = lqr_carrotTarget(p, sg, zeroOff, P);  z0 = c0(3);
bestCost = inf; best = []; bestM = -inf; fallback = [];
for q = 1:numel(cands)
    cand = cands{q};
    roq = lqr_rolloutRisk(p, v, sg, cand, obs, idx, Rs, P, bld);
    cost = P.cost.wLat*abs(cand.lat) + P.cost.wAlong*abs(cand.along);
    if ~isnan(cand.z)
        if cand.z >= z0, cost = cost + P.cost.wUp*(cand.z - z0); else, cost = cost + P.cost.wDown*(z0 - cand.z); end
    end
    if strncmp(cand.name,'EMERG',5), cost = cost + P.cost.emerg; end
    if A.active && strcmp(cand.name, A.off.name), cost = P.cost.hyst*cost; end
    if roq.margin >= P.buf && cost < bestCost
        bestCost = cost; best = cand; best.margin = roq.margin;
    end
    if roq.margin > bestM, bestM = roq.margin; fallback = cand; fallback.margin = roq.margin; end
end
emergency = false;
if isempty(best), best = fallback; emergency = true; end

A.off = rmfield(best,'margin'); A.active = true;
if isnan(best.z), zs = 'mission alt'; else, zs = sprintf('%.1f m', best.z); end
key = sprintf('%s|%.0f|%.0f', best.name, best.lat, ternary(isnan(best.z), -1, best.z));
if strcmp(key, A.lastKey), return; end
A.lastKey = key;
fprintf('[t=%6.2f s] %s DECIDE : %s%s -> target Z = %s, lateral = %+.1f m, along = %+.1f m (predicted clearance %.1f m)\n', ...
    t, tag, best.name, ternary(emergency,' (best effort)',''), zs, best.lat, best.along, best.margin);
end

function A = lqr_tryRelease(A, t, P, zeroOff, farEnough)
if ~A.active, return; end
if farEnough, A.clearTimer = A.clearTimer + P.replanDt; else, A.clearTimer = 0; end
if A.clearTimer >= P.releaseTime
    A.active = false; A.off = zeroOff; A.clearTimer = 0; A.lastKey = '';
    fprintf('[t=%6.2f s] UAV%d AVOID  : obstacle cleared -> avoidance OFF\n', t, P.id);
end
end

function cands = lqr_buildCandidates(p, sg, ro, Rs, P)
cc = P.cand;
thr = find(ro.per < P.buf);
S = sg.S; T = sg.T; dxy = T(1:2) - S(1:2); Ls = norm(dxy);
if Ls < 1, u = [1;0]; ref0 = T(1:2); else, u = dxy/Ls; ref0 = S(1:2); end
perp = [-u(2); u(1)];
zo = ro.tpos(3,thr);  Rt = Rs(thr);
lat_o = perp'*(ro.tpos(1:2,thr) - ref0);
extra = P.buf + cc.reserve;
zc = max(zo + Rt + extra);
zd = min(zo - Rt - extra);
latL = max(max(lat_o + Rt + extra),  cc.minLat);
latR = min(min(lat_o - Rt - extra), -cc.minLat);
mkc = @(n,z,l,a,s) struct('name',n,'z',z,'lat',l,'along',a,'speedScale',s);
cands = {};
canUp = sg.vertOK && zc <= P.zMax;
canDn = sg.vertOK && zd >= P.zMin;
if canUp, cands{end+1} = mkc('CLIMB', zc, 0, 0, 1); end
if canDn, cands{end+1} = mkc('DESCEND', zd, 0, 0, 1); end
cands{end+1} = mkc('LEFT',  NaN, latL, 0, 1);
cands{end+1} = mkc('RIGHT', NaN, latR, 0, 1);
if Ls >= 1
    cands{end+1} = mkc('FORWARD',  NaN, 0, cc.fwdAlong, cc.fwdSpeed);
    cands{end+1} = mkc('BACKWARD', NaN, 0, cc.backAlong, 1);
end
if canUp
    zm = p(3) + cc.mixFrac*(zc - p(3));
    cands{end+1} = mkc('CLIMB+LEFT',  zm, cc.mixFrac*latL, 0, 1);
    cands{end+1} = mkc('CLIMB+RIGHT', zm, cc.mixFrac*latR, 0, 1);
    cands{end+1} = mkc('EMERGENCY 3D', min(zc + cc.emergUp, P.zMax), cc.emergLat*latL, cc.emergAlong, 1);
end
if canDn
    zm = p(3) + cc.mixFrac*(zd - p(3));
    cands{end+1} = mkc('DESCEND+LEFT',  zm, cc.mixFrac*latL, 0, 1);
    cands{end+1} = mkc('DESCEND+RIGHT', zm, cc.mixFrac*latR, 0, 1);
end
end

function ro = lqr_rolloutRisk(p, v, sg, off, obs, idx, Rs, P, bld)
N = round(P.Th/P.dtPred); nO = numel(idx);
ro.per = inf(1,nO); ro.tpos = zeros(3,nO); ro.tt = zeros(1,nO); ro.hit = false;
pq = p; vq = v;
for kk = 1:N
    tq = kk*P.dtPred;
    cq = lqr_carrotTarget(pq, sg, off, P);
    [pq, vq] = lqr_refStep(pq, vq, cq, off.speedScale, P, P.dtPred);
    for j = 1:nO
        o = obs(idx(j));
        po = o.pos + o.vel*tq;
        m = norm(pq - po) - Rs(j);
        if m < ro.per(j), ro.per(j) = m; ro.tpos(:,j) = po; ro.tt(j) = tq; end
    end
    if lqr_inBuilding(pq, bld, P.bldMargin, P.bldZMargin), ro.hit = true; end
end
ro.margin = min(ro.per);
if ro.hit, ro.margin = min(ro.margin, -(P.buf + P.cand.reserve + P.margin)); end
end

function tf = lqr_inBuilding(q, bld, m, zm)
tf = false;
for b = 1:size(bld,1)
    if abs(q(1)-bld(b,1)) < bld(b,3)/2 + m && abs(q(2)-bld(b,2)) < bld(b,4)/2 + m && q(3) < bld(b,5) + zm
        tf = true; return;
    end
end
end

function c = lqr_carrotTarget(p, sg, off, P)
S = sg.S; T = sg.T; dxy = T(1:2) - S(1:2); Ls = norm(dxy);
if Ls < 1
    perp = [0;1];
    cxy = T(1:2) + perp*off.lat;
    cz  = T(3);
else
    u = dxy/Ls; perp = [-u(2); u(1)];
    s  = min(max(dot(p(1:2)-S(1:2), u), 0), Ls);
    st = min(s + P.look, Ls) + off.along;
    st = min(max(st, -P.cand.minLat*5), Ls);
    cxy = S(1:2) + u*st + perp*off.lat;
    cz  = S(3) + (T(3)-S(3))*min(max(st/Ls,0),1);
end
if ~isnan(off.z), cz = off.z; end
c = [cxy; cz];
end

function [pr, vr] = lqr_refStep(pr, vr, carrot, vScale, P, dt)
e = carrot - pr; exy = e(1:2); dxy = norm(exy);
if dxy > 1e-6, vh = exy/dxy*min(P.kRef*dxy, P.vMaxH*vScale); else, vh = [0;0]; end
vdn = min(P.vDown, P.touchV0 + P.touchGain*pr(3));
vz  = min(max(P.kRefZ*e(3), -vdn), P.vUp);
dv  = [vh - vr(1:2); vz - vr(3)];
lim = P.aMaxH*dt; dvh = norm(dv(1:2));
if dvh > lim, dv(1:2) = dv(1:2)/dvh*lim; end
dv(3) = min(max(dv(3), -P.aMaxV*dt), P.aMaxV*dt);
vr = vr + dv;  pr = pr + vr*dt;
end