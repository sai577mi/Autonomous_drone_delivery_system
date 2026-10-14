function out = drone_dynamic_avoidance(varargin)
%DRONE_DYNAMIC_AVOIDANCE  Autonomous delivery drone with dynamic 3D obstacle avoidance.
%
%   Run:   drone_dynamic_avoidance
%          drone_dynamic_avoidance('cam','overview')      % fixed camera
%          drone_dynamic_avoidance('animate',false)       % numbers + graphs only
%          drone_dynamic_avoidance('seed',3,'animStride',2)
%
%   Signal chain (every control step, dt = 5 ms):
%     Radar -> Obstacle prediction -> Avoidance decision -> New 3D target
%     -> Reference generator (rate/accel limits) -> Position PID (X,Y,Z)
%     -> Required force F = m(a+g) -> Thrust + desired roll/pitch
%     -> Attitude PD + J*w_dot + w x J*w = tau -> Motor mixer (4 motors)
%     -> Motor lag, F_i = kf*w_i^2 -> Rigid-body dynamics (gravity, drag, wind, payload)
%
%   Mission: HOME -> TAKEOFF -> WP1 -> WP2 -> CUSTOMER -> DELIVERY -> RETURN -> LAND
%   Avoidance is active in every phase.
%
%   At the end of the run TWO figures are drawn:
%     1) individual graphs: position X/Y/Z, velocity, acceleration,
%        roll/pitch/yaw angles, roll/pitch/yaw rates (each on its own plot)
%     2) combined graphs: position, velocity, acceleration, attitude, angular
%        rates (all three axes overlaid) + 3D path coloured by mission phase
%
%   Tested with: MATLAB R2024b (no toolboxes needed).

out = [];
opts = struct('animate',true,'seed',7,'tMax',240,'cam','follow','animStride',3);
for ii = 1:2:numel(varargin), opts.(varargin{ii}) = varargin{ii+1}; end
try, rng(opts.seed); catch, randn('state',opts.seed); rand('state',opts.seed); end %#ok<NASGU>
close all; clc;

P = defaultParams(opts);
[bld, trees] = makeScene();
segs = makeMission();
obs  = makeObstacles();
nO   = numel(obs);
obs0 = obs;

zeroOff = struct('name','NONE','z',NaN,'lat',0,'along',0,'speedScale',1);
A = struct('lastKey','','active',false,'off',zeroOff,'clearTimer',0,'det',[], ...
           'seen',false(1,nO),'threat',false(1,nO));

%% ------------------------------------------------------------------ state
dt = P.dt;
d = P.d; c = P.c;
Amix = [1 1 1 1; -d d d -d; -d -d d d; -c c -c c];     % [T;tx;ty;tz] = Amix*F
invA = inv(Amix);

p = [0;0;0]; v = zeros(3,1); ang = zeros(3,1); om = zeros(3,1);
wm = P.wMin*ones(4,1);                 % motor speeds [rad/s]
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
Lg.mode = cell(1,nRec); Lg.obsP = zeros(3,nO,nRec); Lg.obsV = zeros(3,nO,nRec);
Lg.thr = false(nO,nRec); Lg.det = false(nO,nRec);
Lg.near = nan(1,nRec); Lg.ttc = nan(1,nRec); Lg.zObs = nan(1,nRec);
Lg.pkg = zeros(3,nRec); Lg.pkgOn = false(1,nRec); Lg.pay = zeros(1,nRec);
r = 0;

fprintf('\n=== Dynamic obstacle-avoidance delivery drone ===\n');
fprintf('Mission: %s\n\n', strjoin({segs.name}, ' -> '));
fprintf('[t=%6.2f s] MISSION: %s\n', 0, segs(1).name);

%% ------------------------------------------------------------------ main loop
for it = 1:nSteps
    t = (it-1)*dt;
    mTot = P.mFrame + payload;

    % ---- world: obstacles + wind --------------------------------------
    obs = stepObstacles(obs, dt, t);
    [wind, gust] = windModel(t, gust, P);

    % ---- radar / prediction / avoidance decision (4 Hz) -------------------
    if mod(it-1, nReplan) == 0
        A = avoidanceUpdate(A, t, p, v, sg, obs, P, bld, zeroOff);
        if A.active && ~strcmp(A.off.name, lastName), nAvoid = nAvoid + 1; end
        if A.active, lastName = A.off.name; else, lastName = 'NONE'; end
    end

    % ---- mission supervisor ---------------------------------------------
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

    % ---- new 3D target (carrot) + smooth reference --------------------------
    off = A.off;
    carrot = carrotTarget(p, sg, off, P);
    [pref, vref] = refStep(pref, vref, carrot, off.speedScale, P, dt);
    lead = pref - p; nl = norm(lead);
    if nl > P.maxLead, pref = p + lead*P.maxLead/nl; end

    % ---- position PID (separate X, Y, Z) -----------------------------------
    e = pref - p;
    integ = integ + e*dt;
    integ = max(min(integ, P.IntLim), -P.IntLim);          % anti-windup
    deriv = (e - eprev)/dt;  eprev = e;
    a_cmd = P.Kp.*e + P.Ki.*integ + P.Kd.*deriv;           % acceleration command
    ah = norm(a_cmd(1:2));
    if ah > P.accH, a_cmd(1:2) = a_cmd(1:2)*P.accH/ah; end % max horizontal accel
    a_cmd(3) = min(max(a_cmd(3), P.accDown), P.accUp);     % max vertical accel

    % ---- required force -> thrust projected on body Z ---------------------------
    Fw = mTot*(a_cmd + [0;0;P.g]);                         % F = m(a_command + g)
    R  = eul2R(ang);
    Tcmd = min(max(dot(Fw, R(:,3)), 0), P.Tmax);           % thrust along body Z

    % ---- desired attitude from force direction (yaw follows travel) --------------
    if norm(vref(1:2)) > 1.5
        dpsi = wrapPi(atan2(vref(2), vref(1)) - psiCmd);
        psiCmd = psiCmd + max(min(dpsi, P.yawRate*dt), -P.yawRate*dt);
    end
    an = a_cmd + [0;0;P.g];
    axb =  cos(psiCmd)*an(1) + sin(psiCmd)*an(2);
    ayb = -sin(psiCmd)*an(1) + cos(psiCmd)*an(2);
    th_d  = max(min(atan2(axb, an(3)),  P.maxTilt), -P.maxTilt);   % pitch (nose down = +)
    phi_d = max(min(atan2(-ayb, an(3)), P.maxTilt), -P.maxTilt);   % roll
    eang = [phi_d; th_d; psiCmd] - ang;  eang(3) = wrapPi(eang(3));

    % ---- attitude control: J*w_dot + w x J*w = tau ---------------------------------
    alpha = P.KattP.*eang - P.KattD.*om;                    % commanded angular accel
    tau   = P.J*alpha + cross(om, P.J*om);

    % ---- motor mixer -> individual motor speeds --------------------------------------
    Fm   = invA*[Tcmd; tau];
    Fm   = min(max(Fm, P.kf*P.wMin^2), P.kf*P.wMax^2);      % max / min motor speed
    wcmd = sqrt(Fm/P.kf);
    wm   = wm + (wcmd - wm)/P.tauM*dt;                       % motor lag
    Fa   = P.kf*wm.^2;                                       % Fi = kf*wi^2
    Ttot = sum(Fa);
    tauA = Amix(2:4,:)*Fa;

    % ---- attitude dynamics -----------------------------------------------------------
    omdot = P.J \ (tauA - cross(om, P.J*om));
    om = om + omdot*dt;
    sp = sin(ang(1)); cp = cos(ang(1)); tt = tan(ang(2)); ct = cos(ang(2));
    angdot = [om(1) + (om(2)*sp + om(3)*cp)*tt;
              om(2)*cp - om(3)*sp;
              (om(2)*sp + om(3)*cp)/ct];
    ang = ang + angdot*dt;  ang(3) = wrapPi(ang(3));

    % ---- translational dynamics: m a = F_thrust + F_gravity + F_drag + F_wind -------------
    Fth = R*[0;0;Ttot];
    Fg  = [0;0;-mTot*P.g];
    Fdr = -P.cd*norm(v)*v;
    Fwd =  P.cd*norm(wind)*wind;
    a   = (Fth + Fg + Fdr + Fwd)/mTot;
    v = v + a*dt;  p = p + v*dt;
    if p(3) <= 0                                             % ground contact
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

    % ---- separation bookkeeping --------------------------------------------------------
    for j = 1:nO
        sep = norm(obs(j).pos - p) - obs(j).r;
        if sep < minSep, minSep = sep; minSepT = t; end
    end

    % ---- logging -------------------------------------------------------------------------
    if mod(it-1, P.recStride) == 0
        r = r + 1;
        Lg.t(r) = t; Lg.p(:,r) = p; Lg.v(:,r) = v; Lg.ang(:,r) = ang; Lg.wm(:,r) = wm;
        Lg.T(r) = Ttot; Lg.W(r) = mTot*P.g; Lg.acmd(:,r) = a_cmd; Lg.pref(:,r) = pref;
        Lg.carrot(:,r) = carrot; Lg.wind(:,r) = wind; Lg.seg(r) = k; Lg.pay(r) = payload;
        c0 = carrotTarget(p, sg, zeroOff, P); Lg.zMis(r) = c0(3);
        if A.active, Lg.mode{r} = A.off.name; else, Lg.mode{r} = 'NONE'; end
        for j = 1:nO
            Lg.obsP(:,j,r) = obs(j).pos; Lg.obsV(:,j,r) = obs(j).vel;
        end
        Lg.thr(:,r) = A.threat(:);
        if ~isempty(A.det)
            Lg.det(:,r) = false; Lg.det([A.det.id], r) = true;
            [Lg.near(r), jn] = min([A.det.dist]);
            Lg.zObs(r) = A.det(jn).height;
            Lg.ttc(r) = min([A.det.ttc]);
        end
        Lg.pkg(:,r) = pkgP; Lg.pkgOn(r) = released;
    end
end

% final sample
r = r + 1;
Lg.t(r) = (it-1)*dt; Lg.p(:,r) = p; Lg.v(:,r) = v; Lg.ang(:,r) = ang; Lg.wm(:,r) = wm;
Lg.T(r) = Ttot; Lg.W(r) = mTot*P.g; Lg.acmd(:,r) = a_cmd; Lg.pref(:,r) = pref;
Lg.carrot(:,r) = carrot; Lg.wind(:,r) = wind; Lg.seg(r) = k; Lg.pay(r) = payload;
Lg.zMis(r) = 0; Lg.mode{r} = 'NONE';
for j = 1:nO, Lg.obsP(:,j,r) = obs(j).pos; Lg.obsV(:,j,r) = obs(j).vel; end
Lg.pkg(:,r) = pkgP; Lg.pkgOn(r) = released;
nR = r;

fprintf('\n=== Summary ===\n');
if landed, fprintf('Mission complete: landed at t = %.1f s\n', Lg.t(nR));
else,      fprintf('Mission NOT finished within %.0f s (last phase: %s)\n', P.tMax, segs(k).name); end
fprintf('Avoidance maneuvers : %d\n', nAvoid);
fprintf('Min separation      : %.2f m (surface to drone centre) at t = %.1f s\n', minSep, minSepT);
fprintf('Max altitude        : %.1f m   max speed %.1f m/s\n', max(Lg.p(3,1:nR)), max(sqrt(sum(Lg.v(:,1:nR).^2,1))));
fprintf('Thrust/weight range : %.2f .. %.2f\n', min(Lg.T(2:nR)./Lg.W(2:nR)), max(Lg.T(1:nR)./Lg.W(1:nR)));
fprintf('Motor speed range   : %.0f .. %.0f rad/s (limit %d)\n\n', min(min(Lg.wm(:,1:nR))), max(max(Lg.wm(:,1:nR))), P.wMax);

out = Lg; out.nR = nR;

% ---- individual + combined graphs --------------------------------------------------
plotSixDOF(Lg, nR, segs);

if opts.animate
    animateFlight(Lg, nR, P, bld, trees, segs, obs0, opts);
end
end

%% ======================================================================
%%  PARAMETERS / SCENE / MISSION
%% ======================================================================
function P = defaultParams(opts)
P.g = 9.81;  P.mFrame = 1.5;  P.mPayload = 0.5;
P.J = diag([0.030 0.030 0.055]);
P.L = 0.25;  P.d = P.L/sqrt(2);
P.kf = 1e-5; P.km = 1.6e-7; P.c = P.km/P.kf;
P.wMax = 1100; P.wMin = 100; P.tauM = 0.03;
P.Tmax = 4*P.kf*P.wMax^2;
P.cd = 0.10;
P.dt = 0.005; P.tMax = opts.tMax; P.recStride = 5; P.replanDt = 0.25;
% position PID  (X, Y, Z)
P.Kp = [2.2;2.2;4.5];  P.Ki = [0.4;0.4;0.8];  P.Kd = [2.6;2.6;3.6];  P.IntLim = [5;5;3];
% attitude PD  (roll, pitch, yaw)
P.KattP = [100;100;8]; P.KattD = [16;16;4];
% smoothness limits
P.vMaxH = 6;  P.aMaxH = 3;  P.vUp = 3;  P.vDown = 2;  P.aMaxV = 2;
P.kRef = 0.7; P.kRefZ = 0.9; P.maxLead = 6; P.look = 12;
P.accH = 4;   P.accUp = 6;  P.accDown = -5;
P.maxTilt = deg2rad(25);  P.yawRate = deg2rad(60);
% radar / avoidance
P.radarRange = 40; P.Th = 8; P.dtPred = 0.1; P.zMin = 2.0; P.zMax = 45;
P.rDrone = 0.5; P.margin = 1.5; P.speedGain = 0.15; P.buf = 0.5; P.releaseTime = 1.5;
% wind
P.windMean = 3.5; P.windVar = 1.5; P.windDir = 0.5; P.gustTau = 2.0; P.gustSigma = 1.0;
P.visScale = 3.5;
end

function segs = makeMission()
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

function [bld, trees] = makeScene()
% [cx cy width depth height]
bld = [ 40 -22 14 12 18;
        55  42 16 14 24;
        30  22 10  8 20;
       118  40 14 12 16;
       160 -30 20 14 26;
       225  30 12 10  7;
       -12 -14  8  8  6;
       190 -45 16 16 22;
        90  60 12 12 15];
hash = @(k) mod(sin(k*12.9898 + 78.233)*43758.5453, 1);   % deterministic pseudo-random
kk = 0;
trees = zeros(0,3);
while size(trees,1) < 55
    kk = kk + 3; x = -30 + 280*hash(kk); y = -80 + 180*hash(kk+1);
    if abs(y) < 7 || abs(x-105) < 7, continue; end                    % roads
    if hypot(x-210, y-20) < 25 || hypot(x, y) < 12, continue; end       % customer / home
    if distPointToRoute([x;y]) < 14, continue; end                      % flight corridor
    if inBuilding([x;y;0], bld), continue; end
    trees(end+1,:) = [x, y, 4 + 4*hash(kk+2)]; %#ok<AGROW>
end
end

function dmin = distPointToRoute(q)
W = [0 0; 70 25; 140 -15; 210 20; 0 0]';
dmin = inf;
for i = 1:size(W,2)-1
    a = W(:,i); b = W(:,i+1); ab = b - a;
    s = min(max(dot(q-a, ab)/dot(ab,ab), 0), 1);
    dmin = min(dmin, norm(q - (a + s*ab)));
end
end

function obs = makeObstacles()
% mk(type, pos, vel, radius, boxLo, boxHi, turnNoise, vzAmp)
obs(1) = mk('bird',  [34;60;11.5], [0;-4;0],    0.5, [-20 -60 6], [260 140 18], 0.015, 0.3);
obs(2) = mk('kite',  [105;6;12.0], [0.6;0.3;0], 1.0, [ 90  -8 10], [125  18 15], 0.0,   0.0);
obs(3) = mk('drone', [283;56.5;7], [-3;-1.5;0], 0.8, [-20 -90  3], [300  90 25], 0.01,  0.0);
obs(4) = mk('bird',  [150;150;9.5],[0;-2;0],    0.5, [ 0 -60 6],   [260 200 16], 0.02,  0.3);
obs(5) = mk('balloon',[52;18.6;10.3],[0;0;0],   1.3, [-1e3 -1e3 0],[1e3 1e3 60], 0.0,   0.0);  % static
obs(6) = mk('bird',  [212;30;4.5], [0;-0.8;0],  0.5, [200 10 2.5], [220 40 8],   0.02,  0.2);
end

function o = mk(type, pos, vel, r, lo, hi, turn, vzAmp)
o.type = type; o.pos = pos(:); o.vel = vel(:); o.v0 = vel(:); o.speed = norm(vel(1:2));
o.r = r; o.lo = lo(:); o.hi = hi(:); o.turn = turn; o.vzAmp = vzAmp;
o.phase = 6.28*rand;
end

function obs = stepObstacles(obs, dt, t)
for i = 1:numel(obs)
    o = obs(i);
    switch o.type
        case 'bird'
            ang = o.turn*randn*sqrt(dt);                                   % heading random walk
            cs = cos(ang); sn = sin(ang);
            o.vel(1:2) = [cs -sn; sn cs]*o.vel(1:2);
            o.vel(1:2) = o.vel(1:2)/max(norm(o.vel(1:2)),1e-6)*o.speed;
            o.vel(3) = o.vzAmp*cos(0.9*t + o.phase);                      % flapping climb/sink
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
    for ax = 1:3                                                          % reflect inside its box
        if o.pos(ax) < o.lo(ax), o.pos(ax) = o.lo(ax); o.vel(ax) = abs(o.vel(ax)); o.v0(ax) = abs(o.v0(ax));
        elseif o.pos(ax) > o.hi(ax), o.pos(ax) = o.hi(ax); o.vel(ax) = -abs(o.vel(ax)); o.v0(ax) = -abs(o.v0(ax)); end
    end
    obs(i) = o;
end
end

function [wind, gust] = windModel(t, gust, P)
Wm = P.windMean + P.windVar*sin(0.07*t);
th = P.windDir + 0.6*sin(0.045*t + 1);
a  = exp(-P.dt/P.gustTau);
gust = a*gust + sqrt(1-a^2)*P.gustSigma*[randn; randn; 0.3*randn];
wind = Wm*[cos(th); sin(th); 0] + gust;
end

%% ======================================================================
%%  RADAR, PREDICTION, AVOIDANCE
%% ======================================================================
function det = radarScan(p, v, obs, P)
det = [];
for i = 1:numel(obs)
    rel = obs(i).pos - p;  dist = norm(rel);
    if dist > P.radarRange, continue; end
    relv = obs(i).vel - v;
    Rs = obs(i).r + P.rDrone + P.margin + P.speedGain*norm(relv);       % required safety distance
    s.id = i; s.type = obs(i).type; s.dist = dist; s.rel = rel; s.relv = relv;
    s.height = obs(i).pos(3);
    s.dir = obs(i).vel/max(norm(obs(i).vel),1e-6);
    s.heading = atan2d(obs(i).vel(2), obs(i).vel(1));
    s.closing = -dot(rel, relv)/max(dist,1e-6);
    s.Rs = Rs;
    s.ttc = timeToCollision(rel, relv, Rs);
    if isempty(det), det = s; else, det(end+1) = s; end %#ok<AGROW>
end
end

function ttc = timeToCollision(rel, relv, Rs)
a = dot(relv,relv); b = 2*dot(rel,relv); cc = dot(rel,rel) - Rs^2;
if cc < 0, ttc = 0; return; end
if a < 1e-9, ttc = inf; return; end
disc = b^2 - 4*a*cc;
if disc < 0, ttc = inf; return; end
tt = (-b - sqrt(disc))/(2*a);
if tt < 0, ttc = inf; else, ttc = tt; end
end

function A = avoidanceUpdate(A, t, p, v, sg, obs, P, bld, zeroOff)
det = radarScan(p, v, obs, P);
A.det = det;
nO = numel(obs);
inRange = false(1,nO);
if ~isempty(det), inRange([det.id]) = true; end
for i = find(inRange & ~A.seen)
    s = det([det.id] == i);
    fprintf(['[t=%6.2f s] RADAR   : %-7s #%d  dist=%5.1f m  rel=[%6.1f %6.1f %6.1f]  relV=[%5.1f %5.1f %5.1f]' ...
             '  height=%5.1f m  heading=%4.0f deg  TTC=%s  safety=%.1f m\n'], ...
        t, s.type, i, s.dist, s.rel, s.relv, s.height, s.heading, ttcStr(s.ttc), s.Rs);
end
A.seen = inRange;
A.threat = false(1,nO);

if isempty(det)
    A = tryRelease(A, t, P, zeroOff, true);
    return
end

idx = [det.id];  Rs = [det.Rs];
ro0 = rolloutRisk(p, v, sg, zeroOff, obs, idx, Rs, P, bld);       % nominal (mission) path risk
if ro0.margin >= P.buf
    A = tryRelease(A, t, P, zeroOff, all([det.dist] > 1.5*Rs));
    return
end

A.clearTimer = 0;
A.threat(idx(ro0.per < P.buf)) = true;
needNew = true;
if A.active
    roc = rolloutRisk(p, v, sg, A.off, obs, idx, Rs, P, bld);
    if roc.margin >= P.buf, needNew = false; end                   % current maneuver still works
end
if ~needNew, return; end

cands = buildCandidates(p, sg, ro0, Rs, A, P, zeroOff);
c0 = carrotTarget(p, sg, zeroOff, P);  z0 = c0(3);
bestCost = inf; best = []; bestM = -inf; fallback = []; 
for q = 1:numel(cands)
    cand = cands{q};
    roq = rolloutRisk(p, v, sg, cand, obs, idx, Rs, P, bld);
    cost = 1.8*abs(cand.lat) + 1.2*abs(cand.along);
    if ~isnan(cand.z)
        if cand.z >= z0, cost = cost + (cand.z - z0); else, cost = cost + 2.5*(z0 - cand.z); end
    end
    if strncmp(cand.name,'EMERG',5), cost = cost + 6; end
    if A.active && strcmp(cand.name, A.off.name), cost = 0.7*cost; end   % hysteresis
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
if strcmp(key, A.lastKey), return; end                              % do not repeat identical messages
A.lastKey = key;
fprintf('[t=%6.2f s] DECIDE  : %s%s -> target Z = %s, lateral = %+.1f m, along = %+.1f m  (predicted clearance %.1f m)\n', ...
    t, best.name, ternary(emergency,' (best effort)',''), zs, best.lat, best.along, best.margin);
for j = find(ro0.per < P.buf)
    fprintf('            threat %s #%d: closest approach in %.1f s at [%.1f %.1f %.1f], predicted miss %.1f m < required %.1f m\n', ...
        det(j).type, idx(j), ro0.tt(j), ro0.tpos(:,j), ro0.per(j)+Rs(j), Rs(j));
end
end

function A = tryRelease(A, t, P, zeroOff, farEnough)
if ~A.active, return; end
if farEnough, A.clearTimer = A.clearTimer + P.replanDt; else, A.clearTimer = 0; end
if A.clearTimer >= P.releaseTime
    A.active = false; A.off = zeroOff; A.clearTimer = 0; A.lastKey = '';
    fprintf('[t=%6.2f s] AVOID   : obstacle cleared -> avoidance OFF, returning to original route / mission altitude\n', t);
end
end

function cands = buildCandidates(p, sg, ro, Rs, A, P, zeroOff) %#ok<INUSL>
thr = find(ro.per < P.buf);
S = sg.S; T = sg.T; dxy = T(1:2) - S(1:2); Ls = norm(dxy);
if Ls < 1, u = [1;0]; ref0 = T(1:2); else, u = dxy/Ls; ref0 = S(1:2); end
perp = [-u(2); u(1)];
zo = ro.tpos(3,thr);  Rt = Rs(thr);
lat_o = perp'*(ro.tpos(1:2,thr) - ref0);
extra = P.buf + 0.7;                      % small reserve on top of the safety distance
zc = max(zo + Rt + extra);               % obstacle height + required clearance
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

function ro = rolloutRisk(p, v, sg, off, obs, idx, Rs, P, bld)
% Kinematic look-ahead of the drone's own path (same reference generator as the
% real controller) against the predicted obstacle positions:
%   predictedPosition = currentPosition + obstacleVelocity * predictionTime
N = round(P.Th/P.dtPred); nO = numel(idx);
ro.per = inf(1,nO); ro.tpos = zeros(3,nO); ro.tt = zeros(1,nO); ro.hit = false;
pq = p; vq = v;
for kk = 1:N
    tq = kk*P.dtPred;
    cq = carrotTarget(pq, sg, off, P);
    [pq, vq] = refStep(pq, vq, cq, off.speedScale, P, P.dtPred);
    for j = 1:nO
        o = obs(idx(j));
        po = o.pos + o.vel*tq;
        m = norm(pq - po) - Rs(j);
        if m < ro.per(j), ro.per(j) = m; ro.tpos(:,j) = po; ro.tt(j) = tq; end
    end
    if inBuilding(pq, bld), ro.hit = true; end
end
ro.margin = min(ro.per);
if ro.hit, ro.margin = min(ro.margin, -5); end
end

function tf = inBuilding(q, bld)
tf = false;
for b = 1:size(bld,1)
    if abs(q(1)-bld(b,1)) < bld(b,3)/2 + 3 && abs(q(2)-bld(b,2)) < bld(b,4)/2 + 3 && ...
       (numel(q) < 3 || q(3) < bld(b,5) + 2)
        tf = true; return;
    end
end
end

function c = carrotTarget(p, sg, off, P)
% 3D point the drone is steered to: a look-ahead point on the mission segment,
% shifted by the avoidance offsets (lateral / along / absolute Z).
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
    cz  = S(3) + (T(3)-S(3))*min(max(st/Ls,0),1);       % glide slope between waypoints
end
if ~isnan(off.z), cz = off.z; end                        % dynamic Z target
c = [cxy; cz];
end

function [pr, vr] = refStep(pr, vr, carrot, vScale, P, dt)
% Smooth reference: max horizontal speed/accel, max climb/descent rate, max vertical accel.
e = carrot - pr; exy = e(1:2); dxy = norm(exy);
if dxy > 1e-6, vh = exy/dxy*min(P.kRef*dxy, P.vMaxH*vScale); else, vh = [0;0]; end
vdn = min(P.vDown, 0.5 + 0.6*pr(3));                    % soft touchdown near ground
vz  = min(max(P.kRefZ*e(3), -vdn), P.vUp);
dv  = [vh - vr(1:2); vz - vr(3)];
lim = P.aMaxH*dt; dvh = norm(dv(1:2));
if dvh > lim, dv(1:2) = dv(1:2)/dvh*lim; end
dv(3) = min(max(dv(3), -P.aMaxV*dt), P.aMaxV*dt);
vr = vr + dv;  pr = pr + vr*dt;
end

%% ======================================================================
%%  SMALL HELPERS
%% ======================================================================
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
%%  GRAPHS: individual (Figure 1) + combined (Figure 2)
%% ======================================================================
function plotSixDOF(Lg, nR, segs)
t   = Lg.t(1:nR);
pos = Lg.p(:,1:nR);
vel = Lg.v(:,1:nR);
ang = rad2deg(Lg.ang(:,1:nR));                 % [roll; pitch; yaw] in deg
acc = zeros(3,nR);  rate = zeros(3,nR);  aacc = zeros(3,nR);
for i = 1:3
    acc(i,:)  = movmean(gradient(vel(i,:), t), 9);
    rate(i,:) = movmean(rad2deg(gradient(unwrap(deg2rad(ang(i,:))), t)), 9);
    aacc(i,:) = movmean(gradient(rate(i,:), t), 15);      % angular acceleration [deg/s^2]
end
spd = sqrt(sum(vel.^2,1));
col = {[0.85 0.1 0.1],[0.1 0.6 0.1],[0.1 0.3 0.9]};   % X/roll, Y/pitch, Z/yaw
seg = Lg.seg(1:nR);
chg = find(diff(seg) ~= 0) + 1;
tPh = t(chg);  kPh = seg(chg);
W = [segs(1).S, segs(1).T, segs(2).T, segs(3).T, segs(4).T, segs(6).T, segs(7).T, segs(8).T];

%% ===== FIGURE 1: every signal on its own graph =====
f1 = figure('Color','w','Position',[30 30 1500 1100],'Name','Individual graphs','NumberTitle','off');
tl = tiledlayout(f1,6,3,'TileSpacing','compact','Padding','compact');
title(tl,'Individual graphs','FontWeight','bold');
groups = { pos, 'Position',     {'X','Y','Z'},                           'm';
           vel, 'Velocity',     {'Vx','Vy','Vz'},                        'm/s';
           acc, 'Acceleration', {'ax','ay','az'},                        'm/s^2';
           ang, 'Angle',        {'Roll \phi','Pitch \theta','Yaw \psi'}, 'deg';
           rate,'Rate',         {'Roll rate','Pitch rate','Yaw rate'},   'deg/s';
           aacc,'Angular accel',{'Roll accel','Pitch accel','Yaw accel'}, 'deg/s^2' };
for g = 1:size(groups,1)
    for i = 1:3
        ax = nexttile(tl); grid(ax,'on'); hold(ax,'on');
        plot(ax, t, groups{g,1}(i,:), 'Color', col{i}, 'LineWidth', 1.3);
        title(ax, sprintf('%s - %s', groups{g,2}, groups{g,3}{i}));
        ylabel(ax, groups{g,4});
        if g == size(groups,1), xlabel(ax,'Time [s]'); end
        addPhaseLines(ax, t, tPh);
    end
end

%% ===== FIGURE 2: combined graphs =====
f2 = figure('Color','w','Position',[70 70 1500 850],'Name','Combined graphs','NumberTitle','off');
tl2 = tiledlayout(f2,2,4,'TileSpacing','compact','Padding','compact');
title(tl2,'Combined graphs','FontWeight','bold');

ax = nexttile(tl2); hold(ax,'on'); grid(ax,'on');
for i = 1:3, plot(ax,t,pos(i,:),'Color',col{i},'LineWidth',1.4); end
title(ax,'Position (X, Y, Z)'); xlabel(ax,'Time [s]'); ylabel(ax,'m');
legend(ax,'X','Y','Z','Location','best'); addPhaseLines(ax,t,tPh);
labelPhases(ax, tPh, kPh, segs);

ax = nexttile(tl2); hold(ax,'on'); grid(ax,'on');
for i = 1:3, plot(ax,t,vel(i,:),'Color',col{i},'LineWidth',1.4); end
plot(ax,t,spd,'k--','LineWidth',1.2);
title(ax,'Velocity (Vx, Vy, Vz, speed)'); xlabel(ax,'Time [s]'); ylabel(ax,'m/s');
legend(ax,'Vx','Vy','Vz','|V|','Location','best'); addPhaseLines(ax,t,tPh);

ax = nexttile(tl2); hold(ax,'on'); grid(ax,'on');
for i = 1:3, plot(ax,t,acc(i,:),'Color',col{i},'LineWidth',1.2); end
title(ax,'Acceleration (ax, ay, az)'); xlabel(ax,'Time [s]'); ylabel(ax,'m/s^2');
legend(ax,'ax','ay','az','Location','best'); addPhaseLines(ax,t,tPh);

ax = nexttile(tl2); hold(ax,'on'); grid(ax,'on');
for i = 1:3, plot(ax,t,ang(i,:),'Color',col{i},'LineWidth',1.4); end
title(ax,'Attitude (roll, pitch, yaw)'); xlabel(ax,'Time [s]'); ylabel(ax,'deg');
legend(ax,'Roll','Pitch','Yaw','Location','best'); addPhaseLines(ax,t,tPh);

ax = nexttile(tl2); hold(ax,'on'); grid(ax,'on');
for i = 1:3, plot(ax,t,rate(i,:),'Color',col{i},'LineWidth',1.2); end
title(ax,'Angular rates (roll, pitch, yaw)'); xlabel(ax,'Time [s]'); ylabel(ax,'deg/s');
legend(ax,'Roll rate','Pitch rate','Yaw rate','Location','best'); addPhaseLines(ax,t,tPh);

ax = nexttile(tl2); hold(ax,'on'); grid(ax,'on');
for i = 1:3, plot(ax,t,aacc(i,:),'Color',col{i},'LineWidth',1.1); end
title(ax,'Angular accel (roll, pitch, yaw)'); xlabel(ax,'Time [s]'); ylabel(ax,'deg/s^2');
legend(ax,'Roll accel','Pitch accel','Yaw accel','Location','best'); addPhaseLines(ax,t,tPh);

ax = nexttile(tl2); hold(ax,'on'); grid(ax,'on');
scatter3(ax,pos(1,:),pos(2,:),pos(3,:),6,seg,'filled');
plot3(ax,W(1,:),W(2,:),W(3,:),'k:');
title(ax,'3D path (colour = mission phase)'); xlabel(ax,'X [m]'); ylabel(ax,'Y [m]'); zlabel(ax,'Z [m]');
view(ax,3); axis(ax,'equal'); colormap(ax,'turbo'); colorbar(ax);
end

function addPhaseLines(ax, t, tPh)
for q = 1:numel(tPh)
    xline(ax, tPh(q), ':', 'Color',[0.45 0.45 0.45], 'HandleVisibility','off');
end
xlim(ax,[t(1) max(t(end), t(1)+eps)]);
end

function labelPhases(ax, tPh, kPh, segs)
yl = ylim(ax);
for q = 1:numel(tPh)
    text(ax, tPh(q), yl(2), [' ' segs(kPh(q)).name], 'Rotation',90, 'FontSize',7, ...
        'Color',[0.3 0.3 0.3], 'VerticalAlignment','top', 'HorizontalAlignment','right');
end
end

%% ======================================================================
%%  3D VISUALIZATION
%% ======================================================================
function animateFlight(Lg, nR, P, bld, trees, segs, obs0, opts)
nO = numel(obs0);
fig = figure('Color',[0.08 0.08 0.10],'Position',[40 40 1560 880],'Name','Dynamic avoidance delivery drone','NumberTitle','off');
ax = axes('Parent',fig,'Position',[0.01 0.02 0.64 0.96]); hold(ax,'on');
set(ax,'Color',[0.62 0.78 0.95],'XColor','none','YColor','none','ZColor','none','Clipping','off');
axis(ax,[-40 260 -100 120 0 45]); daspect(ax,[1 1 1]); camproj(ax,'perspective');

% ----- ground, roads, pad ------------------------------------------------------
patch(ax,'XData',[-60 270 270 -60],'YData',[-110 -110 130 130],'ZData',[0 0 0 0],'FaceColor',[0.36 0.58 0.32],'EdgeColor','none');
patch(ax,'XData',[-40 260 260 -40],'YData',[-4 -4 4 4],'ZData',0.05*ones(1,4),'FaceColor',[0.22 0.22 0.24],'EdgeColor','none');
patch(ax,'XData',[101 109 109 101],'YData',[-100 -100 120 120],'ZData',0.05*ones(1,4),'FaceColor',[0.22 0.22 0.24],'EdgeColor','none');
plot3(ax,[-40 260],[0 0],[0.08 0.08],'--','Color',[0.9 0.9 0.5],'LineWidth',1);
plot3(ax,[105 105],[-100 120],[0.08 0.08],'--','Color',[0.9 0.9 0.5],'LineWidth',1);
th = linspace(0,2*pi,30);
patch(ax,'XData',3*cos(th),'YData',3*sin(th),'ZData',0.1*ones(size(th)),'FaceColor',[0.95 0.8 0.1],'EdgeColor','k');
text(ax,0,0,0.3,'H','Color','k','FontWeight','bold','HorizontalAlignment','center');

% ----- buildings ----------------------------------------------------------------
bcol = [0.65 0.65 0.70; 0.72 0.60 0.55; 0.55 0.62 0.72; 0.70 0.70 0.60];
for b = 1:size(bld,1)
    drawBox(ax,[bld(b,1) bld(b,2) bld(b,5)/2],[bld(b,3) bld(b,4) bld(b,5)],bcol(mod(b-1,4)+1,:),1);
end
% ----- trees -----------------------------------------------------------------------
[Xc,Yc,Zc] = cylinder([1 0],10);  [Xt,Yt,Zt] = cylinder(1,8);
for i = 1:size(trees,1)
    tx = trees(i,1); ty = trees(i,2); h = trees(i,3);
    surf(ax, tx+0.25*Xt, ty+0.25*Yt, 0.4*h*Zt,'FaceColor',[0.4 0.25 0.1],'EdgeColor','none');
    surf(ax, tx+0.28*h*Xc, ty+0.28*h*Yc, 0.3*h+0.7*h*Zc,'FaceColor',[0.1 0.42+0.1*mod(i,3)/2 0.15],'EdgeColor','none');
end
% ----- waypoints / nominal route -----------------------------------------------------
W = [segs(1).S, segs(1).T, segs(2).T, segs(3).T, segs(4).T, segs(6).T, segs(7).T, segs(8).T];
plot3(ax,W(1,:),W(2,:),W(3,:),':','Color',[1 1 1],'LineWidth',1.2);
plot3(ax,W(1,2:5),W(2,2:5),W(3,2:5),'o','MarkerSize',8,'MarkerFaceColor',[1 0.9 0.1],'MarkerEdgeColor','k');
lbl = {'TAKEOFF','WP1','WP2','CUSTOMER'};
for i = 1:4, text(ax,W(1,i+1),W(2,i+1),W(3,i+1)+2.5,lbl{i},'Color','k','FontWeight','bold','HorizontalAlignment','center'); end
light(ax,'Position',[-100 -150 200],'Style','local'); lighting(ax,'gouraud');

% ----- drone, obstacles, radar -------------------------------------------------------
dr  = buildDrone(ax, P);
ob  = buildObstacles(ax, obs0);
Hr  = hgtransform('Parent',ax);
[Xs,Ys,Zs] = sphere(22);
surf(P.radarRange*Xs,P.radarRange*Ys,P.radarRange*Zs,'Parent',Hr,'FaceColor',[0.1 0.9 0.9],'FaceAlpha',0.04,'EdgeColor',[0.1 0.9 0.9],'EdgeAlpha',0.10);
plot3(P.radarRange*cos(th),P.radarRange*sin(th),0*th,'Parent',Hr,'Color',[0.2 1 1],'LineWidth',1);
hTrail  = plot3(ax,nan,nan,nan,'-','Color',[0.1 0.3 1],'LineWidth',2.2);
hTrailA = plot3(ax,nan,nan,nan,'-','Color',[1 0.2 0.1],'LineWidth',3);
hTgt    = plot3(ax,nan,nan,nan,'gd','MarkerFaceColor','g','MarkerSize',9);
hTgtL   = plot3(ax,nan,nan,nan,'g-','LineWidth',1);
hShadow = plot3(ax,nan,nan,nan,'k.','MarkerSize',18);
Hpkg = hgtransform('Parent',ax,'Visible','off');
drawBox(Hpkg,[0 0 0],P.visScale*[0.22 0.22 0.14],[0.75 0.55 0.3],1);
hHUD = annotation(fig,'textbox',[0.012 0.60 0.20 0.38],'String','','Color','w','BackgroundColor',[0 0 0], ...
    'FaceAlpha',0.55,'EdgeColor','none','FontName','Consolas','FontSize',10,'Interpreter','none','FitBoxToText','off');

% ----- live telemetry panels ------------------------------------------------------------
tEnd = Lg.t(nR);
pa = @(y0) axes('Parent',fig,'Position',[0.71 y0 0.275 0.175],'Color',[0.12 0.12 0.15], ...
    'XColor',[0.8 0.8 0.8],'YColor',[0.8 0.8 0.8],'GridColor',[0.5 0.5 0.5],'FontSize',8,'XLim',[0 tEnd]);
a1 = pa(0.79); hold(a1,'on'); grid(a1,'on'); title(a1,'Altitude: actual / reference / mission / obstacle','Color','w','FontSize',9);
h1z = plot(a1,nan,nan,'c','LineWidth',1.6); h1r = plot(a1,nan,nan,'y--'); h1m = plot(a1,nan,nan,'w:','LineWidth',1.4); h1o = plot(a1,nan,nan,'m','LineWidth',1.2);
ylabel(a1,'m','Color','w'); ylim(a1,[0 max(25,max(Lg.p(3,1:nR))+3)]);
a2 = pa(0.58); hold(a2,'on'); grid(a2,'on'); title(a2,'Total thrust vs weight','Color','w','FontSize',9);
h2T = plot(a2,nan,nan,'g','LineWidth',1.6); h2W = plot(a2,nan,nan,'r--','LineWidth',1.4); ylabel(a2,'N','Color','w'); ylim(a2,[0 max(Lg.T(1:nR))*1.1]);
a3 = pa(0.37); hold(a3,'on'); grid(a3,'on'); title(a3,'Motor speeds M1-M4','Color','w','FontSize',9);
mc = {[1 0.3 0.3],[1 0.7 0.2],[0.3 0.8 1],[0.6 0.5 1]};
for j = 1:4, h3(j) = plot(a3,nan,nan,'Color',mc{j}); end %#ok<AGROW>
ylabel(a3,'rad/s','Color','w'); ylim(a3,[0 P.wMax*1.05]); legend(a3,'M1','M2','M3','M4','TextColor','w','Color',[0.12 0.12 0.15],'Location','northeast','FontSize',7,'Orientation','horizontal');
a4 = pa(0.16); hold(a4,'on'); grid(a4,'on'); title(a4,'Roll / Pitch / Yaw','Color','w','FontSize',9);
h4(1) = plot(a4,nan,nan,'r'); h4(2) = plot(a4,nan,nan,'g'); h4(3) = plot(a4,nan,nan,'c'); ylabel(a4,'deg','Color','w'); xlabel(a4,'time [s]','Color','w');
ylim(a4,[-40 40]);

pAv = Lg.p(:,1:nR);  isNone = strcmp(Lg.mode(1:nR),'NONE'); pAv(:,isNone) = nan;
phase = zeros(1,4); spinDir = [1 -1 1 -1];
mp = [P.d -P.d; P.d P.d; -P.d P.d; -P.d -P.d];
camPos = []; 
frames = unique([1:opts.animStride:nR, nR]);

for i = frames
    if ~ishandle(fig), return; end
    pk = Lg.p(:,i);  Rk = eul2R(Lg.ang(:,i));
    set(dr.H,'Matrix', makehgtform('translate',pk')*[Rk zeros(3,1); 0 0 0 1]*makehgtform('scale',P.visScale));
    for j = 1:4
        phase(j) = phase(j) + spinDir(j)*Lg.wm(j,i)/700*0.9*opts.animStride;
        set(dr.Hs(j),'Matrix',makehgtform('translate',[mp(j,:) 0.085])*makehgtform('zrotate',phase(j)));
    end
    set(dr.pkg,'Visible',ternary(Lg.pay(i) > 0,'on','off'));
    set(Hpkg,'Visible',ternary(Lg.pkgOn(i),'on','off'));
    if Lg.pkgOn(i), set(Hpkg,'Matrix',makehgtform('translate',Lg.pkg(:,i)')); end
    set(Hr,'Matrix',makehgtform('translate',pk'));
    set(hTrail,'XData',Lg.p(1,1:i),'YData',Lg.p(2,1:i),'ZData',Lg.p(3,1:i));
    set(hTrailA,'XData',pAv(1,1:i),'YData',pAv(2,1:i),'ZData',pAv(3,1:i));
    set(hTgt,'XData',Lg.carrot(1,i),'YData',Lg.carrot(2,i),'ZData',Lg.carrot(3,i));
    set(hTgtL,'XData',[pk(1) Lg.carrot(1,i)],'YData',[pk(2) Lg.carrot(2,i)],'ZData',[pk(3) Lg.carrot(3,i)]);
    set(hShadow,'XData',pk(1),'YData',pk(2),'ZData',0.1);

    for j = 1:nO
        po = Lg.obsP(:,j,i); vo = Lg.obsV(:,j,i);
        yaw = atan2(vo(2),vo(1)); if norm(vo(1:2)) < 0.05, yaw = 0; end
        set(ob.H(j),'Matrix',makehgtform('translate',po')*makehgtform('zrotate',yaw));
        if ~isempty(ob.Hwl{j})
            fl = 0.7*sin(2*pi*1.8*Lg.t(i) + j);
            set(ob.Hwl{j},'Matrix',makehgtform('xrotate',fl)); set(ob.Hwr{j},'Matrix',makehgtform('xrotate',-fl));
        end
        pp = po + 3*vo;                                      % predicted position 3 s ahead
        colr = [1 0.6 0.1]; lw = 1; vis = 'off';
        if Lg.det(j,i), vis = 'on'; end
        if Lg.thr(j,i), colr = [1 0 0]; lw = 2.5; end
        set(ob.pred(j),'XData',[po(1) pp(1)],'YData',[po(2) pp(2)],'ZData',[po(3) pp(3)],'Color',colr,'LineWidth',lw,'Visible',vis);
        set(ob.predM(j),'XData',pp(1),'YData',pp(2),'ZData',pp(3),'MarkerEdgeColor',colr,'Visible',vis);
        set(ob.txt(j),'Position',po'+[0 0 ob.r(j)+2.2],'Color',ternary(Lg.thr(j,i),[1 0.1 0.1],[0.1 0.1 0.1]));
    end

    set(h1z,'XData',Lg.t(1:i),'YData',Lg.p(3,1:i)); set(h1r,'XData',Lg.t(1:i),'YData',Lg.pref(3,1:i));
    set(h1m,'XData',Lg.t(1:i),'YData',Lg.zMis(1:i)); set(h1o,'XData',Lg.t(1:i),'YData',Lg.zObs(1:i));
    set(h2T,'XData',Lg.t(1:i),'YData',Lg.T(1:i)); set(h2W,'XData',Lg.t(1:i),'YData',Lg.W(1:i));
    for j = 1:4, set(h3(j),'XData',Lg.t(1:i),'YData',Lg.wm(j,1:i)); end
    for j = 1:3, set(h4(j),'XData',Lg.t(1:i),'YData',rad2deg(Lg.ang(j,1:i))); end

    vh = norm(Lg.v(1:2,i)); ws = norm(Lg.wind(1:2,i)); wd = atan2d(Lg.wind(2,i),Lg.wind(1,i));
    nearTxt = 'none in radar range';
    if ~isnan(Lg.near(i))
        nearTxt = sprintf('%.1f m  TTC %s', Lg.near(i), ttcStr(Lg.ttc(i)));
    end
    set(hHUD,'String',{ sprintf('t = %6.1f s', Lg.t(i)); ...
        sprintf('PHASE   : %s', segs(Lg.seg(i)).name); ...
        sprintf('AVOID   : %s', Lg.mode{i}); ...
        sprintf('POS     : %6.1f %6.1f %5.1f m', pk); ...
        sprintf('Z target: %5.1f m (mission %.1f)', Lg.pref(3,i), Lg.zMis(i)); ...
        sprintf('SPEED   : %4.1f m/s  climb %+4.1f', vh, Lg.v(3,i)); ...
        sprintf('THRUST  : %5.1f N  weight %5.1f N', Lg.T(i), Lg.W(i)); ...
        sprintf('MOTORS  : %4.0f %4.0f %4.0f %4.0f', Lg.wm(:,i)); ...
        sprintf('RPY deg : %+5.1f %+5.1f %+6.1f', rad2deg(Lg.ang(:,i))); ...
        sprintf('WIND    : %.1f m/s  dir %4.0f deg', ws, wd); ...
        sprintf('NEAREST : %s', nearTxt) });

    desired = pk + [-30; -32; 16];
    if strcmp(opts.cam,'follow')
        if isempty(camPos), camPos = desired; else, camPos = 0.9*camPos + 0.1*desired; end
        set(ax,'CameraPosition',camPos','CameraTarget',(pk+[8;3;0])','CameraViewAngle',32,'CameraUpVector',[0 0 1]);
    else
        view(ax,[38 24]); set(ax,'CameraViewAngle',30);
    end
    drawnow limitrate;
end
end

function h = drawBox(par, c, sz, col, alpha)
V = [-1 -1 -1; 1 -1 -1; 1 1 -1; -1 1 -1; -1 -1 1; 1 -1 1; 1 1 1; -1 1 1];
V = V.*(sz(:)'/2) + c(:)';
F = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
h = patch('Parent',par,'Vertices',V,'Faces',F,'FaceColor',col,'EdgeColor','k','EdgeAlpha',0.25,'FaceAlpha',alpha);
end

function dr = buildDrone(ax, P)
H = hgtransform('Parent',ax);  d = P.d;
drawBox(H,[0 0 0],[0.30 0.20 0.07],[0.15 0.15 0.18],1);
drawBox(H,[0 0 0.05],[0.18 0.14 0.04],[0.92 0.92 0.97],1);
drawBox(H,[0.16 0 0.02],[0.04 0.10 0.04],[0.95 0.1 0.1],1);                 % nose marker
plot3([-d d],[-d d],[0 0],'Parent',H,'Color',[0.1 0.1 0.1],'LineWidth',4);
plot3([-d d],[d -d],[0 0],'Parent',H,'Color',[0.1 0.1 0.1],'LineWidth',4);
mp = [d -d; d d; -d d; -d -d];
[Xm,Ym,Zm] = cylinder(0.035,12);
pc = {[1 0.3 0.3],[1 0.3 0.3],[0.3 0.6 1],[0.3 0.6 1]};
th = linspace(0,2*pi,24);
for j = 1:4
    surf(Xm+mp(j,1),Ym+mp(j,2),Zm*0.05+0.02,'Parent',H,'FaceColor',[0.25 0.25 0.25],'EdgeColor','none');
    Hs(j) = hgtransform('Parent',H,'Matrix',makehgtform('translate',[mp(j,:) 0.085])); %#ok<AGROW>
    patch('Parent',Hs(j),'XData',0.17*cos(th),'YData',0.17*sin(th),'ZData',0*th,'FaceColor',pc{j},'FaceAlpha',0.18,'EdgeColor','none');
    patch('Parent',Hs(j),'XData',[-0.17 0.17 0.17 -0.17],'YData',[-0.012 -0.012 0.012 0.012],'ZData',zeros(1,4),'FaceColor',pc{j},'EdgeColor','none');
end
% landing gear
for sx = [-1 1]
    for sy = [-1 1]
        plot3([0.10*sx 0.13*sx],[0.07*sy 0.16*sy],[-0.03 -0.22],'Parent',H,'Color','k','LineWidth',1.5);
    end
end
plot3([-0.2 0.2],[0.16 0.16],[-0.22 -0.22],'Parent',H,'Color','k','LineWidth',2);
plot3([-0.2 0.2],[-0.16 -0.16],[-0.22 -0.22],'Parent',H,'Color','k','LineWidth',2);
dr.pkg = drawBox(H,[0 0 -0.12],[0.22 0.22 0.14],[0.75 0.55 0.3],1);        % delivery package
dr.H = H; dr.Hs = Hs;
end

function ob = buildObstacles(ax, obs0)
n = numel(obs0);
for i = 1:n
    o = obs0(i);
    H = hgtransform('Parent',ax); ob.H(i) = H; ob.Hwl{i} = []; ob.Hwr{i} = [];
    switch o.type
        case 'bird'
            vs = 1.8;
            [X,Y,Z] = ellipsoid(0,0,0,0.5*vs,0.15*vs,0.13*vs,10);
            surf(X,Y,Z,'Parent',H,'FaceColor',[0.35 0.22 0.1],'EdgeColor','none');
            Hl = hgtransform('Parent',H); Hr = hgtransform('Parent',H);
            wv = vs*[0.2 0 0; -0.2 0 0; -0.35 0.9 0; 0.0 0.9 0];
            patch('Parent',Hl,'Vertices',wv,'Faces',[1 2 3 4],'FaceColor',[0.25 0.15 0.08],'EdgeColor','none');
            patch('Parent',Hr,'Vertices',wv.*[1 -1 1],'Faces',[1 2 3 4],'FaceColor',[0.25 0.15 0.08],'EdgeColor','none');
            ob.Hwl{i} = Hl; ob.Hwr{i} = Hr;
        case 'kite'
            patch('Parent',H,'Vertices',[0 0 1.4; 0 0.9 0; 0 0 -1.4; 0 -0.9 0],'Faces',[1 2 3 4],'FaceColor',[0.95 0.2 0.3],'EdgeColor','k');
            plot3([0 0 0 0 0],[0 0.3 -0.3 0.3 -0.3],[-1.4 -2.2 -3 -3.8 -4.6],'Parent',H,'Color',[1 0.8 0.1],'LineWidth',1.5);
        case 'drone'
            drawBox(H,[0 0 0],[0.9 0.6 0.25],[0.2 0.2 0.25],1);
            plot3([-0.9 0.9],[-0.9 0.9],[0 0],'Parent',H,'Color','k','LineWidth',2.5);
            plot3([-0.9 0.9],[0.9 -0.9],[0 0],'Parent',H,'Color','k','LineWidth',2.5);
            th = linspace(0,2*pi,16);
            for q = [-0.9 0.9]
                for s = [-0.9 0.9]
                    patch('Parent',H,'XData',q+0.45*cos(th),'YData',s+0.45*sin(th),'ZData',0.15+0*th,'FaceColor',[1 0.5 0.1],'FaceAlpha',0.5,'EdgeColor','none');
                end
            end
        otherwise  % balloon
            [X,Y,Z] = sphere(16);
            surf(o.r*X,o.r*Y,o.r*Z,'Parent',H,'FaceColor',[0.95 0.75 0.1],'EdgeColor',[0.6 0.3 0],'EdgeAlpha',0.4);
    end
    ob.pred(i)  = plot3(ax,nan,nan,nan,'-','Color',[1 0.6 0.1],'Visible','off');
    ob.predM(i) = plot3(ax,nan,nan,nan,'o','Color',[1 0.6 0.1],'MarkerSize',6,'Visible','off');
    ob.txt(i)   = text(ax,0,0,0,sprintf('%s #%d',o.type,i),'FontSize',8,'FontWeight','bold','HorizontalAlignment','center');
    ob.r(i)     = o.r;
end
end
