function out = drone_swarm_lqr(varargin)
%DRONE_SWARM_LQR  Multi-drone delivery swarm, LQR control, dynamic 3D obstacle avoidance.
%
%   Run:   drone_swarm_lqr
%          drone_swarm_lqr('nDrones',5)
%          drone_swarm_lqr('nDrones',2,'twr',3,'mPayload',1.0,'windMean',6)
%          drone_swarm_lqr('animate',false)            % numbers + graphs only
%          drone_swarm_lqr('cam','overview','seed',3)
%
%   NO hard-coded numbers in the simulation code: every value lives in
%   defaultConfig() (override any field with a name/value pair) or is DERIVED
%   from physics (inertia from geometry, wMax from thrust/weight, limits from tilt
%   and thrust authority, controller gains from LQR/Riccati).  Each drone gets its
%   own parameter set (unit-to-unit spread, cfg.spread) and its own LQR gains.
%
%   Signal chain (every control step, per drone):
%     Radar -> prediction -> avoidance decision -> new 3D target (carrot)
%     -> reference generator (rate/accel limits)
%     -> POSITION LQR (3 axes, states [pos err; vel err; integral], gains from Riccati)
%     -> F = m(a+g) -> thrust + desired roll/pitch
%     -> ATTITUDE LQR (roll, pitch, yaw, gains from Riccati) -> tau = J*alpha + w x J*w
%     -> motor mixer -> motor lag, F_i = kf*w_i^2 -> rigid-body dynamics
%        (gravity, drag, wind + gusts, payload mass change)
%
%   Swarm: every drone treats the other drones as moving obstacles (radar +
%   constant-velocity prediction), and drones cruise on separate altitude layers.
%   Mission per drone: HOME -> TAKEOFF -> WP1 -> WP2 -> CUSTOMER -> DELIVERY ->
%   RETURN -> LAND.  Avoidance is active in every phase.
%
%   At the end of the run, TWO figures PER DRONE are drawn:
%     1) individual graphs: position X/Y/Z, velocity, acceleration,
%        roll/pitch/yaw angles, roll/pitch/yaw rates (each on its own plot)
%     2) combined graphs: position, velocity, acceleration, attitude, angular
%        rates (all three axes overlaid) + 3D path coloured by mission phase
%
%   Needs MATLAB R2024b, no toolboxes (LQR is solved here with a Schur method).

cfg = defaultConfig();
if mod(numel(varargin),2) ~= 0, error('Options must be name/value pairs.'); end
for ii = 1:2:numel(varargin)
    if ~isfield(cfg, varargin{ii}), warning('Unknown option "%s" ignored.', varargin{ii}); continue; end
    cfg.(varargin{ii}) = varargin{ii+1};
end
rng(cfg.seed); close all; clc;
N = cfg.nDrones; dt = cfg.dt;

%% ------------------------------------------------------------ build everything
P = droneParams(1, N, cfg);  M = makeMission(1, N, cfg);
for i = 2:N, P(i) = droneParams(i, N, cfg); M(i) = makeMission(i, N, cfg); end
world = makeWorld(cfg, M);
[bld, trees] = makeScene(cfg, M, world);
env  = makeObstacles(cfg, M, world);
nEnv = numel(env);  env0 = env;
zeroOff = struct('name','NONE','z',NaN,'lat',0,'along',0,'speedScale',1);
printParams(P);

D = initDrone(P(1), M(1), nEnv+N-1, zeroOff);
for i = 2:N, D(i) = initDrone(P(i), M(i), nEnv+N-1, zeroOff); end

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
    L(i).near = nan(1,nRec); L(i).ttc = nan(1,nRec);
end
G.t = zeros(1,nRec); G.obsP = zeros(3,nEnv,nRec); G.obsV = zeros(3,nEnv,nRec);
G.minDD = nan(1,nRec); G.minDO = nan(1,nRec);
r = 0; minDD = inf; minDDt = 0; minDO = inf; minDOt = 0;

fprintf('\n=== Swarm of %d delivery drones (LQR) ===\n', N);
for i = 1:N, fprintf('[t=%6.2f s] UAV%d MISSION: %s\n', 0, i, D(i).segs(1).name); end

%% ------------------------------------------------------------ main loop
for it = 1:nSteps
    t = (it-1)*dt;
    env = stepObstacles(env, dt, t);
    for i = 1:N
        if D(i).landed, continue; end
        if mod(it-1, nReplan) == 0
            obsI = [env, swarmObstacles(D, P, i)];
            D(i) = replan(D(i), t, obsI, P(i), bld, zeroOff);
        end
        [wnd, D(i).gust] = windModel(t, D(i).gust, cfg, dt);
        D(i).wind = wnd;
        D(i) = stepDrone(D(i), P(i), t, wnd);
    end

    % ---- separation bookkeeping (surface to surface) -----------------------
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

    % ---- logging -------------------------------------------------------------
    isLast = all([D.landed]) || it == nSteps;
    if mod(it-1, recStride) == 0 || isLast
        r = r + 1;
        G.t(r) = t; G.minDD(r) = dd; G.minDO(r) = dobs;
        for j = 1:nEnv, G.obsP(:,j,r) = env(j).pos; G.obsV(:,j,r) = env(j).vel; end
        for i = 1:N
            L(i).p(:,r) = D(i).p; L(i).v(:,r) = D(i).v; L(i).ang(:,r) = D(i).ang;
            L(i).wm(:,r) = D(i).wm; L(i).T(r) = D(i).Ttot;
            L(i).W(r) = (P(i).mFrame + D(i).payload)*P(i).g;
            L(i).pref(:,r) = D(i).pref; L(i).carrot(:,r) = D(i).carrot;
            L(i).wind(:,r) = D(i).wind; L(i).seg(r) = D(i).k; L(i).pay(r) = D(i).payload;
            L(i).pkg(:,r) = D(i).pkgP; L(i).pkgOn(r) = D(i).released;
            if D(i).A.active, L(i).mode{r} = D(i).A.off.name; else, L(i).mode{r} = 'NONE'; end
            if ~isempty(D(i).A.det)
                L(i).near(r) = min([D(i).A.det.dist]);
                L(i).ttc(r)  = min([D(i).A.det.ttc]);
            end
        end
    end
    if isLast, break; end
end
nR = r;

%% ------------------------------------------------------------ summary
fprintf('\n=== Summary ===\n');
for i = 1:N
    if D(i).landed, fprintf('UAV%d: mission complete, landed.  ', i);
    else, fprintf('UAV%d: NOT finished (last phase: %s).  ', i, D(i).segs(D(i).k).name); end
    sp = sqrt(sum(L(i).v(:,1:nR).^2,1));
    fprintf('avoid maneuvers %d | max alt %.1f m | max speed %.1f m/s | T/W %.2f..%.2f | motors %.0f..%.0f rad/s (limit %.0f)\n', ...
        D(i).nAvoid, max(L(i).p(3,1:nR)), max(sp), min(L(i).T(2:nR)./L(i).W(2:nR)), ...
        max(L(i).T(1:nR)./L(i).W(1:nR)), min(min(L(i).wm(:,1:nR))), max(max(L(i).wm(:,1:nR))), P(i).wMax);
end
fprintf('Total flight time        : %.1f s\n', G.t(nR));
fprintf('Min drone-drone clearance: %.2f m (surface to surface) at t = %.1f s\n', minDD, minDDt);
fprintf('Min drone-obstacle clear.: %.2f m (surface to surface) at t = %.1f s\n\n', minDO, minDOt);

out.L = L; out.G = G; out.P = P; out.M = M; out.cfg = cfg; out.nR = nR;

% ---- individual + combined graphs for every drone ---------------------------------
plotSixDOF(L, G, nR, M);

if cfg.animate
    animateSwarm(L, G, nR, P, M, cfg, bld, trees, env0, world);
end
end

%% ======================================================================
%%  CONFIGURATION  (the only place numbers live; every field is overridable)
%% ======================================================================
function cfg = defaultConfig()
% ---- run control
cfg.nDrones = 3;  cfg.seed = 7;  cfg.tMax = 300;  cfg.dt = 0.005;
cfg.recDt = 0.025;  cfg.replanDt = 0.25;
cfg.animate = true;  cfg.animStride = 4;  cfg.cam = 'follow';  cfg.visScale = 3.5;
cfg.camOffset = [-30; -32; 16];  cfg.camTargetOffset = [8; 3; 0];  cfg.camSmooth = 0.1;
% ---- environment
cfg.g = 9.81;  cfg.rho = 1.225;
cfg.windMean = 3.5;  cfg.windVar = 1.5;  cfg.windVarFreq = 0.07;
cfg.windDir = 0.5;   cfg.windDirVar = 0.6;  cfg.windDirFreq = 0.045;  cfg.windDirPhase = 1;
cfg.gustTau = 2.0;   cfg.gustSigma = 1.0;   cfg.gustVertFrac = 0.3;
% ---- airframe (nominal; each drone gets +/- cfg.spread of these)
cfg.mFrame = 1.5;  cfg.mPayload = 0.5;  cfg.motorMassFrac = 0.08;
cfg.armLength = 0.25;  cfg.coreRadius = 0.12;  cfg.rotorRadius = 0.17;
cfg.kf = 1e-5;  cfg.km = 1.6e-7;  cfg.twr = 2.5;  cfg.wMinFrac = 0.15;  cfg.tauM = 0.03;
cfg.dragCoef = 1.1;  cfg.frontalArea = 0.15;  cfg.spread = 0.05;
cfg.pkgSize = [0.22 0.22 0.14];
% ---- flight envelope (acceleration limits are derived from these)
cfg.maxTiltDeg = 25;  cfg.yawRateDeg = 60;
cfg.tiltUse = 0.9;  cfg.thrustUse = 0.4;  cfg.descentUse = 0.5;
cfg.vMaxH = 6;  cfg.vUp = 3;  cfg.vDown = 2;  cfg.aMaxH = 3;  cfg.aMaxV = 2;
cfg.kRef = 0.7;  cfg.kRefZ = 0.9;  cfg.maxLead = 6;  cfg.look = 12;
cfg.touchV0 = 0.5;  cfg.touchGain = 0.6;  cfg.yawMinSpeed = 1.5;
% ---- LQR design (Bryson's rule: weights = 1/(allowed deviation)^2)
cfg.lqr.posTolXY = 1.5;  cfg.lqr.posTolZ = 1.0;  cfg.lqr.velTol = 2.0;
cfg.lqr.intTol = 10;     cfg.lqr.intLimFrac = 0.5;
cfg.lqr.angTolDeg = 5;   cfg.lqr.rateTol = 1.5;   cfg.lqr.alphaFrac = 0.08;
cfg.lqr.yawTolDeg = 15;  cfg.lqr.yawRateTol = 1.0; cfg.lqr.alphaFracYaw = 0.3;
% ---- mission geometry (per drone, generated from these)
cfg.routeLength = 210;  cfg.homeSpacing = 10;  cfg.fanDeg = 12;
cfg.cruiseAlt = 10;  cfg.layerGap = 4;  cfg.deliveryAlt = 3;
cfg.wp1Frac = 0.33;  cfg.wp1Lat = 0.09;  cfg.wp2Frac = 0.66;  cfg.wp2Lat = -0.13;
% ---- mission supervisor
cfg.wpRadius = 4;  cfg.custRadius = 1.5;  cfg.custSpeed = 0.7;  cfg.custZTol = 0.6;
cfg.altTol = 0.5;  cfg.homeRadius = 3;  cfg.landZ = 0.12;
cfg.releaseDelay = 2.0;  cfg.deliveryDwell = 4.5;  cfg.releaseDrop = 0.4;  cfg.pkgDrag = 0.3;
% ---- radar / avoidance
cfg.radarRange = 40;  cfg.Th = 8;  cfg.dtPred = 0.1;  cfg.zMin = 2;  cfg.zMax = 45;
cfg.margin = 1.5;  cfg.speedGain = 0.15;  cfg.buf = 0.5;  cfg.releaseTime = 1.5;
cfg.cost = struct('wLat',1.8,'wAlong',1.2,'wUp',1.0,'wDown',2.5,'emerg',6,'hyst',0.7);
cfg.cand = struct('reserve',0.7,'minLat',1.0,'mixFrac',0.75,'fwdAlong',12,'fwdSpeed',1.35, ...
                  'backAlong',-8,'emergUp',3,'emergLat',0.5,'emergAlong',-4);
% ---- scene
cfg.nBuildings = 8;  cfg.nTrees = 40;  cfg.bldWidth = [8 20];  cfg.bldHeight = [6 26];
cfg.treeHeight = [4 8];  cfg.routeClear = 14;  cfg.bldMargin = 3;  cfg.bldZMargin = 2;  cfg.worldPad = 40;
% ---- dynamic obstacles (birds, kites, foreign drones, balloons)
cfg.nObstacles = 6;  cfg.obsTypes = {'bird','kite','drone','balloon'};
cfg.obsAlt = [4 16];  cfg.obsLateral = 40;
cfg.obsSpec.bird    = struct('r',0.5,'speed',4.0,'turn',0.015,'vz',0.3);
cfg.obsSpec.kite    = struct('r',1.0,'speed',0.7,'turn',0.0,  'vz',0.0);
cfg.obsSpec.drone   = struct('r',0.8,'speed',3.4,'turn',0.01, 'vz',0.0);
cfg.obsSpec.balloon = struct('r',1.3,'speed',0.0,'turn',0.0,  'vz',0.0);
end

%% ======================================================================
%%  DRONE PARAMETERS + LQR DESIGN
%% ======================================================================
function P = droneParams(i, N, cfg) %#ok<INUSD>
rs = RandStream('mt19937ar','Seed',cfg.seed*1000 + i);
sp = @() 1 + cfg.spread*(2*rand(rs) - 1);              % unit-to-unit variation
P.id = i;  P.g = cfg.g;
% --- mass & geometry
P.mFrame = cfg.mFrame*sp();  P.mPayload = cfg.mPayload*sp();
P.mMotor = cfg.motorMassFrac*P.mFrame;                  % each of 4 motors
P.L = cfg.armLength*sp();  P.d = P.L/sqrt(2);  P.rCore = cfg.coreRadius*sp();
P.rBody = P.L + cfg.rotorRadius;  P.rDrone = P.rBody;   % collision radius
mc = P.mFrame - 4*P.mMotor;  Jc = 0.4*mc*P.rCore^2;     % core = solid sphere
P.J = diag([Jc + 4*P.mMotor*P.d^2, Jc + 4*P.mMotor*P.d^2, Jc + 4*P.mMotor*P.L^2]);
% --- rotors / motors
P.kf = cfg.kf*sp();  P.km = cfg.km*sp();  P.c = P.km/P.kf;
P.mDesign = P.mFrame + P.mPayload;
P.wHover = sqrt(P.mDesign*P.g/(4*P.kf));
P.twr = cfg.twr*sp();
P.wMax = sqrt(P.twr)*P.wHover;  P.wMin = cfg.wMinFrac*P.wHover;
P.Tmax = 4*P.kf*P.wMax^2;  P.tauM = cfg.tauM*sp();
d = P.d;  c = P.c;
P.Amix = [1 1 1 1; -d d d -d; -d -d d d; -c c -c c];    % [T;tx;ty;tz] = Amix*F
P.invA = inv(P.Amix);
P.mp = d*[1 -1; 1 1; -1 1; -1 -1];                      % motor x,y positions (matches Amix)
% --- aerodynamics
P.cd = 0.5*cfg.rho*cfg.dragCoef*cfg.frontalArea;
% --- limits derived from physics
P.maxTilt = deg2rad(cfg.maxTiltDeg);  P.yawRate = deg2rad(cfg.yawRateDeg);
P.accH = cfg.tiltUse*P.g*tan(P.maxTilt);
P.accUp = cfg.thrustUse*(P.twr - 1)*P.g;
P.accDown = -cfg.descentUse*P.g;
% --- pass-through settings
f = {'vMaxH','vUp','vDown','aMaxH','aMaxV','kRef','kRefZ','maxLead','look','touchV0','touchGain', ...
     'yawMinSpeed','radarRange','Th','dtPred','zMin','zMax','margin','speedGain','buf','releaseTime', ...
     'replanDt','wpRadius','custRadius','custSpeed','custZTol','altTol','homeRadius','landZ', ...
     'releaseDelay','deliveryDwell','releaseDrop','pkgDrag','cost','cand','bldMargin','bldZMargin','dt'};
for q = 1:numel(f), P.(f{q}) = cfg.(f{q}); end
% --- torque authority -> max angular accelerations
Fh = P.mDesign*P.g/4;  Fmax = P.kf*P.wMax^2;  Fmin = P.kf*P.wMin^2;
dF = min(Fmax - Fh, Fh - Fmin);
P.tauMax = [4*P.d*dF; 4*P.d*dF; 4*P.c*dF];
P.alphaMax = P.tauMax./diag(P.J);
% --- POSITION LQR: per axis x = [e; ev; int(e)], xdot = A x + B a_cmd
q = cfg.lqr;
aXY = P.accH;  aZ = min(P.accUp, -P.accDown);
kxy = lqrPos(q.posTolXY, q.velTol, q.intTol, aXY);
kz  = lqrPos(q.posTolZ,  q.velTol, q.intTol, aZ);
P.Kpos = [kxy; kxy; kz];                                % rows: [Kp Kv Ki] for X, Y, Z
P.IntLim = q.intLimFrac*q.intTol*ones(3,1);
% --- ATTITUDE LQR: per axis x = [angle err; rate], u = angular acceleration
angTol  = [deg2rad(q.angTolDeg); deg2rad(q.angTolDeg); deg2rad(q.yawTolDeg)];
rateTol = [q.rateTol; q.rateTol; q.yawRateTol];
frac    = [q.alphaFrac; q.alphaFrac; q.alphaFracYaw];
P.Katt = zeros(3,2);
for a = 1:3
    Q = diag([1/angTol(a)^2, 1/rateTol(a)^2]);  R = 1/(frac(a)*P.alphaMax(a))^2;
    P.Katt(a,:) = lqrGain([0 1; 0 0], [0; 1], Q, R);    % rows: roll, pitch, yaw -> [Kang Krate]
end
end

function K = lqrPos(posTol, velTol, intTol, aMax)
A = [0 1 0; 0 0 0; 1 0 0];  B = [0; 1; 0];
Q = diag(1./[posTol velTol intTol].^2);  R = 1/aMax^2;
K = lqrGain(A, B, Q, R);
end

function K = lqrGain(A, B, Q, R)
% Continuous-time LQR via ordered real Schur form of the Hamiltonian (no toolbox).
n = size(A,1);
H = [A, -B*(R\B'); -Q, -A'];
[U, T] = schur(H, 'real');
[U, ~] = ordschur(U, T, 'lhp');
U1 = U(1:n,1:n);  U2 = U(n+1:end,1:n);
Pm = U2/U1;  Pm = (Pm + Pm')/2;
K = R\(B'*Pm);
if any(real(eig(A - B*K)) >= 0), error('LQR design failed: closed loop not stable (check tolerances).'); end
end

function printParams(P)
rows = {
 'frame mass [kg]',                 @(p) p.mFrame
 'payload mass [kg]',               @(p) p.mPayload
 'motor mass, each [kg]',           @(p) p.mMotor
 'arm length L [m]',                @(p) p.L
 'collision radius [m]',            @(p) p.rBody
 'Jxx = Jyy [kg m^2]',              @(p) p.J(1,1)
 'Jzz [kg m^2]',                    @(p) p.J(3,3)
 'thrust coeff kf [N s^2]',         @(p) p.kf
 'torque coeff km [N m s^2]',       @(p) p.km
 'yaw ratio c = km/kf [m]',         @(p) p.c
 'hover motor speed [rad/s]',       @(p) p.wHover
 'min motor speed [rad/s]',         @(p) p.wMin
 'max motor speed [rad/s]',         @(p) p.wMax
 'max total thrust [N]',            @(p) p.Tmax
 'thrust/weight (loaded)',          @(p) p.twr
 'motor time const [s]',            @(p) p.tauM
 'drag coeff cd [N s^2/m^2]',       @(p) p.cd
 'max tilt [deg]',                  @(p) rad2deg(p.maxTilt)
 'max horiz accel [m/s^2]',         @(p) p.accH
 'max climb accel [m/s^2]',         @(p) p.accUp
 'max descent accel [m/s^2]',       @(p) p.accDown
 'max roll/pitch torque [N m]',     @(p) p.tauMax(1)
 'max yaw torque [N m]',            @(p) p.tauMax(3)
 'LQR pos XY  Kp [1/s^2]',          @(p) p.Kpos(1,1)
 'LQR pos XY  Kv [1/s]',            @(p) p.Kpos(1,2)
 'LQR pos XY  Ki [1/s^3]',          @(p) p.Kpos(1,3)
 'LQR pos Z   Kp',                  @(p) p.Kpos(3,1)
 'LQR pos Z   Kv',                  @(p) p.Kpos(3,2)
 'LQR pos Z   Ki',                  @(p) p.Kpos(3,3)
 'LQR roll/pitch K_angle',          @(p) p.Katt(1,1)
 'LQR roll/pitch K_rate',           @(p) p.Katt(1,2)
 'LQR yaw K_angle',                 @(p) p.Katt(3,1)
 'LQR yaw K_rate',                  @(p) p.Katt(3,2) };
fprintf('\n=== Drone parameters (derived from config, one column per drone) ===\n');
fprintf('%-30s', 'parameter');  for i = 1:numel(P), fprintf('%12s', sprintf('UAV%d', i)); end;  fprintf('\n');
for q = 1:size(rows,1)
    fprintf('%-30s', rows{q,1});
    for i = 1:numel(P), fprintf('%12.4g', rows{q,2}(P(i))); end
    fprintf('\n');
end
end

%% ======================================================================
%%  MISSION / WORLD
%% ======================================================================
function M = makeMission(i, N, cfg)
slot = i - (N+1)/2;                                      % centred index (swarm fans out)
home = [0; slot*cfg.homeSpacing; 0];
ang  = deg2rad(slot*cfg.fanDeg);
cruise = cfg.cruiseAlt + (i-1)*cfg.layerGap;             % own altitude layer
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

function world = makeWorld(cfg, M)
pts = [M.poly];
world.lo = min(pts,[],2) - cfg.worldPad;  world.hi = max(pts,[],2) + cfg.worldPad;
end

function dmin = distToRoutes(q, M)
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

function [bld, trees] = makeScene(cfg, M, world)
span = world.hi - world.lo;
bld = zeros(0,5);  tries = 0;
while size(bld,1) < cfg.nBuildings && tries < 5000
    tries = tries + 1;
    w = cfg.bldWidth(1) + diff(cfg.bldWidth)*rand;  dp = cfg.bldWidth(1) + diff(cfg.bldWidth)*rand;
    h = cfg.bldHeight(1) + diff(cfg.bldHeight)*rand;
    cxy = world.lo + span.*rand(2,1);
    if distToRoutes(cxy, M) < hypot(w,dp)/2 + cfg.routeClear, continue; end
    bld(end+1,:) = [cxy' w dp h]; %#ok<AGROW>
end
trees = zeros(0,3);  tries = 0;
while size(trees,1) < cfg.nTrees && tries < 5000
    tries = tries + 1;
    cxy = world.lo + span.*rand(2,1);
    if distToRoutes(cxy, M) < cfg.routeClear, continue; end
    if inBuilding([cxy;0], bld, cfg.bldMargin, cfg.bldZMargin), continue; end
    trees(end+1,:) = [cxy' cfg.treeHeight(1) + diff(cfg.treeHeight)*rand]; %#ok<AGROW>
end
end

function obs = makeObstacles(cfg, M, world)
nO = cfg.nObstacles;  nT = numel(cfg.obsTypes);
for n = 1:nO
    type = cfg.obsTypes{mod(n-1, nT) + 1};  sp = cfg.obsSpec.(type);
    m = M(randi(numel(M)));
    dxy = m.cust(1:2) - m.home(1:2);  u = dxy/norm(dxy);  perp = [-u(2); u(1)];
    pxy = m.home(1:2) + rand*dxy + perp*(rand-0.5)*cfg.obsLateral;
    z = cfg.obsAlt(1) + diff(cfg.obsAlt)*rand;
    hdg = 2*pi*rand;  vel = sp.speed*[cos(hdg); sin(hdg); 0];
    lo = [world.lo; cfg.zMin + 0.5];  hi = [world.hi; cfg.zMax - 5];
    obs(n) = mk(type, [pxy; z], vel, sp.r, lo, hi, sp.turn, sp.vz, type); %#ok<AGROW>
end
end

function o = mk(type, pos, vel, r, lo, hi, turn, vzAmp, label)
o.type = type; o.label = label; o.pos = pos(:); o.vel = vel(:); o.v0 = vel(:);
o.speed = norm(vel(1:2)); o.r = r; o.lo = lo(:); o.hi = hi(:);
o.turn = turn; o.vzAmp = vzAmp; o.phase = 2*pi*rand;
end

function lst = swarmObstacles(D, P, i)
% Other swarm members as moving obstacles for drone i.
lst = [];
for j = 1:numel(D)
    if j == i, continue; end
    o = mk('swarm', D(j).p, D(j).v, P(j).rBody, zeros(3,1), zeros(3,1), 0, 0, sprintf('UAV%d', j));
    lst = [lst, o]; %#ok<AGROW>
end
end

function obs = stepObstacles(obs, dt, t)
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
    for ax = 1:3                                              % reflect inside its box
        if o.pos(ax) < o.lo(ax), o.pos(ax) = o.lo(ax); o.vel(ax) = abs(o.vel(ax)); o.v0(ax) = abs(o.v0(ax));
        elseif o.pos(ax) > o.hi(ax), o.pos(ax) = o.hi(ax); o.vel(ax) = -abs(o.vel(ax)); o.v0(ax) = -abs(o.v0(ax)); end
    end
    obs(i) = o;
end
end

function [wind, gust] = windModel(t, gust, cfg, dt)
Wm = cfg.windMean + cfg.windVar*sin(cfg.windVarFreq*t);
th = cfg.windDir + cfg.windDirVar*sin(cfg.windDirFreq*t + cfg.windDirPhase);
a  = exp(-dt/cfg.gustTau);
gust = a*gust + sqrt(1-a^2)*cfg.gustSigma*[randn; randn; cfg.gustVertFrac*randn];
wind = Wm*[cos(th); sin(th); 0] + gust;
end

%% ======================================================================
%%  ONE DRONE: STATE, CONTROL (LQR), DYNAMICS
%% ======================================================================
function d = initDrone(P, M, nO, zeroOff)
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

function d = replan(d, t, obs, P, bld, zeroOff)
d.A = avoidanceUpdate(d.A, t, d.p, d.v, d.segs(d.k), obs, P, bld, zeroOff);
if d.A.active && ~strcmp(d.A.off.name, d.lastName), d.nAvoid = d.nAvoid + 1; end
if d.A.active, d.lastName = d.A.off.name; else, d.lastName = 'NONE'; end
end

function d = stepDrone(d, P, t, wind)
dt = P.dt;  tag = sprintf('UAV%d', P.id);
sg = d.segs(d.k);
mTot = P.mFrame + d.payload;

% ---- mission supervisor ------------------------------------------------------
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

% ---- new 3D target + smooth reference ---------------------------------------------
off = d.A.off;
d.carrot = carrotTarget(d.p, sg, off, P);
[d.pref, d.vref] = refStep(d.pref, d.vref, d.carrot, off.speedScale, P, dt);
lead = d.pref - d.p;  nl = norm(lead);
if nl > P.maxLead, d.pref = d.p + lead*P.maxLead/nl; end

% ---- position LQR:  a_cmd = -K [p-pref ; v-vref ; int(p-pref)] ---------------------------
ep = d.p - d.pref;  evl = d.v - d.vref;
d.integ = max(min(d.integ + ep*dt, P.IntLim), -P.IntLim);          % anti-windup
a_cmd = -(P.Kpos(:,1).*ep + P.Kpos(:,2).*evl + P.Kpos(:,3).*d.integ);
ah = norm(a_cmd(1:2));
if ah > P.accH, a_cmd(1:2) = a_cmd(1:2)*P.accH/ah; end
a_cmd(3) = min(max(a_cmd(3), P.accDown), P.accUp);
d.a_cmd = a_cmd;

% ---- required force -> thrust along body Z ------------------------------------------------
Fw = mTot*(a_cmd + [0;0;P.g]);
R  = eul2R(d.ang);
Tcmd = min(max(dot(Fw, R(:,3)), 0), P.Tmax);

% ---- desired attitude from force direction (yaw follows travel) ---------------------------
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

% ---- attitude LQR:  alpha = Kang*e - Krate*w ;  tau = J*alpha + w x J*w ---------------------
alpha = P.Katt(:,1).*eang - P.Katt(:,2).*d.om;
tau   = P.J*alpha + cross(d.om, P.J*d.om);

% ---- motor mixer -> motor speeds with lag ---------------------------------------------------
Fm   = P.invA*[Tcmd; tau];
Fm   = min(max(Fm, P.kf*P.wMin^2), P.kf*P.wMax^2);
wcmd = sqrt(Fm/P.kf);
d.wm = d.wm + (wcmd - d.wm)/P.tauM*dt;
Fa   = P.kf*d.wm.^2;
Ttot = sum(Fa);
tauA = P.Amix(2:4,:)*Fa;

% ---- attitude dynamics ---------------------------------------------------------------------------
omdot = P.J \ (tauA - cross(d.om, P.J*d.om));
d.om = d.om + omdot*dt;
sp = sin(d.ang(1)); cp = cos(d.ang(1)); tt = tan(d.ang(2)); ct = cos(d.ang(2));
angdot = [d.om(1) + (d.om(2)*sp + d.om(3)*cp)*tt;
          d.om(2)*cp - d.om(3)*sp;
          (d.om(2)*sp + d.om(3)*cp)/ct];
d.ang = d.ang + angdot*dt;  d.ang(3) = wrapPi(d.ang(3));

% ---- translational dynamics -------------------------------------------------------------------------
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

%% ======================================================================
%%  RADAR, PREDICTION, AVOIDANCE
%% ======================================================================
function det = radarScan(p, v, obs, P)
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
tag = sprintf('UAV%d', P.id);
det = radarScan(p, v, obs, P);
A.det = det;
nO = numel(obs);
inRange = false(1,nO);
if ~isempty(det), inRange([det.id]) = true; end
for i = find(inRange & ~A.seen)
    s = det([det.id] == i);
    fprintf(['[t=%6.2f s] %s RADAR  : %-7s #%d dist=%5.1f m rel=[%6.1f %6.1f %6.1f] relV=[%5.1f %5.1f %5.1f]' ...
             ' height=%5.1f m heading=%4.0f deg TTC=%s safety=%.1f m\n'], ...
        t, tag, s.type, i, s.dist, s.rel, s.relv, s.height, s.heading, ttcStr(s.ttc), s.Rs);
end
A.seen = inRange;
A.threat = false(1,nO);

if isempty(det)
    A = tryRelease(A, t, P, zeroOff, true);
    return
end

idx = [det.id];  Rs = [det.Rs];
ro0 = rolloutRisk(p, v, sg, zeroOff, obs, idx, Rs, P, bld);
if ro0.margin >= P.buf
    A = tryRelease(A, t, P, zeroOff, all([det.dist] > 1.5*Rs));
    return
end

A.clearTimer = 0;
A.threat(idx(ro0.per < P.buf)) = true;
needNew = true;
if A.active
    roc = rolloutRisk(p, v, sg, A.off, obs, idx, Rs, P, bld);
    if roc.margin >= P.buf, needNew = false; end
end
if ~needNew, return; end

cands = buildCandidates(p, sg, ro0, Rs, P);
c0 = carrotTarget(p, sg, zeroOff, P);  z0 = c0(3);
bestCost = inf; best = []; bestM = -inf; fallback = [];
for q = 1:numel(cands)
    cand = cands{q};
    roq = rolloutRisk(p, v, sg, cand, obs, idx, Rs, P, bld);
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
for j = find(ro0.per < P.buf)
    fprintf('             threat %s #%d: closest approach in %.1f s, predicted miss %.1f m < required %.1f m\n', ...
        det(j).type, idx(j), ro0.tt(j), ro0.per(j)+Rs(j), Rs(j));
end
end

function A = tryRelease(A, t, P, zeroOff, farEnough)
if ~A.active, return; end
if farEnough, A.clearTimer = A.clearTimer + P.replanDt; else, A.clearTimer = 0; end
if A.clearTimer >= P.releaseTime
    A.active = false; A.off = zeroOff; A.clearTimer = 0; A.lastKey = '';
    fprintf('[t=%6.2f s] UAV%d AVOID  : obstacle cleared -> avoidance OFF, back to route / mission altitude\n', t, P.id);
end
end

function cands = buildCandidates(p, sg, ro, Rs, P)
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

function ro = rolloutRisk(p, v, sg, off, obs, idx, Rs, P, bld)
% Look-ahead of the drone's own reference path vs predicted obstacle positions:
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
    if inBuilding(pq, bld, P.bldMargin, P.bldZMargin), ro.hit = true; end
end
ro.margin = min(ro.per);
if ro.hit, ro.margin = min(ro.margin, -(P.buf + P.cand.reserve + P.margin)); end
end

function tf = inBuilding(q, bld, m, zm)
tf = false;
for b = 1:size(bld,1)
    if abs(q(1)-bld(b,1)) < bld(b,3)/2 + m && abs(q(2)-bld(b,2)) < bld(b,4)/2 + m && q(3) < bld(b,5) + zm
        tf = true; return;
    end
end
end

function c = carrotTarget(p, sg, off, P)
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

function [pr, vr] = refStep(pr, vr, carrot, vScale, P, dt)
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
%%  6-DOF PLOTS: individual graphs + combined graphs (two figures per drone)
%% ======================================================================
function plotSixDOF(L, G, nR, M)
N = numel(L);
t = G.t(1:nR);
col = {[0.85 0.1 0.1],[0.1 0.6 0.1],[0.1 0.3 0.9]};    % colour per axis (X/roll, Y/pitch, Z/yaw)
for n = 1:N
    % ---------------- signals ----------------
    pos = L(n).p(:,1:nR);
    vel = L(n).v(:,1:nR);
    ang = rad2deg(L(n).ang(:,1:nR));                   % [roll; pitch; yaw] in deg
    acc = zeros(3,nR);  rate = zeros(3,nR);  aacc = zeros(3,nR);
    for i = 1:3
        acc(i,:)  = movmean(gradient(vel(i,:), t), 9);
        rate(i,:) = movmean(rad2deg(gradient(unwrap(deg2rad(ang(i,:))), t)), 9);
        aacc(i,:) = movmean(gradient(rate(i,:), t), 15);      % angular acceleration [deg/s^2]
    end
    spd = sqrt(sum(vel.^2,1));
    seg = L(n).seg(1:nR);
    chg = find(diff(seg) ~= 0) + 1;
    tPh = t(chg);  kPh = seg(chg);

    %% ===== FIGURE 1: every signal on its own graph =====
    f1 = figure('Color','w','Position',[30+25*(n-1) 30+25*(n-1) 1500 1100], ...
                'Name',sprintf('UAV%d individual graphs',n),'NumberTitle','off');
    tl = tiledlayout(f1,6,3,'TileSpacing','compact','Padding','compact');
    title(tl,sprintf('UAV%d - individual graphs',n),'FontWeight','bold');

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
    f2 = figure('Color','w','Position',[70+25*(n-1) 70+25*(n-1) 1500 850], ...
                'Name',sprintf('UAV%d combined graphs',n),'NumberTitle','off');
    tl2 = tiledlayout(f2,2,4,'TileSpacing','compact','Padding','compact');
    title(tl2,sprintf('UAV%d - combined graphs',n),'FontWeight','bold');

    ax = nexttile(tl2); hold(ax,'on'); grid(ax,'on');
    for i = 1:3, plot(ax,t,pos(i,:),'Color',col{i},'LineWidth',1.4); end
    title(ax,'Position (X, Y, Z)'); xlabel(ax,'Time [s]'); ylabel(ax,'m');
    legend(ax,'X','Y','Z','Location','best'); addPhaseLines(ax,t,tPh);
    labelPhases(ax, tPh, kPh, M(n));

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
    plot3(ax,M(n).W3(1,:),M(n).W3(2,:),M(n).W3(3,:),'k:');
    title(ax,'3D path (colour = mission phase)'); xlabel(ax,'X [m]'); ylabel(ax,'Y [m]'); zlabel(ax,'Z [m]');
    view(ax,3); axis(ax,'equal'); colormap(ax,'turbo'); colorbar(ax);
end
end

function addPhaseLines(ax, t, tPh)
for q = 1:numel(tPh)
    xline(ax, tPh(q), ':', 'Color',[0.45 0.45 0.45], 'HandleVisibility','off');
end
xlim(ax,[t(1) max(t(end), t(1)+eps)]);
end

function labelPhases(ax, tPh, kPh, Mn)
yl = ylim(ax);
for q = 1:numel(tPh)
    text(ax, tPh(q), yl(2), [' ' Mn.segs(kPh(q)).name], 'Rotation',90, 'FontSize',7, ...
        'Color',[0.3 0.3 0.3], 'VerticalAlignment','top', 'HorizontalAlignment','right');
end
end

%% ======================================================================
%%  3D VISUALIZATION
%% ======================================================================
function animateSwarm(L, G, nR, P, M, cfg, bld, trees, env0, world)
N = numel(P);  nO = numel(env0);  col = lines(max(N,7));
zTop = max(30, cfg.cruiseAlt + N*cfg.layerGap + 15);
fig = figure('Color',[0.08 0.08 0.10],'Position',[40 40 1560 880],'Name','LQR delivery swarm','NumberTitle','off');
ax = axes('Parent',fig,'Position',[0.01 0.02 0.64 0.96]); hold(ax,'on');
set(ax,'Color',[0.62 0.78 0.95],'XColor','none','YColor','none','ZColor','none','Clipping','off');
axis(ax,[world.lo(1) world.hi(1) world.lo(2) world.hi(2) 0 zTop]); daspect(ax,[1 1 1]); camproj(ax,'perspective');

% ground
gx = [world.lo(1) world.hi(1) world.hi(1) world.lo(1)]; gy = [world.lo(2) world.lo(2) world.hi(2) world.hi(2)];
patch(ax,'XData',gx,'YData',gy,'ZData',zeros(1,4),'FaceColor',[0.36 0.58 0.32],'EdgeColor','none');
th = linspace(0,2*pi,30);
% buildings / trees
bcol = [0.65 0.65 0.70; 0.72 0.60 0.55; 0.55 0.62 0.72; 0.70 0.70 0.60];
for b = 1:size(bld,1)
    drawBox(ax,[bld(b,1) bld(b,2) bld(b,5)/2],[bld(b,3) bld(b,4) bld(b,5)],bcol(mod(b-1,4)+1,:),1);
end
[Xc,Yc,Zc] = cylinder([1 0],10);  [Xt,Yt,Zt] = cylinder(1,8);
for i = 1:size(trees,1)
    tx = trees(i,1); ty = trees(i,2); h = trees(i,3);
    surf(ax, tx+0.25*Xt, ty+0.25*Yt, 0.4*h*Zt,'FaceColor',[0.4 0.25 0.1],'EdgeColor','none');
    surf(ax, tx+0.28*h*Xc, ty+0.28*h*Yc, 0.3*h+0.7*h*Zc,'FaceColor',[0.1 0.42+0.1*mod(i,3)/2 0.15],'EdgeColor','none');
end
% routes, pads, drones
for i = 1:N
    W = M(i).W3;
    plot3(ax,W(1,:),W(2,:),W(3,:),':','Color',col(i,:),'LineWidth',1.2);
    h0 = M(i).home;  c0 = M(i).cust;
    patch(ax,'XData',h0(1)+cfg.homeRadius*cos(th),'YData',h0(2)+cfg.homeRadius*sin(th),'ZData',0.1*ones(size(th)),'FaceColor',col(i,:),'EdgeColor','k');
    text(ax,h0(1),h0(2),0.4,sprintf('H%d',i),'Color','k','FontWeight','bold','HorizontalAlignment','center');
    patch(ax,'XData',c0(1)+cfg.custRadius*2*cos(th),'YData',c0(2)+cfg.custRadius*2*sin(th),'ZData',0.1*ones(size(th)),'FaceColor',col(i,:),'EdgeColor','k');
    text(ax,c0(1),c0(2),c0(3)+3,sprintf('CUSTOMER %d',i),'Color','k','FontWeight','bold','HorizontalAlignment','center');
    plot3(ax,W(1,3:4),W(2,3:4),W(3,3:4),'o','MarkerSize',7,'MarkerFaceColor',col(i,:),'MarkerEdgeColor','k');
end
light(ax,'Position',[world.lo(1) world.lo(2) 200],'Style','local'); lighting(ax,'gouraud');

for i = 1:N
    dr(i) = buildDrone(ax, P(i), col(i,:), cfg); %#ok<AGROW>
    hTr(i) = plot3(ax,nan,nan,nan,'-','Color',col(i,:),'LineWidth',2); %#ok<AGROW>
    hAv(i) = plot3(ax,nan,nan,nan,'-','Color',[1 0.15 0.1],'LineWidth',3); %#ok<AGROW>
    hTg(i) = plot3(ax,nan,nan,nan,'d','MarkerFaceColor',col(i,:),'MarkerEdgeColor','k','MarkerSize',8); %#ok<AGROW>
    hSh(i) = plot3(ax,nan,nan,nan,'k.','MarkerSize',16); %#ok<AGROW>
    Hpkg(i) = hgtransform('Parent',ax,'Visible','off'); %#ok<AGROW>
    drawBox(Hpkg(i),[0 0 0],cfg.visScale*cfg.pkgSize,[0.75 0.55 0.3],1);
    pAv{i} = L(i).p(:,1:nR); pAv{i}(:, strcmp(L(i).mode(1:nR),'NONE')) = nan; %#ok<AGROW>
    spd{i} = sqrt(sum(L(i).v(:,1:nR).^2,1)); %#ok<AGROW>
end
ob = buildObstacles(ax, env0);
hHUD = annotation(fig,'textbox',[0.012 0.76 0.42 0.22],'String','','Color','w','BackgroundColor',[0 0 0], ...
    'FaceAlpha',0.55,'EdgeColor','none','FontName','Consolas','FontSize',9,'Interpreter','none','FitBoxToText','off');

% telemetry panels
tEnd = G.t(nR);
pa = @(y0) axes('Parent',fig,'Position',[0.71 y0 0.275 0.175],'Color',[0.12 0.12 0.15], ...
    'XColor',[0.8 0.8 0.8],'YColor',[0.8 0.8 0.8],'GridColor',[0.5 0.5 0.5],'FontSize',8,'XLim',[0 tEnd]);
a1 = pa(0.79); hold(a1,'on'); grid(a1,'on'); title(a1,'Altitude per drone [m]','Color','w','FontSize',9);
a2 = pa(0.58); hold(a2,'on'); grid(a2,'on'); title(a2,'Ground speed per drone [m/s]','Color','w','FontSize',9);
a3 = pa(0.37); hold(a3,'on'); grid(a3,'on'); title(a3,'Thrust / weight per drone','Color','w','FontSize',9);
a4 = pa(0.16); hold(a4,'on'); grid(a4,'on'); title(a4,'Clearance: drone-drone (w) / drone-obstacle (m) [m]','Color','w','FontSize',9);
xlabel(a4,'time [s]','Color','w');
for i = 1:N
    h1(i) = plot(a1,nan,nan,'Color',col(i,:),'LineWidth',1.5); %#ok<AGROW>
    h2(i) = plot(a2,nan,nan,'Color',col(i,:),'LineWidth',1.5); %#ok<AGROW>
    h3(i) = plot(a3,nan,nan,'Color',col(i,:),'LineWidth',1.5); %#ok<AGROW>
end
h4a = plot(a4,nan,nan,'w','LineWidth',1.5);  h4b = plot(a4,nan,nan,'m','LineWidth',1.5);
ylim(a1,[0 zTop]);
ylim(a2,[0 max(1, max(cellfun(@max,spd))*1.1)]);
tw = cellfun(@(l) max(l.T(2:nR)./l.W(2:nR)), num2cell(L));  ylim(a3,[0 max(1.5, max(tw)*1.1)]);
yl = [min([G.minDD(1:nR) G.minDO(1:nR)]) 40];  ylim(a4,[min(0,yl(1)) yl(2)]);

phase = zeros(N,4); spinDir = [1 -1 1 -1];
camPos = [];
frames = unique([1:cfg.animStride:nR, nR]);
for i = frames
    if ~ishandle(fig), return; end
    cen = zeros(3,1);
    for q = 1:N
        pk = L(q).p(:,i);  Rk = eul2R(L(q).ang(:,i));  cen = cen + pk/N;
        set(dr(q).H,'Matrix', makehgtform('translate',pk')*[Rk zeros(3,1); 0 0 0 1]*makehgtform('scale',cfg.visScale));
        for j = 1:4
            phase(q,j) = phase(q,j) + spinDir(j)*L(q).wm(j,i)/P(q).wHover*0.9*cfg.animStride;
            set(dr(q).Hs(j),'Matrix',makehgtform('translate',[P(q).mp(j,:) 0.085])*makehgtform('zrotate',phase(q,j)));
        end
        set(dr(q).pkg,'Visible',ternary(L(q).pay(i) > 0,'on','off'));
        set(Hpkg(q),'Visible',ternary(L(q).pkgOn(i),'on','off'));
        if L(q).pkgOn(i), set(Hpkg(q),'Matrix',makehgtform('translate',L(q).pkg(:,i)')); end
        set(hTr(q),'XData',L(q).p(1,1:i),'YData',L(q).p(2,1:i),'ZData',L(q).p(3,1:i));
        set(hAv(q),'XData',pAv{q}(1,1:i),'YData',pAv{q}(2,1:i),'ZData',pAv{q}(3,1:i));
        set(hTg(q),'XData',L(q).carrot(1,i),'YData',L(q).carrot(2,i),'ZData',L(q).carrot(3,i));
        set(hSh(q),'XData',pk(1),'YData',pk(2),'ZData',0.1);
        set(h1(q),'XData',G.t(1:i),'YData',L(q).p(3,1:i));
        set(h2(q),'XData',G.t(1:i),'YData',spd{q}(1:i));
        set(h3(q),'XData',G.t(1:i),'YData',L(q).T(1:i)./max(L(q).W(1:i),eps));
    end
    set(h4a,'XData',G.t(1:i),'YData',G.minDD(1:i)); set(h4b,'XData',G.t(1:i),'YData',G.minDO(1:i));
    for j = 1:nO
        po = G.obsP(:,j,i); vo = G.obsV(:,j,i);
        yaw = atan2(vo(2),vo(1)); if norm(vo(1:2)) < 0.05, yaw = 0; end
        set(ob.H(j),'Matrix',makehgtform('translate',po')*makehgtform('zrotate',yaw));
        if ~isempty(ob.Hwl{j})
            fl = 0.7*sin(2*pi*1.8*G.t(i) + j);
            set(ob.Hwl{j},'Matrix',makehgtform('xrotate',fl)); set(ob.Hwr{j},'Matrix',makehgtform('xrotate',-fl));
        end
        set(ob.txt(j),'Position',po'+[0 0 ob.r(j)+2.2]);
    end
    hud = cell(N+2,1);  hud{1} = sprintf('t = %6.1f s', G.t(i));
    for q = 1:N
        vh = norm(L(q).v(1:2,i));
        hud{q+1} = sprintf('UAV%d %-19s AVD:%-13s z=%5.1f v=%4.1f T/W=%.2f', q, M(q).segs(L(q).seg(i)).name, ...
            L(q).mode{i}, L(q).p(3,i), vh, L(q).T(i)/max(L(q).W(i),eps));
    end
    hud{N+2} = sprintf('min clearance  drone-drone %.1f m | drone-obstacle %.1f m', G.minDD(i), G.minDO(i));
    set(hHUD,'String',hud);

    if strcmp(cfg.cam,'follow')
        desired = cen + cfg.camOffset;
        if isempty(camPos), camPos = desired; else, camPos = (1-cfg.camSmooth)*camPos + cfg.camSmooth*desired; end
        set(ax,'CameraPosition',camPos','CameraTarget',(cen+cfg.camTargetOffset)','CameraViewAngle',32,'CameraUpVector',[0 0 1]);
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

function dr = buildDrone(ax, P, col, cfg)
% Drawn in metres from this drone's own parameters (arm length, rotor radius, package size).
H = hgtransform('Parent',ax);  d = P.d;  rr = cfg.rotorRadius;
drawBox(H,[0 0 0],[1.7*d 1.1*d 0.07],[0.15 0.15 0.18],1);
drawBox(H,[0 0 0.05],[1.0*d 0.8*d 0.04],col,1);
drawBox(H,[0.9*d 0 0.02],[0.04 0.10 0.04],[0.95 0.1 0.1],1);               % nose marker
plot3([-d d],[-d d],[0 0],'Parent',H,'Color',[0.1 0.1 0.1],'LineWidth',4);
plot3([-d d],[d -d],[0 0],'Parent',H,'Color',[0.1 0.1 0.1],'LineWidth',4);
[Xm,Ym,Zm] = cylinder(0.035,12);
th = linspace(0,2*pi,24);
for j = 1:4
    surf(Xm+P.mp(j,1),Ym+P.mp(j,2),Zm*0.05+0.02,'Parent',H,'FaceColor',[0.25 0.25 0.25],'EdgeColor','none');
    Hs(j) = hgtransform('Parent',H,'Matrix',makehgtform('translate',[P.mp(j,:) 0.085])); %#ok<AGROW>
    patch('Parent',Hs(j),'XData',rr*cos(th),'YData',rr*sin(th),'ZData',0*th,'FaceColor',col,'FaceAlpha',0.18,'EdgeColor','none');
    patch('Parent',Hs(j),'XData',[-rr rr rr -rr],'YData',[-0.012 -0.012 0.012 0.012],'ZData',zeros(1,4),'FaceColor',col,'EdgeColor','none');
end
for sx = [-1 1]
    for sy = [-1 1]
        plot3([0.55*d*sx 0.75*d*sx],[0.4*d*sy 0.9*d*sy],[-0.03 -0.22],'Parent',H,'Color','k','LineWidth',1.5);
    end
end
dr.pkg = drawBox(H,[0 0 -0.12],cfg.pkgSize,[0.75 0.55 0.3],1);
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
    ob.txt(i) = text(ax,0,0,0,sprintf('%s #%d',o.type,i),'FontSize',8,'FontWeight','bold','HorizontalAlignment','center','Color',[0.1 0.1 0.1]);
    ob.r(i)   = o.r;
end
end
