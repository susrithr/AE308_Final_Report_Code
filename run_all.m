%% RUN_ALL.M — GROUP 18 CONTROL SYSTEMS PROJECT
% Non-collocated angular positioning of the torsion disc
clear;
clc; 
close all;

%% SUBMISSION SETTINGS
groupNo = 18; 
scriptDir = fileparts(mfilename('fullpath'));
if isempty(scriptDir), scriptDir = pwd; end
figsDir = fullfile(scriptDir,'figs');
if ~exist(figsDir,'dir'), mkdir(figsDir); end
resultsFile = fullfile(scriptDir,'results.txt');
if exist(resultsFile,'file'), delete(resultsFile); end
diary(resultsFile);
diary on;

s = tf('s');

%% PLANT MODEL
G = 14957.265/(s*(s + 2.2296)*(s^2 + 2.3456*s + 355.555));

r_deg = 30;
r = deg2rad(r_deg);
t = 0:0.001:10;

%% CONTROLLER DEFINITIONS

% Baseline proportional controller
Kp = 0.100;
C_P = Kp;
L_P = C_P*G;
T_P = feedback(L_P,1);

% Design 1: Ideal PID controller
Ti = 10;
Td = 0.15;
K_PID_low = 0.08;
K_PID_high = 0.25;

C_PID_low = K_PID_low*(1 + 1/(Ti*s) + Td*s);
C_PID_high = K_PID_high*(1 + 1/(Ti*s) + Td*s);

L_PID_low = C_PID_low*G;
L_PID_high = C_PID_high*G;

T_PID_low = feedback(L_PID_low,1);
T_PID_high = feedback(L_PID_high,1);

% Design 2: Phase-lead compensator

Kc = 0.12646;
Tlead = 0.61871;
alpha = 0.41767;

C_lead = Kc*(1 + Tlead*s)/(1 + alpha*Tlead*s);
L_lead = C_lead*G;
T_lead = feedback(L_lead,1);

%% SECTION 1 — PLANT MODELLING
% fig_pz: Pole-zero map of the open-loop plant
fig_pz = figure('Name','fig pz - Plant Pole-Zero Map');
pzmap(G);
grid on;
title(sprintf('%s — Group %d','Open-Loop Plant Pole-Zero Map',groupNo));

% fig_bode_plant: Bode plot of the plant
fig_bode_plant = figure('Name','fig bode plant - Plant Bode Plot');
bode(G);
grid on;
title(sprintf('%s — Group %d','Open-Loop Plant Bode Plot',groupNo));

%% SECTION 2 — ROUTH / ROOT LOCUS / NYQUIST

% fig_rl: Root locus of the uncompensated plant
fig_rl = figure('Name','fig rl - Plant Root Locus');
rlocus(G);
grid on;
title(sprintf('%s — Group %d','Root Locus of the Uncompensated Plant',groupNo));

% fig_nyq_P: Nyquist plot of the uncompensated plant
fig_nyq_P = figure('Name','fig nyq P - Plant Nyquist Plot');
nyquist(G);
grid on;
title(sprintf('%s — Group %d','Nyquist Plot of the Uncompensated Plant',groupNo));
xlim([-2.5 1.5]);
ylim([-2 2]);

%% SECTION 3 — SPECIFICATION TRANSLATION
% fig_region: Illustrative admissible pole region.

sigma_req = 1.5267;
zeta_req = 0.5594;

fig_region = figure('Name','fig region - Admissible Pole Region');
hold on; grid on; axis equal;
xlim([-12 1]);
ylim([-12 12]);
xline(-sigma_req,'r--','Re(s) = -1.5267','LineWidth',1.4);
theta = acos(zeta_req);
ymax = 12;
xline_dummy = []; 
yy = linspace(-ymax,ymax,500);
xx = -abs(yy)/tan(theta);
plot(xx,yy,'b--','LineWidth',1.4);
[xg,yg] = meshgrid(linspace(-12,-sigma_req,250), ...
                   linspace(-12,12,250));
zg = -xg./sqrt(xg.^2 + yg.^2);
admissible = zg >= zeta_req;
contourf(xg,yg,double(admissible),[1 1], ...
         'FaceColor',[0.80 0.90 1.00], ...
         'FaceAlpha',0.35,'LineStyle','none');
xline(0,'k-');
yline(0,'k-');
xlabel('Real Axis (s^{-1})');
ylabel('Imaginary Axis (rad/s)');
title(sprintf('%s — Group %d','Admissible Dominant Pole Region',groupNo));
legend('Settling-time boundary','Damping-ratio boundaries', ...
       'Admissible region','Location','best');
hold off;

%% SECTION 4 — BASELINE CONTROLLER
% fig_baseline: 30-degree step response for baseline P controller
[y_P, t_P] = step(T_P,t);
y_P_deg = rad2deg(r*y_P);

fig_baseline = figure('Name','fig baseline - Baseline P Step Response');
plot(t_P,y_P_deg,'LineWidth',1.5);
hold on;
yline(r_deg,'k--','30 degree reference');
grid on;
xlabel('Time (s)');
ylabel('Output angle (degree)');
title(sprintf('%s — Group %d','Baseline P Controller: 30 Degree Step Response',groupNo));
legend('Output','Reference','Location','best');
hold off;

% fig_impulse: Impulse response of baseline closed-loop system
fig_impulse = figure('Name','fig impulse - Baseline Impulse Response');
impulse(T_P);
grid on;
title(sprintf('%s — Group %d','Baseline P Controller: Closed-Loop Impulse Response',groupNo));

%% SECTION 5 — DESIGN 1 (IDEAL PID)
% fig_rl_d1: Root locus of the plant with the fixed PID zero structure.
C_PID_shape = 1 + 1/(Ti*s) + Td*s;

fig_rl_d1 = figure('Name','fig rl d1 - Ideal PID Root Locus');
rlocus(C_PID_shape*G);
grid on;
title(sprintf('%s — Group %d','Root Locus for Ideal PID Controller Structure',groupNo));

hold on;
p_low = pole(T_PID_low);
p_high = pole(T_PID_high);
plot(real(p_low),imag(p_low),'ro','MarkerSize',7,'LineWidth',1.3);
plot(real(p_high),imag(p_high),'ks','MarkerSize',7,'LineWidth',1.3);
legend('PID root locus','Poles at K = 0.08','Poles at K = 0.25', ...
       'Location','best');
hold off;

%% SECTION 6 — DESIGN 2 AND CONTROLLER COMPARISON
% fig_bode_designs: Bode plots of the lead and ideal PID loop designs
fig_bode_designs = figure('Name','fig bode designs - Controller Bode Comparison');
bodeplot(L_lead,L_PID_low,L_PID_high);
grid on;
legend('Lead compensator','Ideal PID K = 0.08','Ideal PID K = 0.25', ...
       'Location','best');
title(sprintf('%s — Group %d','Bode Plots of Design 1 and Design 2 Open-Loop Systems',groupNo));

% fig_cmp: Closed-loop step-response comparison of the controller designs
[y_lead_cmp,t_lead_cmp] = step(T_lead,t);
[y_low_cmp,t_low_cmp] = step(T_PID_low,t);
[y_high_cmp,t_high_cmp] = step(T_PID_high,t);

fig_cmp = figure('Name','fig cmp - Controller Comparison');
plot(t_P,y_P_deg,'LineWidth',1.2);
hold on;
plot(t_lead_cmp,rad2deg(r*y_lead_cmp),'LineWidth',1.5);
plot(t_low_cmp,rad2deg(r*y_low_cmp),'--','LineWidth',1.2);
plot(t_high_cmp,rad2deg(r*y_high_cmp),'-.','LineWidth',1.2);
yline(r_deg,'k:','30 degree reference');
grid on;
xlabel('Time (s)');
ylabel('Output angle (degree)');
title(sprintf('%s — Group %d','Closed-Loop Controller Comparison: 30 Degree Step',groupNo));
legend('Baseline P','Lead compensator','Ideal PID K = 0.08', ...
       'Ideal PID K = 0.25','Reference','Location','best');
hold off;

%% SECTION 7 — NOMINAL VALIDATION
% fig_val: Lead design nominal frequency-domain validation (Bode plot)
fig_val = figure('Name','fig val - Lead Design Frequency Validation');
margin(L_lead);
grid on;
title(sprintf('%s — Group %d','Lead Compensator: Gain and Phase Margin Validation',groupNo));

% fig_cl_poles: Closed-loop pole locations for the lead design
fig_cl_poles = figure('Name','fig cl poles - Lead Closed-Loop Poles');
pzmap(T_lead);
grid on;
title(sprintf('%s — Group %d','Lead Compensator: Closed-Loop Pole Locations',groupNo));

% fig_ST: Lead design 30-degree step response and settling-time limits
[y_lead,t_lead] = step(T_lead,t);
y_lead_deg = rad2deg(r*y_lead);
info_lead = stepinfo(T_lead);

fig_ST = figure('Name','fig ST - Lead Step and Settling Time');
plot(t_lead,y_lead_deg,'LineWidth',1.5);
hold on;
yline(r_deg,'k--','Final value');
yline(r_deg*1.02,'r:','+2% settling band');
yline(r_deg*0.98,'r:','-2% settling band');
xline(info_lead.SettlingTime,'m--', ...
      sprintf('Settling time = %.3f s',info_lead.SettlingTime));
grid on;
xlabel('Time (s)');
ylabel('Output angle (degree)');
title(sprintf('%s — Group %d','Lead Compensator: 30 Degree Step Response',groupNo));
legend('Output','Reference','+2% band','-2% band', ...
       'Settling time','Location','best');
hold off;

%% SECTION 8 — ILLUSTRATIVE ACTUATOR LIMIT
% fig_sat: Illustrative control signal and saturation limits.
Umax = 1;
e_lead = r*(1 - y_lead);
u_lead = lsim(C_lead,e_lead,t_lead);

fig_sat = figure('Name','fig sat - Illustrative Actuator Limit');
plot(t_lead,u_lead,'LineWidth',1.5);
hold on;
yline(Umax,'r--','Upper illustrative limit');
yline(-Umax,'r--','Lower illustrative limit');
grid on;
xlabel('Time (s)');
ylabel('Control signal u(t)');
title(sprintf('%s — Group %d','Lead Controller Control Signal with Illustrative Saturation Limits',groupNo));
legend('Control signal','Upper limit','Lower limit','Location','best');
hold off;

%% PRINT NUMERICAL VALIDATION RESULTS
[GM_lead,PM_lead,Wcg_lead,Wcp_lead] = margin(L_lead);

fprintf('\n===== LEAD COMPENSATOR VALIDATION =====\n');
fprintf('Phase margin       = %.4f deg\n',PM_lead);
fprintf('Gain margin        = %.4f dB\n',20*log10(GM_lead));
fprintf('Gain crossover     = %.4f rad/s\n',Wcg_lead);
fprintf('Phase crossover    = %.4f rad/s\n',Wcp_lead);
fprintf('Overshoot          = %.4f %%\n',info_lead.Overshoot);
fprintf('Settling time      = %.4f s\n',info_lead.SettlingTime);
fprintf('Rise time          = %.4f s\n',info_lead.RiseTime);
fprintf('Closed-loop poles:\n');
disp(pole(T_lead));

%% FIGURE INDEX
fprintf('\n===== FIGURE INDEX =====\n');
fprintf('Section 1: fig_pz, fig_bode_plant\n');
fprintf('Section 2: fig_rl, fig_nyq_P\n');
fprintf('Section 3: fig_region\n');
fprintf('Section 4: fig_baseline, fig_impulse\n');
fprintf('Section 5: fig_rl_d1\n');
fprintf('Section 6: fig_bode_designs, fig_cmp\n');
fprintf('Section 7: fig_val, fig_cl_poles, fig_ST\n');
fprintf('Section 8: fig_sat\n');


%% SAVE ALL FIGURES AS PDF FILES
% Every open figure is exported to the figs folder.
figHandles = findall(0,'Type','figure');
for k = 1:numel(figHandles)
    fig = figHandles(k);
    figName = get(fig,'Name');
    if isempty(figName)
        figName = sprintf('figure_%02d',k);
    end
    safeName = regexprep(figName,'[^a-zA-Z0-9_-]','_');
    pdfPath = fullfile(figsDir,sprintf('%02d_%s.pdf',k,safeName));
    try
        exportgraphics(fig,pdfPath,'ContentType','vector');
    catch
        % Fallback for older MATLAB releases
        set(fig,'PaperPositionMode','auto');
        print(fig,pdfPath,'-dpdf','-bestfit');
    end
end

fprintf('\nFigures exported to: %s\n',figsDir);
fprintf('Results written to: %s\n',resultsFile);
diary off;
