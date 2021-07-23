close all;

%% Parameters

% test data prefix
ds_prefix = 'res/';

% test data sufix
ds_sufix = 'rampa_diagonal_01';

% save flag
flag_save = true;

% picture position
fig_pos = [1000 200 800 700];


%% Preamble

% getting current time
t = datetime('now');
ds = datestr(t, 'yyyymmddTHHMMSS');

%% Treating variables

aux_log_time = [];
for i=1:length(log_time)
    aux_log_time(i) = log_time(1,i) + log_time(2,i) * 1e-9;
end

% differentiating
aux_log_time_diff = diff(aux_log_time);

% mounting time array
log_time_corr = 0;
for i=1:length(aux_log_time_diff)
   log_time_corr(end+1) = log_time_corr(end) + aux_log_time_diff(i);
end

% cmd pos
log_laj_cmd_pos_corr = [];
for i=1:length(log_laj_cmd_pos)
    log_laj_cmd_pos_corr(:,i) = rosi_correct_joints_signal(log_laj_cmd_pos(:,i));
end


%% Plotting

% error theta
f1 = figure;
f1.Position = fig_pos;

ax1 = subplot(3,1,1);
plot (log_time_corr, log_error_theta, '*k');
axis([log_time_corr(1), log_time_corr(end), min(abs(log_error_theta)), max(abs(log_error_theta))]);
title('Error angle - $\alpha_e$', 'Interpreter','latex');
%xlabel('Time [s]');
ylabel('Angle [rad]', 'Interpreter','latex');
set(gca, 'xticklabel','');
grid on;

% Command signal
ax2 = subplot(3,1,2)

plot(log_time_corr, log_laj_delta_sig(1,:), '*b');
hold on;
plot(log_time_corr, log_laj_delta_sig(2,:), '*g');
plot(log_time_corr, log_laj_delta_sig(3,:), '*m');
plot(log_time_corr, log_laj_delta_sig(4,:), '*r');

axis([log_time_corr(1), log_time_corr(end), min(min(log_laj_delta_sig)), max(max(log_laj_delta_sig))]);
title('Propellant command signal - $\delta_{\textrm{Pi}}$', 'Interpreter', 'latex');
%xlabel('Time [s]');
ylabel('Command signal', 'Interpreter', 'latex');
legend('$\delta_{P1}$','$\delta_{P2}$','$\delta_{P3}$','$\delta_{P4}$', 'Interpreter', 'latex');
set(gca, 'xticklabel','');
grid on;


% lever arms position
ax3 = subplot(3,1,3)

plot(log_time_corr, log_laj_pos(1,:), '-b');
hold on;
plot(log_time_corr, log_laj_pos(2,:), '-g');
plot(log_time_corr, log_laj_pos(3,:), '-m');
plot(log_time_corr, log_laj_pos(4,:), '-r');

plot(log_time_corr, log_laj_cmd_pos_corr(1,:),'*b');
plot(log_time_corr, log_laj_cmd_pos_corr(2,:),'*g');
plot(log_time_corr, log_laj_cmd_pos_corr(3,:),'*m');
plot(log_time_corr, log_laj_cmd_pos_corr(4,:),'*r');

plot([log_time_corr(1) log_time_corr(end)],[la_angle_min la_angle_min], '--k');
plot([log_time_corr(1) log_time_corr(end)],[la_angle_max la_angle_max], '--k');
hold off;

title('Lever Arms Position', 'Interpreter', 'latex');
xlabel('Time [s]', 'Interpreter', 'latex');
ylabel('Angular Position [rad]', 'Interpreter', 'latex');
legend('$q_1$','$q_2$','$q_3$','$q_4$',...
       '$u_{P1}$','$u_{P2}$','$u_{P3}$','$u_{P4}$',...
        '$q_\textrm{min}$','$q_\textrm{max}$', 'Interpreter', 'latex');
axis([log_time_corr(1), log_time_corr(end), 0, pi]);
grid on;


linkaxes([ax1,ax2,ax3],'x')


% Command set-point


% hold on;
% plot([log_time_corr(1) log_time_corr(end)],[la_angle_min la_angle_min], '--k');
% plot([log_time_corr(1) log_time_corr(end)],[la_angle_max la_angle_max], '--k');
% title('Angular set-point Position');
% xlabel('Time [s]');
% ylabel('Angular Position [radians]');
% legend('$u_{\textrm{P1}}$','$u_{\textrm{P2}}$','$u_{\textrm{P3}}$','$u_{\textrm{P4}}$', 'Interpreter', 'latex');
% axis([log_time_corr(1), log_time_corr(end), 0, pi]);
% grid on;



%% Saving data

if flag_save 
    
    % saving string
    ss = strcat(ds_prefix,ds,'_',ds_sufix);
    
    % saving data variables
    save(strcat(ss,'_data'), 'aux_log_time', 'log_time_corr', 'log_laj_pos', 'log_error_theta', 'log_laj_delta_sig', 'log_laj_cmd_pos', 'la_angle_min', 'la_angle_max');

    % saving figure
    saveas(f1, strcat(ss,'_fig'), 'fig');
    saveas(f1, strcat(ss,'_png'), 'png');
    saveas(f1, strcat(ss,'_eps'), 'eps');
    
end















