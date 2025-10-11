%% Task 2

data = load('task2PlotValues.mat');

fieldnames(data)
ds = data.data

current     = ds{1}.Values;
omega_dot   = ds{2}.Values;
omega_diff  = ds{3}.Values;
omega_simp  = ds{4}.Values;
omega_ss    = ds{5}.Values;
omega_tf    = ds{6}.Values;
%% Task3

data = load('task3PlotValues.mat');
fieldnames(data)
ds = data.data

current     = ds{1}.Values;
omega_dot   = ds{2}.Values;
omega_diff  = ds{3}.Values;
omega_simp  = ds{4}.Values;
omega_ss    = ds{5}.Values;
omega_tf    = ds{6}.Values;

%% Task4 Values

data = load('task4PlotValues.mat');
fieldnames(data)
ds = data.data

omega_dot   = ds{1}.Values;
omega_diff  = ds{2}.Values;
omega_simp  = ds{3}.Values;
omega_ss    = ds{4}.Values;
omega_tf    = ds{5}.Values;

%% Angular Velocity Plots

% Lista med färger och titlar för varje plot
colors = [1 0.4118 0.1608;
          1 0.0745 0.6510;
          0.1333 0.7098 0.4510;
          0.3922 0.8314 0.0745;
          0.0745, 0.6235, 1;
          0.7176, 0.2745, 1];

titles = {'Block Diagram Angular Velocity vs Time', ...
          'State-space model Angular Velocity vs Time', ...
          'Transfer Function Angular Velocity vs Time', ...
          'Simscape Angular Velocity vs Time', ...
          'Block Diagram Current vs Time', ...
          'Block Diagram Angular Acceleration vs Time'};

ylabelText = 'Angular Velocity [rad/s]';
xlabelText = 'Time [s]';

% Samla simuleringsdata i en cell-array
simData = {omega_diff, omega_ss, omega_tf, omega_simp, omega_dot, current};

%% Angular Velocity Plots

% Skapa separata figurer
for i = 1:4
    fig = figure('Name',titles{i}, 'Color', [1 1 1]);
    ax = axes('Parent',fig, ...
              'Color',[1 1 1], ...            % vit axelbakgrund
              'XColor',[0 0 0], ...           % svart x-tick
              'YColor',[0 0 0], ...           % svart y-tick
              'GridColor',[0.8 0.8 0.8], ...  % ljusgrå grid
              'GridAlpha',0.5, ...         % gör grid lite transparent
              'FontSize',14, ...
              'TickDir','out', ...
              'XGrid','on', 'YGrid','on');   % grid på
    plot(ax, simData{i}.time, simData{i}.Data(:,1), 'LineWidth',2, 'Color', colors(i,:),'Marker', 'o', 'MarkerSize',2, 'MarkerFaceColor', colors(i,:));
    xlabel(ax, xlabelText,'FontSize',16.4, 'Color', [0 0 0]);
    ylabel(ax, ylabelText,'FontSize',16.4, 'Color', [0 0 0]);
    title(ax, titles{i},'FontSize',18.4, 'Color', [0 0 0]);
    %legend(ax, 'Angular Velocity','TextColor',[0 0 0]);
    grid on
    set(gca,'FontSize',14,'TickDir','out','XGrid','on','YGrid','on');
    set(gcf, "Theme", "light");

    omega_max = max(simData{i}.Data(:,1))
    n_max = 60 * omega_max / 2 * pi

end


%% Current plot

fig = figure('Name',titles{5}, 'Color', [1 1 1]);
ax = axes('Parent',fig, ...
          'Color',[1 1 1], ...            % vit axelbakgrund
          'XColor',[0 0 0], ...           % svart x-tick
          'YColor',[0 0 0], ...           % svart y-tick
          'GridColor',[0.8 0.8 0.8], ...  % ljusgrå grid
          'GridAlpha',0.5, ...         % gör grid lite transparent
          'FontSize',14, ...
          'TickDir','out', ...
          'XGrid','on', 'YGrid','on');   % grid på
plot(ax, simData{6}.time, simData{6}.Data(:,1), 'LineWidth',2, 'Color', colors(6,:),'Marker', 'o', 'MarkerSize',2, 'MarkerFaceColor', colors(6,:));
xlabel(ax, xlabelText,'FontSize',16.4, 'Color', [0 0 0]);
ylabel(ax, 'Current [A]','FontSize',16.4, 'Color', [0 0 0]);
title(ax, titles{5},'FontSize',18.4, 'Color', [0 0 0]);
%legend(ax, 'Current','TextColor',[0 0 0]);
grid on
set(gca,'FontSize',14,'TickDir','out','XGrid','on','YGrid','on');
set(gcf, "Theme", "light");

torque_max = max(simData{6}.Data(:,1))*K_M

%% Angular acceleration plot

fig = figure('Name',titles{6}, 'Color', [1 1 1]);
ax = axes('Parent',fig, ...
          'Color',[1 1 1], ...            % vit axelbakgrund
          'XColor',[0 0 0], ...           % svart x-tick
          'YColor',[0 0 0], ...           % svart y-tick
          'GridColor',[0.8 0.8 0.8], ...  % ljusgrå grid
          'GridAlpha',0.5, ...         % gör grid lite transparent
          'FontSize',14, ...
          'TickDir','out', ...
          'XGrid','on', 'YGrid','on');   % grid på
plot(ax, simData{5}.time, simData{5}.Data(:,1), 'LineWidth',2, 'Color', colors(5,:),'Marker', 'o', 'MarkerSize',2, 'MarkerFaceColor', colors(5,:));
xlabel(ax, xlabelText,'FontSize',16.4, 'Color', [0 0 0]);
ylabel(ax, 'Angular Acceleration [rad/s^2]','FontSize',16.4, 'Color', [0 0 0]);
title(ax, titles{6},'FontSize',18.4, 'Color', [0 0 0]);
%legend(ax, 'Angular Velocity','TextColor',[0 0 0]);
grid on
set(gca,'FontSize',14,'TickDir','out','XGrid','on','YGrid','on');
set(gcf, "Theme", "light");

alpha_max = max(simData{5}.Data(:,1))


%% Mechanical time constant

tau = 0; 
index = 1;

while simData{2}.Data(index,1) <= (0.632 * omega_max)
    tau = simData{6}.time(i);
    index = index + 1;

end

tau