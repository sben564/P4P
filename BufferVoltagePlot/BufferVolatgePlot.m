time_100hz = BufferVoltageReadings{:,1};
voltage_100hz = BufferVoltageReadings{:,2};

time_50hz = BufferVoltageReadings{:,4};
voltage_50hz = BufferVoltageReadings{:,5};

time_10hz = BufferVoltageReadings{:,7};
voltage_10hz = BufferVoltageReadings{:,8};

figure;

% First subplot (Top)
subplot(3, 1, 1); 
plot(time_100hz, voltage_100hz, 'r', 'LineWidth', 1.5);
title('Voltage vs Time (100Hz)');
xlabel('Time (s)');
ylabel('Voltage (V)');
grid on;

% Second subplot (Middle)
subplot(3, 1, 2); 
plot(time_50hz, voltage_50hz, 'g', 'LineWidth', 1.5);
title('Voltage vs Time (50Hz)');
xlabel('Time (s)');
ylabel('Voltage (V)');
grid on;

% Third subplot (Bottom)
subplot(3, 1, 3); 
plot(time_10hz, voltage_10hz, 'b', 'LineWidth', 1.5);
title('Voltage vs Time (10Hz)');
xlabel('Time (s)');
ylabel('Voltage (V)');
grid on;
