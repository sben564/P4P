data = readtable('TransmittervsReciverData.xlsx');

receiver_data = data{1:128,1};
time_data = data{1:128,2};
transmitter_data = data{1:128,3};

figure;
plot(time_data,receiver_data,'-b');
hold on;
%%plot(time_data,transmitter_data,'-g');
legend('Receiver Data','Transmitter Data');
xlabel('Time (s)')
ylabel('Amplitude')
grid on;