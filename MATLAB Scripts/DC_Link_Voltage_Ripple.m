

%System Parameters
phase_inductance = 56*10^-6; %Henries
HV_Bus = 300; %Volts
f_switching = 10000; %Hertz
acceptable_ripple = 1; %Percent



%Range of DC bus link capacitance values
C = [50:1:2000]; %uF

%Max Voltage Ripple at 50% Duty Cycle
v_ripple = (HV_Bus) ./ (32 .* (phase_inductance) .* (C.*10^-6) .* (f_switching).^2);

%Plot Setp
hold on;
plot(C, v_ripple);
plot(C, (acceptable_ripple./100).*HV_Bus.*ones(size(C))); %Percentage of Bus Voltage

title('Bus Ripple Voltage vs. Bus Link Capacitance for 50% Duty Cycle');
xlabel('Bus Link Capacitance (uF)');
ylabel('Bus Ripple Voltage');
legend('Voltage Ripple','Acceptable Voltage Ripple');
grid on;

%% Required Current Ripple

current_max_ripple = (.25.*HV_Bus)./(f_switching.*phase_inductance);

current_max_ripple_rms = current_max_ripple./sqrt(2)

