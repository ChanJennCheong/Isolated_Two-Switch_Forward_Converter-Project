% =========================================================================
% === EEEE2073 Buck Converter Type III Compensator Design (Optimized)   ===
% =========================================================================

% --- 1. Power Stage Parameters ---
L = 3.64e-5;        % Inductance (H)
C = 3.64e-5;        % Capacitance (F)
ESR = 0.1;          % Equivalent Series Resistance (ohm)
Rmax = 8.533 * 4;   % Max Load Resistance at 25% load (ohm)
Vso_max = 325.27;   % DC Input Voltage (V)

% --- 2. Control Loop Architecture Parameters ---
alpha = 1.0;                % Direct feedback (no voltage divider)
K_pwm_n = 1.0 / Vso_max;    % PWM Gain with Input Feed-Forward cancellation
K_MISC = alpha * K_pwm_n * Vso_max; % Total frequency-independent gain (= 1.0)

% --- 3. Plant Transfer Function & Target Setup ---
w_res = 1 / sqrt(L * C);    % Natural resonant frequency (rad/s)
wc_target = 2.0 * w_res;    % Target crossover frequency (Double w_res for speed)

s = tf('s');
GF = (1 + s * ESR * C) / (s^2 * L * C + s * (L/Rmax + C*ESR) + 1);

[mag, pha] = bode(GF, wc_target);
magGF = mag(1); 
phaGF = pha(1);

% --- 4. K-Factor Calculations ---
target_PM = 80; % Target 80 degrees for critically damped response (zero overshoot)
B = target_PM - 180 + 90 - phaGF;
k = (tan((B/4 + 45) * pi/180))^2;

G = 1 / (K_MISC * magGF);
A = G * wc_target / k;

% Calculate Ideal Components (Starting with R1 = 100k to protect Op-Amp)
R1_initial = 100000;  
C2_id = 1 / (R1_initial * wc_target * G);
C1_id = C2_id * (k - 1);
R2_id = sqrt(k) / (wc_target * C1_id);
R3_id = R1_initial / (k - 1);
C3_id = 1 / (R3_id * wc_target * sqrt(k));

% --- 5. E24 Standard Value Rounding ---
R1_new = 100000; 
R2_std = get_E24(R2_id);
R3_std = get_E24(R3_id);
C1_std = get_E24(C1_id);
C2_std = get_E24(C2_id);
C3_std = get_E24(C3_id);

fprintf('\n=== Final Optimized Standard Components (E24 Series) ===\n');
fprintf('R1 = %d Ω\n', R1_new);
fprintf('R2 = %.0f Ω\n', R2_std);
fprintf('R3 = %.0f Ω\n', R3_std);
fprintf('C1 = %.1f nF\n', C1_std * 1e9);
fprintf('C2 = %.3f nF (%.0f pF)\n', C2_std * 1e9, C2_std * 1e12);
fprintf('C3 = %.1f nF\n', C3_std * 1e9);

% --- 6. Verify Actual Performance with E24 Components ---
k_actual = 1 + R1_new / R3_std;
wz_actual = 1 / (R2_std * C1_std);
wp_actual = 1 / (R3_std * C3_std);
wc_actual = sqrt(wz_actual * wp_actual);
G_actual = 1 / (R1_new * wc_actual * C2_std);
A_actual = G_actual * wc_actual / k_actual;

Gc_actual = A_actual * (1 + s/wz_actual)^2 / (s * (1 + s/wp_actual)^2);
Gloop = K_MISC * Gc_actual * GF;

[GM, PM, Wg, Wc] = margin(Gloop);

fprintf('\n=== Final Stability Verification ===\n');
fprintf('Crossover Frequency = %.1f rad/s (%.2f kHz)\n', Wc, Wc / (2 * pi * 1000));
fprintf('Phase Margin = %.1f°\n', PM);
if isinf(GM)
    fprintf('Gain Margin = Infinite\n');
else
    fprintf('Gain Margin = %.2f dB\n', 20*log10(GM));
end

% =========================================================================
% === 7. POSTER-READY BODE PLOT GENERATION                              ===
% =========================================================================

figure('Name', 'Final System Bode Plot', 'Color', 'w');
margin(Gloop); 
grid on;

% Make lines thicker for printed poster
set(findall(gcf,'type','line'),'linewidth', 2); 

% --- FOOLPROOF RED CIRCLE AT CROSSOVER FREQUENCY ---
% Get the exact phase at the crossover frequency and squeeze it into a clean number
[~, pha_target] = bode(Gloop, Wc); 
pha_target = squeeze(pha_target); 

% Find all internal axes in the special BodePlot object
all_axes = findall(gcf, 'Type', 'axes');

% Safely increase the font size for all found axes to make it poster-ready
set(all_axes, 'FontSize', 12);

y_positions = zeros(length(all_axes), 1);
for i = 1:length(all_axes)
    pos = get(all_axes(i), 'Position');
    y_positions(i) = pos(2); % Look at the bottom edge of the graph
end

% The graph with the lowest Y-position is the Phase plot
[~, bottom_idx] = min(y_positions);
phase_axes = all_axes(bottom_idx);

% Draw the circle and text directly on the Phase plot
hold(phase_axes, 'on');

% Plot the thick red circle
plot(phase_axes, Wc, pha_target, 'ro', 'MarkerSize', 14, 'LineWidth', 3);

% Add the PM text label next to the circle
label_text = sprintf('  PM \n  %.1f°', PM);
text(phase_axes, Wc * 1.15, pha_target, label_text, ...
    'Color', 'red', 'FontWeight', 'bold', 'FontSize', 12);
    
hold(phase_axes, 'off');

% =========================================================================
% === HELPER FUNCTION: Round to nearest E24 value                       ===
% =========================================================================
function val_std = get_E24(val)
    E24 = [1.0, 1.1, 1.2, 1.3, 1.5, 1.6, 1.8, 2.0, 2.2, ...
           2.4, 2.7, 3.0, 3.3, 3.6, 3.9, 4.3, 4.7, 5.1, ...
           5.6, 6.2, 6.8, 7.5, 8.2, 9.1];
    decade = 10^floor(log10(val));
    norm_val = val / decade;
    [~, idx] = min(abs(E24 - norm_val));
    val_std = E24(idx) * decade;
end