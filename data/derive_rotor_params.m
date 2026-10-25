function derive_rotor_params
%% Open the data dictionary 
dd  = Simulink.data.dictionary.open('Quad.sldd');
sec = getSection(dd, 'Design Data');
getp = @(name) getValue(getEntry(sec, name)).Value;   % read a parameter value

%% raw inputs (with sources)

% Thrust test point for a 10x4.5 in (1045) prop, static (hover) conditions.
% Source: Vianna Jr., De Paula, Leandro, Bonfim, "Stability Control of a
% Quad-Rotor Using a PID Controller", Brazilian Journal of Instrumentation
% and Control, 2013, DOI 10.3895/S2318-45312013000100003, sec. II-A:
% EPP-1045 prop, 1.72 N per rotor measured at 65 rev/s (load cell + Hall sensor).
T_test = 1.72;        % N
n_test = 65;          % rev/s

% Air density, ISA sea level. Source: ISO 2533 (International Standard Atmosphere).
rho = 1.225;          % kg/m^3

% Gravity. Must match the Mechanism Configuration block ([0 0 -9.80665]).
% Source: standard gravity, CGPM 1901.
g = 9.80665;          % m/s^2

% Figure of merit (hover efficiency vs. ideal momentum theory).
% ESTIMATE, no measured source yet. Replace kQ with measured torque data
% (e.g. UIUC Propeller Database, Vol. 1, APC 10x4.7 SF static CP) when available.
FM = 0.55;            % -

% Spin direction per rotor [FL RL RR FR]
rotor_dir = [-1 1 -1 1];

% Motor first-order time constant (commanded -> actual rotor speed).
% Source: Ashraf et al., "Modelling and Parameters Identification of a Quadrotor
% Using a Custom Test Rig", IJPEDS 9(2), 2018, sec. 3.5: 10 % throttle step,
% 63 % rise time = 0.07 s (EMAX XA2212 + 1047 prop on a DJI F450).
% Cross-check: Valavanis et al. slides, slide 41, use 0.05 s.
tau_m = 0.07;         % s

% Motor KV and battery voltage, for the speed limit.
% KV: DJI 2212 920 KV motor spec. V: 3S LiPo nominal, 3 cells x 3.7 V.
KV     = 920;         % rpm/V
V_batt = 3*3.7;       % V

%% Values read from the dictionary
m_hub   = getp('m_hub');
m_arm   = getp('m_arm');
m_motor = getp('m_motor');
m_prop  = getp('m_prop');
prop_dims = getp('prop_dims');
arm_dims  = getp('arm_dims');
h_motor   = getp('h_motor');

D = prop_dims(1);% prop diameter, m
A = pi*(D/2)^2;  % rotor disc area, m^2

%% Derived quantities

% Thrust coefficient: T = kT * Omega^2, Omega in rad/s
Omega_test = 2*pi*n_test; % rad/s
kT = T_test / Omega_test^2;  % N*s^2/rad^2

% Drag-torque coefficient from momentum theory (Leishman, Principles of
% Helicopter Aerodynamics, ch. 2):
%   P = T^1.5 / (FM*sqrt(2*rho*A)),  Q = P/Omega,  T = kT*Omega^2
%   => kQ = kT^1.5 / (FM*sqrt(2*rho*A))   (independent of Omega)
kQ = kT^1.5 / (FM*sqrt(2*rho*A));  % N*m*s^2/rad^2

% Total mass from the mass budget (same parts Simscape sums)
m_total = m_hub + 4*(m_arm + m_motor + m_prop);   % kg

% Hover speed per rotor: 4*kT*Omega_h^2 = m*g
Omega_h = sqrt(m_total*g / (4*kT));               % rad/s

% Rotor speed limit = no-load speed KV*V. UPPER BOUND: the loaded maximum is
% lower (typ. 80-85 %). Replace with a loaded value from a thrust table.
Omega_max = KV*V_batt*2*pi/60;                    % rad/s
TW_max    = 4*kT*Omega_max^2 / (m_total*g);       % thrust-to-weight at the limit

% Cross-checks (printed only, not written)
CT_check  = kT*4*pi^2 / (rho*D^4);    % nondimensional, paper: 0.0798
rpm_h     = Omega_h*60/(2*pi);  % rpm
z_motor   = arm_dims(3)/2 + h_motor/2;% motor centre above hub centre, m
z_prop    = arm_dims(3)/2 + h_motor; % prop centre above hub centre, m
z_cg_hand = 4*(m_motor*z_motor + m_prop*z_prop) / m_total;
z_cg_dict = getp('z_cg');

%% results into the dictionary
setParam(sec, 'kT',        kT,        'N*s^2/rad^2',   'Thrust coeff, T = kT*Omega^2. Derived in derive_rotor_params.m');
setParam(sec, 'kQ',        kQ,        'N*m*s^2/rad^2', 'Drag torque coeff, Q = kQ*Omega^2. ESTIMATE (FM assumed). derive_rotor_params.m');
setParam(sec, 'rotor_dir', rotor_dir, '',              'Spin direction [FL RL RR FR], +1 = CCW from above (PX4 quad X)');
setParam(sec, 'Omega_h',   Omega_h,   'rad/s',         'Hover speed per rotor, sqrt(m*g/(4*kT)). derive_rotor_params.m');
setParam(sec, 'tau_m',     tau_m,     's',             'Motor time constant, measured (IJPEDS 2018). derive_rotor_params.m');
setParam(sec, 'Omega_max', Omega_max, 'rad/s',         'Rotor speed limit = KV*V no-load. UPPER BOUND. derive_rotor_params.m');
saveChanges(dd);

%% Report
fprintf('\n--- derive_rotor_params ---\n');
fprintf('kT        = %.4e N*s^2/rad^2\n', kT);
fprintf('kQ        = %.4e N*m*s^2/rad^2  (ESTIMATE, FM = %.2f)\n', kQ, FM);
fprintf('kQ/kT     = %.4f m\n', kQ/kT);
fprintf('m_total   = %.4f kg        (Inertia Sensor: 0.935)\n', m_total);
fprintf('Omega_h   = %.2f rad/s = %.0f rpm\n', Omega_h, rpm_h);
fprintf('C_T check = %.4f           (paper: 0.0798)\n', CT_check);
fprintf('tau_m     = %.3f s\n', tau_m);
fprintf('Omega_max = %.1f rad/s = %.0f rpm (UPPER BOUND), T/W at limit = %.2f\n', Omega_max, Omega_max*60/(2*pi), TW_max);
if isnumeric(z_cg_dict)
    fprintf('z_cg hand = %.6e m, dictionary = %.6e m\n', z_cg_hand, z_cg_dict);
    if abs(z_cg_hand - z_cg_dict) > 1e-4
        warning('z_cg in the dictionary differs from the hand value by > 0.1 mm. Re-measure with the Inertia Sensor.');
    end
else
    % z_cg was entered as an expression (Simulink.data.Expression), not a number
    fprintf('z_cg hand = %.6e m, dictionary holds a %s, not a number:\n', z_cg_hand, class(z_cg_dict));
    disp(z_cg_dict);
end

end

%% Local function: create or replace a Simulink.Parameter entry
function setParam(sec, name, value, unit, desc)
    p = Simulink.Parameter(value);     % fresh object: Dimensions follow the value
    p.Unit = unit;
    p.Description = desc;
    if exist(sec, name)
        setValue(getEntry(sec, name), p);
    else
        addEntry(sec, name, p);
    end
end