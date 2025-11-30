startup_rvc;

%% =============================
%  Modelo DH (offsets de HOME)
% ==============================

global Robot;

q1 = -pi/2;
q2 = -pi/2;
q3 = 0;
q4 = 0;
pi2 = sym(pi)/2;
L1 = 0.045;
L2 = 0.107;
L3 = 0.107;
L4 = 0.109;
% Traslación al primer marco
H01 = trotz(q1) * transl(0,0,L1) * transl(0,0,0) * trotx(-pi/2)

H12 = trotz(q2) * transl(0,0,0) * transl(L2,0,0) * trotx(-pi/2)

H23 = trotz(q3) * transl(0,0,0) * transl(L3,0,0) * trotx(0)

H34 = trotz(q4) * transl(0,0,0) * transl(L4,0,0) * trotx(0)


H04 = H01 * H12 * H23 * H34

ws = [-0.3 0.3 -0.3 0.3 -0.1 0.3];
plot_options = {'workspace',ws,'scale',0.2,'noa', ...
                'view',[125 25], 'tilesize',0.05, ...
                'ortho', 'lightpos',[2 2 10], ...
                'floorlevel',0, 'base'};

% ---------- Definición de eslabones (DH MODIFICADO) ----------
% A_i(q_i) = trotz(theta_i) * transl(0,0,d_i) * transl(a_i,0,0) * trotx(alpha_i)


L(1) = Link('revolute', ...
            'd',     L1, ...
            'a',     0, ...
            'alpha', -pi/2, ...
            'offset', 0, ...
            'qlim',  [-pi pi], ...
            'standard');

L(2) = Link('revolute', ...
            'd',     0, ...
            'a',     L2, ...
            'alpha', 0, ...
            'offset', 0, ...
            'qlim',  [-pi pi], ...
            'standard');

L(3) = Link('revolute', ...
            'd',     0, ...
            'a',     L3, ...
            'alpha', 0, ...
            'offset', 0, ...
            'qlim',  [-pi pi], ...
            'standard');

L(4) = Link('revolute', ...
            'd',     0, ...
            'a',     L4, ...
            'alpha', 0, ...
            'offset', 0, ...
            'qlim',  [-pi pi], ...
            'standard');

% ---------- Crear robot ----------
global Robot current_q animTimer;
Robot = SerialLink(L, 'name', 'MiRobot4R', 'plotopt', plot_options);

% (Opcional) Herramienta/TCP, si quieres añadir un offset extra:
% Robot.tool = trotz(0) * transl(0,0,0) * trotx(0);

% ---------- Probar con tu configuración de ejemplo (HOME) ----------
q_home    = [-pi/2, -pi/2, 0, 0];
current_q = q_home;

figure;
Robot.plot(q_home);   % interfaz gráfica inicial
hold on;
trplot(eye(4),'length',0.05);

% ---------- Cinemática directa con SerialLink en HOME ----------
TCP = Robot.fkine(q_home)
trplot(TCP, 'length', 0.04);

%% ============================================================
%  Visualizar el robot en tiempo real desde ROS 2 (/joint_states)
% =============================================================
%
% Requiere que ROS 2 esté
% configurado en el mismo ROS_DOMAIN_ID que tu sistema ROS 2.
%
% 1) En ROS 2: lanzar el nodo de control que publica /joint_states
%    (control_servo).
% 2) En MATLAB: ejecutar ESTE script (Lab5.m) UNA sola vez.
%    El robot se actualizará automáticamente cuando cambien las
%    articulaciones en ROS.

try
    node = ros2node("matlab_joint_listener");
    % Tipo exactamente como aparece en 'ros2 msg list'
    sub  = ros2subscriber(node, "/joint_states", "sensor_msgs/JointState", @jointStateCallback);
    disp("Suscrito a /joint_states desde MATLAB (Lab5.m).");

    % Timer para animar el robot a partir de current_q (evita animar dentro del callback)
    animTimer = timer( ...
        'ExecutionMode', 'fixedRate', ...
        'Period', 0.05, ...           % 20 Hz
        'TimerFcn', @animateRobot);
    start(animTimer);
catch ME
    warning("No se pudo crear el nodo ROS 2 o el suscriptor a /joint_states:\n  %s", ME.message);
end

%% ============================
%  Función de callback ROS 2
% ============================

function jointStateCallback(msg)
% Callback para actualizar solo la configuración articular deseada (current_q)
    global current_q;

    % Mapeo de nombres de joints de ROS 2 al orden DH del Robot
    % y control de frecuencia de actualización para evitar recursión.
    persistent idx_base idx_shoulder idx_elbow idx_wrist lastUpdateTic

    names = string(msg.name);

    if isempty(idx_base)
        idx_base     = find(names == "phantomx_pincher_arm_shoulder_pan_joint",   1);
        idx_shoulder = find(names == "phantomx_pincher_arm_shoulder_lift_joint",  1);
        idx_elbow    = find(names == "phantomx_pincher_arm_elbow_flex_joint",     1);
        idx_wrist    = find(names == "phantomx_pincher_arm_wrist_flex_joint",     1);

        if isempty(idx_base) || isempty(idx_shoulder) || isempty(idx_elbow) || isempty(idx_wrist)
            % Aún no tenemos todas las articulaciones en este mensaje
            return;
        end
    end

    if numel(msg.position) < max([idx_base, idx_shoulder, idx_elbow, idx_wrist])
        return;
    end

    % Ángulos en ROS (radianes)
    q1_ros = msg.position(idx_base);
    q2_ros = msg.position(idx_shoulder);
    q3_ros = msg.position(idx_elbow);
    q4_ros = msg.position(idx_wrist);

    % Aplicar los mismos offsets que en Python para pasar a tu modelo DH:
    %   q1_dh = q1_ros - pi/2
    %   q2_dh = q2_ros - pi/2
    %   q3_dh = q3_ros
    %   q4_dh = q4_ros
    q1 = q1_ros - pi/2;
    q2 = q2_ros - pi/2;
    q3 = q3_ros;
    q4 = q4_ros;

    q = [q1 q2 q3 q4];

    % Solo actualizamos la variable global; la animación la hace el timer
    current_q = q;
end

function animateRobot(~, ~)
% Función periódica que anima el robot con la última configuración conocida
    global Robot current_q;
    if isempty(Robot) || isempty(current_q)
        return;
    end

    Robot.animate(current_q);
end
