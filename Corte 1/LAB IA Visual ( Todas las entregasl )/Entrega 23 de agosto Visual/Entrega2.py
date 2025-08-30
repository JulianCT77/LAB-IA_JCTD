import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import Button
from scipy.integrate import solve_ivp
import matplotlib.patches as patches
import tkinter as tk
from tkinter import ttk

class InvertedPendulumSimulator2:
    def __init__(self):
        """
        Inicializa el simulador de péndulo invertido con parámetros físicos,
        estado inicial y configuración de la interfaz gráfica.
        Combina la estética visual del primer código con la funcionalidad
        mejorada del segundo código (control swing-up + PID).
        """
        
        # ==============================================
        # PARÁMETROS FÍSICOS DEL SISTEMA
        # ==============================================
        self.g = 9.81       # Aceleración gravitacional [m/s²]
        self.m = 0.1        # Masa del péndulo [kg]
        self.M = 1.0        # Masa del carro [kg]
        self.L = 0.5        # Longitud del péndulo [m]
        self.b = 0.0001        # Coeficiente de fricción del carro [N·s/m]
        self.force_mag = 20.0  # Magnitud máxima de fuerza aplicable [N]
        
        # Estado inicial del sistema [posición carro, velocidad carro, ángulo péndulo, velocidad angular]
        self.state = np.array([0.0, 0.0, np.pi, 0.0])  # Inicia colgando hacia abajo
        
        # Variables de control / simulación
        self.force = 0.0    # Fuerza actual aplicada al carro
        self.dt = 0.02      # Paso de tiempo para la simulación [s]
        self.time_elapsed = 0  # Tiempo transcurrido en la simulación
        
        # ---------------------------------------------------
        # CONTROL PID (para balanceo)
        # ---------------------------------------------------
        self.Kp = 150.0    # Proporcional
        self.Ki = 0.5      # Integral
        self.Kd = 25.0     # Derivativo
        self.integral_error = 0.0
        self.last_error = 0.0
        
        # Datos para historial
        self.history_t, self.history_x, self.history_theta = [], [], []
        
        # Configuración de las ventanas gráficas
        self.setup_main_window()  # Configura la ventana principal de simulación
        self.setup_control_panel()  # Configura los controles interactivos
        
        # Variables para la ventana de ecuaciones matemáticas
        self.math_window = None  # Referencia a la ventana de ecuaciones
        self.math_window_open = False  # Estado de la ventana de ecuaciones

    def setup_main_window(self):
        """
        Configura la ventana principal de visualización con todos los elementos gráficos.
        """
        plt.style.use('seaborn-v0_8-darkgrid')
        self.fig, self.ax = plt.subplots(figsize=(12, 7))
        self.fig.subplots_adjust(bottom=0.2)
        self.fig.patch.set_facecolor('#f5f5f5')
        
        # Inicialmente centramos alrededor de 0
        self.view_half_width = 3.0
        self.ax.set_xlim(-self.view_half_width, self.view_half_width)
        self.ax.set_ylim(-0.2, 1.5)
        self.ax.set_facecolor('#fafafa')
        self.ax.grid(True, linestyle=':', alpha=0.7)
        self.ax.set_title('SIMULADOR DE PÉNDULO INVERTIDO', 
                         fontsize=14, pad=20, color='#2c3e50', fontweight='bold')
        
        # Hacemos el suelo "muy ancho" y luego movemos la ventana (panning) en update()
        self.track = patches.Rectangle((-10000, -0.1), 20000, 0.05, 
                                     facecolor='#7f8c8d', edgecolor='#34495e', linewidth=1.0)
        self.ax.add_patch(self.track)
        
        # Carro (más grande que en el código original para mejor visualización)
        self.cart_width = 0.6
        self.cart_height = 0.2
        self.cart = patches.Rectangle((-self.cart_width/2, -self.cart_height/2), 
                                     self.cart_width, self.cart_height,
                                     facecolor='#3498db', edgecolor='#2980b9', linewidth=2)
        self.ax.add_patch(self.cart)
        
        # Péndulo
        self.pendulum_rod, = self.ax.plot([], [], '-', color='#2c3e50', lw=4)
        self.pendulum_bob = patches.Circle((0, 0), 0.08, 
                                        facecolor='#e74c3c', edgecolor='#c0392b', linewidth=2)
        self.ax.add_patch(self.pendulum_bob)
        
        # Texto de estado con formato mejorado
        self.status_text = self.ax.text(
            0.02, 0.95,
            'Posición carro: 0.00 m\n'
            'Velocidad carro: 0.00 m/s\n'
            'Ángulo: 0.00°\n'
            'Veloc. angular: 0.00°/s\n'
            'Fuerza aplicada: 0.00 N\n'
            'Modo control: Swing-Up',
            transform=self.ax.transAxes,
            bbox=dict(boxstyle='round', facecolor='white', alpha=0.9, 
                    edgecolor='#dddddd', pad=0.5),
            fontfamily='monospace',
            fontsize=11,
            verticalalignment='top'
        )

    def setup_control_panel(self):
        reset_ax = plt.axes([0.3, 0.05, 0.2, 0.075])
        self.btn_reset = Button(reset_ax, 'REINICIAR', 
                              color='#2ecc71', hovercolor='#27ae60')
        self.btn_reset.label.set_fontweight('bold')
        self.btn_reset.on_clicked(self.reset_simulation)
        
        eq_ax = plt.axes([0.55, 0.05, 0.2, 0.075])
        self.btn_eq = Button(eq_ax, 'ECUACIONES', 
                           color='#9b59b6', hovercolor='#8e44ad')
        self.btn_eq.label.set_fontweight('bold')
        self.btn_eq.on_clicked(self.show_equations_window)

    def show_equations_window(self, event=None):
        if self.math_window_open:
            if self.math_window:
                self.math_window.lift()
            return
        
        self.math_window_open = True
        self.math_window = tk.Toplevel()
        self.math_window.title("Modelo Matemático")
        self.math_window.geometry("800x650")
        self.math_window.protocol("WM_DELETE_WINDOW", self.close_math_window)
        
        main_frame = ttk.Frame(self.math_window)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=20, pady=20)
        
        title = ttk.Label(main_frame, text="MODELO MATEMÁTICO", 
                         font=('Helvetica', 14, 'bold'))
        title.pack(pady=10)
        
        params_frame = ttk.LabelFrame(main_frame, text="Parámetros del Sistema", 
                                    padding=10)
        params_frame.pack(fill=tk.X, pady=10)
        
        params = [
            ("Gravedad (g):", f"{self.g} m/s²"),
            ("Masa péndulo (m):", f"{self.m} kg"),
            ("Masa carro (M):", f"{self.M} kg"),
            ("Longitud (L):", f"{self.L} m"),
            ("Fricción (b):", f"{self.b} N·s/m"),
            ("Fuerza máxima:", f"{self.force_mag} N"),
            ("Paso tiempo:", f"{self.dt} s"),
            ("Kp, Ki, Kd:", f"{self.Kp}, {self.Ki}, {self.Kd}"),
        ]
        
        for name, value in params:
            frame = ttk.Frame(params_frame)
            frame.pack(fill=tk.X, padx=5, pady=2)
            ttk.Label(frame, text=name, width=22, anchor='e').pack(side=tk.LEFT)
            ttk.Label(frame, text=value, font=('Courier', 10)).pack(side=tk.LEFT)
        
        eq_frame = ttk.LabelFrame(main_frame, text="Ecuaciones del Movimiento y Control", 
                                padding=10)
        eq_frame.pack(fill=tk.BOTH, expand=True, pady=10)
        
        eq_text = """
        MODELO MATEMÁTICO DEL PÉNDULO INVERTIDO:

        Ecuaciones no lineales:
        ẍ = [F - bẋ + mLθ̇² sinθ - m g sinθ cosθ] / [M + m sin²θ]
        θ̈ = [g sinθ - cosθ (F - bẋ + mLθ̇² sinθ)/(M + m sin²θ)] / L

        CONTROL HÍBRIDO USADO:

        1. Swing-Up (para ángulos grandes |θ| > 0.2 rad):
        - Calcula energía actual: E = 0.5·m·(L·θ̇)² - m·g·L·cosθ
        - Energía deseada: E_deseada = -m·g·L (péndulo invertido)
        - Aplica fuerza proporcional a la diferencia de energía:
          F = k·(E - E_deseada)·sign(θ̇·cosθ)

        2. PID de balanceo (para ángulos pequeños |θ| ≤ 0.2 rad):
        - Control PID sobre el ángulo (referencia = 0 rad)
        - Fuerza incremental con límites ±F_max
        - Parámetros: Kp={}, Ki={}, Kd={}
        """.format(self.Kp, self.Ki, self.Kd)
        
        text = tk.Text(eq_frame, font=('Courier', 11), wrap=tk.WORD, padx=10, pady=10)
        text.insert(tk.END, eq_text)
        text.config(state=tk.DISABLED)
        text.pack(fill=tk.BOTH, expand=True)
        
        close_btn = ttk.Button(main_frame, text="CERRAR", 
                             command=self.close_math_window, style='Accent.TButton')
        close_btn.pack(pady=10)
        
        style = ttk.Style()
        style.configure('Accent.TButton', foreground='white', background='#e74c3c')

    def close_math_window(self):
        if self.math_window:
            self.math_window.destroy()
        self.math_window_open = False

    def get_status_text(self):
        theta = self.state[2]
        # Convertir el ángulo a nuestra convención deseada:
        # 0° = vertical arriba, 180°/-180° = vertical abajo
        display_angle = (np.degrees(theta) + 180) % 360 - 180  # Rango [-180,180]
        display_angle = -display_angle  # Invertimos para que vertical arriba sea 0°
        
        # Ajuste final para mostrar 0° cuando está vertical arriba
        if abs(display_angle) < 0.1:  # Umbral pequeño para considerar "vertical exacto"
            display_angle = 0.0
            
        angular_vel_deg = -np.degrees(self.state[3])  # Ajustamos velocidad angular
        
        # Determinar modo de control
        control_mode = "Swing-Up" if abs(theta) > 0.2 else "PID Balanceo"
        
        return (
            f"Posición carro: {self.state[0]:>7.3f} m\n"
            f"Velocidad carro: {self.state[1]:>6.3f} m/s\n"
            f"Ángulo péndulo: {display_angle:>6.2f}°\n"
            f"Veloc. angular: {angular_vel_deg:>6.2f}°/s\n"
            f"Fuerza aplicada: {self.force:>6.2f} N\n"
            f"Modo control: {control_mode}"
        )
        
    def dynamics(self, t, y, force):
        x, x_dot, theta, theta_dot = y
        sin_theta = np.sin(theta)
        cos_theta = np.cos(theta)

        total_mass = self.M + self.m
        pendulum_mass_length = self.m * self.L

        temp = (force + pendulum_mass_length * theta_dot**2 * sin_theta - self.b * x_dot) / total_mass
        theta_acc = (self.g * sin_theta - cos_theta * temp) / \
                    (self.L * (4.0/3.0 - (self.m * cos_theta**2) / total_mass))
        x_acc = temp - (pendulum_mass_length * theta_acc * cos_theta) / total_mass

        return [x_dot, x_acc, theta_dot, theta_acc]

    def swing_up_control(self):
        x, x_dot, theta, theta_dot = self.state

        # Normalizar ángulo a [-pi, pi]
        theta = ((theta + np.pi) % (2*np.pi)) - np.pi

        # Energía actual y deseada
        E = 0.5 * self.m * (self.L * theta_dot)**2 - self.m * self.g * self.L * np.cos(theta)
        E_desired = -self.m * self.g * self.L

        # Ley de control swing-up
        k = 10.0
        force = k * (E - E_desired) * np.sign(theta_dot * np.cos(theta))

        return np.clip(force, -self.force_mag, self.force_mag)

    def pid_balance_control(self):
        error = 0.0 - self.state[2]
        self.integral_error += error * self.dt
        self.integral_error = np.clip(self.integral_error, -10, 10)

        derivative_error = (error - self.last_error) / self.dt
        derivative_error = 0.9 * derivative_error + 0.1 * self.last_error
        self.last_error = error

        pid_output = (self.Kp * error +
                      self.Ki * self.integral_error +
                      self.Kd * derivative_error)

        self.force += pid_output * self.dt
        self.force = np.clip(self.force, -self.force_mag, self.force_mag)

        return self.force

    def update(self, i):
        theta = ((self.state[2] + np.pi) % (2*np.pi)) - np.pi

        # Selección de estrategia de control
        if abs(theta) > 0.2:  # Swing-up para ángulos grandes
            self.force = self.swing_up_control()
        else:  # PID para balanceo cerca del punto inestable
            self.force = self.pid_balance_control()

        # Integrar dinámica
        sol = solve_ivp(self.dynamics, [0, self.dt], self.state,
                       args=(self.force,), t_eval=[self.dt], method='RK45', rtol=1e-6)
        new_state = sol.y[:, -1]

        # Actualizar estado
        self.state = new_state
        self.time_elapsed += self.dt
        
        # Guardar datos para posible visualización posterior
        self.history_t.append(self.time_elapsed)
        self.history_x.append(self.state[0])
        self.history_theta.append(self.state[2])
        
        # ==============================================
        # ACTUALIZACIÓN GRÁFICA
        # ==============================================
        x = self.state[0]
        theta = self.state[2]
        pendulum_x = x + self.L * np.sin(theta)
        pendulum_y = self.cart_height/2 - self.L * np.cos(theta)
        
        # Actualizar posición del carro
        self.cart.set_xy((x - self.cart_width/2, -self.cart_height/2))
        
        # Actualizar péndulo
        self.pendulum_rod.set_data([x, pendulum_x], 
                                  [self.cart_height/2, pendulum_y])
        self.pendulum_bob.center = (pendulum_x, pendulum_y)
        
        # Centrar vista en la posición actual del carro
        left = x - self.view_half_width
        right = x + self.view_half_width
        self.ax.set_xlim(left, right)

        # Actualizar texto de estado
        self.status_text.set_text(self.get_status_text())
        
        return self.cart, self.pendulum_rod, self.pendulum_bob, self.status_text

    def reset_simulation(self, event):
        self.state = np.array([0.0, 0.0, np.pi, 0.0])
        self.force = 0.0
        self.integral_error = 0.0
        self.last_error = 0.0
        self.time_elapsed = 0
        self.history_t.clear()
        self.history_x.clear()
        self.history_theta.clear()
        self.status_text.set_text(self.get_status_text())

    def run(self):
        # blit=False porque cambiamos los límites de los ejes dinámicamente
        self.ani = animation.FuncAnimation(
            self.fig, self.update, 
            frames=10000,
            interval=self.dt*1000,
            blit=False,
            repeat=True
        )
        plt.show()

