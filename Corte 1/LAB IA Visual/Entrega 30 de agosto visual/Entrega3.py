import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import Button
from scipy.integrate import solve_ivp
import matplotlib.patches as patches
import tkinter as tk
from tkinter import ttk
import skfuzzy as fuzz
from skfuzzy import control as ctrl
import networkx as nx

class FuzzyPendulumController:
    def __init__(self):
        """
        Controlador difuso optimizado para el péndulo invertido.
        """
        print("Inicializando controlador difuso optimizado...")
        
        # Variables de entrada con rangos optimizados
        self.angle = ctrl.Antecedent(np.arange(-np.pi/2, np.pi/2, 0.01), 'angle')
        self.angular_velocity = ctrl.Antecedent(np.arange(-5, 5, 0.1), 'angular_velocity')
        self.position = ctrl.Antecedent(np.arange(-2.0, 2.0, 0.1), 'position')
        
        # Variable de salida
        self.force = ctrl.Consequent(np.arange(-20, 20, 0.1), 'force')
        
        # Funciones de membresía OPTIMIZADAS para el ángulo
        self.angle['NB'] = fuzz.trimf(self.angle.universe, [-np.pi/2, -np.pi/3, -np.pi/6])
        self.angle['N'] = fuzz.trimf(self.angle.universe, [-np.pi/3, -np.pi/12, 0])
        self.angle['Z'] = fuzz.trimf(self.angle.universe, [-np.pi/24, 0, np.pi/24])
        self.angle['P'] = fuzz.trimf(self.angle.universe, [0, np.pi/12, np.pi/3])
        self.angle['PB'] = fuzz.trimf(self.angle.universe, [np.pi/6, np.pi/3, np.pi/2])
        
        # Funciones de membresía para la velocidad angular
        self.angular_velocity['NB'] = fuzz.trimf(self.angular_velocity.universe, [-5, -4, -2])
        self.angular_velocity['N'] = fuzz.trimf(self.angular_velocity.universe, [-3, -1.5, 0])
        self.angular_velocity['Z'] = fuzz.trimf(self.angular_velocity.universe, [-0.5, 0, 0.5])
        self.angular_velocity['P'] = fuzz.trimf(self.angular_velocity.universe, [0, 1.5, 3])
        self.angular_velocity['PB'] = fuzz.trimf(self.angular_velocity.universe, [2, 4, 5])
        
        # Funciones de membresía para la posición
        self.position['NL'] = fuzz.trimf(self.position.universe, [-2.0, -1.5, -1.0])
        self.position['N'] = fuzz.trimf(self.position.universe, [-1.2, -0.6, 0])
        self.position['Z'] = fuzz.trimf(self.position.universe, [-0.3, 0, 0.3])
        self.position['P'] = fuzz.trimf(self.position.universe, [0, 0.6, 1.2])
        self.position['PL'] = fuzz.trimf(self.position.universe, [1.0, 1.5, 2.0])
        
        # Funciones de membresía para la fuerza
        self.force['NL'] = fuzz.trimf(self.force.universe, [-20, -15, -10])
        self.force['NM'] = fuzz.trimf(self.force.universe, [-12, -8, -4])
        self.force['NS'] = fuzz.trimf(self.force.universe, [-6, -3, 0])
        self.force['Z'] = fuzz.trimf(self.force.universe, [-2, 0, 2])
        self.force['PS'] = fuzz.trimf(self.force.universe, [0, 3, 6])
        self.force['PM'] = fuzz.trimf(self.force.universe, [4, 8, 12])
        self.force['PL'] = fuzz.trimf(self.force.universe, [10, 15, 20])
        
        # REGLAS OPTIMIZADAS para mejor estabilidad
        rules = [
            # Reglas para estabilización en posición vertical (prioridad máxima)
            ctrl.Rule(self.angle['Z'] & self.angular_velocity['Z'] & self.position['Z'], self.force['Z']),
            ctrl.Rule(self.angle['Z'] & self.angular_velocity['Z'] & self.position['N'], self.force['PS']),
            ctrl.Rule(self.angle['Z'] & self.angular_velocity['Z'] & self.position['P'], self.force['NS']),
            
            # Reglas para corrección de ángulo con velocidad cero
            ctrl.Rule(self.angle['N'] & self.angular_velocity['Z'], self.force['NM']),
            ctrl.Rule(self.angle['P'] & self.angular_velocity['Z'], self.force['PM']),
            ctrl.Rule(self.angle['NB'] & self.angular_velocity['Z'], self.force['NL']),
            ctrl.Rule(self.angle['PB'] & self.angular_velocity['Z'], self.force['PL']),
            
            # Reglas para amortiguamiento (corrección de velocidad angular)
            ctrl.Rule(self.angle['Z'] & self.angular_velocity['N'], self.force['NS']),
            ctrl.Rule(self.angle['Z'] & self.angular_velocity['P'], self.force['PS']),
            ctrl.Rule(self.angle['Z'] & self.angular_velocity['NB'], self.force['NM']),
            ctrl.Rule(self.angle['Z'] & self.angular_velocity['PB'], self.force['PM']),
            
            # Reglas combinadas ángulo-velocidad
            ctrl.Rule(self.angle['N'] & self.angular_velocity['N'], self.force['NM']),
            ctrl.Rule(self.angle['N'] & self.angular_velocity['P'], self.force['NS']),
            ctrl.Rule(self.angle['P'] & self.angular_velocity['N'], self.force['PS']),
            ctrl.Rule(self.angle['P'] & self.angular_velocity['P'], self.force['PM']),
            
            # Reglas para casos extremos
            ctrl.Rule(self.angle['NB'] & self.angular_velocity['NB'], self.force['NL']),
            ctrl.Rule(self.angle['NB'] & self.angular_velocity['PB'], self.force['NM']),
            ctrl.Rule(self.angle['PB'] & self.angular_velocity['NB'], self.force['PM']),
            ctrl.Rule(self.angle['PB'] & self.angular_velocity['PB'], self.force['PL']),
            
            # Reglas para control de posición
            ctrl.Rule(self.position['NL'], self.force['PS']),
            ctrl.Rule(self.position['PL'], self.force['NS']),
            ctrl.Rule(self.position['N'] & self.angle['Z'], self.force['PS']),
            ctrl.Rule(self.position['P'] & self.angle['Z'], self.force['NS'])
        ]
        
        # Sistema de control
        self.control_system = ctrl.ControlSystem(rules)
        self.controller = ctrl.ControlSystemSimulation(self.control_system)
        
        print("Controlador difuso optimizado inicializado correctamente!")
        
    def compute_force(self, angle, angular_velocity, position):
        """
        Calcula la fuerza de control usando lógica difusa optimizada.
        """
        try:
            # Normalizar ángulo al rango [-pi, pi]
            angle = (angle + np.pi) % (2 * np.pi) - np.pi
            
            # Limitar valores con márgenes de seguridad
            angle = np.clip(angle, -np.pi/1.5, np.pi/1.5)
            angular_velocity = np.clip(angular_velocity, -6, 6)
            position = np.clip(position, -1.8, 1.8)
            
            self.controller.input['angle'] = angle
            self.controller.input['angular_velocity'] = angular_velocity
            self.controller.input['position'] = position
            
            self.controller.compute()
            force = self.controller.output['force']
            
            # Suavizar la fuerza y limitar
            return np.clip(force, -20, 20)
            
        except Exception as e:
            print(f"Error en control difuso: {e}")
            return 0.0

class InvertedPendulumSimulator:
    def __init__(self):
        """
        Simulador de péndulo invertido con control difuso optimizado.
        """
        # ==============================================
        # PARÁMETROS FÍSICOS DEL SISTEMA OPTIMIZADOS
        # ==============================================
        self.g = 9.81       # Aceleración gravitacional [m/s²]
        self.m = 0.2        # Masa del péndulo [kg] - Reducida para mejor control
        self.M = 1.0        # Masa del carro [kg] - Aumentada para mayor estabilidad
        self.L = 0.5        # Longitud del péndulo [m]
        self.b = 0.05       # Coeficiente de fricción del carro [N·s/m] - Reducido
        self.force_mag = 20.0  # Magnitud máxima de fuerza aplicable [N]
        
        # Estado inicial del sistema
        self.state = np.array([0.0, 0.0, 0.05, 0.0])  # Ángulo inicial más pequeño
        
        # Variables de control
        self.force = 0.0
        self.dt = 0.02  # Paso de tiempo optimizado
        self.max_pos = 2.0
        
        # Controlador difuso mejorado
        self.fuzzy_controller = FuzzyPendulumController()
        self.use_fuzzy_control = True
        
        # Para visualización
        self.fuzzy_values = {
            'angle': 0.0,
            'angular_velocity': 0.0,
            'position': 0.0,
            'force': 0.0
        }
        
        # Historial para gráficos
        self.history = {
            'time': [],
            'angle': [],
            'position': [],
            'force': [],
            'angular_velocity': []
        }
        
        # Configuración de ventanas
        self.setup_main_window()
        self.setup_control_panel()
        
        self.math_window = None
        self.math_window_open = False
        self.fuzzy_window = None
        self.fuzzy_window_open = False
        self.graph_window = None
        self.graph_window_open = False

    def setup_main_window(self):
        """
        Configura la ventana principal de visualización.
        """
        plt.style.use('default')
        self.fig, self.ax = plt.subplots(figsize=(14, 8))
        self.fig.subplots_adjust(bottom=0.25)
        self.fig.patch.set_facecolor('#f0f0f0')
        
        self.ax.set_xlim(-2.5, 2.5)
        self.ax.set_ylim(-0.5, 1.2)
        self.ax.set_facecolor('#fafafa')
        self.ax.grid(True, linestyle=':', alpha=0.7)
        self.ax.set_title('PÉNDULO INVERTIDO - CONTROL DIFUSO OPTIMIZADO', 
                         fontsize=14, pad=20, color='#2c3e50', fontweight='bold')
        
        # Elementos gráficos
        self.track = patches.Rectangle((-2.5, -0.1), 5, 0.05, 
                                     facecolor='#7f8c8d', edgecolor='#34495e', linewidth=1.5)
        self.ax.add_patch(self.track)
        
        self.cart = patches.Rectangle((-0.25, -0.06), 0.5, 0.12, 
                                    facecolor='#3498db', edgecolor='#2980b9', linewidth=2)
        self.ax.add_patch(self.cart)
        
        self.pendulum_rod, = self.ax.plot([], [], '-', color='#2c3e50', lw=3)
        self.pendulum_bob = patches.Circle((0, 0), 0.06, 
                                        facecolor='#e74c3c', edgecolor='#c0392b', linewidth=2)
        self.ax.add_patch(self.pendulum_bob)
        
        # Línea central de referencia
        self.ax.axvline(x=0, color='red', linestyle='--', alpha=0.5, label='Centro')
        
        # Indicador de posición objetivo
        self.target_line = self.ax.axhline(y=0.5, color='green', linestyle=':', alpha=0.7, label='Posición objetivo')
        
        # Panel de información
        self.status_text = self.ax.text(
            0.02, 0.95,
            'Posición carro: 0.00 m\n'
            'Velocidad carro: 0.00 m/s\n'
            'Ángulo: 0.00°\n'
            'Velocidad angular: 0.00°/s\n'
            'Fuerza aplicada: 0.00 N\n'
            'Modo control: DIFUSO OPTIMIZADO',
            transform=self.ax.transAxes,
            bbox=dict(boxstyle='round', facecolor='white', alpha=0.9, 
                    edgecolor='#dddddd', pad=0.5),
            fontfamily='monospace',
            fontsize=10,
            verticalalignment='top'
        )
        
        # Leyenda
        self.ax.legend(loc='upper right')
        
        # Eventos de teclado
        self.fig.canvas.mpl_connect('key_press_event', self.on_key_press)
        self.fig.canvas.mpl_connect('key_release_event', self.on_key_release)

    def setup_control_panel(self):
        """
        Configura los botones de control.
        """
        # Botón de reinicio
        reset_ax = plt.axes([0.1, 0.05, 0.15, 0.075])
        self.btn_reset = Button(reset_ax, 'REINICIAR', color='#2ecc71', hovercolor='#27ae60')
        self.btn_reset.label.set_fontweight('bold')
        self.btn_reset.on_clicked(self.reset_simulation)
        
        # Botón de ecuaciones
        eq_ax = plt.axes([0.3, 0.05, 0.15, 0.075])
        self.btn_eq = Button(eq_ax, 'ECUACIONES', color='#9b59b6', hovercolor='#8e44ad')
        self.btn_eq.label.set_fontweight('bold')
        self.btn_eq.on_clicked(self.show_equations_window)
        
        # Botón de control difuso
        fuzzy_ax = plt.axes([0.5, 0.05, 0.15, 0.075])
        self.btn_fuzzy = Button(fuzzy_ax, 'CONTROL DIFUSO', color='#e67e22', hovercolor='#d35400')
        self.btn_fuzzy.label.set_fontweight('bold')
        self.btn_fuzzy.on_clicked(self.show_fuzzy_window)
        
        # Botón de grafo de reglas
        graph_ax = plt.axes([0.7, 0.05, 0.15, 0.075])
        self.btn_graph = Button(graph_ax, 'GRAFO REGLAS', color='#34495e', hovercolor='#2c3e50')
        self.btn_graph.label.set_fontweight('bold')
        self.btn_graph.on_clicked(self.show_rules_graph)
        
        # Toggle control difuso
        self.fuzzy_toggle_ax = plt.axes([0.87, 0.05, 0.1, 0.075])
        self.fuzzy_toggle = Button(self.fuzzy_toggle_ax, 'DIFUSO: ON', color='#2ecc71')
        self.fuzzy_toggle.on_clicked(self.toggle_fuzzy_control)
        
        # Botón para gráficos de desempeño
        perf_ax = plt.axes([0.1, 0.14, 0.15, 0.075])
        self.btn_perf = Button(perf_ax, 'DESEMPEÑO', color='#f39c12', hovercolor='#e67e22')
        self.btn_perf.label.set_fontweight('bold')
        self.btn_perf.on_clicked(self.show_performance_plots)

    def toggle_fuzzy_control(self, event):
        """Activa/desactiva el control difuso."""
        self.use_fuzzy_control = not self.use_fuzzy_control
        if self.use_fuzzy_control:
            self.fuzzy_toggle.label.set_text('DIFUSO: ON')
            self.fuzzy_toggle.color = '#2ecc71'
            self.status_text.set_text(self.get_status_text().replace("MANUAL", "DIFUSO OPTIMIZADO"))
        else:
            self.fuzzy_toggle.label.set_text('DIFUSO: OFF')
            self.fuzzy_toggle.color = '#e74c3c'
            self.status_text.set_text(self.get_status_text().replace("DIFUSO OPTIMIZADO", "MANUAL"))
        self.fig.canvas.draw_idle()

    def show_performance_plots(self, event=None):
        """Muestra gráficos de desempeño del controlador."""
        if len(self.history['time']) == 0:
            return
            
        fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(12, 8))
        
        # Gráfico de ángulo
        ax1.plot(self.history['time'], np.degrees(self.history['angle']))
        ax1.set_ylabel('Ángulo (°)')
        ax1.set_title('Evolución del Ángulo del Péndulo')
        ax1.grid(True)
        ax1.axhline(y=0, color='r', linestyle='--', alpha=0.5)
        
        # Gráfico de posición
        ax2.plot(self.history['time'], self.history['position'])
        ax2.set_ylabel('Posición (m)')
        ax2.set_title('Posición del Carro')
        ax2.grid(True)
        ax2.axhline(y=0, color='r', linestyle='--', alpha=0.5)
        
        # Gráfico de fuerza
        ax3.plot(self.history['time'], self.history['force'])
        ax3.set_xlabel('Tiempo (s)')
        ax3.set_ylabel('Fuerza (N)')
        ax3.set_title('Fuerza de Control Aplicada')
        ax3.grid(True)
        ax3.axhline(y=0, color='r', linestyle='--', alpha=0.5)
        
        # Gráfico de velocidad angular
        ax4.plot(self.history['time'], np.degrees(self.history['angular_velocity']))
        ax4.set_xlabel('Tiempo (s)')
        ax4.set_ylabel('Velocidad Angular (°/s)')
        ax4.set_title('Velocidad Angular del Péndulo')
        ax4.grid(True)
        ax4.axhline(y=0, color='r', linestyle='--', alpha=0.5)
        
        plt.tight_layout()
        plt.show()

    def show_rules_graph(self, event=None):
        """
        Muestra un grafo de las reglas difusas usando networkx.
        """
        if self.graph_window_open:
            if self.graph_window:
                self.graph_window.lift()
            return
        
        self.graph_window_open = True
        self.graph_window = tk.Toplevel()
        self.graph_window.title("Grafo de Reglas Difusas Optimizadas")
        self.graph_window.geometry("1000x700")
        self.graph_window.protocol("WM_DELETE_WINDOW", self.close_graph_window)
        
        # Crear grafo
        G = nx.DiGraph()
        
        # Añadir nodos y aristas para las reglas principales
        main_rules = [
            ("Ángulo Z + Vel Z + Pos Z", "Fuerza Z"),
            ("Ángulo N + Vel Z", "Fuerza NM"),
            ("Ángulo P + Vel Z", "Fuerza PM"),
            ("Ángulo Z + Vel N", "Fuerza NS"),
            ("Ángulo Z + Vel P", "Fuerza PS"),
            ("Posición NL", "Fuerza PS"),
            ("Posición PL", "Fuerza NS")
        ]
        
        for input_node, output_node in main_rules:
            G.add_edge(input_node, output_node)
        
        # Crear figura de matplotlib
        fig, ax = plt.subplots(figsize=(12, 9))
        
        # Dibujar el grafo
        pos = nx.spring_layout(G, k=3, iterations=100)
        nx.draw_networkx_nodes(G, pos, node_color='lightblue', 
                              node_size=2500, ax=ax, alpha=0.8)
        nx.draw_networkx_edges(G, pos, edge_color='gray', 
                              arrows=True, arrowsize=25, ax=ax, width=2)
        nx.draw_networkx_labels(G, pos, font_size=9, ax=ax, font_weight='bold')
        
        ax.set_title("Grafo de Reglas del Controlador Difuso Optimizado", 
                    fontsize=14, fontweight='bold')
        ax.axis('off')
        
        # Embedder la figura en tkinter
        from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
        canvas = FigureCanvasTkAgg(fig, master=self.graph_window)
        canvas.draw()
        canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        # Botón de cierre
        close_btn = ttk.Button(self.graph_window, text="CERRAR", 
                             command=self.close_graph_window)
        close_btn.pack(pady=10)

    def close_graph_window(self):
        """Cierra la ventana del grafo."""
        if self.graph_window:
            self.graph_window.destroy()
        self.graph_window_open = False

    def show_fuzzy_window(self, event=None):
        """
        Muestra ventana con información del controlador difuso optimizado.
        """
        if self.fuzzy_window_open:
            if self.fuzzy_window:
                self.fuzzy_window.lift()
            return
        
        self.fuzzy_window_open = True
        self.fuzzy_window = tk.Toplevel()
        self.fuzzy_window.title("Control Difuso Optimizado - Información")
        self.fuzzy_window.geometry("1200x900")
        self.fuzzy_window.protocol("WM_DELETE_WINDOW", self.close_fuzzy_window)
        
        main_frame = ttk.Frame(self.fuzzy_window)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=20, pady=20)
        
        title = ttk.Label(main_frame, text="SISTEMA DE CONTROL DIFUSO OPTIMIZADO", 
                         font=('Helvetica', 16, 'bold'))
        title.pack(pady=10)
        
        # Información del controlador optimizado
        info_text = """
        CONTROLADOR DIFUSO OPTIMIZADO PARA PÉNDULO INVERTIDO
        
        MEJORAS IMPLEMENTADAS:
        - Funciones triangulares para mejor respuesta
        - Conjuntos difusos optimizados para estabilidad
        - Reglas simplificadas y más efectivas
        - Mejor manejo de casos límite
        - Control más agresivo para ángulos grandes
        - Control más suave para ángulos pequeños
        
        Variables de entrada:
        - Ángulo del péndulo (θ): [-π/2, π/2] radianes
        - Velocidad angular (θ̇): [-5, 5] rad/s
        - Posición del carro (x): [-2.0, 2.0] metros
        
        Variable de salida:
        - Fuerza aplicada (F): [-20, 20] Newtons
        
        Conjuntos difusos:
        - Ángulo: NB, N, Z, P, PB (Triangulares)
        - Velocidad angular: NB, N, Z, P, PB (Triangulares)  
        - Posición: NL, N, Z, P, PL (Triangulares)
        - Fuerza: NL, NM, NS, Z, PS, PM, PL (Triangulares)
        
        Base de reglas: 20 reglas difusas optimizadas
        Método de inferencia: Mamdani
        Método de defuzzificación: Centroide
        """
        
        info_label = ttk.Label(main_frame, text=info_text, justify=tk.LEFT,
                              font=('Courier', 9))
        info_label.pack(pady=10, fill=tk.X)
        
        # Valores actuales
        values_frame = ttk.LabelFrame(main_frame, text="Valores Actuales", padding=10)
        values_frame.pack(fill=tk.X, pady=10)
        
        self.angle_var = tk.StringVar(value="Ángulo: 0.0000 rad")
        self.ang_vel_var = tk.StringVar(value="Vel. Angular: 0.0000 rad/s")
        self.position_var = tk.StringVar(value="Posición: 0.0000 m")
        self.force_var = tk.StringVar(value="Fuerza: 0.0000 N")
        
        ttk.Label(values_frame, textvariable=self.angle_var, font=('Courier', 10)).pack(anchor='w')
        ttk.Label(values_frame, textvariable=self.ang_vel_var, font=('Courier', 10)).pack(anchor='w')
        ttk.Label(values_frame, textvariable=self.position_var, font=('Courier', 10)).pack(anchor='w')
        ttk.Label(values_frame, textvariable=self.force_var, font=('Courier', 10)).pack(anchor='w')
        
        update_btn = ttk.Button(values_frame, text="Actualizar Valores", 
                               command=self.update_fuzzy_values)
        update_btn.pack(pady=5)
        
        # Mostrar gráficos de las funciones de membresía
        plot_frame = ttk.LabelFrame(main_frame, text="Funciones de Membresía", padding=10)
        plot_frame.pack(fill=tk.BOTH, expand=True, pady=10)
        
        # Crear figura con subplots
        from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
        from matplotlib.figure import Figure
        
        fig = Figure(figsize=(10, 8))
        canvas = FigureCanvasTkAgg(fig, master=plot_frame)
        canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        ax1 = fig.add_subplot(221)
        for label in self.fuzzy_controller.angle.terms:
            ax1.plot(self.fuzzy_controller.angle.universe, 
                    self.fuzzy_controller.angle[label].mf, 
                    label=label, linewidth=2)
        ax1.set_title('Ángulo del Péndulo')
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        
        ax2 = fig.add_subplot(222)
        for label in self.fuzzy_controller.angular_velocity.terms:
            ax2.plot(self.fuzzy_controller.angular_velocity.universe, 
                    self.fuzzy_controller.angular_velocity[label].mf, 
                    label=label, linewidth=2)
        ax2.set_title('Velocidad Angular')
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        
        ax3 = fig.add_subplot(223)
        for label in self.fuzzy_controller.position.terms:
            ax3.plot(self.fuzzy_controller.position.universe, 
                    self.fuzzy_controller.position[label].mf, 
                    label=label, linewidth=2)
        ax3.set_title('Posición del Carro')
        ax3.legend()
        ax3.grid(True, alpha=0.3)
        
        ax4 = fig.add_subplot(224)
        for label in self.fuzzy_controller.force.terms:
            ax4.plot(self.fuzzy_controller.force.universe, 
                    self.fuzzy_controller.force[label].mf, 
                    label=label, linewidth=2)
        ax4.set_title('Fuerza de Control')
        ax4.legend()
        ax4.grid(True, alpha=0.3)
        
        fig.tight_layout()
        canvas.draw()
        
        close_btn = ttk.Button(main_frame, text="CERRAR", 
                             command=self.close_fuzzy_window)
        close_btn.pack(pady=10)

    def update_fuzzy_values(self):
        """Actualiza los valores en la ventana de control difuso."""
        if self.fuzzy_window_open:
            self.angle_var.set(f"Ángulo: {self.fuzzy_values['angle']:.4f} rad")
            self.ang_vel_var.set(f"Vel. Angular: {self.fuzzy_values['angular_velocity']:.4f} rad/s")
            self.position_var.set(f"Posición: {self.fuzzy_values['position']:.4f} m")
            self.force_var.set(f"Fuerza: {self.fuzzy_values['force']:.4f} N")

    def close_fuzzy_window(self):
        """Cierra la ventana de control difuso."""
        if self.fuzzy_window:
            self.fuzzy_window.destroy()
        self.fuzzy_window_open = False

    def dynamics(self, t, y, F):
        """
        Ecuaciones diferenciales del sistema optimizadas.
        """
        x, x_dot, theta, theta_dot = y
        sin_theta = np.sin(theta)
        cos_theta = np.cos(theta)
        
        # Ecuaciones optimizadas para mejor estabilidad numérica
        temp = (F + self.m * self.L * theta_dot**2 * sin_theta - self.b * x_dot) / (self.M + self.m)
        theta_ddot = (self.g * sin_theta - cos_theta * temp) / (self.L * (4/3 - self.m * cos_theta**2 / (self.M + self.m)))
        x_ddot = temp - (self.m * self.L * theta_ddot * cos_theta) / (self.M + self.m)
        
        return [x_dot, x_ddot, theta_dot, theta_ddot]

    def update(self, i):
        """
        Función de actualización con control difuso optimizado.
        """
        # Calcular fuerza de control
        if self.use_fuzzy_control:
            x, x_dot, theta, theta_dot = self.state
            
            # Guardar valores para visualización
            self.fuzzy_values = {
                'angle': theta,
                'angular_velocity': theta_dot,
                'position': x,
                'force': self.force
            }
            
            # Calcular fuerza usando controlador difuso optimizado
            self.force = self.fuzzy_controller.compute_force(theta, theta_dot, x)
            
            # Actualizar ventana si está abierta
            if self.fuzzy_window_open:
                self.update_fuzzy_values()
        else:
            # Control manual
            self.fuzzy_values = {
                'angle': self.state[2],
                'angular_velocity': self.state[3],
                'position': self.state[0],
                'force': self.force
            }
        
        # Resolver ecuaciones diferenciales
        sol = solve_ivp(self.dynamics, [0, self.dt], self.state, 
                       args=(self.force,), t_eval=[self.dt], method='RK45')
        new_state = sol.y[:, -1]
        
        # Limitar posición
        if abs(new_state[0]) > self.max_pos:
            new_state[0] = np.sign(new_state[0]) * self.max_pos
            new_state[1] = 0
        
        self.state = new_state
        
        # Guardar historial
        current_time = i * self.dt
        self.history['time'].append(current_time)
        self.history['angle'].append(self.state[2])
        self.history['position'].append(self.state[0])
        self.history['force'].append(self.force)
        self.history['angular_velocity'].append(self.state[3])
        
        # Limitar tamaño del historial
        if len(self.history['time']) > 1000:
            for key in self.history:
                self.history[key] = self.history[key][-1000:]
        
        # Actualizar gráficos
        x = self.state[0]
        theta = self.state[2]
        pendulum_x = x + self.L * np.sin(theta)
        pendulum_y = self.L * np.cos(theta)
        
        self.cart.set_xy([x-0.25, -0.06])
        self.pendulum_rod.set_data([x, pendulum_x], [0, pendulum_y])
        self.pendulum_bob.center = (pendulum_x, pendulum_y)
        self.status_text.set_text(self.get_status_text())
        
        return self.cart, self.pendulum_rod, self.pendulum_bob, self.status_text

    def get_status_text(self):
        """
        Texto de estado que incluye información del control difuso.
        """
        theta = self.state[2]
        theta_deg = np.degrees(theta) % 360
        if theta_deg > 180:
            theta_deg -= 360
        
        angular_vel_deg = np.degrees(self.state[3])
        
        control_mode = "DIFUSO OPTIMIZADO" if self.use_fuzzy_control else "MANUAL"
        
        return (
            f"Posición carro: {self.state[0]:>7.3f} m\n"
            f"Velocidad carro: {self.state[1]:>6.3f} m/s\n"
            f"Ángulo péndulo: {theta_deg:>6.2f}°\n"
            f"Veloc. angular: {angular_vel_deg:>6.2f}°/s\n"
            f"Fuerza aplicada: {self.force:>6.2f} N\n"
            f"Modo control: {control_mode}"
        )

    def on_key_press(self, event):
        """Maneja eventos de teclado."""
        if event.key == 'a':
            self.force = -self.force_mag
        elif event.key == 'd':
            self.force = self.force_mag

    def on_key_release(self, event):
        """Detiene el movimiento al soltar teclas."""
        if event.key in ['a', 'd']:
            self.force = 0.0

    def reset_simulation(self, event):
        """Reinicia la simulación."""
        self.state = np.array([0.0, 0.0, 0.05, 0.0])
        self.force = 0.0
        self.history = {'time': [], 'angle': [], 'position': [], 'force': [], 'angular_velocity': []}
        self.status_text.set_text(self.get_status_text())

    def show_equations_window(self, event=None):
        """Muestra ventana con ecuaciones."""
        if self.math_window_open:
            if self.math_window:
                self.math_window.lift()
            return
        
        self.math_window_open = True
        self.math_window = tk.Toplevel()
        self.math_window.title("Ecuaciones del Péndulo Invertido")
        self.math_window.geometry("800x600")
        self.math_window.protocol("WM_DELETE_WINDOW", self.close_math_window)
        
        main_frame = ttk.Frame(self.math_window)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=20, pady=20)
        
        title = ttk.Label(main_frame, text="ECUACIONES DEL PÉNDULO INVERTIDO", 
                         font=('Helvetica', 16, 'bold'))
        title.pack(pady=10)
        
        # Ecuaciones del sistema
        eq_text = """
        ECUACIONES DIFERENCIALES DEL SISTEMA:
        
        Variables de estado:
        - x: Posición del carro
        - ẋ: Velocidad del carro
        - θ: Ángulo del péndulo (desde la vertical)
        - θ̇: Velocidad angular del péndulo
        
        Ecuaciones de movimiento:
        
        (M + m)ẍ + mLθ̈cosθ - mLθ̇²sinθ + bẋ = F
        
        mLẍcosθ + (4/3)mL²θ̈ - mgLsinθ = 0
        
        Donde:
        - M = Masa del carro
        - m = Masa del péndulo
        - L = Longitud del péndulo
        - b = Coeficiente de fricción
        - g = Aceleración gravitacional
        - F = Fuerza aplicada
        
        Forma simplificada para simulación:
        
        ẍ = [F + mL(θ̇²sinθ - θ̈cosθ) - bẋ] / (M + m)
        
        θ̈ = [g sinθ - (F cosθ)/(M + m)] / [L(4/3 - m cos²θ/(M + m))]
        """
        
        eq_label = ttk.Label(main_frame, text=eq_text, justify=tk.LEFT,
                            font=('Courier', 10))
        eq_label.pack(pady=10, fill=tk.BOTH, expand=True)
        
        close_btn = ttk.Button(main_frame, text="CERRAR", 
                             command=self.close_math_window)
        close_btn.pack(pady=10)

    def close_math_window(self):
        """Cierra ventana de ecuaciones."""
        if self.math_window:
            self.math_window.destroy()
        self.math_window_open = False

    def run(self):
        """Inicia la simulación."""
        self.ani = animation.FuncAnimation(
            self.fig, self.update, 
            frames=1000,
            interval=self.dt*1000,
            blit=True,
            repeat=True
        )
        plt.show()

# Ejecutar la simulación
if __name__ == "__main__":
    print("Iniciando simulador de péndulo invertido con control difuso optimizado...")
    simulator = InvertedPendulumSimulator()
    simulator.run()