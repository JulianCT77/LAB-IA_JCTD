import numpy as np  # Para operaciones numéricas y matemáticas
import matplotlib.pyplot as plt  # Para visualización gráfica
import matplotlib.animation as animation  # Para crear animaciones
from matplotlib.widgets import Button  # Para añadir botones interactivos
from scipy.integrate import solve_ivp  # Para resolver ecuaciones diferenciales
import matplotlib.patches as patches  # Para dibujar formas geométricas
import tkinter as tk  # Para crear interfaces gráficas
from tkinter import ttk  # Widgets temáticos de tkinter

class InvertedPendulumSimulator:
    def __init__(self):
        """
        Inicializa el simulador de péndulo invertido con parámetros físicos,
        estado inicial y configuración de la interfaz gráfica.
        """
        
        # ==============================================
        # PARÁMETROS FÍSICOS DEL SISTEMA
        # ==============================================
        self.g = 9.81       # Aceleración gravitacional [m/s²]
        self.m = 0.25       # Masa del péndulo [kg]
        self.M = 0.5        # Masa del carro [kg]
        self.L = 0.5       # Longitud del péndulo [m]
        self.b = 0.1        # Coeficiente de fricción del carro [N·s/m]
        self.force_mag = 10.0  # Magnitud máxima de fuerza aplicable [N]
        
        # Estado inicial del sistema [posición carro, velocidad carro, ángulo péndulo, velocidad angular]
        self.state = np.array([0.0, 0.0, 0.1, 0.0])  # Pequeña perturbación inicial en el ángulo
        
        # Variables de control
        self.force = 0.0    # Fuerza actual aplicada al carro
        self.dt = 0.02      # Paso de tiempo para la simulación [s]
        self.max_pos = 2.5  # Límite de posición del carro [m]
        
        # Configuración de las ventanas gráficas
        self.setup_main_window()  # Configura la ventana principal de simulación
        self.setup_control_panel()  # Configura los controles interactivos
        
        # Variables para la ventana de ecuaciones matemáticas
        self.math_window = None  # Referencia a la ventana de ecuaciones
        self.math_window_open = False  # Estado de la ventana de ecuaciones
        
        
        
        
    def pene(self,respuesta):
        
        self.respuesta=respuesta
        
        if respuesta == 'n' :
        
         return  print("mateoooooooooo")
        
     

    def setup_main_window(self):
        """
        Configura la ventana principal de visualización con todos los elementos gráficos:
        - Carril por donde se mueve el carro
        - Carro (rectángulo móvil)
        - Péndulo (varilla y masa)
        - Panel de información de estado
        - Eventos de teclado para control
        """
        
        # Configuración del estilo y figura principal
        plt.style.use('seaborn-v0_8-darkgrid')  # Nombre correcto en nuevas versiones
        self.fig, self.ax = plt.subplots(figsize=(12, 7))  # Crea figura y ejes
        self.fig.subplots_adjust(bottom=0.2)  # Ajusta espacio para controles
        self.fig.patch.set_facecolor('#f5f5f5')  # Color de fondo
        
        # Configuración del área de simulación
        self.ax.set_xlim(-3, 3)  # Límites en eje X
        self.ax.set_ylim(-0.2, 1.5)  # Límites en eje Y
        self.ax.set_facecolor('#fafafa')  # Color de fondo
        self.ax.grid(True, linestyle=':', alpha=0.7)  # Rejilla
        self.ax.set_title('SIMULADOR DE PÉNDULO INVERTIDO', 
                         fontsize=14, pad=20, color='#2c3e50', fontweight='bold')  # Título
        
        # ==============================================
        # ELEMENTOS GRÁFICOS PRINCIPALES
        # ==============================================
        
        # Carril por donde se mueve el carro (rectángulo gris)
        self.track = patches.Rectangle((-3, -0.1), 6, 0.05, 
                                     facecolor='#7f8c8d', edgecolor='#34495e', linewidth=1.5)
        self.ax.add_patch(self.track)
        
        # Carro (rectángulo azul)
        self.cart = patches.Rectangle((-0.3, -0.08), 0.6, 0.16, 
                                    facecolor='#3498db', edgecolor='#2980b9', linewidth=2)
        self.ax.add_patch(self.cart)
        
        # Varilla del péndulo (línea)
        self.pendulum_rod, = self.ax.plot([], [], '-', color='#2c3e50', lw=4)
        
        # Masa del péndulo (círculo rojo)
        self.pendulum_bob = patches.Circle((0, 0), 0.08, 
                                        facecolor='#e74c3c', edgecolor='#c0392b', linewidth=2)
        self.ax.add_patch(self.pendulum_bob)
        
        # ==============================================
        # PANEL DE INFORMACIÓN DE ESTADO
        # ==============================================
        self.status_text = self.ax.text(
            0.02, 0.95,  # Posición (coordenadas relativas)
            'Posición carro: 0.00 m\n'
            'Velocidad carro: 0.00 m/s\n'
            'Ángulo: 0.00°\n'
            'Velocidad angular: 0.00°/s\n'
            'Fuerza aplicada: 0.00 N',
            transform=self.ax.transAxes,  # Coordenadas relativas al eje
            bbox=dict(boxstyle='round', facecolor='white', alpha=0.9, 
                    edgecolor='#dddddd', pad=0.5),  # Caja con estilo
            fontfamily='monospace',  # Fuente monoespaciada
            fontsize=11,  # Tamaño de fuente
            verticalalignment='top'  # Alineación vertical
        )
        
        # ==============================================
        # EVENTOS DE TECLADO PARA CONTROL
        # ==============================================
        self.fig.canvas.mpl_connect('key_press_event', self.on_key_press)
        self.fig.canvas.mpl_connect('key_release_event', self.on_key_release)

    def setup_control_panel(self):
        """
        Configura los botones de control en la parte inferior de la ventana principal:
        - Botón de reinicio
        - Botón para mostrar ecuaciones matemáticas
        """
        
        # Botón de reinicio (verde)
        reset_ax = plt.axes([0.3, 0.05, 0.2, 0.075])  # Define área del botón
        self.btn_reset = Button(reset_ax, 'REINICIAR', 
                              color='#2ecc71', hovercolor='#27ae60')
        self.btn_reset.label.set_fontweight('bold')  # Texto en negrita
        self.btn_reset.on_clicked(self.reset_simulation)  # Asigna función al click
        
        # Botón de ecuaciones (morado)
        eq_ax = plt.axes([0.55, 0.05, 0.2, 0.075])  # Define área del botón
        self.btn_eq = Button(eq_ax, 'ECUACIONES', 
                           color='#9b59b6', hovercolor='#8e44ad')
        self.btn_eq.label.set_fontweight('bold')  # Texto en negrita
        self.btn_eq.on_clicked(self.show_equations_window)  # Asigna función al click

    def show_equations_window(self, event=None):
        """
        Muestra una ventana secundaria con las ecuaciones matemáticas que gobiernan
        el sistema y los parámetros físicos.
        """
        
        # Evita abrir múltiples ventanas
        if self.math_window_open:
            if self.math_window:
                self.math_window.lift()  # Trae al frente si ya está abierta
            return
        
        self.math_window_open = True
        self.math_window = tk.Toplevel()  # Crea ventana secundaria
        self.math_window.title("Modelo Matemático")
        self.math_window.geometry("800x650")  # Tamaño de la ventana
        self.math_window.protocol("WM_DELETE_WINDOW", self.close_math_window)  # Manejo de cierre
        
        # Frame principal (contenedor)
        main_frame = ttk.Frame(self.math_window)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=20, pady=20)
        
        # Título de la ventana
        title = ttk.Label(main_frame, text="MODELO MATEMÁTICO", 
                         font=('Helvetica', 14, 'bold'))
        title.pack(pady=10)
        
        # ==============================================
        # PANEL DE PARÁMETROS FÍSICOS
        # ==============================================
        params_frame = ttk.LabelFrame(main_frame, text="Parámetros del Sistema", 
                                    padding=10)
        params_frame.pack(fill=tk.X, pady=10)
        
        # Lista de parámetros con sus valores
        params = [
            ("Gravedad (g):", f"{self.g} m/s²"),
            ("Masa péndulo (m):", f"{self.m} kg"),
            ("Masa carro (M):", f"{self.M} kg"),
            ("Longitud (L):", f"{self.L} m"),
            ("Fricción (b):", f"{self.b} N·s/m"),
            ("Fuerza máxima:", f"{self.force_mag} N"),
            ("Paso tiempo:", f"{self.dt} s")
        ]
        
        # Crea etiquetas para cada parámetro
        for name, value in params:
            frame = ttk.Frame(params_frame)
            frame.pack(fill=tk.X, padx=5, pady=2)
            ttk.Label(frame, text=name, width=20, anchor='e').pack(side=tk.LEFT)
            ttk.Label(frame, text=value, font=('Courier', 10)).pack(side=tk.LEFT)
        
        # ==============================================
        # PANEL DE ECUACIONES
        # ==============================================
        eq_frame = ttk.LabelFrame(main_frame, text="Ecuaciones del Movimiento", 
                                padding=10)
        eq_frame.pack(fill=tk.BOTH, expand=True, pady=10)
        
        # Texto con las ecuaciones formateadas
        eq_text = """
        Sistema de ecuaciones diferenciales:
        
        ẍ = [F - bẋ + mLθ̇²sinθ - mg sinθ cosθ] / [M + m sin²θ]
        
        θ̈ = [g sinθ - cosθ (F - bẋ + mLθ̇²sinθ)/(M + m sin²θ)] / L
        
        
        Variables de estado:
        
        x₁ = posición del carro (x)
        x₂ = velocidad del carro (ẋ)
        x₃ = ángulo del péndulo (θ) [rad]
        x₄ = velocidad angular (θ̇) [rad/s]
        
        
        Representación en espacio de estados:
        
        dx₁/dt = x₂
        
        dx₂/dt = [F - b x₂ + m L x₄² sin x₃ - m g sin x₃ cos x₃] / [M + m sin² x₃]
        
        dx₃/dt = x₄
        
        dx₄/dt = [g sin x₃ - cos x₃ (F - b x₂ + m L x₄² sin x₃)/(M + m sin² x₃)] / L
        """
        
        # Widget de texto para mostrar las ecuaciones
        text = tk.Text(eq_frame, font=('Courier', 11), wrap=tk.WORD, padx=10, pady=10)
        text.insert(tk.END, eq_text)
        text.config(state=tk.DISABLED)  # Hace el texto de solo lectura
        text.pack(fill=tk.BOTH, expand=True)
        
        # Botón para cerrar la ventana
        close_btn = ttk.Button(main_frame, text="CERRAR", 
                             command=self.close_math_window, style='Accent.TButton')
        close_btn.pack(pady=10)
        
        # Estilo para el botón de cerrar
        style = ttk.Style()
        style.configure('Accent.TButton', foreground='white', background='#e74c3c')

    def close_math_window(self):
        """Cierra la ventana de ecuaciones matemáticas y actualiza el estado"""
        if self.math_window:
            self.math_window.destroy()
        self.math_window_open = False

    def get_status_text(self):
        """
        Genera el texto con el estado actual del sistema para mostrarlo en el panel.
        
        Returns:
            str: Texto formateado con posición, velocidad, ángulo, velocidad angular y fuerza aplicada.
        """
        
        # Convierte el ángulo a grados y lo ajusta al rango [-180°, 180°]
        theta = self.state[2]
        theta_deg = np.degrees(theta) % 360
        if theta_deg > 180:
            theta_deg -= 360
        
        # Convierte la velocidad angular de rad/s a °/s
        angular_vel_deg = np.degrees(self.state[3])
        
        # Texto formateado con alineación derecha para los valores
        return (
            f"Posición carro: {self.state[0]:>7.3f} m\n"
            f"Velocidad carro: {self.state[1]:>6.3f} m/s\n"
            f"Ángulo péndulo: {theta_deg:>6.2f}°\n"
            f"Veloc. angular: {angular_vel_deg:>6.2f}°/s\n"
            f"Fuerza aplicada: {self.force:>6.2f} N"
        )

    def dynamics(self, t, y, F):
        """
        Define las ecuaciones diferenciales que gobiernan el sistema del péndulo invertido.
        
        Args:
            t (float): Tiempo actual (no se usa explícitamente, necesario para solve_ivp)
            y (list): Vector de estado [x, x_dot, theta, theta_dot]
            F (float): Fuerza aplicada al carro
            
        Returns:
            list: Derivadas del vector de estado [dx/dt, d²x/dt², dθ/dt, d²θ/dt²]
        """
        
        x, x_dot, theta, theta_dot = y  # Desempaqueta el vector de estado
        
        # Pre-cálculo de términos trigonométricos
        sin_theta = np.sin(theta)
        cos_theta = np.cos(theta)
        
        # Denominador común para ambas ecuaciones
        denom = self.M + self.m*sin_theta**2
        
        # Ecuación para la aceleración del carro (d²x/dt²)
        x_ddot = (F - self.b*x_dot + self.m*self.L*theta_dot**2*sin_theta 
                 - self.m*self.g*sin_theta*cos_theta) / denom
        
        # Ecuación para la aceleración angular del péndulo (d²θ/dt²)
        theta_ddot = (self.g*sin_theta - cos_theta*(F - self.b*x_dot 
                     + self.m*self.L*theta_dot**2*sin_theta)/denom) / self.L
        
        return [x_dot, x_ddot, theta_dot, theta_ddot]

    def update(self, i):
        """
        Función de actualización para la animación. Se ejecuta en cada frame.
        
        Args:
            i (int): Número de frame (no se usa directamente)
            
        Returns:
            tuple: Elementos gráficos actualizados para la animación
        """
        
        # Resuelve las ecuaciones diferenciales para el paso de tiempo actual
        sol = solve_ivp(self.dynamics, [0, self.dt], self.state, 
                       args=(self.force,), t_eval=[self.dt])
        new_state = sol.y[:, -1]  # Obtiene el estado al final del paso de tiempo
        
        # Limita la posición del carro para que no salga de los límites
        if abs(new_state[0]) > self.max_pos:
            new_state[0] = np.sign(new_state[0]) * self.max_pos
            new_state[1] = 0  # Detiene el carro al llegar al límite
        
        self.state = new_state  # Actualiza el estado del sistema
        
        # ==============================================
        # ACTUALIZACIÓN DE ELEMENTOS GRÁFICOS
        # ==============================================
        
        # Calcula posición del péndulo
        x = self.state[0]  # Posición actual del carro
        theta = self.state[2]  # Ángulo actual del péndulo
        pendulum_x = x + self.L * np.sin(theta)  # Posición x de la masa
        pendulum_y = self.L * np.cos(theta)  # Posición y de la masa
        
        # Actualiza posición del carro
        self.cart.set_xy([x-0.3, -0.08])  # El carro es un rectángulo de ancho 0.6
        
        # Actualiza posición de la varilla del péndulo
        self.pendulum_rod.set_data([x, pendulum_x], [0, pendulum_y])
        
        # Actualiza posición de la masa del péndulo
        self.pendulum_bob.center = (pendulum_x, pendulum_y)
        
        # Actualiza el texto del panel de estado
        self.status_text.set_text(self.get_status_text())
        
        # Devuelve los elementos que deben ser actualizados en la animación
        return self.cart, self.pendulum_rod, self.pendulum_bob, self.status_text

    def on_key_press(self, event):
        """
        Maneja eventos de teclado para controlar el carro.
        - Tecla 'a': aplica fuerza hacia la izquierda
        - Tecla 'd': aplica fuerza hacia la derecha
        """
        if event.key == 'a':
            self.force = -self.force_mag+1  # Fuerza negativa (izquierda)ffffff
        elif event.key == 'd':
            self.force = self.force_mag-1  # Fuerza positiva (derecha)               ffff

    def on_key_release(self, event):
        """
        Detiene el movimiento cuando se sueltan las teclas de control.
        """
        if event.key in ['a', 'd']:
            self.force = 0.0  # Cesa la fuerza aplicada

    def reset_simulation(self, event):
        """
        Reinicia la simulación al estado inicial.
        """
        self.state = np.array([0.0, 0.0, 0.1, 0.0])  # Estado inicial con pequeña perturbación
        self.force = 0.0  # Fuerza a cero
        
        # Actualiza inmediatamente el panel de estado
        self.status_text.set_text(self.get_status_text())

    def run(self):
        """
        Inicia la simulación y la animación.
        """
        # Crea la animación llamando a la función update cada dt milisegundos
        self.ani = animation.FuncAnimation(
            self.fig, self.update, 
            frames=1000,  # Número máximo de frames
            interval=self.dt*1000,  # Intervalo entre frames en ms
            blit=True,  # Optimización para solo redibujar partes cambiadas
            repeat=True  # Repite la animación indefinidamente
        )
        plt.show()  # Muestra la ventana principal