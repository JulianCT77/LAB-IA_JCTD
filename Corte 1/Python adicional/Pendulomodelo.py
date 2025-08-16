import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from scipy.integrate import solve_ivp
from matplotlib.patches import Rectangle, Circle

class InvertedPendulum:
    def __init__(self, cart_mass=1.0, pole_mass=0.1, pole_length=0.5, 
                 gravity=9.81, friction_cart=0.1, friction_pivot=0.01, 
                 dt=0.02, max_force=15.0):
        """
        Inicializa el sistema del péndulo invertido
        
        Args:
            cart_mass: masa del carro (kg)
            pole_mass: masa del péndulo (kg)
            pole_length: longitud del péndulo (m)
            gravity: gravedad (m/s²)
            friction_cart: fricción del carro (kg/s)
            friction_pivot: fricción del pivote (kg·m²/s)
            dt: paso de tiempo (s)
            max_force: fuerza máxima aplicable (N)
        """
        # Parámetros físicos
        self.M = cart_mass
        self.m = pole_mass
        self.l = pole_length
        self.g = gravity
        self.b = friction_cart
        self.c = friction_pivot
        self.dt = dt
        self.max_force = max_force
        
        # Centro de masa y momento de inercia
        self.l_cm = pole_length / 2
        self.I = (1/12) * pole_mass * pole_length**2
        
        # Estado inicial [x, x_dot, theta, theta_dot]
        self.state = np.array([0.0, 0.0, np.pi + 0.1, 0.0])  # Ligeramente desviado
        
        # Configurar la figura
        self.fig = plt.figure(figsize=(14, 10))
        self.setup_plots()
        
        # Historial para guardar datos
        self.history = {
            'time': [],
            'states': [],
            'forces': [],
            'energy': []
        }
    
    def setup_plots(self):
        """Configura los subplots y elementos visuales"""
        gs = self.fig.add_gridspec(3, 2)
        
        # Subplot para la simulación
        self.ax_sim = self.fig.add_subplot(gs[0:2, 0])
        self.ax_sim.set_xlim([-2.5, 2.5])
        self.ax_sim.set_ylim([-0.5, 1.5])
        self.ax_sim.set_aspect('equal')
        self.ax_sim.grid(True)
        self.ax_sim.set_title('Simulación del Péndulo Invertido')
        
        # Subplots para análisis
        self.ax_math = self.fig.add_subplot(gs[0, 1])
        self.ax_pos = self.fig.add_subplot(gs[1, 1])
        self.ax_force = self.fig.add_subplot(gs[2, 0])
        self.ax_energy = self.fig.add_subplot(gs[2, 1])
        
        # Elementos de la simulación
        self.cart = Rectangle((0, 0), 0.4, 0.2, fc='blue', ec='black')
        self.pole_line, = self.ax_sim.plot([], [], 'r-', lw=3)
        self.pole_mass = Circle((0, 0), 0.05, fc='red')
        self.time_text = self.ax_sim.text(0.02, 0.95, '', transform=self.ax_sim.transAxes)
        
        self.ax_sim.add_patch(self.cart)
        self.ax_sim.add_patch(self.pole_mass)
        
        # Configurar ejes de análisis
        self.ax_math.axis('off')
        self.ax_pos.set_title('Posición y Ángulo vs Tiempo')
        self.ax_pos.grid(True)
        self.ax_force.set_title('Fuerza Aplicada vs Tiempo')
        self.ax_force.grid(True)
        self.ax_energy.set_title('Energía del Sistema vs Tiempo')
        self.ax_energy.grid(True)
    
    def compute_dynamics(self, state, force):
        """
        Calcula las derivadas del estado usando las ecuaciones de movimiento
        
        Args:
            state: [x, x_dot, theta, theta_dot]
            force: fuerza aplicada al carro (N)
            
        Returns:
            [x_dot, x_ddot, theta_dot, theta_ddot]
        """
        x, x_dot, theta, theta_dot = state
        sin_theta = np.sin(theta)
        cos_theta = np.cos(theta)
        
        # Denominador común
        den = (self.M + self.m)*(self.I + self.m*self.l_cm**2) - (self.m*self.l_cm*cos_theta)**2
        
        # Aceleración angular
        num_theta = (self.m*self.l_cm*cos_theta * (force - self.b*x_dot + self.m*self.l_cm*theta_dot**2*sin_theta) 
                    - (self.M + self.m)*(self.m*self.g*self.l_cm*sin_theta + self.c*theta_dot))
        theta_ddot = num_theta / den
        
        # Aceleración lineal
        num_x = ((self.I + self.m*self.l_cm**2)*(force - self.b*x_dot + self.m*self.l_cm*theta_dot**2*sin_theta) 
                - self.m*self.l_cm*cos_theta*(self.m*self.g*self.l_cm*sin_theta + self.c*theta_dot))
        x_ddot = num_x / den
        
        return [x_dot, x_ddot, theta_dot, theta_ddot]
    
    def compute_energy(self, state):
        """
        Calcula la energía total del sistema
        
        Args:
            state: [x, x_dot, theta, theta_dot]
            
        Returns:
            Energía total (J)
        """
        x, x_dot, theta, theta_dot = state
        cos_theta = np.cos(theta)
        sin_theta = np.sin(theta)
        
        # Energía cinética
        K_cart = 0.5 * self.M * x_dot**2
        vx = x_dot + self.l_cm * theta_dot * cos_theta
        vy = -self.l_cm * theta_dot * sin_theta
        K_pole = 0.5 * self.m * (vx**2 + vy**2) + 0.5 * self.I * theta_dot**2
        
        # Energía potencial
        U = self.m * self.g * self.l_cm * (1 + cos_theta)
        
        return K_cart + K_pole + U
    
    def lqr_controller(self, state):
        """
        Controlador LQR para estabilizar el péndulo
        
        Args:
            state: [x, x_dot, theta, theta_dot]
            
        Returns:
            Fuerza de control (N)
        """
        # Matriz de ganancia (pre-calculada)
        K = np.array([-2.0, -3.5, -50.0, -8.0])
        
        # Estado deseado [x=0, x_dot=0, theta=pi, theta_dot=0]
        desired = np.array([0.0, 0.0, np.pi, 0.0])
        
        # Error de estado
        error = state - desired
        error[2] = (error[2] + np.pi) % (2*np.pi) - np.pi  # Ajuste angular
        
        # Ley de control
        force = -np.dot(K, error)
        
        # Limitar fuerza
        return np.clip(force, -self.max_force, self.max_force)
    
    def update_simulation(self, i):
        """
        Actualiza la simulación para cada frame de la animación
        """
        t = i * self.dt
        
        # Calcular fuerza de control
        force = self.lqr_controller(self.state)
        
        # Integrar ecuaciones de movimiento
        sol = solve_ivp(lambda t, y: self.compute_dynamics(y, force),
                       [0, self.dt], self.state,
                       t_eval=[self.dt], method='RK45')
        
        # Actualizar estado
        self.state = sol.y[:, 0]
        self.state[0] = np.clip(self.state[0], -2.5, 2.5)  # Limitar posición
        
        # Guardar datos
        self.history['time'].append(t)
        self.history['states'].append(self.state.copy())
        self.history['forces'].append(force)
        self.history['energy'].append(self.compute_energy(self.state))
        
        # Actualizar visualización
        x, _, theta, _ = self.state
        pole_x = x + self.l * np.sin(theta)
        pole_y = self.l * np.cos(theta)
        
        self.cart.set_xy([x - 0.2, -0.1])
        self.pole_line.set_data([x, pole_x], [0, pole_y])
        self.pole_mass.center = (pole_x, pole_y)
        
        self.time_text.set_text(
            f'Tiempo: {t:.2f}s\n'
            f'Posición: {x:.3f}m\n'
            f'Ángulo: {np.degrees(theta-np.pi):.2f}°\n'
            f'Fuerza: {force:.2f}N'
        )
        
        # Actualizar gráficos de análisis
        if i > 0:
            self.update_analysis_plots()
        
        return self.cart, self.pole_line, self.pole_mass, self.time_text
    
    def update_analysis_plots(self):
        """Actualiza los gráficos de análisis"""
        times = np.array(self.history['time'])
        states = np.array(self.history['states'])
        forces = np.array(self.history['forces'])
        energy = np.array(self.history['energy'])
        
        # Limpiar y actualizar gráficos
        self.ax_pos.clear()
        self.ax_force.clear()
        self.ax_energy.clear()
        
        # Gráfico de posición y ángulo
        self.ax_pos.plot(times, states[:, 0], 'b-', label='Posición (m)')
        self.ax_pos.plot(times, np.degrees(states[:, 2]-np.pi), 'r-', label='Ángulo (°)')
        self.ax_pos.legend()
        self.ax_pos.grid(True)
        
        # Gráfico de fuerza
        self.ax_force.plot(times, forces, 'g-')
        self.ax_force.grid(True)
        self.ax_force.set_ylabel('Fuerza (N)')
        self.ax_force.set_xlabel('Tiempo (s)')
        
        # Gráfico de energía
        self.ax_energy.plot(times, energy, 'k-', label='Total')
        
        # Calcular energías cinética y potencial
        kinetic = energy - self.m*self.g*self.l_cm*(1 + np.cos(states[:, 2]))
        potential = self.m*self.g*self.l_cm*(1 + np.cos(states[:, 2]))
        
        self.ax_energy.plot(times, kinetic, 'm--', label='Cinética')
        self.ax_energy.plot(times, potential, 'c--', label='Potencial')
        self.ax_energy.legend()
        self.ax_energy.grid(True)
        self.ax_energy.set_ylabel('Energía (J)')
        self.ax_energy.set_xlabel('Tiempo (s)')
    
    def show_mathematical_analysis(self):
        """Muestra el análisis matemático en el subplot correspondiente"""
        # Ecuaciones de movimiento
        eq_motion = r"""
        $\begin{aligned}
        (M + m)\ddot{x} + ml\cos\theta\ddot{\theta} - ml\dot{\theta}^2\sin\theta + b\dot{x} &= F \\
        ml\cos\theta\ddot{x} + (I + ml^2)\ddot{\theta} + c\dot{\theta} - mgl\sin\theta &= 0
        \end{aligned}$
        """
        
        # Función de transferencia
        transfer_func = r"""
        $\frac{\Phi(s)}{F(s)} = \frac{mls^2}{[(M + m)s^2 + bs][(I + ml^2)s^2 + cs + mgl] - m^2l^2s^4}$
        """
        
        # Mostrar en el subplot
        self.ax_math.text(0.5, 0.8, "Ecuaciones de Movimiento:", fontsize=12, ha='center')
        self.ax_math.text(0.5, 0.6, eq_motion, fontsize=12, ha='center')
        self.ax_math.text(0.5, 0.3, "Función de Transferencia Linealizada:", fontsize=12, ha='center')
        self.ax_math.text(0.5, 0.1, transfer_func, fontsize=12, ha='center')
    
    def simulate(self, total_time=10.0):
        """
        Ejecuta la simulación completa
        
        Args:
            total_time: tiempo total de simulación (s)
        """
        # Mostrar análisis matemático
        self.show_mathematical_analysis()
        
        # Configurar animación
        frames = int(total_time / self.dt)
        self.ani = animation.FuncAnimation(
            self.fig, self.update_simulation, frames=frames,
            interval=self.dt*1000, blit=True
        )
        
        plt.tight_layout()
        plt.show()

# Ejecutar la simulación
if __name__ == "__main__":
    pendulum = InvertedPendulum(
        cart_mass=1.0,
        pole_mass=0.2,
        pole_length=0.6,
        friction_cart=0.1,
        friction_pivot=0.01,
        dt=0.01,
        max_force=20.0
    )
    pendulum.simulate(total_time=10.0)