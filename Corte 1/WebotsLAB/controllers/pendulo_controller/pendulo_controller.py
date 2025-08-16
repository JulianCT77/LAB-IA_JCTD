from controller import Robot, Keyboard
import math

class ControladorPenduloInvertido:
    def __init__(self):
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())
        
        # Parámetros físicos (DEBEN coincidir con el modelo VRML)
        self.g = 9.81       # Gravedad [m/s²]
        self.m = 0.5        # Masa de la bola [kg]
        self.M = 5.0        # Masa del carro [kg]
        self.L = 0.25       # Longitud del péndulo [m] (de la articulación al centro de masa)
        self.l = 0.5        # Longitud total del brazo [m]
        self.r = 0.04       # Radio de las ruedas [m]
        
        # Configuración inicial
        self.setup_devices()
        self.setup_control()
        self.reset_state()
        
        print(f"Parámetros físicos cargados: m={self.m} kg, M={self.M} kg, L={self.L} m")

    def setup_devices(self):
        self.keyboard = Keyboard()
        self.keyboard.enable(self.timestep)
        
        # Motores
        self.motor_izq = self.robot.getDevice('rueda_izq')
        self.motor_der = self.robot.getDevice('rueda_der')
        self.motor_izq.setPosition(float('inf'))
        self.motor_der.setPosition(float('inf'))
        self.motor_izq.setVelocity(0)
        self.motor_der.setVelocity(0)
        
        # Sensores
        self.gps = self.robot.getDevice('gps_carro')
        self.gps.enable(self.timestep)
        self.sensor_pendulo = self.robot.getDevice('sensor_pendulo')
        self.sensor_pendulo.enable(self.timestep)

    def setup_control(self):
        # Parámetros de control
        self.max_torque = 10.0     # [Nm]
        self.force_mag_manual = 40.0  # [N]
        self.force_limit = 60.0    # [N]
        
        # Control PID (ajustados para m=0.5, M=5.0, L=0.25)
        self.Kp = 150.0
        self.Ki = 2.0
        self.Kd = 30.0

    def reset_state(self):
        self.prev_time = self.robot.getTime()
        self.prev_position = self.gps.getValues()[0]
        self.prev_angle = self.sensor_pendulo.getValue()
        self.integral_error = 0.0
        self.prev_error = 0.0

    def read_keyboard_force(self):
        key = self.keyboard.getKey()
        if key == ord('A') or key == ord('a') or key == Keyboard.LEFT:
            return -self.force_mag_manual
        elif key == ord('D') or key == ord('d') or key == Keyboard.RIGHT:
            return self.force_mag_manual
        return 0.0

    def pid_control(self, theta, dt):
        # Normalizar ángulo (0 = vertical hacia arriba)
        error = (theta + math.pi) % (2 * math.pi) - math.pi
        
        # PID
        self.integral_error += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        self.prev_error = error
        
        # Fuerza de control basada en modelo físico
        force = -(self.Kp * error + self.Ki * self.integral_error + self.Kd * derivative)
        
        # Considerar dinámica del sistema (opcional, avanzado)
        # force *= (self.M + self.m)  # Compensar masa total
        
        return max(min(force, self.force_limit), -self.force_limit)

    def apply_force_to_wheels(self, force):
        # Convertir fuerza lineal a torque en ruedas: τ = F * r
        torque = force * self.r
        torque = max(min(torque, self.max_torque), -self.max_torque)
        self.motor_izq.setTorque(torque)
        self.motor_der.setTorque(torque)

    def update_state(self):
        t = self.robot.getTime()
        dt = t - self.prev_time if t > self.prev_time else self.timestep/1000.0
        pos = self.gps.getValues()[0]
        theta = self.sensor_pendulo.getValue()
        
        # Cálculo de velocidades
        x_dot = (pos - self.prev_position) / dt if dt > 0 else 0.0
        theta_dot = (theta - self.prev_angle) / dt if dt > 0 else 0.0
        
        self.prev_time = t
        self.prev_position = pos
        self.prev_angle = theta
        
        return pos, x_dot, theta, theta_dot, dt

    def print_status(self, pos, x_dot, theta, theta_dot, force):
        theta_deg = math.degrees((theta + math.pi) % (2*math.pi) - math.pi)
        print(f"Pos: {pos:.3f}m | Vel: {x_dot:.3f}m/s | θ: {theta_deg:6.2f}° | F: {force:6.2f}N")

    def run(self):
        # Esperar inicialización física
        for _ in range(10):
            self.robot.step(self.timestep)
        
        print("Control iniciado. Usa A/D para mover el carro")
        while self.robot.step(self.timestep) != -1:
            pos, x_dot, theta, theta_dot, dt = self.update_state()

            f_manual = self.read_keyboard_force()
            f_pid = self.pid_control(theta, dt)
            f_total = max(min(f_pid + f_manual, self.force_limit), -self.force_limit)
            
            self.apply_force_to_wheels(f_total)
            
            if int(self.robot.getTime() * 100) % 10 == 0:
                self.print_status(pos, x_dot, theta, theta_dot, f_total)

if __name__ == "__main__":
    controller = ControladorPenduloInvertido()
    controller.run()