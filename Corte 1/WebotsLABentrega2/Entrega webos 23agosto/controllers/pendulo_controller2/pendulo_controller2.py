from controller import Robot
import math
import time

class PIDController:
    def __init__(self, Kp, Ki, Kd, max_output):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.max_output = max_output
        self.reset()
    
    def reset(self):
        self.integral = 0
        self.prev_error = 0
        self.prev_time = time.time()
    
    def compute(self, error):
        current_time = time.time()
        dt = current_time - self.prev_time
        
        if dt <= 0:
            return 0
        
        # Cálculo de términos PID
        p = self.Kp * error
        self.integral += error * dt
        i = self.Ki * self.integral
        d = self.Kd * (error - self.prev_error) / dt
        
        output = p + i + d
        
        # Limitar salida con suavizado
        output = max(-self.max_output, min(output, self.max_output))
        
        self.prev_error = error
        self.prev_time = current_time
        
        return output

class PenduloController:
    def __init__(self):
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())
        
        # Configuración de dispositivos
        self.motor_izq = self.robot.getDevice('motor_izq')
        self.motor_der = self.robot.getDevice('motor_der')
        self.sensor_pendulo = self.robot.getDevice('sensor_pendulo')
        self.gps = self.robot.getDevice('gps_carro')
        
        # Habilitar sensores
        self.sensor_pendulo.enable(self.timestep)
        self.gps.enable(self.timestep)
        
        # Configurar motores
        self.motor_izq.setPosition(float('inf'))
        self.motor_der.setPosition(float('inf'))
        self.motor_izq.setVelocity(0)
        self.motor_der.setVelocity(0)
        
        # Controladores PID optimizados
        self.pid_angle = PIDController(Kp=180, Ki=15, Kd=40, max_output=250)  # Control del ángulo
        self.pid_position = PIDController(Kp=12, Ki=0.5, Kd=8, max_output=30)  # Control de posición
        
        # Variables de estado
        self.last_update = 0
        self.integral_reset_counter = 0
        self.stabilized = False

    def run(self):
        print("=== SISTEMA DE CONTROL ACTIVADO ===")
        print("Estrategia: Control en cascada con prioridad al ángulo")
        
        # Fase de inicialización
        print("Inicializando...")
        for i in range(100):
            if i == 30:  # Pequeño impulso inicial
                self.motor_izq.setTorque(-5)
                self.motor_der.setTorque(5)
            elif i == 35:
                self.motor_izq.setTorque(0)
                self.motor_der.setTorque(0)
            self.robot.step(self.timestep)
        
        print("Control activo en marcha")
        
        while self.robot.step(self.timestep) != -1:
            # Leer sensores
            angle = self.sensor_pendulo.getValue()
            position = self.gps.getValues()[0]
            
            # Control en cascada (prioridad al ángulo)
            angle_error = -angle  # Objetivo: 0°
            position_error = -position  # Objetivo: 0m
            
            # Resetear integral si el error es muy pequeño
            if abs(angle_error) < 0.02 and abs(position_error) < 0.02:
                self.integral_reset_counter += 1
                if self.integral_reset_counter > 20:
                    self.pid_angle.integral = 0
                    self.pid_position.integral = 0
                    self.integral_reset_counter = 0
            else:
                self.integral_reset_counter = 0
            
            # Computar fuerzas de control
            force_angle = self.pid_angle.compute(angle_error) * 1.5  # Prioridad al ángulo
            force_position = self.pid_position.compute(position_error)
            
            # Combinación ponderada
            total_force = force_angle + force_position * 0.7  # Reducción de influencia de posición
            
            # Convertir a torque (radio de 0.05m)
            torque = total_force * 0.05
            
            # Aplicar torque a los motores
            self.motor_izq.setTorque(-torque)
            self.motor_der.setTorque(torque)
            
            # Monitoreo y visualización
            current_time = self.robot.getTime()
            if current_time - self.last_update > 0.5:
                angle_deg = math.degrees(angle)
                print(f"\nTiempo: {current_time:.2f}s")
                print(f"Ángulo: {angle_deg:7.2f}° | Posición: {position:7.3f}m")
                print(f"Fuerza Ángulo: {force_angle:7.2f}N | Fuerza Pos: {force_position:7.2f}N")
                print(f"Torque Aplicado: {torque:7.2f}N*m")
                
                # Verificar estabilización
                if abs(angle_deg) < 3 and abs(position) < 0.05:
                    if not self.stabilized:
                        print("¡Sistema estabilizado en posición cero!")
                        self.stabilized = True
                else:
                    self.stabilized = False
                
                self.last_update = current_time

if __name__ == "__main__":
    controller = PenduloController()
    controller.run()