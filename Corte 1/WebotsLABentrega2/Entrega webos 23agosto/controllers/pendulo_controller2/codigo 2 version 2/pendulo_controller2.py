from controller import Robot
import math
import time

class LowPassFilter:
    def __init__(self, alpha):
        self.alpha = alpha
        self.filtered_value = None
        
    def update(self, new_value):
        if self.filtered_value is None:
            self.filtered_value = new_value
        else:
            self.filtered_value = self.alpha * new_value + (1 - self.alpha) * self.filtered_value
        return self.filtered_value

class PIDController:
    def __init__(self, Kp, Ki, Kd, max_output, windup_limit=10.0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.max_output = max_output
        self.windup_limit = windup_limit
        self.reset()
    
    def reset(self):
        self.integral = 0
        self.prev_error = 0
        self.prev_time = time.time()
        self.prev_output = 0
    
    def compute(self, error, dt):
        if dt <= 0:
            return self.prev_output
        
        p = self.Kp * error
        self.integral += error * dt
        self.integral = max(-self.windup_limit, min(self.integral, self.windup_limit))
        i = self.Ki * self.integral
        d = self.Kd * (error - self.prev_error) / dt
        
        output = p + i + d
        output = max(-self.max_output, min(output, self.max_output))
        
        max_change = self.max_output * dt * 2
        if abs(output - self.prev_output) > max_change:
            output = self.prev_output + math.copysign(max_change, output - self.prev_output)
        
        self.prev_error = error
        self.prev_time = time.time()
        self.prev_output = output
        return output

class PenduloController:
    def __init__(self):
        self.robot = Robot()
        self.timestep = self.robot.getBasicTimeStep() / 1000.0
        
        self.motor_izq = self.robot.getDevice('motor_izq')
        self.motor_der = self.robot.getDevice('motor_der')
        self.sensor_pendulo = self.robot.getDevice('sensor_pendulo')
        self.gps = self.robot.getDevice('gps_carro')
        
        self.sensor_pendulo.enable(int(self.timestep * 1000))
        self.gps.enable(int(self.timestep * 1000))
        
        self.motor_izq.setPosition(float('inf'))
        self.motor_der.setPosition(float('inf'))
        self.motor_izq.setVelocity(0)
        self.motor_der.setVelocity(0)
        
        self.angle_filter = LowPassFilter(0.98)
        self.position_filter = LowPassFilter(0.9)
        
        self.pid_angle = PIDController(Kp=800, Ki=20, Kd=200, max_output=500, windup_limit=25)
        self.pid_position = PIDController(Kp=10, Ki=0.2, Kd=5, max_output=20, windup_limit=5)
        
        self.last_update = self.robot.getTime()
        self.integral_reset_counter = 0
        self.stabilized = False
        self.failure_count = 0
        self.recovery_mode = False
        self.last_position = 0
        self.velocity_estimate = 0
        self.control_active = False
        self.stabilization_count = 0

    def normalize_angle(self, angle):
        return ((angle + math.pi) % (2 * math.pi)) - math.pi

    def estimate_velocity(self, current_position, dt):
        if dt > 0:
            new_velocity = (current_position - self.last_position) / dt
            self.velocity_estimate = 0.8 * self.velocity_estimate + 0.2 * new_velocity
            self.last_position = current_position
        return self.velocity_estimate

    def check_failure(self, angle):
        angle_deg = math.degrees(abs(angle))
        if angle_deg > 15:
            self.failure_count += 1
            if self.failure_count > 2:
                return True
        else:
            self.failure_count = max(0, self.failure_count - 1)
        return False

    def recovery_procedure(self):
        print("Iniciando recuperación...")
        self.recovery_mode = True
        self.pid_angle.reset()
        self.pid_position.reset()
        
        for i in range(15):
            self.robot.step(int(self.timestep * 1000))
            angle = self.normalize_angle(self.sensor_pendulo.getValue())
            impulse = 50 * math.copysign(1, angle) * (0.5 + 0.3 * math.sin(i * 0.5))
            self.motor_izq.setTorque(-impulse)
            self.motor_der.setTorque(impulse)
        
        self.recovery_mode = False
        print("Recuperación completada")

    def run(self):
        print("=== CONTROL DE PÉNDULO INVERTIDO ===")
        
        # Fase inicial de calibración
        for i in range(30):  # Reducido a 30 iteraciones (~0.12s)
            self.robot.step(int(self.timestep * 1000))
            self.motor_izq.setTorque(-5)
            self.motor_der.setTorque(5)
        
        self.control_active = True
        print("Control activo")
        
        while self.robot.step(int(self.timestep * 1000)) != -1:
            raw_angle = self.sensor_pendulo.getValue()
            raw_position = self.gps.getValues()[0]
            
            angle = self.angle_filter.update(self.normalize_angle(raw_angle))
            position = self.position_filter.update(raw_position)
            
            current_time = self.robot.getTime()
            dt = max(0.001, current_time - self.last_update)
            self.last_update = current_time
            
            velocity = self.estimate_velocity(position, dt)
            
            if self.check_failure(angle):
                self.recovery_procedure()
                continue
            
            angle_error = -angle
            position_error = -position if abs(position) < 0.5 else 0  # Limitar posición a ±0.5m
            
            if abs(angle_error) < 0.015 and abs(position_error) < 0.03 and self.control_active:
                self.integral_reset_counter += 1
                if self.integral_reset_counter > 30:
                    self.pid_angle.integral *= 0.8
                    self.pid_position.integral *= 0.8
                    self.integral_reset_counter = 0
            else:
                self.integral_reset_counter = 0
            
            force_angle = self.pid_angle.compute(angle_error, dt)
            force_position = self.pid_position.compute(position_error, dt)
            
            damping = -3.0 * velocity
            position_weight = 0.1 if abs(angle_error) > 0.02 else 0.3  # Priorizar ángulo
            total_force = force_angle + force_position * position_weight + damping
            
            torque = total_force * 0.3
            
            self.motor_izq.setTorque(-torque)
            self.motor_der.setTorque(torque)
            
            angle_deg = math.degrees(angle)
            print(f"Tiempo: {current_time:.2f}s | Ángulo: {angle_deg:7.2f}° | Pos: {position:7.3f}m | Vel: {velocity:6.3f}m/s | Torque: {torque:7.2f}N*m | AngleError: {angle_error:.3f}")
            if abs(angle_deg) < 0.5 and abs(position) < 0.03 and self.control_active:
                self.stabilization_count += 1
                if self.stabilization_count > 50:  # Requiere 50 iteraciones (~0.2s)
                    if not self.stabilized:
                        print("¡Sistema estabilizado!")
                        self.stabilized = True
            else:
                self.stabilization_count = 0
                self.stabilized = False

if __name__ == "__main__":
    controller = PenduloController()
    controller.run()