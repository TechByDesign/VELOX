import numpy as np
import json
import matplotlib.pyplot as plt

class Car_Calculations:
    def __init__(self, mass, radius, time, friction_coeff, max_velocity, g=9.8):
        self.mass = mass
        self.radius = radius
        self.time = time
        self.friction_coeff = friction_coeff
        self.max_velocity = max_velocity
        self.g = g

    def friction_force(self):
        return self.friction_coeff * self.mass * self.g

    def thrust_force(self, velocity):
        return self.mass * velocity / self.time  # F = ma = m(v/t)

    def total_force(self, velocity):
        return self.friction_force() + self.thrust_force(velocity)

    def torque(self, velocity):
        return self.radius * self.total_force(velocity)

    def rpm(self, velocity):
        return (60 * velocity) / (2 * np.pi * self.radius)

    def power(self, velocity):
        return (self.torque(velocity) * self.rpm(velocity) * np.pi) / 30  # Watts

    def power_derivative(self, velocity):
        # dP/dv = 2*(m/t)*v + μ*m*g
        return 2 * (self.mass / self.time) * velocity + self.friction_coeff * self.mass * self.g

    def plot_power_x_velocity(self):
        velocities = np.linspace(0, self.max_velocity, 100)
        powers = [self.power(v) for v in velocities]
        derivatives = [self.power_derivative(v) for v in velocities]

        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 12))
        
        # Plot Power vs. Velocity
        ax1.plot(velocities, powers, 'b-', linewidth=2)
        ax1.set_title("Electric Car: Velocity vs. Power", fontsize=14)
        ax1.set_xlabel("Velocity (m/s)", fontsize=12)
        ax1.set_ylabel("Power (W)", fontsize=12)
        ax1.grid(True, linestyle='--', alpha=0.6)

        plt.tight_layout()
        plt.show()

# User input
with open('Motor_calculations/car_and_motor_val.json', 'r') as f:
    data = json.load(f)
mass = data["mass"]  # e.g., 300
radius = data["radius"]  # e.g., 0.25
time = data["time"]  # e.g., 12
max_velocity = data["max_velocity"] # e.g., 10.5
friction_coeff = data["friction_coeff"]

# Create car instance and plot
car = Car_Calculations(mass, radius, time, friction_coeff, max_velocity)
car.plot_power_x_velocity()
