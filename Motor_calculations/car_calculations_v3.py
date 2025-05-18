import math
import json
import numpy as np
from dataclasses import dataclass
from typing import Dict, List

@dataclass
class VehicleParameters:
    acceleration: float
    mass: float
    tire_radius: float
    friction_constant: float
    gear_efficiency: float
    listed_power_rating: float
    revolutions_per_minute: float
    acceleration_due_to_gravity: float = 9.80665

class MotorRequirementsCalculator:
    def __init__(self, params: VehicleParameters):
        self.params = params

    def calculate_forces(self) -> Dict[str, float]:
        """Calculate friction and acceleration forces"""
        friction_force = (self.params.mass * self.params.friction_constant * 
                         self.params.acceleration_due_to_gravity)
        acceleration_force = self.params.mass * self.params.acceleration
        return {
            'friction': friction_force,
            'acceleration': acceleration_force,
            'total': friction_force + acceleration_force
        }

    def calculate_torque(self) -> float:
        """Calculate required torque including safety factor"""
        forces = self.calculate_forces()
        return ((forces['total'] * self.params.tire_radius) / 
                self.params.gear_efficiency) * 1.1

    def calculate_rpm(self, velocity: float) -> float:
        """Calculate required RPM for given velocity"""
        return (velocity * 60) / (2 * math.pi * self.params.tire_radius)

    def analyze_speed_range(self, max_speed: float, steps: int = 10) -> Dict[str, List[float]]:
        """Analyze requirements across speed range"""
        speeds = np.linspace(0, max_speed, steps)
        results = {
            'speeds': speeds,
            'torque': [],
            'rpm': [],
            'power': []
        }
        
        for speed in speeds:
            torque = self.calculate_torque()
            rpm = self.calculate_rpm(speed)
            power = (torque * rpm * math.pi) / 30
            
            results['torque'].append(torque)
            results['rpm'].append(rpm)
            results['power'].append(power)
            
        return results

    def analyze_motor_capability(self, input_power: float) -> Dict[str, float]:
        """
        Analyze achievable speed given input power
        Args:
            input_power: Power rating of motor in Watts
        """
        # Calculate initial torque from input power
        initial_torque = (30/math.pi) * (input_power / self.params.revolutions_per_minute)
        
        # Iterate to find maximum achievable speed
        speed = 0
        step = 0.1
        while True:
            required_rpm = self.calculate_rpm(speed)
            required_torque = self.calculate_torque()
            current_power = (required_torque * required_rpm * math.pi) / 30
            
            if current_power > input_power:
                break
            speed += step
        
        gear_ratio = required_torque / initial_torque
        achieved_speed = speed - step  # Step back to last valid speed
        
        return {
            'achievable_speed': achieved_speed,
            'required_torque': required_torque,
            'required_rpm': self.calculate_rpm(achieved_speed),
            'gear_ratio': gear_ratio
        }

def main():
    # Load existing JSON configuration
    with open("Motor_calculations/car_and_motor_val.json") as f:
        data = json.load(f)
        params = data["car_parameters"]
        mass_data = data["mass_breakdown"]

    # Create vehicle parameters from JSON
    vehicle = VehicleParameters(
        acceleration=params["acceleration"],
        mass=params["mass"],
        tire_radius=params["tire_radius"],
        friction_constant=params["friction_constant"],
        gear_efficiency=params["gear_efficiency"],
        listed_power_rating=params["listed_power_rating"],
        revolutions_per_minute=params["revolutions_per_minute"]
    )

    calculator = MotorRequirementsCalculator(vehicle)

    # Mode selection
    print("\nSelect analysis mode:")
    print("1: Analyze speed range requirements")
    print("2: Analyze motor capability")
    mode = input("Enter mode (1 or 2): ")

    if mode == "1":
        max_speed_kmh = float(input("Enter maximum speed (km/h): "))
        max_speed_ms = max_speed_kmh / 3.6  # Convert km/h to m/s
        results = calculator.analyze_speed_range(max_speed_ms)
        
        print("\n=== Speed Range Analysis ===")
        for i, speed in enumerate(results['speeds']):
            print(f"\nAt {speed*3.6:.1f} km/h:")  # Display in km/h
            print(f"Required Torque: {results['torque'][i]:.2f} Nm")
            print(f"Required RPM: {results['rpm'][i]:.2f}")
            print(f"Required Power: {results['power'][i]:.2f} W")

    elif mode == "2":
        input_power = vehicle.listed_power_rating
        capability = calculator.analyze_motor_capability(input_power)
        
        print("\n=== Motor Capability Analysis ===")
        print(f"Input Power: {input_power:.2f} W")
        print(f"Achievable Speed: {capability['achievable_speed']*3.6:.1f} km/h")
        print(f"Required Torque: {capability['required_torque']:.2f} Nm")
        print(f"Required RPM: {capability['required_rpm']:.2f}")
        print(f"Required Gear Ratio: {capability['gear_ratio']:.2f}")
        print(f"Total Vehicle Mass: {mass_data['Total']} kg")

if __name__ == "__main__":
    main()

