import math
import json


with open("mass_data.json") as f:
    data = json.load(f)

total_mass = data["mass_breakdown"]["Total"]
frame_mass = data["mass_breakdown"]["Frame"]
person_data = data["mass_breakdown"]["Person"]

print(f"total mass: {total_mass} \n")
print(f"frame mass: {frame_mass} \n")
print(f"persoon data: {person_data} \n")


'''
Acceleration (m/s^2): 0.7
Mass (kg): 300
Tire Radius (m): 0.25
Friction Constant (n/a): 0.02
Gear Efficiency (0-1): 0.85
'''

class Velox:
    def __init__(self, acceleration, mass, tire_radius, friction_constant, gear_efficiency, listed_power_rating, revolutions_per_minute, velocity, acceleration_due_to_gravity=9.80665, ):
        self.acceleration = acceleration
        self.mass = mass
        self.tire_radius = tire_radius
        self.friction_constant = friction_constant
        self.gear_efficiency = gear_efficiency
        self.listed_power_rating = listed_power_rating
        self.revolutions_per_minute = revolutions_per_minute
        self.velocity = velocity
        self.acceleration_due_to_gravity = acceleration_due_to_gravity

    def total_force(self):
        friction_force = self.mass * self.friction_constant * self.acceleration_due_to_gravity
        acceleration_force = self.mass *self.acceleration
        return friction_force + acceleration_force
    
    def initial_torque(self):
        return (30/math.pi)*(self.listed_power_rating/self.revolutions_per_minute)

    # Torque equals to Friction * Tire Radius / gear efficiency * (1 + 0.1)
    def final_torque(self):
        return self.friction_force*{self.tire_radius}/(self.gear_efficiency*(1+0.1))

    def torque_w_acc(self):
        return ((self.total_force * self.mass * self.acceleration) * self.tire_radius / self.gear_efficiency) * (1+0.1) # this is also target torque

    def target_rpm(self):
        return (self.listed_power_rating/self.torque_w_acc)*(30/math.pi)
    
    def target_power_rating(self):
        return (self.torque_w_acc*math.pi/30)*self.torque_w_acc

    def gear_ratio(self):
       gear_ratio = self.torque_w_acc/self.calculated_torque
       driving_gear_teeth = 8.0
       driven_gear_teeth = gear_ratio*driving_gear_teeth
       return driven_gear_teeth
    
    def v_ms(self):
        v_ms = self.velocity / 3.6 # conversion of velocity m/s
        return v_ms

    def angular_velocity(self):
        angular_velocity = self.v_ms / self.tire_radius # angular velocity 
        return angular_velocity
    
    def rpm_angular(self): # calculating rpm through angular velocity
        return (self.angular_velocity*60)/(2*math.pi)
        
acceleration = float(input("Acceleration for the car (m/s^2): "))         
mass = float(input("Mass for the car (kg): ")) # mass of total with frame, battery, misc, & 2 people
tire_radius = float(input("Tire radius for the car (m): "))
friction_constant = float(input("Friction constant (n/a): "))
gear_efficiency = float(input("Gear Efficiency (0-1): "))
listed_power_rating = int(input("Listed Power Rating (watts): "))
revolutions_per_minute = int(input("Listed Revolutions Per Minute (rotations/minutex): "))
# velocity = float(input("Velocity of the car (km/h): "))

velox = Velox(acceleration, mass, tire_radius, friction_constant, gear_efficiency, listed_power_rating, revolutions_per_minute)


