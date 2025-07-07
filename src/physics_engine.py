"""
Physics Engine Module

Implements 6-DoF flight dynamics simulation including gravitational forces,
aerodynamic calculations, and propulsion modeling.
"""

import numpy as np
from scipy.spatial.transform import Rotation
from typing import Tuple, Dict, Any
import math

from .drone_model import DroneModel
from .environment import Environment


class PhysicsEngine:
    """
    Physics engine for drone flight simulation.
    
    Implements:
    - 6-DoF flight dynamics
    - Gravitational force calculations
    - Aerodynamic force and moment calculations
    - Propulsion modeling
    - Numerical integration
    """
    
    def __init__(self, drone: DroneModel, environment: Environment):
        """
        Initialize physics engine.
        
        Args:
            drone: Drone model instance
            environment: Environment instance
        """
        self.drone = drone
        self.environment = environment
        
        # Integration method
        self.integration_method = "rk4"  # or "euler"
        
        # Aerodynamic coefficients cache
        self._aero_cache = {}
        
        # Physical constants
        self.AIR_DENSITY_SEA_LEVEL = 1.225  # kg/m³
        self.GRAVITY_EARTH = 9.81  # m/s²
    
    def step(self, dt: float, control_inputs: Dict[str, float]) -> Dict[str, Any]:
        """
        Perform one physics simulation step.
        
        Args:
            dt: Time step in seconds
            control_inputs: Control inputs dictionary
            
        Returns:
            Dictionary with force and moment information
        """
        # Calculate forces and moments
        forces = self._calculate_forces(control_inputs)
        moments = self._calculate_moments(control_inputs)
        
        # Integrate equations of motion
        if self.integration_method == "rk4":
            self._integrate_rk4(dt, forces, moments)
        else:
            self._integrate_euler(dt, forces, moments)
        
        # Update drone state
        self._update_drone_state(dt, control_inputs)
        
        return {
            'forces': forces,
            'moments': moments,
            'acceleration': self.drone.state.acceleration,
            'angular_acceleration': self.drone.state.angular_acceleration
        }
    
    def _calculate_forces(self, control_inputs: Dict[str, float]) -> np.ndarray:
        """
        Calculate total forces acting on the drone.
        
        Args:
            control_inputs: Control inputs dictionary
            
        Returns:
            3D force vector in body frame
        """
        # Initialize forces
        forces = np.zeros(3)
        
        # Gravitational force
        gravity_force = self._calculate_gravity_force()
        forces += gravity_force
        
        # Aerodynamic forces
        if self.environment.has_atmosphere():
            aero_force = self._calculate_aerodynamic_forces()
            forces += aero_force
        
        # Propulsion forces
        thrust_force = self._calculate_thrust_forces(control_inputs)
        forces += thrust_force
        
        return forces
    
    def _calculate_moments(self, control_inputs: Dict[str, float]) -> np.ndarray:
        """
        Calculate total moments acting on the drone.
        
        Args:
            control_inputs: Control inputs dictionary
            
        Returns:
            3D moment vector in body frame
        """
        # Initialize moments
        moments = np.zeros(3)
        
        # Aerodynamic moments
        if self.environment.has_atmosphere():
            aero_moments = self._calculate_aerodynamic_moments()
            moments += aero_moments
        
        # Propulsion moments
        thrust_moments = self._calculate_thrust_moments(control_inputs)
        moments += thrust_moments
        
        return moments
    
    def _calculate_gravity_force(self) -> np.ndarray:
        """Calculate gravitational force in body frame."""
        # Get current attitude
        roll, pitch, yaw = self.drone.state.attitude
        
        # Create rotation matrix from body to world frame
        rotation = Rotation.from_euler('xyz', [roll, pitch, yaw])
        
        # Gravity vector in world frame (pointing down)
        gravity_world = np.array([0, 0, -self.environment.gravity * self.drone.get_current_mass()])
        
        # Transform to body frame
        gravity_body = rotation.inv().apply(gravity_world)
        
        return gravity_body
    
    def _calculate_aerodynamic_forces(self) -> np.ndarray:
        """Calculate aerodynamic forces."""
        # Get air density at current altitude
        altitude = self.drone.state.position[2]
        air_density = self.environment.get_air_density(altitude)
        
        # Get velocity in body frame
        velocity_body = self._get_velocity_body_frame()
        airspeed = np.linalg.norm(velocity_body)
        
        if airspeed < 0.1:  # Avoid division by zero
            return np.zeros(3)
        
        # Dynamic pressure
        q = 0.5 * air_density * airspeed**2
        
        # Velocity unit vector
        velocity_unit = velocity_body / airspeed
        
        # Calculate angle of attack and sideslip
        alpha = math.atan2(velocity_body[2], velocity_body[0])  # Angle of attack
        beta = math.atan2(velocity_body[1], velocity_body[0])   # Sideslip angle
        
        # Aerodynamic coefficients
        cd = self._get_drag_coefficient(alpha, beta)
        cl = self._get_lift_coefficient(alpha, beta)
        cy = self._get_side_force_coefficient(alpha, beta)
        
        # Reference area
        s_ref = self.drone.specs.reference_area
        
        # Forces in stability axes
        drag = -cd * q * s_ref  # Opposite to velocity
        lift = cl * q * s_ref   # Perpendicular to velocity in xz plane
        side_force = cy * q * s_ref  # Perpendicular to velocity in xy plane
        
        # Transform to body frame
        # Simplified transformation assuming small angles
        forces = np.array([
            drag * velocity_unit[0] + lift * math.sin(alpha),
            side_force,
            drag * velocity_unit[2] - lift * math.cos(alpha)
        ])
        
        return forces
    
    def _calculate_thrust_forces(self, control_inputs: Dict[str, float]) -> np.ndarray:
        """Calculate thrust forces from propulsion system."""
        thrust_total = 0.0
        
        if self.drone.specs.propulsion_type in ['electric_rotors', 'hybrid']:
            # For rotorcraft, thrust is primarily vertical
            thrust_cmd = control_inputs.get('thrust', 0.0)
            max_thrust = self.drone.get_max_thrust()
            thrust_total = thrust_cmd * max_thrust
            
            # Thrust vector in body frame (z-axis up)
            thrust_vector = np.array([0, 0, thrust_total])
            
        elif self.drone.specs.propulsion_type == 'thrusters':
            # For thrusters, calculate thrust in all directions
            thrust_x = control_inputs.get('thrust_x', 0.0)
            thrust_y = control_inputs.get('thrust_y', 0.0)
            thrust_z = control_inputs.get('thrust_z', 0.0)
            
            max_thrust = self.drone.get_max_thrust()
            
            thrust_vector = np.array([
                thrust_x * max_thrust,
                thrust_y * max_thrust,
                thrust_z * max_thrust
            ])
        else:
            thrust_vector = np.zeros(3)
        
        # Apply efficiency factor
        thrust_vector *= self.drone.specs.efficiency
        
        return thrust_vector
    
    def _calculate_aerodynamic_moments(self) -> np.ndarray:
        """Calculate aerodynamic moments."""
        # Get air density at current altitude
        altitude = self.drone.state.position[2]
        air_density = self.environment.get_air_density(altitude)
        
        # Get velocity in body frame
        velocity_body = self._get_velocity_body_frame()
        airspeed = np.linalg.norm(velocity_body)
        
        if airspeed < 0.1:
            return np.zeros(3)
        
        # Dynamic pressure
        q = 0.5 * air_density * airspeed**2
        
        # Angular velocities
        p, q_rate, r = self.drone.state.angular_velocity
        
        # Moment coefficients (simplified)
        cl_p = -0.5  # Roll damping
        cm_q = -0.3  # Pitch damping
        cn_r = -0.1  # Yaw damping
        
        # Reference dimensions
        wingspan = self.drone.specs.wingspan
        chord = self.drone.specs.length
        
        # Damping moments
        roll_moment = cl_p * q * self.drone.specs.reference_area * wingspan * p * wingspan / (2 * airspeed)
        pitch_moment = cm_q * q * self.drone.specs.reference_area * chord * q_rate * chord / (2 * airspeed)
        yaw_moment = cn_r * q * self.drone.specs.reference_area * wingspan * r * wingspan / (2 * airspeed)
        
        return np.array([roll_moment, pitch_moment, yaw_moment])
    
    def _calculate_thrust_moments(self, control_inputs: Dict[str, float]) -> np.ndarray:
        """Calculate moments from thrust forces."""
        moments = np.zeros(3)
        
        if self.drone.specs.propulsion_type in ['electric_rotors', 'hybrid']:
            # For rotorcraft, calculate moments from differential thrust
            roll_cmd = control_inputs.get('roll', 0.0)
            pitch_cmd = control_inputs.get('pitch', 0.0)
            yaw_cmd = control_inputs.get('yaw', 0.0)
            
            # Moment arms (simplified)
            arm_length = self.drone.specs.wingspan / 2.0
            max_thrust = self.drone.get_max_thrust()
            
            # Calculate moments
            moments[0] = roll_cmd * max_thrust * arm_length * 0.1   # Roll
            moments[1] = pitch_cmd * max_thrust * arm_length * 0.1  # Pitch
            moments[2] = yaw_cmd * max_thrust * arm_length * 0.05   # Yaw
            
        elif self.drone.specs.propulsion_type == 'thrusters':
            # For thrusters, moments come from thruster placement
            roll_cmd = control_inputs.get('roll', 0.0)
            pitch_cmd = control_inputs.get('pitch', 0.0)
            yaw_cmd = control_inputs.get('yaw', 0.0)
            
            max_thrust = self.drone.get_max_thrust()
            arm_length = max(self.drone.specs.length, self.drone.specs.wingspan) / 2.0
            
            moments[0] = roll_cmd * max_thrust * arm_length * 0.2
            moments[1] = pitch_cmd * max_thrust * arm_length * 0.2
            moments[2] = yaw_cmd * max_thrust * arm_length * 0.2
        
        return moments
    
    def _integrate_rk4(self, dt: float, forces: np.ndarray, moments: np.ndarray):
        """Integrate using 4th order Runge-Kutta method."""
        # Current state
        pos = self.drone.state.position.copy()
        vel = self.drone.state.velocity.copy()
        att = self.drone.state.attitude.copy()
        ang_vel = self.drone.state.angular_velocity.copy()
        
        # Mass and inertia
        mass = self.drone.get_current_mass()
        inertia = self.drone.inertia_matrix
        
        # RK4 integration for position and velocity
        k1_vel = forces / mass
        k1_pos = vel
        
        k2_vel = forces / mass
        k2_pos = vel + 0.5 * dt * k1_vel
        
        k3_vel = forces / mass
        k3_pos = vel + 0.5 * dt * k2_vel
        
        k4_vel = forces / mass
        k4_pos = vel + dt * k3_vel
        
        # Update position and velocity
        self.drone.state.position = pos + (dt / 6.0) * (k1_pos + 2*k2_pos + 2*k3_pos + k4_pos)
        self.drone.state.velocity = vel + (dt / 6.0) * (k1_vel + 2*k2_vel + 2*k3_vel + k4_vel)
        self.drone.state.acceleration = k1_vel
        
        # RK4 integration for attitude and angular velocity
        k1_ang_acc = np.linalg.inv(inertia) @ (moments - np.cross(ang_vel, inertia @ ang_vel))
        k1_att_rate = ang_vel
        
        k2_ang_acc = np.linalg.inv(inertia) @ (moments - np.cross(ang_vel, inertia @ ang_vel))
        k2_att_rate = ang_vel + 0.5 * dt * k1_ang_acc
        
        k3_ang_acc = np.linalg.inv(inertia) @ (moments - np.cross(ang_vel, inertia @ ang_vel))
        k3_att_rate = ang_vel + 0.5 * dt * k2_ang_acc
        
        k4_ang_acc = np.linalg.inv(inertia) @ (moments - np.cross(ang_vel, inertia @ ang_vel))
        k4_att_rate = ang_vel + dt * k3_ang_acc
        
        # Update attitude and angular velocity
        self.drone.state.attitude = att + (dt / 6.0) * (k1_att_rate + 2*k2_att_rate + 2*k3_att_rate + k4_att_rate)
        self.drone.state.angular_velocity = ang_vel + (dt / 6.0) * (k1_ang_acc + 2*k2_ang_acc + 2*k3_ang_acc + k4_ang_acc)
        self.drone.state.angular_acceleration = k1_ang_acc
        
        # Normalize attitude angles
        self.drone.state.attitude = self._normalize_angles(self.drone.state.attitude)
    
    def _integrate_euler(self, dt: float, forces: np.ndarray, moments: np.ndarray):
        """Integrate using Euler method."""
        # Mass and inertia
        mass = self.drone.get_current_mass()
        inertia = self.drone.inertia_matrix
        
        # Linear motion
        acceleration = forces / mass
        self.drone.state.acceleration = acceleration
        self.drone.state.velocity += acceleration * dt
        self.drone.state.position += self.drone.state.velocity * dt
        
        # Angular motion
        angular_acceleration = np.linalg.inv(inertia) @ (moments - np.cross(
            self.drone.state.angular_velocity, inertia @ self.drone.state.angular_velocity))
        self.drone.state.angular_acceleration = angular_acceleration
        self.drone.state.angular_velocity += angular_acceleration * dt
        self.drone.state.attitude += self.drone.state.angular_velocity * dt
        
        # Normalize attitude angles
        self.drone.state.attitude = self._normalize_angles(self.drone.state.attitude)
    
    def _update_drone_state(self, dt: float, control_inputs: Dict[str, float]):
        """Update drone state including fuel/battery consumption."""
        # Calculate power consumption
        thrust_cmd = control_inputs.get('thrust', 0.0)
        power_consumption = self._calculate_power_consumption(thrust_cmd)
        
        # Update fuel/battery
        self.drone.update_fuel_consumption(power_consumption, dt)
        self.drone.state.power_consumption = power_consumption
        
        # Update current mass
        self.drone.state.current_mass = self.drone.get_current_mass()
    
    def _calculate_power_consumption(self, thrust_cmd: float) -> float:
        """Calculate power consumption based on thrust command."""
        if self.drone.specs.propulsion_type in ['electric_rotors', 'hybrid']:
            # For electric propulsion
            base_power = self.drone.specs.power_consumption
            power = base_power * thrust_cmd
        elif self.drone.specs.propulsion_type == 'thrusters':
            # For thrusters, power is proportional to thrust
            max_thrust = self.drone.get_max_thrust()
            power = thrust_cmd * max_thrust * 10.0  # Simplified model
        else:
            power = 0.0
        
        return power
    
    def _get_velocity_body_frame(self) -> np.ndarray:
        """Get velocity in body frame."""
        # Get current attitude
        roll, pitch, yaw = self.drone.state.attitude
        
        # Create rotation matrix from world to body frame
        rotation = Rotation.from_euler('xyz', [roll, pitch, yaw])
        
        # Transform velocity to body frame
        velocity_body = rotation.inv().apply(self.drone.state.velocity)
        
        return velocity_body
    
    def _get_drag_coefficient(self, alpha: float, beta: float) -> float:
        """Get drag coefficient based on angle of attack and sideslip."""
        # Simplified drag model
        cd0 = self.drone.specs.drag_coefficient
        cd_alpha = 0.1  # Induced drag coefficient
        
        cd = cd0 + cd_alpha * (alpha**2 + beta**2)
        
        return cd
    
    def _get_lift_coefficient(self, alpha: float, beta: float) -> float:
        """Get lift coefficient based on angle of attack."""
        # Simplified lift model
        cl0 = self.drone.specs.lift_coefficient
        cl_alpha = 2.0 * math.pi  # Lift curve slope
        
        cl = cl0 + cl_alpha * alpha
        
        # Stall model
        alpha_stall = math.radians(15)  # 15 degrees
        if abs(alpha) > alpha_stall:
            cl *= (1.0 - (abs(alpha) - alpha_stall) / alpha_stall)
        
        return cl
    
    def _get_side_force_coefficient(self, alpha: float, beta: float) -> float:
        """Get side force coefficient based on sideslip angle."""
        # Simplified side force model
        cy_beta = -0.5  # Side force coefficient
        
        cy = cy_beta * beta
        
        return cy
    
    def _normalize_angles(self, angles: np.ndarray) -> np.ndarray:
        """Normalize angles to [-π, π] range."""
        normalized = angles.copy()
        for i in range(len(normalized)):
            while normalized[i] > math.pi:
                normalized[i] -= 2 * math.pi
            while normalized[i] < -math.pi:
                normalized[i] += 2 * math.pi
        return normalized
