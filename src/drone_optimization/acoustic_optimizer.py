"""
Acoustic Optimizer Module

Implements acoustic optimization algorithms for drone rotors based on research
in uneven step angles and aeroacoustic noise reduction techniques.
"""

import numpy as np
from typing import Dict, List, Tuple, Optional
from dataclasses import dataclass


@dataclass
class AcousticAnalysisResult:
    """Results from acoustic analysis of a rotor configuration."""
    
    blade_passage_frequency: float  # Hz
    harmonic_frequencies: List[float]  # Hz
    discrete_tone_levels: List[float]  # dB
    broadband_noise_level: float  # dB
    overall_sound_pressure_level: float  # dB
    perceived_noise_level: float  # dB
    noise_reduction_achieved: float  # dB relative to baseline


@dataclass
class OptimizationConstraints:
    """Constraints for acoustic optimization process."""
    
    max_angular_variation: float = 0.10  # Maximum ±10% variation
    min_angular_variation: float = 0.03  # Minimum ±3% variation  
    max_thrust_asymmetry: float = 0.05   # Maximum 5% thrust asymmetry
    max_vibration_increase: float = 0.3  # Maximum 30% vibration increase
    target_noise_reduction: float = 3.0  # Target 3+ dB reduction


class AcousticOptimizer:
    """
    Acoustic optimization engine for drone rotor designs.
    
    Implements research-based optimization of rotor blade angular spacing
    to reduce discrete tones and redistribute acoustic energy into broadband noise.
    """
    
    def __init__(self):
        """Initialize acoustic optimizer with research parameters."""
        
        # Psychoacoustic weighting factors
        self.psychoacoustic_weights = {
            'discrete_tones': 2.0,      # Discrete tones are more annoying
            'broadband_noise': 1.0,     # Broadband is less annoying
            'low_frequency': 1.5,       # Low frequencies are more intrusive
            'high_frequency': 0.8       # High frequencies are less intrusive
        }
        
        # Blade-vortex interaction (BVI) parameters
        self.bvi_parameters = {
            'vortex_strength_factor': 0.1,
            'interaction_distance': 0.05,  # meters
            'decay_rate': 2.0
        }
        
        # Acoustic propagation parameters
        self.propagation_parameters = {
            'air_density': 1.225,        # kg/m³
            'speed_of_sound': 343.0,     # m/s
            'atmospheric_absorption': 0.02  # dB/m
        }
    
    def optimize_angular_spacing(self, 
                                num_blades: int,
                                design_rpm: float,
                                rotor_diameter: float,
                                constraints: Optional[OptimizationConstraints] = None) -> List[float]:
        """
        Optimize angular spacing of rotor blades for acoustic performance.
        
        Uses genetic algorithm approach to find optimal uneven spacing that
        minimizes discrete tones while maintaining aerodynamic performance.
        
        Args:
            num_blades: Number of rotor blades
            design_rpm: Design operating RPM
            rotor_diameter: Rotor diameter in meters
            constraints: Optimization constraints
            
        Returns:
            List of optimized angular spacings in degrees
        """
        if constraints is None:
            constraints = OptimizationConstraints()
        
        nominal_angle = 360.0 / num_blades
        
        # Generate initial population of spacing configurations
        population_size = 50
        generations = 100
        
        population = self._generate_initial_population(
            num_blades, nominal_angle, constraints, population_size
        )
        
        # Evolve population through genetic algorithm
        for generation in range(generations):
            # Evaluate fitness of each configuration
            fitness_scores = []
            for config in population:
                acoustic_score = self._evaluate_acoustic_fitness(
                    config, design_rpm, rotor_diameter
                )
                aerodynamic_penalty = self._evaluate_aerodynamic_penalty(config)
                
                # Combined fitness (higher is better)
                fitness = acoustic_score - aerodynamic_penalty
                fitness_scores.append(fitness)
            
            # Select best configurations for next generation
            population = self._genetic_selection(
                population, fitness_scores, population_size
            )
            
            # Apply mutation and crossover
            population = self._genetic_evolution(population, constraints)
        
        # Return best configuration
        final_fitness = [
            self._evaluate_acoustic_fitness(config, design_rpm, rotor_diameter) -
            self._evaluate_aerodynamic_penalty(config)
            for config in population
        ]
        
        best_index = np.argmax(final_fitness)
        return population[best_index]
    
    def analyze_acoustic_performance(self,
                                   angular_spacing: List[float],
                                   design_rpm: float,
                                   rotor_diameter: float,
                                   num_blades: int) -> AcousticAnalysisResult:
        """
        Perform detailed acoustic analysis of a rotor configuration.
        
        Calculates blade passage frequencies, harmonic content, and
        predicts noise reduction compared to symmetric design.
        """
        # Calculate blade passage frequency
        bpf = (design_rpm / 60.0) * num_blades
        
        # Generate harmonic frequencies (up to 5th harmonic)
        harmonics = [bpf * i for i in range(1, 6)]
        
        # Calculate discrete tone levels for each harmonic
        discrete_levels = self._calculate_discrete_tone_levels(
            angular_spacing, harmonics, rotor_diameter
        )
        
        # Calculate broadband noise level
        broadband_level = self._calculate_broadband_noise(
            design_rpm, rotor_diameter
        )
        
        # Calculate overall sound pressure level
        oaspl = self._calculate_overall_spl(discrete_levels, broadband_level)
        
        # Calculate perceived noise level with psychoacoustic weighting
        pnl = self._calculate_perceived_noise_level(
            harmonics, discrete_levels, broadband_level
        )
        
        # Calculate noise reduction vs. symmetric baseline
        baseline_oaspl = self._calculate_baseline_noise(
            design_rpm, rotor_diameter, num_blades
        )
        noise_reduction = baseline_oaspl - oaspl
        
        return AcousticAnalysisResult(
            blade_passage_frequency=bpf,
            harmonic_frequencies=harmonics,
            discrete_tone_levels=discrete_levels,
            broadband_noise_level=broadband_level,
            overall_sound_pressure_level=oaspl,
            perceived_noise_level=pnl,
            noise_reduction_achieved=noise_reduction
        )
    
    def _generate_initial_population(self,
                                   num_blades: int,
                                   nominal_angle: float,
                                   constraints: OptimizationConstraints,
                                   population_size: int) -> List[List[float]]:
        """Generate initial population of angular spacing configurations."""
        population = []
        
        for _ in range(population_size):
            config = []
            remaining_angle = 360.0
            
            for i in range(num_blades - 1):
                # Random variation within constraints
                max_var = constraints.max_angular_variation * nominal_angle
                min_var = constraints.min_angular_variation * nominal_angle
                
                variation = np.random.uniform(-max_var, max_var)
                angle = nominal_angle + variation
                
                # Ensure positive angle and account for remaining total
                angle = max(10.0, min(angle, remaining_angle - 10.0 * (num_blades - i - 1)))
                
                config.append(angle)
                remaining_angle -= angle
            
            # Last angle completes the circle
            config.append(remaining_angle)
            
            population.append(config)
        
        return population
    
    def _evaluate_acoustic_fitness(self,
                                 angular_spacing: List[float],
                                 design_rpm: float,
                                 rotor_diameter: float) -> float:
        """
        Evaluate acoustic fitness of an angular spacing configuration.
        
        Higher scores indicate better acoustic performance (more noise reduction).
        """
        # Calculate periodicity disruption factor
        nominal_angle = 360.0 / len(angular_spacing)
        periodicity_factor = 0.0
        
        for i, angle in enumerate(angular_spacing):
            deviation = abs(angle - nominal_angle) / nominal_angle
            periodicity_factor += deviation
        
        periodicity_factor /= len(angular_spacing)
        
        # Calculate blade-vortex interaction reduction
        bvi_reduction = self._calculate_bvi_reduction(angular_spacing)
        
        # Calculate discrete tone energy redistribution
        tone_redistribution = self._calculate_tone_redistribution(
            angular_spacing, design_rpm
        )
        
        # Combined acoustic fitness score
        acoustic_score = (
            periodicity_factor * 100.0 +      # Periodicity disruption
            bvi_reduction * 50.0 +             # BVI reduction
            tone_redistribution * 75.0         # Tone redistribution
        )
        
        return acoustic_score
    
    def _evaluate_aerodynamic_penalty(self, angular_spacing: List[float]) -> float:
        """
        Calculate aerodynamic penalty for uneven angular spacing.
        
        Higher values indicate greater performance degradation.
        """
        # Calculate thrust asymmetry
        nominal_angle = 360.0 / len(angular_spacing)
        asymmetry_factor = 0.0
        
        for angle in angular_spacing:
            deviation = abs(angle - nominal_angle) / nominal_angle
            asymmetry_factor += deviation ** 2  # Quadratic penalty
        
        asymmetry_factor = np.sqrt(asymmetry_factor / len(angular_spacing))
        
        # Calculate vibration penalty
        vibration_factor = asymmetry_factor * 1.5
        
        # Calculate efficiency loss
        efficiency_loss = asymmetry_factor * 2.0
        
        # Combined aerodynamic penalty
        aero_penalty = (
            asymmetry_factor * 20.0 +    # Thrust asymmetry
            vibration_factor * 15.0 +    # Vibration increase
            efficiency_loss * 25.0       # Efficiency loss
        )
        
        return aero_penalty
    
    def _calculate_discrete_tone_levels(self,
                                      angular_spacing: List[float],
                                      harmonics: List[float],
                                      rotor_diameter: float) -> List[float]:
        """Calculate discrete tone sound pressure levels for each harmonic."""
        levels = []
        
        for i, freq in enumerate(harmonics):
            # Base tone level (decreases with harmonic number)
            base_level = 60.0 - i * 6.0  # dB
            
            # Reduction due to uneven spacing
            spacing_factor = self._calculate_spacing_factor(angular_spacing)
            reduction = spacing_factor * (5.0 - i * 0.5)  # Maximum 5 dB reduction for fundamental
            
            # Diameter scaling
            diameter_factor = 20 * np.log10(rotor_diameter / 0.25)  # Reference 0.25m diameter
            
            level = base_level - reduction + diameter_factor
            levels.append(max(30.0, level))  # Minimum noise floor
        
        return levels
    
    def _calculate_broadband_noise(self, design_rpm: float, rotor_diameter: float) -> float:
        """Calculate broadband noise level."""
        # Base broadband level
        base_level = 45.0  # dB
        
        # RPM scaling (6 dB per doubling)
        rpm_factor = 20 * np.log10(design_rpm / 3000.0)  # Reference 3000 RPM
        
        # Diameter scaling
        diameter_factor = 20 * np.log10(rotor_diameter / 0.25)
        
        return base_level + rpm_factor + diameter_factor
    
    def _calculate_spacing_factor(self, angular_spacing: List[float]) -> float:
        """Calculate spacing factor for acoustic benefit calculation."""
        nominal_angle = 360.0 / len(angular_spacing)
        
        # Calculate standard deviation of spacing
        deviations = [(angle - nominal_angle) for angle in angular_spacing]
        std_dev = np.std(deviations)
        
        # Normalize to percentage variation
        variation = std_dev / nominal_angle
        
        # Convert to acoustic benefit factor (0 to 1)
        return min(1.0, variation / 0.1)  # Full benefit at 10% variation
    
    def _calculate_bvi_reduction(self, angular_spacing: List[float]) -> float:
        """Calculate blade-vortex interaction reduction factor."""
        # Uneven spacing disrupts periodic BVI
        spacing_variation = np.std(angular_spacing) / np.mean(angular_spacing)
        
        # BVI reduction is proportional to spacing variation
        bvi_reduction = min(1.0, spacing_variation / 0.08)  # Full reduction at 8% variation
        
        return bvi_reduction
    
    def _calculate_tone_redistribution(self, angular_spacing: List[float], design_rpm: float) -> float:
        """Calculate discrete tone energy redistribution factor."""
        # Energy redistribution increases with spacing irregularity
        nominal_angle = 360.0 / len(angular_spacing)
        
        redistribution_factor = 0.0
        for i in range(len(angular_spacing)):
            next_i = (i + 1) % len(angular_spacing)
            angle_diff = abs(angular_spacing[i] - angular_spacing[next_i])
            redistribution_factor += angle_diff / nominal_angle
        
        redistribution_factor /= len(angular_spacing)
        
        return min(1.0, redistribution_factor)
    
    def _genetic_selection(self, population: List[List[float]], 
                          fitness_scores: List[float], 
                          population_size: int) -> List[List[float]]:
        """Select best configurations for next generation."""
        # Sort by fitness (descending)
        sorted_indices = np.argsort(fitness_scores)[::-1]
        
        # Keep top 50% and add random selection from remaining
        elite_size = population_size // 2
        
        new_population = []
        
        # Elite selection
        for i in range(elite_size):
            new_population.append(population[sorted_indices[i]].copy())
        
        # Tournament selection for remaining spots
        for _ in range(population_size - elite_size):
            # Tournament selection with size 3
            tournament_indices = np.random.choice(len(population), 3, replace=False)
            tournament_fitness = [fitness_scores[i] for i in tournament_indices]
            winner_idx = tournament_indices[np.argmax(tournament_fitness)]
            new_population.append(population[winner_idx].copy())
        
        return new_population
    
    def _genetic_evolution(self, population: List[List[float]], 
                          constraints: OptimizationConstraints) -> List[List[float]]:
        """Apply mutation and crossover to population."""
        new_population = []
        
        for i in range(0, len(population), 2):
            parent1 = population[i]
            parent2 = population[min(i + 1, len(population) - 1)]
            
            # Crossover
            if np.random.random() < 0.7:  # 70% crossover rate
                child1, child2 = self._crossover(parent1, parent2)
            else:
                child1, child2 = parent1.copy(), parent2.copy()
            
            # Mutation
            if np.random.random() < 0.1:  # 10% mutation rate
                child1 = self._mutate(child1, constraints)
            if np.random.random() < 0.1:
                child2 = self._mutate(child2, constraints)
            
            new_population.extend([child1, child2])
        
        return new_population[:len(population)]
    
    def _crossover(self, parent1: List[float], parent2: List[float]) -> Tuple[List[float], List[float]]:
        """Perform crossover between two parent configurations."""
        crossover_point = np.random.randint(1, len(parent1) - 1)
        
        child1 = parent1[:crossover_point] + parent2[crossover_point:]
        child2 = parent2[:crossover_point] + parent1[crossover_point:]
        
        # Normalize to ensure total = 360 degrees
        child1 = self._normalize_angles(child1)
        child2 = self._normalize_angles(child2)
        
        return child1, child2
    
    def _mutate(self, configuration: List[float], 
               constraints: OptimizationConstraints) -> List[float]:
        """Apply mutation to a configuration."""
        mutated = configuration.copy()
        
        # Select random gene to mutate
        gene_idx = np.random.randint(len(mutated))
        
        # Apply random mutation within constraints
        nominal_angle = 360.0 / len(mutated)
        max_var = constraints.max_angular_variation * nominal_angle
        
        mutation = np.random.uniform(-max_var * 0.1, max_var * 0.1)
        mutated[gene_idx] += mutation
        
        # Normalize to maintain total = 360 degrees
        return self._normalize_angles(mutated)
    
    def _normalize_angles(self, angles: List[float]) -> List[float]:
        """Normalize angle list to sum to 360 degrees while maintaining relative proportions."""
        total = sum(angles)
        if total > 0:
            normalized = [angle * 360.0 / total for angle in angles]
            
            # Ensure all angles are positive and reasonable
            min_angle = 360.0 / len(angles) * 0.3  # Minimum 30% of nominal
            max_angle = 360.0 / len(angles) * 1.7  # Maximum 170% of nominal
            
            for i in range(len(normalized)):
                normalized[i] = max(min_angle, min(max_angle, normalized[i]))
            
            # Re-normalize after clamping
            total = sum(normalized)
            normalized = [angle * 360.0 / total for angle in normalized]
            
            return normalized
        else:
            # Fallback to equal spacing
            nominal = 360.0 / len(angles)
            return [nominal] * len(angles)
    
    def _calculate_overall_spl(self, discrete_levels: List[float], broadband_level: float) -> float:
        """Calculate overall sound pressure level from discrete tones and broadband."""
        # Energy sum of all components
        total_energy = 10 ** (broadband_level / 10.0)
        
        for level in discrete_levels:
            total_energy += 10 ** (level / 10.0)
        
        return 10.0 * np.log10(total_energy)
    
    def _calculate_perceived_noise_level(self, frequencies: List[float], 
                                       discrete_levels: List[float], 
                                       broadband_level: float) -> float:
        """Calculate perceived noise level with psychoacoustic weighting."""
        weighted_energy = 0.0
        
        # Weight discrete tones
        for freq, level in zip(frequencies, discrete_levels):
            if freq < 500:
                weight = self.psychoacoustic_weights['low_frequency']
            else:
                weight = self.psychoacoustic_weights['high_frequency']
            
            weight *= self.psychoacoustic_weights['discrete_tones']
            weighted_energy += weight * (10 ** (level / 10.0))
        
        # Weight broadband noise
        broadband_weight = self.psychoacoustic_weights['broadband_noise']
        weighted_energy += broadband_weight * (10 ** (broadband_level / 10.0))
        
        return 10.0 * np.log10(weighted_energy)
    
    def _calculate_baseline_noise(self, design_rpm: float, 
                                rotor_diameter: float, 
                                num_blades: int) -> float:
        """Calculate baseline noise level for symmetric rotor design."""
        # Simplified baseline calculation
        base_level = 65.0  # dB for typical small drone
        
        # RPM scaling
        rpm_factor = 20 * np.log10(design_rpm / 3000.0)
        
        # Diameter scaling  
        diameter_factor = 20 * np.log10(rotor_diameter / 0.25)
        
        # Blade number effect
        blade_factor = 3 * np.log10(num_blades / 4.0)
        
        return base_level + rpm_factor + diameter_factor + blade_factor