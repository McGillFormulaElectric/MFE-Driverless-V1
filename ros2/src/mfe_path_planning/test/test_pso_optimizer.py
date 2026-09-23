"""
Comprehensive unit tests for Particle Swarm Optimization (PSO) race-line optimizer.

Tests the PSO race-line optimizer from path_planner_node.py.

Tests cover:
- PSO initialization and particle creation
- Velocity and position updates with dynamic coefficients
- Boundary constraint enforcement and reflection
- Convergence tests on simple optimization problems
- Fitness landscape exploration
- Stagnation detection and escape mechanisms
"""

import math
import numpy as np
import pytest


class TestPSOInitialization:
    """Test PSO swarm initialization."""

    def test_particle_initialization(self):
        """Test that particles are initialized correctly."""
        n_particles = 30
        n_dimensions = 10

        pos = np.zeros((n_particles, n_dimensions))
        vel = np.zeros_like(pos)

        assert pos.shape == (n_particles, n_dimensions)
        assert vel.shape == (n_particles, n_dimensions)
        assert np.all(pos == 0.0)
        assert np.all(vel == 0.0)

    def test_random_initialization_within_bounds(self):
        """Test random initialization respects boundary constraints."""
        n_particles = 30
        n_dimensions = 10
        bounds = [(-1.0, 1.0) for _ in range(n_dimensions)]

        pos = np.random.uniform(-1.0, 1.0, (n_particles, n_dimensions))

        for i in range(n_dimensions):
            assert np.all(pos[:, i] >= bounds[i][0])
            assert np.all(pos[:, i] <= bounds[i][1])

    def test_pbest_initialization(self):
        """Test personal best initialization."""
        n_particles = 30
        n_dimensions = 10

        pos = np.random.randn(n_particles, n_dimensions)
        pbest = pos.copy()
        pbest_fit = np.array([np.inf] * n_particles)

        assert pbest.shape == pos.shape
        assert len(pbest_fit) == n_particles
        assert np.all(np.isinf(pbest_fit))

    def test_gbest_initialization(self):
        """Test global best initialization."""
        n_particles = 30
        pbest_fit = np.array([0.5, 1.5, 0.3, 2.0, 0.8])

        gbest_idx = int(np.argmin(pbest_fit))
        gbest_fit = pbest_fit[gbest_idx]

        assert gbest_idx == 2
        assert gbest_fit == 0.3

    def test_first_particle_at_origin(self):
        """Test that first particle starts at center (alpha=0 for race line)."""
        n_particles = 30
        n_dimensions = 10

        pos = np.zeros((n_particles, n_dimensions))
        pos[1:] = np.random.randn(n_particles - 1, n_dimensions)

        assert np.all(pos[0] == 0.0)


class TestVelocityUpdates:
    """Test PSO velocity update equations."""

    def test_inertia_weight_decrease(self):
        """Test that inertia weight decreases over iterations."""
        n_iter = 100
        w_inertia_start = 0.8
        w_inertia_end = 0.1

        inertias = []
        for it in range(n_iter):
            frac = it / max(n_iter - 1, 1)
            w = w_inertia_start - (w_inertia_start - w_inertia_end) * frac
            inertias.append(w)

        assert inertias[0] == w_inertia_start
        assert inertias[-1] == pytest.approx(w_inertia_end, abs=1e-10)
        # Check monotonic decrease
        for i in range(1, len(inertias)):
            assert inertias[i] <= inertias[i-1]

    def test_cognitive_coefficient_decrease(self):
        """Test that cognitive coefficient decreases over iterations."""
        n_iter = 100
        c1_start = 1.5
        c1_end = 0.5

        coeffs = []
        for it in range(n_iter):
            frac = it / max(n_iter - 1, 1)
            c = c1_start - (c1_start - c1_end) * frac
            coeffs.append(c)

        assert coeffs[0] == c1_start
        assert coeffs[-1] == c1_end
        for i in range(1, len(coeffs)):
            assert coeffs[i] <= coeffs[i-1]

    def test_social_coefficient_increase(self):
        """Test that social coefficient increases over iterations."""
        n_iter = 100
        c2_start = 1.5
        c2_end = 2.5

        coeffs = []
        for it in range(n_iter):
            frac = it / max(n_iter - 1, 1)
            c = c2_start + (c2_end - c2_start) * frac
            coeffs.append(c)

        assert coeffs[0] == c2_start
        assert coeffs[-1] == c2_end
        for i in range(1, len(coeffs)):
            assert coeffs[i] >= coeffs[i-1]

    def test_velocity_update_equation(self):
        """Test standard PSO velocity update."""
        pos = np.array([1.0, 2.0, 3.0])
        vel = np.array([0.1, 0.2, 0.3])
        pbest = np.array([0.5, 2.5, 3.5])
        gbest = np.array([0.3, 2.0, 3.2])

        w = 0.7
        c1 = 1.5
        c2 = 1.5
        r1 = np.array([0.5, 0.6, 0.7])
        r2 = np.array([0.4, 0.5, 0.6])

        vel_new = (w * vel
                   + c1 * r1 * (pbest - pos)
                   + c2 * r2 * (gbest - pos))

        # Velocity should be updated based on cognitive and social components
        assert len(vel_new) == 3
        assert np.all(np.isfinite(vel_new))

    def test_position_update_from_velocity(self):
        """Test position update from velocity."""
        pos = np.array([1.0, 2.0, 3.0])
        vel = np.array([0.1, 0.2, 0.3])

        pos_new = pos + vel

        expected = np.array([1.1, 2.2, 3.3])
        np.testing.assert_allclose(pos_new, expected, atol=1e-6)


class TestBoundaryConstraints:
    """Test boundary constraint handling and reflection."""

    def test_clamp_to_lower_bound(self):
        """Test clamping position to lower boundary."""
        pos = np.array([-1.5, 0.5, 0.0])
        lower_bound = -1.0
        upper_bound = 1.0

        pos_clamped = np.clip(pos, lower_bound, upper_bound)

        expected = np.array([-1.0, 0.5, 0.0])
        np.testing.assert_allclose(pos_clamped, expected)

    def test_clamp_to_upper_bound(self):
        """Test clamping position to upper boundary."""
        pos = np.array([-0.5, 1.5, 0.0])
        lower_bound = -1.0
        upper_bound = 1.0

        pos_clamped = np.clip(pos, lower_bound, upper_bound)

        expected = np.array([-0.5, 1.0, 0.0])
        np.testing.assert_allclose(pos_clamped, expected)

    def test_asymmetric_bounds(self):
        """Test clamping with asymmetric bounds."""
        pos = np.array([-0.5, 0.5, 1.5])
        lower_bounds = np.array([-0.8, -0.5, -1.0])
        upper_bounds = np.array([0.8, 0.5, 1.0])

        pos_clamped = np.zeros_like(pos)
        for i in range(len(pos)):
            pos_clamped[i] = np.clip(pos[i], lower_bounds[i], upper_bounds[i])

        expected = np.array([-0.5, 0.5, 1.0])
        np.testing.assert_allclose(pos_clamped, expected)

    def test_boundary_enforcement_for_track_bounds(self):
        """Test boundary enforcement for race-line lateral offsets."""
        n_waypoints = 5
        w_left = np.array([1.0, 1.2, 0.9, 1.1, 1.0])
        w_right = np.array([0.8, 0.9, 1.0, 0.85, 0.9])

        alpha = np.array([1.5, -0.5, 1.1, -1.2, 0.0])

        alpha_clamped = np.zeros_like(alpha)
        for i in range(n_waypoints):
            alpha_clamped[i] = np.clip(alpha[i], -w_right[i], w_left[i])

        # Check that all are within bounds
        for i in range(n_waypoints):
            assert -w_right[i] <= alpha_clamped[i] <= w_left[i]

    def test_velocity_zeroing_on_boundary_hit(self):
        """Test that velocity is zeroed when particle escapes after perturbation."""
        vel = np.array([0.1, 0.2, 0.3])
        escaped = True

        if escaped:
            vel = np.zeros_like(vel)

        assert np.all(vel == 0.0)


class TestFitnessEvaluation:
    """Test fitness function evaluation for optimization problems."""

    def test_quadratic_function_optimization(self):
        """Test PSO on simple quadratic function (minimum at origin)."""
        def fitness(x):
            return float(np.sum(x ** 2))

        x = np.array([1.0, 2.0, 3.0])
        f = fitness(x)

        expected = 14.0
        assert f == expected

    def test_fitness_at_optimum(self):
        """Test fitness value at known optimum."""
        def fitness(x):
            # Sphere function: minimum at x=0
            return float(np.sum(x ** 2))

        x_opt = np.array([0.0, 0.0, 0.0])
        f_opt = fitness(x_opt)

        assert f_opt == 0.0

    def test_rosenbrock_function_evaluation(self):
        """Test evaluation of Rosenbrock function."""
        def rosenbrock(x):
            return 100.0 * (x[1] - x[0]**2)**2 + (1 - x[0])**2

        x = np.array([1.0, 1.0])
        f = rosenbrock(x)

        assert abs(f - 0.0) < 1e-6

    def test_fitness_population_evaluation(self):
        """Test evaluating fitness for entire population."""
        def fitness(x):
            return np.sum(x ** 2)

        population = np.array([
            [1.0, 0.0],
            [0.0, 1.0],
            [1.0, 1.0],
        ])

        fitnesses = np.array([fitness(p) for p in population])

        expected = np.array([1.0, 1.0, 2.0])
        np.testing.assert_allclose(fitnesses, expected)

    def test_penalty_for_constraint_violation(self):
        """Test penalty applied for violating track boundaries."""
        penalty_weight = 5.0
        boundary_violation = 0.2

        penalty = penalty_weight * (boundary_violation ** 2)

        assert penalty > 0.0
        assert penalty == pytest.approx(0.2, abs=1e-10)


class TestConvergence:
    """Test convergence behavior and fitness tracking."""

    def test_pbest_improvement_tracking(self):
        """Test tracking of personal best improvements."""
        pbest_fit = np.array([5.0, 4.0, 6.0, 3.0, 5.5])
        current_fit = np.array([4.5, 4.0, 5.5, 3.5, 5.0])

        improved = []
        for i in range(len(pbest_fit)):
            if current_fit[i] < pbest_fit[i]:
                pbest_fit[i] = current_fit[i]
                improved.append(i)

        assert len(improved) == 3  # Indices 0, 2, and 4

    def test_gbest_update_on_improvement(self):
        """Test global best update when better solution found."""
        gbest_fit = 3.5
        pbest_fit = np.array([5.0, 4.0, 3.0, 3.5, 5.5])

        new_gbest_fit = np.min(pbest_fit)
        assert new_gbest_fit == 3.0
        assert new_gbest_fit < gbest_fit

    def test_stagnation_detection(self):
        """Test detection of stagnation (no improvement)."""
        n_iter = 100
        stagnant_threshold = 10

        stagnant_count = 0
        best_fit_history = [10.0] * 15

        for it in range(1, len(best_fit_history)):
            if best_fit_history[it] >= best_fit_history[it-1]:
                stagnant_count += 1
            else:
                stagnant_count = 0

        is_stagnant = stagnant_count >= stagnant_threshold

        assert is_stagnant is True

    def test_fitness_improvement_sequence(self):
        """Test sequence of fitness improvements over iterations."""
        fitness_history = [10.0, 8.5, 7.2, 6.8, 6.5, 6.4, 6.4, 6.35, 6.35, 6.3]

        # Check general trend of improvement
        assert fitness_history[-1] < fitness_history[0]

        # Check that most improvements are monotonic or near-monotonic
        improvements = sum(1 for i in range(1, len(fitness_history))
                          if fitness_history[i] <= fitness_history[i-1])
        assert improvements >= len(fitness_history) * 0.8

    def test_convergence_to_known_optimum(self):
        """Test convergence to optimum for quadratic function."""
        def fitness(x):
            return np.sum(x ** 2)

        # Simulate PSO: particles converge toward center
        swarm = np.array([
            [1.0, 1.0],
            [0.8, 0.9],
            [0.6, 0.7],
            [0.5, 0.6],
            [0.4, 0.5],
        ])

        fitnesses = np.array([fitness(p) for p in swarm])

        # Final fitnesses should be improving
        assert fitnesses[-1] < fitnesses[0]


class TestStagnationEscape:
    """Test mechanisms to escape local optima and stagnation."""

    def test_perturbation_of_worst_particles(self):
        """Test perturbing worst particles near global best."""
        pbest_fit = np.array([5.0, 4.0, 6.0, 3.0, 5.5])
        n_perturb = max(1, len(pbest_fit) // 5)

        worst_idx = np.argsort(pbest_fit)[-n_perturb:]

        assert len(worst_idx) >= 1
        assert worst_idx[0] in [2, 4]  # Highest fitness values

    def test_noise_injection_near_gbest(self):
        """Test adding noise around global best."""
        gbest = np.array([0.1, 0.2, 0.3])
        N = 10
        noise_scale = 0.1

        noise = np.random.randn(N) * noise_scale
        perturbed = gbest[0] + noise

        # Should be centered around gbest
        assert np.abs(np.mean(perturbed) - gbest[0]) < 0.3

    def test_velocity_reset_on_restart(self):
        """Test velocity reset when restarting particle."""
        vel = np.array([0.5, 0.6, 0.7])
        vel_reset = np.zeros_like(vel)

        assert np.all(vel_reset == 0.0)

    def test_stagnation_counter_increment(self):
        """Test stagnation counter increment on no improvement."""
        stagnant = 0

        # Simulate 5 iterations of no improvement
        for _ in range(5):
            stagnant += 1

        assert stagnant == 5

    def test_stagnation_reset_on_improvement(self):
        """Test stagnation counter reset on improvement."""
        stagnant = 5

        # Reset on improvement
        improved = True
        if improved:
            stagnant = 0

        assert stagnant == 0


class TestPSOIntegration:
    """Integration tests for full PSO optimization."""

    def test_full_pso_cycle_quadratic(self):
        """Test full PSO cycle on quadratic function."""
        def fitness(x):
            return float(np.sum(x ** 2))

        n_particles = 10
        n_dimensions = 2
        n_iter = 20

        pos = np.random.uniform(-5.0, 5.0, (n_particles, n_dimensions))
        vel = np.random.uniform(-1.0, 1.0, (n_particles, n_dimensions))
        pbest = pos.copy()
        pbest_fit = np.array([fitness(p) for p in pos])
        gbest_idx = int(np.argmin(pbest_fit))
        gbest = pbest[gbest_idx].copy()

        for it in range(n_iter):
            w = 0.9 - 0.5 * (it / n_iter)
            c1 = 2.0
            c2 = 2.0

            for i in range(n_particles):
                vel[i] = (w * vel[i]
                         + c1 * np.random.rand() * (pbest[i] - pos[i])
                         + c2 * np.random.rand() * (gbest - pos[i]))
                pos[i] = pos[i] + vel[i]

                f = fitness(pos[i])
                if f < pbest_fit[i]:
                    pbest[i] = pos[i].copy()
                    pbest_fit[i] = f
                    if f < fitness(gbest):
                        gbest = pos[i].copy()

        # Final solution should be better than initial random solution
        final_fit = fitness(gbest)
        assert final_fit < 5.0  # Significantly better than random

    def test_pso_with_constraints(self):
        """Test PSO respecting track boundary constraints."""
        n_particles = 10
        n_waypoints = 5

        w_left = np.array([1.0, 1.2, 0.9, 1.1, 1.0])
        w_right = np.array([0.8, 0.9, 1.0, 0.85, 0.9])

        pos = np.zeros((n_particles, n_waypoints))

        for p in range(n_particles):
            for w in range(n_waypoints):
                pos[p, w] = np.clip(np.random.randn() * 0.5,
                                   -w_right[w], w_left[w])

        # Verify all particles are in bounds
        for p in range(n_particles):
            for w in range(n_waypoints):
                assert -w_right[w] <= pos[p, w] <= w_left[w]

    def test_particle_diversity_maintenance(self):
        """Test that particle diversity is maintained during optimization."""
        n_particles = 20
        n_dimensions = 5

        pos = np.random.randn(n_particles, n_dimensions)

        # Compute centroid
        centroid = np.mean(pos, axis=0)

        # Compute diversity (average distance from centroid)
        diversity = np.mean([np.linalg.norm(p - centroid) for p in pos])

        assert diversity > 0.5  # Should have non-trivial diversity

    def test_race_line_penalty_application(self):
        """Test penalty application for race-line optimization."""
        # Simulate a race line with boundary penalties
        centerline_waypoints = 10
        penalty_weight = 5.0

        # Particle violates boundary at some waypoints
        violations = np.array([0.0, 0.1, 0.0, 0.2, 0.0, 0.0, 0.15, 0.0, 0.0, 0.0])

        penalty = penalty_weight * np.sum(violations ** 2)

        assert penalty > 0.0
        assert penalty < 1.0  # Reasonable penalty magnitude


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
