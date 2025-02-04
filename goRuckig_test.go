package goRuckig

import (
	"fmt"
	"testing"
	"time"
)

// Setup function that works for both testing and benchmarking
func setupRuckig(tb interface{ Fatal(args ...interface{}) }, dof int) (*Ruckig, *InputParameter, *OutputParameter) {
	ruckig := NewRuckig(0.01, dof)
	if ruckig == nil {
		tb.Fatal("Failed to create Ruckig instance")
	}

	input := NewInputParameter(dof)
	if input == nil {
		tb.Fatal("Failed to create InputParameter instance")
	}

	output := NewOutputParameter(dof)
	if output == nil {
		tb.Fatal("Failed to create OutputParameter instance")
	}

	return ruckig, input, output
}

func TestRuckigTrajectory(t *testing.T) {
	ruckig, input, output := setupRuckig(t, 3)
	defer ruckig.Destroy()
	defer input.Destroy()
	defer output.Destroy()

	// Set standard input parameters
	input.SetCurrentPosition([]float64{0.0, 0.0, 0.5})
	input.SetCurrentVelocity([]float64{0.0, -2.2, -0.5})
	input.SetCurrentAcceleration([]float64{0.0, 2.5, -0.5})
	input.SetTargetPosition([]float64{5.0, -2.0, -3.5})
	input.SetTargetVelocity([]float64{0.0, -0.5, -2.0})
	input.SetTargetAcceleration([]float64{0.0, 0.0, 0.5})
	input.SetMaxVelocity([]float64{3.0, 1.0, 3.0})
	input.SetMaxAcceleration([]float64{3.0, 2.0, 1.0})
	input.SetMaxJerk([]float64{4.0, 3.0, 2.0})

	// Validate trajectory generation
	fmt.Println("t | position")
	for ruckig.Update(input, output) == 0 { // 0 indicates Result::Working
		fmt.Printf("%.2f | %v\n", output.Time(), output.NewPosition())
		output.PassToInput(input)
	}

	if output.TrajectoryDuration() <= 0 {
		t.Errorf("Expected positive trajectory duration, got %.2f", output.TrajectoryDuration())
	}
	fmt.Printf("Trajectory duration: %.2f [s]\n", output.TrajectoryDuration())
}

func TestRuckigEdgeCases(t *testing.T) {
	ruckig, input, output := setupRuckig(t, 3)
	defer ruckig.Destroy()
	defer input.Destroy()
	defer output.Destroy()

	edgeCases := []struct {
		description   string
		currentPos    []float64
		currentVel    []float64
		currentAcc    []float64
		targetPos     []float64
		targetVel     []float64
		targetAcc     []float64
		maxVelocity   []float64
		maxAccel      []float64
		maxJerk       []float64
		expectedValid bool
	}{
		{
			"Basic edge case with zero initial and target velocities",
			[]float64{0.0, 0.0, 0.0}, []float64{0.0, 0.0, 0.0}, []float64{0.0, 0.0, 0.0},
			[]float64{100.0, 100.0, 100.0}, []float64{0.0, 0.0, 0.0}, []float64{0.0, 0.0, 0.0},
			[]float64{1.0, 1.0, 1.0}, []float64{1.0, 1.0, 1.0}, []float64{1.0, 1.0, 1.0},
			true,
		},
		{
			"Edge case with negative velocities",
			[]float64{0.0, 0.0, 0.0}, []float64{-10.0, -10.0, -10.0}, []float64{0.0, 0.0, 0.0},
			[]float64{50.0, 50.0, 50.0}, []float64{0.0, 0.0, 0.0}, []float64{0.0, 0.0, 0.0},
			[]float64{5.0, 5.0, 5.0}, []float64{5.0, 5.0, 5.0}, []float64{2.0, 2.0, 2.0},
			true,
		},
		{
			"Case with extreme jerk limits",
			[]float64{1.0, 1.0, 1.0}, []float64{0.0, 0.0, 0.0}, []float64{0.0, 0.0, 0.0},
			[]float64{10.0, 10.0, 10.0}, []float64{0.0, 0.0, 0.0}, []float64{0.0, 0.0, 0.0},
			[]float64{10.0, 10.0, 10.0}, []float64{10.0, 10.0, 10.0}, []float64{1000.0, 1000.0, 1000.0},
			true,
		},
	}

	for _, ec := range edgeCases {
		t.Run(ec.description, func(t *testing.T) {
			input.SetCurrentPosition(ec.currentPos)
			input.SetCurrentVelocity(ec.currentVel)
			input.SetCurrentAcceleration(ec.currentAcc)
			input.SetTargetPosition(ec.targetPos)
			input.SetTargetVelocity(ec.targetVel)
			input.SetTargetAcceleration(ec.targetAcc)
			input.SetMaxVelocity(ec.maxVelocity)
			input.SetMaxAcceleration(ec.maxAccel)
			input.SetMaxJerk(ec.maxJerk)

			result := ruckig.Update(input, output)
			if (result == 0) != ec.expectedValid {
				t.Errorf("Expected result validity: %v, but got %v", ec.expectedValid, result == 0)
			}
			if ec.expectedValid {
				if output.TrajectoryDuration() <= 0 {
					t.Errorf("Expected positive trajectory duration, got %.2f", output.TrajectoryDuration())
				}
				fmt.Printf("Trajectory duration: %.2f [s]\n", output.TrajectoryDuration())
			}
		})
	}
}

func BenchmarkRuckigPerformance(b *testing.B) {
	ruckig, input, output := setupRuckig(b, 3)
	defer ruckig.Destroy()
	defer input.Destroy()
	defer output.Destroy()

	input.SetCurrentPosition([]float64{0.0, 0.0, 0.0})
	input.SetCurrentVelocity([]float64{0.0, 0.0, 0.0})
	input.SetCurrentAcceleration([]float64{0.0, 0.0, 0.0})
	input.SetTargetPosition([]float64{10.0, 10.0, 10.0})
	input.SetTargetVelocity([]float64{0.0, 0.0, 0.0})
	input.SetTargetAcceleration([]float64{0.0, 0.0, 0.0})
	input.SetMaxVelocity([]float64{5.0, 5.0, 5.0})
	input.SetMaxAcceleration([]float64{2.0, 2.0, 2.0})
	input.SetMaxJerk([]float64{10.0, 10.0, 10.0})

	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		if ruckig.Update(input, output) != 0 {
			b.Fatal("Trajectory generation failed")
		}
		output.PassToInput(input)
	}
}
func TestRuckigAdditionalEdgeCases(t *testing.T) {
	ruckig := NewRuckig(0.01, 3)
	if ruckig == nil {
		t.Fatal("Failed to create Ruckig instance")
	}
	defer ruckig.Destroy()

	t.Run("High Initial Velocity and Low Target Velocity", func(t *testing.T) {
		input := NewInputParameter(3)
		output := NewOutputParameter(3)
		defer input.Destroy()
		defer output.Destroy()

		input.SetCurrentPosition([]float64{0.0, 0.0, 0.0})
		input.SetCurrentVelocity([]float64{5.0, 5.0, 5.0})
		input.SetCurrentAcceleration([]float64{0.0, 0.0, 0.0})

		input.SetTargetPosition([]float64{10.0, 10.0, 10.0})
		input.SetTargetVelocity([]float64{0.1, 0.1, 0.1})
		input.SetMaxVelocity([]float64{10.0, 10.0, 10.0})
		input.SetMaxAcceleration([]float64{5.0, 5.0, 5.0})
		input.SetMaxJerk([]float64{1.0, 1.0, 1.0})

		for ruckig.Update(input, output) == 0 {
			output.PassToInput(input)
		}

		fmt.Printf("Trajectory duration: %.2f [s]\n", output.TrajectoryDuration())
	})

	t.Run("Low Jerk Limits", func(t *testing.T) {
		input := NewInputParameter(3)
		output := NewOutputParameter(3)
		defer input.Destroy()
		defer output.Destroy()

		input.SetCurrentPosition([]float64{0.0, 0.0, 0.0})
		input.SetCurrentVelocity([]float64{0.0, 0.0, 0.0})
		input.SetCurrentAcceleration([]float64{0.0, 0.0, 0.0})

		input.SetTargetPosition([]float64{1.0, 1.0, 1.0})
		input.SetTargetVelocity([]float64{0.0, 0.0, 0.0})
		input.SetMaxVelocity([]float64{1.0, 1.0, 1.0})
		input.SetMaxAcceleration([]float64{0.5, 0.5, 0.5})
		input.SetMaxJerk([]float64{0.1, 0.1, 0.1})

		for ruckig.Update(input, output) == 0 {
			output.PassToInput(input)
		}

		fmt.Printf("Trajectory duration: %.2f [s]\n", output.TrajectoryDuration())
	})

	t.Run("Non-Uniform Constraints Across Degrees of Freedom", func(t *testing.T) {
		input := NewInputParameter(3)
		output := NewOutputParameter(3)
		defer input.Destroy()
		defer output.Destroy()

		input.SetCurrentPosition([]float64{0.0, 0.0, 0.0})
		input.SetCurrentVelocity([]float64{0.0, 0.0, 0.0})
		input.SetCurrentAcceleration([]float64{0.0, 0.0, 0.0})

		input.SetTargetPosition([]float64{5.0, 5.0, 5.0})
		input.SetTargetVelocity([]float64{0.0, 0.0, 0.0})
		input.SetMaxVelocity([]float64{2.0, 4.0, 6.0})
		input.SetMaxAcceleration([]float64{1.0, 2.0, 3.0})
		input.SetMaxJerk([]float64{0.5, 1.0, 1.5})

		for ruckig.Update(input, output) == 0 {
			output.PassToInput(input)
		}

		fmt.Printf("Trajectory duration: %.2f [s]\n", output.TrajectoryDuration())
	})

	t.Run("Rapid Changes in Target Position", func(t *testing.T) {
		input := NewInputParameter(3)
		output := NewOutputParameter(3)
		defer input.Destroy()
		defer output.Destroy()

		input.SetCurrentPosition([]float64{0.0, 0.0, 0.0})
		input.SetCurrentVelocity([]float64{0.0, 0.0, 0.0})
		input.SetCurrentAcceleration([]float64{0.0, 0.0, 0.0})

		input.SetTargetPosition([]float64{20.0, -20.0, 10.0})
		input.SetTargetVelocity([]float64{0.0, 0.0, 0.0})
		input.SetMaxVelocity([]float64{10.0, 10.0, 10.0})
		input.SetMaxAcceleration([]float64{5.0, 5.0, 5.0})
		input.SetMaxJerk([]float64{2.0, 2.0, 2.0})

		for ruckig.Update(input, output) == 0 {
			output.PassToInput(input)
		}

		fmt.Printf("Trajectory duration: %.2f [s]\n", output.TrajectoryDuration())
	})
}

func TestRuckigPerformance(t *testing.T) {
	ruckig := NewRuckig(0.01, 3)
	if ruckig == nil {
		t.Fatal("Failed to create Ruckig instance")
	}
	defer ruckig.Destroy()

	input := NewInputParameter(3)
	output := NewOutputParameter(3)
	defer input.Destroy()
	defer output.Destroy()

	// Set up parameters for performance testing
	input.SetCurrentPosition([]float64{0.0, 0.0, 0.0})
	input.SetCurrentVelocity([]float64{0.0, 0.0, 0.0})
	input.SetCurrentAcceleration([]float64{0.0, 0.0, 0.0})
	input.SetTargetPosition([]float64{100.0, 100.0, 100.0})
	input.SetTargetVelocity([]float64{0.0, 0.0, 0.0})
	input.SetMaxVelocity([]float64{10.0, 10.0, 10.0})
	input.SetMaxAcceleration([]float64{5.0, 5.0, 5.0})
	input.SetMaxJerk([]float64{2.0, 2.0, 2.0})

	// Measure performance
	startTime := time.Now()
	updateCount := 0

	for ruckig.Update(input, output) == 0 {
		output.PassToInput(input)
		updateCount++
	}

	duration := time.Since(startTime)
	fmt.Printf("Performance test: %d updates in %s\n", updateCount, duration)
}
