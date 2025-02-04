package goRuckig

/*
#cgo CXXFLAGS: -Iruckig/include -std=c++17
#cgo LDFLAGS: -Lruckig/build -lruckig

// Include <stdbool.h> for C boolean type compatibility
#include <stdbool.h>

typedef struct {
    double delta_time;
    int dof;
} RuckigInstance;

RuckigInstance* create_ruckig_instance(double delta_time, int dof);
void destroy_ruckig_instance(RuckigInstance* instance);
void ruckig_reset(RuckigInstance* instance);
bool ruckig_validate_input(void* input, bool check_current_state, bool check_target_state);
void* create_input_parameter(int dof);
void destroy_input_parameter(void* input);
void* create_output_parameter(int dof);
void destroy_output_parameter(void* output);
int ruckig_calculate(void* instance, void* input, void* output);
int ruckig_update(void* instance, void* input, void* output);
void set_current_position(void* input, double* positions, int dof);
void set_target_position(void* input, double* positions, int dof);
void set_current_velocity(void* input, double* velocities, int dof);
void set_current_acceleration(void* input, double* accelerations, int dof);
void set_target_velocity(void* input, double* velocities, int dof);
void set_target_acceleration(void* input, double* accelerations, int dof);
void set_max_velocity(void* input, double* velocities, int dof);
void set_max_acceleration(void* input, double* accelerations, int dof);
void set_max_jerk(void* input, double* jerks, int dof);
double get_output_time(void* output);
void get_new_position(void* output, double* positions, int dof);
void pass_output_to_input(void* output, void* input);
double get_trajectory_duration(void* output);
*/
import "C"
import "unsafe"

// Ruckig represents the Ruckig OTG instance
type Ruckig struct {
	instance unsafe.Pointer
	dof      int
}

// InputParameter represents the input parameters for Ruckig
type InputParameter struct {
	ptr unsafe.Pointer
}

// OutputParameter represents the output parameters for Ruckig
type OutputParameter struct {
	ptr unsafe.Pointer
}

// NewRuckig creates a new Ruckig instance
func NewRuckig(deltaTime float64, dof int) *Ruckig {
	instance := C.create_ruckig_instance(C.double(deltaTime), C.int(dof))
	if instance == nil {
		return nil // Handle the case where instance creation fails
	}
	return &Ruckig{instance: unsafe.Pointer(instance), dof: dof}
}

// Destroy releases the Ruckig instance
func (r *Ruckig) Destroy() {
	if r.instance != nil {
		C.destroy_ruckig_instance((*C.RuckigInstance)(r.instance))
		r.instance = nil
	}
}

// Reset forces a new trajectory calculation
func (r *Ruckig) Reset() {
	if r.instance != nil {
		C.ruckig_reset((*C.RuckigInstance)(r.instance))
	}
}

// NewInputParameter creates a new InputParameter instance
func NewInputParameter(dof int) *InputParameter {
	ptr := C.create_input_parameter(C.int(dof))
	if ptr == nil {
		return nil
	}
	return &InputParameter{ptr: unsafe.Pointer(ptr)}
}

// Destroy releases the InputParameter instance
func (in *InputParameter) Destroy() {
	if in.ptr != nil {
		C.destroy_input_parameter(in.ptr)
		in.ptr = nil
	}
}

// NewOutputParameter creates a new OutputParameter instance
func NewOutputParameter(dof int) *OutputParameter {
	ptr := C.create_output_parameter(C.int(dof))
	if ptr == nil {
		return nil
	}
	return &OutputParameter{ptr: unsafe.Pointer(ptr)}
}

// Destroy releases the OutputParameter instance
func (out *OutputParameter) Destroy() {
	if out.ptr != nil {
		C.destroy_output_parameter(out.ptr)
		out.ptr = nil
	}
}

// ValidateInput validates the input parameters
func (in *InputParameter) ValidateInput(checkCurrentState, checkTargetState bool) bool {
	return bool(C.ruckig_validate_input(in.ptr, C.bool(checkCurrentState), C.bool(checkTargetState)))
}

// Calculate performs trajectory calculation
func (r *Ruckig) Calculate(input *InputParameter, output *OutputParameter) int {
	if r.instance == nil || input.ptr == nil || output.ptr == nil {
		return -1 // Return an error code if any instance is nil
	}
	return int(C.ruckig_calculate(r.instance, input.ptr, output.ptr))
}

// Update updates the trajectory and retrieves the next output state
func (r *Ruckig) Update(input *InputParameter, output *OutputParameter) int {
	if r.instance == nil || input.ptr == nil || output.ptr == nil {
		return -1 // Return an error code if any instance is nil
	}
	return int(C.ruckig_update(r.instance, input.ptr, output.ptr))
}

// SetCurrentPosition sets the current position for the input parameters
func (in *InputParameter) SetCurrentPosition(positions []float64) {
	C.set_current_position(in.ptr, (*C.double)(&positions[0]), C.int(len(positions)))
}

// SetTargetPosition sets the target position for the input parameters
func (in *InputParameter) SetTargetPosition(positions []float64) {
	C.set_target_position(in.ptr, (*C.double)(&positions[0]), C.int(len(positions)))
}

// SetCurrentVelocity sets the current velocity for the input parameters
func (in *InputParameter) SetCurrentVelocity(velocities []float64) {
	C.set_current_velocity(in.ptr, (*C.double)(&velocities[0]), C.int(len(velocities)))
}

// SetCurrentAcceleration sets the current acceleration for the input parameters
func (in *InputParameter) SetCurrentAcceleration(accelerations []float64) {
	C.set_current_acceleration(in.ptr, (*C.double)(&accelerations[0]), C.int(len(accelerations)))
}

// SetTargetVelocity sets the target velocity for the input parameters
func (in *InputParameter) SetTargetVelocity(velocities []float64) {
	C.set_target_velocity(in.ptr, (*C.double)(&velocities[0]), C.int(len(velocities)))
}

// SetTargetAcceleration sets the target acceleration for the input parameters
func (in *InputParameter) SetTargetAcceleration(accelerations []float64) {
	C.set_target_acceleration(in.ptr, (*C.double)(&accelerations[0]), C.int(len(accelerations)))
}

// SetMaxVelocity sets the maximum velocity for the input parameters
func (in *InputParameter) SetMaxVelocity(velocities []float64) {
	C.set_max_velocity(in.ptr, (*C.double)(&velocities[0]), C.int(len(velocities)))
}

// SetMaxAcceleration sets the maximum acceleration for the input parameters
func (in *InputParameter) SetMaxAcceleration(accelerations []float64) {
	C.set_max_acceleration(in.ptr, (*C.double)(&accelerations[0]), C.int(len(accelerations)))
}

// SetMaxJerk sets the maximum jerk for the input parameters
func (in *InputParameter) SetMaxJerk(jerks []float64) {
	C.set_max_jerk(in.ptr, (*C.double)(&jerks[0]), C.int(len(jerks)))
}

// Time returns the current time from the output parameters
func (out *OutputParameter) Time() float64 {
	return float64(C.get_output_time(out.ptr))
}

// NewPosition returns the new position from the output parameters
func (out *OutputParameter) NewPosition() []float64 {
	dof := 3 // Adjust this to match the degrees of freedom
	positions := make([]float64, dof)
	C.get_new_position(out.ptr, (*C.double)(&positions[0]), C.int(dof))
	return positions
}

// PassToInput passes the output parameters to the input for the next cycle
func (out *OutputParameter) PassToInput(input *InputParameter) {
	C.pass_output_to_input(out.ptr, input.ptr)
}

// TrajectoryDuration returns the duration of the trajectory
func (out *OutputParameter) TrajectoryDuration() float64 {
	return float64(C.get_trajectory_duration(out.ptr))
}
