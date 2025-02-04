#include <vector>
#include "ruckig/ruckig.hpp"
extern "C" {
    void* create_ruckig_instance(double delta_time, int dof) {
        try {
            return new ruckig::Ruckig<ruckig::DynamicDOFs>(dof, delta_time);
        } catch (...) {
            return nullptr;
        }
    }

    void destroy_ruckig_instance(void* instance) {
        try {
            delete static_cast<ruckig::Ruckig<ruckig::DynamicDOFs>*>(instance);
        } catch (...) {
        }
    }

    void* create_input_parameter(int dof) {
        try {
            return new ruckig::InputParameter<ruckig::DynamicDOFs>(dof);
        } catch (...) {
            return nullptr;
        }
    }

    void destroy_input_parameter(void* input) {
        try {
            delete static_cast<ruckig::InputParameter<ruckig::DynamicDOFs>*>(input);
        } catch (...) {
        }
    }

    void* create_output_parameter(int dof) {
        try {
            return new ruckig::OutputParameter<ruckig::DynamicDOFs>(dof);
        } catch (...) {
            return nullptr;
        }
    }

    void destroy_output_parameter(void* output) {
        try {
            delete static_cast<ruckig::OutputParameter<ruckig::DynamicDOFs>*>(output);
        } catch (...) {
        }
    }

    void ruckig_reset(void* instance) {
        try {
            auto* ruckig = static_cast<ruckig::Ruckig<ruckig::DynamicDOFs>*>(instance);
            ruckig->reset();
        } catch (...) {
        }
    }

    void* ruckig_filter_positions(void* input, double* threshold_distances, int dof_count, int* output_size) {
        try {
            auto* in = static_cast<ruckig::InputParameter<ruckig::DynamicDOFs>*>(input);
            std::vector<std::vector<double>> filtered_positions;
            std::vector<double> threshold_distance(dof_count);

            for (int i = 0; i < dof_count; ++i) {
                threshold_distance[i] = threshold_distances[i];
            }

            filtered_positions = in->intermediate_positions;
            *output_size = static_cast<int>(filtered_positions.size());

            return static_cast<void*>(new std::vector<std::vector<double>>(filtered_positions));
        } catch (...) {
            return nullptr;
        }
    }

    int ruckig_calculate(void* instance, void* input, void* output) {
        try {
            auto* ruckig = static_cast<ruckig::Ruckig<ruckig::DynamicDOFs>*>(instance);
            auto* in = static_cast<ruckig::InputParameter<ruckig::DynamicDOFs>*>(input);
            auto* out = static_cast<ruckig::OutputParameter<ruckig::DynamicDOFs>*>(output);
            return static_cast<int>(ruckig->calculate(*in, out->trajectory));
        } catch (...) {
            return static_cast<int>(ruckig::Result::ErrorInvalidInput);
        }
    }

    int ruckig_update(void* instance, void* input, void* output) {
        try {
            auto* ruckig = static_cast<ruckig::Ruckig<ruckig::DynamicDOFs>*>(instance);
            auto* in = static_cast<ruckig::InputParameter<ruckig::DynamicDOFs>*>(input);
            auto* out = static_cast<ruckig::OutputParameter<ruckig::DynamicDOFs>*>(output);
            return static_cast<int>(ruckig->update(*in, *out));
        } catch (...) {
            return static_cast<int>(ruckig::Result::ErrorInvalidInput);
        }
    }

    // Define ruckig_validate_input function
    bool ruckig_validate_input(void* input, bool checkCurrentState, bool checkTargetState) {
        try {
            auto* in = static_cast<ruckig::InputParameter<ruckig::DynamicDOFs>*>(input);
            return in->validate(checkCurrentState, checkTargetState);
        } catch (...) {
            return false;
        }
    }

    void set_current_position(void* input, double* positions, int dof) {
            try {
                auto* in = static_cast<ruckig::InputParameter<ruckig::DynamicDOFs>*>(input);
                in->current_position.assign(positions, positions + dof);
            } catch (...) {
            }
        }

    void set_target_position(void* input, double* positions, int dof) {
           try {
                auto* in = static_cast<ruckig::InputParameter<ruckig::DynamicDOFs>*>(input);
                in->target_position.assign(positions, positions + dof);
           } catch (...) {
           }
        }
    void set_current_velocity(void* input, double* velocities, int dof) {
            try {
                auto* in = static_cast<ruckig::InputParameter<ruckig::DynamicDOFs>*>(input);
                in->current_velocity.assign(velocities, velocities + dof);
            } catch (...) {
            }
        }
    void set_current_acceleration(void* input, double* accelerations, int dof) {
             try {
                 auto* in = static_cast<ruckig::InputParameter<ruckig::DynamicDOFs>*>(input);
                 in->current_acceleration.assign(accelerations, accelerations + dof);
             } catch (...) {
             }
         }
    void set_target_velocity(void* input, double* velocities, int dof) {
            try {
                auto* in = static_cast<ruckig::InputParameter<ruckig::DynamicDOFs>*>(input);
                in->target_velocity.assign(velocities, velocities + dof);
            } catch (...) {
            }
        }
    void set_target_acceleration(void* input, double* accelerations, int dof) {
            try {
                auto* in = static_cast<ruckig::InputParameter<ruckig::DynamicDOFs>*>(input);
                in->target_acceleration.assign(accelerations, accelerations + dof);
            } catch (...) {
            }
        }
    void set_max_velocity(void* input, double* velocities, int dof) {
            try {
                auto* in = static_cast<ruckig::InputParameter<ruckig::DynamicDOFs>*>(input);
                in->max_velocity.assign(velocities, velocities + dof);
            } catch (...) {
            }
        }
    void set_max_acceleration(void* input, double* accelerations, int dof) {
            try {
                auto* in = static_cast<ruckig::InputParameter<ruckig::DynamicDOFs>*>(input);
                in->max_acceleration.assign(accelerations, accelerations + dof);
            } catch (...) {
            }
        }

    void set_max_jerk(void* input, double* jerks, int dof) {
            try {
                auto* in = static_cast<ruckig::InputParameter<ruckig::DynamicDOFs>*>(input);
                in->max_jerk.assign(jerks, jerks + dof);
            } catch (...) {
            }
        }
    double get_output_time(void* output) {
            try {
                auto* out = static_cast<ruckig::OutputParameter<ruckig::DynamicDOFs>*>(output);
                return out->time;
            } catch (...) {
                return -1.0;
            }
        }

        // Function to get the new position from the OutputParameter
    void get_new_position(void* output, double* positions, int dof) {
            try {
                auto* out = static_cast<ruckig::OutputParameter<ruckig::DynamicDOFs>*>(output);
                for (int i = 0; i < dof; ++i) {
                    positions[i] = out->new_position[i];
                }
            } catch (...) {
            }
        }

        // Function to pass output parameters to input for the next cycle
    void pass_output_to_input(void* output, void* input) {
            try {
                auto* out = static_cast<ruckig::OutputParameter<ruckig::DynamicDOFs>*>(output);
                auto* in = static_cast<ruckig::InputParameter<ruckig::DynamicDOFs>*>(input);
                out->pass_to_input(*in);
            } catch (...) {
            }
        }

        // Function to get the trajectory duration from the OutputParameter
    double get_trajectory_duration(void* output) {
            try {
                auto* out = static_cast<ruckig::OutputParameter<ruckig::DynamicDOFs>*>(output);
                return out->trajectory.get_duration();
            } catch (...) {
                return -1.0;
            }
        }
}