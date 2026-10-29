# Landing backend build and regression gate.
#
# The backend is the Rust crate in ../rust/landing (cargo). This Makefile keeps
# the historical entry points: `make -C CLanding` produces build/landing_backend
# and `make -C CLanding test` runs the C-ABI regression tests in ../Validation
# against the Rust static library (liblanding.a), which exports the same C
# interface that the public headers in include/ describe.
.DEFAULT_GOAL := all

CC ?= clang
ifeq ($(origin CC),default)
CC := clang
endif
CFLAGS ?= -O2 -std=c17 -Wall -Wextra -Werror -pthread
CPPFLAGS ?= -Iinclude -Iapp -Iguidance -Iprediction -Icontrol -Ivehicle -Icalibration -Itransport -Itelemetry -Ipersistence -I../ShuttleSim/include
LDFLAGS ?= -pthread
LDLIBS ?= -lm -lsqlite3 -ldl

REPO_ROOT := ..
BUILD ?= build
VALIDATION_ROOT := $(REPO_ROOT)/Validation
TOOLS_ROOT := $(REPO_ROOT)/Tools

CARGO ?= cargo
RUST_MANIFEST := $(REPO_ROOT)/rust/landing/Cargo.toml
RUST_TARGET_DIR ?= $(REPO_ROOT)/rust/landing/target
# Match the Python interpreter's architecture on Intel-Python-on-Apple-Silicon setups
# (same rule as the top-level Makefile's `ascent` target).
RUST_TRIPLE ?=
ifeq ($(RUST_TRIPLE),)
RUST_OUT := $(RUST_TARGET_DIR)/release
RUST_TARGET_FLAG :=
else
RUST_OUT := $(RUST_TARGET_DIR)/$(RUST_TRIPLE)/release
RUST_TARGET_FLAG := --target $(RUST_TRIPLE)
endif
RUST_BIN := $(RUST_OUT)/landing_backend
RUST_LIB := $(RUST_OUT)/liblanding.a

# Built with the C ABI headers that the Rust library implements.
CNANO_ROOT := $(REPO_ROOT)/ThirdParty/krpc-cnano-0.6.0
NANOPB_ROOT := $(REPO_ROOT)/ThirdParty/nanopb-0.4.9.1
CNANO_COMPAT_ROOT := $(BUILD)/cnano_compat
CNANO_COMPAT := $(CNANO_COMPAT_ROOT)/krpc_cnano
CNANO_INCLUDE_FLAGS := -isystem $(CNANO_ROOT)/include -isystem $(CNANO_COMPAT_ROOT) -isystem $(NANOPB_ROOT)
CNANO_DEFINES := -DKRPC_COMMUNICATION_CUSTOM=1 -DKRPC_ERROR_MESSAGES=1 -DKRPC_ERROR_MESSAGE_LENGTH=512
PROJECT_CPPFLAGS := $(CPPFLAGS) -D_DEFAULT_SOURCE $(CNANO_INCLUDE_FLAGS) $(CNANO_DEFINES)
CNANO_HEADERS := $(CNANO_COMPAT)/pb.h $(CNANO_COMPAT)/pb_common.h $(CNANO_COMPAT)/pb_encode.h $(CNANO_COMPAT)/pb_decode.h

TARGET := $(BUILD)/landing_backend

.PHONY: all clean rust test qualification deorbit-test architecture-test mm305-admission-test taem-frames-energy-test taem-native-stack-test vessel-physics-test flight-control-yaw-test flight-control-quaternion-test backend-protocol-test mm305-recovery-monitor-test
.SECONDARY: $(CNANO_HEADERS)

all: $(TARGET)

# cargo decides what is out of date; the copy is skipped when nothing changed.
rust:
	$(CARGO) build --release $(RUST_TARGET_FLAG) --manifest-path $(RUST_MANIFEST)

$(RUST_BIN) $(RUST_LIB): rust

$(TARGET): $(RUST_BIN) $(CNANO_HEADERS) | $(BUILD)
	cmp -s $(RUST_BIN) $@ || cp $(RUST_BIN) $@

debug:
	$(CARGO) build $(RUST_TARGET_FLAG) --manifest-path $(RUST_MANIFEST)
	mkdir -p $(BUILD)
	cp $(RUST_TARGET_DIR)/$(if $(RUST_TRIPLE),$(RUST_TRIPLE)/)debug/landing_backend $(TARGET)

$(CNANO_COMPAT)/%.h: $(NANOPB_ROOT)/%.h
	mkdir -p $(CNANO_COMPAT)
	cp $< $@

$(BUILD):
	mkdir -p $@

clean:
	rm -rf $(BUILD)
	$(CARGO) clean --manifest-path $(RUST_MANIFEST)

# Small current regression gate. Every default test uses supported headers/APIs;
# none includes a production .c file or depends on file-local guidance helpers.
architecture-test: $(TARGET)
	$(CC) $(PROJECT_CPPFLAGS) $(CFLAGS) \
		$(VALIDATION_ROOT)/ShuttleArchitectureContractTests.c \
		$(RUST_LIB) $(LDFLAGS) $(LDLIBS) \
		-o $(BUILD)/shuttle_architecture_contract_tests
	$(BUILD)/shuttle_architecture_contract_tests

mm305-admission-test: $(TARGET)
	$(CC) $(PROJECT_CPPFLAGS) $(CFLAGS) \
		$(VALIDATION_ROOT)/MM305AdmissionTests.c \
		$(RUST_LIB) \
		$(RUST_LIB) $(LDFLAGS) $(LDLIBS) \
		-o $(BUILD)/mm305_admission_tests
	$(BUILD)/mm305_admission_tests

taem-frames-energy-test: | $(BUILD)
	$(CC) $(PROJECT_CPPFLAGS) $(CFLAGS) \
		$(VALIDATION_ROOT)/TAEMFramesEnergyTests.c $(RUST_LIB) \
		$(LDFLAGS) $(LDLIBS) -o $(BUILD)/taem_frames_energy_tests
	$(BUILD)/taem_frames_energy_tests

taem-native-stack-test: $(TARGET)
	$(CC) $(PROJECT_CPPFLAGS) $(CFLAGS) \
		$(VALIDATION_ROOT)/TAEMNativeStackTests.c \
		$(RUST_LIB) \
		$(LDFLAGS) $(LDLIBS) -o $(BUILD)/taem_native_stack_tests
	cd $(REPO_ROOT) && "$(abspath $(BUILD)/taem_native_stack_tests)"

.PHONY: final-delivery-parity-test
final-delivery-parity-test: $(TARGET)
	$(CC) $(PROJECT_CPPFLAGS) $(CFLAGS) \
		$(VALIDATION_ROOT)/FinalDeliveryParityTests.c \
		$(RUST_LIB) \
		$(LDFLAGS) $(LDLIBS) -o $(BUILD)/final_delivery_parity_tests
	cd $(REPO_ROOT) && "$(abspath $(BUILD)/final_delivery_parity_tests)"

test: final-delivery-parity-test

vessel-physics-test: $(TARGET)
	$(CC) $(PROJECT_CPPFLAGS) $(CFLAGS) \
		$(VALIDATION_ROOT)/VesselPhysicsTests.c \
		$(RUST_LIB) \
		$(LDFLAGS) $(LDLIBS) -o $(BUILD)/vessel_physics_tests
	$(BUILD)/vessel_physics_tests

flight-control-yaw-test: $(TARGET)
	$(CC) $(PROJECT_CPPFLAGS) $(CFLAGS) \
		$(VALIDATION_ROOT)/FlightControlYawTests.c $(RUST_LIB) \
		$(LDFLAGS) $(LDLIBS) -o $(BUILD)/flight_control_yaw_tests
	$(BUILD)/flight_control_yaw_tests

flight-control-quaternion-test: $(TARGET)
	$(CC) $(PROJECT_CPPFLAGS) $(CFLAGS) \
		$(VALIDATION_ROOT)/FlightControlQuaternionTests.c $(RUST_LIB) \
		$(LDFLAGS) $(LDLIBS) -o $(BUILD)/flight_control_quaternion_tests
	$(BUILD)/flight_control_quaternion_tests

.PHONY: taem-tracker-allocation-test
taem-tracker-allocation-test: $(TARGET)
	$(CC) $(PROJECT_CPPFLAGS) $(CFLAGS) \
		$(VALIDATION_ROOT)/TAEMTrackerAllocationTests.c \
		$(RUST_LIB) \
		$(LDFLAGS) $(LDLIBS) -o $(BUILD)/taem_tracker_allocation_tests
	cd $(REPO_ROOT) && "$(abspath $(BUILD)/taem_tracker_allocation_tests)"

# This inexpensive test protects the UT/longitude transform consumed by the
# native MM305 solver when a deorbit originates from a live KSP orbital state.
.PHONY: terminal-planet-frame-test
terminal-planet-frame-test: $(TARGET)
	$(CC) $(PROJECT_CPPFLAGS) $(CFLAGS) \
		$(VALIDATION_ROOT)/TerminalPlanetFrameTests.c \
		$(RUST_LIB) \
		$(LDFLAGS) $(LDLIBS) -o $(BUILD)/terminal_planet_frame_tests
	cd $(REPO_ROOT) && "$(abspath $(BUILD)/terminal_planet_frame_tests)"

test: terminal-planet-frame-test

test: taem-tracker-allocation-test

.PHONY: terminal-propagation-convergence-test
terminal-propagation-convergence-test: $(TARGET)
	$(CC) $(PROJECT_CPPFLAGS) $(CFLAGS) \
		$(VALIDATION_ROOT)/TerminalPropagationConvergenceTests.c \
		$(RUST_LIB) \
		$(LDFLAGS) $(LDLIBS) -o $(BUILD)/terminal_propagation_convergence_tests
	cd $(REPO_ROOT) && "$(abspath $(BUILD)/terminal_propagation_convergence_tests)"

test: terminal-propagation-convergence-test

$(BUILD)/sim.o: ../ShuttleSim/src/sim.c | $(BUILD)
	$(CC) $(PROJECT_CPPFLAGS) $(CFLAGS) -MMD -MP -c $< -o $@

$(BUILD)/scenario.o: ../ShuttleSim/src/scenario.c | $(BUILD)
	$(CC) $(PROJECT_CPPFLAGS) $(CFLAGS) -MMD -MP -c $< -o $@

-include $(BUILD)/sim.d $(BUILD)/scenario.d

backend-protocol-test: $(TARGET)
	cd $(REPO_ROOT) && python3 Validation/BackendProtocolTests.py "$(abspath $(TARGET))"

mm305-feasibility-test: $(TARGET)
	$(CC) $(PROJECT_CPPFLAGS) $(CFLAGS) \
		$(VALIDATION_ROOT)/MM305FeasibilityPropertyTests.c \
		$(RUST_LIB) \
		$(LDFLAGS) $(LDLIBS) -o $(BUILD)/mm305_feasibility_tests
	cd $(REPO_ROOT) && "$(abspath $(BUILD)/mm305_feasibility_tests)" $(MM305_FEASIBILITY_ARGS)

mm305-recovery-monitor-test: $(TARGET)
	$(CC) $(PROJECT_CPPFLAGS) $(CFLAGS) \
		$(VALIDATION_ROOT)/MM305RecoveryMonitorTests.c \
		$(RUST_LIB) \
		$(LDFLAGS) $(LDLIBS) -o $(BUILD)/mm305_recovery_monitor_tests
	$(BUILD)/mm305_recovery_monitor_tests

touchdown-quality-test: $(TARGET) $(BUILD)/sim.o $(BUILD)/scenario.o
	$(CC) $(PROJECT_CPPFLAGS) $(CFLAGS) \
		$(VALIDATION_ROOT)/TouchdownQualityTests.c \
		$(BUILD)/sim.o $(BUILD)/scenario.o \
		$(RUST_LIB) \
		$(LDFLAGS) $(LDLIBS) -o $(BUILD)/touchdown_quality_tests
	cd $(REPO_ROOT) && "$(abspath $(BUILD)/touchdown_quality_tests)"

entry-forecast-admission-test: $(TARGET)
	$(CC) $(PROJECT_CPPFLAGS) $(CFLAGS) \
		$(VALIDATION_ROOT)/EntryForecastAdmissionTests.c \
		$(RUST_LIB) \
		$(LDFLAGS) $(LDLIBS) -o $(BUILD)/entry_forecast_admission_tests
	cd $(REPO_ROOT) && "$(abspath $(BUILD)/entry_forecast_admission_tests)"

deorbit-test: $(TARGET)
	$(CC) $(PROJECT_CPPFLAGS) $(CFLAGS) $(VALIDATION_ROOT)/DeorbitStageAdmissionTests.c \
		$(RUST_LIB) \
		$(LDFLAGS) $(LDLIBS) -o $(BUILD)/deorbit_stage_admission_tests
	$(CC) $(PROJECT_CPPFLAGS) $(CFLAGS) $(VALIDATION_ROOT)/EngineRoleTests.c \
		$(RUST_LIB) \
		$(LDFLAGS) $(LDLIBS) -o $(BUILD)/engine_role_tests
	$(CC) $(PROJECT_CPPFLAGS) $(CFLAGS) $(VALIDATION_ROOT)/ThrustAlignmentTests.c \
		$(RUST_LIB) \
		$(LDFLAGS) $(LDLIBS) -o $(BUILD)/thrust_alignment_tests
	cd $(REPO_ROOT) && "$(abspath $(BUILD)/deorbit_stage_admission_tests)" && \
		"$(abspath $(BUILD)/engine_role_tests)" && \
		"$(abspath $(BUILD)/thrust_alignment_tests)"
	@echo "Deorbit planning, admission, engine-role and thrust-alignment tests passed."

# Rust-side tests: white-box regressions that exercise file-private functions
# (the deorbit window search and the polar MM305 inlet / Final capture geometry).
.PHONY: rust-test
rust-test:
	$(CARGO) test --release $(RUST_TARGET_FLAG) --manifest-path $(RUST_MANIFEST)

# Fast offline regression gate (~10 s); must pass on every commit.
test: rust-test entry-forecast-admission-test architecture-test mm305-admission-test taem-frames-energy-test vessel-physics-test flight-control-yaw-test flight-control-quaternion-test backend-protocol-test mm305-recovery-monitor-test touchdown-quality-test deorbit-test

# Slow physical gates (~4 min). mm305-feasibility-test is a documented open
# gate: it currently fails its non-vacuity check (see VALIDATION.md).
qualification: test taem-native-stack-test mm305-feasibility-test

.PHONY: throttle-feedback-test
throttle-feedback-test: $(TARGET)
	$(CC) $(PROJECT_CPPFLAGS) $(CFLAGS) $(VALIDATION_ROOT)/ThrottleFeedbackTests.c \
		$(RUST_LIB) $(LDFLAGS) $(LDLIBS) \
		-o $(BUILD)/throttle_feedback_tests
	$(BUILD)/throttle_feedback_tests

test: throttle-feedback-test

# Offline analysis tools linked against the Rust library; none connects to KSP.
# Offline deorbit planner benchmark. The polar fixture reconstructs an actual
# recorded pre-plan orbital state.
$(BUILD)/benchmark_deorbit_planning: $(TOOLS_ROOT)/benchmark_deorbit_planning.c $(TARGET)
	$(CC) $(PROJECT_CPPFLAGS) $(CFLAGS) $< $(RUST_LIB) $(LDFLAGS) $(LDLIBS) -o $@

.PHONY: benchmark-deorbit
benchmark-deorbit: $(BUILD)/benchmark_deorbit_planning
	cd $(REPO_ROOT) && KSP_LANDER_TRACE_PLAN_TIMING=1 "$(abspath $<)" --polar-plan

# Frozen MM304 live-state energy/TAEM inlet audit.
$(BUILD)/mm304_energy_replay: $(TOOLS_ROOT)/mm304_energy_replay.c $(TARGET)
	$(CC) $(PROJECT_CPPFLAGS) $(CFLAGS) $< $(RUST_LIB) $(LDFLAGS) $(LDLIBS) -o $@

$(BUILD)/predictor_recorded_control_replay: $(TOOLS_ROOT)/predictor_recorded_control_replay.c $(TARGET)
	$(CC) $(PROJECT_CPPFLAGS) $(CFLAGS) $< $(RUST_LIB) $(LDFLAGS) $(LDLIBS) -o $@

.PHONY: spiral-hac-test
spiral-hac-test: $(TARGET)
	$(CC) $(PROJECT_CPPFLAGS) $(CFLAGS) $(VALIDATION_ROOT)/SpiralHACTests.c \
		$(RUST_LIB) \
		$(LDFLAGS) $(LDLIBS) -o $(BUILD)/spiral_hac_tests
	cd $(REPO_ROOT) && "$(abspath $(BUILD)/spiral_hac_tests)" $(SPIRAL_ARGS)
