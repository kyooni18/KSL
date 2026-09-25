# Keep plain `make` building the backend, not the validation-overlay helper.
.DEFAULT_GOAL := all

# make predefines CC=cc, so `CC ?= clang` would never apply; honour an explicit
# CC from the command line/environment and otherwise use clang as intended.
ifeq ($(origin CC),default)
CC := clang
endif
CFLAGS ?= -O2 -std=c17 -Wall -Wextra -Werror -pthread
CPPFLAGS ?= -Iinclude -Iapp -Iguidance -Iprediction -Icontrol -Ivehicle -Icalibration -Itransport -Itelemetry -Ipersistence -I../ShuttleSim/include
LDFLAGS ?= -pthread
LDLIBS ?= -lm -lsqlite3

REPO_ROOT := ..
BUILD ?= build
VALIDATION_ROOT := $(REPO_ROOT)/Validation
TOOLS_ROOT := $(REPO_ROOT)/Tools

TARGET := $(BUILD)/landing_backend
CNANO_ROOT := $(REPO_ROOT)/ThirdParty/krpc-cnano-0.6.0
NANOPB_ROOT := $(REPO_ROOT)/ThirdParty/nanopb-0.4.9.1
CNANO_COMPAT_ROOT := $(BUILD)/cnano_compat
CNANO_COMPAT := $(CNANO_COMPAT_ROOT)/krpc_cnano
CNANO_INCLUDE_FLAGS := -isystem $(CNANO_ROOT)/include -isystem $(CNANO_COMPAT_ROOT) -isystem $(NANOPB_ROOT)
CNANO_DEFINES := -DKRPC_COMMUNICATION_CUSTOM=1 -DKRPC_ERROR_MESSAGES=1 -DKRPC_ERROR_MESSAGE_LENGTH=512
PROJECT_CPPFLAGS := $(CPPFLAGS) -D_DEFAULT_SOURCE $(CNANO_INCLUDE_FLAGS) $(CNANO_DEFINES)
UPSTREAM_CNANO_WARNING_FLAGS := -Wno-error=unused-parameter -Wno-error=unused-function -Wno-error=missing-field-initializers

VPATH := app:guidance:prediction:control:vehicle:calibration:transport:telemetry:persistence:include

FLIGHT_PHYSICS_SOURCES := ../ShuttleSim/src/math3.c ../ShuttleSim/src/quat.c \
	../ShuttleSim/src/world.c ../ShuttleSim/src/aero.c ../ShuttleSim/src/attitude.c \
	../ShuttleSim/src/flight_physics.c
FLIGHT_PHYSICS_OBJECTS := $(patsubst ../ShuttleSim/src/%.c,$(BUILD)/flight_physics/%.o,$(FLIGHT_PHYSICS_SOURCES))

APP_SOURCES := controller.c main.c
GUIDANCE_SOURCES := entry_alpha.c entry_drag_reference.c entry_energy_control.c entry_lateral.c entry_exec.c taem_exec.c \
	guidance_common.c guidance_core.c guidance_entry.c guidance_taem.c guidance_final.c \
	guidance_terminal.c guidance.c taem_frames_energy.c taem_geometry.c terminal_model.c \
	terminal_propagator.c taem_reachability.c taem_tracker.c taem_planner.c terminal_solver.c \
	taem_candidate_search.c
PREDICTION_SOURCES := predictor.c decision_envelope.c async_prediction_policy.c
CONTROL_SOURCES := flight_control.c
VEHICLE_SOURCES := vessel_physics.c aero_certification.c math.c models.c
CALIBRATION_SOURCES := calibration.c calibration_observation.c trajectory_calibration.c
TRANSPORT_SOURCES := krpc.c krpc_cnano_client.c krpc_cnano_transport.c krpc_cnano_batch.c
TELEMETRY_SOURCES := json.c sim_telemetry.c planner_log.c flight_log_codec.c
PERSISTENCE_SOURCES := physics_store.c
SOURCES := $(APP_SOURCES) $(GUIDANCE_SOURCES) $(PREDICTION_SOURCES) $(CONTROL_SOURCES) \
	$(VEHICLE_SOURCES) $(CALIBRATION_SOURCES) $(TRANSPORT_SOURCES) $(TELEMETRY_SOURCES) \
	$(PERSISTENCE_SOURCES)
OBJECTS := $(SOURCES:%.c=$(BUILD)/%.o) $(FLIGHT_PHYSICS_OBJECTS)
DEPFILES := $(OBJECTS:.o=.d)

CNANO_SOURCE_NAMES := decoder encoder error krpc krpc.pb memory utils communication
CNANO_OBJECTS := $(CNANO_SOURCE_NAMES:%=$(BUILD)/cnano/%.o)
NANOPB_SOURCE_NAMES := pb_common pb_encode pb_decode
NANOPB_OBJECTS := $(NANOPB_SOURCE_NAMES:%=$(BUILD)/nanopb/%.o)
CNANO_HEADERS := $(CNANO_COMPAT)/pb.h $(CNANO_COMPAT)/pb_common.h $(CNANO_COMPAT)/pb_encode.h $(CNANO_COMPAT)/pb_decode.h

.PHONY: all clean debug test qualification architecture-test mm305-admission-test taem-frames-energy-test taem-native-stack-test vessel-physics-test entry-lateral-feedback-test backend-protocol-test
.SECONDARY: $(CNANO_HEADERS)

all: $(TARGET)

debug: CFLAGS := -O0 -g -std=c17 -Wall -Wextra -Werror -pthread
debug: $(TARGET)

$(TARGET): $(OBJECTS) $(CNANO_OBJECTS) $(NANOPB_OBJECTS)
	$(CC) $(LDFLAGS) $^ $(LDLIBS) -o $@

$(BUILD)/%.o: %.c $(CNANO_HEADERS) | $(BUILD)
	$(CC) $(PROJECT_CPPFLAGS) $(CFLAGS) -MMD -MP -c $< -o $@

$(BUILD)/flight_physics/%.o: ../ShuttleSim/src/%.c $(CNANO_HEADERS) | $(BUILD)/flight_physics
	$(CC) $(PROJECT_CPPFLAGS) $(CFLAGS) -MMD -MP -c $< -o $@

-include $(DEPFILES)

$(BUILD)/predictor_recorded_control_replay: $(TOOLS_ROOT)/predictor_recorded_control_replay.c $(BUILD)/physics_store.o $(BUILD)/vessel_physics.o $(BUILD)/json.o $(BUILD)/math.o
	$(CC) $(PROJECT_CPPFLAGS) $(CFLAGS) $< $(BUILD)/physics_store.o $(BUILD)/vessel_physics.o $(BUILD)/json.o $(BUILD)/math.o $(LDFLAGS) $(LDLIBS) -o $@

$(BUILD)/cnano/%.o: $(CNANO_ROOT)/src/%.c $(CNANO_HEADERS) | $(BUILD)/cnano
	$(CC) $(PROJECT_CPPFLAGS) $(CFLAGS) $(UPSTREAM_CNANO_WARNING_FLAGS) -include $(CURDIR)/include/krpc_cnano_transport.h -c $< -o $@

$(BUILD)/nanopb/%.o: $(NANOPB_ROOT)/%.c $(CNANO_HEADERS) | $(BUILD)/nanopb
	$(CC) -I$(NANOPB_ROOT) $(CFLAGS) -c $< -o $@

$(CNANO_COMPAT)/%.h: $(NANOPB_ROOT)/%.h
	mkdir -p $(CNANO_COMPAT)
	cp $< $@

$(BUILD):
	mkdir -p $@
$(BUILD)/cnano:
	mkdir -p $@
$(BUILD)/nanopb:
	mkdir -p $@
$(BUILD)/flight_physics:
	mkdir -p $@

clean:
	rm -rf $(BUILD)

# Small current regression gate. Every default test uses supported headers/APIs;
# none includes a production .c file or depends on file-local guidance helpers.
architecture-test: $(TARGET)
	$(CC) $(PROJECT_CPPFLAGS) $(CFLAGS) \
		$(VALIDATION_ROOT)/ShuttleArchitectureContractTests.c \
		$(BUILD)/entry_exec.o $(BUILD)/taem_exec.o $(LDFLAGS) $(LDLIBS) \
		-o $(BUILD)/shuttle_architecture_contract_tests
	$(BUILD)/shuttle_architecture_contract_tests

mm305-admission-test: $(TARGET)
	$(CC) $(PROJECT_CPPFLAGS) $(CFLAGS) \
		$(VALIDATION_ROOT)/MM305AdmissionTests.c \
		$(BUILD)/decision_envelope.o $(BUILD)/math.o $(BUILD)/models.o $(BUILD)/json.o \
		$(BUILD)/taem_planner.o $(BUILD)/taem_geometry.o $(BUILD)/taem_frames_energy.o $(BUILD)/taem_reachability.o \
		$(BUILD)/flight_physics/math3.o $(BUILD)/flight_physics/world.o $(BUILD)/flight_physics/aero.o \
		$(BUILD)/entry_exec.o $(BUILD)/taem_exec.o $(LDFLAGS) $(LDLIBS) \
		-o $(BUILD)/mm305_admission_tests
	$(BUILD)/mm305_admission_tests

taem-frames-energy-test: | $(BUILD)
	$(CC) $(PROJECT_CPPFLAGS) $(CFLAGS) \
		$(VALIDATION_ROOT)/TAEMFramesEnergyTests.c taem_frames_energy.c \
		$(LDFLAGS) -lm -o $(BUILD)/taem_frames_energy_tests
	$(BUILD)/taem_frames_energy_tests

taem-native-stack-test: $(TARGET)
	$(CC) $(PROJECT_CPPFLAGS) $(CFLAGS) \
		$(VALIDATION_ROOT)/TAEMNativeStackTests.c \
		$(filter-out $(BUILD)/main.o,$(OBJECTS)) $(CNANO_OBJECTS) $(NANOPB_OBJECTS) \
		$(LDFLAGS) $(LDLIBS) -o $(BUILD)/taem_native_stack_tests
	cd $(REPO_ROOT) && "$(abspath $(BUILD)/taem_native_stack_tests)"

vessel-physics-test: $(TARGET)
	$(CC) $(PROJECT_CPPFLAGS) $(CFLAGS) \
		$(VALIDATION_ROOT)/VesselPhysicsTests.c \
		$(filter-out $(BUILD)/main.o,$(OBJECTS)) $(CNANO_OBJECTS) $(NANOPB_OBJECTS) \
		$(LDFLAGS) $(LDLIBS) -o $(BUILD)/vessel_physics_tests
	$(BUILD)/vessel_physics_tests

entry-lateral-feedback-test: $(TARGET)
	$(CC) $(PROJECT_CPPFLAGS) $(CFLAGS) \
		$(VALIDATION_ROOT)/EntryLateralFeedbackTests.c $(BUILD)/entry_lateral.o \
		$(LDFLAGS) $(LDLIBS) -o $(BUILD)/entry_lateral_feedback_tests
	$(BUILD)/entry_lateral_feedback_tests

entry-energy-law-test: $(TARGET)
	$(CC) $(PROJECT_CPPFLAGS) $(CFLAGS) \
		$(VALIDATION_ROOT)/EntryEnergyLawTests.c $(BUILD)/entry_energy_control.o $(BUILD)/math.o \
		$(LDFLAGS) $(LDLIBS) -o $(BUILD)/entry_energy_law_tests
	$(BUILD)/entry_energy_law_tests

backend-protocol-test: $(TARGET)
	cd $(REPO_ROOT) && python3 Validation/BackendProtocolTests.py "$(abspath $(TARGET))"

test: architecture-test mm305-admission-test taem-frames-energy-test taem-native-stack-test vessel-physics-test entry-lateral-feedback-test entry-energy-law-test backend-protocol-test

qualification: test
