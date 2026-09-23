CC ?= clang
CFLAGS ?= -O2 -std=c17 -Wall -Wextra -Werror -pthread
CPPFLAGS ?= -I.
LDFLAGS ?= -pthread
LDLIBS ?= -lm -lsqlite3

BUILD ?= build
TARGET := $(BUILD)/landing_backend
CNANO_ROOT := ../ThirdParty/krpc-cnano-0.6.0
NANOPB_ROOT := ../ThirdParty/nanopb-0.4.9.1
CNANO_COMPAT_ROOT := $(BUILD)/cnano_compat
CNANO_COMPAT := $(CNANO_COMPAT_ROOT)/krpc_cnano
CNANO_INCLUDE_FLAGS := -isystem $(CNANO_ROOT)/include -isystem $(CNANO_COMPAT_ROOT) -isystem $(NANOPB_ROOT)
CNANO_DEFINES := -DKRPC_COMMUNICATION_CUSTOM=1 -DKRPC_ERROR_MESSAGES=1 -DKRPC_ERROR_MESSAGE_LENGTH=512
PROJECT_CPPFLAGS := $(CPPFLAGS) $(CNANO_INCLUDE_FLAGS) $(CNANO_DEFINES)
UPSTREAM_CNANO_WARNING_FLAGS := -Wno-error=unused-parameter -Wno-error=unused-function -Wno-error=missing-field-initializers

SOURCES := vessel_physics.c aero_certification.c json.c sim_telemetry.c math.c decision_envelope.c calibration_observation.c models.c planner_log.c flight_log_codec.c krpc.c krpc_cnano_client.c krpc_cnano_transport.c krpc_cnano_batch.c flight_control.c physics_store.c entry_alpha.c entry_drag_reference.c entry_lateral.c predictor.c guidance.c entry_exec.c taem_exec.c calibration.c trajectory_calibration.c async_prediction_policy.c controller.c main.c
OBJECTS := $(SOURCES:%.c=$(BUILD)/%.o)

CNANO_SOURCE_NAMES := decoder encoder error krpc krpc.pb memory utils communication
CNANO_OBJECTS := $(CNANO_SOURCE_NAMES:%=$(BUILD)/cnano/%.o)
NANOPB_SOURCE_NAMES := pb_common pb_encode pb_decode
NANOPB_OBJECTS := $(NANOPB_SOURCE_NAMES:%=$(BUILD)/nanopb/%.o)
CNANO_HEADERS := $(CNANO_COMPAT)/pb.h $(CNANO_COMPAT)/pb_common.h $(CNANO_COMPAT)/pb_encode.h $(CNANO_COMPAT)/pb_decode.h

.PHONY: all clean debug test qualification offline-acceptance cnano-transport-test native-component-tests guidance-attitude-slew-test
.SECONDARY: $(CNANO_HEADERS)

all: $(TARGET)

debug: CFLAGS := -O0 -g -std=c17 -Wall -Wextra -Werror -pthread
debug: $(TARGET)

$(TARGET): $(OBJECTS) $(CNANO_OBJECTS) $(NANOPB_OBJECTS)
	$(CC) $(LDFLAGS) $^ $(LDLIBS) -o $@

$(BUILD)/%.o: %.c landing.h decision_envelope.h calibration_observation.h planner_log.h flight_log_codec.h sim_telemetry.h async_prediction_policy.h entry_alpha.h entry_drag_reference.h entry_lateral.h entry_exec.h taem_exec.h json.h $(CNANO_HEADERS) | $(BUILD)
	$(CC) $(PROJECT_CPPFLAGS) $(CFLAGS) -c $< -o $@

$(BUILD)/predictor_recorded_control_replay: ../Tools/predictor_recorded_control_replay.c $(BUILD)/physics_store.o $(BUILD)/vessel_physics.o $(BUILD)/json.o $(BUILD)/math.o
	$(CC) $(PROJECT_CPPFLAGS) $(CFLAGS) $< $(BUILD)/physics_store.o $(BUILD)/vessel_physics.o $(BUILD)/json.o $(BUILD)/math.o $(LDFLAGS) $(LDLIBS) -o $@

$(BUILD)/cnano/%.o: $(CNANO_ROOT)/src/%.c $(CNANO_HEADERS) | $(BUILD)/cnano
	$(CC) $(PROJECT_CPPFLAGS) $(CFLAGS) $(UPSTREAM_CNANO_WARNING_FLAGS) -include $(CURDIR)/krpc_cnano_transport.h -c $< -o $@

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

clean:
	rm -rf $(BUILD)

cnano-transport-test: $(CNANO_HEADERS) $(CNANO_OBJECTS) $(NANOPB_OBJECTS) $(BUILD)/krpc_cnano_transport.o
	$(CC) $(PROJECT_CPPFLAGS) $(CFLAGS) ../Validation/CNanoFakeTransport.c ../Validation/CNanoTransportTests.c $(BUILD)/krpc_cnano_transport.o $(CNANO_OBJECTS) $(NANOPB_OBJECTS) $(LDFLAGS) $(LDLIBS) -o $(BUILD)/cnano_transport_tests
	$(BUILD)/cnano_transport_tests
	$(CC) $(PROJECT_CPPFLAGS) $(CFLAGS) ../Validation/CNanoFakeTransport.c ../Validation/CNanoBatchTests.c $(BUILD)/krpc_cnano_transport.o $(BUILD)/krpc_cnano_batch.o $(CNANO_OBJECTS) $(NANOPB_OBJECTS) $(LDFLAGS) $(LDLIBS) -o $(BUILD)/cnano_batch_tests
	$(BUILD)/cnano_batch_tests

native-component-tests: $(TARGET) sim-telemetry-semantics-test decision-envelope-test terminal-contract-test sim-planet-model-test guidance-attitude-slew-test
	$(CC) $(PROJECT_CPPFLAGS) $(CFLAGS) ../Validation/FlightControlTests.c $(BUILD)/flight_control.o $(BUILD)/math.o $(LDFLAGS) $(LDLIBS) -o $(BUILD)/flight_control_tests
	$(BUILD)/flight_control_tests
	$(CC) $(PROJECT_CPPFLAGS) $(CFLAGS) ../Validation/SimTelemetrySemanticsTests.c $(BUILD)/sim_telemetry.o $(BUILD)/models.o $(BUILD)/json.o $(BUILD)/math.o $(BUILD)/entry_exec.o $(BUILD)/taem_exec.o $(LDFLAGS) $(LDLIBS) -o $(BUILD)/sim_telemetry_semantics_tests
	$(BUILD)/sim_telemetry_semantics_tests
	$(CC) $(PROJECT_CPPFLAGS) $(CFLAGS) ../Validation/TaemInterfaceCaptureEnergyTests.c $(BUILD)/decision_envelope.o $(BUILD)/math.o $(BUILD)/models.o $(BUILD)/json.o $(BUILD)/entry_exec.o $(BUILD)/taem_exec.o $(LDFLAGS) $(LDLIBS) -o $(BUILD)/taem_interface_capture_energy_tests
	$(BUILD)/taem_interface_capture_energy_tests
	$(CC) $(PROJECT_CPPFLAGS) $(CFLAGS) ../Validation/VehicleFlightLogTests.c $(BUILD)/flight_log_codec.o $(BUILD)/json.o $(LDFLAGS) $(LDLIBS) -o $(BUILD)/vehicle_flight_log_tests
	$(BUILD)/vehicle_flight_log_tests
	$(CC) $(PROJECT_CPPFLAGS) $(CFLAGS) ../Validation/AsyncPredictionPolicyTests.c $(BUILD)/async_prediction_policy.o $(LDFLAGS) $(LDLIBS) -o $(BUILD)/async_prediction_policy_tests
	$(BUILD)/async_prediction_policy_tests
	$(CC) $(PROJECT_CPPFLAGS) $(CFLAGS) ../Validation/PhysicsStoreTests.c $(BUILD)/physics_store.o $(BUILD)/vessel_physics.o $(BUILD)/json.o $(BUILD)/math.o $(LDFLAGS) $(LDLIBS) -o $(BUILD)/physics_store_tests
	$(BUILD)/physics_store_tests
	$(CC) $(PROJECT_CPPFLAGS) $(CFLAGS) ../Validation/CNanoFakeTransport.c ../Validation/CNanoClientTests.c $(filter-out $(BUILD)/main.o,$(OBJECTS)) $(CNANO_OBJECTS) $(NANOPB_OBJECTS) $(LDFLAGS) $(LDLIBS) -o $(BUILD)/cnano_client_tests
	$(BUILD)/cnano_client_tests

guidance-attitude-slew-test: $(TARGET)
	$(CC) $(PROJECT_CPPFLAGS) $(CFLAGS) \
		../Validation/GuidanceAttitudeSlewTests.c \
		$(filter-out $(BUILD)/main.o $(BUILD)/guidance.o,$(OBJECTS)) \
		$(CNANO_OBJECTS) $(NANOPB_OBJECTS) $(LDFLAGS) $(LDLIBS) \
		-o $(BUILD)/guidance_attitude_slew_tests
	$(BUILD)/guidance_attitude_slew_tests

# Keep the established guidance/predictor regression suite and add the native
# C-Nano transport, flight-control and SQLite-store contracts to the default gate.
test: $(TARGET) cnano-transport-test native-component-tests mm304-offline-test
	$(CC) $(PROJECT_CPPFLAGS) $(CFLAGS) ../Validation/VesselPhysicsTests.c $(filter-out $(BUILD)/main.o,$(OBJECTS)) $(CNANO_OBJECTS) $(NANOPB_OBJECTS) $(LDFLAGS) $(LDLIBS) -o $(BUILD)/vessel_physics_tests
	$(BUILD)/vessel_physics_tests
	$(CC) $(PROJECT_CPPFLAGS) $(CFLAGS) \
		../Validation/ApproachSequenceContractTests.c \
		$(filter-out $(BUILD)/main.o $(BUILD)/guidance.o,$(OBJECTS)) $(CNANO_OBJECTS) $(NANOPB_OBJECTS) $(LDFLAGS) $(LDLIBS) -o $(BUILD)/approach_sequence_contract_tests
	$(BUILD)/approach_sequence_contract_tests
	$(CC) $(PROJECT_CPPFLAGS) $(CFLAGS) ../Validation/TerminalPreviewRecoveryTests.c $(filter-out $(BUILD)/main.o $(BUILD)/guidance.o,$(OBJECTS)) $(CNANO_OBJECTS) $(NANOPB_OBJECTS) $(LDFLAGS) $(LDLIBS) -o $(BUILD)/terminal_preview_recovery_tests
	$(BUILD)/terminal_preview_recovery_tests
	$(CC) $(PROJECT_CPPFLAGS) $(CFLAGS) \
		../Validation/EntryAlphaTests.c $(BUILD)/entry_alpha.o $(BUILD)/math.o $(LDFLAGS) $(LDLIBS) -o $(BUILD)/entry_alpha_tests
	$(BUILD)/entry_alpha_tests
	$(CC) $(PROJECT_CPPFLAGS) $(CFLAGS) \
		../Validation/EntryLateralTests.c $(BUILD)/entry_lateral.o $(LDFLAGS) $(LDLIBS) -o $(BUILD)/entry_lateral_tests
	$(BUILD)/entry_lateral_tests
	$(CC) $(PROJECT_CPPFLAGS) $(CFLAGS) \
		../Validation/EntryExecTests.c entry_exec.c $(LDFLAGS) $(LDLIBS) -o $(BUILD)/entry_exec_tests
	$(BUILD)/entry_exec_tests
	$(CC) $(PROJECT_CPPFLAGS) $(CFLAGS) \
		../Validation/TAEMExecTests.c $(BUILD)/taem_exec.o $(LDFLAGS) $(LDLIBS) -o $(BUILD)/taem_exec_tests
	$(BUILD)/taem_exec_tests
	$(CC) $(PROJECT_CPPFLAGS) $(CFLAGS) \
		../Validation/EntryDragReferenceTests.c $(BUILD)/entry_drag_reference.o $(BUILD)/entry_alpha.o $(BUILD)/math.o $(BUILD)/models.o $(BUILD)/json.o $(BUILD)/entry_exec.o $(BUILD)/taem_exec.o $(LDFLAGS) $(LDLIBS) -o $(BUILD)/entry_drag_reference_tests
	$(BUILD)/entry_drag_reference_tests
	$(CC) $(PROJECT_CPPFLAGS) $(CFLAGS) \
		../Validation/EntryPredictorSupervisionTests.c \
		$(BUILD)/vessel_physics.o $(BUILD)/json.o $(BUILD)/math.o $(BUILD)/models.o $(BUILD)/predictor.o $(BUILD)/guidance.o $(BUILD)/decision_envelope.o \
		$(BUILD)/entry_alpha.o $(BUILD)/entry_drag_reference.o $(BUILD)/entry_lateral.o $(BUILD)/entry_exec.o $(BUILD)/taem_exec.o \
		$(LDFLAGS) $(LDLIBS) -o $(BUILD)/entry_predictor_supervision_tests
	$(BUILD)/entry_predictor_supervision_tests
	$(CC) $(PROJECT_CPPFLAGS) $(CFLAGS) ../Validation/PlannerLogTests.c $(filter-out $(BUILD)/main.o,$(OBJECTS)) $(CNANO_OBJECTS) $(NANOPB_OBJECTS) $(LDFLAGS) $(LDLIBS) -o $(BUILD)/planner_log_tests
	$(BUILD)/planner_log_tests
	$(CC) $(PROJECT_CPPFLAGS) $(CFLAGS) \
		../Validation/ShuttleArchitectureContractTests.c \
		$(BUILD)/entry_exec.o $(BUILD)/taem_exec.o $(LDFLAGS) $(LDLIBS) -o $(BUILD)/shuttle_architecture_contract_tests
	$(BUILD)/shuttle_architecture_contract_tests

qualification: test
	$(CC) $(PROJECT_CPPFLAGS) $(CFLAGS) \
		../Validation/GuidanceIntegrationContractTests.c \
		$(filter-out $(BUILD)/main.o $(BUILD)/guidance.o,$(OBJECTS)) $(CNANO_OBJECTS) $(NANOPB_OBJECTS) $(LDFLAGS) $(LDLIBS) -o $(BUILD)/guidance_integration_contract_tests
	$(BUILD)/guidance_integration_contract_tests
	$(CC) $(PROJECT_CPPFLAGS) $(CFLAGS) \
		../Validation/TAEMProductionIntegrationContractTests.c \
		$(filter-out $(BUILD)/main.o $(BUILD)/guidance.o,$(OBJECTS)) $(CNANO_OBJECTS) $(NANOPB_OBJECTS) $(LDFLAGS) $(LDLIBS) -o $(BUILD)/taem_production_integration_contract_tests
	$(BUILD)/taem_production_integration_contract_tests
	$(CC) $(PROJECT_CPPFLAGS) $(CFLAGS) \
		../Validation/CNanoFakeTransport.c ../Validation/OfflineLandingGateTests.c \
		$(filter-out $(BUILD)/main.o $(BUILD)/guidance.o,$(OBJECTS)) $(CNANO_OBJECTS) $(NANOPB_OBJECTS) $(LDFLAGS) $(LDLIBS) -o $(BUILD)/offline_landing_gate_tests
	$(BUILD)/offline_landing_gate_tests

.PHONY: decision-envelope-test terminal-contract-test
decision-envelope-test: $(TARGET)
	$(CC) $(PROJECT_CPPFLAGS) $(CFLAGS) ../Validation/DecisionEnvelopeTests.c $(filter-out $(BUILD)/main.o,$(OBJECTS)) $(CNANO_OBJECTS) $(NANOPB_OBJECTS) $(LDFLAGS) $(LDLIBS) -o $(BUILD)/decision_envelope_tests
	$(BUILD)/decision_envelope_tests

terminal-contract-test: $(TARGET)
	$(CC) $(PROJECT_CPPFLAGS) $(CFLAGS) ../Validation/TerminalRecoveryContractTests.c $(filter-out $(BUILD)/main.o $(BUILD)/guidance.o,$(OBJECTS)) $(CNANO_OBJECTS) $(NANOPB_OBJECTS) $(LDFLAGS) $(LDLIBS) -o $(BUILD)/terminal_recovery_contract_tests
	$(BUILD)/terminal_recovery_contract_tests

.PHONY: sim-planet-model-test
sim-planet-model-test: $(TARGET)
	$(CC) $(PROJECT_CPPFLAGS) $(CFLAGS) ../Validation/SimPlanetModelTests.c $(filter-out $(BUILD)/main.o,$(OBJECTS)) $(CNANO_OBJECTS) $(NANOPB_OBJECTS) $(LDFLAGS) $(LDLIBS) -o $(BUILD)/sim_planet_model_tests
	$(BUILD)/sim_planet_model_tests

offline-acceptance:
	@lock=.offline-acceptance.lock; \
	if ! mkdir "$$lock" 2>/dev/null; then \
		owner=$$(cat "$$lock/pid" 2>/dev/null || true); \
		if [ -n "$$owner" ] && ! kill -0 "$$owner" 2>/dev/null; then \
			rm -f "$$lock/pid"; rmdir "$$lock" 2>/dev/null || true; \
			mkdir "$$lock" 2>/dev/null || { echo "offline-acceptance busy; retry after the current broad gate" >&2; exit 75; }; \
		else \
			if [ -n "$$owner" ]; then echo "offline-acceptance busy (pid $$owner); retry after the current broad gate" >&2; else echo "offline-acceptance busy; retry after the current broad gate" >&2; fi; \
			exit 75; \
		fi; \
	fi; \
	echo $$$$ > "$$lock/pid"; \
	trap 'rm -f .offline-acceptance.lock/pid; rmdir .offline-acceptance.lock 2>/dev/null || true' 0 1 2 15; \
	$(MAKE) clean all && \
	$(MAKE) qualification && \
	(cd .. && python3 Validation/BackendProtocolTests.py) && \
	(cd .. && python3 Validation/CNanoArchitectureTests.py) && \
	(cd .. && python3 Validation/HeadlessFlightGuardTests.py) && \
	(cd .. && python3 Validation/CampaignProvenanceTests.py) && \
	(cd .. && python3 Validation/PostflightAcceptanceTests.py) && \
	(cd .. && python3 Validation/Postflight75kmAnalyzerTests.py) && \
	(cd .. && KSP_REAL_PHYSICS_DB="$$PWD/Runtime/Physics/observations.sqlite3" CLanding/build/physics_store_tests)

.PHONY: dynamic-interface-test
dynamic-interface-test: $(TARGET)
	$(CC) $(PROJECT_CPPFLAGS) $(CFLAGS) ../Validation/DynamicTaemInterfaceTests.c $(filter-out $(BUILD)/main.o $(BUILD)/guidance.o,$(OBJECTS)) $(CNANO_OBJECTS) $(NANOPB_OBJECTS) $(LDFLAGS) $(LDLIBS) -o $(BUILD)/dynamic_taem_interface_tests
	$(BUILD)/dynamic_taem_interface_tests

.PHONY: mm304-offline-test
mm304-offline-test: $(TARGET)
	$(CC) $(PROJECT_CPPFLAGS) $(CFLAGS) ../Validation/MM304OfflineTests.c $(filter-out $(BUILD)/main.o $(BUILD)/guidance.o,$(OBJECTS)) $(CNANO_OBJECTS) $(NANOPB_OBJECTS) $(LDFLAGS) $(LDLIBS) -o $(BUILD)/mm304_offline_tests
	$(BUILD)/mm304_offline_tests

.PHONY: mm304-recorded-replay
mm304-recorded-replay: mm304-offline-test
	cd .. && python3 Validation/MM304RecordedReplay.py

.PHONY: mm304-predictor-parity-test
mm304-predictor-parity-test: $(TARGET)
	$(CC) $(PROJECT_CPPFLAGS) $(CFLAGS) ../Validation/MM304PredictorParityTests.c $(filter-out $(BUILD)/main.o,$(OBJECTS)) $(CNANO_OBJECTS) $(NANOPB_OBJECTS) $(LDFLAGS) $(LDLIBS) -o $(BUILD)/mm304_predictor_parity_tests
	$(BUILD)/mm304_predictor_parity_tests

.PHONY: mm304-worker-test
mm304-worker-test: $(TARGET)
	$(CC) $(PROJECT_CPPFLAGS) $(CFLAGS) ../Validation/MM304WorkerTests.c $(filter-out $(BUILD)/main.o $(BUILD)/controller.o,$(OBJECTS)) $(CNANO_OBJECTS) $(NANOPB_OBJECTS) $(LDFLAGS) $(LDLIBS) -o $(BUILD)/mm304_worker_tests
	$(BUILD)/mm304_worker_tests

.PHONY: mm304-seam-test
mm304-seam-test: dynamic-interface-test mm304-offline-test mm304-worker-test mm304-predictor-parity-test

.PHONY: hac-execution-selection-test
hac-execution-selection-test: $(TARGET)
	$(CC) $(PROJECT_CPPFLAGS) $(CFLAGS) ../Validation/HACExecutionSelectionTests.c $(filter-out $(BUILD)/main.o $(BUILD)/guidance.o,$(OBJECTS)) $(CNANO_OBJECTS) $(NANOPB_OBJECTS) $(LDFLAGS) $(LDLIBS) -o $(BUILD)/hac_execution_selection_tests
	$(BUILD)/hac_execution_selection_tests

.PHONY: terminal-preview-scheduling-test
terminal-preview-scheduling-test: $(TARGET)
	$(CC) $(PROJECT_CPPFLAGS) $(CFLAGS) ../Validation/TerminalPreviewSchedulingTests.c $(filter-out $(BUILD)/main.o $(BUILD)/guidance.o,$(OBJECTS)) $(CNANO_OBJECTS) $(NANOPB_OBJECTS) $(LDFLAGS) $(LDLIBS) -o $(BUILD)/terminal_preview_scheduling_tests
	$(BUILD)/terminal_preview_scheduling_tests

sim-guidance-runner: $(TARGET)
	$(CC) $(PROJECT_CPPFLAGS) $(CFLAGS) ../ShuttleSim/src/guidance_runner.c \
		$(filter-out $(BUILD)/main.o,$(OBJECTS)) $(CNANO_OBJECTS) $(NANOPB_OBJECTS) \
		$(LDFLAGS) $(LDLIBS) -o $(BUILD)/sim_guidance_runner

.PHONY: sim-telemetry-semantics-test
sim-telemetry-semantics-test: $(BUILD)/sim_telemetry.o $(BUILD)/json.o
	$(CC) $(PROJECT_CPPFLAGS) $(CFLAGS) -DSIM_TELEMETRY_TEST_STANDALONE=1 \
		../Validation/SimTelemetrySemanticsTests.c $(BUILD)/sim_telemetry.o $(BUILD)/json.o \
		$(BUILD)/math.o \
		$(LDFLAGS) $(LDLIBS) -o $(BUILD)/sim_telemetry_semantics_tests
	$(BUILD)/sim_telemetry_semantics_tests

.PHONY: math-model-semantics-test
math-model-semantics-test: $(BUILD)/math.o
	$(CC) $(PROJECT_CPPFLAGS) $(CFLAGS) ../Validation/MathModelSemanticsTests.c \
		$(BUILD)/math.o $(LDFLAGS) $(LDLIBS) -o $(BUILD)/math_model_semantics_tests
	$(BUILD)/math_model_semantics_tests

.PHONY: model-initialization-test
model-initialization-test: $(BUILD)/models.o $(BUILD)/math.o $(BUILD)/json.o
	$(CC) $(PROJECT_CPPFLAGS) $(CFLAGS) ../Validation/ModelInitializationTests.c \
		$(BUILD)/models.o $(BUILD)/math.o $(BUILD)/json.o \
		$(LDFLAGS) $(LDLIBS) -o $(BUILD)/model_initialization_tests
	$(BUILD)/model_initialization_tests
