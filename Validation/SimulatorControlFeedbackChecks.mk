# Use with CLanding/Makefile; keep local build/customization changes separate.
.PHONY: simulator-control-feedback-test
simulator-control-feedback-test: $(TARGET)
	$(CC) $(PROJECT_CPPFLAGS) $(CFLAGS) \
		$(VALIDATION_ROOT)/SimulatorControlFeedbackTests.c \
		../ShuttleSim/src/sim.c ../ShuttleSim/src/scenario.c \
		$(filter-out $(BUILD)/main.o,$(OBJECTS)) $(CNANO_OBJECTS) $(NANOPB_OBJECTS) \
		$(LDFLAGS) $(LDLIBS) -o $(BUILD)/simulator_control_feedback_tests
	cd $(REPO_ROOT) && "$(abspath $(BUILD)/simulator_control_feedback_tests)"
