# Run from the repository root:
# make -C CLanding -f Makefile -f ../Validation/STSRepairTests.mk sts-repair-test
# Separate overlay preserves the existing native-suite ownership and targets.
.PHONY: sts-repair-test sts-entry-frame-test sts-runner-repair-test

sts-entry-frame-test: $(TARGET)
	$(CC) $(PROJECT_CPPFLAGS) $(CFLAGS) \
		$(VALIDATION_ROOT)/EntryVerticalFrameTests.c \
		$(filter-out $(BUILD)/main.o,$(OBJECTS)) $(CNANO_OBJECTS) $(NANOPB_OBJECTS) \
		$(LDFLAGS) $(LDLIBS) -o $(BUILD)/entry_vertical_frame_tests
	cd $(REPO_ROOT) && "$(abspath $(BUILD)/entry_vertical_frame_tests)"

sts-runner-repair-test:
	python3 -m unittest discover -s $(VALIDATION_ROOT) -p 'test_headless_*.py'
	python3 -m unittest discover -s $(VALIDATION_ROOT) -p test_ksp_load_guard.py

sts-repair-test: sts-entry-frame-test sts-runner-repair-test

# Explicit run-9 STS experiment: not the native default/STSN property fixture,
# and not independent certification of the provisional single-flight model.
.PHONY: sts-run9-mm305-test
$(BUILD)/mm305_configured_tests: $(TARGET) $(VALIDATION_ROOT)/MM305FeasibilityPropertyTests.c
	$(CC) $(PROJECT_CPPFLAGS) $(CFLAGS) \
		$(VALIDATION_ROOT)/MM305FeasibilityPropertyTests.c \
		$(filter-out $(BUILD)/main.o,$(OBJECTS)) $(CNANO_OBJECTS) $(NANOPB_OBJECTS) \
		$(LDFLAGS) $(LDLIBS) -o $@

sts-run9-mm305-test: $(BUILD)/mm305_configured_tests
	cd $(REPO_ROOT) && KSP_LANDER_MODEL_PROFILE=sts-provisional \
		"$(abspath $(BUILD)/mm305_configured_tests)" \
		--configuration Configuration/sts.json --mass-kg 25816.072266

.PHONY: sts-model-identity-test
sts-model-identity-test: $(TARGET)
	$(CC) $(PROJECT_CPPFLAGS) $(CFLAGS) \
		$(VALIDATION_ROOT)/ModelIdentityTests.c \
		$(filter-out $(BUILD)/main.o,$(OBJECTS)) $(CNANO_OBJECTS) $(NANOPB_OBJECTS) \
		$(LDFLAGS) $(LDLIBS) -o $(BUILD)/model_identity_tests
	cd $(REPO_ROOT) && "$(abspath $(BUILD)/model_identity_tests)"
	python3 -m unittest discover -s $(VALIDATION_ROOT) -p test_model_identity.py

sts-repair-test: sts-model-identity-test

.PHONY: sts-taem-frame-test
sts-taem-frame-test: $(TARGET)
	$(CC) $(PROJECT_CPPFLAGS) $(CFLAGS) \
		$(VALIDATION_ROOT)/TAEMVerticalFrameTests.c \
		$(filter-out $(BUILD)/main.o,$(OBJECTS)) $(CNANO_OBJECTS) $(NANOPB_OBJECTS) \
		$(LDFLAGS) $(LDLIBS) -o $(BUILD)/taem_vertical_frame_tests
	cd $(REPO_ROOT) && "$(abspath $(BUILD)/taem_vertical_frame_tests)"

sts-repair-test: sts-taem-frame-test

# Read-only, offline recorded-state energy / route audit.
$(BUILD)/mm304_energy_replay: $(TARGET) $(TOOLS_ROOT)/mm304_energy_replay.c
	$(CC) $(PROJECT_CPPFLAGS) $(CFLAGS) $(TOOLS_ROOT)/mm304_energy_replay.c \
		$(filter-out $(BUILD)/main.o,$(OBJECTS)) $(CNANO_OBJECTS) $(NANOPB_OBJECTS) \
		$(LDFLAGS) $(LDLIBS) -o $@
