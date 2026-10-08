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
