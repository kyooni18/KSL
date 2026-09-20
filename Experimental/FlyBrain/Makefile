CC ?= cc
CFLAGS ?= -O3 -std=c11 -Wall -Wextra -Wpedantic
LDFLAGS ?=
LDLIBS ?= -lm

BUILD := build
COMMON := $(BUILD)/flybrain.o $(BUILD)/simulator.o

.PHONY: all clean distclean train validate

all: $(BUILD)/flybrain_train $(BUILD)/flybrain_validate

$(BUILD):
	mkdir -p $(BUILD)

$(BUILD)/flybrain.o: flybrain.c flybrain.h | $(BUILD)
	$(CC) $(CFLAGS) -c flybrain.c -o $@

$(BUILD)/simulator.o: simulator.c simulator.h flybrain.h | $(BUILD)
	$(CC) $(CFLAGS) -c simulator.c -o $@

$(BUILD)/flybrain_train: train.c simulator.h flybrain.h $(COMMON)
	$(CC) $(CFLAGS) train.c $(COMMON) $(LDFLAGS) $(LDLIBS) -o $@

$(BUILD)/flybrain_validate: validate.c simulator.h flybrain.h $(COMMON)
	$(CC) $(CFLAGS) validate.c $(COMMON) $(LDFLAGS) $(LDLIBS) -o $@

train: $(BUILD)/flybrain_train
	./$(BUILD)/flybrain_train trained_params.txt

validate: $(BUILD)/flybrain_validate
	./$(BUILD)/flybrain_validate trained_params.txt

clean:
	rm -rf $(BUILD)

distclean: clean
	rm -f trained_params.txt
