CFLAGS := -MMD -g
CXXFLAGS := $(CFLAGS) -std=c++23
LDFLAGS := -lm

CFLAGS += -std=gnu99

INCLUDE_DIRS := -Iinclude

CFLAGS += $(INCLUDE_DIRS)
CXXFLAGS += $(INCLUDE_DIRS)

CSRC := $(shell find . -type f -name '*.c')
CXXSRC := $(shell find . -type f -name '*.cpp')
OBJ := $(CXXSRC:.cpp=.o) $(CSRC:.c=.o)
DEP := $(OBJ:.o=.d)

TARGET := asm_image

all: $(TARGET)

$(TARGET): gperf.out $(OBJ)
	@echo "  LD     $@"
	@$(CXX) -o $@ $(OBJ) $(LDFLAGS)

gperf.out: asm_keywords.gperf
	@echo "  GEN    $@"
	@gperf --output-file=$@ --language=ANSI-C -t --lookup-function-name=asm_keyword_lookup --ignore-case $<


%.o: %.cpp
	@echo "  CXX    $@"
	@$(CXX) $(CXXFLAGS) -c -o $@ $<

%.o: %.c
	@echo "  CC     $@"
	@$(CC) $(CFLAGS) -c -o $@ $<

.PHONY: clean
clean:
	rm -f $(TARGET) $(OBJ) $(DEP) gperf.out

.PHONY: run
run: $(TARGET)
	./$(TARGET) -c test.s

-include $(DEP)
$(OBJ): Makefile

print-%:
	@echo $* = $($*)
