CFLAGS := -MMD -std=gnu99 -O2 -g
LDFLAGS := -lm

INCLUDE_DIRS := -Iinclude

CFLAGS += $(INCLUDE_DIRS)

CSRC := $(shell find . -type f -name '*.c')
OBJ := $(CSRC:.c=.o)
DEP := $(OBJ:.o=.d)

TARGET := em.bin

all: $(TARGET)

../asm/out.bin:
	$(MAKE) -C ../asm run

$(TARGET): $(OBJ)
	@echo "  LD     $@"
	@$(CC) -o $@ $^ $(LDFLAGS)

%.o: %.c
	@echo "  CC     $@"
	@$(CC) $(CFLAGS) -c -o $@ $<

.PHONY: clean
clean:
	rm -f $(TARGET) $(OBJ) $(DEP)

.PHONY: run
run: $(TARGET) ../asm/out.bin
	./$(TARGET)

-include $(DEP)
$(OBJ): Makefile

print-%:
	@echo $* = $($*)

