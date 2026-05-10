CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra -pedantic
LDFLAGS ?= -lm
TARGET ?= robot_spatial
SRC = robot.c

.PHONY: all clean run

all: $(TARGET)

$(TARGET): $(SRC) robot.h
	$(CC) $(CFLAGS) $(SRC) -o $(TARGET) $(LDFLAGS)

run: $(TARGET)
	./$(TARGET)

clean:
	rm -f $(TARGET) $(TARGET).exe *.o
