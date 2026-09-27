CC ?= gcc
CFLAGS := -std=c11 -O2 -Wall -Wextra -Werror -pedantic -Istubs -I../stm32/Core/Inc
SRC := test_main.c \
 ../stm32/Core/Src/mpu6050.c \
 ../stm32/Core/Src/quaternion.c \
 ../stm32/Core/Src/imu_calibration.c \
 ../stm32/Core/Src/attitude_eskf.c \
 ../stm32/Core/Src/sensor_fusion.c \
 ../stm32/Core/Src/attitude_controller.c \
 ../stm32/Core/Src/control_allocation.c \
 ../stm32/Core/Src/flight_supervisor.c \
 ../stm32/Core/Src/flight_app.c

test_runner: $(SRC)
	$(CC) $(CFLAGS) $(SRC) -lm -o $@

test: test_runner
	./test_runner

clean:
	rm -f test_runner

.PHONY: test clean
