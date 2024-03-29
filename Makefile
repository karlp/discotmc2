
PROJECT	= discotmc2

FREERTOS_DIR ?= /home/karlp/src/FreeRTOSv10.2.1/FreeRTOS
FREERTOS_PORT = $(FREERTOS_DIR)/Source/portable/GCC/ARM_CM3
FREERTOS_INC = $(FREERTOS_DIR)/Source/include
FREERTOS_SRC = $(FREERTOS_DIR)/Source
FREERTOS_MMG = $(FREERTOS_DIR)/Source/portable/MemMang
FREERTOS_SRCS = list.c queue.c tasks.c timers.c port.c heap_1.c
FREERTOS_SRCS += FreeRTOS-openocd.c

VPATH += $(FREERTOS_SRC) $(FREERTOS_PORT) $(FREERTOS_MMG)
# Inclusion of header files
INCLUDES += $(patsubst %,-I%, . $(FREERTOS_INC) $(FREERTOS_PORT))

SHARED_DIR = ../common
VPATH += $(SHARED_DIR)
INCLUDES += $(patsubst %,-I%, . $(SHARED_DIR))

LDFLAGS += -Wl,--undefined=uxTopUsedPriority
LDLIBS += -lm

CFILES = $(PROJECT).c
CFILES += $(FREERTOS_SRCS)
CFILES += funcgen.c funcgen-plat.c funcgen-plat-arch-l1.c
CFILES += trace.c trace_stdio.c
#CFILES += usart_stdio.c

OPENCM3_DIR = libs/libopencm3
DEVICE = stm32l151x8x_a
#DEVICE = stm32l151x6
OOCD_FILE = openocd.stm32l1-generic.cfg
#OOCD_FILE = board/stm32ldiscovery.cfg

include rules.mk
