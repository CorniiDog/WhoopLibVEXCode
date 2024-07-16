# VEXcode makefile 2019_03_26_01

# Clean all to ensure it functions correctly
.PHONY: clean all
all: clean build

# show compiler output
VERBOSE = 0

# include toolchain options
include vex/mkenv.mk

# location of the project source cpp and c files
SRC_C  = $(wildcard src/*.cpp) 
SRC_C += $(wildcard src/*/*.cpp) 
SRC_C += $(wildcard src/*/*/*.cpp) 
SRC_C += $(wildcard src/*/*/*/*.cpp) 
SRC_C += $(wildcard src/*.c)


OBJ = $(addprefix $(BUILD)/, $(addsuffix .o, $(basename $(SRC_C))) )

# location of include files that c and cpp files depend on
SRC_H  = $(wildcard include/*.h)
SRC_H  += $(wildcard include/*/*.h)
SRC_H  += $(wildcard include/*/*/*.h)
SRC_H  += $(wildcard include/*/*/*/*.h)
SRC_H  += $(wildcard include/*.hpp)
SRC_H  += $(wildcard include/*/*.hpp)
SRC_H  += $(wildcard include/*/*/*.hpp)

# additional dependancies
SRC_A  = makefile

# project header file locations
INC_F  = include

# set compiler flags after including the environment to ensure they are not overridden
CXX_FLAGS += -fexceptions
CXX_FLAGS += -std=gnu++17 #-std=c++17
CXX_FLAGS += -Wc++17-extensions

# build targets
#all: $(BUILD)/$(PROJECT).bin # removed and replaced with build
build: $(BUILD)/$(PROJECT).bin

# include build rules
include vex/mkrules.mk
