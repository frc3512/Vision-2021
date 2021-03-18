CXX=arm-raspbian10-linux-gnueabihf-g++
DEPS_CFLAGS=-Iinclude -Iinclude/opencv -Iinclude
DEPS_LIBS=-Llib -lwpilibc -lwpiHal -lcameraserver -lntcore -lcscore -lopencv_aruco -lopencv_dnn -lopencv_highgui -lopencv_ml -lopencv_objdetect -lopencv_shape -lopencv_stitching -lopencv_superres -lopencv_videostab -lopencv_calib3d -lopencv_videoio -lopencv_imgcodecs -lopencv_features2d -lopencv_video -lopencv_photo -lopencv_imgproc -lopencv_flann -lopencv_core -lwpiutil -latomic
EXE=Vision
DESTDIR?=/home/pi/
SRCDIR := src
OBJDIR := build

.PHONY: clean build install

build: ${EXE}

install: build
	cp ${EXE} runCamera ${DESTDIR}

clean:
	-$(RM) -r $(OBJDIR)/$(SRCDIR)
	-$(RM) $(EXE)

# Make does not offer a recursive wildcard function, so here's one:
rwildcard=$(wildcard $1$2) $(foreach dir,$(wildcard $1*),$(call rwildcard,$(dir)/,$2))

# Recursively find all C++ source files
SRC_CPP := $(call rwildcard,$(SRCDIR)/,*.cpp)

# Create raw list of object files
CPP_OBJ := $(SRC_CPP:.cpp=.o)

# Create list of object files for build
CPP_OBJ := $(addprefix $(OBJDIR)/,$(CPP_OBJ))

${EXE}: ${CPP_OBJ}
	${CXX} -pthread -g -o $@ $^ ${DEPS_LIBS} -Wl,--unresolved-symbols=ignore-in-shared-libs

$(OBJDIR)/%.o: %.cpp
	@mkdir -p $(@D)
	@echo Building CPP object $@
	${CXX} -pthread -g -Og -c -o $@ -std=c++17 ${CXXFLAGS} ${DEPS_CFLAGS} $<
