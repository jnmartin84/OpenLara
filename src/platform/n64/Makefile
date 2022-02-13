# usual mips toolchain/libdragon makefile defines
# where things live, tool names
ROOTDIR = $(N64_INST)
GCCN64PREFIX = $(ROOTDIR)/bin/mips64-elf-
CHKSUM64PATH = $(ROOTDIR)/bin/chksum64
MKDFSPATH = $(ROOTDIR)/bin/mkdfs
MKSPRITEPATH = $(ROOTDIR)/bin/mksprite
HEADERPATH = $(ROOTDIR)/mips64-elf/lib
N64TOOL = $(ROOTDIR)/bin/n64tool
HEADERNAME = header
CC = $(GCCN64PREFIX)gcc
CXX = $(GCCN64PREFIX)g++
AS = $(GCCN64PREFIX)as
LD = $(GCCN64PREFIX)ld
OBJCOPY = $(GCCN64PREFIX)objcopy
OBJDUMP = $(GCCN64PREFIX)objdump
LINK_FLAGS = -L/home/Jason/n64dev/libhfx/build -L$(ROOTDIR)/mips64-elf/lib -L$(ROOTDIR)/lib -lhfx -ldragon -lc -lm -lstdc++ -ldragonsys -lhfx -lc -lm -Tn64.ld
PROG_NAME = openlara

SRCS = main.cpp
OBJS = OpenLara.o

CXXFLAGS := -mno-branch-likely -mno-llsc -mabi=o64 -I/home/Jason/n64dev/libhfx/include -I/home/Jason/n64dev/libhfx/libhfx_src/include -fno-exceptions -fno-rtti -mfix4300 -g $(DEFINES) -std=c++11 -fno-builtin -march=vr4300 -mtune=vr4300 -O2 -Wall -G4 -Wl,--defsym -Wl,__cxa_pure_virtual=0 -Wl,--defsym -Wl,__cxa_deleted_virtual=0 -Wundef -Wwrite-strings -Wnon-virtual-dtor -Wno-multichar -I../.. -I$(ROOTDIR)/mips64-elf/include -fno-use-cxa-atexit
OBJS = osfp.o n64_memcpy.o n64_memset.o $(SRCS:.cpp=.o)
DEPS = $(SRCS:.cpp=.d)

$(PROG_NAME).z64: $(PROG_NAME).elf rom.dfs
	$(OBJCOPY) $(PROG_NAME).elf $(PROG_NAME).bin -O binary
	rm -f $(PROG_NAME).z64
	$(N64TOOL) -l 67104768 \
		-h $(HEADERPATH)/$(HEADERNAME) \
		-o $(PROG_NAME).z64 -t $(PROG_NAME) $(PROG_NAME).bin \
		-s 1M rom.dfs
	chmod ugo=rw $(PROG_NAME).z64
	$(CHKSUM64PATH) $(PROG_NAME).z64

$(PROG_NAME).elf: $(OBJS)
	$(LD) -o $(PROG_NAME).elf $(OBJS) $(LINK_FLAGS)
	$(OBJDUMP) -t $(PROG_NAME).elf > $(PROG_NAME)_symbols.txt
	cat $(PROG_NAME)_symbols.txt | grep 'F .text' > $(PROG_NAME)_functions.txt

%.o: %.cpp %.c
	$(CXX) $(CXXFLAGS) -MMD -c $< -o $*.o

rom.dfs:
	$(MKDFSPATH) rom.dfs ./filesystem/

clean:
	rm -f *.o *.d *.dfs

-include $(DEPS)
