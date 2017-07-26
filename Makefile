#
# Execute 'make' to create prucode.bin and cam_app
# Other options:
# make clean
# make all
# make pru
# make project
# make prucode
# make clean
#

pru = prucode_cam
project = cam_app
dtfrag = BB-BONE-OVCAM-00A0

LIB_PATH = .
LIBRARIES = pthread prussdrv m cairo
INCLUDES = -I. ${LIB_PATH}

SOURCES =  BBBCAM.c camimg.c
OBJECTS = $(SOURCES:%.c=%.o)

EXTRA_DEFINE =
CCCFLAGS = $(EXTRA_DEFINE)
CC = gcc
CFLAGS = $(EXTRA_DEFINE)
PASM = pasm
DTC = dtc

all : $(pru) $(project) $(dtfrag)
pru : $(pru)
project: $(SOURCES)
dtfrag: $(dtfrag)

$(OBJECTS):$(@:.o=.c)
	$(CC) $(INCLUDES) $(CFLAGS) -c -o $@ $(@:.o=.c)

$(project) : $(OBJECTS)
	$(CC) $(OBJECTS) $(LIB_PATH:%=-L%) $(LIBRARIES:%=-l%) -o cam_app


clean :
	rm -rf *.o *.bin $(project) core *~

$(pru) : $(pru:%=%.p)
	$(PASM) -b $@.p

$(dtfrag): $(dtfrag).dts
	$(DTC) -@ -O dtb -o $@.dtbo $@.dts

.SUFFIXES: .c.d

%.d: %.c
	$(SHELL) -ec "$(CC) -M $(CPPFLAGS) $< | sed 's/$*\\.o[ :]*/$@ &/g' > $@" -include $(SOURCES:.c=.d)

