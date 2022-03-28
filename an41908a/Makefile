MPP_DIR=$(HOME)/projects/cameras/sdk/Hi3516CV500_SDK_V2.0.2.1/smp/a7_linux/mpp

all: an41908a-orig

an41908a-orig: CC=arm-himix200-linux-gcc
an41908a-orig: CFLAGS=-I$(MPP_DIR)/include
an41908a-orig: LDFLAGS=-L$(MPP_DIR)/lib
an41908a-orig: LDLIBS=-lisp -l_hildci -l_hidehaze -l_hidrc -l_hiacs -l_hiir_auto -l_hicalcflicker -lmpi -pthread -lsecurec -lVoiceEngine -lupvqe -ldnvqe -lm

an41908a-%: an41908a.o
	$(CC) -o $@ $(LDFLAGS) $(LDLIBS) $^
	sudo cp $@ /mnt/noc/sdk/majestic.cv500

clean:
	-rm -rf an41908a-* *.o
