OBJ = main.o fftmisc.o  fourierf.o
FILE = main.c fftmisc.c  fourierf.c
CFLAGS = -static -O3 -g

fft: ${OBJ} Makefile
	$(TESTCOMPILER) $(TESTFLAG)  ${CFLAGS} ${OBJ} -o fft -lm
fftmisc.o: fftmisc.c
	$(TESTCOMPILER) $(TESTFLAG) ${CFLAGS} -c fftmisc.c
fourierf.o: fourierf.c
	$(TESTCOMPILER) $(TESTFLAG) ${CFLAGS} -c fourierf.c
main.o: main.c
	$(TESTCOMPILER) $(TESTFLAG) ${CFLAGS} -c main.c

clean:
	rm -rf *.o fft output*
