CC=gcc
CFLAGS=
OBJ=hdlc_test.o hdlc.o

hdlc_test: $(OBJ)
	$(CC) -o $@ $^ $(CFLAGS)

.PHONY: clean

clean:
	rm -f *.o hdlc_test
