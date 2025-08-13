CC=gcc
CFLAGS=-O2 -Wall -I./h_files
SRC=$(wildcard c_files/*.c) app.c
OUT=ecu_app

all: $(OUT)

$(OUT): $(SRC)
	$(CC) $(CFLAGS) -o $(OUT) $(SRC)

clean:
	-del /q $(OUT) 2>nul || true
	-rm -f $(OUT) || true
