CC = g++
CFLAGS = -Wall -O2 -std=c++17 -pthread -Wno-format-truncation
TARGET = sdg2122_fe

MIDASSYS = /home/muon/Installs/midas
CFLAGS += -I$(MIDASSYS)/include -I$(MIDASSYS)/mxml -I$(MIDASSYS)/mjson \
          -D_DEFAULT_SOURCE -D_XOPEN_SOURCE=600 $(shell pkg-config --cflags libusb-1.0)
LIBS = $(shell pkg-config --libs libusb-1.0)

SRCS = sdg2122_fe.cxx
OBJS = $(SRCS:.cxx=.o)

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) -o $(TARGET) $(OBJS) \
	-L$(MIDASSYS)/lib -lmidas $(MIDASSYS)/lib/libmidas.a \
	$(LIBS) -lrt -pthread -lm -ldl -rdynamic $(MIDASSYS)/lib/mfe.o

%.o: %.cxx
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f $(OBJS) $(TARGET)

.PHONY: all clean