
PROJECT = test
OBJECTS = test.o Fixed.o FixedMatrix.o
CC = gcc
CPP = g++
LD = g++
LDFLAGS=-lfixmath
LIBS=-L./libfixmath

$(PROJECT) : $(OBJECTS)
	(cd libfixmath && make)
	$(LD) $(LDFLAGS) $(LIBS) -o $@ $(OBJECTS)

.c.o :
	$(CPP) -c $@ $<

clean :
	(cd libfixmath && make clean)
	rm -rf $(PROJECT) $(OBJECTS)
