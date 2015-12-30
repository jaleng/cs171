###############################################################################
# Makefile for modeler
# written by: Parker Won
###############################################################################

CC = g++
FLAGS = -Wall -g -std=c++14
LDFLAGS = -L/usr/X11R6/lib -L/usr/local/lib
LDLIBS = -lGLEW -lGL -lGLU -lglut
INCLUDE = -I../lib -I/usr/include -I/usr/X11R6/include -I/usr/include/GL -I../..
SOURCES = main.cpp model.o commands.o command_line.o Renderer.o Scene.o UI.o Utilities.o Shader.o Assignment.o
EXENAME = modeler

all: $(EXENAME)
	ctags -Re *

$(EXENAME): $(SOURCES)
	$(CC) $(FLAGS) -o $(EXENAME) $(INCLUDE) $(LDFLAGS) $(SOURCES) $(LDLIBS)

%.o: %.cpp %.hpp
	$(CC) $(FLAGS) -o $@ $(LIBDIR) $(INCLUDE) $(LDFLAGS) -c $< $(LDLIBS)

clean:
	rm -rf *.o $(EXENAME)

.PHONY: all clean
