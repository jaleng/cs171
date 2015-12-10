###############################################################################
# CS/CNS 171 Fall 2015
#
# This is a template Makefile for OpenGL programs. Edit it however you find
# convenient.
# 
# The current version of this file should compile OpenGL programs just fine on
# Debian-based Linux operating systems.
#
# If you run Mac OS or other distributions of Linux, then you may have to
# fool around with the 'INCLUDE' and 'LIBDIR' lines below before the Makefile
# can compile the OpenGL parts successfully.
###############################################################################
CC = g++
FLAGS = -std=c++14 -g -o

INCLUDE = -I/usr/X11R6/include -I/usr/include/GL -I/usr/include -I../../
LIBDIR = -L/usr/X11R6/lib -L/usr/local/lib
SOURCES = *.cpp
LIBS = -lGLEW -lGL -lGLU -lglut -lm

EXENAME = keyframe

all: keyframe keyframe_smooth

keyframe: keyframe.cpp *.h
	$(CC) $(FLAGS) keyframe $(INCLUDE) $(LIBDIR) keyframe.cpp $(LIBS)

keyframe_smooth: keyframe_smooth.cpp *.h
	$(CC) $(FLAGS) keyframe_smooth $(INCLUDE) $(LIBDIR) keyframe_smooth.cpp $(LIBS)

clean:
	rm -f *.o $(EXENAME)

.PHONY: all clean
