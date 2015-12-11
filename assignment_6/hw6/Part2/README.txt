I copied all of the keyframe and interpolated frame .obj files
into this directory for less code and ease-of-use.
They are unaltered from the originals.

To compile
-----------------
make




./keyframe_ibar I_Bar/test.script 800 800
----------------------------------------------------------
Assignment 6 Part 2.1: The I-Bar Animator
use 'f' to advance a frame.




./keyframe_smooth
----------------------------------------------------------
Assignment 6 Part 2.2: Interpolating a Smoothing Bunny
The program reads in the keyframes(0,5,10,15,20), interpolates them,
then reads in the precalculated interpolated frames and compares.

If the difference between any vertex component from its expected value
is greater than .001, the program outputs
"INCORRECT: interpolated frames do not match"

Otherwise it outputs
"CORRECT: interpolated frames match"
