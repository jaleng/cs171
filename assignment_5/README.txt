// To test
// ./smooth [scene_description_file.txt] [xres] [yres] [h, smoothing_factor]
./smooth scene_bunny.txt 800 800 0.0128

// To smooth while program is running,
// Press 's' on keyboard
// This will smooth by the given smoothing factor each time 's' is pushed.

// BUG
// For some smoothing factors, the image appears to smooth correctly,
// for others, we see the mesh grow spikes (incorrect behavior).
