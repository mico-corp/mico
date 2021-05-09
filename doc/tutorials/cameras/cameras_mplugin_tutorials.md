Cameras Plugin tutorials {#tutorial_cameras_mplugin}
=====================

The cameras plugin has currently two possible streamers of images, 

<center>
| \image html tutorials/cameras/block_single_image_flusher.png width=320px  | \image html tutorials/cameras/block_streamer_webcam.png width=320px |
| ------------- | ------------- |
|  Single image flusher | Webcam streamer  |
</center>

The single image flusher has been created to allow the user to send a single image in a controller manner. Each time the flush button is pressed, the block sends an image to its output. The Streamer Webcam has been created to allow the user to open usb cameras and extract an stream of images from it. This block can be configured at a fixed frame rate (or frecuency of images) using the spin box shown in the image. The device ID correspond to an internal index of cameras connected to the computer, 0 means the first camera, 1 the second and so on.


Following gif shows an example of usage.

\image html tutorials/cameras/basic_streamrs.gif width=640px
