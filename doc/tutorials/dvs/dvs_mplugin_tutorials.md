DVS Plugin tutorials {#tutorial_dvs_mplugin}
=====================


```
    DISCLAIMER! This pluging is only available on ubuntu in the current version.
```

An event camera, also known as a neuromorphic camera, silicon retina or dynamic vision sensor, is an imaging sensor that responds to local changes in brightness. Event cameras do not capture images using a shutter as conventional cameras do. Instead, each pixel inside an event camera operates independently and asynchronously, reporting changes in brightness as they occur, and staying silent otherwise. Modern event cameras have microsecond temporal resolution, 120 dB dynamic range, and less under/overexposure and motion blur than frame cameras. [Text from Wikipedia](https://en.wikipedia.org/wiki/Event_camera).

This module provides for various blocks to use [Inivation](https://inivation.com) DVS cameras and process the event data streams in different manner. Following image shows how to use the camera block to visualize the stream of events.

\image html tutorials/dvs/dvs_visualize.png width=640px

This input stream might be noise in some situations, for that reason, a noise filter block is provided. This block filters an stream of events an output it cleaned. 

\image html tutorials/dvs/dvs_filter.png width=640px

At last, a coner detector algorithm based on event is provided too. Following image shows how to use it and visualize the results.

\image html tutorials/dvs/dvs_corner.png width=640px