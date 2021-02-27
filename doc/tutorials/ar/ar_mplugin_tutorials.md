# Augmented Reality Plugin tutorials.


### Camera calibration.

For better undestanding of what is camera calibration, please refer to other great tutorials as the one provided by OpenCV library [link](https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_calib3d/py_calibration/py_calibration.html).

To start, you need to print a calibration chessboard pattern. This can be found in many places on internet. For simplicity, you can download this one:

\image html tutorials/ar/chessboard.jpg width=320px

Then run the calibrationApp that is bundled with the AR mico plugin. This app can be ran from command line with the following input arguments.

./calibration_app [cameraIdx] [nPointsBoardVert] [nPointsBoardHoriz] [squareSize    (mm)] [outCalibFile]

A windows will pop-up with the stream of the camera you have chosen. If you place the chessboard pattern in front of the camera, a set of coloured points with lines will appear. It means that the camera has detected the pattern and is waiting for you to save an screenshot. In order to do that, left-click on the window. A new message will appear on the terminal saying that an image has been recorder. 

\image html tutorials/ar/camera_calibration.gif width=320px

We recommend to record at least 12-16 images, placing the calibration pattern at different distances and angles, as shown in the previous gif. Once you have recorded a sufficient number of images, use right-click on the window to stop streaming and start the calibration process. Once it finishes, the application will end and a file will be placed in the location of the folder where the application was ran.


### Printing ARUCO markers

The AR pluging works by detecting specific markers on camera's stream of images. For simplicity, [ARUCO])(https://www.uco.es/investiga/grupos/ava/node/26) markers have been used. These were deceloped by the university of Cordoba in Spain, and popularized in the field of research and robotics thanks to ROS and OpenCV. 

For more information, refere to the developers webpage or OpenCV webpage. In this documentation, we have uploaded two sample aruco markers to be used in this tutorial, but there exists many of them.

<center>
| \image html tutorials/ar/aruco_marker_70.jpg width=320px  | \image html tutorials/ar/aruco_marker_80.jpg width=320px |
| ------------- | ------------- |
| Aruco marker 1 | Aruco marker 2  |
</center>

Print them and proceed to the following section.

### Camera position estimation and main coordinate system

As mentioned in the previous section, the AR system works by detecting ARUCO markers on the camera stream. There are detected using the BlockArucoCoordinates. This block get an input stream of images and output a set of marker detections. These can be filtered using the BlockFilterAruco which get the set of markers and select the desired one by ID. This detections contains two kind of information: The ID of the marker and the 3D pose of it (represented by a 4x4 affine transform matrix).
<center>
| \image html tutorials/ar/BlockArucoDetector.png width=320px  | \image html tutorials/ar/BlockArucoFilter.png width=320px |
| ------------- | ------------- |
| BlockArucoDetector | BlockArucoFilter  |
</center>
At first, create a block for the camera and previsualize the stream in an Image Visualizer. You should have something as in the following image.

\image html tutorials/ar/view_camera_stream.png width=640px

With these two simple blocks, you can have access to the stream of the images and visualize it on real time. 

Then, the next step is to use that stream to detect the ARUCO markers, and use that information to feed our AR system. Create four new blocks:

1. Block Aruco CS. This block takes the stream of images and outputs the detections. If the ID is set to 0, the **all_coordinates** out put is enabled and streams all the detected markers. If the id is set to any other number, the **coordinates** output is enabled, streaming just the data of the marker with that ID. The second argument is the path to the calibration file that we have created at the beginning of this tutorial.
2. Block filter Aruco CS. This block is used to filter the set of all detected markers from the **all_coordinates** stream from the Block Aruco CS.
3. Inverse Transform. This block perform the inverse transformation of a 4x4 affine transformation. The reason for needing this block is explained later in the tutorial.
4. Block AR viewer. This block instantiate the 3D viewer. Is based in OpenGL and takes to important streams. The first one is the position of the camera from a fixed frame and the second one if the image stream of the camera. 

\image html tutorials/ar/view_ar_viewer_main_coordinate.png width=640px

Up to this point, you should be able to move the camera facing the marker and a 3D axes will appear, indicating the fixed coordinate frame. 

### Adding new entities

Now it is time to add more entities to the game. To do that, we will use the other Aruco Marker and two new blocks:

1. Block Mesh. This new block takes as input the path to a mesh file to be display in the AR viewer. This blocks connects underneath with the AR viewer block without any extra connection. The only input is the coordinate of the mesh to be drawn in the viewer. 
2. Multiply Transform. This block takes two 4x4 affine matrices and multiply them.


Another Block Aruco filter is needed to get the new ID and the output stream will be connected to the Block Mesh. The result is a 3D visualization of a cool monkey head in front of your table.

\image html tutorials/ar/test_ar_monkey.gif width=640px
