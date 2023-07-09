# Image path extractor (PathPilot UI)
<strong style="color:orange">NOTE: This project has some limiting bugs (listed at the end), be patient while using it. :)</strong>

The goal for this project is to have a UI that runs within the PathPilot software and enables you to extract paths from images and get the robot arm to draw them.

To get this project to run, all you have to do is load the custom_ui.py script and click "CYCLE START".

<strong style="color:orange">NOTE: Remember to set up a user frame and name it "drawing_user_frame"</strong>

![UI overview](/image_path_extractor_pathpilot/assets/IDP-overview.jpg)


## Issues that you might encounter:
- Each time you run the project for the first time in the PathPilot software it will install some python packages. 
- The "RESET" or "STOP" button might not work while running the program, you will either wait till the robot arm is done drawing or just use the e-stop button to stop the robot arm.
- If you are new to the Tormach robot arm you will have to learn how to set up user frames.
- If the image you are drawing is pixelated that pixelation will be baked in the extracted paths.

## Future improvements (TODO):
- Fix package installation (To make sure it doesn't install packages every time it's first ran)
- Fix pause-start issue
- Add a stop button that doesn't cancel the whole program but just the drawing process.
- Adding the ability to draw paths directly in the preview scene.
- Adding the ability to edit the extracted paths
- Make the file explorer pretty
- Adding a multiple edge detection algorithm selection (Currently, only using the canny edge detection algorithm implemented by OpenCV)

