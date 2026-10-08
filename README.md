NOTE: This code was built off of the files that came with the Elegoo Smart Robot Car V4.0 With Camera, and sample code provided by Edge Impulse.

This is the main repository for our ECE 4333 project. We should keep all code in this repository, and keep it up-to-date to make things easy. I have created the first commit.

To Do (High Priority):
* ~~PID Controller Inner Loop / Gyroscope (must send mapping data?) / Camera (Adam)~~
* Kinematics Prediction (Corey)
* State Machine (Corey)
* Traffic Light and stop sign management (Corey)
* Add E-stop button, data logging/visualizing mechanism in the web client, or see if you can store the data somewhere on the robot (non-volatile) (Greg)
* Make it so camera is no longer flipped
* ~~Gate the AI processing behind a sufficiently sized colour blob being detected (Adam)~~
* ~~Communicate object size and stop sign presence to the Elegoo board. You should not have to change the existing capture_rgb888 function other than adding the send function in the appropriate place. Also finish the rest of A2.~~


To Do (Low Priority)
* ~~Convert RGB to HSV and look at hue to get a more accurate colour (Adam)~~ 
* Look into python script for processing (Adam/Greg)
* AI rec. only works when stream is off, implement a function that can help do both
* Calibrate colour detection better (just have to change RGB values, colour_detect() works perfectly)
* ~~Determine if we should make a separate freeRTOS task for the AI stuff~~
* ~~Add the option to switch between resolutions that it captures (curently fixed at QQVGA, breaks if it changes)~~ unnecessary
* ~~Find a way to turn the 96x96 frame's bounding boxes into something we can put on QVGA images, ect~~ unnecessary
* ~~Add distance detection (could be an OR of the blob size and stop sign in same location, then use the blog size to determine distance)~~


## Resources
[Beginner guide] (https://git-scm.com/docs/gittutorial)

[Commiting and pushing changes] (https://www.geeksforgeeks.org/git/difference-between-git-commit-and-git-push/)

[Useful commands] (https://git-scm.com/cheat-sheet)
