NOTE: This code was built off of the files that came with the Elegoo Smart Robot Car V4.0 With Camera, and sample code provided by Edge Impulse.

This is the main repository for our ECE 4333 project. We should keep all code in this repository, and keep it up-to-date to make things easy. I have created the first commit.

To Do (High Priority):
* Gate the AI processing behind a sufficient colour blob
* ~~Communicate object size and stop sign presence to the Elegoo board. You should not have to change the existing capture_rgb888 function other than adding the send function in the appropriate place. Also finish the rest of A2.~~
* Make Presentation
* Write Assignment


To Do (Low Priority)
* AI rec. only works when stream is off
* Determine if we should make a separate freeRTOS task for the AI stuff
* Add the option to switch between resolutions that it captures (curently fixed at QQVGA, breaks if it changes)
* Find a way to turn the 96x96 frame's bounding boxes into something we can put on QVGA images, ect
* ~~Add distance detection (could be an OR of the blob size and stop sign in same location, then use the blog size to determine distance)~~
* Calibrate colour detection better (just have to change RGB values, colour_detect() works perfectly)

## Resources
[Beginner guide] (https://git-scm.com/docs/gittutorial)

[Commiting and pushing changes] (https://www.geeksforgeeks.org/git/difference-between-git-commit-and-git-push/)

[Useful commands] (https://git-scm.com/cheat-sheet)
