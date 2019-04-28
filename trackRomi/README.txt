In order to run the code:

python TrackRomi.py --map mapNoRomi.jpg --image romiInMap.png --video romiMap2.mp4


--map specify the address of a picture of the map without the romi
--romiInMap specify any picture of the map where the romi is present to calibrate the image
--video specify the actual video to analyse or could be changed to webcam

In the terminal, all instructions are specified, however:

During cropping:
r = reset the region of interest
c = save region of interest

During calibration:
s = save the calibration

Workflow:
1. load images, A picture of the map with no Romi will pop to make the base gridlines. Press "space" bar to continue
2. A picture of the map with Romi will pop to select region of interest to calibrate. Press "space bar" to continue after saving region of interest
3. A picture of the map for the actual video will pop to select region of interest for the video
