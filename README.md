# direction-estimation-of-moving-camera

Method-1:
FILE: object_movement.py
First of all I did thresholding in such a way that, the sky and the rest things in frame can be separated. Then I enclosed the lower part of the frame in a circle and calculated it’s centre and in every frame as the centre position changes I put some values like if the movement in x direction is greater than a particular value then it’s some direction else(if negavtive) the opposite one. Like wise same concept was used for y direction. This method does not give precise results because the contour of lower half of the frame keeps on changing. But I gives results but not that good.
Before this segmentation I also tried to separate the scooter’s handle by giving some hsv color range but that didnt work.
https://www.youtube.com/watch?v=0VWeiT86Gac

Method-2:
FILE: vehicle_flow_detection_v01.py
The concept of this method was new for me “optical flow” finding direction vector of pixel movement, when the previous frame is compared with the current frame. In this I showed the direction vectors of the pixel and it gives good results just visually I tried some way to in the next method to get some estimate of direction.
https://www.youtube.com/watch?v=PXxo2MoGWLQ

Method-3:
FILE: vehicle_direction_detection.py
In this method I was finding the mean angle bewteen the vectors if the distance between them is greater than the threshold value. So, on the basis of mean angle I was giving the direction if it’s between 0-90 then right and like that. It is also not that much accurate the problem I think is it taking some false vectors which should not be taken.
https://www.youtube.com/watch?v=gJRLBbBOVZw
