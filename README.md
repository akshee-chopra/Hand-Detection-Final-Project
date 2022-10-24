# Hand-Detection-Final-Project

This was a final project I made in BWSI (MIT BeaverWorks Summer Institute). The point of the project is that the drone's webcam can detect a hand, and if that hand is making a fist, then it should follow the movement of the hand autonomously. I implemented the autonomous movement using a PID controller, which is essentially trying to minimize the error of the roll-movement (a specific axis of a drone). The error is the distance between the targetPoint (the center of the fist) and the current point (the center of the drone's webcam). 
