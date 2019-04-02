# FYP Code

This program enables a biped robot, the legs of which are basede on teh hind legs of the mammals of the felidae family, to hop in place.

Robot assembly is mechanically constrained to allow vertical translation and roll motion of the torso only.

Assembly consists of a torso and two legs each having:

1. Thigh link
2. Shin link
3. Foot link
4. Toe

4 motors, 2 for each leg, are placed on the torso and control the motion of the hip (torso-thigh) joint and knee (thigh-shin) joint. 
Ankle (shin-foot) joint is underactuated and spring-loaed.

Electronics include:

~ Arduino Mega 2560
~ Pololu brushed DC motors
~ LM298 h-bridges
~ Ultrasonic sensor (SR-04)
~ Hall-effect quadrature encoders (1120 counts per rev.)
~ Push buttons

#Program

Control strategy is essentially built upon Raibert's 3D hopper control algorithm, albeit the mechanical assembly is completely different having
articulate linkages rather than a prismatic leg.




