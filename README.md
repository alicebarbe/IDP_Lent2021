# IDP_Lent2021

![CAD of robot model](https://github.com/alicebarbe/IDP_Lent2021/blob/figures/cad.png "CAD Model")

# Overview of Software

![Software Overview](https://github.com/alicebarbe/IDP_Lent2021/blob/figures/IDP%20Software%20Approach1024_1.png "Software Approach")

![Movement Block Diagram](https://github.com/alicebarbe/IDP_Lent2021/blob/figures/Movement%20Block%20Diagram.png "Movement Block Diagram")

# Robot/Server Communication Protocol
DLC = 20

All messages of form ```tuple<int,double,double>```. All sent messages are therefore 20 bytes long. This has a typedef known as “message”.

Message Format: ```<Identifier , X coordinate , Z Coordinate> ```

Every message sent by the robots should be preceded by a message updating the server to its location.

### Identifiers: 

**First Digit:**

- 1: Green Robot
- 2: Red Robot

**Second/Third Digit:**

Robot to Server

- 10: Hello I am on message
- 20: Coordinates are robot’s GPS location
- 25: Coordinates are the robot’s destination location
- 29: The other robot is about to collide with me
- 30: Block at current target location is Green
- 40: Block at current target location is red
- 50: There is a block at the location I am sending you
- 60: I have finished scanning, where do i go
- 70: I have dealt with my current block, where do i go
- 71-79: Reserved for testing/debugging signals

Server to Robot

- 80: Scan for blocks
- 90: go to location I am sending you
- 00: go to location I am sending you where you will find a block
- 97: Start again after collision
- 98: Stop until further notice
- 99: Emergency

### Example messages:

```200,   0.8,   1.1```

The server is telling the red robot to go to x = 0.8, z = 1.1 where it will find a block.

```120, 0.0, 0.96```

The green robot is telling the server that it is at x = 0.0, z = 0.96.
