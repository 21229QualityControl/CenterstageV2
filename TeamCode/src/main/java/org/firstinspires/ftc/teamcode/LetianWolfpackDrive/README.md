# Wolfpack Inspired Drive Code
This is a mini project programmed by Letian Li, based off of Wolfpack Machina's Movement Breakdown [video](https://www.youtube.com/watch?v=ri06orPFaKo) for the Freight Frenzy season.

This project optimizes for max power movement and smooths movement by using curvature.


# Tuning process

### 1. Collecting max velocities (or movement bias)

Make sure you have enough space in the drive direction as the robot will move in said direction at full power for a given amount of time.

1. Run MaxVelStraightTest to get the robot's max velocity forwards. Record this as maxVelocityX in WolfpackDrive.java.
2. Run MaxVelStrafeTest to get the robot's max velocity strafing. Record this as maxVelocityY (positive) in WolfpackDrive.java.
    - Make sure the robot strafes in the correct direction.
    - Also recommended to strafe and record both left and right.

### 2. Test and tune centripetal correction

1. Run WolfpackDriveTest and adjust the centripetalWeighting constant in WolfpackDrive.java
   - This number should be pretty small

### 3. Have fun
