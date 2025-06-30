# 2025-CRA-Commons
Common code base for CRA robots created after the 2025 season based on 2025 wpilib. With this common 
code base and instructions on robot hardware you should be able to create a swerve drive robot that\
can follow autonomous routines developed with path planner and that has localization determined by 
vision and odometry.

## Step by step for implementation on your robot

1. Clone and fork this repository.
2. Open this new cloned folder in VS Code 2025.
3. Build the code.  If you are in CPS you will probably not be able to access any of the referenced libraries in github.io so you might have to build at home. 
  If you get errors trying to build the code that is probably because CPS is blocking github.io (look at errors and see if that is the case).
4. Edit .wpilib\wpilib_preferences.json, specifically the year and your team number.
5. Open this link as a reference, yagsl: https://docs.yagsl.com/
7. Look in src\main\deploy\swerve and you will see two sample swerve folders: ava and ralph.  Copy or rename one of these to the name of your robot and you will start editing those values.
8. To help with that, open https://broncbotz3481.github.io/YAGSL-Example/
9. Let's start with swervedrive.json.  You need to know the type of IMU you have (navX or pidgeon or something else there's a big dropdown) and the CAN id of the IMU.  Enter those and the generated json will change.  When you
   get what looks right, copy the code into the swervedrive.json you just copied.  The rest of the values are probably unchanged.
10. Next we will work on physicalproperties.json.  You probably only need to find the drive and steer gear ratios. You can find that from your swerve drive manufacturer.  Also, the diameter is the for your wheel
    diameter in inches (by default that is 4, which might be correct already).  The gear ratios are probably the hardest to find.
11. 
