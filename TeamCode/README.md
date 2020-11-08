# 1. Introduction
This is the TeamCode module for the InfinityTech robotics team software.  This readme documents the InfinityTech team software architecture for the 2020-2021 Ultimate goal season here.

# Installing Android Studio
InfinityTech uses Android Studio for all development.  We don't use the Blocks or OnBots methods you may have seen on the FTC website.  Here are the basic instructions for downloading Android Studio and setting it up:
1.  Download Android Studio.  Unfortunately the current version broke the plugin we need for wireless Wifi development so you'll need to download the older 4.0.1 version until Google fixes the problem.  
The 4.0.1 version for Windows is here: https://redirector.gvt1.com/edgedl/android/studio/install/4.0.1.0/android-studio-ide-193.6626763-windows.exe 
and here for Mac: https://redirector.gvt1.com/edgedl/android/studio/install/4.0.1.0/android-studio-ide-193.6626763-mac.dmg
2.  Install Android Studio on either a Windows PC or a Mac.  The Chrome version can't be installed on your school Chromebooks.
3.  Start Android Studio and select "Configure->Plugins" (at the bottom of the start page).
4.  Search for the "Android Wifi ADB" plugin i and hit "Install" on the right (note that the plugin is already installed on my computer):
  ![Install Android Wifi ADB plugin](docs/install_android_wifiadb_plugin.PNG)

## Downloading the TeamCode from Github
Once you have installed Android Studio, you will need to "Clone" the code from Github as follows:
1.  Goto to https://github.com and sign in.  If you don't already have a Github id, you'll need to create one at the "Sign Up" button on the upper right of the website home page.
2.  Start Android Studio and select "Get From Version Control"  on the start page:
  ![Get From Version Control option](docs/get_from_version_control.PNG)
3.  Enter the team repository URL below into the "Get From Version Control" dialog:  https://github.com/scottarush/InfinityTech_UltimateGoal:  For the "Directory:" enter a location on your computer's hard drive to store the code, such as _C:\Documents\My_UltimateGoal_Code_, but the code can go anywhere and the name of the directory doesn't matter:
![Enter Github URL](docs/entering_github_url_in_android_studio.PNG)
4.  Hit the "Clone" button at the bottom and Android Studio should automatically clone the team repository.  Once the download finishes, Android Studio should build the code without any problems.  If you have issues, we'll need to look at those during a robotics meeting. 

## Control and Expansion Hubs
The rev robotics website has a good overview of both the Control and Expansion hubs here:  https://docs.revrobotics.com/rev-control-system/control-system-overview/control-hub-basics.  

### Developing with the Control Hub
Starting with the 2020-2021 Ultimate Goal season, InfinityTech uses Control hubs and, if we need more outputs, an attached Expansion hub.  We don't use an Expansion hub and a "Robot Controller" mobile phone on the robot.  Developing robot code using the Control hub is much much faster.  The basic process is:
1.  Configure the attached hardware on the Control Hub (motors, servos, etc.) using the _Driver Station_ phone using the instructions here:  https://docs.revrobotics.com/rev-control-system/getting-started/control-hub/configuration.  Note that you only need to update the configuration when something changes (or you have to switch to a new Driver State phone that isn't set up yet.
2.  Start Android Studio and open up your UltimateGoal project on your computer.
3.  Turn on power to the Control Hub on the robot
3.  Connect your computer to the Control Hub's Wifi Hotspot.  We named _InfinityTech13684_.  

# Autonomous Mode
InfinityTech has developed a framework for autonomous mode control using state machines, a Kalman Filter, and a guidance controller that implements "Proportional Integral Control" or _PID_ algorithms.  This section provides a brief overview of each of these elements.

## State Machine Compiler
InfinityTech uses the open source _State Machine Compiler_ or _SMC_ project hosted on SourceForge here:  http://smc.sourceforge.net/, specifically the Java version.  SMC is an automatic code generator that takes in a description of a state machine and generates Java code.  A _state machine_ or, to use the exact computer science term, a _Finite State Machine_ is a very powerful way to program computers to know what actions to take in response to inputs.  Wikipedia has a very good description of state machines here:  https://en.wikipedia.org/wiki/Finite-state_machine

### SMC Input File
SMC uses a text file with a .sm extension and a specific syntax to describe our state machine.  Here's a snippet from a demo running on our Mecanum wheel drivetrain that executes the following sequence:
1.  Rotate the robot 90 degrees
2.  Drive 24 inches forward
3.  Rotate back 90 degrees
4.  Strafe back 24 inches to the original starting position.

~~~
    Start
        Entry {
            rotateToHeading(90);
        }
    {
        evRotationComplete Drive { }
    }

    Drive
        Entry{
            moveStraight(24d);
         }
    {
        evMoveComplete RotateBack {}
    }

    RotateBack
        Entry{
            rotateToHeading(0);
        }
    {
        evRotationComplete StrafeBack {}
    }
    StrafeBack
        Entry{
            strafe(-24d);
        }
    {
        evMoveComplete Complete {}
    }
   /*
    * Final state
    */
    Complete
        Entry{
            stop();
        }
~~~

The website has detailed documentation the syntax of the _.sm_ input file format.  However, basically you have decide what are the "states" of the sequence you want to create, which in the above example are Start, Drive, RotateBack, StrafeBack, and Complete.  You then have to have "events" that trigger transitions between the states.  For example, the _evRotationComplete_ event is triggered by the `GuidanceController` framework once the robot completes a rotation command.  Similarly, the _evMoveComplete_ event is generated by the `GuidanceController` whenever a straight or strafe movement has completed.

### Running the State Machine Compiler Generator
The SMC generator itself is a Java program that has to be run from the command line, but there is a short batch file in the code that allow you to quickly regenerate by right-clicking the gen_autosm.bat file and selecting the "Run Cmd Script" (not shell) in Android Studio.  

#### Note on SMC.jar library
The gen_autosm.bat file  points to a _Smc.jar_ file located in the 'TeamCode/lib' directory.  Note that 'Smc.jar' is only used to re-generate the state machine Java code.  Smc.jar isn't used as a library at runtime.  Only  4 base class files are needed at runtime, and these have been copied into the _Teamcode/statemap_ package.


### Instructions for buidling ejml
The ejml library is necessary for the KalmanFilter operation.  However, the default library from maven won't work as it was compiled for an Android version newer than the FTC control hub supports.

The source has been modified from the libejml-0.39 to get it to compile with the older Android K supported by the control hub.  libejml-0.39 is an Android Studio module that was used to create the libejml.jar file in the 'lib' directory.  You don't have to rebuild the libejml.jar, and the source code for the module is retained in this fork only for archive purposes.   

Make sure to follow these instructions when setting up the repository:
1.  Add the following to the TeamCode/build.gradle file if it s not there:
    dependencies{
        implementation files('lib/libejml.jar')
    }

2.  Make sure the source options in the TeamCode/build.common.gradle file has the following:
    compileOptions {
        sourceCompatibility JavaVersion.VERSION_1_8
        targetCompatibility JavaVersion.VERSION_1_8
    }
    The default is VERSION_1_7 which won't work.