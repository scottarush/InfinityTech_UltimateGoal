##Introduction
This is the TeamCode module for the InfinityTech robotics team software.  This module was originally created for the 2019-2020 Skystone season and has been updated for teh 2020-2021 Ultimate goal season here.

This README page contains special instructions needed to build InfinityTechs code.

##State Machine Compiler
For autonomous mode, InfinityTech uses the open source State Machine Compiler project hosted on SourceForge here:  http://smc.sourceforge.net/.  The generator is run from a command line using the gen_autosm.bat file which points to a 'Smc.jar' file located in the 'TeamCode/lib' directory.  Note that 'Smc.jar' is only used to update the state machine.  There are only 3 base class files needed at runtime and these have been copied into the 'statemap' package due to extraneous build issues when directly referencing the main 'Smc.jar' class.  

##Instructions for buidling ejml
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