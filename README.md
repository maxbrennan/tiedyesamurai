# FRC Robot

## Getting Started

### Tools

To get started, install these tools:
- IntelliJ Community Edition    
    - [Windows](https://www.jetbrains.com/idea/download/download-thanks.html?platform=windows&code=IIC) 
    - [MacOS](https://www.jetbrains.com/idea/download/download-thanks.html?platform=macM1&code=IIC)
- [FRC Game Tools from NI (Windows Only)](https://download.ni.com/support/nipkg/products/ni-f/ni-frc-2025-game-tools/25.0/online/ni-frc-2025-game-tools_25.0_online.exe)

## Building and Deploying

Make sure you are connected to the robot's wifi: **FRC-AP-10933-samurai** with the password **samurai2465**
Use the gradle task **deployprogramStartfrcJavaroborio**  

## Driver Station Debugging

If you are having issues communicating and running code on the RoboRio, check the following things:
- The team number should be set to 10933
- You are connected through wifi to the Roborio
- The RoboRio is turned on
- You've built & deployed your program 
  
## Transfering Subscriptions and Effects Into Application

  1) Go to the platform and checkout the branch with all of the effects and subscriptions that you want to use
  2) Run the **jar** task in order to regenerate the jar file located under build\libs
  3) Copy the regenerated jar
  4) Go to the application and checkout the branch you're working in
  5) Paste the jar file under \libs replacing the file currently there with the same name
  6) Right click the new file and select the **Add as Library** option 

