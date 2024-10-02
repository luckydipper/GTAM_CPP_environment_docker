# What this repository is? 
GTSAM 4.2 docker environment.  
C++11, Cmake and build essential are installed.  

# How to turn on?
1. install docker  
2. `sudo ./buildDocker.bash`  
3. `sudo ./execute.bash`  

## If you use MacOS.
`ERROR: failed to solve: ubuntu:20.04: error getting credentials - err: exit status 1, out: `  
edit $HOME/.docker/config.json file  
"credsStore": "desktop" -> "credsStore": "osxkeychain"  
ref : https://forums.docker.com/t/error-failed-to-solve-error-getting-credentials-err-exit-status-1-out/136124/4

## Docker hub
https://hub.docker.com/repository/docker/luckydipper/gtsam_tutorial/general
