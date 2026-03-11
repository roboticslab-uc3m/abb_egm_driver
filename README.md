# abb_egm_driver
ROS 2 drivers for ABB robots featuring EGM.

## How-To: WSL + EGM
In order to communicate WSL with RobotStudio (or the real robot), follow these instructions ([SO](https://stackoverflow.com/a/68872599)):

1. Launch PowerShell and note down the IP address returned by `$(wsl hostname -I)`.
1. In your RobotStudio project, provided that EGM support has been already enabled, look for *Communication > UDP Unicast Device* in the controller configuration, and fill in the *Remote Address* field of the *UCdevice* entry with the previous IP address. For instance:
   ![WSL configuration](fig/wsl-config.png)
1. Launch PowerShell with elevated rights and issue the following command:
   ```
   netsh interface portproxy set v4tov4 listenport=6599 listenaddress=0.0.0.0 connectport=6510 connectaddress=$(wsl hostname -I)
   ```
