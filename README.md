# isaac_warehouse_project

## Isaac Sim 5.1 Setup

- Download the Isaac Sim 5.1 zip file and perform quick install as given in the [Link](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/quick-install.html)
- Download the full assets zip file, extract and link it to Isaac Sim

*Note 1:* How to link assets folder to Isaac Sim
Assuming the installation folder is ```~/isaac-sim/isaac-sim-standalone-5.1.0-linux-x86_64```
Edit the file in ```apps/isaacsim.exp.base.kit``` 
- Add the text under the ```[settings]``` tag at the end
  ```
    persistent.isaac.asset_root.default = "/media/raviteja/Ubuntu_Data2/isaac-sim-assets/Assets/Isaac/5.1"

    exts."isaacsim.asset.browser".folders = [
        "/media/raviteja/Ubuntu_Data2/isaac-sim-assets/Assets/Isaac/5.1/Isaac/Robots",
        "/media/raviteja/Ubuntu_Data2/isaac-sim-assets/Assets/Isaac/5.1/Isaac/Environments",
        "/media/raviteja/Ubuntu_Data2/isaac-sim-assets/Assets/Isaac/5.1/Isaac/Props",
        "/media/raviteja/Ubuntu_Data2/isaac-sim-assets/Assets/Isaac/5.1/Isaac/Materials",
        "/media/raviteja/Ubuntu_Data2/isaac-sim-assets/Assets/Isaac/5.1/Isaac/Sensors",
        "/media/raviteja/Ubuntu_Data2/isaac-sim-assets/Assets/Isaac/5.1/Isaac/People",
        "/media/raviteja/Ubuntu_Data2/isaac-sim-assets/Assets/Isaac/5.1/Isaac/IsaacLab",
        "/media/raviteja/Ubuntu_Data2/isaac-sim-assets/Assets/Isaac/5.1/Isaac/Samples",
    ]
  ```
  

