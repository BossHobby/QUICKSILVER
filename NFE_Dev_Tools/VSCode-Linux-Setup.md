VSCode Linux Debug-Setup
========================

Replace <board-name> in the following appropriately.

1. Install vscode
2. Run the following
```shell
apt install openocd dfu-util gcc-arm-none-eabi gdb-arm-none-eabi libnewlib-arm-none-eabi libstdc++-arm-none-eabi-newlib
```
3. Open Folder `<guano-repo>/QUICKSILVER` with VSCode
4. Install "Cortex-Debug" extension and the "STM32F4 Device Support Plugin"
5. Select Debug on the left, "Add Configuration..." in the Dropdown, Compare with the following:
```json
{
  "name": "Cortex Debug",
  "cwd": "${workspaceRoot}",
  "executable": "${workspaceRoot}/build/quicksilver.<board-name>.debug.elf",
  "request": "launch",
  "type": "cortex-debug",
  "preLaunchTask": "build",
  "servertype": "openocd",
  "configFiles": [
      "${workspaceRoot}/openocd_stm32f4.cfg"
  ]
}
```
6. Create File "openocd_stm32f4.cfg" in Project Root:
```
source [find interface/stlink-v2.cfg]
source [find target/stm32f4x.cfg]

# use hardware reset, connect under reset
reset_config srst_only srst_nogate
```
6. Create File ".vscode/tasks.json":
```json
{
  // See https://go.microsoft.com/fwlink/?LinkId=733558
  // for the documentation about the tasks.json format
  "version": "2.0.0",
  "tasks": [
    {
      "label": "build",
      "type": "shell",
      "command": "make MODE=release clean <board-name>",
      "problemMatcher": [
        "$gcc"
      ],
      "group": {
        "kind": "build",
        "isDefault": true
      }
    }
  ]
}
```
7. Press F5 to Debug, Press CTRL+SHIFT+B for build
