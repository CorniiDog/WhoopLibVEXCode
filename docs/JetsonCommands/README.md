# Jetson Commands

## reboot_jetson

This function would reboot the Jetson Nano

```
/**
* Restarts Jetson Nano
*/
void reboot_jetson();
```

#### Example:

```cpp
jetson_commander.reboot_jetson();
```

## shutdown_jetson

```
/**
* Shuts down Jetson Nano
*/
void shutdown_jetson();
```

#### Example:

```cpp
jetson_commander.shutdown_jetson();
```

## restart_vision_process

This restarts the processes for the Jetson Nano. It is quicker than shutting down, but may take a bit to initialize.

```
/**
* Restarts the vision process on Jetson Nano
*/
void restart_vision_process();
```

#### Example:

```cpp
jetson_commander.restart_vision_process();
```

## initialize

This is to be used at the start of the robot's program to send an initialization message to the Jetson Nano.

```
/**
* Sends initialization message to Jetson Nano
*/
void initialize();
```

#### Example:

```cpp
void pre_auton(void)
{

  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  manager.start();
  robot_drivetrain.set_state(drivetrainState::mode_disabled);
  controller1.notify("Initializing", 2);
  jetson_commander.initialize(); // Initialize Jetson Commander
}
```

## is_connected_to_jetson

Returns true if the V5 Brain is connected to the Jetson Nano and established positive contact

```
/**
* Returns true if connected to Jetson Nano
* @returns true if connected, false otherwise (delay 5-6 seconds)
*/
bool is_connected_to_jetson();
```

#### Example:

```cpp
bool is_connected = jetson_commander.is_connected_to_jetson();

if(is_connected){
    // Do something if the Jetson is connected
}
else{
    // Do another thing
}

```

Checkout [Drivetrain Commands](DrivetrainCommands/README.md).