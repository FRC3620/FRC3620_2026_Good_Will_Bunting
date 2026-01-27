# IO Assignments

## Digital IO
* DIO 0: Practice Bot Jumper


## Analog IO


## PWM


## Motor Controllers


# CANCoders


# PDB assignments


# Vision

## Joehan limelight locations

### limelight-front:


### limelight-back:



# Driver Controller

## Slow Drive Mode
- **Robot Oriented limited to 30% power**
  - **Xbox Controller**: Hold Left Bumper
  - **FlySky Controller**: Toggle SWF

## General Controls
- **NavX Reset (Square Up Robot)**
  - **Xbox Controller**: Press A Button
  - **FlySky Controller**: Toggle SWA

## Button Box Mappings
### ESEF Positioning Commands
- **A1, C1**
  - Moves to L1 position
  - Returns to Home on release
- **A2, C2**
  - Moves to L2 position
  - Returns to Home on release
- **A3, C3**
  - Moves to L3 position
  - Returns to Home on release
- **A4, C4**
  - Moves to L4 position
  - Returns to Home on release

### End Effector Commands
- **A1, A2, A3, A4, C1, C2, C3, C4 (Right Trigger)**
  - Runs End Effector until Coral is gone (0.9 speed)
  - Stops End Effector on release

### Station Pickup Commands
- **D2**
  - Moves to Station Pickup Position
  - Runs End Effector until Coral is detected (0.4 speed)
- **D3**
  - Moves to Station Pickup Position
  - Runs End Effector at 0.2 speed

### Algae Claw Controls
- **B4**
  - Moves to Barge Position
  - Runs End Effector at -0.95 speed
  - Stops on release
- **B2**
  - Moves to Algae L2 Position, runs End Effector (0.45 speed)
  - After timeout, moves to Algae L2 Remove Position
  - Returns Home on release
- **B3**
  - Moves to Algae L3 Position, runs End Effector (0.45 speed)
  - After timeout, moves to Algae L3 Remove Position
  - Returns Home on release

### AFI Subsystem Controls
- **B1**
  - Moves Pivot to 15 degrees, runs Roller (0.5 speed)
  - Moves Pivot to 80 degrees, runs Roller at 0.02 speed
  - Right Trigger: Runs Roller at -0.5 speed, stops on release

### Climber Commands
- **D1 (Right Trigger)**
  - Moves to Climb Position, sets Climber Power to 0.7
  - Stops Climber on release
- **D1 (Left Trigger)**
  - Moves to Climb Position, sets Climber Power to -1
  - Stops Climber on release


