# 2018PowerUpCode
Subsystems: 
	BoxCatcher
    - moveFoward()
    - moveBack()
    - turnWheel()
  Elevator
    - moveUp()
    - moveDown()
  Winch 
    - rotate()
  LimeLight
    - lightOn()
    - lightOff()
Automate: 
  - seek and get box and stop
  - 4 set positions for elevator
  - intake and retract 
Autonomous:
  - move forward 
  - deliver box to scale 
  - deliver box to switch
Sensors needed: 
  - limit switch 
  - absolute encoder 
  - gyroscope 
OI: 
  - different driving modes 
