// Required Functionality
// 1 neo 
// 3 possible statuses Ejecting, Intaking, Idle
    // Ejecting speed:-0.4  Intaking Speed: 1 Idle Speed: 0.05
// 2 GamePieces  Cone, Cube
    // If gamepiece == Cone invert the speed for each state
// Don't worry about any LED effects
// External Subsystem methods:
    // isFull()
    // startEjecting()
    // startIntaking()
    // setIdle()
    // setGamePiece( GamePiece gampiece)
    // setStatus(Status status)
    // Superclass abstract methods
// In a periodic function check for isFull with the following logic
      
                // if (motor.getOutputCurrent() > RobotConstants.Arm.Claw.kCurrentMax) {
                //     currentSpikeCounter++;
                //     if (currentSpikeCounter > RobotConstants.Arm.Claw.kCurrentSpikeIterations) {
                //         isFull = true;
                //         setStatus(Status.IDLE);
                //         currentSpikeCounter = 0;
                //         setIdle();
                //     }
                // } else {
                //     currentSpikeCounter = 0;
                // }
                // break; 
    // Note that calls to sensors (such as motor.getOutputCurrent()) should be done through an input class where the method is called in the function updateInputs(LoggableInputs inputs) and future references to the value are called through inputs.motorOutputCurrent
// In a periodic Funtion check for complete ejection should be don through the following logic
                // ejectCounter++;
                // if (ejectCounter > RobotConstants.Arm.Claw.kEjectIterations) {
                //     ejectCounter = 0;
                //     isFull = false;
                // }
    //Note that this logic should only be called while status is ejecting

// Please do not just copy the snippets from above, they will not directly work, just use them as inspiration
// Ask Armaan if you have any questions relating to advantagekit
// This resource will probably be helpful https://github.com/Mechanical-Advantage/AdvantageKit/blob/main/docs/CODE-STRUCTURE.md 
// Feel free to look at the Arm subsystems and anything in mustanglib
            