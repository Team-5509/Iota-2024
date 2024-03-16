// RobotBuilder Version: 6.1
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

// ROBOTBUILDER TYPE: Command.

package frc.robot.commands;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkBase.IdleMode;

import frc.robot.RobotContainer;
// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import frc.robot.subsystems.IntakeArm;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS

/**
 *
 */
public class Flipper extends Command {
     private double maxFlipperPosition;
     private double minFlipperPosition;
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS
        private final IntakeArm m_intakeArm;
 
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS


    public Flipper(IntakeArm subsystem) {


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES

        m_intakeArm = subsystem;
        addRequirements(m_intakeArm);

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
    }
   
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        maxFlipperPosition = 0.030;
        minFlipperPosition = 0.441;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
       
        double speed = 0;
        XboxController aux_Controller = RobotContainer.getInstance().getAuxController();
        
        int pov = aux_Controller.getPOV();
        boolean up = pov == 0 || pov == 45 || pov == 315;
        boolean down = pov == 180 || pov == 135 || pov == 225;

        if (up) {
            speed = .2;
        }
        if (down) {
            speed = -.2;
        }
        if(!(up || down)){
            speed = 0;
        }
        //check if method should be called. 
        if (  m_intakeArm.getFlipperPosition() > maxFlipperPosition && speed < 0 ||  m_intakeArm.getFlipperPosition() < minFlipperPosition && speed > 0 || m_intakeArm.getCurrent() >= 30){
           // m_intakeArm.setIdleMode(IdleMode.kCoast);
            m_intakeArm.moveIntakeInBounds(0);
        }
        else{
            m_intakeArm.moveIntakeInBounds(speed);
        }
        // check min and max dont hardcode max
        //m_intakeArm.setIdleMode(IdleMode.kBrake);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public boolean runsWhenDisabled() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
        return false;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
    }
}
