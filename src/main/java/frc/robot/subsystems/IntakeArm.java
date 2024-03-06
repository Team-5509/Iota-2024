// RobotBuilder Version: 6.1
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

// ROBOTBUILDER TYPE: Subsystem.

package frc.robot.subsystems;


import frc.robot.commands.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS

// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS


/**
 *
 */
public class IntakeArm extends SubsystemBase {
    private DigitalInput top = new DigitalInput(0);
    private DigitalInput bottom = new DigitalInput(1);
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    
    /**
    *
    */
    private CANSparkMax flipper = new CANSparkMax(16, MotorType.kBrushless);
    
    //private DutyCycleEncoder flipperEncoder = new DutyCycleEncoder(2);
    public IntakeArm() {
        
        flipper.setIdleMode(CANSparkMax.IdleMode.kBrake);
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        
        SmartDashboard.putNumber("Duty Encoder", getFlipperPosition());
        SmartDashboard.putNumber("Arm Curent", flipper.getOutputCurrent());

    }

    @Override
    public void simulationPeriodic() {
        
        // This method will be called once per scheduler run when in simulation

    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    public void moveIntakeInBounds(double speed){
        flipper.set(speed);

        // if(top.get() && speed > 0 || false){//bottom.get() && speed < 0){
        //     flipper.set(0);
        // }else{
            // flipper.set(speed);
        // }
    }

    public double getFlipperPosition(){
        return flipper.getAbsoluteEncoder(Type.kDutyCycle).getPosition();
    }
    public double getCurrent(){
        return flipper.getOutputCurrent();
    }

}

