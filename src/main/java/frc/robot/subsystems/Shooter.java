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

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS


/**
 *
 */
public class Shooter extends SubsystemBase {
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    
    /**
    *
    */
    private WPI_TalonSRX shooterMotorL;
    private WPI_TalonSRX shooterMotorR;
    
    private final static double MAXSHOOTERSPEED = 111500;
    private static int timeout = 30;
    public Shooter() {
        shooterMotorL = new WPI_TalonSRX(14);
        shooterMotorR = new WPI_TalonSRX(15);
        shooterMotorL.configFactoryDefault();
        shooterMotorL.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, timeout);
        shooterMotorL.config_kF(0, 0.007);
        //ShooterMotor.config_kP(0, .1);
        shooterMotorL.config_kI(0, .0000035);

        shooterMotorR.configFactoryDefault();
        shooterMotorR.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, timeout);
        shooterMotorR.config_kF(0, 0.007);
        //ShooterMotor.config_kP(0, .1);
        shooterMotorR.config_kI(0, .0000035);
        shooterMotorR.setInverted(true);

        
        // shooterMotorR.follow(shooterMotorL);
        
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        double shooterVelocityL = shooterMotorL.getSelectedSensorVelocity(0);
        SmartDashboard.putNumber("Left Shooter velocity", shooterVelocityL);
        double LError = shooterMotorL.getClosedLoopError(0);
        SmartDashboard.putNumber("Left Shooter Error", LError);
        double desiredVelocityL = shooterMotorL.getClosedLoopTarget();
        SmartDashboard.putNumber("Left Shooter desiredVelocity", desiredVelocityL);
        boolean LAtTarget = Math.abs(LError) < 3000;
        SmartDashboard.putBoolean("Left shooter ready", LAtTarget);

        double shooterVelocityR = shooterMotorR.getSelectedSensorVelocity(0);
        SmartDashboard.putNumber("Right Shooter velocity", shooterVelocityR);
        double RError = shooterMotorR.getClosedLoopError(0);
        SmartDashboard.putNumber("Right Shooter Error", RError);
        double desiredVelocityR = shooterMotorR.getClosedLoopTarget();
        SmartDashboard.putNumber("Right Shooter desiredVelocity", desiredVelocityR);
        boolean RAtTarget = Math.abs(RError) < 3000;
        SmartDashboard.putBoolean("Right shooter ready", RAtTarget);

    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    public void shoot(double percentSpeed){
       //ShooterMotor.set(ControlMode.PercentOutput,percentSpeed);
        SmartDashboard.putNumber("Shooter Power", percentSpeed);
        shooterMotorL.set(ControlMode.Velocity,percentSpeed*MAXSHOOTERSPEED);
        shooterMotorR.set(ControlMode.Velocity,percentSpeed*MAXSHOOTERSPEED);
       
    } 
    public void stop(){
        shooterMotorL.set(ControlMode.PercentOutput, 0);
        shooterMotorR.set(ControlMode.PercentOutput, 0);
    
    }
}

