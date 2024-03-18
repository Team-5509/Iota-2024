// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.util.List;
import java.util.function.DoubleSupplier;
import swervelib.SwerveController;
import swervelib.math.SwerveMath;

/**
 * An example command that uses an example subsystem.
 */
public class TeleopDrive extends Command {

  private final SwerveSubsystem swerve;
  private final DoubleSupplier vX, vY, heading;
  private double rotationSpeed;

  /**
   * Used to drive a swerve robot in full field-centric mode. vX and vY supply
   * translation inputs, where x is
   * torwards/away from alliance wall and y is left/right. headingHorzontal and
   * headingVertical are the Cartesian
   * coordinates from which the robot's angle will be derived— they will be
   * converted to a polar angle, which the robot
   * will rotate to.
   *
   * @param swerve  The swerve drivebase subsystem.
   * @param vX      DoubleSupplier that supplies the x-translation joystick input.
   *                Should be in the range -1 to 1 with
   *                deadband already accounted for. Positive X is away from the
   *                alliance wall.
   * @param vY      DoubleSupplier that supplies the y-translation joystick input.
   *                Should be in the range -1 to 1 with
   *                deadband already accounted for. Positive Y is towards the left
   *                wall when looking through the driver
   *                station glass.
   * @param heading DoubleSupplier that supplies the robot's heading angle.
   */
  public TeleopDrive(SwerveSubsystem swerve, DoubleSupplier vX, DoubleSupplier vY,
      DoubleSupplier heading) {
    this.swerve = swerve;
    this.vX = vX;
    this.vY = vY;
    this.heading = heading;
    
    rotationSpeed = 0;

    addRequirements(swerve);
  }
  public TeleopDrive(SwerveSubsystem swerve, double vX, double vY,
      double heading, double o) {
    
    this.swerve = swerve;
    rotationSpeed = 0;
    DoubleSupplier dx = () -> vX;
    DoubleSupplier dy = () -> vY;
    DoubleSupplier dTheta = () -> heading;
    addRequirements(swerve);
    //DoubleSupplier speedY = () -> vY;
    DoubleSupplier no = () -> 0;
    swerve.drive(new ChassisSpeeds(vX, vY, heading));//no,dy,no);
    this.vX = dx;
    this.vY = dy;
    this.heading = dTheta;
  }
  

  @Override
  public void initialize() {
    
  }
  private boolean pressed = false;
  private boolean previousPressed = false;
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double finnese = (RobotContainer.getInstance().getdriverXbox().getRightTriggerAxis() * .4 - 1) * -1;
    DoubleSupplier fVX = () -> vX.getAsDouble() * finnese;
    DoubleSupplier fVY = () -> vY.getAsDouble() * finnese;
    if (Math.abs(heading.getAsDouble()) > swerve.getSwerveController().config.angleJoyStickRadiusDeadband) {
      rotationSpeed = finnese * heading.getAsDouble()*swerve.getSwerveController().config.maxAngularVelocity;
    }
    else {
      rotationSpeed = 0;
    }
    boolean firstBool = RobotContainer.getInstance().getdriverXbox().getLeftBumper();
    if(!firstBool && previousPressed){
      pressed = !pressed;
    }
    previousPressed = firstBool;
    //double finnese = (RobotContainer.getInstance().getdriverXbox().getRightTriggerAxis() * .8 - 1) * -1;
    ChassisSpeeds desiredSpeeds = swerve.getTargetSpeeds(fVX.getAsDouble(), fVY.getAsDouble() , new Rotation2d(rotationSpeed));
    
    // Limit velocity to prevent tippy
    Translation2d translation = SwerveController.getTranslation2d(desiredSpeeds);
    
    // translation = SwerveMath.limitVelocity(translation, swerve.getFieldVelocity(), swerve.getPose(),
    //     Constants.LOOP_TIME, Constants.ROBOT_MASS, List.of(Constants.CHASSIS),
    //     swerve.getSwerveDriveConfiguration());
    SmartDashboard.putNumber("LimitedTranslation", translation.getX());
    SmartDashboard.putString("Translation", translation.toString());

    // Make the robot move
    if(!pressed){
      swerve.drive(translation, rotationSpeed * .5 * finnese, false);
    }else{
      //swerve.driveFieldOriented(desiredSpeeds);
    }
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

}
