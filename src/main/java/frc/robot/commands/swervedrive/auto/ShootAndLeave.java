package frc.robot.commands.swervedrive.auto;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.RunIntake;
import frc.robot.commands.ShootAtPercentSpeed;
import frc.robot.commands.swervedrive.drivebase.TeleopDrive;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class ShootAndLeave extends SequentialCommandGroup{
     
    public ShootAndLeave(SwerveSubsystem drivetrain, Shooter shooter, Intake outTake){
        
        shooter.shootAuto(.85);
        addCommands(
            //new DriveXFeet(-5.5, drivetrain),
            
            Commands.race(
                (new ShootAtPercentSpeed(true, 0.863, shooter)).withTimeout(5),//
                (new WaitCommand(2)).andThen(
                    (new RunIntake(outTake, -1))).withTimeout(2)
                    
            ),
            (new TeleopDrive(drivetrain, .75, -0.2 , 0, 0)).withTimeout(2)
        );
        shooter.shootAuto(0);//
    }
}
