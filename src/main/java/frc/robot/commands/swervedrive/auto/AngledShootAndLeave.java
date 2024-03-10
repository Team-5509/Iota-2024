package frc.robot.commands.swervedrive.auto;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AutoIntake;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.RunIntake;
import frc.robot.commands.ShootAtPercentSpeed;
import frc.robot.commands.swervedrive.drivebase.TeleopDrive;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class AngledShootAndLeave extends SequentialCommandGroup{
     
    public AngledShootAndLeave(SwerveSubsystem drivetrain, Shooter shooter, Intake outTake){
        
        //shooter.shootAuto(.85);
        addCommands(
            //new DriveXFeet(-5.5, drivetrain),
            (new AutoIntake(1, outTake)).withTimeout(1),
            Commands.race(
                (new AutoShoot(1, shooter)),//
                (new WaitCommand(1)).andThen(
                    (new AutoIntake(-1, outTake)))
                    
            ).withTimeout(5),
            (new TeleopDrive(drivetrain, .8, -0.2 , 0, 0)).withTimeout(1.5),
            Commands.race((new AutoShoot(0, shooter)),//
                (new AutoIntake(0, outTake))
                ).withTimeout(1)
            
        );
        //shooter.shootAuto(0);//
    }
}
