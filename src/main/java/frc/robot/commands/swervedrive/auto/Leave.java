package frc.robot.commands.swervedrive.auto;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.RunIntake;
import frc.robot.commands.ShootAtPercentSpeed;
import frc.robot.commands.swervedrive.drivebase.TeleopDrive;
import frc.robot.subsystems.*;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class Leave extends SequentialCommandGroup{
     
    public Leave(SwerveSubsystem drivetrain, Shooter shooter, Intake outTake){
        addCommands((new TeleopDrive(drivetrain, .75, -0.2 , 0, 0)).withTimeout(2));
        
        
        // addCommands(
        //     //new DriveXFeet(-5.5, drivetrain),
        //     race(
        //         (new ShootAtPercentSpeed(0.863, shooter)).withTimeout(5),
        //         (new WaitCommand(2)).andThen(
        //             (new RunIntake(outTake, -1))).withTimeout(2)
                    
        //     )
        // );
    }
}
