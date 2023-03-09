package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Rotation;

public class RotateArm extends CommandBase{
    Rotation rotation;
    double angle;
    //Subject to change
    double maxRange=0.04;
    //Creates a new RoateArm

    public RotateArm(final Rotation rotation, double newAngle){
        //Use addRequirements() here to declare subsystem dependencies
        addRequirements(rotation);
        this.rotation=rotation;
        angle=newAngle;
    }

    //Called when the command is initially scheduled
    @Override
    public void initialize(){}

    //Called everytime the scheduler runs while the command is scheduled
    @Override
    public void execute(){
        if (rotation.getAveragePosition()>angle + maxRange){
            //subject to change
            rotation.rotate(0.1);
        }
        else if (rotation.getAveragePosition()<angle - maxRange){
            //subject to change
            rotation.rotate(-0.1);
        } else {
            rotation.rotate(0);
        }
    }

    @Override
    public void end(boolean interrupted){
        rotation.stopRotate();
    }

    @Override
    public boolean isFinished(){
        if (rotation.getAveragePosition()>angle + maxRange){
  
            return false;
        }
        else if (rotation.getAveragePosition()<angle - maxRange){
          
            return false;
        } else {
            return true;
        }
    }
}
