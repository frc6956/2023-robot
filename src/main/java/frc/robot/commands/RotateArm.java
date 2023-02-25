package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Rotation;

public class RotateArm extends CommandBase{
    Rotation rotation;
    double angle;
    //Subject to change
    double maxRange=0.5;
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
        if (rotation.getAveragePosition()>angle){
            //subject to change
            rotation.rotate(-0.07);
        }
        else if (rotation.getAveragePosition()<angle){
            //subject to change
            rotation.rotate(0.07);
        }
    }

    @Override
    public void end(boolean interrupted){
        rotation.stopRotate();
    }

    @Override
    public boolean isFinished(){
        if(angle >= 0 && rotation.getAveragePosition()>=angle){
            return true;
        }
        else if (angle<=0 && rotation.getAverageArmAngle()<=angle){
            return true;
        }
        else{
            return false;
        }
    }
}
