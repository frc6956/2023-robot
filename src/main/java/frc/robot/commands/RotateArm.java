package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Rotation;

public class RotateArm extends CommandBase{
    Rotation rotation;
    double position;
    //Subject to change
    double maxRange=0;
    //Creates a new RoateArm

    public RotateArm(final Rotation rotation, double newPosition){
        //Use addRequirements() here to declare subsystem dependencies
        addRequirements(rotation);
        this.rotation=rotation;
        position=newPosition;
    }

    //Called when the command is initially scheduled
    @Override
    public void initialize(){}

    //Called everytime the scheduler runs while the command is scheduled
    @Override
    public void execute(){
        if (rotation.getAveragePosition()>position){
            //subject to change
            rotation.setArmAngle(-0.07);
        }
        else if (rotation.getAveragePosition()<position){
            //subject to change
            rotation.setArmAngle(0.07);
        }
    }

    @Override
    public void end(boolean interrupted){
        rotation.stopRotate();
    }

    @Override
    public boolean isFinished(){
        if(position >= 0 && rotation.getAveragePosition()>=position){
            return true;
        }
        else if (position<=0 && rotation.getAverageArmAngle()<=position){
            return true;
        }
        else{
            return false;
        }
    }
}
