package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Rotation;

public class RotateArm extends CommandBase{
    Rotation rotation;
    double targetAngle;
    //Subject to change
    double maxRange=0.04;
    boolean finished = false;
    //Creates a new RoateArm

    public RotateArm(final Rotation rotation, double newAngle){
        //Use addRequirements() here to declare subsystem dependencies
        addRequirements(rotation);
        this.rotation=rotation;
        targetAngle=newAngle;
    }

    //Called when the command is initially scheduled
    @Override
    public void initialize(){
        finished = false;
    }


    //Called everytime the scheduler runs while the command is scheduled
    @Override
    public void execute(){
        double kP = 0.04;
        double error = rotation.getAverageArmAngle() - targetAngle;
        
        double output = error * kP;
    
        output = Math.copySign(Math.min(0.2, Math.abs(output)), output);
        
        if (Math.abs(error) < 0.2){
            output = 0;
            finished = true;
        }

    
        rotation.rotate(output);
    }

    @Override
    public void end(boolean interrupted){
        rotation.stopRotate();
    }

    @Override
    public boolean isFinished(){
        return finished;
    }
}
