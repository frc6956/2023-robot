package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Extension;


public class ExtendArm extends CommandBase{
    Extension extension;
    double targetPosition;
    boolean finished = false;

    public ExtendArm(final Extension extension, final double newPosition){
        targetPosition=newPosition;
        this.extension=extension;
        addRequirements(extension);
    }



    @Override
    public void initialize(){
        finished = false;
    }

    //Xalled evvery time the scheduler tuns while th ecommand is scheduled
    @Override 
    public void execute(){
        
        double kP = 0.5;
        double error =  targetPosition - extension.getExtensionAveragePosition();
        
        double output = error * kP;
    
        output = Math.copySign(Math.min(0.7, Math.abs(output)), output);
        if (Math.abs(error) < 0.4){
            output = 0; 
            finished = true;
        } 

    
        extension.extendArm(output);
    }

    //Called once the command ends or is interrupted
    @Override
    public void end(boolean interrupted){
        extension.stopArm();
    }

    //Returns true when the command should end
    @Override
    public boolean isFinished(){
        
        return finished;
    }

}