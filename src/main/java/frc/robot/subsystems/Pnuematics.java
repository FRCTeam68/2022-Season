package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;


import  frc.robot.Constants;

public class Pnuematics extends SubsystemBase {
    
    // Create both of the Intake Pnuematics
    private DoubleSolenoid intakeMover;

    private DoubleSolenoid primaryLift; // Not sure what each of them do yet, so I'll just number them
    private DoubleSolenoid primaryClamp;
    private DoubleSolenoid secondaryLift;
    private DoubleSolenoid secondaryClamp;

    private Compressor compressor;
    
    public Pnuematics(){

        /*
        compressor = new Compressor(Constants.AIR_PUMP_CAN, PneumaticsModuleType.REVPH);
        compressor.enableDigital();
        */
         intakeMover = new DoubleSolenoid(19,PneumaticsModuleType.REVPH, 2, 13);
         primaryLift = new DoubleSolenoid(19, PneumaticsModuleType.REVPH, 3, 12);
        // primaryClamp = new DoubleSolenoid(Constants.AIR_PUMP_CAN, Constants.INTAKE_PCM_B, Constants.INTAKE_PCM_A);
        // secondaryLift = new DoubleSolenoid(Constants.AIR_PUMP_CAN, Constants.INTAKE_PCM_B, Constants.INTAKE_PCM_A);
        // secondaryClamp = new DoubleSolenoid(Constants.AIR_PUMP_CAN, Constants.INTAKE_PCM_B, Constants.INTAKE_PCM_A);
        
    }

    @Override
    public void periodic(){

    }
/*
    public double getCompressorPressure(){
        return compressor.getPressure();
    }
*/
/*
    public Compressor getCompressor(){
        return compressor;
    }
*/
    // I have no clue if we'll actually need much of this, because I'm not entirely sure what intake will be doing
    public void setSolenoidOut(DoubleSolenoid solenoid){
        DoubleSolenoid temp = solenoid;
        temp.set(Value.kForward);
    }

    // Hopefully this whole referencing a reference will act as a pointer. If not, I'm gonna be real mad.
    public void setSolenoidIn(DoubleSolenoid solenoid){
        DoubleSolenoid temp = solenoid;
        temp.set(Value.kReverse);
    }

    public void setIntakeIn() {
        intakeMover.set(Value.kReverse);
      }
    
      public void setIntakeOut() {
        intakeMover.set(Value.kForward);
      }
      public Value getIntakePCM() {
        return intakeMover.get();
      }
      public void changeIntakeMode() {
        if (this.getIntakePCM() == Value.kReverse) {
          this.setIntakeOut();
        } else {
          this.setIntakeIn();
        }
      }

      public void extendLift() {
        primaryLift.set(Value.kReverse);
      }
    
      public void retractLift() {
        primaryLift.set(Value.kForward);
      }
      public Value getLiftPCM() {
        return primaryLift.get();
      }
      public void changeLiftMode() {
        if (this.getLiftPCM() == Value.kForward) {
          this.extendLift();
        } else {
          this.retractLift();
        }
      }

    public Value getSolenoidPosition(DoubleSolenoid solenoid){
        return solenoid.get();
    }

    public DoubleSolenoid getPrimaryLift(){
        return primaryLift;
    }

    
    public DoubleSolenoid getPrimaryClamp(){
        return primaryClamp;
    }

    
    public DoubleSolenoid getSecondaryLift(){
        return secondaryLift;
    }

    
    public DoubleSolenoid getSecondaryClamp(){
        return secondaryClamp;
    }

    public DoubleSolenoid getIntakeSolenoid() {
        return intakeMover;
    }


}
