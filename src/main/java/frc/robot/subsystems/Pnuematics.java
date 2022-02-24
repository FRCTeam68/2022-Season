package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pnuematics extends SubsystemBase {
    
    // Create both of the Intake Pnuematics
    private DoubleSolenoid intakeMover;

    private DoubleSolenoid endgameSolenoid1; // Not sure what each of them do yet, so I'll just number them
    private DoubleSolenoid endgameSolenoid2;
    private DoubleSolenoid endgameSolenoid3;
    private DoubleSolenoid endgameSolenoid4;

    private Compressor compressor;
    
    public Pnuematics(){

        // Init Double Solenoids a.k.a. Pnuematic cylinders or whatever they are called. Apparently we need 4 endgame ones and
        // couple of other things.

        // intakeMover = new DoubleSolenoid(Constants.AIR_PUMP_CAN, Constants.INTAKE_PCM_B, Constants.INTAKE_PCM_A);
        // endgameSolenoid1 = new DoubleSolenoid(Constants.AIR_PUMP_CAN, Constants.INTAKE_PCM_B, Constants.INTAKE_PCM_A);
        // endgameSolenoid2 = new DoubleSolenoid(Constants.AIR_PUMP_CAN, Constants.INTAKE_PCM_B, Constants.INTAKE_PCM_A);
        // endgameSolenoid3 = new DoubleSolenoid(Constants.AIR_PUMP_CAN, Constants.INTAKE_PCM_B, Constants.INTAKE_PCM_A);
        // endgameSolenoid4 = new DoubleSolenoid(Constants.AIR_PUMP_CAN, Constants.INTAKE_PCM_B, Constants.INTAKE_PCM_A);
        compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
    }

    @Override
    public void periodic(){

    }

    public double getCompressorPressure(){
        return compressor.getPressure();
    }

    public Compressor getCompressor(){
        return compressor;
    }

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

    public void changeSolenoidPosition(DoubleSolenoid solenoid){
        DoubleSolenoid temp = solenoid;
        if (this.getSolenoidPosition(temp) == Value.kForward){
            this.setSolenoidIn(temp);
        }else {
            this.setSolenoidOut(temp);
        }
    }

    public Value getSolenoidPosition(DoubleSolenoid solenoid){
        return solenoid.get();
    }

    public DoubleSolenoid getEndgameSolenoid1(){
        return endgameSolenoid1;
    }

    
    public DoubleSolenoid getEndgameSolenoid2(){
        return endgameSolenoid2;
    }

    
    public DoubleSolenoid getEndgameSolenoid3(){
        return endgameSolenoid3;
    }

    
    public DoubleSolenoid getEndgameSolenoid4(){
        return endgameSolenoid4;
    }

    public DoubleSolenoid getIntakeSolenoid() {
        return intakeMover;
    }


}
