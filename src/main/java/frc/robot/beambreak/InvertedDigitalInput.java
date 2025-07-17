package frc.robot.beambreak;

import edu.wpi.first.wpilibj.DigitalInput;

/** A digital input (ie a beambreak or limit switch) whose normal state is reversed */
public class InvertedDigitalInput extends DigitalInput { // why does this even exist
  public InvertedDigitalInput(int channel) {
    super(channel);
  }

  public boolean get() {
    return !super.get();
  }
}
