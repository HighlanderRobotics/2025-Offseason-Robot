package frc.robot.beambreak;

import edu.wpi.first.wpilibj.DigitalInput;

public class BeamBreakIOReal implements BeamBreakIO {
  final DigitalInput beambreak;

  public BeamBreakIOReal() {
    beambreak = new DigitalInput(1);
  }

  public void updateInputs(BeambreakIOInputsAutoLogged inputs) {
    inputs.get = beambreak.get();
  }
}
