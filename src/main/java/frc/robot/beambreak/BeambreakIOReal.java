package frc.robot.beambreak;

import edu.wpi.first.wpilibj.DigitalInput;

public class BeambreakIOReal implements BeambreakIO {
  final DigitalInput beambreak;

  public BeambreakIOReal(int id, boolean invert) {
    beambreak = invert ? new InvertedDigitalInput(id) : new DigitalInput(id);
  }

  public void updateInputs(BeambreakIOInputsAutoLogged inputs) {
    inputs.get = beambreak.get();
  }
}
