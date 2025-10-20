package frc.robot.canrange;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import edu.wpi.first.units.measure.Distance;

public class CANrangeIOReal implements CANrangeIO {
  private final CANrange canrange;

  private final StatusSignal<Distance> distance;

  public CANrangeIOReal(int CANrangeID) {
    canrange = new CANrange(CANrangeID, "*");
    distance = canrange.getDistance();

    // TODO: adjust config vlaues and may change depending also diff configs for each
    // not completely sure which are needed
    final CANrangeConfiguration config = new CANrangeConfiguration();
    config.ToFParams.UpdateFrequency = 50; // update frequency in Hz

    canrange.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(CANrangeIOInputs inputs) {
    BaseStatusSignal.refreshAll(distance);

    inputs.distanceM = distance.getValueAsDouble();
  }
}
