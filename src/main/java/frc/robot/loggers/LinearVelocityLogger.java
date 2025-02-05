package frc.robot.loggers;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.LinearVelocity;

@CustomLoggerFor(LinearVelocity.class)
public class LinearVelocityLogger extends ClassSpecificLogger<LinearVelocity> {
  public LinearVelocityLogger() {
    super(LinearVelocity.class);
  }

  @Override
  public void update(EpilogueBackend backend, LinearVelocity LinearVelocity) {
    backend.log("Inches per sec", LinearVelocity.in(Units.InchesPerSecond));
    backend.log("Feet per sec", LinearVelocity.in(Units.FeetPerSecond));
    backend.log("Meters per sec", LinearVelocity.in(Units.MetersPerSecond));
  }
}
