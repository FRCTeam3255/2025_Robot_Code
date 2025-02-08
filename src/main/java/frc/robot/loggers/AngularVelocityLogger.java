package frc.robot.loggers;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;

@CustomLoggerFor(AngularVelocity.class)
public class AngularVelocityLogger extends ClassSpecificLogger<AngularVelocity> {
  public AngularVelocityLogger() {
    super(AngularVelocity.class);
  }

  @Override
  public void update(EpilogueBackend backend, AngularVelocity AngularVelocity) {
    backend.log("Degrees per sec", AngularVelocity.in(Units.DegreesPerSecond));
    backend.log("Rotations per sec", AngularVelocity.in(Units.RotationsPerSecond));
    backend.log("RPM", AngularVelocity.in(Units.RPM));
  }
}
