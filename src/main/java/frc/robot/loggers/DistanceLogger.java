package frc.robot.loggers;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;

@CustomLoggerFor(Distance.class)
public class DistanceLogger extends ClassSpecificLogger<Distance> {
  public DistanceLogger() {
    super(Distance.class);
  }

  @Override
  public void update(EpilogueBackend backend, Distance distance) {
    backend.log("Inches", distance.in(Units.Inches));
    backend.log("Feet", distance.in(Units.Feet));
    backend.log("Meters", distance.in(Units.Meters));
  }
}
