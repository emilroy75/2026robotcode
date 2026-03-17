package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.IntegerArraySubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class FuelDetection {
  // network table
  private final NetworkTable table;
  // Cluster subscribers
  private final IntegerSubscriber clusterCount;
  private final DoubleArraySubscriber clusterX1;
  private final DoubleArraySubscriber clusterY1;
  private final DoubleArraySubscriber clusterX2;
  private final DoubleArraySubscriber clusterY2;
  private final IntegerArraySubscriber clusterBalls;
  private final DoubleArraySubscriber clusterDist;
  private final DoubleArraySubscriber clusterOffset;

  // Single ball subscribers
  private final IntegerSubscriber singleCount;
  private final DoubleArraySubscriber singleX1;
  private final DoubleArraySubscriber singleY1;
  private final DoubleArraySubscriber singleX2;
  private final DoubleArraySubscriber singleY2;
  private final DoubleArraySubscriber singleDist;
  private final DoubleArraySubscriber singleOffset;

  public FuelDetection() {
    table = NetworkTableInstance.getDefault().getTable("FuelDetection");
    // cluster
    clusterCount = table.getIntegerTopic("clusters/count").subscribe(0);
    clusterX1 = table.getDoubleArrayTopic("clusters/x1").subscribe(new double[] {});
    clusterY1 = table.getDoubleArrayTopic("clusters/y1").subscribe(new double[] {});
    clusterX2 = table.getDoubleArrayTopic("clusters/x2").subscribe(new double[] {});
    clusterY2 = table.getDoubleArrayTopic("clusters/y2").subscribe(new double[] {});
    clusterBalls = table.getIntegerArrayTopic("clusters/balls").subscribe(new long[] {});
    clusterDist = table.getDoubleArrayTopic("clusters/distance").subscribe(new double[] {});
    clusterOffset = table.getDoubleArrayTopic("clusters/offset").subscribe(new double[] {});

    // Singles
    singleCount = table.getIntegerTopic("singles/count").subscribe(0);
    singleX1 = table.getDoubleArrayTopic("singles/x1").subscribe(new double[] {});
    singleY1 = table.getDoubleArrayTopic("singles/y1").subscribe(new double[] {});
    singleX2 = table.getDoubleArrayTopic("singles/x2").subscribe(new double[] {});
    singleY2 = table.getDoubleArrayTopic("singles/y2").subscribe(new double[] {});
    singleDist = table.getDoubleArrayTopic("singles/distance").subscribe(new double[] {});
    singleOffset = table.getDoubleArrayTopic("singles/offset").subscribe(new double[] {});
  }
  // Cluster getters
  public long getClusterCount() {
    return clusterCount.get();
  }

  public double[] getClusterX1() {
    return clusterX1.get();
  }

  public double[] getClusterY1() {
    return clusterY1.get();
  }

  public double[] getClusterX2() {
    return clusterX2.get();
  }

  public double[] getClusterY2() {
    return clusterY2.get();
  }

  public long[] getClusterBalls() {
    return clusterBalls.get();
  }

  public double[] getClusterDistances() {
    return clusterDist.get();
  }

  public double[] getClusterOffsets() {
    return clusterOffset.get();
  }

  // Single getters
  public long getSingleCount() {
    return singleCount.get();
  }

  public double[] getSingleX1() {
    return singleX1.get();
  }

  public double[] getSingleY1() {
    return singleY1.get();
  }

  public double[] getSingleX2() {
    return singleX2.get();
  }

  public double[] getSingleY2() {
    return singleY2.get();
  }

  public double[] getSingleDistances() {
    return singleDist.get();
  }

  public double[] getSingleOffsets() {
    return singleOffset.get();
  }

  // Utility methods
  public boolean hasClusters() {
    return getClusterCount() > 0;
  }

  public boolean hasSingles() {
    return getSingleCount() > 0;
  }
}
