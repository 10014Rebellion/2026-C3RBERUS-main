package frc.robot.systems.shooter.indexers;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.tuning.LoggedTunableNumber;
import frc.robot.systems.shooter.ShooterConstants.IndexerConstants;

public class Indexers extends SubsystemBase {
  private final IndexerIO mLeaderIndexerIO;
  private final IndexerIO mFollowerIndexerIO;

  private final IndexerInputsAutoLogged mLeaderIndexerInputs = new IndexerInputsAutoLogged();
  private final IndexerInputsAutoLogged mFollowerIndexerInputs = new IndexerInputsAutoLogged();

  private final LoggedTunableNumber tIndexerKP = new LoggedTunableNumber("Indexer/Control/kP", IndexerConstants.kIndexerControlConfig.pdController().kP());
  private final LoggedTunableNumber tIndexerKD = new LoggedTunableNumber("Indexer/Control/kD", IndexerConstants.kIndexerControlConfig.pdController().kD());
  private final LoggedTunableNumber tIndexerKS = new LoggedTunableNumber("Indexer/Control/kS", IndexerConstants.kIndexerControlConfig.feedforward().getKs());
  private final LoggedTunableNumber tIndexerKV = new LoggedTunableNumber("Indexer/Control/kV", IndexerConstants.kIndexerControlConfig.feedforward().getKv());
  private final LoggedTunableNumber tIndexerKA = new LoggedTunableNumber("Indexer/Control/kA", IndexerConstants.kIndexerControlConfig.feedforward().getKa());
  
  private SimpleMotorFeedforward mIndexerFeedForward = IndexerConstants.kIndexerControlConfig.feedforward();
  
  public Indexers(IndexerIO pLeaderIndexerIO, IndexerIO pFollowerIndexerIO) {
    this.mLeaderIndexerIO = pLeaderIndexerIO;
    this.mFollowerIndexerIO = pFollowerIndexerIO;
  }

  public double getAvgIndexerRPS() {
    return (mLeaderIndexerInputs.iIndexerVelocityRPS + mFollowerIndexerInputs.iIndexerVelocityRPS) / 2.0;
  }

  public void setIndexerRPS(double pDesiredRPS) {
    double calculatedFF = mIndexerFeedForward.calculateWithVelocities(getAvgIndexerRPS(), pDesiredRPS);
    mLeaderIndexerIO.setMotorVelocity(pDesiredRPS, calculatedFF);
    mFollowerIndexerIO.enforceFollower();
  }

  public void setIndexerVolts(double pVolts) {
    mLeaderIndexerIO.setMotorVolts(pVolts);
    mFollowerIndexerIO.enforceFollower();
  }

  public void stopIndexerMotor() {
    mLeaderIndexerIO.stopMotor();
    mFollowerIndexerIO.enforceFollower();
  }

  private void setBothPDConstants(double pKP, double pKD) {
    mLeaderIndexerIO.setPDConstants(pKP, pKD);
    mFollowerIndexerIO.setPDConstants(pKP, pKD);
  }
  
  @Override
  public void periodic() {
    mLeaderIndexerIO.updateInputs(mLeaderIndexerInputs);
    mFollowerIndexerIO.updateInputs(mFollowerIndexerInputs);

    refreshTuneables();
    mFollowerIndexerIO.enforceFollower();

    Logger.processInputs("Indexer/Leader", mLeaderIndexerInputs);
    Logger.processInputs("Indexer/Follower", mFollowerIndexerInputs);
  }

  private void refreshTuneables() {
    LoggedTunableNumber.ifChanged( hashCode(), 
      () -> setBothPDConstants(tIndexerKP.get(), tIndexerKD.get()), 
      tIndexerKP, tIndexerKD
    );
  
    LoggedTunableNumber.ifChanged( hashCode(), 
      () -> mIndexerFeedForward = new SimpleMotorFeedforward(tIndexerKS.get(), tIndexerKV.get(), tIndexerKA.get()), 
      tIndexerKS, tIndexerKV, tIndexerKA
    );
  }
}
