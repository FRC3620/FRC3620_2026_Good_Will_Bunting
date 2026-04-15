package frc.robot.Helpers;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.RobotContainer;

import java.util.function.Consumer;

public class OrchestraManager {

    private final Orchestra m_orchestra = new Orchestra();
    private boolean m_isLoaded = false;

    // Called once to apply AudioConfigs to every motor that might ever be used
    private final TalonFX[] m_allMotors;

    public OrchestraManager(TalonFX... allMotors) {
        m_allMotors = allMotors;
        AudioConfigs audioConfigs = new AudioConfigs()
                .withAllowMusicDurDisable(true)
                .withBeepOnBoot(false)
                .withBeepOnConfig(false);
        for (TalonFX motor : allMotors) {
            if (motor == null)
                continue;
            motor.getConfigurator().apply(audioConfigs);
        }
    }

    // SongConfig is just a lambda that receives the OrchestraManager
    // and calls addInstrument() however it wants for that song
    @FunctionalInterface
    public interface SongConfig {
        void configure(OrchestraManager mgr);
    }

    public enum OrchestraSong {
        TITANIUM("Titanium", "titanium.chrp", mgr -> {
            // MELODY 1 (Primary - Track 3) - Most important, give it the most prominent
            // motors
            mgr.addInstrument(RobotContainer.shooterSubsystem.getMotor1(), 3); // Primary melody
            mgr.addInstrument(RobotContainer.shooterSubsystem.getMotor2(), 3); // Double the melody for volume

            // MELODY 2 (Secondary - Track 13) - Harmony/countermelody
            mgr.addInstrument(RobotContainer.swerveSubsystem.getModule(0).getSteerMotor(), 13);
            mgr.addInstrument(RobotContainer.swerveSubsystem.getModule(1).getSteerMotor(), 13);

            // BASS LINE (Track 4) - Essential for rhythm, low notes
            mgr.addInstrument(RobotContainer.swerveSubsystem.getModule(2).getSteerMotor(), 4);
            mgr.addInstrument(RobotContainer.swerveSubsystem.getModule(3).getSteerMotor(), 4);

            // RHYTHM/CHORDS (Track 0 & 1) - Support tracks
            mgr.addInstrument(RobotContainer.intakeShoulderSubsystem.getMotor(), 0); // Rhythm guitar/pad
            mgr.addInstrument(RobotContainer.shooterHoodSubsystem.getMotor(), 1); // Harmony support

            // DRUM/PERCUSSION elements (Track 9 is massive polyphony - likely drums)
            mgr.addInstrument(RobotContainer.turretSubsystem.getMotor(), 9);
            mgr.addInstrument(RobotContainer.preshooterSubsystem.getMotor(), 9);
            mgr.addInstrument(RobotContainer.swerveSubsystem.getModule(0).getDriveMotor(), 9);
            mgr.addInstrument(RobotContainer.swerveSubsystem.getModule(1).getDriveMotor(), 9);

            // ACCENTS & FILLS (Track 6, 10, 12 - similar patterns)
            mgr.addInstrument(RobotContainer.swerveSubsystem.getModule(2).getDriveMotor(), 6);
            mgr.addInstrument(RobotContainer.swerveSubsystem.getModule(3).getDriveMotor(), 10);
            mgr.addInstrument(RobotContainer.conveyerSubsystem.getMotor(), 12);

            // SPARSE tracks (Track 2, 5, 7, 8, 11) - add texture
            mgr.addInstrument(RobotContainer.intakeRollerSubsystem.getMotor1(), 2); // Occasional accents
            mgr.addInstrument(RobotContainer.intakeRollerSubsystem.getMotor2(), 5); // Very sparse (3 notes only)
            mgr.addInstrument(RobotContainer.intakeAgitatorSubsystem.getMotor(), 7); // Effects
        }),

        CALLMEMAYBE("Call Me Maybe", "CallMeMaybe.chrp", mgr -> {
            // MELODY (Track 1) - Assign to the most powerful/loudest motors
            mgr.addInstrument(RobotContainer.shooterSubsystem.getMotor1(), 1);
            mgr.addInstrument(RobotContainer.shooterSubsystem.getMotor2(), 1);
            mgr.addInstrument(RobotContainer.swerveSubsystem.getModule(0).getSteerMotor(), 1);
            mgr.addInstrument(RobotContainer.swerveSubsystem.getModule(1).getSteerMotor(), 1);
            mgr.addInstrument(RobotContainer.swerveSubsystem.getModule(2).getSteerMotor(), 1);
            mgr.addInstrument(RobotContainer.swerveSubsystem.getModule(3).getSteerMotor(), 1);
            mgr.addInstrument(RobotContainer.intakeShoulderSubsystem.getMotor(), 1);
            mgr.addInstrument(RobotContainer.shooterHoodSubsystem.getMotor(), 1);
            mgr.addInstrument(RobotContainer.turretSubsystem.getMotor(), 1);
            mgr.addInstrument(RobotContainer.preshooterSubsystem.getMotor(), 1);

            // BASS/ACCOMPANIMENT (Track 2) - Assign to drive motors and remaining
            mgr.addInstrument(RobotContainer.swerveSubsystem.getModule(0).getDriveMotor(), 2);
            mgr.addInstrument(RobotContainer.swerveSubsystem.getModule(1).getDriveMotor(), 2);
            mgr.addInstrument(RobotContainer.swerveSubsystem.getModule(2).getDriveMotor(), 2);
            mgr.addInstrument(RobotContainer.swerveSubsystem.getModule(3).getDriveMotor(), 2);
            mgr.addInstrument(RobotContainer.conveyerSubsystem.getMotor(), 2);
            mgr.addInstrument(RobotContainer.intakeRollerSubsystem.getMotor1(), 2);
            mgr.addInstrument(RobotContainer.intakeRollerSubsystem.getMotor2(), 2);
            mgr.addInstrument(RobotContainer.intakeAgitatorSubsystem.getMotor(), 2);
        }),

        d("Call Me Maybe", "CallMeMaybe.chrp", mgr -> {
            mgr.addInstrument(RobotContainer.shooterSubsystem.getMotor1(), 0);
            mgr.addInstrument(RobotContainer.shooterSubsystem.getMotor2(), 0);
            mgr.addInstrument(RobotContainer.swerveSubsystem.getModule(0).getSteerMotor(), 0);
            mgr.addInstrument(RobotContainer.swerveSubsystem.getModule(1).getSteerMotor(), 0);
            mgr.addInstrument(RobotContainer.swerveSubsystem.getModule(2).getSteerMotor(), 0);
            mgr.addInstrument(RobotContainer.swerveSubsystem.getModule(3).getSteerMotor(), 0);
            mgr.addInstrument(RobotContainer.intakeShoulderSubsystem.getMotor(), 0);
            mgr.addInstrument(RobotContainer.shooterHoodSubsystem.getMotor(), 0);
            mgr.addInstrument(RobotContainer.turretSubsystem.getMotor(), 0);
            mgr.addInstrument(RobotContainer.preshooterSubsystem.getMotor(), 0);
            mgr.addInstrument(RobotContainer.swerveSubsystem.getModule(0).getDriveMotor(), 0);
            mgr.addInstrument(RobotContainer.swerveSubsystem.getModule(1).getDriveMotor(), 0);
            mgr.addInstrument(RobotContainer.swerveSubsystem.getModule(2).getDriveMotor(), 0);
            mgr.addInstrument(RobotContainer.swerveSubsystem.getModule(3).getDriveMotor(), 0);
            mgr.addInstrument(RobotContainer.conveyerSubsystem.getMotor(), 0);
            mgr.addInstrument(RobotContainer.intakeRollerSubsystem.getMotor1(), 0);
            mgr.addInstrument(RobotContainer.intakeRollerSubsystem.getMotor2(), 0);
            mgr.addInstrument(RobotContainer.intakeAgitatorSubsystem.getMotor(), 0);
        });

        public final String displayName;
        public final String filename;
        public final SongConfig config;

        OrchestraSong(String displayName, String filename, SongConfig config) {
            this.displayName = displayName;
            this.filename = filename;
            this.config = config;
        }
    }

    /**
     * Load a song and reconfigure all motor-to-track assignments for it.
     * Clears previous assignments first.
     */
    public void loadSong(OrchestraSong song) {
        if (song == null)
            return;
        m_orchestra.stop();
        m_orchestra.clearInstruments();

        // Apply the song's specific motor/track layout
        song.config.configure(this);

        // Now load the file
        var status = m_orchestra.loadMusic(song.filename);
        m_isLoaded = status.isOK();
    }

    /**
     * Add a motor to a specific track. AudioConfigs already applied in constructor.
     * Safe to call with null (skipped).
     */
    public void addInstrument(TalonFX motor, int track) {
        if (motor == null)
            return;
        m_orchestra.addInstrument(motor, track);
    }

    public void play() {
        if (m_isLoaded)
            m_orchestra.play();
    }

    public void pause() {
        m_orchestra.pause();
    }

    public void stop() {
        m_orchestra.stop();
    }

    public boolean isPlaying() {
        return m_orchestra.isPlaying();
    }

    public void reload(OrchestraSong song) {
        m_orchestra.stop();
        loadSong(song);
    }
}