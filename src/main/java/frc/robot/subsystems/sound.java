package frc.robot.subsystems;

import java.io.File;

import javax.sound.sampled.AudioInputStream;
import javax.sound.sampled.AudioSystem;
import javax.sound.sampled.Clip;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class sound extends SubsystemBase {

    public sound() {
        // Nothing needed in constructor — binding happens in RobotContainer
    }

    public void playSound(String filePath) {
        try {
            AudioInputStream audio = AudioSystem.getAudioInputStream(new File(filePath));
            Clip clip = AudioSystem.getClip();
            clip.open(audio);
            clip.start();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}