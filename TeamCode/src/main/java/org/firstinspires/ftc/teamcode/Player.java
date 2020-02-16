package org.firstinspires.ftc.teamcode;

import android.media.MediaPlayer;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.HashMap;

class Player {
    boolean MusicPaused;

    private HardwareMap hardwareMap;

    private HashMap<String, Integer> Songs;

    private MediaPlayer MP;

    Player(HardwareMap hw) {
        Songs = new HashMap<>();
        Songs.put("ucanttouchthis",R.raw.ucanttouchthis);
        Songs.put("megalovania",R.raw.megalovania);
        Songs.put("coconutmall",R.raw.coconutmall);

        hardwareMap = hw;

        MusicPaused = true;
    }

    void setSong(String name) {
        MP = MediaPlayer.create(hardwareMap.appContext,Songs.get(name));
    }

    void start() {
        MP.start();
        MusicPaused = false;
    }

    void stop() {
        MP.stop();
        MusicPaused = true;
    }
}
