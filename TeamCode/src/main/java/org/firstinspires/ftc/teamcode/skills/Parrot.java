package org.firstinspires.ftc.teamcode.skills;

import android.media.AudioManager;
import android.media.MediaPlayer;
import android.media.SoundPool;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.R;

import java.util.ArrayList;


public class Parrot {
    HardwareMap hardwareMap = null;
    Telemetry telemetry;
//    SoundPool s = new SoundPool(1, AudioManager.STREAM_MUSIC, 0);
    boolean sequence = false;
    ArrayList<Integer> sequenceList = new ArrayList<>();

    public Parrot(HardwareMap hardwareMap, Telemetry t){
        this.hardwareMap = hardwareMap;
        this.telemetry = t;

//        s.setOnLoadCompleteListener(new SoundPool.OnLoadCompleteListener() {
//            @Override
//            public void onLoadComplete(SoundPool soundPool, int sampleId, int status) {
//                if (sequence){
//                    if (status == 0) {
//                        //play first piece
//                        s.play(sampleId, 1, 1, 1, 0, 1);
//                        if (sequenceList.size() > 0) {
//                            //load middle
//                            int resourceID = sequenceList.get(sequenceList.size() - 1);
//                            sequenceList.clear();
//                            play(resourceID);
//                        }
//                        else{
//                            CommentLast();
//                            sequence = false;
//                        }
//                    }
//                }
//                else {
//                    if (status == 0) {
//                        s.play(sampleId, 1, 1, 1, 0, 1);
//                        telemetry.addData("Sound", "Playing # %d", sampleId);
//                        telemetry.update();
//                    } else {
//                        telemetry.addData("Sound", "Failed with status %d", status);
//                        telemetry.update();
//                    }
//                }
//            }
//        });
    }

    public void playGold(){
        play(R.raw.gold, 0);
    }

    public void playLeft(){
        play(R.raw.left, 0);
    }

    public void playRight(){
        play(R.raw.right, 0);
    }

    public void playCenter(){
        play(R.raw.center, 0);
    }

    public void playISeeGold(){
        play(R.raw.seegold, 0);
    }

    public void playClever(){
        play(R.raw.clever, 0);
    }

    public void playColorBlind(){
        play(R.raw.colorblind, 0);
    }

    public void playHumble(){
        play(R.raw.humble, 0);
    }

    public void playLoveGold(){
        play(R.raw.lovegold, 0);
    }

    public void playMyPrecious(){
        play(R.raw.myprecious, 0);
    }

    public void playToldYa(){
        play(R.raw.toldya, 0);
    }

    public void CommentFirst(){
        int index = random(1, 3);
        switch (index){
            case 1:
                playMyPrecious();
                break;
            case 2:
                playISeeGold();
                break;
            case 3:
                playLoveGold();
                break;
        }
    }

    public int getCommentID(){
        int index = random(1, 4);
        int rId = 0;
        switch (index){
            case 1:
                rId = R.raw.toldya;
                break;
            case 2:
                rId = R.raw.clever;
                break;
            case 3:
                rId = R.raw.colorblind;
                break;
            case 4:
                rId = R.raw.humble;
                break;
        }
        return rId;
    }

    public void playSequence(GoldPosition goldPosition){
        int commentID = getCommentID();
        int id = 0;
        if (goldPosition == GoldPosition.Left){
            id = R.raw.left;
        } else if (goldPosition == GoldPosition.Center){
            id = R.raw.center;
        } else if (goldPosition == GoldPosition.Right){
            id = R.raw.right;
        }
        play(id, commentID);
    }

    protected void play(int resourceID, final  int next){
        MediaPlayer player = MediaPlayer.create(hardwareMap.appContext, resourceID);
        player.setOnCompletionListener(new MediaPlayer.OnCompletionListener() {
            @Override
            public void onCompletion(MediaPlayer mediaPlayer) {
                if (next > 0){
                    play(next, 0);
                }
            }
        });
        player.start();
    }

    int random(int min, int max)
    {
        int range = (max - min) + 1;
        return (int)(Math.random() * range) + min;
    }
}
