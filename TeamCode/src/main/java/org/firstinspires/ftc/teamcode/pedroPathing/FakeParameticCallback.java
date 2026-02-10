package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;

public class FakeParameticCallback {
    private final double t;
    private final Runnable runnable;
    private final Follower follower;
    public FakeParameticCallback(double t, Runnable runnable, Follower follower){
        this.t = t;
        this.runnable = runnable;
        this.follower = follower;
    }

    public boolean check(){
        if(follower.atParametricEnd() || follower.getPathCompletion() >= t){
            runnable.run();
            return true;
        }
        return false;
    }
}
