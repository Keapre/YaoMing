package org.firstinspires.ftc.teamcode.OpMode.Auto.Far.Scoop;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Util.Globals.Alliance;
import org.firstinspires.ftc.teamcode.Util.Info;

//@Autonomous(name = "Far Blue Scoop", group = "Far")
public class FarBlue extends Far {
    @Override
    public void init() {
        Info.alliance = Alliance.BLUE;
        super.init();
    }
}
