package org.firstinspires.ftc.teamcode.OpMode.Auto.Far;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Util.Globals.Alliance;
import org.firstinspires.ftc.teamcode.Util.Info;

@Autonomous(name = "Far Red", group = "Far")
public class FarRed extends Far {
    @Override
    public void init() {
        Info.alliance = Alliance.RED;
        super.init();
    }
}
