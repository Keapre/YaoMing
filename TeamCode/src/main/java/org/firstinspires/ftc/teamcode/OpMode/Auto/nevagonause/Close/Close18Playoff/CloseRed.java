package org.firstinspires.ftc.teamcode.OpMode.Auto.nevagonause.Close.Close18Playoff;

import org.firstinspires.ftc.teamcode.Util.Globals.Alliance;
import org.firstinspires.ftc.teamcode.Util.Info;

//@Autonomous(name = "Auto Red 18 Playoff")
public class CloseRed extends Close18 {
    @Override
    public void init() {
        Info.alliance = Alliance.RED;

        super.init();
    }

}