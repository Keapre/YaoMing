package org.firstinspires.ftc.teamcode.OpMode.Auto.nevagonause.Close.Close27MainSenzori;

import org.firstinspires.ftc.teamcode.Util.Globals.Alliance;
import org.firstinspires.ftc.teamcode.Util.Info;

//@Autonomous(name = "Auto Blue 27 (main senzori)")
public class CloseBlue extends Close27MainSensors {
    @Override
    public void init() {
        Info.alliance = Alliance.BLUE;

        super.init();
    }

}