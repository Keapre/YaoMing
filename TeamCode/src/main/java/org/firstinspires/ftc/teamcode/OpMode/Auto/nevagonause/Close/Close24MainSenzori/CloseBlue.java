package org.firstinspires.ftc.teamcode.OpMode.Auto.nevagonause.Close.Close24MainSenzori;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Util.Globals.Alliance;
import org.firstinspires.ftc.teamcode.Util.Info;

@Autonomous(name = "Auto Blue 24 (prioritieze flow)")
public class CloseBlue extends Close24MainSensors {
    @Override
    public void init() {
        Info.alliance = Alliance.BLUE;

        super.init();
    }

}