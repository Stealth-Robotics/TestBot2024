package org.firstinspires.ftc.teamcode.common;

import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.TeleStealth;
import org.firstinspires.ftc.teamcode.subsystems.FollowerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimeLightSubsystem;
import org.stealthrobotics.library.Alliance;

public class AutoBase extends TeleStealth {
    private GamepadEx driver;
    private GamepadEx operator;
    private FollowerSubsystem fss;
    private LimeLightSubsystem llss;

    private Boolean hasOperator;
    private Alliance alliance;

    public AutoBase(Alliance alliance, Boolean hasOperator){
        this.hasOperator = hasOperator;
        this.alliance = alliance;
    }

    @Override
    public void whileWaitingToStart(){


    }

    public void initialize(){
    }
}
