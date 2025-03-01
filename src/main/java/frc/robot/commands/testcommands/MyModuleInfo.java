package frc.robot.commands.testcommands;
import swervelib.SwerveModule;

class MyModuleInfo {
    public int index;
    public SwerveModule obj;

    public MyModuleInfo(int moduleIndex, SwerveModule moduleObj) {
        this.index = moduleIndex;
        this.obj = moduleObj;
    }
}
