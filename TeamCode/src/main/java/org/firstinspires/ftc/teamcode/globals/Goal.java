package org.firstinspires.ftc.teamcode.globals;

/** Which HIVE goal shooter aim at. HIVE tip = switch. */
public enum Goal {
    A,
    B;

    /** Other goal. */
    public Goal other() {
        return this == A ? B : A;
    }
}
