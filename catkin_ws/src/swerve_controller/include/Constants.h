#pragma once

#include "units/units.h"

namespace tigertronics {
    namespace constants {
        //PHYSICAL DIMENSIONS
        static constexpr units::meter_t drivebaseLength = 24_in;
        static constexpr units::meter_t drivebaseWidth = 23_in;
        static constexpr units::meter_t driveWheelRadius = 2.5_in / 2;


        //ENCODER INFO
        static constexpr int ctreEncoderTicksPerRev = 4096;

        //SWERVE TURNING MOTOR CONFIG
        static constexpr double swerveTurningkP = 12;
        static constexpr double swerveTurningkI = 0;
        static constexpr double swerveTurningkD = 120;
        static constexpr units::radians_per_second_t swerveTurningMaxVel = 3600_rad_per_s;
        //im too lazy to define radial acceleration right now so this is in ticks/100ms/s
        static constexpr double swerveTurningMaxAccel = 1500;
        static constexpr int swerveTurningErrorAllowance = 5;

        //SWERVE DRIVE MOTOR CONFIG
        static constexpr double swerveDrivekF = 0.000015;
        static constexpr double swerveDrivekP = 0.1;
        static constexpr double swerveDrivekI = 0;
        static constexpr double swerveDrivekD = 0;        
    }

    namespace ports {
        //SWERVE PORTS
        static constexpr int swerveFLTurn = 6;
        static constexpr int swerveFRTurn = 7;
        static constexpr int swerveBLTurn = 8;
        static constexpr int swerveBRTurn = 9;
        static constexpr int swerveFLDrive = 2;
        static constexpr int swerveFRDrive = 3;
        static constexpr int swerveBLDrive = 4;
        static constexpr int swerveBRDrive = 5;
    }
}
