// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

/**
 * Definition of the HapticGlove teleoperation RPC service
 */
service HapticGloveTeleoperationService {

    /**
     * Enable/Disable motion of robot.
     * true : robot move enabled.
     * false : robot move disabled.
     *
     * @return true if the procedure was successful, false otherwise
     */
    bool enableMoveRobot(1: bool value);

}
