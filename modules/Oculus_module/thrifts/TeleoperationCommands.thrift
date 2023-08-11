// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

service TeleoperationCommands
{
    /**
     * Call this method to prepare the Oculus Module for Teleoperation.
     * @return true/false in case of success/failure;
     */
    bool prepareTeleoperation();

    /**
     * Run the entire Oculus Module for Teleoperation.
     * @return true/false in case of success/failure;
     */
    bool runTeleoperation();

}
