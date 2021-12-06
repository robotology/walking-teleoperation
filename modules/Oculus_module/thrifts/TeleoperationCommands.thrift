/**
 * @file TeleoperationCommands.thrift
 * @authors Kourosh Darvish <kourosh.darvish@iit.it>
 * @copyright 2021 Artificial and Mechanical Intelligence Lab - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2021
 */

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
