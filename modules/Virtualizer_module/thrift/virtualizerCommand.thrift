// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

service VirtualizerCommands
{
    /**
     * Reset the player orientation
     */
    oneway void resetPlayerOrientation();

    /**
     * Reset the player height
     */
    oneway void resetPlayerHeight();

     /**
     * Reset the player still angle
     */
    oneway void forceStillAngle();
}
