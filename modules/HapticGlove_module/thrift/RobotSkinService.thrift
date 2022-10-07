
/**
 * Definition of the RobotSkin RPC service
 */
service RobotSkinService {

    /**
     * Percentage dedicated to absolute skin data for providing the vibrotactile feedback.
     * the value is between [0,1]
     * 0 : 0% absolute skin value for the vibrotactile feedback; 100% derivative skin value for the vibrotactile feedback
     * 1 : 100% absolute skin value for the vibrotactile feedback; 0% derivative skin value for the vibrotactile feedback
     *
     * @return true if the procedure was successful, false otherwise
     */
    bool setAbsoluteSkinValuePercentage(1: double value);

    /**
     * Smoothing gain for computing the smoothed skin derivative.
     * the value is between [0,1]
     * 1 : no smoothing
     *
     * @return true if the procedure was successful, false otherwise
     */
    bool setSkinDerivativeSmoothingGain(1: double value);

    /**
     * Feedback gain applied to the contact value for the given finger.
     *
     * @return true if the procedure was successful, false otherwise
     */
    bool setContactFeedbackGain(1: i32 finger, 2: double value);

    /**
     * Feedback gain applied to the contact value for all the fingers.
     *
     * @return true if the procedure was successful, false otherwise
     */
    bool setContactFeedbackGainAll(1: double value);

    /**
     * Threshold applied to the contact value for the given finger.
     *
     * @return true if the procedure was successful, false otherwise
     */
    bool setContactThreshold(1: i32 finger, 2: double value);

    /**
     * Threshold applied to the contact value for all the fingers.
     *
     * @return true if the procedure was successful, false otherwise
     */
    bool setContactThresholdAll(1: double value);

    /**
     * Standard Deviation multiplier for the threshold 
     * applied to the contact value for the given finger.
     *
     * @return true if the procedure was successful, false otherwise
     */
    bool setContactThresholdMultiplier(1: i32 finger, 2: double value);

    /**
     * Standard Deviation multiplier for the threshold 
     * applied to the contact value for all the fingers.
     *
     * @return true if the procedure was successful, false otherwise
     */
    bool setContactThresholdMultiplierAll(1: double value);

    /**
     * Feedback gain applied to the contact derivative value for the given finger.
     *
     * @return true if the procedure was successful, false otherwise
     */
    bool setDerivativeFeedbackGain(1: i32 finger, 2: double value);

    /**
     * Feedback gain applied to the contact derivative value for all the fingers.
     *
     * @return true if the procedure was successful, false otherwise
     */
    bool setDerivativeFeedbackGainAll(1: double value);

    /**
     * Threshold applied to the contact derivative value for the given finger.
     *
     * @return true if the procedure was successful, false otherwise
     */
    bool setDerivativeThreshold(1: i32 finger, 2: double value);

    /**
     * Threshold applied to the contact derivative value for all the fingers.
     *
     * @return true if the procedure was successful, false otherwise
     */
    bool setDerivativeThresholdAll(1: double value);

    /**
     * Standard Deviation multiplier for the threshold 
     * applied to the contact derivative value for the given finger.
     *
     * @return true if the procedure was successful, false otherwise
     */
    bool setDerivativeThresholdMultiplier(1: i32 finger, 2: double value);

    /**
     * Standard Deviation multiplier for the threshold 
     * applied to the contact derivative value for all the fingers.
     *
     * @return true if the procedure was successful, false otherwise
     */
    bool setDerivativeThresholdMultiplierAll(1: double value);

}
