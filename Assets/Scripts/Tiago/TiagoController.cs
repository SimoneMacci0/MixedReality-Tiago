using System.Collections;
using UnityEngine;

using RosMessageTypes.TiagoUnity;
using System.Linq;

using Point = RosMessageTypes.Geometry.PointMsg;
using V3 = RosMessageTypes.Geometry.Vector3Msg;

public class TiagoController : MonoBehaviour
{
    // Robot object
    private GameObject tiago;
    private GameObject base_link;

    // Arrays of ArticulationBodies for controlling arms
    private ArticulationBody[] leftArmArticulationBodies;
    private ArticulationBody[] leftGripper;
    private ArticulationBody[] rightArmArticulationBodies;
    private ArticulationBody[] rightGripper;
    private ArticulationBody torsoArticulationBody;

    // Variables for trajectory execution
    private string lastActionPlanned = "";
    private string lastArmActionPlanned = "";
    private ActionServiceResponse lastActionResponse;
    private int steps;
    private float jointAssignmentWait = 0.01f;
    public bool busy = false;

    public void Initialize(GameObject robot, GameObject baseLink, int steps)
    {
        tiago = robot;
        base_link = baseLink;

        this.steps = steps;

        // Get reference to robot and go to home position
        GetRobotReference();
    }

    private void GetRobotReference()
    {
        string base_link = "base_footprint/base_link";
        string torso_link = base_link + "/torso_fixed_link/torso_lift_link";
        torsoArticulationBody = tiago.transform.Find(torso_link).GetComponent<ArticulationBody>();

        leftArmArticulationBodies = new ArticulationBody[7];
        string side = "left";
        string arm_1_link = torso_link + "/arm_" + side  + "_1_link";
        leftArmArticulationBodies[0] = tiago.transform.Find(arm_1_link).GetComponent<ArticulationBody>();

        string arm_2_link = arm_1_link + "/arm_" + side + "_2_link";
        leftArmArticulationBodies[1] = tiago.transform.Find(arm_2_link).GetComponent<ArticulationBody>();

        string arm_3_link = arm_2_link + "/arm_" + side + "_3_link";
        leftArmArticulationBodies[2] = tiago.transform.Find(arm_3_link).GetComponent<ArticulationBody>();

        string arm_4_link = arm_3_link + "/arm_" + side + "_4_link";
        leftArmArticulationBodies[3] = tiago.transform.Find(arm_4_link).GetComponent<ArticulationBody>();

        string arm_5_link = arm_4_link + "/arm_" + side + "_5_link";
        leftArmArticulationBodies[4] = tiago.transform.Find(arm_5_link).GetComponent<ArticulationBody>();

        string arm_6_link = arm_5_link + "/arm_" + side + "_6_link";
        leftArmArticulationBodies[5] = tiago.transform.Find(arm_6_link).GetComponent<ArticulationBody>();

        string arm_7_link = arm_6_link + "/arm_" + side + "_7_link";
        leftArmArticulationBodies[6] = tiago.transform.Find(arm_7_link).GetComponent<ArticulationBody>();

        string gripper_left_link = arm_7_link + "/arm_" + side + "_tool_link/gripper_" + side + "_link";
        string left_gripper_left_finger = gripper_left_link + "/gripper_" + side + "_left_finger_link";
        string left_gripper_right_finger = gripper_left_link + "/gripper_" + side + "_right_finger_link";

        leftGripper = new ArticulationBody[2];
        leftGripper[0] = tiago.transform.Find(left_gripper_left_finger).GetComponent<ArticulationBody>();
        leftGripper[1] = tiago.transform.Find(left_gripper_right_finger).GetComponent<ArticulationBody>();

        rightArmArticulationBodies = new ArticulationBody[7];
        side = "right";
        arm_1_link = torso_link + "/arm_" + side + "_1_link";
        rightArmArticulationBodies[0] = tiago.transform.Find(arm_1_link).GetComponent<ArticulationBody>();

        arm_2_link = arm_1_link + "/arm_" + side + "_2_link";
        rightArmArticulationBodies[1] = tiago.transform.Find(arm_2_link).GetComponent<ArticulationBody>();

        arm_3_link = arm_2_link + "/arm_" + side + "_3_link";
        rightArmArticulationBodies[2] = tiago.transform.Find(arm_3_link).GetComponent<ArticulationBody>();

        arm_4_link = arm_3_link + "/arm_" + side + "_4_link";
        rightArmArticulationBodies[3] = tiago.transform.Find(arm_4_link).GetComponent<ArticulationBody>();

        arm_5_link = arm_4_link + "/arm_" + side + "_5_link";
        rightArmArticulationBodies[4] = tiago.transform.Find(arm_5_link).GetComponent<ArticulationBody>();

        arm_6_link = arm_5_link + "/arm_" + side + "_6_link";
        rightArmArticulationBodies[5] = tiago.transform.Find(arm_6_link).GetComponent<ArticulationBody>();

        arm_7_link = arm_6_link + "/arm_" + side + "_7_link";
        rightArmArticulationBodies[6] = tiago.transform.Find(arm_7_link).GetComponent<ArticulationBody>();

        string gripper_right_link = arm_7_link + "/arm_" + side + "_tool_link/wrist_" + side + "_ft_link/wrist_" + side + "_ft_tool_link/gripper_" + side + "_link";
        string right_gripper_left_finger = gripper_right_link + "/gripper_" + side + "_left_finger_link";
        string right_gripper_right_finger = gripper_right_link + "/gripper_" + side + "_right_finger_link";

        rightGripper = new ArticulationBody[2];
        rightGripper[0] = tiago.transform.Find(right_gripper_left_finger).GetComponent<ArticulationBody>();
        rightGripper[1] = tiago.transform.Find(right_gripper_right_finger).GetComponent<ArticulationBody>();

        MoveToRestPosition();

    }

    // Routine to teleport the robot's base_link to a new gobal pose
    public void TeleportRobot(Vector3 position, Quaternion rotation)
    {
        base_link.GetComponent<ArticulationBody>().TeleportRoot(position, rotation);
    }

    // Routine to teleport the robot's base_link to a relative pose wrt the current one
    public void TeleportRobotPosition(Vector3 position)
    {
        base_link.GetComponent<ArticulationBody>().TeleportRoot(position, base_link.transform.rotation);
    }

    // Functions to close and open the robot's gripper
    public void CloseGripper(ArticulationBody[] gripper)
    {
        var leftDrive = gripper[0].xDrive;
        var rightDrive = gripper[1].xDrive;

        leftDrive.target = 0.02f;
        rightDrive.target = 0.02f;

        gripper[0].xDrive = leftDrive;
        gripper[1].xDrive = rightDrive;
    }

    public void OpenGripper(ArticulationBody[] gripper)
    {
        var leftDrive = gripper[0].xDrive;
        var rightDrive = gripper[1].xDrive;

        // Hardcoded values for open gripper (extracted from Tiago's documentation)
        leftDrive.target = 0.045f;
        rightDrive.target = 0.045f;

        gripper[0].xDrive = leftDrive;
        gripper[1].xDrive = rightDrive;
    }

    /* ---------------------
     * Utility methods to drive the robot (arms and torso) to their initial rest configurations when the application starts
     * Can be set up in order to copy the real robot's configuration if connected
     */

    public void MoveToRestPosition()
    {
        StartCoroutine(MoveArmToRestRoutine(leftArmArticulationBodies, leftGripper));
        StartCoroutine(MoveArmToRestRoutine(rightArmArticulationBodies, rightGripper));
        StartCoroutine(MoveTorsoToRestRoutine(0.01f));
    }
    
    private IEnumerator MoveArmToRestRoutine(ArticulationBody[] armArticulationBody, ArticulationBody[] gripperArticulationBody)
    {
        float[] target = { -1.1f, 1.469f, 2.721f, 1.717f, -1.567f, 1.385f, 0.0f };
        float[] lastJointState = { 0, 0, 0, 0, 0, 0, 0 };
        var steps = 100;
        for (int i = 0; i < armArticulationBody.Length; i++)
        {
            target[i] = Mathf.Rad2Deg * (float)target[i];
        }
        for (int i = 0; i <= steps; i++)
        {
            for (int joint = 0; joint < armArticulationBody.Length; joint++)
            {
                var joint1XDrive = armArticulationBody[joint].xDrive;
                joint1XDrive.target = lastJointState[joint] + (target[joint] - lastJointState[joint]) * (float)(1.0f / steps) * (float)i;
                armArticulationBody[joint].xDrive = joint1XDrive;
            }

            yield return new WaitForSeconds(jointAssignmentWait);
        }
        OpenGripper(gripperArticulationBody);
    }

    private IEnumerator MoveTorsoToRestRoutine(float target)
    {
        var steps = 1;
        var torsoDrive = torsoArticulationBody.xDrive;
        var lastTorsoValue = torsoDrive.target;
        for (int i = 0; i <= steps; i++)
        {            
            torsoDrive.target = lastTorsoValue + (target - lastTorsoValue) * (float)(1.0f / steps) * (float)i;
            torsoArticulationBody.xDrive = torsoDrive;
            lastTorsoValue = torsoDrive.target;
            yield return new WaitForSeconds(jointAssignmentWait);
        }
    }

    // ---------------------

    /* Utility method to extract the joint configuration as an array of double from a given array of ArticulationObject */
    double[] GetCurrentJointStates(ArticulationBody[] arm)
    {
        var jointAngles = new double[8];
        jointAngles[0] = torsoArticulationBody.xDrive.target;
        for (int i = 1; i < 8; i++)
        {
            // Convert from degrees to radiants as ROS expects rads for planning
            jointAngles[i] = Mathf.Deg2Rad * arm[i-1].xDrive.target;
        }
        
        return jointAngles;
    }

    /* Utility method to contruct a planning request object accepted by the ROS service
     * Params:
     *      - arm: the arm for which a plan is sought;
     *      - actionType: the type of action for which a plan is sought (pick_place, handover ...)
     *      - pickPos: a 3D point corresponding to the position of the object to pick (for pick_place actions) / handover position;
     *      - placePos: a 3D point defining the location to place the object / useless for handover actions
     *      - graspDir: a 3D vector defining the direction of approach and grasping for the robot. Two possibilities are currently contemplated:
     *              1. Frontal (forward) grasp = (1,0,0);
     *              2. Vertical (descending) grasp = (0,0,-1).
     */
    public ActionServiceRequest PlanningRequest(string arm, string actionType, Point pickPos, Point placePos, V3 graspDir)
    {
        var request = new ActionServiceRequest();

        request.action_type = actionType;
        request.pick_pos = pickPos;
        if(actionType != "handover")
        {
            request.place_pos = placePos;
        }
        var bodies = arm == "left" ? leftArmArticulationBodies : rightArmArticulationBodies;
        // Get current joint configuration for the given arm from the articulation bodies objects
        request.joint_angles = GetCurrentJointStates(bodies);
        request.grasp_direction = graspDir;

        // Store the current information of arm and action type for later holographic rendering
        lastArmActionPlanned = arm;
        lastActionPlanned = actionType;

        return request;
    }

    /* Public callback method accessible from external Interface, called when the planning service returns
     * If an action is returned, instantiate the coroutine to render it as holographic animation 
     */
    public void PlanningServiceResponse(ActionServiceResponse response)
    {
        // If the planner actually found a solution...
        if (response.planning_result)
        {
            Debug.Log("Trajectory returned.");
            lastActionResponse = response;
            StartCoroutine(RenderFullHolographicAction(response));
        }
        else 
            Debug.Log("No trajectory returned from MoveIt.");
    }

    /* Routine to holo-render all trajectories in a given action.
     * Calls the RenderHolographicTrajectory method iteratively on all sub-trajectories compisosing the global action, depending
     * on which type of action is being executed (handover, pick-place ...)
     */
    private IEnumerator RenderFullHolographicAction(ActionServiceResponse response)
    {
        var armArticulationBodies = lastArmActionPlanned == "left" ? leftArmArticulationBodies : rightArmArticulationBodies;

        // Pre-grasp trajectory is always rendered
        yield return RenderHolographicTrajectory(response.planned_action.pre_grasp_trajectory, armArticulationBodies, this.steps);
        // Render the following only if pick and place action
        if(lastActionPlanned != "handover")
        {
            yield return RenderHolographicTrajectory(response.planned_action.grasp_trajectory, armArticulationBodies, steps);
            yield return RenderHolographicTrajectory(response.planned_action.move_trajectory, armArticulationBodies, steps);
            yield return RenderHolographicTrajectory(response.planned_action.place_trajectory, armArticulationBodies, steps);
            yield return RenderHolographicTrajectory(response.planned_action.return_trajectory, armArticulationBodies, Mathf.RoundToInt(steps / 2));
        }
        // Action complete, controller no more busy
        busy = false;
    }

    /* Internal routine for rendering a single trajectory.
     * Params:
     *      - The trajectory to render;
     *      - The array of articulation bodies that compose the kinematic chain;
     *      - An integer number of steps, which defines the "granularity" of the holographic animation.
     */
    private IEnumerator RenderHolographicTrajectory(RosMessageTypes.Moveit.RobotTrajectoryMsg trajectory, ArticulationBody[] articulationBodies, int steps)
    {
        // Initial joint configuration
        var lastJointState = GetCurrentJointStates(articulationBodies).Select(r => (double)r * Mathf.Rad2Deg).ToArray();

        // For each joint configuration in the given trajectory...
        for (int jointConfigIndex = 0; jointConfigIndex < trajectory.joint_trajectory.points.Length; jointConfigIndex++)
        {
            // Get next joint config and convert from radiant to degree angles (for Unity)
            var jointPositions = trajectory.joint_trajectory.points[jointConfigIndex].positions;
            double[] result = jointPositions.Select(r => (double)r * Mathf.Rad2Deg).ToArray();

            // Get reference to torso drive to control torso al well according to the trajectory being rendered
            var torsoDrive = torsoArticulationBody.xDrive;
            var lastTorsoValue = torsoDrive.target;

            // Steps drive the smoothness of the animation
            for (int i = 0; i <= steps; i++)
            {
                // For each joint in the kinematic chain...
                for (int joint = 0; joint < articulationBodies.Length; joint++)
                {
                    // compute next joint position based on next joint configuration extracted from the trajectory
                    var joint1XDrive = articulationBodies[joint].xDrive;
                    joint1XDrive.target = (float)(lastJointState[joint + 1] + (result[joint + 1] - lastJointState[joint + 1]) * (1.0f / steps) * i);
                    articulationBodies[joint].xDrive = joint1XDrive;
                }

                // Same for torso to achieve next position
                torsoDrive.target = lastTorsoValue + ((float)jointPositions[0] - lastTorsoValue) * (float)(1.0f / steps) * (float)i;
                torsoArticulationBody.xDrive = torsoDrive;

                // Wait few milliseconds to ensure updates to all joints are received
                yield return new WaitForSeconds(jointAssignmentWait);
            }

            lastTorsoValue = torsoDrive.target;
            lastJointState = result;
        }
    }

    public void CompleteHandover()
    {
        var gripper = lastArmActionPlanned == "left" ? leftGripper : rightGripper;
        var articulationBody = lastArmActionPlanned == "left" ? leftArmArticulationBodies : rightArmArticulationBodies;
        CloseGripper(gripper);
        StartCoroutine(RenderHolographicTrajectory(lastActionResponse.planned_action.return_trajectory, articulationBody, steps));

    }
}
