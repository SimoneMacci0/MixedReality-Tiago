using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using RosMessageTypes.TiagoUnity;
using Twist = RosMessageTypes.Geometry.TwistMsg;
using NavSrvResponse = RosMessageTypes.Nav.GetPlanResponse;
using PoseStamped = RosMessageTypes.Geometry.PoseStampedMsg;
using System.Linq;

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

    // Wheels articulation bodies
    private ArticulationBody[] wheels;
    private ArticulationBody[] casterWheels;

    // Parameters for mobile base control
    public bool receivedFirstCommand = false;
    private float maxLinearSpeed = 1.5f;
    private float maxRotationalSpeed = 1;//
    private float wheelRadius = 0.135f; 
    private float trackWidth = 0.4f; 
    private float forceLimit = 10;
    private float stiffness = 0.0f;
    private float damping = 10;

    // Variables for trajectory execution
    private int steps;
    private float jointAssignmentWait = 0.01f;
    private enum Poses
    {
        PreGrasp,
        Grasp,
        PickUp,
        Move,
        Place,
        Return
    };

    // Start is called before the first frame update
    public void Initialize(GameObject robot, GameObject baseLink, float maxLinSpeed, float maxRotSpeed, int steps)
    {
        tiago = robot;
        base_link = baseLink;
        maxRotationalSpeed = maxLinSpeed;
        maxRotationalSpeed = maxRotSpeed;
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

        // Get reference to driving wheels and change articulation parameters to ensure smooth motion
        wheels = new ArticulationBody[2];
        string left_wheel = base_link + "/suspension_left_link/wheel_left_link";
        string right_wheel = base_link + "/suspension_right_link/wheel_right_link";
        wheels[0] = tiago.transform.Find(left_wheel).GetComponent<ArticulationBody>();
        wheels[1] = tiago.transform.Find(right_wheel).GetComponent<ArticulationBody>();
        foreach(ArticulationBody wheel in wheels)
        {
            SetJointDriveParameters(wheel);
        }

        // Get reference to caster wheels and change articulation parameters to ensure free rotation
        casterWheels = new ArticulationBody[4];
        string caster_back_left = base_link + "/caster_back_left_1_link/caster_back_left_2_link";
        string caster_back_right = base_link + "/caster_back_right_1_link/caster_back_right_2_link";
        string caster_front_left = base_link + "/caster_front_left_1_link/caster_front_left_2_link";
        string caster_front_right = base_link + "/caster_front_right_1_link/caster_front_right_2_link";
        casterWheels[0] = tiago.transform.Find(caster_back_left).GetComponent<ArticulationBody>();
        casterWheels[1] = tiago.transform.Find(caster_back_right).GetComponent<ArticulationBody>();
        casterWheels[2] = tiago.transform.Find(caster_front_left).GetComponent<ArticulationBody>();
        casterWheels[3] = tiago.transform.Find(caster_front_right).GetComponent<ArticulationBody>();
        foreach(ArticulationBody casterWheel in casterWheels)
        {
            SetJointDriveParameters(casterWheel);
        }

        MoveToRestPosition();

    }

    // Set parameters of joint drives for mobile base
    private void SetJointDriveParameters(ArticulationBody joint)
    {
        ArticulationDrive drive = joint.xDrive;
        drive.stiffness = stiffness;
        drive.forceLimit = forceLimit;
        drive.damping = damping;
        joint.xDrive = drive;
    }

    // Apply twist vector to mobile base
    public void MobileBaseCommand(Twist msg)
    {
        if(!receivedFirstCommand)
        {
            receivedFirstCommand = true;
        }

        var speed = 2.0f * (float)msg.linear.x;
        var rotSpeed = 5.0f * (float)msg.angular.z;

        if (speed > maxLinearSpeed)
        {
            speed = maxLinearSpeed;
        }
        if (rotSpeed > maxRotationalSpeed)
        {
            rotSpeed = maxRotationalSpeed;
        }

        float wheelSpeedRight = (float)(Mathf.Rad2Deg * ((2 * speed + rotSpeed * trackWidth) / (2 * wheelRadius)));
        float wheelSpeedLeft = (float)(Mathf.Rad2Deg * ((2 * speed - rotSpeed * trackWidth) / (2 * wheelRadius)));

        SetWheelSpeed(wheels[0], wheelSpeedLeft);
        SetWheelSpeed(wheels[1], wheelSpeedRight);
    }

    private void SetWheelSpeed(ArticulationBody joint, float wheelSpeed)
    {
        ArticulationDrive drive = joint.xDrive;
        drive.targetVelocity = wheelSpeed;
        joint.xDrive = drive;
    }

    private void CloseGripper(ArticulationBody[] gripper)
    {
        var leftDrive = gripper[0].xDrive;
        var rightDrive = gripper[1].xDrive;

        leftDrive.target = 0.0f;
        rightDrive.target = 0.0f;

        gripper[0].xDrive = leftDrive;
        gripper[1].xDrive = rightDrive;
    }

    private void OpenGripper(ArticulationBody[] gripper)
    {
        var leftDrive = gripper[0].xDrive;
        var rightDrive = gripper[1].xDrive;

        leftDrive.target = 0.045f;
        rightDrive.target = 0.045f;

        gripper[0].xDrive = leftDrive;
        gripper[1].xDrive = rightDrive;
    }

    // Move to rest position with open grippers
    public void MoveToRestPosition()
    {
        StartCoroutine(MoveArmToRestRoutine(leftArmArticulationBodies, leftGripper));
        StartCoroutine(MoveArmToRestRoutine(rightArmArticulationBodies, rightGripper));
        StartCoroutine(MoveTorsoRoutine(0.01f));
    }

    public void TeleportRobot(Vector3 relPosition, Quaternion relRotation)
    {
        base_link.GetComponent<ArticulationBody>().TeleportRoot(base_link.transform.position + relPosition, Quaternion.Inverse(relRotation) * base_link.transform.rotation);
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

    private IEnumerator MoveTorsoRoutine(float target)
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

    double[] GetCurrentJointStates(ArticulationBody[] arm)
    {
        var jointAngles = new double[8];
        jointAngles[0] = torsoArticulationBody.xDrive.target;
        for (int i = 1; i < 8; i++)
        {
            jointAngles[i] = Mathf.Deg2Rad * arm[i-1].xDrive.target;
        }
        
        return jointAngles;
    }

    public PickPlaceServiceRequest PlanningRequest(Vector3 pickPos, Vector3 placePos, Quaternion pickOr, Quaternion placeOr)
    {
        PickPlaceServiceRequest request = new PickPlaceServiceRequest();
        
        request.pick_pose = new RosMessageTypes.Geometry.PoseMsg
        {
            position = pickPos.To<FLU>(),
            orientation = pickOr.To<FLU>()
        };

        request.place_pose = new RosMessageTypes.Geometry.PoseMsg
        {
            position = placePos.To<FLU>(),
            orientation = placeOr.To<FLU>()
        };

        request.joint_angles = GetCurrentJointStates(leftArmArticulationBodies);

        return request;
    }

    public void ROSServiceResponse(PickPlaceServiceResponse response)
    {
        if (response.arm_trajectory.trajectory.Length > 0)
        {
            Debug.Log("Trajectory returned.");
            StartCoroutine(ExecuteTrajectory(response));
        }
        else
        {
            Debug.Log("No trajectory returned from MoveIt.");
        }
    }

    private IEnumerator ExecuteTrajectory(PickPlaceServiceResponse response)
    {
        var lastJointState = GetCurrentJointStates(leftArmArticulationBodies).Select(r =>(double)r* Mathf.Rad2Deg).ToArray();
        var steps = this.steps;
        for (int poseIndex = 0; poseIndex < response.arm_trajectory.trajectory.Length; poseIndex++)
        {
            if(poseIndex == response.arm_trajectory.trajectory.Length - 1)
            {
                steps = 2;
            }

            for (int jointConfigIndex = 0; jointConfigIndex < response.arm_trajectory.trajectory[poseIndex].joint_trajectory.points.Length; jointConfigIndex++)
            {
                var jointPositions = response.arm_trajectory.trajectory[poseIndex].joint_trajectory.points[jointConfigIndex].positions;
                double[] result = jointPositions.Select(r => (double)r * Mathf.Rad2Deg).ToArray();

                var torsoDrive = torsoArticulationBody.xDrive;
                var lastTorsoValue = torsoDrive.target;
                
                for (int i = 0; i <= steps; i++)
                {
                    for (int joint = 0; joint < leftArmArticulationBodies.Length; joint++)
                    {
                        var joint1XDrive = leftArmArticulationBodies[joint].xDrive;
                        joint1XDrive.target = (float)(lastJointState[joint+1] + (result[joint+1] - lastJointState[joint+1]) * (1.0f / steps) * i);
                        leftArmArticulationBodies[joint].xDrive = joint1XDrive;
                    }

                    torsoDrive.target = lastTorsoValue + ((float)jointPositions[0] - lastTorsoValue) * (float)(1.0f / steps) * (float)i;
                    torsoArticulationBody.xDrive = torsoDrive;
                    

                    yield return new WaitForSeconds(jointAssignmentWait);
                }
                lastTorsoValue = torsoDrive.target;
                lastJointState = result;

            }
            // Make sure gripper is open at the beginning
            if (poseIndex == (int)Poses.PreGrasp || poseIndex == (int)Poses.Place)
            {
                yield return new WaitForSeconds(0.5f);
                OpenGripper(leftGripper);
            }
            // Close gripper on object grasping
            if (poseIndex == (int)Poses.Grasp)
            {
                yield return new WaitForSeconds(0.5f);
                CloseGripper(leftGripper);
            }
        }
    }
}
