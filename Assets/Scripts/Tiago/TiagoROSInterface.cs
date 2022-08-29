using System.Collections;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.UrdfImporter;

using RosMessageTypes.TiagoHoloDt;
using RosMessageTypes.MoveBase;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;

using PoseStamped = RosMessageTypes.Geometry.PoseStampedMsg;
using PointMsg = RosMessageTypes.Geometry.Point32Msg;
using Path = RosMessageTypes.Nav.PathMsg;
using Bool = RosMessageTypes.Std.BoolMsg;

public class TiagoROSInterface : MonoBehaviour
{
    // ROS Connector
    private ROSConnection ros;

    // ROS topics and services names
    private string baseFootprintPoseService = "/base_footprint_pose_service";
    private string jointStateService = "/joint_state_service";

    private string navigationPathTopic = "/navigation_plan";
    private string navigationTargetReachedTopic = "/move_base/result";

    private string leftGroupMotionPlannerSrv = "/left_group/plan_action";
    private string rightGroupMotionPlannerSrv = "/right_group/plan_action";

    private string plannedActionTopic = "/planned_action";
    private string handoverTriggeredTopic = "/handover_triggered";

    // Tiago Reference
    public GameObject tiago;
    public GameObject baseFootprint;
    public GameObject baseLink;
    private TiagoController controller;

    // Variables for object grasping
    public GameObject leftHandoverPos;
    public GameObject rightHandoverPos;

    // Scene objects
    public GameObject scene;
    public GameObject marker;
    public GameObject[] interactables;

    // Variable for rendering holographic arm trajectories
    public int steps;

    // Variables for referencing grasping frames
    public GameObject leftGraspingFrame;
    public GameObject rightGraspingFrame;
    private float graspOffset = 0.06f;

    // Variable specifying delay after which send navigation goal
    public int delay;

    // Other variables
    private float depthOffset = 0.04f;
    private float heightOffset = 0.03f;
    private Vector3 basePos;
    private Quaternion baseRot;


    IEnumerator Start()
    {
        // Get connection instance to ROS
        ros = ROSConnection.GetOrCreateInstance();
        // Disable listening to TF messages for HoloLens performance optimization
        ros.listenForTFMessages = false;

        yield return new WaitForSeconds(0.5f);
    }

    // Method to spawn the robot in place the first time the QR marker is detected
    public void SpawnScene()
    {
        StartCoroutine(SpawnSceneRoutine());
    }

    // Internal routine to spawn robot
    private IEnumerator SpawnSceneRoutine()
    {
        var markerPosition = marker.transform.position;
        // Disable marker behavior so that continuous tracking is disabled
        marker.GetComponent<Behaviour>().enabled = false;
        marker.SetActive(false);

        var spawnPosition = markerPosition + Vector3.forward * depthOffset + Vector3.down * heightOffset;
        scene.transform.SetPositionAndRotation(spawnPosition, Quaternion.identity);
        scene.SetActive(true);

        // Initialize robot controller
        controller = gameObject.AddComponent<TiagoController>();

        // Wait 1 second for correct initialization of the controller script
        yield return new WaitForSeconds(1.0f);

        // Init controller with sim values
        controller.Initialize(tiago, baseLink, steps);

        // Register various topics and services from ROS 
        ros.Subscribe<Path>(navigationPathTopic, OnNavigationPlanReceived);
        ros.Subscribe<MoveBaseActionResultMsg>(navigationTargetReachedTopic, OnNavigationTargetReached);
        ros.Subscribe<PlannedActionWithTypeAndArmMsg>(plannedActionTopic, OnActionPlanReceived);
        ros.Subscribe<Bool>(handoverTriggeredTopic, OnHandoverTriggered);

        ros.RegisterRosService<PoseServiceRequest, PoseServiceResponse>(baseFootprintPoseService);
        ros.RegisterRosService<JointStateServiceRequest, JointStateServiceResponse>(jointStateService);
        ros.RegisterRosService<ActionServiceRequest, ActionServiceResponse>(leftGroupMotionPlannerSrv);
        ros.RegisterRosService<ActionServiceRequest, ActionServiceResponse>(rightGroupMotionPlannerSrv);

        // Send request to get initial robot's pose
        ros.SendServiceMessage<PoseServiceResponse>(baseFootprintPoseService, new PoseServiceRequest(), BaseLinkPoseServiceResponse);
        ros.SendServiceMessage<JointStateServiceResponse>(jointStateService, new JointStateServiceRequest(), controller.JointStateServiceResponse);
    }


    // -------------------------------------
    // METHODS RELATED TO ROBOT'S MOBILE BASE AND HOLOGRAPHIC NAVIGATION


    // Update robot's base_footprint pose with respect to ROS map frame
    public void BaseLinkPoseServiceResponse(PoseServiceResponse response)
    {
        basePos = new PointMsg(
            (float)response.base_link_pose.position.x, 
            (float)response.base_link_pose.position.y, 
            (float)response.base_link_pose.position.z).From<FLU>();
        baseRot = response.base_link_pose.orientation.From<FLU>();
    }

    // Callback function called whenever a navigation plan has been received
    public void OnNavigationPlanReceived(Path path)
    {
        // Disable interactable objects whenever robot moves to a new location
        foreach(GameObject interactable in interactables)
        {
            interactable.SetActive(false);
        }

        if (path != null)
        {
            StartCoroutine(HoloNavigationRoutine(path));
        }
        else
            Debug.Log("Empty path message received");
    }

    // Internal routine that renders the navigation plan as holographic animation
    private IEnumerator HoloNavigationRoutine(Path path)
    {
        // Disable urdf robot gravity for teleporting smoothly
        tiago.GetComponent<UrdfRobot>().SetRigidbodiesUseGravity();

        // Get initial pose for coherent visualization
        var initPos = baseLink.transform.position;
        var initRot = baseLink.transform.rotation;

        foreach (PoseStamped ps in path.poses)
        {
            // Get i-th navigation point from the overall path
            var position = new PointMsg((float)ps.pose.position.x, (float)ps.pose.position.y, (float)ps.pose.position.z).From<FLU>();
            var orientation = ps.pose.orientation.From<FLU>();

            // Need to transform ROS coordinate to Unity and then project everything in the Unity fixed frame
            var newPos = initRot * Quaternion.Inverse(baseRot) * (position - basePos) + initPos;
            var newRot = initRot * Quaternion.Inverse(baseRot) * orientation;

            controller.TeleportRobot(newPos, newRot);

            // Wait interval for holo path animation
            yield return new WaitForSeconds(0.03f);
        }
    }

    // On navigation complete, update robot's pose for coherent visualization
    public void OnNavigationTargetReached(MoveBaseActionResultMsg msg)
    {
        var request = new PoseServiceRequest();
        ros.SendServiceMessage<PoseServiceResponse>(baseFootprintPoseService, request, UpdateRobotPoseOnTargetReached);
    }

    public void UpdateRobotPoseOnTargetReached(PoseServiceResponse response)
    {
        basePos = new PointMsg(
            (float)response.base_link_pose.position.x,
            (float)response.base_link_pose.position.y,
            (float)response.base_link_pose.position.z).From<FLU>();
        baseRot = response.base_link_pose.orientation.From<FLU>();

        // re-enable gravity 
        tiago.GetComponent<UrdfRobot>().SetRigidbodiesUseGravity();
    }


    // -------------------------------------
    // METHODS RELATED TO ROBOT'S UPPER LIMBS MOVEMENTS AND MOTION PLANNING


    // Callback function invoked whenever a new action has been planned by MoveIt and needs to be holo-rendered
    public void OnActionPlanReceived(PlannedActionWithTypeAndArmMsg action)
    {
        if (action.action_type == "handover")
        {
            StartCoroutine(SpawnInteractableOnMovementCompletion(action.planning_arm, 0));
        }
        controller.ActionPlanningServiceResponse(action);
    }

    // Internal routine to spawn holographic object after completion of the robot's action
    private IEnumerator SpawnInteractableOnMovementCompletion(string arm, int idx)
    {
        // Set busy variable to true and wait for completion of holographic action
        controller.busy = true;
        while (controller.busy)
        {
            yield return new WaitForSeconds(0.25f);
        }

        // Spawn holographic item at its expected pose for subsequent handover phase with human
        var graspingFrame = arm == "left" ? leftGraspingFrame : rightGraspingFrame;

        interactables[idx].SetActive(true);
        interactables[idx].transform.SetPositionAndRotation(leftGraspingFrame.transform.position + graspOffset * Vector3.back, Quaternion.identity);
    }

    // Callback function invoked when Digital Twin signal that human wants to perform handover with robot
    public void OnHandoverTriggered(Bool msg)
    {
        // Call internal routine which completes the handover phase
        controller.CompleteHandover();
    } 
}
