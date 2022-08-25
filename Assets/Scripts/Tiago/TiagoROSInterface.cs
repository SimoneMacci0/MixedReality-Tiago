using System.Collections;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.UrdfImporter;

using RosMessageTypes.TiagoUnity;
using RosMessageTypes.MoveBase;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;

using PoseStamped = RosMessageTypes.Geometry.PoseStampedMsg;
using PointMsg = RosMessageTypes.Geometry.Point32Msg;
using Point = RosMessageTypes.Geometry.PointMsg;
using V3 = RosMessageTypes.Geometry.Vector3Msg;
using Path = RosMessageTypes.Nav.PathMsg;

public class TiagoROSInterface : MonoBehaviour
{
    // ROS Connector
    private ROSConnection ros;

    // ROS topics and services names
    private string baseFootprintPoseService = "/base_footprint_pose_service";

    private string navigationPathTopic = "/navigation_plan";
    private string navigationTargetReachedTopic = "/move_base/result";

    private string leftGroupMotionPlannerSrv = "/left_group/plan_action";
    private string rightGroupMotionPlannerSrv = "/right_group/plan_action";

    // Tiago Reference
    public GameObject tiago;
    public GameObject baseFootprint;
    public GameObject baseLink;
    public GameObject laser;
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

    public void SpawnScene()
    {
        StartCoroutine(SpawnSceneRoutine());
    }

    private IEnumerator SpawnSceneRoutine()
    {
        var markerPosition = marker.transform.position;
        //imageTarget.GetComponent<ImageTargetBehaviour>().enabled = false;
        marker.GetComponent<Behaviour>().enabled = false;
        marker.SetActive(false);

        var spawnPosition = /*imTargetPosition + Vector3.forward * depthOffset - Vector3.up * heightOffset*/ markerPosition;
        scene.transform.SetPositionAndRotation(spawnPosition, Quaternion.identity);
        scene.SetActive(true);

        // Initialize robot controller
        controller = gameObject.AddComponent<TiagoController>();

        // Wait 1 second for correct initialization of the gameobject
        yield return new WaitForSeconds(1.0f);

        // Init controller with sim values
        controller.Initialize(tiago, baseLink, steps);

        // Register various topics and services from ROS 
        ros.Subscribe<Path>(navigationPathTopic, NavigationPlanCallback);
        ros.Subscribe<MoveBaseActionResultMsg>(navigationTargetReachedTopic, NavigationTargetReachedCallback);

        ros.RegisterRosService<PoseServiceRequest, PoseServiceResponse>(baseFootprintPoseService);
        ros.RegisterRosService<ActionServiceRequest, ActionServiceResponse>(leftGroupMotionPlannerSrv);
        ros.RegisterRosService<ActionServiceRequest, ActionServiceResponse>(rightGroupMotionPlannerSrv);

        // Send request to get initial robot's pose
        var request = new PoseServiceRequest();
        ros.SendServiceMessage<PoseServiceResponse>(baseFootprintPoseService, request, BaseLinkPoseServiceResponse);
    }

    // -------------------------------------
    // METHODS RELATED TO ROBOT'S MOBILE BASE AND HOLOGRAPHIC NAVIGATION

    // Update robot's base_link pose with respect to ROS map frame
    public void BaseLinkPoseServiceResponse(PoseServiceResponse response)
    {
        basePos = new PointMsg(
            (float)response.base_link_pose.position.x, 
            (float)response.base_link_pose.position.y, 
            (float)response.base_link_pose.position.z).From<FLU>();
        baseRot = response.base_link_pose.orientation.From<FLU>();
    }

    // Callback function called whenever a navigation plan has been received
    public void NavigationPlanCallback(Path path)
    {
        if (path != null)
        {
            StartCoroutine(HoloNavigationRoutine(path));
        }
        else
            Debug.Log("Empty path message received");
    }

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
    public void NavigationTargetReachedCallback(MoveBaseActionResultMsg msg)
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

    public void PlanHandoverMotion(string arm)
    {
        var handoverPos = arm == "left" ? leftHandoverPos : rightHandoverPos;
        var plannerSrv = arm == "left" ? leftGroupMotionPlannerSrv : rightGroupMotionPlannerSrv;

        // Convert from Unity to ROS coordinates 
        var rosHandoverPos = new Point();
        rosHandoverPos.x = handoverPos.transform.localPosition.z;
        rosHandoverPos.y = -handoverPos.transform.localPosition.x;
        rosHandoverPos.z = handoverPos.transform.localPosition.y;

        var request = controller.PlanningRequest(arm, "handover", rosHandoverPos, null, new V3(1, 0, 0));

        // Send request to plan handover and set controller to busy, to wait for completion of holographic motion 
        ros.SendServiceMessage<ActionServiceResponse>(plannerSrv, request, controller.PlanningServiceResponse);
        StartCoroutine(SpawnInteractableOnMovementCompletion(arm, 0)); 
    }

    private IEnumerator SpawnInteractableOnMovementCompletion(string arm, int idx)
    {
        controller.busy = true;
        while (controller.busy)
        {
            yield return new WaitForSeconds(0.25f);
        }
        var graspingFrame = arm == "left" ? leftGraspingFrame : rightGraspingFrame;
        interactables[idx].SetActive(true);
        interactables[idx].transform.SetPositionAndRotation(leftGraspingFrame.transform.position + graspOffset * Vector3.back, Quaternion.identity);
    }

    public void OnHandoverDetected()
    {
        controller.CompleteHandover();
    }


}
