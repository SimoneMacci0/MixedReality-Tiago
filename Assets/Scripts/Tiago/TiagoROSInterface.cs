using System.Collections;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.UrdfImporter;

using RosMessageTypes.TiagoUnity;
using RosMessageTypes.MoveBase;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;

using Header = RosMessageTypes.Std.HeaderMsg;
using PoseStamped = RosMessageTypes.Geometry.PoseStampedMsg;
using NavSrvRequest = RosMessageTypes.Nav.GetPlanRequest;
using NavSrvResponse = RosMessageTypes.Nav.GetPlanResponse;
using PointMsg = RosMessageTypes.Geometry.Point32Msg;
using Point = RosMessageTypes.Geometry.PointMsg;
using V3 = RosMessageTypes.Geometry.Vector3Msg;

public class TiagoROSInterface : MonoBehaviour
{
    // ROS Connector
    private ROSConnection ros;

    // ROS topics and services names
    private string navigationTargetReachedTopic = "move_base/result";
    private string moveBaseMakePlanService = "/move_base/make_plan";
    private string baseLinkPoseService = "/base_footprint_pose_service";
    private string moveBaseSendNavGoalTargetTopic = "/move_base_simple/goal";
    private string leftGroupMotionPlannerSrv = "/left_group/tiago_unity_motion_planner";
    private string rightGroupMotionPlannerSrv = "/right_group/tiago_unity_motion_planner";

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
    public GameObject interactable;
    //public GameObject placeObject;
    public GameObject navTarget1;

    // Variable for rendering holographic arm trakectories
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
        ros.RegisterPublisher<PoseStamped>(moveBaseSendNavGoalTargetTopic);
        ros.RegisterRosService<NavSrvRequest, NavSrvResponse>(moveBaseMakePlanService);
        ros.RegisterRosService<PoseServiceRequest, PoseServiceResponse>(baseLinkPoseService);
        ros.Subscribe<MoveBaseActionResultMsg>(navigationTargetReachedTopic, NavTargetCallback);
        ros.RegisterRosService<ActionServiceRequest, ActionServiceResponse>(leftGroupMotionPlannerSrv);
        ros.RegisterRosService<ActionServiceRequest, ActionServiceResponse>(rightGroupMotionPlannerSrv);

        // Send request to get initial robot's pose
        var request = new PoseServiceRequest();
        ros.SendServiceMessage<PoseServiceResponse>(baseLinkPoseService, request, BaseLinkPoseServiceResponse);
    }

    // -------------------------------------
    // METHODS RELATED TO ROBOT'S MOBILE BASE AND HOLOGRAPHIC NAVIGATION

    // Call make_plan service to retrieve  navigation plan
    public void GetNavigationPlan()
    {
        // Start pose
        var basePose = new PoseStamped
        {
            header = new Header
            {
                frame_id = "map"
            },
            pose =
            {
                position = basePos.To<FLU>(),
                orientation = baseRot.To<FLU>(),
            }
        };

        // Goal pose to be reached
        var goalPose = new PoseStamped
        {
            header = new Header
            {
                frame_id = "map"
            },
            pose =
            {
                position = (basePos + (baseRot) * (navTarget1.transform.localPosition)).To<FLU>(),
                orientation = Quaternion.identity.To<FLU>(),
            }
        };

        // Send request
        var request = new NavSrvRequest();
        request.start = basePose;
        request.goal = goalPose;

        ros.SendServiceMessage<NavSrvResponse>(moveBaseMakePlanService, request, NavServiceResponse);

    }

    private IEnumerator SendGoalAfterDelay(PoseStamped goal)
    {
        yield return new WaitForSeconds(delay);
        ros.Publish(moveBaseSendNavGoalTargetTopic, goal);
    }

    // Update robot's base_link pose with respect to ROS map frame
    public void BaseLinkPoseServiceResponse(PoseServiceResponse response)
    {
        Debug.Log(response.base_link_pose.position);
        basePos = new PointMsg(
            (float)response.base_link_pose.position.x, 
            (float)response.base_link_pose.position.y, 
            (float)response.base_link_pose.position.z).From<FLU>();
        baseRot = response.base_link_pose.orientation.From<FLU>();
    }

    public void NavServiceResponse(NavSrvResponse response)
    {
        if (response.plan.poses.Length > 0)
        {
            navTarget1.SetActive(false);
            StartCoroutine(PathHoloNavigationRoutine(response));
        }
        else
            Debug.Log("No plan found!");
    }

    private IEnumerator PathHoloNavigationRoutine(NavSrvResponse response)
    {
        // Disable urdf robot gravity for teleporting smoothly
        tiago.GetComponent<UrdfRobot>().SetRigidbodiesUseGravity();

        // Get initial pose for coherent visualization
        var initPos = baseLink.transform.position;
        var initRot = baseLink.transform.rotation;

        // Send goal position to robot controller after delay
        StartCoroutine(SendGoalAfterDelay(response.plan.poses[response.plan.poses.Length - 2]));

        // Repeat holo path N times
        for (int t=0; t < 1; t++)
        {
            int i = 0;
            foreach (PoseStamped ps in response.plan.poses)
            {
                // Get i-th navigation point from the overall path
                var position = new PointMsg((float)ps.pose.position.x, (float)ps.pose.position.y, (float)ps.pose.position.z).From<FLU>();
                var orientation = ps.pose.orientation.From<FLU>();

                // Need to transform ROS coordinate to Unity and then project everything in the Unity fixed frame
                var newPos = initRot * Quaternion.Inverse(baseRot) * (position - basePos) + initPos;
                var newRot = initRot * Quaternion.Inverse(baseRot) * orientation;

                if (i != response.plan.poses.Length - 1)
                {
                    controller.TeleportRobot(newPos, newRot);

                    // Wait interval for holo path animation
                    yield return new WaitForSeconds(0.025f);
                }
                else if (i == response.plan.poses.Length - 1)
                {
                    var diff = newPos - initPos;
                    //posDiff = new Vector3(diff.x, diff.y, diff.z);
                }
                i++;
            }
        }
    }

    // On navigation complete, update robot's pose for coherent visualization
    public void NavTargetCallback(MoveBaseActionResultMsg msg)
    {
        //Debug.Log(msg.status.text);

        var request = new PoseServiceRequest();
        ros.SendServiceMessage<PoseServiceResponse>(baseLinkPoseService, request, UpdateRobotPoseOnTargetReached);
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

        // Set target for navigation active again in front of the robot's base
        navTarget1.SetActive(true);
        navTarget1.transform.localPosition = new Vector3(0, 0, 0.5f);
    }

    // -------------------------------------
    // METHODS RELATED TO ROBOT'S UPPER LIMBS MOVEMENTS AND MOTION PLANNING

    public void PlanHandoverMotion(string arm)
    {
        var handoverPos = arm == "left" ? leftHandoverPos : rightHandoverPos;

        // Convert from Unity to ROS coordinates 
        var rosHandoverPos = new Point();
        rosHandoverPos.x = handoverPos.transform.localPosition.z;
        rosHandoverPos.y = -handoverPos.transform.localPosition.x;
        rosHandoverPos.z = handoverPos.transform.localPosition.y;

        var request = controller.PlanningRequest(arm, "handover", rosHandoverPos, null, new V3(1, 0, 0));

        // Send request to plan handover and set controller to busy, to wait for completion of holographic motion 
        ros.SendServiceMessage<ActionServiceResponse>("/" + arm + "_group/tiago_unity_motion_planner", request, controller.PlanningServiceResponse);
        StartCoroutine(SpawnInteractableOnMovementCompletion(arm)); 
    }

    private IEnumerator SpawnInteractableOnMovementCompletion(string arm)
    {
        controller.busy = true;
        while (controller.busy)
        {
            yield return new WaitForSeconds(0.25f);
        }
        var graspingFrame = arm == "left" ? leftGraspingFrame : rightGraspingFrame;
        interactable.SetActive(true);
        interactable.transform.SetPositionAndRotation(leftGraspingFrame.transform.position + graspOffset * Vector3.back, Quaternion.identity);
    }

    public void OnHandoverDetected()
    {
        controller.CompleteHandover();
    }


}
