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

public class TiagoROSInterface : MonoBehaviour
{
    // ROS Connector
    private ROSConnection ros;

    // ROS topics and services names
    //private string plannerServiceName = "tiago_unity_motion_planner";
    private string navigationTargetReachedTopic = "move_base/result";
    private string moveBaseMakePlanService = "/move_base/make_plan";
    private string baseLinkPoseService = "/base_footprint_pose_service";
    private string moveBaseSendNavGoalTargetTopic = "/move_base_simple/goal";

    // Tiago Reference
    public GameObject tiago;
    public GameObject baseFootprint;
    public GameObject baseLink;
    public GameObject laser;
    private TiagoController controller;

    // Variables for object grasping
    private float liftOffset = 0.1f;

    // Scene objects
    public GameObject scene;
    public GameObject marker;
    public GameObject pickObject;
    public GameObject placeObject;
    public GameObject navTarget1;

    // Variable for rendering holographic arm trakectories
    public int steps;

    // Variable specifying delay after which send navigation goal
    public int delay;

    // Other variables
    private Vector3 basePos;
    private Quaternion baseRot;
    private Vector3 posDiff;


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
        
        // Send request to get initial robot's pose
        var request = new PoseServiceRequest();
        ros.SendServiceMessage<PoseServiceResponse>(baseLinkPoseService, request, BaseLinkPoseServiceResponse);
    }

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
                    posDiff = new Vector3(diff.x, diff.y, diff.z);
                }
                i++;
            }
        }
    }

    // On navigation complete, update robot's pose for coherent visualization
    public void NavTargetCallback(MoveBaseActionResultMsg msg)
    {
        Debug.Log(msg.status.text);

        var request = new PoseServiceRequest();
        ros.SendServiceMessage<PoseServiceResponse>(baseLinkPoseService, request, UpdateRobotPoseOnTargetReached);
    }

    public void UpdateRobotPoseOnTargetReached(PoseServiceResponse response)
    {
        var oldBasePos = new Vector3(basePos.x, basePos.y, basePos.z);
        var oldBaseRot = new Quaternion(baseRot.x, baseRot.y, baseRot.z, baseRot.w);

        basePos = new PointMsg(
            (float)response.base_link_pose.position.x,
            (float)response.base_link_pose.position.y,
            (float)response.base_link_pose.position.z).From<FLU>();
        baseRot = response.base_link_pose.orientation.From<FLU>();

        var baseDiff = baseLink.transform.rotation * Quaternion.Inverse(baseRot) * (basePos - oldBasePos) - posDiff;

        //var newPos = baseLink.transform.rotation * Quaternion.Inverse(baseRot) * (lastPoint - basePos) + baseLink.transform.position;
        var newPos = baseLink.transform.position + baseDiff;
        //var newRot = baseLink.transform.rotation * Quaternion.Inverse(oldBaseRot) * baseRot;
        controller.TeleportRobotPosition(newPos);

        // re-enable gravity 
        tiago.GetComponent<UrdfRobot>().SetRigidbodiesUseGravity();

        // Set target for navigation active again in front of the robot's base
        navTarget1.SetActive(true);
        navTarget1.transform.localPosition = new Vector3(0, 0, 0.5f);
    }

    /*public void PlanMotion()
    {
        var pickPosition = pickObject.transform.localPosition;
        var pickPositionWRTBaseLink = baseLink.transform.InverseTransformPoint(pickPosition);
        pickPositionWRTBaseLink.y += liftOffset;
        var pickOrientation = Quaternion.Euler(0, 0, -90);

        var placePosition = placeObject.transform.localPosition;
        var placePositionWRTBaseLink = baseLink.transform.InverseTransformPoint(placePosition);
        placePositionWRTBaseLink.y = pickPositionWRTBaseLink.y;
        var placeOrientation = pickOrientation;

        PickPlaceServiceRequest request = null;
        request = controller.PlanningRequest(pickPositionWRTBaseLink, placePositionWRTBaseLink, pickOrientation, placeOrientation);
        ros.SendServiceMessage<PickPlaceServiceResponse>("left_group/" + plannerServiceName, request, controller.ROSServiceResponse);
    }*/

}
