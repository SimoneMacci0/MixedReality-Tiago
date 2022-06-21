using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.UrdfImporter;

using RosMessageTypes.TiagoUnity;
using RosMessageTypes.MoveBase;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using Header = RosMessageTypes.Std.HeaderMsg;
using TimeMsg = RosMessageTypes.BuiltinInterfaces.TimeMsg;
using PoseStamped = RosMessageTypes.Geometry.PoseStampedMsg;
using Pose = RosMessageTypes.Geometry.PoseMsg;
using Scan = RosMessageTypes.Sensor.LaserScanMsg;
using Twist = RosMessageTypes.Geometry.TwistMsg;
using Odometry = RosMessageTypes.Nav.OdometryMsg;
using NavSrvRequest = RosMessageTypes.Nav.GetPlanRequest;
using NavSrvResponse = RosMessageTypes.Nav.GetPlanResponse;
using System;

public class TiagoROSInterface : MonoBehaviour
{
    // ROS Connector
    private ROSConnection ros;
    private bool connectedToROS = false;

    // ROS topics and services names
    //private string plannerServiceName = "tiago_unity_motion_planner";
    //private string odometryFrameTopic = "odometry_frame";
    private string navigationTargetReachedTopic = "move_base/result";
    //private string scanTopicName = "scan";
    private string moveBaseMakePlanService = "/move_base/make_plan";
    private string baseLinkPoseService = "/base_link_pose_service";

    // Tiago Reference
    public GameObject tiago;
    public GameObject baseFootprint;
    public GameObject baseLink;
    public GameObject laser;
    private TiagoController controller;
    private LidarSensor lidarSensor;

    // Variables for object grasping
    private float liftOffset = 0.1f;

    // Scene objects
    public GameObject scene;
    public GameObject marker;
    public GameObject pickObject;
    public GameObject placeObject;
    public GameObject navTarget1;

    // Variables for rendering trajectories
    public int steps;

    // Variables for mobile base speed
    public float maxLinearSpeed;
    public float maxRotationalSpeed;

    // Variables for laser sensor
    public float lidarSensorAngle;
    public int lidarSensorDensity;
    public float lidarSensorDist;
    public int lidarSensorFreq;

    // Variables for publishing topics
    private float odomTopicFreq = 0.25f;
    private float odomTimeElasped = 0.0f;
    private float scanTimeElapsed = 0.0f;

    // Other variables
    private List<GameObject> trajectory;
    private GameObject prefab;
    private Vector3 initPos;
    private Quaternion initRot;
    private bool isFirstPlan = true;
    private bool hasUpdatedPose = false;
    private Vector3 basePos;
    private Quaternion baseRot;
    private Vector3 relPosition;
    private Quaternion relRotation;

    // Start is called before the first frame update
    IEnumerator Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.listenForTFMessages = false;

        prefab = Resources.Load("Prefabs/TrajPoint") as GameObject;
        trajectory = new List<GameObject>();
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
        yield return new WaitForSeconds(1.0f);
        controller.Initialize(tiago, baseLink, maxLinearSpeed, maxRotationalSpeed, steps, spawnPosition);

        // Initialize laser Sensor
        //lidarSensor = laser.AddComponent<LidarSensor>();
        //lidarSensor.Init(lidarSensorAngle, lidarSensorDensity, lidarSensorDist, lidarSensorFreq, baseLink);

        /*ros.RegisterRosService<PickPlaceServiceRequest, PickPlaceServiceResponse>("left_group/" + plannerServiceName);
        ros.RegisterPublisher<Pose>(odometryFrameTopic);
        ros.RegisterPublisher<PoseStamped>(navigationTargetTopic);
        ros.RegisterPublisher<Scan>(scanTopicName);
        ros.Subscribe<Twist>("/nav_vel", controller.MobileBaseCommand);*/
        //ros.Subscribe<Pose>("/tiago_base_link", controller.TeleportRobot);

        ros.RegisterRosService<NavSrvRequest, NavSrvResponse>(moveBaseMakePlanService);
        ros.RegisterRosService<PoseServiceRequest, PoseServiceResponse>(baseLinkPoseService);
        ros.Subscribe<MoveBaseActionResultMsg>(navigationTargetReachedTopic, NavTargetCallback);

        // Send request to get initial robot's pose
        var request = new PoseServiceRequest();
        ros.SendServiceMessage<PoseServiceResponse>(baseLinkPoseService, request, BaseLinkPoseServiceResponse);
    }

    /*public void publishOdometryFrame()
    {
        var odomPose = new Pose
        {
            position = (baseLink.transform.position).To<FLU>(),
            orientation = (baseLink.transform.rotation).To<FLU>(),
        };
        ros.Publish(odometryFrameTopic, odomPose);
    }*/

    /*public void publishScan()
    {
        var scan = lidarSensor.doScan();
        ros.Publish(scanTopicName, scan);
    }*/

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

    public void GetNavigationPlan()
    {
        /*foreach(GameObject go in trajectory)
        {
            Destroy(go);
        }
        trajectory.Clear();*/

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

        var request = new NavSrvRequest();
        request.start = basePose;
        request.goal = goalPose;

        ros.SendServiceMessage<NavSrvResponse>(moveBaseMakePlanService, request, NavServiceResponse);
    }

    public void BaseLinkPoseServiceResponse(PoseServiceResponse response)
    {
        basePos = new RosMessageTypes.Geometry.Point32Msg(
            (float)response.base_link_pose.position.x, 
            (float)response.base_link_pose.position.y, 
            (float)response.base_link_pose.position.z).From<FLU>();
        baseRot = response.base_link_pose.orientation.From<FLU>();

        if(isFirstPlan)
        {
            initPos = new Vector3(basePos.x, basePos.y, basePos.z);
            initRot = new Quaternion(baseRot.x, baseRot.y, baseRot.z, baseRot.w);
            isFirstPlan = false;
        }
    }

    public void UpdateRobotPoseCallback(PoseServiceResponse response)
    {
        basePos = new RosMessageTypes.Geometry.Point32Msg(
            (float)response.base_link_pose.position.x,
            (float)response.base_link_pose.position.y,
            (float)response.base_link_pose.position.z).From<FLU>();
        baseRot = response.base_link_pose.orientation.From<FLU>();

        //relPosition = basePos - initPos;
        //relRotation = Quaternion.Inverse(baseRot) * initRot;
        //controller.TeleportRobotRelative(relPosition, relRotation);

        //initPos = new Vector3(basePos.x, basePos.y, basePos.z);
        initRot = new Quaternion(baseRot.x, baseRot.y, baseRot.z, baseRot.w);

        /*foreach (GameObject go in trajectory)
        {
            Destroy(go);
        }
        trajectory.Clear();*/
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
        tiago.GetComponent<UrdfRobot>().SetRigidbodiesUseGravity();

        var initPos = baseLink.transform.position;
        var initRot = baseLink.transform.rotation;
        for (int t=0; t < 3; t++)
        {
            int i = 0;
            foreach (PoseStamped ps in response.plan.poses)
            {
                if (i % 2 == 0 && i != response.plan.poses.Length - 1)
                {
                    var position = new RosMessageTypes.Geometry.Point32Msg((float)ps.pose.position.x, (float)ps.pose.position.y, (float)ps.pose.position.z).From<FLU>();
                    var orientation = ps.pose.orientation.From<FLU>();

                    var newPos = initRot * Quaternion.Inverse(baseRot) * (position - basePos) + initPos + Vector3.up * 0.1f;
                    var newRot = initRot * Quaternion.Inverse(baseRot) * orientation;
                    controller.TeleportRobot(newPos, newRot);

                    yield return new WaitForSeconds(0.1f);

                    //var newGo = Instantiate(prefab, Vector3.zero, Quaternion.identity);
                    //newGo.transform.parent = baseLink.transform;

                    //newGo.transform.localPosition = Vector3.up * 0.1f + Quaternion.Inverse(baseRot) * (position - basePos);
                    //trajectory.Add(newGo);
                }
                i++;
            }
        }
        tiago.GetComponent<UrdfRobot>().SetRigidbodiesUseGravity();

        navTarget1.SetActive(true);
        navTarget1.transform.localPosition = new Vector3(0, 0, 0.5f);
    }

    public void NavTargetCallback(MoveBaseActionResultMsg msg)
    {
        Debug.Log(msg.status.text);

        var request = new PoseServiceRequest();
        ros.SendServiceMessage<PoseServiceResponse>(baseLinkPoseService, request, UpdateRobotPoseCallback);
    }

    /*public void PublishNavigationTarget()
    {
        var header = new Header
        {
            seq = (uint)1,
            stamp = new TimeMsg
            {
                sec = (uint)DateTimeOffset.Now.ToUnixTimeSeconds(),
                nanosec = 0,
            },
            frame_id = "map"
        };
        var targetPos = new PoseStamped
        {
            header = header,
            pose = new Pose
            {
                position = (navTarget1.transform.position).To<FLU>(),
                orientation = (navTarget1.transform.rotation).To<FLU>()
            }
        };
        ros.Publish(navigationTargetTopic, targetPos);
    }*/


    public void Update()
    {
        if (connectedToROS)
        {
            /*var delta = Time.deltaTime;
            odomTimeElasped += delta;
            scanTimeElapsed += delta;
            if (odomTimeElasped > odomTopicFreq)
            {
                publishOdometryFrame();
                odomTimeElasped = 0.0f;
            }
            if(scanTimeElapsed > 1.0f/lidarSensorFreq)
            {
                publishScan();
                scanTimeElapsed = 0.0f;
            }*/
        }
    }

}
