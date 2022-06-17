using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;

using Header = RosMessageTypes.Std.HeaderMsg;
using TimeMsg = RosMessageTypes.BuiltinInterfaces.TimeMsg;
using Pose = RosMessageTypes.Geometry.PoseMsg;
using PoseStamped = RosMessageTypes.Geometry.PoseStampedMsg;
using System;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;

public class ROSNavigationInterface : MonoBehaviour
{
    public GameObject vuforiaMarker;
    public GameObject robotBase;
    public GameObject navTarget;
    public GameObject button;

    private ROSConnection ros;
    private string navigationTargetTopic = "ros_unity/target_goal";
    private Vector3 robotBaseOffset = 0.1f * Vector3.down + 0.1f * Vector3.forward;
    private Vector3 targetSpawnOffset = 0.1f * Vector3.down + Vector3.forward;
    private Vector3 buttonSpawnOffset = 0.7f * Vector3.up + 0.1f * Vector3.forward + 0.2f * Vector3.right;

    private void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.listenForTFMessages = false;

        ros.RegisterPublisher<Pose>(navigationTargetTopic);
    }

    public void Spawn()
    {
        vuforiaMarker.GetComponent<Behaviour>().enabled = false;

        robotBase.transform.SetPositionAndRotation(vuforiaMarker.transform.position + robotBaseOffset, Quaternion.Euler(0, 180, 0));
        robotBase.SetActive(true);

        button.transform.SetPositionAndRotation(vuforiaMarker.transform.position + buttonSpawnOffset, Quaternion.identity);
        button.SetActive(true);

        navTarget.transform.SetPositionAndRotation(vuforiaMarker.transform.position + targetSpawnOffset, Quaternion.Euler(0,180,0));
        navTarget.SetActive(true);

        vuforiaMarker.SetActive(false);
    }

    public void PublishNavigationTarget()
    {
        var robotBaseRelative = robotBase.transform.InverseTransformPoint(navTarget.transform.position) * robotBase.transform.localScale.x;

        var targetPos = new Pose
        {
            position = robotBaseRelative.To<FLU>(),
            orientation = navTarget.transform.rotation.To<FLU>()
        };
        ros.Publish(navigationTargetTopic, targetPos);
    }

}
