using System;
using UnityEngine;
using Scan = RosMessageTypes.Sensor.LaserScanMsg;
using Header = RosMessageTypes.Std.HeaderMsg;
using Time = RosMessageTypes.BuiltinInterfaces.TimeMsg;

public class LidarSensor : MonoBehaviour
{
    // Sensor parameters
    private float arcAngle;
    private int numLines;
    private float maxDist;

    private float[] ranges;

    private float angleMin;
    private float angleMax;
    private float angleIncrement;

    private GameObject base_link;

    private float scanFreq { get; set; }

    public void Init(float angle, int lines, float dist, int scansPerSec, GameObject base_link)
    {
        this.arcAngle = angle;
        this.numLines = lines;
        this.maxDist = dist;

        this.base_link = base_link;

        scanFreq = 1.0f / scansPerSec;

        angleMin = -angle / 2 + angle / (2 * lines);
        angleIncrement = angle / lines;
        angleMax = -angle / 2 + ((lines - 1) * angleIncrement) + angle / (2 * lines);
    }

    public Scan doScan()
    {
        ranges = new float[numLines];
        var color = Color.red;
        color.a = 0.3f;
        for (int l = 0; l < numLines; l++)
        {
            var shootVec = base_link.transform.rotation * Quaternion.AngleAxis(-1 * arcAngle / 2 + (l * arcAngle / numLines) + arcAngle / (2 * numLines), Vector3.up) * Vector3.forward;
            RaycastHit hit;
            if (Physics.Raycast(transform.position, shootVec, out hit, maxDist))
            {
                Debug.DrawLine(transform.position, hit.point, color, 0.2f);
                ranges[l] = hit.distance;
            }
            else ranges[l] = maxDist;
        }

        DateTime now = DateTime.Now;
        var scanMsg = new Scan
        {
            header = new Header
            {
                seq = (uint)1,
                stamp = new Time
                {
                    sec = (uint)DateTimeOffset.Now.ToUnixTimeSeconds(),
                    nanosec = 0,
                },
                frame_id = "base_laser_link"
            },
            angle_min = angleMax * (float)Math.PI / 180.0f,
            angle_max = angleMin * (float)Math.PI / 180.0f, 
            angle_increment = -angleIncrement * (float)Math.PI / 180.0f,
            time_increment = scanFreq,
            scan_time = scanFreq,
            range_min = 0,
            range_max = maxDist,
            ranges = ranges
        };
        return scanMsg;
    }
}