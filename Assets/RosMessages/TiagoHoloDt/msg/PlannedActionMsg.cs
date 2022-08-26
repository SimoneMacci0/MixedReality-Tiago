//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.TiagoHoloDt
{
    [Serializable]
    public class PlannedActionMsg : Message
    {
        public const string k_RosMessageName = "tiago_holo_dt/PlannedAction";
        public override string RosMessageName => k_RosMessageName;

        public Moveit.RobotTrajectoryMsg pre_grasp_trajectory;
        public Moveit.RobotTrajectoryMsg grasp_trajectory;
        public Moveit.RobotTrajectoryMsg move_trajectory;
        public Moveit.RobotTrajectoryMsg place_trajectory;
        public Moveit.RobotTrajectoryMsg return_trajectory;

        public PlannedActionMsg()
        {
            this.pre_grasp_trajectory = new Moveit.RobotTrajectoryMsg();
            this.grasp_trajectory = new Moveit.RobotTrajectoryMsg();
            this.move_trajectory = new Moveit.RobotTrajectoryMsg();
            this.place_trajectory = new Moveit.RobotTrajectoryMsg();
            this.return_trajectory = new Moveit.RobotTrajectoryMsg();
        }

        public PlannedActionMsg(Moveit.RobotTrajectoryMsg pre_grasp_trajectory, Moveit.RobotTrajectoryMsg grasp_trajectory, Moveit.RobotTrajectoryMsg move_trajectory, Moveit.RobotTrajectoryMsg place_trajectory, Moveit.RobotTrajectoryMsg return_trajectory)
        {
            this.pre_grasp_trajectory = pre_grasp_trajectory;
            this.grasp_trajectory = grasp_trajectory;
            this.move_trajectory = move_trajectory;
            this.place_trajectory = place_trajectory;
            this.return_trajectory = return_trajectory;
        }

        public static PlannedActionMsg Deserialize(MessageDeserializer deserializer) => new PlannedActionMsg(deserializer);

        private PlannedActionMsg(MessageDeserializer deserializer)
        {
            this.pre_grasp_trajectory = Moveit.RobotTrajectoryMsg.Deserialize(deserializer);
            this.grasp_trajectory = Moveit.RobotTrajectoryMsg.Deserialize(deserializer);
            this.move_trajectory = Moveit.RobotTrajectoryMsg.Deserialize(deserializer);
            this.place_trajectory = Moveit.RobotTrajectoryMsg.Deserialize(deserializer);
            this.return_trajectory = Moveit.RobotTrajectoryMsg.Deserialize(deserializer);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.pre_grasp_trajectory);
            serializer.Write(this.grasp_trajectory);
            serializer.Write(this.move_trajectory);
            serializer.Write(this.place_trajectory);
            serializer.Write(this.return_trajectory);
        }

        public override string ToString()
        {
            return "PlannedActionMsg: " +
            "\npre_grasp_trajectory: " + pre_grasp_trajectory.ToString() +
            "\ngrasp_trajectory: " + grasp_trajectory.ToString() +
            "\nmove_trajectory: " + move_trajectory.ToString() +
            "\nplace_trajectory: " + place_trajectory.ToString() +
            "\nreturn_trajectory: " + return_trajectory.ToString();
        }

#if UNITY_EDITOR
        [UnityEditor.InitializeOnLoadMethod]
#else
        [UnityEngine.RuntimeInitializeOnLoadMethod]
#endif
        public static void Register()
        {
            MessageRegistry.Register(k_RosMessageName, Deserialize);
        }
    }
}