using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Microsoft.Kinect;
using Rug.Osc;
using System.Numerics;
using Microsoft.Samples.Kinect.Avateering;

namespace Microsoft.Samples.Kinect.SkeletonBasics.Network.BodySender
{
    public class MessageBuilder
    {


        private List<double> depthToWorld(float x, float y, float z)
        {
            //http://nicolas.burrus.name/index.php/Research/KinectCalibration
            double fx_d = 5.9421434211923247e+02;
            double fy_d = 5.9104053696870778e+02;
            double cx_d = 3.3930780975300314e+02;
            double cy_d = 2.4273913761751615e+02;

            List<double> result = new List<double>();
            double depth = ((double)(z) * -0.0030711016 + 3.3309495161);

            /*result.x = .5 * (float)((x - cx_d) * depth / fx_d);
            result.y = -.5 * (float)((y - cy_d) * depth / fy_d);
            result.z = (float)(depth);*/

            result.Add(.5 * (float)((x - cx_d) * depth / fx_d));
            result.Add(-.5 * (float)((y - cy_d) * depth / fy_d));
            result.Add((float)(depth));
            return result;
        }

        public OscMessage BuildJointMessage(Skeleton body, Joint joint)
        {

            //https://msdn.microsoft.com/en-us/library/hh973073.aspx


            string jointName = joint.JointType.ToString();
            if (jointName.IndexOf("Left") >= 0)
            {
                jointName = jointName.Replace("Left", "Right");
            }
            else if (jointName.IndexOf("Right") >= 0)
            {
                jointName = jointName.Replace("Right", "Left");
            }

            var address = String.Format("/{0}",jointName);
            var position = joint.Position;
            var q = body.BoneOrientations[joint.JointType].AbsoluteRotation.Quaternion; // .AbsoluteRotation.Quaternion;
            var jointQ = new Quaternion(q.X, q.Y, q.Z, q.W);

            JointType start = body.BoneOrientations[joint.JointType].StartJoint;
            var parent = body.BoneOrientations[start].AbsoluteRotation.Quaternion;
            var parentQ = Quaternion.Normalize(new Quaternion(parent.X, parent.Y, parent.Z, parent.W));


            var newq = new Quaternion();


            float angle = (float)Math.Acos(Vector3.Dot(new Vector3(0, 1, 0), new Vector3(jointQ.X, jointQ.Y, jointQ.Z)));
            //best so far
            var newq1 = Quaternion.CreateFromAxisAngle(new Vector3(0, 1, 0), jointQ.W);
            //newq = KinectHelper.GetShortestRotationBetweenVectors(new Vector3(0, 1, 0), new Vector3(jointQ.X, jointQ.Y, jointQ.Z));
            //newq = Quaternion.Normalize(Quaternion.CreateFromAxisAngle(new Vector3(0, 1, 0), jointQ.W));
            newq = jointQ;


            newq = KinectHelper.RotationBetweenQuaternions(parentQ, jointQ);
            newq = Quaternion.Slerp(newq, newq1, .3f);
            return new OscMessage(address, (body.Position.X + position.X), (body.Position.Y + position.Y), (body.Position.Z + position.Z), newq.W, -newq.X, -newq.Z, newq.Y);




            /*
            //var newQ = new Quaternion(q.X, -q.Y, q.Z, q.W);
            //return new OscMessage(address, (body.Position.X + position.X), (body.Position.Y + position.Y), (body.Position.Z + position.Z), newQ.W, newQ.X, newQ.Y, newQ.Z);
            JointType start = body.BoneOrientations[joint.JointType].StartJoint;
            JointType end = body.BoneOrientations[joint.JointType].EndJoint;

            var parent = body.BoneOrientations[start].AbsoluteRotation.Quaternion;
            var child = body.BoneOrientations[end].AbsoluteRotation.Quaternion;
            var parentQ = Quaternion.Normalize(new Quaternion(parent.X, parent.Y, parent.Z, parent.W));
            var childQ = Quaternion.Normalize(new Quaternion(child.X, child.Y, child.Z, child.W));
            var jointQ = Quaternion.Normalize(new Quaternion(q.X, q.Y, q.Z, q.W));
            //https://social.msdn.microsoft.com/Forums/en-US/3f9e03b4-2670-41b5-9a91-2b72c77fe843/using-kinect-v2-jointorientations-along-with-threejs-skinnedmesh?forum=kinectv2sdk
            var newq =(Quaternion.Inverse(parentQ) * jointQ);
            /*newq=Quaternion.Lerp(newq, jointQ, 1);

            newq=KinectHelper.RotationBetweenQuaternions(parentQ, childQ);
            newq = childQ; // Quaternion.Slerp(newq, jointQ, 1);
            
            //return new OscMessage(address, (body.Position.X + position.X), (body.Position.Y + position.Y), (body.Position.Z + position.Z), newq.W, newq.X,newq.Y, newq.Z);
            */
            


          




            /*Matrix4 m = body.BoneOrientations[joint.JointType].AbsoluteRotation.Matrix;
            Matrix4x4 newM = new Matrix4x4(m.M11, m.M12, m.M13, m.M14, m.M21, m.M22, m.M23, m.M24, m.M31, m.M32, m.M33, m.M34, m.M41, m.M42, m.M43, m.M44);
            Matrix4x4 newOutM = Matrix4x4.CreateRotationY(180);
            Matrix4x4.Invert(newM, out newOutM);
            return new OscMessage(address, (body.Position.X + position.X), (body.Position.Y + position.Y), (body.Position.Z + position.Z), newOutM.M11, newOutM.M12, newOutM.M13,m.M21,m.M22,m.M23,m.M31,m.M32,m.M33);
            */

            /*
            //Vector3 vec0 = sourceValue;
            Vector3 v = KinectHelper.Position(body, joint.JointType);
            Quaternion rot = Quaternion.CreateFromAxisAngle(v, q.W);
            return new OscMessage(address, (body.Position.X + position.X), (body.Position.Y + position.Y), (body.Position.Z + position.Z), -rot.W, -rot.X, rot.Z, rot.Y);
            */


            //return new OscMessage(address, world[0], world[1], world[2], -q.W, -q.X, q.Y, q.Z);


            //


        }

        /*public OscMessage BuildHandMessage(Body body, string key, HandState state, TrackingConfidence confidence)
        {
            var address = String.Format("/bodies/{0}/hands/{1}", body.TrackingId, key);
            //System.Diagnostics.Debug.WriteLine(address);
            return new OscMessage(address, state.ToString(), confidence.ToString());
        }*/
    }
}
