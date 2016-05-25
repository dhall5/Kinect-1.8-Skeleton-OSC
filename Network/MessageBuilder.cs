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
            string jointName = joint.JointType.ToString();
            if (jointName.IndexOf("Left") >= 0)
            {
                jointName = jointName.Replace("Left", "Right");
            }
            else if (jointName.IndexOf("Right") >= 0)
            {
                jointName = jointName.Replace("Right", "Left");
            }
            
            var jointRotation = body.BoneOrientations[joint.JointType].AbsoluteRotation.Matrix;
            var position = body.Joints[joint.JointType].Position;
            var rotation = body.BoneOrientations[joint.JointType].AbsoluteRotation.Matrix;
            var qrotation = body.BoneOrientations[joint.JointType].AbsoluteRotation.Quaternion;
  
            Quaternion qOrientation = new Quaternion(qrotation.X, qrotation.Y, qrotation.Z, qrotation.W);

            Matrix4x4 mjointRot = new Matrix4x4(jointRotation.M11, jointRotation.M12, jointRotation.M13, jointRotation.M14, jointRotation.M21, jointRotation.M22, jointRotation.M23, jointRotation.M24, jointRotation.M31, jointRotation.M32, jointRotation.M33, jointRotation.M34, jointRotation.M41, jointRotation.M42, jointRotation.M43, jointRotation.M44);


            //System.Numerics.Vector4 v = new System.Numerics.Vector4(qrotation.X, qrotation.Y, qrotation.Z, qrotation.W);
            //Plane pln = new Plane(v);
            //Matrix4x4 reflectionParent = System.Numerics.Matrix4x4.CreateReflection(pln);
            //qSend = Quaternion.CreateFromRotationMatrix(reflectionParent);
            //qSend.W = qrotation.W;

            var flipMat = new Matrix4x4(1, 0, 0, 0,
                                       0, -1, 0, 0,
                                       0, 0, 1, 0,
                                       0, 0, 0, 1
                                   );
            mjointRot = flipMat * mjointRot * flipMat;



            var address = String.Format("/{0}", joint.JointType);
            if (KinectHelper.BoneOrientationIsValid(qOrientation))
            {
                Quaternion qSend;
                qSend =  Quaternion.CreateFromRotationMatrix(mjointRot);
                
                return new OscMessage(address, (body.Position.X + position.X), (body.Position.Y + position.Y), (body.Position.Z + position.Z), qSend.W, qSend.X, qSend.Y, qSend.Z);

            }
            return new OscMessage(address, (body.Position.X + position.X), (body.Position.Y + position.Y), (body.Position.Z + position.Z));

            

        }




        /*public OscMessage BuildHandMessage(Body body, string key, HandState state, TrackingConfidence confidence)
        {
            var address = String.Format("/bodies/{0}/hands/{1}", body.TrackingId, key);
            //System.Diagnostics.Debug.WriteLine(address);
            return new OscMessage(address, state.ToString(), confidence.ToString());
        }*/

    }
}

