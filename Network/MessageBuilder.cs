using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Microsoft.Kinect;
using Rug.Osc;


namespace Microsoft.Samples.Kinect.SkeletonBasics.Network.BodySender
{
    public class MessageBuilder
    {


        private List<double>  depthToWorld(float x, float y, float z)
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

        public OscMessage BuildJointMessage(Skeleton body,  Joint joint)
        {
         
            //https://msdn.microsoft.com/en-us/library/hh973073.aspx
            var address = String.Format("/{0}", joint.JointType);
            var position = joint.Position;


            var q = body.BoneOrientations[joint.JointType].AbsoluteRotation.Quaternion;
            List<double> world = depthToWorld((body.Position.X + position.X), (body.Position.Y + position.Y), (body.Position.Z + position.Z));


            //return new OscMessage(address, world[0], world[1], world[2], -q.W, -q.X, q.Y, q.Z);


            return new OscMessage(address, (body.Position.X+position.X),(body.Position.Y+position.Y),(body.Position.Z+position.Z), q.W, q.X, q.Y, q.Z);

            //return new OscMessage(address, (body.Position.X + position.X), (body.Position.Y + position.Y), (body.Position.Z + position.Z), m.M11,m.M12,m.M13,m.M21,m.M22,m.M23,m.M31,m.M32,m.M33);


            //boneOrientations[JointType.HipRight].HierarchicalRotation
            //var address = String.Format("/bodies/{0}/joints/{1}", body.TrackingId, joint.JointType);



            //System.Diagnostics.Debug.WriteLine(address);

        }

        /*public OscMessage BuildHandMessage(Body body, string key, HandState state, TrackingConfidence confidence)
        {
            var address = String.Format("/bodies/{0}/hands/{1}", body.TrackingId, key);
            //System.Diagnostics.Debug.WriteLine(address);
            return new OscMessage(address, state.ToString(), confidence.ToString());
        }*/
    }
}
