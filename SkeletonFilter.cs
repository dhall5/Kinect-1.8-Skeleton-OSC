using Microsoft.Kinect;
using Microsoft.Samples.Kinect.Avateering.Filters;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

namespace Microsoft.Samples.Kinect.SkeletonBasics
{
    class SkeletonFilter
    {
        /// <summary>
        /// Initializes a new instance of the SkeletonFilter class.
        /// </summary>
        public SkeletonFilter(KinectSensor kinectSensor)
        {
            sensor = kinectSensor;



             this.SkeletonDrawn = true;
            this.AvatarHipCenterHeight = 0;

            this.drawBoneConstraintsSkeleton = false;

            // Skeleton fixups
            this.frameTimer = new Timer();
            this.lastNuiTime = 0;
            this.FloorClipPlane = new Tuple<float, float, float, float>(0, 0, 0, 0);
            this.clippedLegs = new SkeletonJointsFilterClippedLegs();
            this.sensorOffsetCorrection = new SkeletonJointsSensorOffsetCorrection();
            this.jointPositionFilter = new SkeletonJointsPositionDoubleExponentialFilter();
            this.boneOrientationConstraints = new BoneOrientationConstraints(this.SkeletonTranslationScaleFactor);
            //this.boneOrientationFilter = new BoneOrientationDoubleExponentialFilter();

            this.filterClippedLegs = true;
            this.tiltCompensate = true;
            this.floorOffsetCompensate = false;
            this.selfIntersectionConstraints = true;
            this.mirrorView = false;
            this.boneConstraints = true;
            this.filterBoneOrientations = true;

            // For many applications we would enable the
            // automatic joint smoothing, however, in this
            // Avateering sample, we perform skeleton joint
            // position corrections, so we will manually
            // filter here after these are complete.

            // Typical smoothing parameters for the joints:
            var jointPositionSmoothParameters = new TransformSmoothParameters
            {
                Smoothing = 0.25f,
                Correction = 0.25f,
                Prediction = 0.75f,
                JitterRadius = 0.1f,
                MaxDeviationRadius = 0.04f
            };

            this.jointPositionFilter.Init(jointPositionSmoothParameters);

            // Setup the bone orientation constraint system
            this.boneOrientationConstraints.AddDefaultConstraints();
            //game.Components.Add(this.boneOrientationConstraints);

            // Typical smoothing parameters for the bone orientations:
            //var boneOrientationSmoothparameters = new TransformSmoothParameters
            //{
            //    Smoothing = 0.5f,
            //    Correction = 0.8f,
            //    Prediction = 0.75f,
            //    JitterRadius = 0.1f,
            //    MaxDeviationRadius = 0.1f
            //};

            //this.boneOrientationFilter.Init(boneOrientationSmoothparameters);


        }
        /// <summary>
        /// This skeleton is the  first tracked skeleton in the frame is used for animation, with constraints and mirroring optionally applied.
        /// </summary>
        private Skeleton skeleton;

        /// <summary>
        /// Compensate the avatar joints for sensor tilt if true.
        /// </summary>
        private bool tiltCompensate;

        /// <summary>
        /// Compensate the avatar joints for sensor height and bring skeleton to floor level if true.
        /// </summary>
        private bool floorOffsetCompensate;


        /// <summary>
        /// The model mesh can be defined at arbitrary non-human size, so re-scale the Kinect translation for drawing the Kinect 3D Skeleton.
        /// </summary>
        private Vector3 SkeletonTranslationScaleFactor { get; set; }


        /// <summary>
        /// Filter to compensate the avatar joints for sensor height and bring skeleton to floor level.
        /// </summary>
        private SkeletonJointsSensorOffsetCorrection sensorOffsetCorrection;

        /// <summary>
        /// Filter to prevent arm-torso self-intersections if true.
        /// </summary>
        private bool selfIntersectionConstraints;

        /// <summary>
        /// The timer for controlling Filter Lerp blends.
        /// </summary>
        private Timer frameTimer;

        /// <summary>
        /// The timer for controlling Filter Lerp blends.
        /// </summary>
        private float lastNuiTime;

        /// <summary>
        /// Filter clipped legs if true.
        /// </summary>
        private bool filterClippedLegs;

        /// <summary>
        /// The filter for clipped legs.
        /// </summary>
        private SkeletonJointsFilterClippedLegs clippedLegs;

        /// <summary>
        /// Mirrors the avatar when true.
        /// </summary>
        private bool mirrorView;

        /// <summary>
        /// Apply joint constraints to joint locations and orientations if true.
        /// </summary>
        private bool boneConstraints;

        /// <summary>
        /// The filter for bone orientations constraints.
        /// </summary>
        private BoneOrientationConstraints boneOrientationConstraints;

        /// <summary>
        /// Draw the Kinect line skeleton using the raw joint positions and joint constraint cones if true.
        /// </summary>
        private bool drawBoneConstraintsSkeleton;

       
        /// <summary>
        /// The filter for joint positions.
        /// </summary>
        private SkeletonJointsPositionDoubleExponentialFilter jointPositionFilter;

        /// <summary>
        /// Filter bone orientations if true.
        /// </summary>
        private bool filterBoneOrientations;

        /// <summary>
        /// The filter for bone orientations.
        /// </summary>
        //private BoneOrientationDoubleExponentialFilter boneOrientationFilter;


        /// <summary>
        /// Gets or sets a value indicating whether the model has been drawn - this flag ensures we only request a frame once per update call
        /// across the entire application.
        /// </summary>
        public bool SkeletonDrawn { get; set; }

        /// <summary>
        /// Gets or sets a value indicating whether this model is visible - this flag ensures we only update if there is visible data.
        /// </summary>
        public bool SkeletonVisible { get; set; }

        /// <summary>
        /// Gets or sets a value indicating whether the first tracked skeleton in the frame is used for animation.
        /// </summary>
        public Skeleton RawSkeleton { get; set; }

        /// <summary>
        /// Gets or sets a value indicating whether Store the floor plane to compensate the skeletons for any Kinect tilt.
        /// </summary>
        public System.Tuple<float, float, float, float> FloorClipPlane { get; set; }

        /// <summary>
        /// Gets or sets a value indicating whether the height of the avatar Hip Center joint off the floor when standing upright.
        /// </summary>
        public float AvatarHipCenterHeight { get; set; }


        /// <summary>
        /// Active Kinect sensor
        /// </summary>
        private KinectSensor sensor;

        /// <summary>
        /// This method retrieves a new skeleton frame if necessary.
        /// </summary>
        /// <param name="gameTime">The elapsed game time.</param>
        public Skeleton Filter(Skeleton skel)
        {
            this.skeleton = skel;

            // If we have already drawn this skeleton, then we should retrieve a new frame
            // This prevents us from calling the next frame more than once per update
            if (null != this.skeleton)
            {
                

                // If required, we should modify the joint positions before we access the bone orientations, as orientations are calculated
                // on the first access, and then whenever a joint position changes. Hence changing joint positions interleaved with accessing
                // rotations will cause unnecessary additional computation.
                float currentNuiTime = (float)this.frameTimer.AbsoluteTime;
                float deltaNuiTime = currentNuiTime - this.lastNuiTime;

                // Fixup Skeleton to improve avatar appearance.
                if (this.filterClippedLegs && null != this.clippedLegs)
                {
                    this.clippedLegs.FilterSkeleton(this.skeleton, deltaNuiTime);
                }

                if (this.selfIntersectionConstraints)
                {
                    // Constrain the wrist and hand joint positions to not intersect the torso
                    SkeletonJointsSelfIntersectionConstraint.Constrain(this.skeleton);
                }

                if (this.tiltCompensate)
                {
                    int elevationAngle = 0;
                    try
                    {
                        elevationAngle = this.sensor.ElevationAngle;
                    }
                    catch (InvalidOperationException)
                    {
                        // Ignore any exception accessing the sensor if it has just been disconnected
                    }

                    // Correct for sensor tilt if we have a valid floor plane or a sensor tilt value from the motor.
                    SkeletonJointsSensorTiltCorrection.CorrectSensorTilt(this.skeleton, this.FloorClipPlane, elevationAngle);
                }

                if (this.floorOffsetCompensate && 0.0f != this.AvatarHipCenterHeight)
                {
                    // Correct for the sensor height from the floor (moves the skeleton to the floor plane) if we have a valid plane, or feet visible in the image.
                    // Note that by default this will not run unless we have set a non-zero AvatarHipCenterHeight
                    this.sensorOffsetCorrection.CorrectSkeletonOffsetFromFloor(this.skeleton, this.FloorClipPlane, this.AvatarHipCenterHeight);
                }

                if (this.mirrorView)
                {
                    SkeletonJointsMirror.MirrorSkeleton(this.skeleton);
                }

                // Filter the joint positions manually, using a double exponential filter.
                this.jointPositionFilter.UpdateFilter(this.skeleton);

                if (this.boneConstraints && null != this.boneOrientationConstraints)
                {
                    // Constrain the joint positions to approximate range of human motion.
                    this.boneOrientationConstraints.Constrain(this.skeleton, this.mirrorView);
                }

                /*if (this.filterBoneOrientations && null != this.boneOrientationFilter)
                {
                    // Double Exponential Filtering of the joint orientations.
                    // Note: This updates the joint orientations directly in the skeleton.
                    // It should be performed after all joint position modifications.
                    this.boneOrientationFilter.UpdateFilter(this.skeleton);
                }
                */
               

                this.lastNuiTime = currentNuiTime;
            }

            return skel;
        }

    }
}
