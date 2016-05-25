//------------------------------------------------------------------------------
// <copyright file="BoneOrientationConstraints.cs" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

namespace Microsoft.Samples.Kinect.Avateering.Filters
{
    using System;
    using System.Collections.Generic;
    using Microsoft.Kinect;
    using System.Numerics;   // using Microsoft.Xna.Framework;
    //using Microsoft.Xna.Framework.Graphics;

    /// <summary>
    /// Filter to correct the joint locations and joint orientations to constraint to range of viable human motion.
    /// </summary>
    public class BoneOrientationConstraints 
    {
        /// <summary>
        /// Number of lines to draw in a circle for constraint cones.
        /// </summary>
        private const int Tesselation = 36;

        /// <summary>
        /// Line length scaling for constraint cones.
        /// </summary>
        private const float LineScale = 0.25f;

        /// <summary>
        /// The Joint Constraints.  
        /// </summary>
        private readonly List<BoneOrientationConstraint> jointConstraints = new List<BoneOrientationConstraint>();

        /// <summary>
        /// Set true if the bone constraints are mirrored.  
        /// </summary>
        private bool constraintsMirrored;

       
        /// <summary>
        /// Initializes a new instance of the BoneOrientationConstraints class.
        /// </summary>
        /// <param name="game">The related game object.</param>
        public BoneOrientationConstraints (Vector3 skeletonTranslationScaleFactor)
        {
            this.SkeletonTranslationScaleFactor = skeletonTranslationScaleFactor;

        }

        /// <summary>
        /// The model mesh can be defined at arbitrary non-human size, so re-scale the Kinect translation for drawing the Kinect 3D Skeleton.
        /// </summary>
        private Vector3 SkeletonTranslationScaleFactor { get; set; }

        /// <summary>
        /// AddJointConstraint - Adds a joint constraint to the system.  
        /// </summary>
        /// <param name="joint">The skeleton joint/bone.</param>
        /// <param name="dir">The absolute dir for the center of the constraint cone.</param>
        /// <param name="angle">The angle of the constraint cone that the bone can move in.</param>
        public void AddBoneOrientationConstraint(JointType joint, Vector3 dir, float angle)
        {
            this.constraintsMirrored = false;
            BoneOrientationConstraint jc = new BoneOrientationConstraint(joint, dir, angle);
            this.jointConstraints.Add(jc);
        }

        /// <summary>
        /// AddDefaultConstraints - Adds a set of default joint constraints for normal human poses.  
        /// This is a reasonable set of constraints for plausible human bio-mechanics.
        /// </summary>
        public void AddDefaultConstraints()
        {
            // Constraints indexed on end joint (i.e. constrains the bone between start and end), but constraint applies at the start joint (i.e. the parent)
            // The constraint is defined in the local coordinate system of the parent bone, relative to the parent bone direction 

            // Acts at Hip Center (constrains hip center to spine bone)
            this.AddBoneOrientationConstraint(JointType.Spine, new Vector3(0.0f, 1.0f, 0.3f), 90.0f);

            // Acts at Spine (constrains spine to shoulder center bone)
            this.AddBoneOrientationConstraint(JointType.ShoulderCenter, new Vector3(0.0f, 1.0f, 0.0f), 50.0f);

            // Acts at Shoulder Center (constrains shoulder center to head bone)
            this.AddBoneOrientationConstraint(JointType.Head, new Vector3(0.0f, 1.0f, 0.3f), 45.0f);

            // Acts at Shoulder Joint (constraints shoulder-elbow bone)
            this.AddBoneOrientationConstraint(JointType.ElbowLeft, new Vector3(0.1f, 0.7f, 0.7f), 80.0f);   // along the bone, (i.e +Y), and forward +Z, enable 80 degrees rotation away from this
            this.AddBoneOrientationConstraint(JointType.ElbowRight, new Vector3(-0.1f, 0.7f, 0.7f), 80.0f);

            // Acts at Elbow Joint (constraints elbow-wrist bone)
            this.AddBoneOrientationConstraint(JointType.WristLeft, new Vector3(0.0f, 0.0f, 1.0f), 90.0f);   // +Z (i.e. so rotates up or down with twist, when arm bent, stop bending backwards)
            this.AddBoneOrientationConstraint(JointType.WristRight, new Vector3(0.0f, 0.0f, 1.0f), 90.0f);

            // Acts at Wrist Joint (constrains wrist-hand bone)
            this.AddBoneOrientationConstraint(JointType.HandLeft, new Vector3(0.0f, 1.0f, 0.0f), 45.0f);    // +Y is along the bone
            this.AddBoneOrientationConstraint(JointType.HandRight, new Vector3(0.0f, 1.0f, 0.0f), 45.0f);

            // Acts at Hip Joint (constrains hip-knee bone)
            this.AddBoneOrientationConstraint(JointType.KneeLeft, new Vector3(0.5f, 0.7f, 0.1f), 65.0f);
            this.AddBoneOrientationConstraint(JointType.KneeRight, new Vector3(-0.5f, 0.7f, 0.1f), 65.0f);

            // Acts at Knee Joint (constrains knee-ankle bone)
            this.AddBoneOrientationConstraint(JointType.AnkleRight, new Vector3(0.0f, 0.7f, -1.0f), 65.0f); // enable bending backwards with -Z
            this.AddBoneOrientationConstraint(JointType.AnkleLeft, new Vector3(0.0f, 0.7f, -1.0f), 65.0f);

            // Acts at Ankle Joint (constrains ankle-foot bone)
            this.AddBoneOrientationConstraint(JointType.FootRight, new Vector3(0.0f, 0.3f, 0.5f), 40.0f);
            this.AddBoneOrientationConstraint(JointType.FootLeft, new Vector3(0.0f, 0.3f, 0.5f), 40.0f);
        }

        /// <summary>
        /// ApplyBoneOrientationConstraints and constrain rotations.
        /// </summary>
        /// <param name="skeleton">The skeleton to correct.</param>
        /// <param name="mirrorView">Set this true if the skeleton joints are mirrored.</param>
        public void Constrain(Skeleton skeleton, bool mirrorView)
        {
            if (null == skeleton || skeleton.TrackingState != SkeletonTrackingState.Tracked)
            {
                return;
            }

            if (this.jointConstraints.Count == 0)
            {
                this.AddDefaultConstraints();
            }

            if (mirrorView != this.constraintsMirrored)
            {
                this.MirrorConstraints();
            }

            // Constraints are defined as a vector with respect to the parent bone vector, and a constraint angle, 
            // which is the maximum angle with respect to the constraint axis that the bone can move through.

            // Calculate constraint values. 0.0-1.0 means the bone is within the constraint cone. Greater than 1.0 means 
            // it lies outside the constraint cone.
            for (int i = 0; i < this.jointConstraints.Count; i++)
            {
                BoneOrientationConstraint jc = this.jointConstraints[i];

                if (skeleton.Joints[jc.Joint].TrackingState == JointTrackingState.NotTracked || jc.Joint == JointType.HipCenter) 
                {
                    // End joint is not tracked or Hip Center has no parent to perform this calculation with.
                    continue;
                }

                // If the joint has a parent, constrain the bone direction to be within the constraint cone
                JointType parentIdx = skeleton.BoneOrientations[jc.Joint].StartJoint;

                // Local bone orientation relative to parent
                Matrix4 boneOrientationRelativeToParent = KinectHelper.Matrix4ToXNAMatrix(skeleton.BoneOrientations[jc.Joint].HierarchicalRotation.Matrix);
                Quaternion boneOrientationRelativeToParentQuat = KinectHelper.Vector4ToXNAQuaternion(skeleton.BoneOrientations[jc.Joint].HierarchicalRotation.Quaternion);

                // Local bone direction is +Y vector in parent coordinate system
                Vector3 boneRelDirVecLs = new Vector3(boneOrientationRelativeToParent.M21, boneOrientationRelativeToParent.M22, boneOrientationRelativeToParent.M23);
                boneRelDirVecLs=Vector3.Normalize(boneRelDirVecLs);

                // Constraint is relative to the parent orientation, which is +Y/identity relative rotation
                Vector3 constraintDirLs = jc.Dir;
                constraintDirLs = Vector3.Normalize(constraintDirLs);

                // Test this against the vector of the bone to find angle
                float boneDotConstraint = Vector3.Dot(boneRelDirVecLs, constraintDirLs);

                // Calculate the constraint value. 0.0 is in the center of the constraint cone, 1.0 and above are outside the cone.
                jc.Constraint = (float)Math.Acos(boneDotConstraint) / ConvertToRadians(jc.Angle);

                this.jointConstraints[i] = jc;

                // Slerp between identity and the inverse of the current constraint rotation by the amount over the constraint amount
                if (jc.Constraint > 1)
                {
                    Quaternion inverseRotation = Quaternion.Inverse(boneOrientationRelativeToParentQuat);
                    Quaternion slerpedInverseRotation = Quaternion.Slerp(Quaternion.Identity, inverseRotation, jc.Constraint - 1);
                    Quaternion constrainedRotation = boneOrientationRelativeToParentQuat * slerpedInverseRotation;

                    // Put it back into the bone orientations
                    skeleton.BoneOrientations[jc.Joint].HierarchicalRotation.Quaternion = KinectHelper.XNAQuaternionToVector4(constrainedRotation);
                    skeleton.BoneOrientations[jc.Joint].HierarchicalRotation.Matrix = KinectHelper.XNAMatrixToMatrix4(Matrix4x4.CreateFromQuaternion(constrainedRotation));
                }
            }

            // Recalculate the absolute rotations from the hierarchical relative rotations
            Array jointTypeValues = Enum.GetValues(typeof(JointType));

            foreach (JointType j in jointTypeValues)
            {
                if (j != JointType.HipCenter)
                {
                    JointType parentIdx = skeleton.BoneOrientations[j].StartJoint;

                    // Calculate the absolute/world equivalents of the hierarchical rotation
                    Quaternion parentRotation = KinectHelper.Vector4ToXNAQuaternion(skeleton.BoneOrientations[parentIdx].AbsoluteRotation.Quaternion);
                    Quaternion relativeRotation = KinectHelper.Vector4ToXNAQuaternion(skeleton.BoneOrientations[j].HierarchicalRotation.Quaternion);

                    // Create a new world rotation
                    Quaternion worldRotation = Quaternion.Multiply(parentRotation, relativeRotation);

                    skeleton.BoneOrientations[j].AbsoluteRotation.Quaternion = KinectHelper.XNAQuaternionToVector4(worldRotation);
                    skeleton.BoneOrientations[j].AbsoluteRotation.Matrix = KinectHelper.XNAMatrixToMatrix4(Matrix4x4.CreateFromQuaternion(worldRotation));
                }
            }
        }

        /// <summary>
        /// Helper method to swap mirror the skeleton bone constraints when the skeleton is mirrored.
        /// </summary>
        private void MirrorConstraints()
        {
            this.SwapJointTypes(JointType.ShoulderLeft, JointType.ShoulderRight);
            this.SwapJointTypes(JointType.ElbowLeft, JointType.ElbowRight);
            this.SwapJointTypes(JointType.WristLeft, JointType.WristRight);
            this.SwapJointTypes(JointType.HandLeft, JointType.HandRight);

            this.SwapJointTypes(JointType.HipLeft, JointType.HipRight);
            this.SwapJointTypes(JointType.KneeLeft, JointType.KneeRight);
            this.SwapJointTypes(JointType.AnkleLeft, JointType.AnkleRight);
            this.SwapJointTypes(JointType.FootLeft, JointType.FootRight);

            for (int i = 0; i < this.jointConstraints.Count; i++)
            {
                BoneOrientationConstraint jc = this.jointConstraints[i];

                // Here we negate the X axis to change the skeleton to mirror the user's movements.
                jc.Dir.X = -jc.Dir.X;

                this.jointConstraints[i] = jc;
            }

            this.constraintsMirrored = !this.constraintsMirrored;
        }

        /// <summary>
        /// Helper method to swap two joint types in the skeleton when mirroring the avatar.
        /// </summary>
        /// <param name="left">The left joint type.</param>
        /// <param name="right">The right joint type.</param>
        private void SwapJointTypes(JointType left, JointType right)
        {
            for (int i = 0; i < this.jointConstraints.Count; i++)
            {
                BoneOrientationConstraint jc = this.jointConstraints[i];

                if (jc.Joint == left)
                {
                    jc.Joint = right;
                }
                else if (jc.Joint == right)
                {
                    jc.Joint = left;
                }

                this.jointConstraints[i] = jc;
            }
        }

        public float ConvertToRadians(float angle)
        {
            return (float)(Math.PI / 180) * angle;
        }


        /// <summary>
        /// Joint Constraint structure to hold the constraint axis, angle and cone direction and associated joint.
        /// </summary>
        private struct BoneOrientationConstraint
        {
            /// <summary>
            /// Constraint cone direction
            /// </summary>
            public Vector3 Dir;

            /// <summary>
            /// Skeleton joint
            /// </summary>
            public JointType Joint;

            /// <summary>
            /// Constraint cone angle
            /// </summary>
            public float Angle;

            /// <summary>
            /// Calculated dynamic value of constraint
            /// </summary>
            public float Constraint;

            /// <summary>
            /// Initializes a new instance of the <see cref="BoneOrientationConstraint"/> struct.
            /// </summary>
            /// <param name="joint">The joint/bone the constraint refers to.</param>
            /// <param name="dir">The constraint cone center direction.</param>
            /// <param name="angle">The constraint cone angle from the center direction.</param>
            public BoneOrientationConstraint(JointType joint, Vector3 dir, float angle)
            {
                this.Joint = joint;
                this.Dir = dir;
                this.Angle = angle;
                this.Constraint = 0;
            }
        }
    }
}
