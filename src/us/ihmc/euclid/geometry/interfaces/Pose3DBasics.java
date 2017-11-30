package us.ihmc.euclid.geometry.interfaces;

import us.ihmc.euclid.axisAngle.interfaces.AxisAngleReadOnly;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.interfaces.Clearable;
import us.ihmc.euclid.interfaces.Transformable;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.tools.QuaternionTools;
import us.ihmc.euclid.tools.RotationMatrixTools;
import us.ihmc.euclid.transform.QuaternionBasedTransform;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

public interface Pose3DBasics extends Pose3DReadOnly, Transformable, Clearable
{
   /**
    * Sets the x-coordinate of the position.
    *
    * @param x the x-coordinate of the position.
    */
   default void setX(double x)
   {
      getPosition().setX(x);
   }

   /**
    * Sets the y-coordinate of the position.
    *
    * @param y the y-coordinate of the position.
    */
   default void setY(double y)
   {
      getPosition().setY(y);
   }

   /**
    * Sets the z-coordinate of the position.
    *
    * @param z the z-coordinate of the position.
    */
   default void setZ(double z)
   {
      getPosition().setZ(z);
   }

   /**
    * Gets the reference of the position part of this pose 3D.
    *
    * @return the position part of this pose 3D.
    */
   Point3DBasics getPosition();

   /**
    * Gets the reference to the orientation part of this pose 3D.
    *
    * @return the orientation part of this pose 3D.
    */
   QuaternionBasics getOrientation();
   
   /**
    * Sets the position coordinates.
    *
    * @param x the x-coordinate of the position.
    * @param y the y-coordinate of the position.
    * @param z the z-coordinate of the position.
    */
   default void setPosition(double x, double y, double z)
   {
      getPosition().set(x, y, z);
   }

   /**
    * Sets the position to the given tuple.
    *
    * @param position the tuple with the new position coordinates. Not modified.
    */
   default void setPosition(Tuple3DReadOnly position)
   {
      setPosition(position.getX(), position.getY(), position.getZ());
   }

   /**
    * Sets the x and y coordinates from the given tuple 2D, the z coordinate remains unchanged.
    *
    * @param position2D the tuple with the new x and y coordinates. Not modified.
    */
   default void setPosition(Tuple2DReadOnly position2D)
   {
      getPosition().set(position2D);
   }

   /**
    * Sets the position from the given tuple 2D and z coordinate.
    *
    * @param position2D the tuple with the new x and y coordinates. Not modified.
    * @param z the new z value.
    */
   default void setPosition(Tuple2DReadOnly position2D, double z)
   {
      getPosition().set(position2D, z);
   }

   /**
    * Sets all the components of this pose 3D.
    * <p>
    * WARNING: the Euler angles or yaw-pitch-roll representation is sensitive to gimbal lock and is
    * sometimes undefined.
    * </p>
    *
    * @param x the x-coordinate of the position.
    * @param y the y-coordinate of the position.
    * @param z the z-coordinate of the position.
    * @param yaw the angle to rotate about the z-axis.
    * @param pitch the angle to rotate about the y-axis.
    * @param roll the angle to rotate about the x-axis.
    */
   default void set(double x, double y, double z, double yaw, double pitch, double roll)
   {
      setPosition(x, y, z);
      setOrientationYawPitchRoll(yaw, pitch, roll);
   }

   /**
    * Sets this pose 3D to the {@code other} pose 3D.
    *
    * @param other the other pose 3D. Not modified.
    */
   default void set(Pose3DReadOnly other)
   {
      setPosition(other.getPosition());
      setOrientation(other.getOrientation());
   }

   /**
    * Sets the orientation part of this pose 3D with the 4 components of a quaternion.
    * <p>
    * The quaternion is normalized internally.
    * </p>
    *
    * @param qx the x-component of the quaternion's vector part.
    * @param qy the y-component of the quaternion's vector part.
    * @param qz the z-component of the quaternion's vector part.
    * @param qs the scalar component of the quaternion.
    */
   default void setOrientation(double qx, double qy, double qz, double qs)
   {
      getOrientation().set(qx, qy, qz, qs);
   }

   /**
    * Sets the orientation part of this pose 3D with the given quaternion.
    *
    * @param orientation the quaternion used to set this pose's orientation. Not modified.
    */
   default void setOrientation(QuaternionReadOnly orientation)
   {
      getOrientation().set(orientation);
   }

   /**
    * Sets the orientation part of this pose 3D with the given rotation matrix.
    *
    * @param orientation the rotation matrix used to set this pose's orientation. Not modified.
    */
   default void setOrientation(RotationMatrixReadOnly orientation)
   {
      getOrientation().set(orientation);
   }

   /**
    * Sets the orientation part of this pose 3D with the given axis-angle.
    *
    * @param orientation the axis-angle used to set this pose's orientation. Not modified.
    */
   default void setOrientation(AxisAngleReadOnly orientation)
   {
      getOrientation().set(orientation);
   }

   /**
    * Sets the orientation part of this pose 3D with the given yaw, pitch, and roll angles.
    * <p>
    * WARNING: the Euler angles or yaw-pitch-roll representation is sensitive to gimbal lock and is
    * sometimes undefined.
    * </p>
    *
    * @param yawPitchRoll array containing the yaw-pitch-roll angles. Not modified.
    */
   default void setOrientationYawPitchRoll(double[] yawPitchRoll)
   {
      getOrientation().setYawPitchRoll(yawPitchRoll);
   }

   /**
    * Sets the orientation part of this pose 3D with the given yaw, pitch, and roll angles.
    * <p>
    * WARNING: the Euler angles or yaw-pitch-roll representation is sensitive to gimbal lock and is
    * sometimes undefined.
    * </p>
    *
    * @param yaw the angle to rotate about the z-axis.
    * @param pitch the angle to rotate about the y-axis.
    * @param roll the angle to rotate about the x-axis.
    */
   default void setOrientationYawPitchRoll(double yaw, double pitch, double roll)
   {
      getOrientation().setYawPitchRoll(yaw, pitch, roll);
   }

   /**
    * Sets this pose 3D to match the given rigid-body transform.
    *
    * @param rigidBodyTransform the transform use to set this pose 3D. Not modified.
    */
   default void set(RigidBodyTransform rigidBodyTransform)
   {
      setPosition(rigidBodyTransform.getTranslationVector());
      setOrientation(rigidBodyTransform.getRotationMatrix());
   }

   /**
    * Sets this pose 3D to match the given quaternion-based transform.
    *
    * @param quaternionBasedTransform the transform use to set this pose 3D. Not modified.
    */
   default void set(QuaternionBasedTransform quaternionBasedTransform)
   {
      setPosition(quaternionBasedTransform.getTranslationVector());
      setOrientation(quaternionBasedTransform.getQuaternion());
   }

   /**
    * Sets both position and orientation.
    *
    * @param position the tuple with the new position coordinates. Not modified.
    * @param orientation the quaternion with the new orientation. Not modified.
    */
   default void set(Tuple3DReadOnly position, QuaternionReadOnly orientation)
   {
      setOrientation(orientation);
      setPosition(position);
   }

   /**
    * Sets both position and orientation.
    *
    * @param position the tuple with the new position coordinates. Not modified.
    * @param orientation the axis-angle with the new orientation. Not modified.
    */
   default void set(Tuple3DReadOnly position, AxisAngleReadOnly orientation)
   {
      setOrientation(orientation);
      setPosition(position);
   }

   /** {@inheritDoc} */
   @Override
   default boolean containsNaN()
   {
      return getOrientation().containsNaN() || getPosition().containsNaN();
   }

   /** {@inheritDoc} */
   @Override
   default void setToNaN()
   {
      getOrientation().setToNaN();
      getPosition().setToNaN();
   }

   /**
    * Sets the position to (0, 0) and the orientation to the neutral quaternion, i.e. zero rotation.
    */
   @Override
   default void setToZero()
   {
      getOrientation().setToZero();
      getPosition().setToZero();
   }

   /**
    * Normalizes the quaternion part of this pose to ensure it is a unit-quaternion describing a
    * proper orientation.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the quaternion contains {@link Double#NaN}, this method is ineffective.
    * </ul>
    * </p>
    */
   default void normalizeQuaternion()
   {
      getOrientation().normalize();
   }

   /**
    * Normalizes the quaternion part of this pose and then limits the angle of the rotation it
    * represents to be &in; [-<i>pi</i>;<i>pi</i>].
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the quaternion contains {@link Double#NaN}, this method is ineffective.
    * </ul>
    * </p>
    */
   default void normalizeQuaternionAndLimitToPi()
   {
      getOrientation().normalizeAndLimitToPi();
   }

   /**
    * Performs a linear interpolation from {@code this} to {@code other} given the percentage
    * {@code alpha}.
    * <p>
    * this.position = (1.0 - alpha) * this.position + alpha * other.position<br>
    * this.orientation = (1.0 - alpha) * this.orientation + alpha * other.orientation
    * </p>
    *
    * @param other the other pose 3D used for the interpolation. Not modified.
    * @param alpha the percentage used for the interpolation. A value of 0 will result in not
    *           modifying {@code this}, while a value of 1 is equivalent to setting {@code this} to
    *           {@code other}.
    */
   default void interpolate(Pose3D other, double alpha)
   {
      getPosition().interpolate(other.getPosition(), alpha);
      getOrientation().interpolate(other.getOrientation(), alpha);
   }

   /**
    * Performs a linear interpolation from {@code pose1} to {@code pose2} given the percentage
    * {@code alpha}.
    * <p>
    * this.position = (1.0 - alpha) * pose1.position + alpha * pose2.position<br>
    * this.orientation = (1.0 - alpha) * pose1.orientation + alpha * pose2.orientation
    * </p>
    *
    * @param pose1 the first pose 3D used in the interpolation. Not modified.
    * @param pose2 the second pose 3D used in the interpolation. Not modified.
    * @param alpha the percentage to use for the interpolation. A value of 0 will result in setting
    *           {@code this} to {@code pose1}, while a value of 1 is equivalent to setting
    *           {@code this} to {@code pose2}.
    */
   default void interpolate(Pose3D pose1, Pose3D pose2, double alpha)
   {
      getPosition().interpolate(pose1.getPosition(), pose2.getPosition(), alpha);
      getOrientation().interpolate(pose1.getOrientation(), pose2.getOrientation(), alpha);
   }

   /**
    * Adds the translation (x, y, z) to this pose 3D assuming it is expressed in the coordinates in
    * which this pose is expressed.
    * <p>
    * If the translation is expressed in the local coordinates described by this pose 3D, use
    * {@link #appendTranslation(double, double, double)}.
    * </p>
    *
    * @param x the translation distance along the x-axis.
    * @param y the translation distance along the y-axis.
    * @param z the translation distance along the z-axis.
    */
   default void prependTranslation(double x, double y, double z)
   {
      getPosition().add(x, y, z);
   }

   /**
    * Adds the given {@code translation} to this pose 3D assuming it is expressed in the coordinates
    * in which this pose is expressed.
    * <p>
    * If the {@code translation} is expressed in the local coordinates described by this pose 3D,
    * use {@link #appendTranslation(Tuple3DReadOnly)}.
    * </p>
    *
    * @param translation tuple containing the translation to apply to this pose 3D. Not modified.
    */
   default void prependTranslation(Tuple3DReadOnly translation)
   {
      prependTranslation(translation.getX(), translation.getY(), translation.getZ());
   }

   /**
    * Rotates the position part of this pose 3D by the given {@code rotation} and prepends it to the
    * orientation part.
    *
    * @param rotation the rotation to prepend to this pose 3D. Not modified.
    */
   default void prependRotation(QuaternionReadOnly rotation)
   {
      rotation.transform(getPosition());
      rotation.transform(getOrientation());
   }

   /**
    * Rotates the position part of this pose 3D by the given {@code rotation} and prepends it to the
    * orientation part.
    *
    * @param rotation the rotation to prepend to this pose 3D. Not modified.
    */
   default void prependRotation(RotationMatrixReadOnly rotation)
   {
      rotation.transform(getPosition());
      rotation.transform(getOrientation());
   }

   /**
    * Rotates the position part of this pose 3D by the given {@code rotation} and prepends it to the
    * orientation part.
    *
    * @param rotation the rotation to prepend to this pose 3D. Not modified.
    */
   default void prependRotation(AxisAngleReadOnly rotation)
   {
      rotation.transform(getPosition());
      rotation.transform(getOrientation());
   }

   /**
    * Prepends a rotation about the z-axis to this pose 3D: Rotates the position part and prepends
    * the rotation to the orientation part.
    *
    * @param yaw the angle to rotate about the z-axis.
    */
   default void prependYawRotation(double yaw)
   {
      RotationMatrixTools.applyYawRotation(yaw, getPosition(), getPosition());
      getOrientation().prependYawRotation(yaw);
   }

   /**
    * Prepends a rotation about the y-axis to this pose 3D: Rotates the position part and prepends
    * the rotation to the orientation part.
    *
    * @param pitch the angle to rotate about the y-axis.
    */
   default void prependPitchRotation(double pitch)
   {
      RotationMatrixTools.applyPitchRotation(pitch, getPosition(), getPosition());
      getOrientation().prependPitchRotation(pitch);
   }

   /**
    * Prepends a rotation about the x-axis to this pose 3D: Rotates the position part and prepends
    * the rotation to the orientation part.
    *
    * @param roll the angle to rotate about the x-axis.
    */
   default void prependRollRotation(double roll)
   {
      RotationMatrixTools.applyRollRotation(roll, getPosition(), getPosition());
      getOrientation().prependRollRotation(roll);
   }

   /**
    * Prepends the given transform to this pose 3D.
    * <p>
    * This is the same as {@link #applyTransform(Transform)}.
    * </p>
    *
    * @param transform the transform to prepend to this pose 3D. Not modified.
    */
   default void prependTransform(RigidBodyTransform transform)
   {
      applyTransform(transform);
   }

   /**
    * Prepends the given transform to this pose 3D.
    * <p>
    * This is the same as {@link #applyTransform(Transform)}.
    * </p>
    *
    * @param transform the transform to prepend to this pose 3D. Not modified.
    */
   default void prependTransform(QuaternionBasedTransform transform)
   {
      applyTransform(transform);
   }

   /**
    * Rotates, then adds the translation (x, y, z) to this pose 3D.
    * <p>
    * Use this method if the translation (x, y, z) is expressed in the local coordinates described
    * by this pose 3D. Otherwise, use {@link #prependTranslation(double, double, double)}.
    * </p>
    *
    * @param x the translation distance along the x-axis.
    * @param y the translation distance along the y-axis.
    * @param z the translation distance along the z-axis.
    */
   default void appendTranslation(double x, double y, double z)
   {
      double thisX = getX();
      double thisY = getY();
      double thisZ = getZ();

      setPosition(x, y, z);
      getOrientation().transform(getPosition());
      getPosition().add(thisX, thisY, thisZ);
   }

   /**
    * Rotates, then adds the given {@code translation} to this pose 3D.
    * <p>
    * Use this method if the {@code translation} is expressed in the local coordinates described by
    * this pose 3D. Otherwise, use {@link #prependTranslation(Tuple3DReadOnly)}.
    * </p>
    *
    * @param translation tuple containing the translation to apply to this pose 3D. Not modified.
    */
   default void appendTranslation(Tuple3DReadOnly translation)
   {
      appendTranslation(translation.getX(), translation.getY(), translation.getZ());
   }

   /**
    * Appends the given rotation to this pose 3D.
    * <p>
    * More precisely, the position part is unchanged while the orientation part is updated as
    * follows:<br>
    * {@code this.orientation = this.orientation * rotation}
    * </p>
    *
    * @param rotation the rotation to append to this pose 3D. Not modified.
    */
   default void appendRotation(QuaternionReadOnly rotation)
   {
      getOrientation().multiply(rotation);
   }

   /**
    * Appends the given rotation to this pose 3D.
    * <p>
    * More precisely, the position part is unchanged while the orientation part is updated as
    * follows:<br>
    * {@code this.orientation = this.orientation * rotation}
    * </p>
    *
    * @param rotation the rotation to append to this pose 3D. Not modified.
    */
   default void appendRotation(RotationMatrixReadOnly rotation)
   {
      getOrientation().multiply(rotation);
   }

   /**
    * Appends a rotation about the z-axis to this pose 3D.
    * <p>
    * More precisely, the position part is unchanged while the orientation part is updated as
    * follows:<br>
    *
    * <pre>
    *                                       / cos(yaw) -sin(yaw) 0 \
    * this.orientation = this.orientation * | sin(yaw)  cos(yaw) 0 |
    *                                       \    0         0     1 /
    * </pre>
    * </p>
    *
    * @param yaw the angle to rotate about the z-axis.
    */
   default void appendYawRotation(double yaw)
   {
      getOrientation().appendYawRotation(yaw);
   }

   /**
    * Appends a rotation about the y-axis to this pose 3D.
    * <p>
    * More precisely, the position part is unchanged while the orientation part is updated as
    * follows:<br>
    *
    * <pre>
    *                                       /  cos(pitch) 0 sin(pitch) \
    * this.orientation = this.orientation * |      0      1     0      |
    *                                       \ -sin(pitch) 0 cos(pitch) /
    * </pre>
    * </p>
    *
    * @param pitch the angle to rotate about the y-axis.
    */
   default void appendPitchRotation(double pitch)
   {
      getOrientation().appendPitchRotation(pitch);
   }

   /**
    * Appends a rotation about the x-axis to this pose 3D.
    * <p>
    * More precisely, the position part is unchanged while the orientation part is updated as
    * follows:<br>
    *
    * <pre>
    *                                       / 1     0          0     \
    * this.orientation = this.orientation * | 0 cos(roll) -sin(roll) |
    *                                       \ 0 sin(roll)  cos(roll) /
    * </pre>
    * </p>
    *
    * @param roll the angle to rotate about the x-axis.
    */
   default void appendRollRotation(double roll)
   {
      RotationMatrixTools.applyRollRotation(roll, getPosition(), getPosition());
      getOrientation().appendRollRotation(roll);
   }

   /**
    * Appends the given {@code transform} to this pose 3D.
    *
    * @param transform the rigid-body transform to append to this pose 3D. Not modified.
    */
   default void appendTransform(RigidBodyTransform transform)
   {
      QuaternionTools.addTransform(getOrientation(), transform.getTranslationVector(), getPosition());
      getOrientation().multiply(transform.getRotationMatrix());
   }

   /**
    * Appends the given {@code transform} to this pose 3D.
    *
    * @param transform the quaternion-based transform to append to this pose 3D. Not modified.
    */
   default void appendTransform(QuaternionBasedTransform transform)
   {
      QuaternionTools.addTransform(getOrientation(), transform.getTranslationVector(), getPosition());
      getOrientation().multiply(transform.getQuaternion());
   }

   /**
    * Transforms the position and orientation parts of this pose 3D by the given {@code transform}.
    *
    * @param transform the geometric transform to apply on this pose 3D. Not modified.
    */
   @Override
   default void applyTransform(Transform transform)
   {
      transform.transform(getPosition());
      transform.transform(getOrientation());
   }

   /**
    * Transforms the position and orientation parts of this pose 3D by the inverse of the given
    * {@code transform}.
    *
    * @param transform the geometric transform to apply on this pose 3D. Not modified.
    */
   @Override
   default void applyInverseTransform(Transform transform)
   {
      transform.inverseTransform(getPosition());
      transform.inverseTransform(getOrientation());
   }
}