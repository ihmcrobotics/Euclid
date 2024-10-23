package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;

/**
 * {@code FixedFrameTuple3DBasics} is the base implementation for {@link FramePoint3D} and
 * {@link FrameVector3D}.
 * <p>
 * In addition to representing a {@link Tuple3DBasics}, a constant {@link ReferenceFrame} is
 * associated to a {@code FixedFrameTuple3DBasics}. This allows, for instance, to enforce, at
 * runtime, that operations on tuples occur in the same coordinate system.
 * </p>
 * <p>
 * When using this interface, the reference frame of this tuple is assumed to be immutable, i.e. the
 * reference frame of this object cannot be changed via this interface.
 * </p>
 * <p>
 * Because a {@code FixedFrameTuple3DBasics} extends {@code Tuple3DBasics}, it is compatible with
 * methods only requiring {@code Tuple3DBasics}. However, these methods do NOT assert that the
 * operation occur in the proper coordinate system. Use this feature carefully and always prefer
 * using methods requiring {@code FixedFrameTuple3DBasics}.
 * </p>
 */
public interface FixedFrameTuple3DBasics extends FrameTuple3DReadOnly, Tuple3DBasics
{
   /**
    * Sets the x and y components of this frame tuple with the x and y components of the given
    * {@code tuple2DReadOnly}.
    * <p>
    * The z component remains unchanged.
    * </p>
    *
    * @param referenceFrame  the coordinate system in which the given {@code tuple2DReadOnly} is
    *                        expressed.
    * @param tuple2DReadOnly the geometry object used to update the geometry object in {@code this}.
    *                        Not modified.
    * @throws ReferenceFrameMismatchException if {@code this.referenceFrame != referenceFrame}.
    */
   default void set(ReferenceFrame referenceFrame, Tuple2DReadOnly tuple2DReadOnly)
   {
      checkReferenceFrameMatch(referenceFrame);
      set(tuple2DReadOnly);
   }

   /**
    * Sets the x and y components of this frame tuple with the x and y components of the given
    * {@code tuple2DReadOnly} and the z-component to the given {@code z}.
    *
    * @param referenceFrame  the coordinate system in which the given {@code tuple2DReadOnly} is
    *                        expressed.
    * @param tuple2DReadOnly the frame tuple to copy the values from. Not modified.
    * @param z               the new z-coordinate for this tuple.
    * @throws ReferenceFrameMismatchException if {@code this.referenceFrame != referenceFrame}.
    */
   default void set(ReferenceFrame referenceFrame, Tuple2DReadOnly tuple2DReadOnly, double z)
   {
      checkReferenceFrameMatch(referenceFrame);
      Tuple3DBasics.super.set(tuple2DReadOnly, z);
   }

   /**
    * Sets this frame tuple to {@code tuple3DReadOnly} and checks that its current frame equal
    * {@code referenceFrame}.
    *
    * @param referenceFrame  the coordinate system in which the given {@code tuple3DReadOnly} is
    *                        expressed.
    * @param tuple3DReadOnly the geometry object used to update the geometry object in {@code this}.
    *                        Not modified.
    * @throws ReferenceFrameMismatchException if {@code this.referenceFrame != referenceFrame}.
    */
   default void set(ReferenceFrame referenceFrame, Tuple3DReadOnly tuple3DReadOnly)
   {
      checkReferenceFrameMatch(referenceFrame);
      set(tuple3DReadOnly);
   }

   /**
    * Sets this frame tuple components to {@code x}, {@code y}, and {@code z} and checks that its
    * current frame equal {@code referenceFrame}.
    *
    * @param referenceFrame the coordinate system in which the given components ares expressed.
    * @param x              the new x component.
    * @param y              the new y component.
    * @param z              the new z component.
    * @throws ReferenceFrameMismatchException if {@code this.referenceFrame != referenceFrame}.
    */
   default void set(ReferenceFrame referenceFrame, double x, double y, double z)
   {
      checkReferenceFrameMatch(referenceFrame);
      set(x, y, z);
   }

   /**
    * Sets the x and y components of this frame tuple with the x and y components of the given
    * {@code frameTuple2DReadOnly}.
    * <p>
    * The z component remains unchanged.
    * </p>
    *
    * @param frameTuple2DReadOnly the frame tuple to copy the values from. Not modified.
    * @throws ReferenceFrameMismatchException if {@code frameTuple2DReadOnly} is not expressed in the
    *                                         same frame as {@code this}.
    */
   default void set(FrameTuple2DReadOnly frameTuple2DReadOnly)
   {
      set(frameTuple2DReadOnly.getReferenceFrame(), frameTuple2DReadOnly);
   }

   /**
    * Sets the x and y components of this frame tuple with the x and y components of the given
    * {@code frameTuple2DReadOnly} and the z-component to the given {@code z}.
    *
    * @param frameTuple2DReadOnly the frame tuple to copy the values from. Not modified.
    * @param z                    the new z-coordinate for this tuple.
    * @throws ReferenceFrameMismatchException if {@code frameTuple2DReadOnly} is not expressed in the
    *                                         same frame as {@code this}.
    */
   default void set(FrameTuple2DReadOnly frameTuple2DReadOnly, double z)
   {
      set(frameTuple2DReadOnly.getReferenceFrame(), frameTuple2DReadOnly, z);
   }

   /**
    * Sets this frame tuple to {@code other}.
    *
    * @param other the other frame tuple to set this to. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same frame as
    *                                         {@code this}.
    */
   default void set(FrameTuple3DReadOnly other)
   {
      set(other.getReferenceFrame(), other);
   }

   /**
    * Sets the x and y components of this frame tuple with the x and y components of the given
    * {@code tuple2DReadOnly} and the z-component to the given {@code z}.
    * <p>
    * If {@code tuple2DReadOnly} is expressed in the frame as {@code this}, then this method is
    * equivalent to {@link #set(ReferenceFrame, Tuple2DReadOnly, double)}.
    * </p>
    * <p>
    * If {@code tuple2DReadOnly} is expressed in a different frame than {@code this}, then {@code this}
    * is set with {@code tuple2DReadOnly} and {@code z}, and then transformed to be expressed in
    * {@code this.getReferenceFrame()}.
    * </p>
    *
    * @param referenceFrame  the coordinate system in which the given {@code tuple2DReadOnly} is
    *                        expressed.
    * @param tuple2DReadOnly the tuple to copy the values from. Not modified.
    * @param z               the new z-coordinate for this tuple.
    */
   default void setMatchingFrame(ReferenceFrame referenceFrame, Tuple2DReadOnly tuple2DReadOnly, double z)
   {
      Tuple3DBasics.super.set(tuple2DReadOnly, z);
      referenceFrame.transformFromThisToDesiredFrame(getReferenceFrame(), this);
   }

   /**
    * Sets the x and y components of this frame tuple with the x and y components of the given
    * {@code frameTuple2DReadOnly} and the z-component to the given {@code z}.
    * <p>
    * If {@code frameTuple2DReadOnly} is expressed in the frame as {@code this}, then this method is
    * equivalent to {@link #set(FrameTuple2DReadOnly, double)}.
    * </p>
    * <p>
    * If {@code frameTuple2DReadOnly} is expressed in a different frame than {@code this}, then
    * {@code this} is set with {@code frameTuple2DReadOnly} and {@code z}, and then transformed to be
    * expressed in {@code this.getReferenceFrame()}.
    * </p>
    *
    * @param frameTuple2DReadOnly the frame tuple to copy the values from. Not modified.
    * @param z                    the new z-coordinate for this tuple.
    */
   default void setMatchingFrame(FrameTuple2DReadOnly frameTuple2DReadOnly, double z)
   {
      setMatchingFrame(frameTuple2DReadOnly.getReferenceFrame(), frameTuple2DReadOnly, z);
   }

   /**
    * Sets this frame tuple to {@code tuple3DReadOnly}.
    * <p>
    * If {@code tuple3DReadOnly} is expressed in the frame as {@code this}, then this method is
    * equivalent to {@link #set(ReferenceFrame, Tuple3DReadOnly)}.
    * </p>
    * <p>
    * If {@code tuple3DReadOnly} is expressed in a different frame than {@code this}, then {@code this}
    * is set to {@code tuple3DReadOnly} once transformed to be expressed in
    * {@code this.getReferenceFrame()}.
    * </p>
    *
    * @param referenceFrame  the coordinate system in which the given {@code tuple3DReadOnly} is
    *                        expressed.
    * @param tuple3DReadOnly the other tuple to set this to. Not modified.
    */
   default void setMatchingFrame(ReferenceFrame referenceFrame, Tuple3DReadOnly tuple3DReadOnly)
   {
      Tuple3DBasics.super.set(tuple3DReadOnly);
      referenceFrame.transformFromThisToDesiredFrame(getReferenceFrame(), this);
   }

   /**
    * Sets this frame tuple to {@code other}.
    * <p>
    * If {@code other} is expressed in the frame as {@code this}, then this method is equivalent to
    * {@link #set(FrameTuple3DReadOnly)}.
    * </p>
    * <p>
    * If {@code other} is expressed in a different frame than {@code this}, then {@code this} is set to
    * {@code other} once transformed to be expressed in {@code this.getReferenceFrame()}.
    * </p>
    *
    * @param other the other frame tuple to set this to. Not modified.
    */
   default void setMatchingFrame(FrameTuple3DReadOnly other)
   {
      setMatchingFrame(other.getReferenceFrame(), other);
   }

   /**
    * Sets this frame tuple components to {@code x}, {@code y} and {@code z}.
    * <p>
    * If the frame of {@code this} is {@code referenceFrame}, then this method is equivalent to
    * {@link #set(ReferenceFrame, double, double, double)}.
    * </p>
    * <p>
    * If {@referenceFrame} is different from the frame of {@code this}, then {@code this} components
    * are set to {@code x}, {@code y} and {@code z} and then transformed to be expressed in
    * {@code this.getReferenceFrame()}.
    * </p>
    *
    * @param referenceFrame the coordinate system in which the given {@code x}, {@code y} and {@code z}
    *                       is expressed.
    * @param x              the new x-coordinate for this tuple.
    * @param y              the new y-coordinate for this tuple.
    * @param z              the new z-coordinate for this tuple.
    */
   default void setMatchingFrame(ReferenceFrame referenceFrame, double x, double y, double z)
   {
      Tuple3DBasics.super.set(x, y, z);
      referenceFrame.transformFromThisToDesiredFrame(getReferenceFrame(), this);
   }

   /**
    * Sets this frame tuple to {@code tuple3DReadOnly} and then calls {@link #absolute()}.
    *
    * @param referenceFrame  the coordinate system in which the given {@code tuple3DReadOnly} is
    *                        expressed.
    * @param tuple3DReadOnly the other tuple to copy the values from. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this.referenceFrame != referenceFrame}.
    */
   default void setAndAbsolute(ReferenceFrame referenceFrame, Tuple3DReadOnly tuple3DReadOnly)
   {
      checkReferenceFrameMatch(referenceFrame);
      Tuple3DBasics.super.setAndAbsolute(tuple3DReadOnly);
   }

   /**
    * Sets this frame tuple to {@code other} and then calls {@link #absolute()}.
    *
    * @param other the other frame tuple to copy the values from. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same frame as
    *                                         {@code this}.
    */
   default void setAndAbsolute(FrameTuple3DReadOnly other)
   {
      setAndAbsolute(other.getReferenceFrame(), other);
   }

   /**
    * Sets this frame tuple to {@code tuple3DReadOnly} and then calls {@link #negate()}.
    *
    * @param referenceFrame  the coordinate system in which the given {@code tuple3DReadOnly} is
    *                        expressed.
    * @param tuple3DReadOnly the other tuple to copy the values from. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this.referenceFrame != referenceFrame}.
    */
   default void setAndNegate(ReferenceFrame referenceFrame, Tuple3DReadOnly tuple3DReadOnly)
   {
      checkReferenceFrameMatch(referenceFrame);
      Tuple3DBasics.super.setAndNegate(tuple3DReadOnly);
   }

   /**
    * Sets this frame tuple to {@code other} and then calls {@link #negate()}.
    *
    * @param other the other frame tuple to copy the values from. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same frame as
    *                                         {@code this}.
    */
   default void setAndNegate(FrameTuple3DReadOnly other)
   {
      setAndNegate(other.getReferenceFrame(), other);
   }

   /**
    * Sets this tuple to {@code tuple3DReadOnly} and then calls {@link #normalize()}.
    *
    * @param referenceFrame  the coordinate system in which the given {@code tuple3DReadOnly} is
    *                        expressed.
    * @param tuple3DReadOnly the other tuple to copy the values from. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this.referenceFrame != referenceFrame}.
    */
   default void setAndNormalize(ReferenceFrame referenceFrame, Tuple3DReadOnly tuple3DReadOnly)
   {
      checkReferenceFrameMatch(referenceFrame);
      setAndNormalize(tuple3DReadOnly);
   }

   /**
    * Sets this frame tuple to {@code other} and then calls {@link #normalize()}.
    *
    * @param other the other frame tuple to copy the values from. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same frame as
    *                                         {@code this}.
    */
   default void setAndNormalize(FrameTuple3DReadOnly other)
   {
      setAndNormalize(other.getReferenceFrame(), other);
   }

   /**
    * Sets this frame tuple to {@code tuple3DReadOnly} and then calls {@link #scale(double)}.
    *
    * @param referenceFrame  the coordinate system in which the given {@code tuple3DReadOnly} is
    *                        expressed.
    * @param scalar          the scale factor to use on this frame tuple.
    * @param tuple3DReadOnly the other tuple to copy the values from. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this.referenceFrame != referenceFrame}.
    */
   default void setAndScale(double scalar, ReferenceFrame referenceFrame, Tuple3DReadOnly tuple3DReadOnly)
   {
      checkReferenceFrameMatch(referenceFrame);
      Tuple3DBasics.super.setAndScale(scalar, tuple3DReadOnly);
   }

   /**
    * Sets this frame tuple to {@code other} and then calls {@link #scale(double)}.
    *
    * @param scalar the scale factor to use on this frame tuple.
    * @param other  the other frame tuple to copy the values from. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same frame as
    *                                         {@code this}.
    */
   default void setAndScale(double scalar, FrameTuple3DReadOnly other)
   {
      setAndScale(scalar, other.getReferenceFrame(), other);
   }

   /**
    * Sets this frame tuple to {@code tuple3DReadOnly} and then calls {@link #clipToMax(double)}.
    *
    * @param referenceFrame  the coordinate system in which the given {@code tuple3DReadOnly} is
    *                        expressed.
    * @param max             the maximum value for each component of this frame tuple.
    * @param tuple3DReadOnly the other tuple to copy the values from. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this.referenceFrame != referenceFrame}.
    */
   default void setAndClipToMax(double max, ReferenceFrame referenceFrame, Tuple3DReadOnly tuple3DReadOnly)
   {
      checkReferenceFrameMatch(referenceFrame);
      Tuple3DBasics.super.setAndClipToMax(max, tuple3DReadOnly);
   }

   /**
    * Sets this frame tuple to {@code other} and then calls {@link #clipToMax(double)}.
    *
    * @param max   the maximum value for each component of this frame tuple.
    * @param other the other frame tuple to copy the values from. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same frame as
    *                                         {@code this}.
    */
   default void setAndClipToMax(double max, FrameTuple3DReadOnly other)
   {
      setAndClipToMax(max, other.getReferenceFrame(), other);
   }

   /**
    * Sets this frame tuple to {@code tuple3DReadOnly} and then calls {@link #clipToMin(double)}.
    *
    * @param referenceFrame  the coordinate system in which the given {@code tuple3DReadOnly} is
    *                        expressed.
    * @param min             the minimum value for each component of this frame tuple.
    * @param tuple3DReadOnly the other tuple to copy the values from. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this.referenceFrame != referenceFrame}.
    */
   default void setAndClipToMin(double min, ReferenceFrame referenceFrame, Tuple3DReadOnly tuple3DReadOnly)
   {
      checkReferenceFrameMatch(referenceFrame);
      Tuple3DBasics.super.setAndClipToMin(min, tuple3DReadOnly);
   }

   /**
    * Sets this frame tuple to {@code other} and then calls {@link #clipToMin(double)}.
    *
    * @param min   the minimum value for each component of this frame tuple.
    * @param other the other frame tuple to copy the values from. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same frame as
    *                                         {@code this}.
    */
   default void setAndClipToMin(double min, FrameTuple3DReadOnly other)
   {
      setAndClipToMin(min, other.getReferenceFrame(), other);
   }

   /**
    * Sets this frame tuple to {@code tuple3DReadOnly} and then calls
    * {@link #clipToMinMax(double, double)}.
    *
    * @param referenceFrame  the coordinate system in which the given {@code tuple3DReadOnly} is
    *                        expressed.
    * @param min             the minimum value for each component of this frame tuple.
    * @param max             the maximum value for each component of this frame tuple.
    * @param tuple3DReadOnly the other tuple to copy the values from. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this.referenceFrame != referenceFrame}.
    */
   default void setAndClipToMinMax(double min, double max, ReferenceFrame referenceFrame, Tuple3DReadOnly tuple3DReadOnly)
   {
      checkReferenceFrameMatch(referenceFrame);
      Tuple3DBasics.super.setAndClipToMinMax(min, max, tuple3DReadOnly);
   }

   /**
    * Sets this frame tuple to {@code other} and then calls {@link #clipToMinMax(double, double)}.
    *
    * @param min   the minimum value for each component of this frame tuple.
    * @param max   the maximum value for each component of this frame tuple.
    * @param other the other frame tuple to copy the values from. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same frame as
    *                                         {@code this}.
    */
   default void setAndClipToMinMax(double min, double max, FrameTuple3DReadOnly other)
   {
      setAndClipToMinMax(min, max, other.getReferenceFrame(), other);
   }

   /**
    * Adds the given frame tuple to this frame tuple.
    * <p>
    * this = this + other
    * </p>
    *
    * @param other the other frame tuple to add to this tuple. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same frame as
    *                                         {@code this}.
    */
   default void add(FrameTuple3DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      Tuple3DBasics.super.add(other);
   }

   /**
    * Sets this frame tuple to the sum of the two given frame tuples.
    * <p>
    * this = frameTuple1 + frameTuple2
    * </p>
    *
    * @param frameTuple1 the first frame tuple to sum. Not modified.
    * @param frameTuple2 the second frame tuple to sum. Not modified.
    * @throws ReferenceFrameMismatchException if either {@code frameTuple1} or {@code frameTuple2} is
    *                                         not expressed in the same frame as {@code this}.
    */
   default void add(FrameTuple3DReadOnly frameTuple1, FrameTuple3DReadOnly frameTuple2)
   {
      checkReferenceFrameMatch(frameTuple1, frameTuple2);
      Tuple3DBasics.super.add(frameTuple1, frameTuple2);
   }

   /**
    * Sets this frame tuple to the sum of the two given tuples.
    * <p>
    * this = frameTuple1 + tuple2
    * </p>
    *
    * @param frameTuple1 the first frame tuple to sum. Not modified.
    * @param tuple2      the second tuple to sum. Not modified.
    * @throws ReferenceFrameMismatchException if {@code frameTuple1} is not expressed in the same frame
    *                                         as {@code this}.
    */
   default void add(FrameTuple3DReadOnly frameTuple1, Tuple3DReadOnly tuple2)
   {
      checkReferenceFrameMatch(frameTuple1);
      Tuple3DBasics.super.add(frameTuple1, tuple2);
   }

   /**
    * Sets this frame tuple to the sum of the two given tuples.
    * <p>
    * this = tuple1 + frameTuple2
    * </p>
    *
    * @param tuple1      the first tuple to sum. Not modified.
    * @param frameTuple2 the second frame tuple to sum. Not modified.
    * @throws ReferenceFrameMismatchException if {@code frameTuple2} is not expressed in the same frame
    *                                         as {@code this}.
    */
   default void add(Tuple3DReadOnly tuple1, FrameTuple3DReadOnly frameTuple2)
   {
      checkReferenceFrameMatch(frameTuple2);
      Tuple3DBasics.super.add(tuple1, frameTuple2);
   }

   /**
    * Subtracts the given frame tuple to this frame tuple.
    * <p>
    * this = this - other
    * </p>
    *
    * @param other the other frame tuple to subtract to this tuple. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same frame as
    *                                         {@code this}.
    */
   default void sub(FrameTuple3DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      Tuple3DBasics.super.sub(other);
   }

   /**
    * Sets this frame tuple to the difference of the two given frame tuples.
    * <p>
    * this = frameTuple1 - frameTuple2
    * </p>
    *
    * @param frameTuple1 the first frame tuple. Not modified.
    * @param frameTuple2 the second frame tuple to subtract to {@code frameTuple1}. Not modified.
    * @throws ReferenceFrameMismatchException if either {@code frameTuple1} or {@code frameTuple2} is
    *                                         not expressed in the same frame as {@code this}.
    */
   default void sub(FrameTuple3DReadOnly frameTuple1, FrameTuple3DReadOnly frameTuple2)
   {
      checkReferenceFrameMatch(frameTuple1, frameTuple2);
      Tuple3DBasics.super.sub(frameTuple1, frameTuple2);
   }

   /**
    * Sets this frame tuple to the difference of the two given tuples.
    * <p>
    * this = tuple1 - frameTuple2
    * </p>
    *
    * @param tuple1      the first tuple. Not modified.
    * @param frameTuple2 the second frame tuple to subtract to {@code frameTuple1}. Not modified.
    * @throws ReferenceFrameMismatchException if {@code frameTuple2} is not expressed in the same frame
    *                                         as {@code this}.
    */
   default void sub(Tuple3DReadOnly tuple1, FrameTuple3DReadOnly frameTuple2)
   {
      checkReferenceFrameMatch(frameTuple2);
      Tuple3DBasics.super.sub(tuple1, frameTuple2);
   }

   /**
    * Sets this frame tuple to the difference of the two given tuples.
    * <p>
    * this = frameTuple1 - tuple2
    * </p>
    *
    * @param frameTuple1 the first frame tuple. Not modified.
    * @param tuple2      the second tuple to subtract to {@code frameTuple1}. Not modified.
    * @throws ReferenceFrameMismatchException if {@code frameTuple1} is not expressed in the same frame
    *                                         as {@code this}.
    */
   default void sub(FrameTuple3DReadOnly frameTuple1, Tuple3DReadOnly tuple2)
   {
      checkReferenceFrameMatch(frameTuple1);
      Tuple3DBasics.super.sub(frameTuple1, tuple2);
   }

   /**
    * Scales this frame tuple and adds {@code other}.
    * <p>
    * this = scalar * this + other
    * </p>
    *
    * @param scalar the scale factor to use.
    * @param other  the frame tuple to add to this. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same frame as
    *                                         {@code this}.
    */
   default void scaleAdd(double scalar, FrameTuple3DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      Tuple3DBasics.super.scaleAdd(scalar, other);
   }

   /**
    * Sets this frame tuple to the sum of {@code frameTuple1} scaled and {@code frameTuple2}.
    * <p>
    * this = scalar * frameTuple1 + frameTuple2
    * </p>
    *
    * @param scalar      the scale factor to use on {@code frameTuple1}.
    * @param frameTuple1 the first frame tuple of the sum. Not modified.
    * @param frameTuple2 the second frame tuple of the sum. Not modified.
    * @throws ReferenceFrameMismatchException if either {@code frameTuple1} or {@code frameTuple2} is
    *                                         not expressed in the same frame as {@code this}.
    */
   default void scaleAdd(double scalar, FrameTuple3DReadOnly frameTuple1, FrameTuple3DReadOnly frameTuple2)
   {
      checkReferenceFrameMatch(frameTuple1, frameTuple2);
      Tuple3DBasics.super.scaleAdd(scalar, frameTuple1, frameTuple2);
   }

   /**
    * Sets this frame tuple to the sum of {@code tuple1} scaled and {@code frameTuple2}.
    * <p>
    * this = scalar * tuple1 + frameTuple2
    * </p>
    *
    * @param scalar      the scale factor to use on {@code tuple1}.
    * @param tuple1      the first tuple of the sum. Not modified.
    * @param frameTuple2 the second frame tuple of the sum. Not modified.
    * @throws ReferenceFrameMismatchException if {@code frameTuple2} is not expressed in the same frame
    *                                         as {@code this}.
    */
   default void scaleAdd(double scalar, Tuple3DReadOnly tuple1, FrameTuple3DReadOnly frameTuple2)
   {
      checkReferenceFrameMatch(frameTuple2);
      Tuple3DBasics.super.scaleAdd(scalar, tuple1, frameTuple2);
   }

   /**
    * Sets this frame tuple to the sum of {@code frameTuple1} scaled and {@code tuple2}.
    * <p>
    * this = scalar * frameTuple1 + tuple2
    * </p>
    *
    * @param scalar      the scale factor to use on {@code frameTuple1}.
    * @param frameTuple1 the first frame tuple of the sum. Not modified.
    * @param tuple2      the second tuple of the sum. Not modified.
    * @throws ReferenceFrameMismatchException if {@code frameTuple1} is not expressed in the same frame
    *                                         as {@code this}.
    */
   default void scaleAdd(double scalar, FrameTuple3DReadOnly frameTuple1, Tuple3DReadOnly tuple2)
   {
      checkReferenceFrameMatch(frameTuple1);
      Tuple3DBasics.super.scaleAdd(scalar, frameTuple1, tuple2);
   }

   /**
    * Scales this frame tuple and subtracts {@code other}.
    * <p>
    * this = scalar * this - other
    * </p>
    *
    * @param scalar the scale factor to use.
    * @param other  the frame tuple to add to this. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same frame as
    *                                         {@code this}.
    */
   default void scaleSub(double scalar, FrameTuple3DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      Tuple3DBasics.super.scaleSub(scalar, other);
   }

   /**
    * Sets this frame tuple to the difference of {@code frameTuple1} scaled and {@code frameTuple2}.
    * <p>
    * this = scalar * frameTuple1 - frameTuple2
    * </p>
    *
    * @param scalar      the scale factor to use on {@code frameTuple1}.
    * @param frameTuple1 the first frame tuple of the difference. Not modified.
    * @param frameTuple2 the second frame tuple of the difference. Not modified.
    * @throws ReferenceFrameMismatchException if either {@code frameTuple1} or {@code frameTuple2} is
    *                                         not expressed in the same frame as {@code this}.
    */
   default void scaleSub(double scalar, FrameTuple3DReadOnly frameTuple1, FrameTuple3DReadOnly frameTuple2)
   {
      checkReferenceFrameMatch(frameTuple1, frameTuple2);
      Tuple3DBasics.super.scaleSub(scalar, frameTuple1, frameTuple2);
   }

   /**
    * Sets this frame tuple to the difference of {@code tuple1} scaled and {@code frameTuple2}.
    * <p>
    * this = scalar * tuple1 - frameTuple2
    * </p>
    *
    * @param scalar      the scale factor to use on {@code frameTuple1}.
    * @param tuple1      the first tuple of the difference. Not modified.
    * @param frameTuple2 the second frame tuple of the difference. Not modified.
    * @throws ReferenceFrameMismatchException if {@code frameTuple2} is not expressed in the same frame
    *                                         as {@code this}.
    */
   default void scaleSub(double scalar, Tuple3DReadOnly tuple1, FrameTuple3DReadOnly frameTuple2)
   {
      checkReferenceFrameMatch(frameTuple2);
      Tuple3DBasics.super.scaleSub(scalar, tuple1, frameTuple2);
   }

   /**
    * Sets this frame tuple to the difference of {@code frameTuple1} scaled and {@code tuple2}.
    * <p>
    * this = scalar * frameTuple1 - tuple2
    * </p>
    *
    * @param scalar      the scale factor to use on {@code frameTuple1}.
    * @param frameTuple1 the first frame tuple of the difference. Not modified.
    * @param frameTuple2 the second tuple of the difference. Not modified.
    * @throws ReferenceFrameMismatchException if {@code frameTuple1} is not expressed in the same frame
    *                                         as {@code this}.
    */
   default void scaleSub(double scalar, FrameTuple3DReadOnly frameTuple1, Tuple3DReadOnly frameTuple2)
   {
      checkReferenceFrameMatch(frameTuple1);
      Tuple3DBasics.super.scaleSub(scalar, frameTuple1, frameTuple2);
   }

   /**
    * Performs a linear interpolation from this frame tuple to {@code other} given the percentage
    * {@code alpha}.
    * <p>
    * this = (1.0 - alpha) * this + alpha * other
    * </p>
    *
    * @param other the other frame tuple used for the interpolation. Not modified.
    * @param alpha the percentage used for the interpolation. A value of 0 will result in not modifying
    *              this frame tuple, while a value of 1 is equivalent to setting this frame tuple to
    *              {@code other}.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same frame as
    *                                         {@code this}.
    */
   default void interpolate(FrameTuple3DReadOnly other, double alpha)
   {
      checkReferenceFrameMatch(other);
      Tuple3DBasics.super.interpolate(other, alpha);
   }

   /**
    * Performs a linear interpolation from {@code frameTuple1} to {@code frameTuple2} given the
    * percentage {@code alpha}.
    * <p>
    * this = (1.0 - alpha) * frameTuple1 + alpha * frameTuple2
    * </p>
    *
    * @param frameTuple1 the first frame tuple used in the interpolation. Not modified.
    * @param frameTuple2 the second frame tuple used in the interpolation. Not modified.
    * @param alpha       the percentage to use for the interpolation. A value of 0 will result in
    *                    setting this frame tuple to {@code frameTuple1}, while a value of 1 is
    *                    equivalent to setting this frame tuple to {@code frameTuple2}.
    * @throws ReferenceFrameMismatchException if either {@code frameTuple1} or {@code frameTuple2} is
    *                                         not expressed in the same frame as {@code this}.
    */
   default void interpolate(FrameTuple3DReadOnly frameTuple1, FrameTuple3DReadOnly frameTuple2, double alpha)
   {
      checkReferenceFrameMatch(frameTuple1, frameTuple2);
      Tuple3DBasics.super.interpolate(frameTuple1, frameTuple2, alpha);
   }

   /**
    * Performs a linear interpolation from {@code tuple1} to {@code frameTuple2} given the percentage
    * {@code alpha}.
    * <p>
    * this = (1.0 - alpha) * tuple1 + alpha * frameTuple2
    * </p>
    *
    * @param tuple1      the first tuple used in the interpolation. Not modified.
    * @param frameTuple2 the second frame tuple used in the interpolation. Not modified.
    * @param alpha       the percentage to use for the interpolation. A value of 0 will result in
    *                    setting this frame tuple to {@code tuple1}, while a value of 1 is equivalent
    *                    to setting this frame tuple to {@code frameTuple2}.
    * @throws ReferenceFrameMismatchException if {@code frameTuple2} is not expressed in the same frame
    *                                         as {@code this}.
    */
   default void interpolate(Tuple3DReadOnly tuple1, FrameTuple3DReadOnly frameTuple2, double alpha)
   {
      checkReferenceFrameMatch(frameTuple2);
      Tuple3DBasics.super.interpolate(tuple1, frameTuple2, alpha);
   }

   /**
    * Performs a linear interpolation from {@code frameTuple1} to {@code tuple2} given the percentage
    * {@code alpha}.
    * <p>
    * this = (1.0 - alpha) * frameTuple1 + alpha * tuple2
    * </p>
    *
    * @param frameTuple1 the first frame tuple used in the interpolation. Not modified.
    * @param tuple2      the second tuple used in the interpolation. Not modified.
    * @param alpha       the percentage to use for the interpolation. A value of 0 will result in
    *                    setting this frame tuple to {@code frameTuple1}, while a value of 1 is
    *                    equivalent to setting this frame tuple to {@code tuple2}.
    * @throws ReferenceFrameMismatchException if {@code frameTuple1} is not expressed in the same frame
    *                                         as {@code this}.
    */
   default void interpolate(FrameTuple3DReadOnly frameTuple1, Tuple3DReadOnly tuple2, double alpha)
   {
      checkReferenceFrameMatch(frameTuple1);
      Tuple3DBasics.super.interpolate(frameTuple1, tuple2, alpha);
   }
}