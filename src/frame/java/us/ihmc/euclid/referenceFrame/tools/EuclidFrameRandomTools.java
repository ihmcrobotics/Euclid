package us.ihmc.euclid.referenceFrame.tools;

import java.util.Random;

import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.geometry.interfaces.Vertex3DSupplier;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.referenceFrame.FrameBoundingBox2D;
import us.ihmc.euclid.referenceFrame.FrameBoundingBox3D;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FrameLine2D;
import us.ihmc.euclid.referenceFrame.FrameLine3D;
import us.ihmc.euclid.referenceFrame.FrameLineSegment2D;
import us.ihmc.euclid.referenceFrame.FrameLineSegment3D;
import us.ihmc.euclid.referenceFrame.FrameMatrix3D;
import us.ihmc.euclid.referenceFrame.FrameOrientation2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameRotationMatrix;
import us.ihmc.euclid.referenceFrame.FrameUnitVector2D;
import us.ihmc.euclid.referenceFrame.FrameUnitVector3D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.FrameVector4D;
import us.ihmc.euclid.referenceFrame.FrameYawPitchRoll;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameOrientation3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVertex2DSupplier;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVertex3DSupplier;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * This class provides random generators to generate random frame geometry objects.
 * <p>
 * The main application is for writing JUnit Tests.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public class EuclidFrameRandomTools
{
   private EuclidFrameRandomTools()
   {
      // Suppresses default constructor, ensuring non-instantiability.
   }

   /**
    * Generates a reference frame with a random transform to world frame.
    *
    * @param random the random generator to use.
    * @return the new random reference frame.
    */
   public static ReferenceFrame nextReferenceFrame(Random random)
   {
      return nextReferenceFrame(random, false);
   }

   /**
    * Generates a reference frame with a random transform to world frame.
    *
    * @param frameName reference frame name
    * @param random the random generator to use.
    * @return the new random reference frame.
    */
   public static ReferenceFrame nextReferenceFrame(String frameName, Random random)
   {
      return nextReferenceFrame(frameName, random, false);
   }

   /**
    * Generates a reference frame with a random transform to world frame.
    *
    * @param random         the random generator to use.
    * @param use2DTransform whether to use a 2D or 3D rotation for the transform used to create the
    *                       random frame.
    * @return the new random reference frame.
    */
   public static ReferenceFrame nextReferenceFrame(Random random, boolean use2DTransform)
   {
      return nextReferenceFrame(random, ReferenceFrame.getWorldFrame(), use2DTransform);
   }

   /**
    * Generates a reference frame with a random transform to world frame.
    *
    * @param frameName      the reference frame name.
    * @param random         the random generator to use.
    * @param use2DTransform whether to use a 2D or 3D rotation for the transform used to create the
    *                       random frame.
    * @return the new random reference frame.
    */
   public static ReferenceFrame nextReferenceFrame(String frameName, Random random, boolean use2DTransform)
   {
      return nextReferenceFrame(frameName, random, ReferenceFrame.getWorldFrame(), use2DTransform);
   }

   /**
    * Generates a reference frame with a random transform to its parent frame.
    *
    * @param random      the random generator to use.
    * @param parentFrame the parent frame of the new reference frame.
    * @return the new random reference frame.
    */
   public static ReferenceFrame nextReferenceFrame(Random random, ReferenceFrame parentFrame)
   {
      return nextReferenceFrame(random, parentFrame, false);
   }

   /**
    * Generates a reference frame with a random transform to its parent frame.
    *
    * @param random         the random generator to use.
    * @param parentFrame    the parent frame of the new reference frame.
    * @param use2DTransform whether to use a 2D or 3D rotation for the transform used to create the
    *                       random frame.
    * @return the new random reference frame.
    */
   public static ReferenceFrame nextReferenceFrame(Random random, ReferenceFrame parentFrame, boolean use2DTransform)
   {
      return nextReferenceFrame("randomFrame" + random.nextInt(), random, parentFrame, use2DTransform);
   }

   /**
    * Generates a reference frame with a random transform to its parent frame.
    *
    * @param frameName   the name of the new frame.
    * @param random      the random generator to use.
    * @param parentFrame the parent frame of the new reference frame.
    * @return the new random reference frame.
    */
   public static ReferenceFrame nextReferenceFrame(String frameName, Random random, ReferenceFrame parentFrame)
   {
      return nextReferenceFrame(frameName, random, parentFrame, false);
   }

   /**
    * Generates a reference frame with a random transform to its parent frame.
    *
    * @param frameName      the name of the new frame.
    * @param random         the random generator to use.
    * @param parentFrame    the parent frame of the new reference frame.
    * @param use2DTransform whether to use a 2D or 3D rotation for the transform used to create the
    *                       random frame.
    * @return the new random reference frame.
    */
   public static ReferenceFrame nextReferenceFrame(String frameName, Random random, ReferenceFrame parentFrame, boolean use2DTransform)
   {
      RigidBodyTransform transformFromParent;
      if (use2DTransform)
         transformFromParent = EuclidCoreRandomTools.nextRigidBodyTransform2D(random);
      else
         transformFromParent = EuclidCoreRandomTools.nextRigidBodyTransform(random);
      return ReferenceFrameTools.constructFrameWithUnchangingTransformFromParent(frameName, parentFrame, transformFromParent);
   }

   /**
    * Creates a tree structure of 20 random reference frames starting off
    * {@code ReferenceFrame.getWorldFrame()}.
    *
    * @param random the random generator to use.
    * @return the array containing the random reference frames and
    *         {@code ReferenceFrame.getWorldFrame()} at the first index.
    */
   public static ReferenceFrame[] nextReferenceFrameTree(Random random)
   {
      return nextReferenceFrameTree(random, false);
   }

   /**
    * Creates a tree structure of 20 random reference frames starting off
    * {@code ReferenceFrame.getWorldFrame()}.
    *
    * @param namePrefix name prefix for each frame
    * @param random the random generator to use.
    * @return the array containing the random reference frames and
    *         {@code ReferenceFrame.getWorldFrame()} at the first index.
    */
   public static ReferenceFrame[] nextReferenceFrameTree(String namePrefix, Random random)
   {
      return nextReferenceFrameTree(namePrefix, random, false);
   }

   /**
    * Creates a tree structure of 20 random reference frames start off
    * {@link ReferenceFrame#getWorldFrame()}.
    *
    * @param random          the random generator to use.
    * @param use2DTransforms whether to use a 2D or 3D rotation for the transform used to create the
    *                        random frames.
    * @return the array containing the random reference frames and
    *         {@code ReferenceFrame.getWorldFrame()} at the first index.
    */
   public static ReferenceFrame[] nextReferenceFrameTree(Random random, boolean use2DTransforms)
   {
      return nextReferenceFrameTree(random, 20, use2DTransforms);
   }

   /**
    * Creates a tree structure of 20 random reference frames start off
    * {@link ReferenceFrame#getWorldFrame()}.
    *
    * @param namePrefix name prefix for each frame
    * @param random          the random generator to use.
    * @param use2DTransforms whether to use a 2D or 3D rotation for the transform used to create the
    *                        random frames.
    * @return the array containing the random reference frames and
    *         {@code ReferenceFrame.getWorldFrame()} at the first index.
    */
   public static ReferenceFrame[] nextReferenceFrameTree(String namePrefix, Random random, boolean use2DTransforms)
   {
      return nextReferenceFrameTree(namePrefix, random, ReferenceFrame.getWorldFrame(), 20, use2DTransforms);
   }

   /**
    * Creates a tree structure of random reference frames starting off
    * {@code ReferenceFrame.getWorldFrame()}.
    *
    * @param random                  the random generator to use.
    * @param numberOfReferenceFrames the number of reference frames to be created.
    * @return the array containing the random reference frames and
    *         {@code ReferenceFrame.getWorldFrame()} at the first index.
    */
   public static ReferenceFrame[] nextReferenceFrameTree(Random random, int numberOfReferenceFrames)
   {
      return nextReferenceFrameTree(random, numberOfReferenceFrames, false);
   }

   /**
    * Creates a tree structure of random reference frames starting off
    * {@code ReferenceFrame.getWorldFrame()}.
    *
    * @param random                  the random generator to use.
    * @param numberOfReferenceFrames the number of reference frames to be created.
    * @param use2DTransforms         whether to use a 2D or 3D rotation for the transform used to
    *                                create the random frames.
    * @return the array containing the random reference frames and
    *         {@code ReferenceFrame.getWorldFrame()} at the first index.
    */
   public static ReferenceFrame[] nextReferenceFrameTree(Random random, int numberOfReferenceFrames, boolean use2DTransforms)
   {
      return nextReferenceFrameTree("randomFrame", random, ReferenceFrame.getWorldFrame(), numberOfReferenceFrames, use2DTransforms);
   }

   /**
    * Creates a tree structure of random reference frames starting off the given {@code rootFrame}.
    *
    * @param frameNamePrefix         prefix to use when creating each random reference frame.
    * @param random                  the random generator to use.
    * @param rootFrame               the base frame from which the tree is to be expanded.
    * @param numberOfReferenceFrames the number of reference frames to be created.
    * @return the array containing the random reference frames and {@code rootFrame} at the first
    *         index.
    */
   public static ReferenceFrame[] nextReferenceFrameTree(String frameNamePrefix, Random random, ReferenceFrame rootFrame, int numberOfReferenceFrames)
   {
      return nextReferenceFrameTree(frameNamePrefix, random, rootFrame, numberOfReferenceFrames, false);
   }

   /**
    * Creates a tree structure of random reference frames starting off the given {@code rootFrame}.
    *
    * @param frameNamePrefix         prefix to use when creating each random reference frame.
    * @param random                  the random generator to use.
    * @param rootFrame               the base frame from which the tree is to be expanded.
    * @param numberOfReferenceFrames the number of reference frames to be created.
    * @param use2DTransforms         whether to use a 2D or 3D rotation for the transform used to
    *                                create the random frames.
    * @return the array containing the random reference frames and {@code rootFrame} at the first
    *         index.
    */
   public static ReferenceFrame[] nextReferenceFrameTree(String frameNamePrefix,
                                                         Random random,
                                                         ReferenceFrame rootFrame,
                                                         int numberOfReferenceFrames,
                                                         boolean use2DTransforms)
   {
      ReferenceFrame[] referenceFrames = new ReferenceFrame[numberOfReferenceFrames + 1];
      referenceFrames[0] = rootFrame;

      for (int i = 0; i < numberOfReferenceFrames; i++)
      {
         int parentFrameIndex = random.nextInt(i + 1);
         ReferenceFrame parentFrame = referenceFrames[parentFrameIndex];
         referenceFrames[i + 1] = nextReferenceFrame(frameNamePrefix + i, random, parentFrame, use2DTransforms);
      }

      return referenceFrames;
   }

   /**
    * Generates a random frame point.
    * <p>
    * {@code framePoint}<sub>i</sub> &in; [-1.0; 1.0].
    * </p>
    *
    * @param random         the random generator to use.
    * @param referenceFrame the random frame point's reference frame.
    * @return the random frame point.
    */
   public static FramePoint3D nextFramePoint3D(Random random, ReferenceFrame referenceFrame)
   {
      return new FramePoint3D(referenceFrame, EuclidCoreRandomTools.nextPoint3D(random));
   }

   /**
    * Generates a random frame point.
    * <p>
    * {@code framePoint.x} &in; [-minMax; minMax]. <br>
    * {@code framePoint.y} &in; [-minMax; minMax]. <br>
    * {@code framePoint.z} &in; [-minMax; minMax]. <br>
    * </p>
    *
    * @param random         the random generator to use.
    * @param referenceFrame the random frame point's reference frame.
    * @param minMax         the maximum absolute value for each coordinate.
    * @return the random frame point.
    * @throws RuntimeException if {@code minMax < 0}.
    */
   public static FramePoint3D nextFramePoint3D(Random random, ReferenceFrame referenceFrame, double minMax)
   {
      return new FramePoint3D(referenceFrame, EuclidCoreRandomTools.nextPoint3D(random, minMax));
   }

   /**
    * Generates a random frame point.
    * <p>
    * {@code framePoint.x} &in; [min; max]. <br>
    * {@code framePoint.y} &in; [min; max]. <br>
    * {@code framePoint.z} &in; [min; max]. <br>
    * </p>
    *
    * @param random         the random generator to use.
    * @param referenceFrame the random frame point's reference frame.
    * @param min            the minimum value for each coordinate.
    * @param max            the maximum value for each coordinate.
    * @return the random frame point.
    * @throws RuntimeException if {@code min > max}.
    */
   public static FramePoint3D nextFramePoint3D(Random random, ReferenceFrame referenceFrame, double min, double max)
   {
      return new FramePoint3D(referenceFrame, EuclidCoreRandomTools.nextPoint3D(random, min, max));
   }

   /**
    * Generates a random frame point.
    * <p>
    * {@code framePoint.x} &in; [-maxAbsoluteX; maxAbsoluteX]. <br>
    * {@code framePoint.y} &in; [-maxAbsoluteY; maxAbsoluteY]. <br>
    * {@code framePoint.z} &in; [-maxAbsoluteZ; maxAbsoluteZ]. <br>
    * </p>
    *
    * @param random         the random generator to use.
    * @param referenceFrame the random frame point's reference frame.
    * @param maxAbsoluteX   the maximum absolute value for the x-coordinate.
    * @param maxAbsoluteY   the maximum absolute value for the y-coordinate.
    * @param maxAbsoluteZ   the maximum absolute value for the z-coordinate.
    * @return the random frame point.
    * @throws RuntimeException if {@code maxAbsoluteX < 0}, {@code maxAbsoluteY < 0},
    *                          {@code maxAbsoluteZ < 0}.
    */
   public static FramePoint3D nextFramePoint3D(Random random, ReferenceFrame referenceFrame, double maxAbsoluteX, double maxAbsoluteY, double maxAbsoluteZ)
   {
      return new FramePoint3D(referenceFrame, EuclidCoreRandomTools.nextPoint3D(random, maxAbsoluteX, maxAbsoluteY, maxAbsoluteZ));
   }

   /**
    * Generates a random frame point.
    * <p>
    * {@code framePoint.x} &in; [minX; maxX]. <br>
    * {@code framePoint.y} &in; [minY; maxY]. <br>
    * {@code framePoint.z} &in; [minZ; maxZ]. <br>
    * </p>
    *
    * @param random         the random generator to use.
    * @param referenceFrame the random frame point's reference frame.
    * @param minX           the minimum value for the x-coordinate.
    * @param maxX           the maximum value for the x-coordinate.
    * @param minY           the minimum value for the y-coordinate.
    * @param maxY           the maximum value for the y-coordinate.
    * @param minZ           the minimum value for the z-coordinate.
    * @param maxZ           the maximum value for the z-coordinate.
    * @return the random frame point.
    * @throws RuntimeException if {@code maxX < minX}, {@code maxY < minY}, {@code maxZ < minZ}.
    */
   public static FramePoint3D nextFramePoint3D(Random random,
                                               ReferenceFrame referenceFrame,
                                               double minX,
                                               double maxX,
                                               double minY,
                                               double maxY,
                                               double minZ,
                                               double maxZ)
   {
      return new FramePoint3D(referenceFrame, EuclidCoreRandomTools.nextPoint3D(random, minX, maxX, minY, maxY, minZ, maxZ));
   }

   /**
    * Generates a random frame vector.
    * <p>
    * {@code frameVector}<sub>i</sub> &in; [-1.0; 1.0].
    * </p>
    *
    * @param random         the random generator to use.
    * @param referenceFrame the random frame vector's reference frame.
    * @return the random frame vector.
    */
   public static FrameVector3D nextFrameVector3D(Random random, ReferenceFrame referenceFrame)
   {
      return new FrameVector3D(referenceFrame, EuclidCoreRandomTools.nextVector3D(random));
   }

   /**
    * Generates a random frame unit vector.
    * <p>
    * This generator uses {@link EuclidCoreRandomTools#nextVector3D(Random)}.
    * </p>
    *
    * @param random         the random generator to use.
    * @param referenceFrame the random frame unit vector's reference frame.
    * @return the random frame unit vector.
    */
   public static FrameUnitVector3D nextFrameUnitVector3D(Random random, ReferenceFrame referenceFrame)
   {
      return new FrameUnitVector3D(referenceFrame, EuclidCoreRandomTools.nextUnitVector3D(random));
   }

   /**
    * Generates a random frame vector.
    * <p>
    * {@code frameVector}<sub>i</sub> &in; [-{@code minMax}<sub>i</sub>; {@code minMax}<sub>i</sub>].
    * </p>
    *
    * @param random         the random generator to use.
    * @param referenceFrame the random frame vector's reference frame.
    * @param minMax         tuple used to bound the maximum absolute value of each component of the
    *                       generated frame vector. Not modified.
    * @return the random frame vector.
    * @throws RuntimeException if any component of {@code minMax} is negative.
    */
   public static FrameVector3D nextFrameVector3D(Random random, ReferenceFrame referenceFrame, Tuple3DReadOnly minMax)
   {
      return new FrameVector3D(referenceFrame, EuclidCoreRandomTools.nextVector3D(random, minMax));
   }

   /**
    * Generates a random frame vector.
    * <p>
    * {@code frameVector}<sub>i</sub> &in; [{@code min}<sub>i</sub>; {@code max}<sub>i</sub>].
    * </p>
    *
    * @param random         the random generator to use.
    * @param referenceFrame the random frame vector's reference frame.
    * @param min            tuple used as upper-bound for each component of the generated frame vector.
    *                       Not modified.
    * @param max            tuple used as lower-bound for each component of the generated frame vector.
    *                       Not modified.
    * @return the random frame vector.
    * @throws RuntimeException if {@code min}<sub>i</sub> > {@code max}<sub>i</sub>.
    */
   public static FrameVector3D nextFrameVector3D(Random random, ReferenceFrame referenceFrame, Tuple3DReadOnly min, Tuple3DReadOnly max)
   {
      return new FrameVector3D(referenceFrame, EuclidCoreRandomTools.nextVector3D(random, min, max));
   }

   /**
    * Generates a random frame vector.
    * <p>
    * {@code frameVector}<sub>i</sub> &in; [{@code min}; {@code max}].
    * </p>
    *
    * @param random         the random generator to use.
    * @param referenceFrame the random frame vector's reference frame.
    * @param min            upper-bound for each component of the generated frame vector. Not modified.
    * @param max            lower-bound for each component of the generated frame vector. Not modified.
    * @return the random frame vector.
    * @throws RuntimeException if {@code min > max}.
    */
   public static FrameVector3D nextFrameVector3D(Random random, ReferenceFrame referenceFrame, double min, double max)
   {
      return new FrameVector3D(referenceFrame, EuclidCoreRandomTools.nextVector3D(random, min, max));
   }

   /**
    * Generates a random vector.
    * <p>
    * {@code frameVector.x} &in; [minX; maxX]. <br>
    * {@code frameVector.y} &in; [minY; maxY]. <br>
    * {@code frameVector.z} &in; [minZ; maxZ]. <br>
    * </p>
    *
    * @param random         the random generator to use.
    * @param referenceFrame the random frame vector's reference frame.
    * @param minX           the minimum value for the x-component.
    * @param maxX           the maximum value for the x-component.
    * @param minY           the minimum value for the y-component.
    * @param maxY           the maximum value for the y-component.
    * @param minZ           the minimum value for the z-component.
    * @param maxZ           the maximum value for the z-component.
    * @return the random vector.
    * @throws RuntimeException if {@code maxX < minX}, {@code maxY < minY}, {@code maxZ < minZ}.
    */
   public static FrameVector3D nextFrameVector3D(Random random,
                                                 ReferenceFrame referenceFrame,
                                                 double minX,
                                                 double maxX,
                                                 double minY,
                                                 double maxY,
                                                 double minZ,
                                                 double maxZ)
   {
      return new FrameVector3D(referenceFrame, EuclidCoreRandomTools.nextVector3D(random, minX, maxX, minY, maxY, minZ, maxZ));
   }

   /**
    * Generates a random frame vector given its length {@code length}.
    *
    * @param random         the random generator to use.
    * @param referenceFrame the random frame vector's reference frame.
    * @param length         the length of the generated frame vector.
    * @return the random frame vector.
    */
   public static FrameVector3D nextFrameVector3DWithFixedLength(Random random, ReferenceFrame referenceFrame, double length)
   {
      return new FrameVector3D(referenceFrame, EuclidCoreRandomTools.nextVector3DWithFixedLength(random, length));
   }

   /**
    * Generates a random frame vector that is perpendicular to {@code vectorToBeOrthogonalTo}.
    *
    * @param random                 the random generator to use.
    * @param vectorToBeOrthogonalTo the frame vector to be orthogonal to. Not modified.
    * @param normalize              whether to normalize the generated frame vector or not.
    * @return the random frame vector.
    */
   public static FrameVector3D nextOrthogonalFrameVector3D(Random random, FrameVector3DReadOnly vectorToBeOrthogonalTo, boolean normalize)
   {
      return nextOrthogonalFrameVector3D(random, vectorToBeOrthogonalTo.getReferenceFrame(), vectorToBeOrthogonalTo, normalize);
   }

   /**
    * Generates a random frame vector that is perpendicular to {@code vectorToBeOrthogonalTo}.
    *
    * @param random                 the random generator to use.
    * @param referenceFrame         the random frame vector's reference frame.
    * @param vectorToBeOrthogonalTo the vector to be orthogonal to. Not modified.
    * @param normalize              whether to normalize the generated frame vector or not.
    * @return the random frame vector.
    */
   public static FrameVector3D nextOrthogonalFrameVector3D(Random random,
                                                           ReferenceFrame referenceFrame,
                                                           Vector3DReadOnly vectorToBeOrthogonalTo,
                                                           boolean normalize)
   {
      return new FrameVector3D(referenceFrame, EuclidCoreRandomTools.nextOrthogonalVector3D(random, vectorToBeOrthogonalTo, normalize));
   }

   /**
    * Generates a random frame point.
    * <p>
    * {@code framePoint}<sub>i</sub> &in; [-1.0; 1.0].
    * </p>
    *
    * @param random         the random generator to use.
    * @param referenceFrame the random frame point's reference frame.
    * @return the random frame point.
    */
   public static FramePoint2D nextFramePoint2D(Random random, ReferenceFrame referenceFrame)
   {
      return new FramePoint2D(referenceFrame, EuclidCoreRandomTools.nextPoint2D(random));
   }

   /**
    * Generates a random frame point.
    * <p>
    * {@code framePoint}<sub>i</sub> &in; [-minMax; minMax].
    * </p>
    *
    * @param random         the random generator to use.
    * @param referenceFrame the random frame point's reference frame.
    * @param minMax         the maximum absolute value for each coordinate.
    * @return the random frame point.
    * @throws RuntimeException if {@code minMax < 0}.
    */
   public static FramePoint2D nextFramePoint2D(Random random, ReferenceFrame referenceFrame, double minMax)
   {
      return new FramePoint2D(referenceFrame, EuclidCoreRandomTools.nextPoint2D(random, minMax));
   }

   /**
    * Generates a random frame point.
    * <p>
    * {@code framePoint.x} &in; [min; max]. <br>
    * {@code framePoint.y} &in; [min; max]. <br>
    * </p>
    *
    * @param random         the random generator to use.
    * @param referenceFrame the random frame point's reference frame.
    * @param min            the minimum value for each coordinate.
    * @param max            the maximum value for each coordinate.
    * @return the random frame point.
    * @throws RuntimeException if {@code min > max}.
    */
   public static FramePoint2D nextFramePoint2D(Random random, ReferenceFrame referenceFrame, double min, double max)
   {
      return new FramePoint2D(referenceFrame, EuclidCoreRandomTools.nextPoint2D(random, min, max));
   }

   /**
    * Generates a random frame point.
    * <p>
    * {@code framePoint.x} &in; [minX; maxX]. <br>
    * {@code framePoint.y} &in; [minY; maxY]. <br>
    * </p>
    *
    * @param random         the random generator to use.
    * @param referenceFrame the random frame point's reference frame.
    * @param minX           the minimum value for the x-coordinate.
    * @param maxX           the maximum value for the x-coordinate.
    * @param minY           the minimum value for the y-coordinate.
    * @param maxY           the maximum value for the y-coordinate.
    * @return the random point.
    * @throws RuntimeException if {@code minX > maxX} or {@code minY > maxY}.
    */
   public static FramePoint2D nextFramePoint2D(Random random, ReferenceFrame referenceFrame, double minX, double maxX, double minY, double maxY)
   {
      return new FramePoint2D(referenceFrame, EuclidCoreRandomTools.nextPoint2D(random, minX, maxX, minY, maxY));
   }

   /**
    * Generates a random frame vector.
    * <p>
    * {@code frameVector}<sub>i</sub> &in; [-1.0; 1.0].
    * </p>
    *
    * @param random         the random generator to use.
    * @param referenceFrame the random frame point's reference frame.
    * @return the random frame vector.
    */
   public static FrameVector2D nextFrameVector2D(Random random, ReferenceFrame referenceFrame)
   {
      return new FrameVector2D(referenceFrame, EuclidCoreRandomTools.nextVector2D(random));
   }

   /**
    * Generates a random frame unit vector.
    * <p>
    * This generator uses {@link EuclidCoreRandomTools#nextVector2D(Random)}.
    * </p>
    *
    * @param random         the random generator to use.
    * @param referenceFrame the random frame unit vector's reference frame.
    * @return the random frame unit vector.
    */
   public static FrameUnitVector2D nextFrameUnitVector2D(Random random, ReferenceFrame referenceFrame)
   {
      return new FrameUnitVector2D(referenceFrame, EuclidCoreRandomTools.nextUnitVector2D(random));
   }

   /**
    * Generates a random frame vector.
    * <p>
    * {@code frameVector}<sub>i</sub> &in; [{@code min}; {@code max}].
    * </p>
    *
    * @param random         the random generator to use.
    * @param referenceFrame the random frame point's reference frame.
    * @param min            upper-bound for each component of the generated frame vector. Not modified.
    * @param max            lower-bound for each component of the generated frame vector. Not modified.
    * @return the random frame vector.
    * @throws RuntimeException if {@code min > max}.
    */
   public static FrameVector2D nextFrameVector2D(Random random, ReferenceFrame referenceFrame, double min, double max)
   {
      return new FrameVector2D(referenceFrame, EuclidCoreRandomTools.nextVector2D(random, min, min));
   }

   /**
    * Generates a random frame vector given its length {@code length}.
    *
    * @param random         the random generator to use.
    * @param referenceFrame the random frame point's reference frame.
    * @param length         the length of the generated frame vector.
    * @return the random frame vector.
    */
   public static FrameVector2D nextFrameVector2DWithFixedLength(Random random, ReferenceFrame referenceFrame, double length)
   {
      return new FrameVector2D(referenceFrame, EuclidCoreRandomTools.nextVector2DWithFixedLength(random, length));
   }

   /**
    * Generates a random frame vector.
    * <p>
    * {@code frameVector}<sub>i</sub> &in; [-{@code minMax}<sub>i</sub>; {@code minMax}<sub>i</sub>].
    * </p>
    *
    * @param random         the random generator to use.
    * @param referenceFrame the random frame point's reference frame.
    * @param minMax         tuple used to bound the maximum absolute value of each component of the
    *                       generated frame vector. Not modified.
    * @return the random frame vector.
    * @throws RuntimeException if any component of {@code minMax} is negative.
    */
   public static FrameVector2D nextFrameVector2D(Random random, ReferenceFrame referenceFrame, Tuple2DReadOnly minMax)
   {
      return new FrameVector2D(referenceFrame, EuclidCoreRandomTools.nextVector2D(random, minMax));
   }

   /**
    * Generates a random frame vector.
    * <p>
    * {@code frameVector}<sub>i</sub> &in; [{@code min}<sub>i</sub>; {@code max}<sub>i</sub>].
    * </p>
    *
    * @param random         the random generator to use.
    * @param referenceFrame the random frame point's reference frame.
    * @param min            tuple used as upper-bound for each component of the generated frame vector.
    *                       Not modified.
    * @param max            tuple used as lower-bound for each component of the generated frame vector.
    *                       Not modified.
    * @return the random frame vector.
    * @throws RuntimeException if {@code min}<sub>i</sub> > {@code max}<sub>i</sub>.
    */
   public static FrameVector2D nextFrameVector2D(Random random, ReferenceFrame referenceFrame, Tuple2DReadOnly min, Tuple2DReadOnly max)
   {
      return new FrameVector2D(referenceFrame, EuclidCoreRandomTools.nextVector2D(random, min, max));
   }

   /**
    * Generates random a yaw-pitch-roll orientation.
    * <p>
    * <ul>
    * <li>yaw &in; [-<i>pi</i>; <i>pi</i>],
    * <li>pitch &in; [-<i>pi</i>/2.0; <i>pi</i>/2.0],
    * <li>roll &in; [-<i>pi</i>; <i>pi</i>],
    * </ul>
    * </p>
    *
    * @param random         the random generator to use.
    * @param referenceFrame the random frame yaw-pitch-roll reference frame.
    * @return the random frame yaw-pitch-roll orientation.
    */
   public static FrameYawPitchRoll nextFrameYawPitchRoll(Random random, ReferenceFrame referenceFrame)
   {
      return new FrameYawPitchRoll(referenceFrame, EuclidCoreRandomTools.nextYawPitchRoll(random));
   }

   /**
    * Generates random a yaw-pitch-roll orientation.
    * <p>
    * <ul>
    * <li>yaw &in; [-{@code minMaxYaw}; {@code minMaxYaw}],
    * <li>pitch &in; [-{@code minMaxPitch}; {@code minMaxPitch}],
    * <li>roll &in; [-{@code minMaxRoll}; {@code minMaxRoll}],
    * </ul>
    * </p>
    *
    * @param random         the random generator to use.
    * @param referenceFrame the random frame yaw-pitch-roll reference frame.
    * @param minMaxYaw      the maximum absolute angle for the generated yaw angle.
    * @param minMaxPitch    the maximum absolute angle for the generated pitch angle.
    * @param minMaxRoll     the maximum absolute angle for the generated roll angle.
    * @return the random frame yaw-pitch-roll orientation.
    * @throws RuntimeException if {@code minMaxYaw < 0}, {@code minMaxPitch < 0},
    *                          {@code minMaxRoll < 0}.
    */
   public static FrameYawPitchRoll nextFrameYawPitchRoll(Random random, ReferenceFrame referenceFrame, double minMaxYaw, double minMaxPitch, double minMaxRoll)
   {
      return new FrameYawPitchRoll(referenceFrame, EuclidCoreRandomTools.nextYawPitchRoll(random, minMaxYaw, minMaxPitch, minMaxRoll));
   }

   /**
    * Generates a random yaw-pitch-roll orientation uniformly distributed on the unit-sphere.
    * <p>
    * The rotation magnitude described by the generated orientation is in [-{@code minMaxAngle};
    * {@code minMaxAngle}].
    * </p>
    *
    * @param random         the random generator to use.
    * @param referenceFrame the random frame yaw-pitch-roll reference frame.
    * @param minMaxAngle    the maximum absolute angle described by the generated orientation.
    * @return the random frame yaw-pitch-roll orientation.
    * @throws RuntimeException if {@code minMaxAngle < 0}.
    */
   public static FrameYawPitchRoll nextFrameYawPitchRollUniform(Random random, ReferenceFrame referenceFrame, double minMaxAngle)
   {
      return new FrameYawPitchRoll(referenceFrame, EuclidCoreRandomTools.nextYawPitchRollUniform(random, minMaxAngle));
   }

   /**
    * Generates a random frame quaternion uniformly distributed on the unit-sphere.
    * <p>
    * The rotation magnitude described by the generated quaternion is in [-<i>pi</i>; <i>pi</i>].
    * </p>
    *
    * @param random         the random generator to use.
    * @param referenceFrame the random frame quaternion's reference frame.
    * @return the random frame quaternion.
    */
   public static FrameQuaternion nextFrameQuaternion(Random random, ReferenceFrame referenceFrame)
   {
      return new FrameQuaternion(referenceFrame, EuclidCoreRandomTools.nextQuaternion(random));
   }

   /**
    * Generates a random frame quaternion uniformly distributed on the unit-sphere.
    * <p>
    * The rotation magnitude described by the generated quaternion is in [-{@code minMaxAngle};
    * {@code minMaxAngle}].
    * </p>
    *
    * @param random         the random generator to use.
    * @param referenceFrame the random frame quaternion's reference frame.
    * @param minMaxAngle    the maximum absolute angle described by the generated quaternion.
    * @return the random frame quaternion.
    * @throws RuntimeException if {@code minMaxAngle < 0}.
    */
   public static FrameQuaternion nextFrameQuaternion(Random random, ReferenceFrame referenceFrame, double minMaxAngle)
   {
      return new FrameQuaternion(referenceFrame, EuclidCoreRandomTools.nextQuaternion(random, minMaxAngle));
   }

   /**
    * Generates an orientation which both value and type are random.
    * <p>
    * The type can be either: quaternion, or yaw-pitch-roll.
    * </p>
    *
    * @param random         the random generator to use.
    * @param referenceFrame the random frame orientation's reference frame.
    * @return the random orientation 3D.
    */
   public static FrameOrientation3DBasics nextFrameOrientation3D(Random random, ReferenceFrame referenceFrame)
   {
      switch (random.nextInt(3))
      {
         case 0:
            return nextFrameQuaternion(random, referenceFrame);
         case 1:
            return nextFrameRotationMatrix(random, referenceFrame);
         default:
            return nextFrameYawPitchRoll(random, referenceFrame);
      }
   }

   /**
    * Generates a random 4D frame vector.
    * <p>
    * {@code vector}<sub>i</sub> &in; [-1.0; 1.0].
    * </p>
    *
    * @param random         the random generator to use.
    * @param referenceFrame the random frame vector's reference frame.
    * @return the random 4D frame vector.
    */
   public static FrameVector4D nextFrameVector4D(Random random, ReferenceFrame referenceFrame)
   {
      return new FrameVector4D(referenceFrame, EuclidCoreRandomTools.nextVector4D(random));
   }

   /**
    * Generates a random 2D frame orientation with a yaw uniformly distributed in [-<i>pi</i>;
    * <i>pi</i>].
    *
    * @param random         the random generator to use.
    * @param referenceFrame the random frame orientation's reference frame.
    * @return the random 2D frame orientation.
    */
   public static FrameOrientation2D nextFrameOrientation2D(Random random, ReferenceFrame referenceFrame)
   {
      return new FrameOrientation2D(referenceFrame, EuclidCoreRandomTools.nextOrientation2D(random));
   }

   /**
    * Generates a random 2D frame pose with a yaw uniformly distributed in [-<i>pi</i>; <i>pi</i>].
    *
    * @param random         the random generator to use.
    * @param referenceFrame the random frame pose reference frame.
    * @return the random 2D frame pose.
    */
   public static FramePose2D nextFramePose2D(Random random, ReferenceFrame referenceFrame)
   {
      return new FramePose2D(referenceFrame, EuclidGeometryRandomTools.nextPose2D(random));
   }

   /**
    * Generates a random 2D frame pose.
    * <p>
    * {@code pose.position}<sub>i</sub> &in; [-{@code positionMinMax}; {@code positionMinMax}].<br>
    * The rotation magnitude described by the orientation part of the generated pose is in
    * [-{@code orientationMinMax}; {@code orientationMinMax}].
    * </p>
    *
    * @param random            the random generator to use.
    * @param referenceFrame    the random frame pose reference frame.
    * @param positionMinMax    the maximum absolute value of each position coordinate.
    * @param orientationMinMax the maximum absolute value of the orientation's magnitude.
    * @return the random 2D frame pose.
    */
   public static FramePose2D nextFramePose2D(Random random, ReferenceFrame referenceFrame, double positionMinMax, double orientationMinMax)
   {
      return new FramePose2D(referenceFrame, EuclidGeometryRandomTools.nextPose2D(random, positionMinMax, orientationMinMax));
   }

   /**
    * Generates a random 3D frame pose with a quaternion uniformly distributed on the unit-sphere.
    * <p>
    * The rotation magnitude described by the generated quaternion is in [-<i>pi</i>; <i>pi</i>].
    * </p>
    *
    * @param random         the random generator to use.
    * @param referenceFrame the random frame pose's reference frame.
    * @return the random 3D frame pose.
    */
   public static FramePose3D nextFramePose3D(Random random, ReferenceFrame referenceFrame)
   {
      return new FramePose3D(referenceFrame, EuclidGeometryRandomTools.nextPose3D(random));
   }

   /**
    * Generates a random 3D frame pose with a quaternion uniformly distributed on the unit-sphere.
    * <p>
    * {@code pose.position}<sub>X</sub> &in; [-{@code maxAbsoluteX}; {@code maxAbsoluteX}].<br>
    * {@code pose.position}<sub>Y</sub> &in; [-{@code maxAbsoluteY}; {@code maxAbsoluteY}].<br>
    * {@code pose.position}<sub>Z</sub> &in; [-{@code maxAbsoluteZ}; {@code maxAbsoluteZ}].<br>
    * The rotation magnitude described by the generated quaternion is in [-<i>pi</i>; <i>pi</i>].
    * </p>
    *
    * @param random         the random generator to use.
    * @param referenceFrame the random frame pose's reference frame.
    * @param maxAbsoluteX   the maximum absolute value of the position x-coordinate.
    * @param maxAbsoluteY   the maximum absolute value of the position y-coordinate.
    * @param maxAbsoluteZ   the maximum absolute value of the position z-coordinate.
    * @return the random 3D frame pose.
    */
   public static FramePose3D nextFramePose3D(Random random, ReferenceFrame referenceFrame, double maxAbsoluteX, double maxAbsoluteY, double maxAbsoluteZ)
   {
      return new FramePose3D(referenceFrame, EuclidGeometryRandomTools.nextPose3D(random, maxAbsoluteX, maxAbsoluteY, maxAbsoluteZ));
   }

   /**
    * Generates a random 3D frame pose with a quaternion uniformly distributed on the unit-sphere.
    * <p>
    * {@code pose.position}<sub>i</sub> &in; [-{@code positionMinMax}; {@code positionMinMax}].<br>
    * The rotation magnitude described by the orientation part of the generated pose is in
    * [-{@code orientationMinMax}; {@code orientationMinMax}].
    * </p>
    *
    * @param random            the random generator to use.
    * @param referenceFrame    the random frame pose's reference frame.
    * @param positionMinMax    the maximum absolute value of each position coordinate.
    * @param orientationMinMax the maximum absolute value of the orientation's magnitude.
    * @return the random 3D frame pose.
    */
   public static FramePose3D nextFramePose3D(Random random, ReferenceFrame referenceFrame, double positionMinMax, double orientationMinMax)
   {
      return new FramePose3D(referenceFrame, EuclidGeometryRandomTools.nextPose3D(random, positionMinMax, orientationMinMax));
   }

   /**
    * Generates a random 2D frame line.
    *
    * @param random         the random generator to use.
    * @param referenceFrame the random frame line's reference frame.
    * @return the random frame line.
    */
   public static FrameLine2D nextFrameLine2D(Random random, ReferenceFrame referenceFrame)
   {
      return new FrameLine2D(referenceFrame, EuclidGeometryRandomTools.nextLine2D(random));
   }

   /**
    * Generates a random 2D frame line.
    *
    * @param random         the random generator to use.
    * @param referenceFrame the random frame line's reference frame.
    * @return the random frame line.
    */
   public static FrameLine3D nextFrameLine3D(Random random, ReferenceFrame referenceFrame)
   {
      return new FrameLine3D(referenceFrame, EuclidGeometryRandomTools.nextLine3D(random));
   }

   /**
    * Generates a random 2D frame line segment.
    *
    * @param random         the random generator to use.
    * @param referenceFrame the random frame line segment's reference frame.
    * @return the random frame line segment.
    */
   public static FrameLineSegment2D nextFrameLineSegment2D(Random random, ReferenceFrame referenceFrame)
   {
      return new FrameLineSegment2D(referenceFrame, EuclidGeometryRandomTools.nextLineSegment2D(random));
   }

   /**
    * Generates a random 3D frame line segment.
    *
    * @param random         the random generator to use.
    * @param referenceFrame the random frame line segment's reference frame.
    * @return the random frame line segment.
    */
   public static FrameLineSegment3D nextFrameLineSegment3D(Random random, ReferenceFrame referenceFrame)
   {
      return new FrameLineSegment3D(referenceFrame, EuclidGeometryRandomTools.nextLineSegment3D(random));
   }

   /**
    * Generates a random bounding box from random center location and random size.
    *
    * @param random         the random generator to use.
    * @param referenceFrame the random frame bounding box's reference frame.
    * @return the random bounding box.
    */
   public static FrameBoundingBox2D nextFrameBoundingBox2D(Random random, ReferenceFrame referenceFrame)
   {
      return new FrameBoundingBox2D(referenceFrame, EuclidGeometryRandomTools.nextBoundingBox2D(random));
   }

   /**
    * Generates a random bounding box from random center location and random size.
    *
    * @param random         the random generator to use.
    * @param referenceFrame the random frame bounding box's reference frame.
    * @param centerMinMax   the maximum absolute value for each coordinate of the bounding box center.
    * @param sizeMax        the maximum size along each axis for the bounding box.
    * @return the random bounding box.
    * @throws RuntimeException if {@code centerMinMax < 0} or {@code sizeMax < 0}.
    */
   public static FrameBoundingBox2D nextFrameBoundingBox2D(Random random, ReferenceFrame referenceFrame, double centerMinMax, double sizeMax)
   {
      return new FrameBoundingBox2D(referenceFrame, EuclidGeometryRandomTools.nextBoundingBox2D(random, centerMinMax, sizeMax));
   }

   /**
    * Generates a random bounding box from random center location and random size.
    *
    * @param random         the random generator to use.
    * @param referenceFrame the random frame bounding box's reference frame.
    * @return the random bounding box.
    */
   public static FrameBoundingBox3D nextFrameBoundingBox3D(Random random, ReferenceFrame referenceFrame)
   {
      return new FrameBoundingBox3D(referenceFrame, EuclidGeometryRandomTools.nextBoundingBox3D(random));
   }

   /**
    * Generates a random bounding box from random center location and random size.
    *
    * @param random         the random generator to use.
    * @param referenceFrame the random frame bounding box's reference frame.
    * @param centerMinMax   the maximum absolute value for each coordinate of the bounding box center.
    * @param sizeMax        the maximum size along each axis for the bounding box.
    * @return the random bounding box.
    * @throws RuntimeException if {@code centerMinMax < 0} or {@code sizeMax < 0}.
    */
   public static FrameBoundingBox3D nextFrameBoundingBox3D(Random random, ReferenceFrame referenceFrame, double centerMinMax, double sizeMax)
   {
      return new FrameBoundingBox3D(referenceFrame, EuclidGeometryRandomTools.nextBoundingBox3D(random, centerMinMax, sizeMax));
   }

   /**
    * Generates a random convex polygon given the maximum absolute coordinate value of its vertices and
    * the size of the point cloud from which it is generated.
    *
    * @param random         the random generator to use.
    * @param referenceFrame the polygon's reference frame.
    * @return the random convex polygon.
    * @throws RuntimeException if {@code maxAbsoluteXY < 0}.
    */
   public static FrameConvexPolygon2D nextFrameConvexPolygon2D(Random random, ReferenceFrame referenceFrame)
   {
      return new FrameConvexPolygon2D(referenceFrame, EuclidGeometryRandomTools.nextConvexPolygon2D(random, 1.0, 10));
   }

   /**
    * Generates a random convex polygon given the maximum absolute coordinate value of its vertices and
    * the size of the point cloud from which it is generated.
    *
    * @param random                 the random generator to use.
    * @param referenceFrame         the polygon's reference frame.
    * @param maxAbsoluteXY          the maximum absolute value for each coordinate of the vertices.
    * @param numberOfPossiblePoints the size of the point cloud to generate that is used for computing
    *                               the random convex polygon. The size of the resulting convex polygon
    *                               will be less than {@code numberOfPossiblePoints}.
    * @return the random convex polygon.
    * @throws RuntimeException if {@code maxAbsoluteXY < 0}.
    */
   public static FrameConvexPolygon2D nextFrameConvexPolygon2D(Random random, ReferenceFrame referenceFrame, double maxAbsoluteXY, int numberOfPossiblePoints)
   {
      return new FrameConvexPolygon2D(referenceFrame, EuclidGeometryRandomTools.nextConvexPolygon2D(random, maxAbsoluteXY, numberOfPossiblePoints));
   }

   /**
    * Generates a fixed-size supplier of random frame vertex 2D.
    *
    * @param random         the random generator to use.
    * @param referenceFrame the reference frame for the vertices.
    * @return the random supplier.
    */
   public static FrameVertex2DSupplier nextFrameVertex2DSupplier(Random random, ReferenceFrame referenceFrame)
   {
      return nextFrameVertex2DSupplier(random, referenceFrame, 20);
   }

   /**
    * Generates a fixed-size supplier of random frame vertex 2D.
    *
    * @param random           the random generator to use.
    * @param referenceFrame   the reference frame for the vertices.
    * @param numberOfVertices the supplier's size.
    * @return the random supplier.
    */
   public static FrameVertex2DSupplier nextFrameVertex2DSupplier(Random random, ReferenceFrame referenceFrame, int numberOfVertices)
   {
      return new FrameVertex2DSupplier()
      {
         Vertex2DSupplier vertex2dSupplier = EuclidGeometryRandomTools.nextVertex2DSupplier(random, numberOfVertices);

         @Override
         public int getNumberOfVertices()
         {
            return vertex2dSupplier.getNumberOfVertices();
         }

         @Override
         public FramePoint2DReadOnly getVertex(int index)
         {
            return new FramePoint2D(referenceFrame, vertex2dSupplier.getVertex(index));
         }
      };
   }

   /**
    * Generates a fixed-size supplier of random frame vertex 3D.
    *
    * @param random         the random generator to use.
    * @param referenceFrame the reference frame for the vertices.
    * @return the random supplier.
    */
   public static FrameVertex3DSupplier nextFrameVertex3DSupplier(Random random, ReferenceFrame referenceFrame)
   {
      return nextFrameVertex3DSupplier(random, referenceFrame, 20);
   }

   /**
    * Generates a fixed-size supplier of random frame vertex 3D.
    *
    * @param random           the random generator to use.
    * @param referenceFrame   the reference frame for the vertices.
    * @param numberOfVertices the supplier's size.
    * @return the random supplier.
    */
   public static FrameVertex3DSupplier nextFrameVertex3DSupplier(Random random, ReferenceFrame referenceFrame, int numberOfVertices)
   {
      return new FrameVertex3DSupplier()
      {
         Vertex3DSupplier vertex2dSupplier = EuclidGeometryRandomTools.nextVertex3DSupplier(random, numberOfVertices);

         @Override
         public int getNumberOfVertices()
         {
            return vertex2dSupplier.getNumberOfVertices();
         }

         @Override
         public FramePoint3DReadOnly getVertex(int index)
         {
            return new FramePoint3D(referenceFrame, vertex2dSupplier.getVertex(index));
         }
      };
   }

   /**
    * Generates a random 3-by-3 frame matrix.
    * <p>
    * {@code matrix}<sub>ij</sub> &in; [-1.0; 1.0].
    * </p>
    *
    * @param random         the random generator to use.
    * @param referenceFrame the frame in which the generated matrix is to be created.
    * @return the random frame matrix.
    */
   public static FrameMatrix3D nextFrameMatrix3D(Random random, ReferenceFrame referenceFrame)
   {
      return new FrameMatrix3D(referenceFrame, EuclidCoreRandomTools.nextMatrix3D(random));
   }

   /**
    * Generates a random 3-by-3 frame matrix.
    * <p>
    * {@code matrix}<sub>ij</sub> &in; [-{@code minMaxValue}; {@code minMaxValue}].
    * </p>
    *
    * @param random         the random generator to use.
    * @param minMaxValue    the maximum absolute value for each element.
    * @param referenceFrame the frame in which the generated matrix is to be created.
    * @return the random frame matrix.
    * @throws RuntimeException if {@code minMaxValue < 0}.
    */
   public static FrameMatrix3D nextFrameMatrix3D(Random random, ReferenceFrame referenceFrame, double minMaxValue)
   {
      return new FrameMatrix3D(referenceFrame, EuclidCoreRandomTools.nextMatrix3D(random, minMaxValue));
   }

   /**
    * Generates a random 3-by-3 frame matrix.
    * <p>
    * {@code matrix}<sub>ij</sub> &in; [{@code minValue}; {@code maxValue}].
    * </p>
    *
    * @param random         the random generator to use.
    * @param minValue       the minimum value for each element.
    * @param maxValue       the maximum value for each element.
    * @param referenceFrame the frame in which the generated matrix is to be created.
    * @return the random frame matrix.
    * @throws RuntimeException if {@code minValue > maxValue}.
    */
   public static FrameMatrix3D nextFrameMatrix3D(Random random, ReferenceFrame referenceFrame, double minValue, double maxValue)
   {
      return new FrameMatrix3D(referenceFrame, EuclidCoreRandomTools.nextMatrix3D(random, minValue, maxValue));
   }

   /**
    * Generates a random frame rotation matrix uniformly distributed on the unit-sphere.
    * <p>
    * The rotation magnitude described by the generated rotation matrix is in [-<i>pi</i>; <i>pi</i>].
    * </p>
    *
    * @param random         the random generator to use.
    * @param referenceFrame the random frame rotation matrix reference frame.
    * @return the random frame rotation matrix.
    */
   public static FrameRotationMatrix nextFrameRotationMatrix(Random random, ReferenceFrame referenceFrame)
   {
      return new FrameRotationMatrix(referenceFrame, EuclidCoreRandomTools.nextQuaternion(random));
   }

   /**
    * Generates a random frame rotation matrix uniformly distributed on the unit-sphere.
    * <p>
    * The rotation magnitude described by the generated rotation matrix is in [-{@code minMaxAngle};
    * {@code minMaxAngle}].
    * </p>
    *
    * @param random         the random generator to use.
    * @param referenceFrame the random frame rotation matrix's reference frame.
    * @param minMaxAngle    the maximum absolute angle described by the generated rotation matrix.
    * @return the random frame rotation matrix.
    * @throws RuntimeException if {@code minMaxAngle < 0}.
    */
   public static FrameRotationMatrix nextFrameRotationMatrix(Random random, ReferenceFrame referenceFrame, double minMaxAngle)
   {
      return new FrameRotationMatrix(referenceFrame, EuclidCoreRandomTools.nextQuaternion(random, minMaxAngle));
   }

   /**
    * Generates a random diagonal 3-by-3 frame matrix.
    * <p>
    * <ul>
    * <li>{@code matrix.getM00()} &in; [-1.0; 1.0].
    * <li>{@code matrix.getM11()} &in; [-1.0; 1.0].
    * <li>{@code matrix.getM22()} &in; [-1.0; 1.0].
    * </ul>
    * </p>
    *
    * @param random         the random generator to use.
    * @param referenceFrame the frame in which the generated matrix is to be created.
    * @return the random diagonal frame matrix.
    */
   public static FrameMatrix3D nextDiagonalFrameMatrix3D(Random random, ReferenceFrame referenceFrame)
   {
      return new FrameMatrix3D(referenceFrame, EuclidCoreRandomTools.nextDiagonalMatrix3D(random));
   }

   /**
    * Generates a random diagonal 3-by-3 frame matrix.
    * <p>
    * <ul>
    * <li>{@code matrix.getM00()} &in; [-{@code minMaxValue}; {@code minMaxValue}].
    * <li>{@code matrix.getM11()} &in; [-{@code minMaxValue}; {@code minMaxValue}].
    * <li>{@code matrix.getM22()} &in; [-{@code minMaxValue}; {@code minMaxValue}].
    * </ul>
    * </p>
    *
    * @param random         the random generator to use.
    * @param minMaxValue    the maximum absolute value for each diagonal element.
    * @param referenceFrame the frame in which the generated matrix is to be created.
    * @return the random diagonal frame matrix.
    * @throws RuntimeException if {@code minMaxValue < 0}.
    */
   public static FrameMatrix3D nextDiagonalFrameMatrix3D(Random random, ReferenceFrame referenceFrame, double minMaxValue)
   {
      return new FrameMatrix3D(referenceFrame, EuclidCoreRandomTools.nextDiagonalMatrix3D(random, minMaxValue));
   }

   /**
    * Generates a random diagonal 3-by-3 matrix.
    * <p>
    * <ul>
    * <li>{@code matrix.getM00()} &in; [{@code minValue}; {@code maxValue}].
    * <li>{@code matrix.getM11()} &in; [{@code minValue}; {@code maxValue}].
    * <li>{@code matrix.getM22()} &in; [{@code minValue}; {@code maxValue}].
    * </ul>
    * </p>
    *
    * @param random         the random generator to use.
    * @param minValue       the minimum value of each diagonal element.
    * @param maxValue       the maximum value of each diagonal element.
    * @param referenceFrame the frame in which the generated matrix is to be created.
    * @return the random diagonal frame matrix.
    * @throws RuntimeException if {@code minValue > maxValue}.
    */
   public static FrameMatrix3D nextDiagonalFrameMatrix3D(Random random, ReferenceFrame referenceFrame, double minValue, double maxValue)
   {
      return new FrameMatrix3D(referenceFrame, EuclidCoreRandomTools.nextDiagonalMatrix3D(random, minValue, maxValue));
   }
}
