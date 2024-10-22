package us.ihmc.euclid.referenceFrame.tools;

import java.util.Collection;
import java.util.List;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.Location;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.exceptions.BoundingBoxException;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.orientation.interfaces.Orientation3DBasics;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.EuclidFrameGeometry;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameOrientation3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameOrientation3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameOrientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * Extension of the tools provided in {@link EuclidGeometryTools} for frame geometries.
 *
 * @author Sylvain Bertrand
 */
public class EuclidFrameTools
{
   private EuclidFrameTools()
   {
      // Suppresses default constructor, ensuring non-instantiability.
   }

   /**
    * Tests if the two given lines are collinear given a tolerance on the angle between in the range
    * ]0; <i>pi</i>/2[. This method returns {@code true} if the two lines are collinear, whether they
    * are pointing in the same direction or in opposite directions.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the direction magnitude of either line is below
    * {@link EuclidGeometryTools#ONE_TEN_MILLIONTH}, this method fails and returns {@code false}.
    * </ul>
    * </p>
    *
    * @param firstPointOnLine1  a first point located on the first line. Not modified.
    * @param secondPointOnLine1 a second point located on the first line. Not modified.
    * @param firstPointOnLine2  a first point located on the second line. Not modified.
    * @param secondPointOnLine2 a second point located on the second line. Not modified.
    * @param angleEpsilon       tolerance on the angle in radians.
    * @param distanceEpsilon    tolerance on the distance to determine if {@code firstPointOnLine2}
    *                           belongs to the first line segment.
    * @return {@code true} if the two line segments are collinear, {@code false} otherwise.
    * @throws IllegalArgumentException        if <tt>angleEpsilon</tt> &notin; [0; <i>pi</i>/2]
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static boolean areLine2DsCollinear(FramePoint2DReadOnly firstPointOnLine1,
                                             FramePoint2DReadOnly secondPointOnLine1,
                                             FramePoint2DReadOnly firstPointOnLine2,
                                             FramePoint2DReadOnly secondPointOnLine2,
                                             double angleEpsilon,
                                             double distanceEpsilon)
   {
      firstPointOnLine1.checkReferenceFrameMatch(secondPointOnLine1, firstPointOnLine2, secondPointOnLine2);
      return EuclidGeometryTools.areLine2DsCollinear(firstPointOnLine1,
                                                     secondPointOnLine1,
                                                     firstPointOnLine2,
                                                     secondPointOnLine2,
                                                     angleEpsilon,
                                                     distanceEpsilon);
   }

   /**
    * Tests if the two given lines are collinear given a tolerance on the angle between in the range
    * ]0; <i>pi</i>/2[. This method returns {@code true} if the two lines are collinear, whether they
    * are pointing in the same direction or in opposite directions.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the direction magnitude of either line is below
    * {@link EuclidGeometryTools#ONE_TEN_MILLIONTH}, this method fails and returns {@code false}.
    * </ul>
    * </p>
    *
    * @param pointOnLine1       point located on the first line. Not modified.
    * @param lineDirection1     the first line direction. Not modified.
    * @param firstPointOnLine2  a first point located on the second line. Not modified.
    * @param secondPointOnLine2 a second point located on the second line. Not modified.
    * @param angleEpsilon       tolerance on the angle in radians.
    * @param distanceEpsilon    tolerance on the distance to determine if {@code firstPointOnLine2}
    *                           belongs to the first line segment.
    * @return {@code true} if the two line segments are collinear, {@code false} otherwise.
    * @throws IllegalArgumentException        if <tt>angleEpsilon</tt> &notin; [0; <i>pi</i>/2]
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static boolean areLine2DsCollinear(FramePoint2DReadOnly pointOnLine1,
                                             FrameVector2DReadOnly lineDirection1,
                                             FramePoint2DReadOnly firstPointOnLine2,
                                             FramePoint2DReadOnly secondPointOnLine2,
                                             double angleEpsilon,
                                             double distanceEpsilon)
   {
      pointOnLine1.checkReferenceFrameMatch(lineDirection1, firstPointOnLine2, secondPointOnLine2);
      return EuclidGeometryTools.areLine2DsCollinear(pointOnLine1, lineDirection1, firstPointOnLine2, secondPointOnLine2, angleEpsilon, distanceEpsilon);
   }

   /**
    * Tests if the two given lines are collinear given a tolerance on the angle between in the range
    * ]0; <i>pi</i>/2[. This method returns {@code true} if the two lines are collinear, whether they
    * are pointing in the same direction or in opposite directions.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the direction magnitude of either line is below
    * {@link EuclidGeometryTools#ONE_TEN_MILLIONTH}, this method fails and returns {@code false}.
    * </ul>
    * </p>
    *
    * @param pointOnLine1    point located on the first line. Not modified.
    * @param lineDirection1  the first line direction. Not modified.
    * @param pointOnLine2    point located on the second line. Not modified.
    * @param lineDirection2  the second line direction. Not modified.
    * @param angleEpsilon    tolerance on the angle in radians.
    * @param distanceEpsilon tolerance on the distance to determine if {@code pointOnLine2} belongs to
    *                        the first line segment.
    * @return {@code true} if the two line segments are collinear, {@code false} otherwise.
    * @throws IllegalArgumentException        if <tt>angleEpsilon</tt> &notin; [0; <i>pi</i>/2]
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static boolean areLine2DsCollinear(FramePoint2DReadOnly pointOnLine1,
                                             FrameVector2DReadOnly lineDirection1,
                                             FramePoint2DReadOnly pointOnLine2,
                                             FrameVector2DReadOnly lineDirection2,
                                             double angleEpsilon,
                                             double distanceEpsilon)
   {
      pointOnLine1.checkReferenceFrameMatch(lineDirection1, pointOnLine2, lineDirection2);
      return EuclidGeometryTools.areLine2DsCollinear(pointOnLine1, lineDirection1, pointOnLine2, lineDirection2, angleEpsilon, distanceEpsilon);
   }

   /**
    * Tests if the two given lines are collinear given a tolerance on the angle between in the range
    * ]0; <i>pi</i>/2[. This method returns {@code true} if the two lines are collinear, whether they
    * are pointing in the same direction or in opposite directions.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the direction magnitude of either line is below
    * {@link EuclidGeometryTools#ONE_TEN_MILLIONTH}, this method fails and returns {@code false}.
    * </ul>
    * </p>
    *
    * @param firstPointOnLine1  a first point located on the first line. Not modified.
    * @param secondPointOnLine1 a second point located on the first line. Not modified.
    * @param firstPointOnLine2  a first point located on the second line. Not modified.
    * @param secondPointOnLine2 a second point located on the second line. Not modified.
    * @param angleEpsilon       tolerance on the angle in radians.
    * @param distanceEpsilon    tolerance on the distance to determine if {@code firstPointOnLine2}
    *                           belongs to the first line segment.
    * @return {@code true} if the two line segments are collinear, {@code false} otherwise.
    * @throws IllegalArgumentException        if <tt>angleEpsilon</tt> &notin; [0; <i>pi</i>/2]
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static boolean areLine3DsCollinear(FramePoint3DReadOnly firstPointOnLine1,
                                             FramePoint3DReadOnly secondPointOnLine1,
                                             FramePoint3DReadOnly firstPointOnLine2,
                                             FramePoint3DReadOnly secondPointOnLine2,
                                             double angleEpsilon,
                                             double distanceEpsilon)
   {
      firstPointOnLine1.checkReferenceFrameMatch(secondPointOnLine1, firstPointOnLine2, secondPointOnLine2);
      return EuclidGeometryTools.areLine3DsCollinear(firstPointOnLine1,
                                                     secondPointOnLine1,
                                                     firstPointOnLine2,
                                                     secondPointOnLine2,
                                                     angleEpsilon,
                                                     distanceEpsilon);
   }

   /**
    * Tests if the two given lines are collinear given a tolerance on the angle between in the range
    * ]0; <i>pi</i>/2[. This method returns {@code true} if the two lines are collinear, whether they
    * are pointing in the same direction or in opposite directions.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the direction magnitude of either line is below
    * {@link EuclidGeometryTools#ONE_TEN_MILLIONTH}, this method fails and returns {@code false}.
    * </ul>
    * </p>
    *
    * @param pointOnLine1    point located on the first line. Not modified.
    * @param lineDirection1  the first line direction. Not modified.
    * @param pointOnLine2    point located on the second line. Not modified.
    * @param lineDirection2  the second line direction. Not modified.
    * @param angleEpsilon    tolerance on the angle in radians.
    * @param distanceEpsilon tolerance on the distance to determine if {@code pointOnLine2} belongs to
    *                        the first line segment.
    * @return {@code true} if the two line segments are collinear, {@code false} otherwise.
    * @throws IllegalArgumentException        if <tt>angleEpsilon</tt> &notin; [0; <i>pi</i>/2]
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static boolean areLine3DsCollinear(FramePoint3DReadOnly pointOnLine1,
                                             FrameVector3DReadOnly lineDirection1,
                                             FramePoint3DReadOnly pointOnLine2,
                                             FrameVector3DReadOnly lineDirection2,
                                             double angleEpsilon,
                                             double distanceEpsilon)
   {
      pointOnLine1.checkReferenceFrameMatch(lineDirection1, pointOnLine2, lineDirection2);
      return EuclidGeometryTools.areLine3DsCollinear(pointOnLine1, lineDirection1, pointOnLine2, lineDirection2, angleEpsilon, distanceEpsilon);
   }

   /**
    * Tests if the two given planes are coincident:
    * <ul>
    * <li>{@code planeNormal1} and {@code planeNormal2} are parallel given the tolerance
    * {@code angleEpsilon}.
    * <li>the distance of {@code pointOnPlane2} from the first plane is less than
    * {@code distanceEpsilon}.
    * </ul>
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the length of either normal is below {@link EuclidGeometryTools#ONE_TEN_MILLIONTH}, this
    * method fails and returns {@code false}.
    * </ul>
    * </p>
    *
    * @param pointOnPlane1   a point on the first plane. Not modified.
    * @param planeNormal1    the normal of the first plane. Not modified.
    * @param pointOnPlane2   a point on the second plane. Not modified.
    * @param planeNormal2    the normal of the second plane. Not modified.
    * @param angleEpsilon    tolerance on the angle in radians to determine if the plane normals are
    *                        parallel.
    * @param distanceEpsilon tolerance on the distance to determine if {@code pointOnPlane2} belongs to
    *                        the first plane.
    * @return {@code true} if the two planes are coincident, {@code false} otherwise.
    * @throws IllegalArgumentException        if <tt>angleEpsilon</tt> &notin; [0; <i>pi</i>/2]
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static boolean arePlane3DsCoincident(FramePoint3DReadOnly pointOnPlane1,
                                               FrameVector3DReadOnly planeNormal1,
                                               FramePoint3DReadOnly pointOnPlane2,
                                               FrameVector3DReadOnly planeNormal2,
                                               double angleEpsilon,
                                               double distanceEpsilon)
   {
      pointOnPlane1.checkReferenceFrameMatch(planeNormal1, pointOnPlane2, planeNormal2);
      return EuclidGeometryTools.arePlane3DsCoincident(pointOnPlane1, planeNormal1, pointOnPlane2, planeNormal2, angleEpsilon, distanceEpsilon);
   }

   /**
    * Tests if the two given vectors are parallel given a tolerance on the angle between the two vector
    * axes in the range ]0; <i>pi</i>/2[. This method returns {@code true} if the two vectors are
    * parallel, whether they are pointing in the same direction or in opposite directions.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the length of either vector is below {@link EuclidGeometryTools#ONE_TEN_MILLIONTH}, this
    * method fails and returns {@code false}.
    * </ul>
    * </p>
    *
    * @param firstVector  the first vector. Not modified.
    * @param secondVector the second vector. Not modified.
    * @param angleEpsilon tolerance on the angle in radians.
    * @return {@code true} if the two vectors are parallel, {@code false} otherwise.
    * @throws IllegalArgumentException        if <tt>angleEpsilon</tt> &notin; [0; <i>pi</i>/2]
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static boolean areVector2DsParallel(FrameVector2DReadOnly firstVector, FrameVector2DReadOnly secondVector, double angleEpsilon)
   {
      firstVector.checkReferenceFrameMatch(secondVector);
      return EuclidGeometryTools.areVector2DsParallel(firstVector, secondVector, angleEpsilon);
   }

   /**
    * Tests if the two given vectors are parallel given a tolerance on the angle between the two vector
    * axes in the range ]0; <i>pi</i>/2[. This method returns {@code true} if the two vectors are
    * parallel, whether they are pointing in the same direction or in opposite directions.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the length of either vector is below {@link EuclidGeometryTools#ONE_TEN_MILLIONTH}, this
    * method fails and returns {@code false}.
    * </ul>
    * </p>
    *
    * @param firstVector  the first vector. Not modified.
    * @param secondVector the second vector. Not modified.
    * @param angleEpsilon tolerance on the angle in radians.
    * @return {@code true} if the two vectors are parallel, {@code false} otherwise.
    * @throws IllegalArgumentException        if <tt>angleEpsilon</tt> &notin; [0; <i>pi</i>/2]
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static boolean areVector3DsParallel(FrameVector3DReadOnly firstVector, FrameVector3DReadOnly secondVector, double angleEpsilon)
   {
      firstVector.checkReferenceFrameMatch(secondVector);
      return EuclidGeometryTools.areVector3DsParallel(firstVector, secondVector, angleEpsilon);
   }

   /**
    * Computes the average 2D point from a given collection of 2D points.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param points the collection of 2D points to compute the average from. Not modified.
    * @return the computed average.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static FramePoint2D averagePoint2Ds(Collection<? extends FramePoint2DReadOnly> points)
   {
      if (points.isEmpty())
         return null;

      FramePoint2D totalPoint = new FramePoint2D(points.iterator().next().getReferenceFrame());

      for (FramePoint2DReadOnly point : points)
      {
         totalPoint.add(point);
      }

      totalPoint.scale(1.0 / points.size());

      return totalPoint;
   }

   /**
    * Computes the average 3D point from a given collection of 3D points.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param points the collection of 3D points to compute the average from. Not modified.
    * @return the computed average.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static FramePoint3D averagePoint3Ds(Collection<? extends FramePoint3DReadOnly> points)
   {
      if (points.isEmpty())
         return null;

      FramePoint3D totalPoint = new FramePoint3D(points.iterator().next().getReferenceFrame());

      for (FramePoint3DReadOnly point : points)
      {
         totalPoint.add(point);
      }

      totalPoint.scale(1.0 / points.size());

      return totalPoint;
   }

   /**
    * Returns the average of two 3D points.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param a the first 3D point. Not modified.
    * @param b the second 3D point. Not modified.
    * @return the computed average.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static FramePoint3D averagePoint3Ds(FramePoint3DReadOnly a, FramePoint3DReadOnly b)
   {
      FramePoint3D average = new FramePoint3D(a);
      average.add(b);
      average.scale(0.5);

      return average;
   }

   /**
    * Computes the complete minimum rotation from {@code firstVector} to the {@code secondVector} and
    * packs it into an {@link Orientation3DBasics}.
    * <p>
    * The rotation angle is computed as the angle from the {@code firstVector} to the
    * {@code secondVector}: <br>
    * {@code rotationAngle = firstVector.angle(secondVector)}. </br>
    * Note: the vectors do not need to be unit length.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>the vectors are the same: the rotation angle is equal to {@code 0.0} and the rotation axis is
    * set to: (1, 0, 0).
    * <li>the vectors are parallel pointing opposite directions: the rotation angle is equal to
    * {@code Math.PI} and the rotation axis is set to: (1, 0, 0).
    * <li>if the length of either normal is below {@code 1.0E-7}: the rotation angle is equal to
    * {@code 0.0} and the rotation axis is set to: (1, 0, 0).
    * </ul>
    * </p>
    * <p>
    * Note: The calculation becomes less accurate as the two vectors are more parallel.
    * </p>
    *
    * @param firstVector    the first vector. Not modified.
    * @param secondVector   the second vector that is rotated with respect to the first vector. Not
    *                       modified.
    * @param rotationToPack the minimum rotation from {@code firstVector} to the {@code secondVector}.
    *                       Modified.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static void orientation3DFromFirstToSecondVector3D(FrameVector3DReadOnly firstVector,
                                                             FrameVector3DReadOnly secondVector,
                                                             Orientation3DBasics rotationToPack)
   {
      firstVector.checkReferenceFrameMatch(secondVector);
      EuclidGeometryTools.orientation3DFromFirstToSecondVector3D(firstVector, secondVector, rotationToPack);
   }

   /**
    * Computes the complete minimum rotation from {@code firstVector} to the {@code secondVector} and
    * packs it into an {@link FixedFrameOrientation3DBasics}.
    * <p>
    * The rotation angle is computed as the angle from the {@code firstVector} to the
    * {@code secondVector}: <br>
    * {@code rotationAngle = firstVector.angle(secondVector)}. </br>
    * Note: the vectors do not need to be unit length.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>the vectors are the same: the rotation angle is equal to {@code 0.0} and the rotation axis is
    * set to: (1, 0, 0).
    * <li>the vectors are parallel pointing opposite directions: the rotation angle is equal to
    * {@code Math.PI} and the rotation axis is set to: (1, 0, 0).
    * <li>if the length of either normal is below {@code 1.0E-7}: the rotation angle is equal to
    * {@code 0.0} and the rotation axis is set to: (1, 0, 0).
    * </ul>
    * </p>
    * <p>
    * Note: The calculation becomes less accurate as the two vectors are more parallel.
    * </p>
    *
    * @param firstVector    the first vector. Not modified.
    * @param secondVector   the second vector that is rotated with respect to the first vector. Not
    *                       modified.
    * @param rotationToPack the minimum rotation from {@code firstVector} to the {@code secondVector}.
    *                       Modified.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static void orientation3DFromFirstToSecondVector3D(FrameVector3DReadOnly firstVector,
                                                             FrameVector3DReadOnly secondVector,
                                                             FixedFrameOrientation3DBasics rotationToPack)
   {
      firstVector.checkReferenceFrameMatch(secondVector, rotationToPack);
      EuclidGeometryTools.orientation3DFromFirstToSecondVector3D(firstVector, secondVector, rotationToPack);
   }

   /**
    * Computes the complete minimum rotation from {@code firstVector} to the {@code secondVector} and
    * packs it into an {@link FrameOrientation3DBasics}.
    * <p>
    * The rotation angle is computed as the angle from the {@code firstVector} to the
    * {@code secondVector}: <br>
    * {@code rotationAngle = firstVector.angle(secondVector)}. </br>
    * Note: the vectors do not need to be unit length.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>the vectors are the same: the rotation angle is equal to {@code 0.0} and the rotation axis is
    * set to: (1, 0, 0).
    * <li>the vectors are parallel pointing opposite directions: the rotation angle is equal to
    * {@code Math.PI} and the rotation axis is set to: (1, 0, 0).
    * <li>if the length of either normal is below {@code 1.0E-7}: the rotation angle is equal to
    * {@code 0.0} and the rotation axis is set to: (1, 0, 0).
    * </ul>
    * </p>
    * <p>
    * Note: The calculation becomes less accurate as the two vectors are more parallel.
    * </p>
    *
    * @param firstVector    the first vector. Not modified.
    * @param secondVector   the second vector that is rotated with respect to the first vector. Not
    *                       modified.
    * @param rotationToPack the minimum rotation from {@code firstVector} to the {@code secondVector}.
    *                       Modified.
    * @throws ReferenceFrameMismatchException if the read-only arguments are not all expressed in the
    *                                         same reference frame.
    */
   public static void orientation3DFromFirstToSecondVector3D(FrameVector3DReadOnly firstVector,
                                                             FrameVector3DReadOnly secondVector,
                                                             FrameOrientation3DBasics rotationToPack)
   {
      firstVector.checkReferenceFrameMatch(secondVector);
      rotationToPack.setReferenceFrame(firstVector.getReferenceFrame());
      EuclidGeometryTools.orientation3DFromFirstToSecondVector3D(firstVector, secondVector, rotationToPack);
   }

   /**
    * Computes the complete minimum rotation from {@code firstVector} to the {@code secondVector} and
    * returns the result as an {@link AxisAngle}.
    * <p>
    * The rotation angle is computed as the angle from the {@code firstVector} to the
    * {@code secondVector}: <br>
    * {@code rotationAngle = firstVector.angle(secondVector)}. </br>
    * Note: the vectors do not need to be unit length.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>the vectors are the same: the rotation angle is equal to {@code 0.0} and the rotation axis is
    * set to: (1, 0, 0).
    * <li>the vectors are parallel pointing opposite directions: the rotation angle is equal to
    * {@code Math.PI} and the rotation axis is set to: (1, 0, 0).
    * <li>if the length of either normal is below {@code 1.0E-7}: the rotation angle is equal to
    * {@code 0.0} and the rotation axis is set to: (1, 0, 0).
    * </ul>
    * </p>
    * <p>
    * Note: The calculation becomes less accurate as the two vectors are more parallel.
    * </p>
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param firstVector  the first vector. Not modified.
    * @param secondVector the second vector that is rotated with respect to the first vector. Not
    *                     modified.
    * @return the minimum rotation from {@code zUp} to the given {@code vector}.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static AxisAngle axisAngleFromFirstToSecondVector3D(FrameVector3DReadOnly firstVector, FrameVector3DReadOnly secondVector)
   {
      firstVector.checkReferenceFrameMatch(secondVector);
      return EuclidGeometryTools.axisAngleFromFirstToSecondVector3D(firstVector, secondVector);
   }

   /**
    * Computes the complete minimum rotation from {@code zUp = (0, 0, 1)} to the given {@code vector}
    * and packs it into an {@link FixedFrameOrientation3DBasics}.
    * <p>
    * The rotation angle is computed as the angle from the {@code zUp} to the {@code vector}: <br>
    * {@code rotationAngle = zUp.angle(vector)}. </br>
    * Note: the vector does not need to be unit length.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>the vector is aligned with {@code zUp}: the rotation angle is equal to {@code 0.0} and the
    * rotation axis is set to: (1, 0, 0).
    * <li>the vector is parallel pointing opposite direction of {@code zUp}: the rotation angle is
    * equal to {@code Math.PI} and the rotation axis is set to: (1, 0, 0).
    * <li>if the length of the given normal is below {@code 1.0E-7}: the rotation angle is equal to
    * {@code 0.0} and the rotation axis is set to: (1, 0, 0).
    * </ul>
    * </p>
    * <p>
    * Note: The calculation becomes less accurate as the two vectors are more parallel.
    * </p>
    *
    * @param vector         the vector that is rotated with respect to {@code zUp}. Not modified.
    * @param rotationToPack the minimum rotation from {@code zUp} to the given {@code vector}.
    *                       Modified.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static void orientation3DFromZUpToVector3D(FrameVector3DReadOnly vector, FixedFrameOrientation3DBasics rotationToPack)
   {
      rotationToPack.checkReferenceFrameMatch(vector.getReferenceFrame());
      EuclidGeometryTools.orientation3DFromFirstToSecondVector3D(Axis3D.Z, vector, rotationToPack);
   }

   /**
    * Computes the complete minimum rotation from {@code zUp = (0, 0, 1)} to the given {@code vector}
    * and packs it into an {@link FrameOrientation3DBasics}.
    * <p>
    * The rotation angle is computed as the angle from the {@code zUp} to the {@code vector}: <br>
    * {@code rotationAngle = zUp.angle(vector)}. </br>
    * Note: the vector does not need to be unit length.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>the vector is aligned with {@code zUp}: the rotation angle is equal to {@code 0.0} and the
    * rotation axis is set to: (1, 0, 0).
    * <li>the vector is parallel pointing opposite direction of {@code zUp}: the rotation angle is
    * equal to {@code Math.PI} and the rotation axis is set to: (1, 0, 0).
    * <li>if the length of the given normal is below {@code 1.0E-7}: the rotation angle is equal to
    * {@code 0.0} and the rotation axis is set to: (1, 0, 0).
    * </ul>
    * </p>
    * <p>
    * Note: The calculation becomes less accurate as the two vectors are more parallel.
    * </p>
    *
    * @param vector         the vector that is rotated with respect to {@code zUp}. Not modified.
    * @param rotationToPack the minimum rotation from {@code zUp} to the given {@code vector}.
    *                       Modified.
    */
   public static void orientation3DFromZUpToVector3D(FrameVector3DReadOnly vector, FrameOrientation3DBasics rotationToPack)
   {
      rotationToPack.setReferenceFrame(vector.getReferenceFrame());
      EuclidGeometryTools.orientation3DFromFirstToSecondVector3D(Axis3D.Z, vector, rotationToPack);
   }

   /**
    * Given two 3D infinitely long lines, this methods computes two points P &in; line1 and Q &in; lin2
    * such that the distance || P - Q || is the minimum distance between the two 3D lines.
    * <a href="http://geomalgorithms.com/a07-_distance.html"> Useful link</a>.
    *
    * @param pointOnLine1              a 3D point on the first line. Not modified.
    * @param lineDirection1            the 3D direction of the first line. Not modified.
    * @param pointOnLine2              a 3D point on the second line. Not modified.
    * @param lineDirection2            the 3D direction of the second line. Not modified.
    * @param closestPointOnLine1ToPack the 3D coordinates of the point P are packed in this 3D point.
    *                                  Modified. Can be {@code null}.
    * @param closestPointOnLine2ToPack the 3D coordinates of the point Q are packed in this 3D point.
    *                                  Modified. Can be {@code null}.
    * @return the minimum distance between the two lines.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static double closestPoint3DsBetweenTwoLine3Ds(FramePoint3DReadOnly pointOnLine1,
                                                         FrameVector3DReadOnly lineDirection1,
                                                         FramePoint3DReadOnly pointOnLine2,
                                                         FrameVector3DReadOnly lineDirection2,
                                                         FixedFramePoint3DBasics closestPointOnLine1ToPack,
                                                         FixedFramePoint3DBasics closestPointOnLine2ToPack)
   {
      pointOnLine1.checkReferenceFrameMatch(lineDirection1, pointOnLine2, lineDirection2);
      if (closestPointOnLine1ToPack != null)
         closestPointOnLine1ToPack.checkReferenceFrameMatch(pointOnLine1);
      if (closestPointOnLine2ToPack != null)
         closestPointOnLine2ToPack.checkReferenceFrameMatch(pointOnLine1);
      return EuclidGeometryTools.closestPoint3DsBetweenTwoLine3Ds(pointOnLine1,
                                                                  lineDirection1,
                                                                  pointOnLine2,
                                                                  lineDirection2,
                                                                  closestPointOnLine1ToPack,
                                                                  closestPointOnLine2ToPack);
   }

   /**
    * Given two 3D infinitely long lines, this methods computes two points P &in; line1 and Q &in; lin2
    * such that the distance || P - Q || is the minimum distance between the two 3D lines.
    * <a href="http://geomalgorithms.com/a07-_distance.html"> Useful link</a>.
    *
    * @param pointOnLine1              a 3D point on the first line. Not modified.
    * @param lineDirection1            the 3D direction of the first line. Not modified.
    * @param pointOnLine2              a 3D point on the second line. Not modified.
    * @param lineDirection2            the 3D direction of the second line. Not modified.
    * @param closestPointOnLine1ToPack the 3D coordinates of the point P are packed in this 3D point.
    *                                  Modified. Can be {@code null}.
    * @param closestPointOnLine2ToPack the 3D coordinates of the point Q are packed in this 3D point.
    *                                  Modified. Can be {@code null}.
    * @return the minimum distance between the two lines.
    * @throws ReferenceFrameMismatchException if the read-only arguments are not all expressed in the
    *                                         same reference frame.
    */
   public static double closestPoint3DsBetweenTwoLine3Ds(FramePoint3DReadOnly pointOnLine1,
                                                         FrameVector3DReadOnly lineDirection1,
                                                         FramePoint3DReadOnly pointOnLine2,
                                                         FrameVector3DReadOnly lineDirection2,
                                                         FramePoint3DBasics closestPointOnLine1ToPack,
                                                         FramePoint3DBasics closestPointOnLine2ToPack)
   {
      pointOnLine1.checkReferenceFrameMatch(lineDirection1, pointOnLine2, lineDirection2);
      if (closestPointOnLine1ToPack != null)
         closestPointOnLine1ToPack.setReferenceFrame(pointOnLine1.getReferenceFrame());
      if (closestPointOnLine2ToPack != null)
         closestPointOnLine2ToPack.setReferenceFrame(pointOnLine1.getReferenceFrame());
      return EuclidGeometryTools.closestPoint3DsBetweenTwoLine3Ds(pointOnLine1,
                                                                  lineDirection1,
                                                                  pointOnLine2,
                                                                  lineDirection2,
                                                                  closestPointOnLine1ToPack,
                                                                  closestPointOnLine2ToPack);
   }

   /**
    * Given two 2D line segments with finite length, this methods computes two points P &in;
    * lineSegment1 and Q &in; lineSegment2 such that the distance || P - Q || is the minimum distance
    * between the two 2D line segments. <a href="http://geomalgorithms.com/a07-_distance.html"> Useful
    * link</a>.
    *
    * @param lineSegmentStart1                the first endpoint of the first line segment. Not
    *                                         modified.
    * @param lineSegmentEnd1                  the second endpoint of the first line segment. Not
    *                                         modified.
    * @param lineSegmentStart2                the first endpoint of the second line segment. Not
    *                                         modified.
    * @param lineSegmentEnd2                  the second endpoint of the second line segment. Not
    *                                         modified.
    * @param closestPointOnLineSegment1ToPack the 2D coordinates of the point P are packed in this 2D
    *                                         point. Modified. Can be {@code null}.
    * @param closestPointOnLineSegment2ToPack the 2D coordinates of the point Q are packed in this 2D
    *                                         point. Modified. Can be {@code null}.
    * @return the minimum distance between the two line segments.
    * @throws ReferenceFrameMismatchException if the read-only arguments are not all expressed in the
    *                                         same reference frame.
    */
   public static double closestPoint2DsBetweenTwoLineSegment2Ds(FramePoint2DReadOnly lineSegmentStart1,
                                                                FramePoint2DReadOnly lineSegmentEnd1,
                                                                FramePoint2DReadOnly lineSegmentStart2,
                                                                FramePoint2DReadOnly lineSegmentEnd2,
                                                                FixedFramePoint2DBasics closestPointOnLineSegment1ToPack,
                                                                FixedFramePoint2DBasics closestPointOnLineSegment2ToPack)
   {
      lineSegmentStart1.checkReferenceFrameMatch(lineSegmentEnd1, lineSegmentStart2, lineSegmentEnd2);
      if (closestPointOnLineSegment1ToPack != null)
         closestPointOnLineSegment1ToPack.checkReferenceFrameMatch(lineSegmentStart1);
      if (closestPointOnLineSegment2ToPack != null)
         closestPointOnLineSegment2ToPack.checkReferenceFrameMatch(lineSegmentStart1);
      return EuclidGeometryTools.closestPoint2DsBetweenTwoLineSegment2Ds(lineSegmentStart1,
                                                                         lineSegmentEnd1,
                                                                         lineSegmentStart2,
                                                                         lineSegmentEnd2,
                                                                         closestPointOnLineSegment1ToPack,
                                                                         closestPointOnLineSegment2ToPack);
   }

   /**
    * Given two 2D line segments with finite length, this methods computes two points P &in;
    * lineSegment1 and Q &in; lineSegment2 such that the distance || P - Q || is the minimum distance
    * between the two 2D line segments. <a href="http://geomalgorithms.com/a07-_distance.html"> Useful
    * link</a>.
    *
    * @param lineSegmentStart1                the first endpoint of the first line segment. Not
    *                                         modified.
    * @param lineSegmentEnd1                  the second endpoint of the first line segment. Not
    *                                         modified.
    * @param lineSegmentStart2                the first endpoint of the second line segment. Not
    *                                         modified.
    * @param lineSegmentEnd2                  the second endpoint of the second line segment. Not
    *                                         modified.
    * @param closestPointOnLineSegment1ToPack the 2D coordinates of the point P are packed in this 2D
    *                                         point. Modified. Can be {@code null}.
    * @param closestPointOnLineSegment2ToPack the 2D coordinates of the point Q are packed in this 2D
    *                                         point. Modified. Can be {@code null}.
    * @return the minimum distance between the two line segments.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static double closestPoint2DsBetweenTwoLineSegment2Ds(FramePoint2DReadOnly lineSegmentStart1,
                                                                FramePoint2DReadOnly lineSegmentEnd1,
                                                                FramePoint2DReadOnly lineSegmentStart2,
                                                                FramePoint2DReadOnly lineSegmentEnd2,
                                                                FramePoint2DBasics closestPointOnLineSegment1ToPack,
                                                                FramePoint2DBasics closestPointOnLineSegment2ToPack)
   {
      lineSegmentStart1.checkReferenceFrameMatch(lineSegmentEnd1, lineSegmentStart2, lineSegmentEnd2);
      if (closestPointOnLineSegment1ToPack != null)
         closestPointOnLineSegment1ToPack.setReferenceFrame(lineSegmentStart1.getReferenceFrame());
      if (closestPointOnLineSegment2ToPack != null)
         closestPointOnLineSegment2ToPack.setReferenceFrame(lineSegmentStart1.getReferenceFrame());
      return EuclidGeometryTools.closestPoint2DsBetweenTwoLineSegment2Ds(lineSegmentStart1,
                                                                         lineSegmentEnd1,
                                                                         lineSegmentStart2,
                                                                         lineSegmentEnd2,
                                                                         closestPointOnLineSegment1ToPack,
                                                                         closestPointOnLineSegment2ToPack);
   }

   /**
    * Given two 3D line segments with finite length, this methods computes two points P &in;
    * lineSegment1 and Q &in; lineSegment2 such that the distance || P - Q || is the minimum distance
    * between the two 3D line segments. <a href="http://geomalgorithms.com/a07-_distance.html"> Useful
    * link</a>.
    *
    * @param lineSegmentStart1                the first endpoint of the first line segment. Not
    *                                         modified.
    * @param lineSegmentEnd1                  the second endpoint of the first line segment. Not
    *                                         modified.
    * @param lineSegmentStart2                the first endpoint of the second line segment. Not
    *                                         modified.
    * @param lineSegmentEnd2                  the second endpoint of the second line segment. Not
    *                                         modified.
    * @param closestPointOnLineSegment1ToPack the 3D coordinates of the point P are packed in this 3D
    *                                         point. Modified. Can be {@code null}.
    * @param closestPointOnLineSegment2ToPack the 3D coordinates of the point Q are packed in this 3D
    *                                         point. Modified. Can be {@code null}.
    * @return the minimum distance between the two line segments.
    * @throws ReferenceFrameMismatchException if the read-only arguments are not all expressed in the
    *                                         same reference frame.
    */
   public static double closestPoint3DsBetweenTwoLineSegment3Ds(FramePoint3DReadOnly lineSegmentStart1,
                                                                FramePoint3DReadOnly lineSegmentEnd1,
                                                                FramePoint3DReadOnly lineSegmentStart2,
                                                                FramePoint3DReadOnly lineSegmentEnd2,
                                                                FixedFramePoint3DBasics closestPointOnLineSegment1ToPack,
                                                                FixedFramePoint3DBasics closestPointOnLineSegment2ToPack)
   {
      lineSegmentStart1.checkReferenceFrameMatch(lineSegmentEnd1, lineSegmentStart2, lineSegmentEnd2);
      if (closestPointOnLineSegment1ToPack != null)
         closestPointOnLineSegment1ToPack.checkReferenceFrameMatch(lineSegmentStart1);
      if (closestPointOnLineSegment2ToPack != null)
         closestPointOnLineSegment2ToPack.checkReferenceFrameMatch(lineSegmentStart1);
      return EuclidGeometryTools.closestPoint3DsBetweenTwoLineSegment3Ds(lineSegmentStart1,
                                                                         lineSegmentEnd1,
                                                                         lineSegmentStart2,
                                                                         lineSegmentEnd2,
                                                                         closestPointOnLineSegment1ToPack,
                                                                         closestPointOnLineSegment2ToPack);
   }

   /**
    * Given two 3D line segments with finite length, this methods computes two points P &in;
    * lineSegment1 and Q &in; lineSegment2 such that the distance || P - Q || is the minimum distance
    * between the two 3D line segments. <a href="http://geomalgorithms.com/a07-_distance.html"> Useful
    * link</a>.
    *
    * @param lineSegmentStart1                the first endpoint of the first line segment. Not
    *                                         modified.
    * @param lineSegmentEnd1                  the second endpoint of the first line segment. Not
    *                                         modified.
    * @param lineSegmentStart2                the first endpoint of the second line segment. Not
    *                                         modified.
    * @param lineSegmentEnd2                  the second endpoint of the second line segment. Not
    *                                         modified.
    * @param closestPointOnLineSegment1ToPack the 3D coordinates of the point P are packed in this 3D
    *                                         point. Modified. Can be {@code null}.
    * @param closestPointOnLineSegment2ToPack the 3D coordinates of the point Q are packed in this 3D
    *                                         point. Modified. Can be {@code null}.
    * @return the minimum distance between the two line segments.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static double closestPoint3DsBetweenTwoLineSegment3Ds(FramePoint3DReadOnly lineSegmentStart1,
                                                                FramePoint3DReadOnly lineSegmentEnd1,
                                                                FramePoint3DReadOnly lineSegmentStart2,
                                                                FramePoint3DReadOnly lineSegmentEnd2,
                                                                FramePoint3DBasics closestPointOnLineSegment1ToPack,
                                                                FramePoint3DBasics closestPointOnLineSegment2ToPack)
   {
      lineSegmentStart1.checkReferenceFrameMatch(lineSegmentEnd1, lineSegmentStart2, lineSegmentEnd2);
      if (closestPointOnLineSegment1ToPack != null)
         closestPointOnLineSegment1ToPack.setReferenceFrame(lineSegmentStart1.getReferenceFrame());
      if (closestPointOnLineSegment2ToPack != null)
         closestPointOnLineSegment2ToPack.setReferenceFrame(lineSegmentStart1.getReferenceFrame());
      return EuclidGeometryTools.closestPoint3DsBetweenTwoLineSegment3Ds(lineSegmentStart1,
                                                                         lineSegmentEnd1,
                                                                         lineSegmentStart2,
                                                                         lineSegmentEnd2,
                                                                         closestPointOnLineSegment1ToPack,
                                                                         closestPointOnLineSegment2ToPack);
   }

   /**
    * Compute the area of a triangle defined by its three vertices: a, b, and c. No specific ordering
    * of the vertices is required.
    *
    * @param a first vertex of the triangle. Not modified.
    * @param b second vertex of the triangle. Not modified.
    * @param c third vertex of the triangle. Not modified.
    * @return the are of the triangle.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static double triangleArea(FramePoint2DReadOnly a, FramePoint2DReadOnly b, FramePoint2DReadOnly c)
   {
      a.checkReferenceFrameMatch(b, c);
      return EuclidGeometryTools.triangleArea(a, b, c);
   }

   /**
    * Computes the area of a triangle defined by its three vertices: a, b, and c. No specific ordering
    * of the vertices is required.
    * <p>
    * This method uses {@link EuclidGeometryTools#triangleAreaHeron2(double, double, double)}.
    * </p>
    *
    * @param a first vertex of the triangle. Not modified.
    * @param b second vertex of the triangle. Not modified.
    * @param c third vertex of the triangle. Not modified.
    * @return the area of the triangle.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static double triangleArea(FramePoint3DReadOnly a, FramePoint3DReadOnly b, FramePoint3DReadOnly c)
   {
      a.checkReferenceFrameMatch(b, c);
      return EuclidGeometryTools.triangleArea(a, b, c);
   }

   /**
    * Computes the coordinates of the center of the circumscribed circle of the triangle ABC.
    * <p>
    * Edge-case, if the problem is degenerate, i.e. the three points are on a line or all equal, this
    * method fails and returns {@code false}.
    * </p>
    * <p>
    * Algorithm from
    * <a href="https://en.wikipedia.org/wiki/Circumscribed_circle#Cartesian_coordinates">Wikipedia
    * article</a>.
    * </p>
    *
    * @param A                  the position of the first vertex of the triangle. Not modified.
    * @param B                  the position of the second vertex of the triangle. Not modified.
    * @param C                  the position of the third vertex of the triangle. Not modified.
    * @param circumcenterToPack the coordinates of the circumscribed circle's center.
    * @return {@code true} if the calculation was successful, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static boolean triangleCircumcenter(FramePoint2DReadOnly A,
                                              FramePoint2DReadOnly B,
                                              FramePoint2DReadOnly C,
                                              FixedFramePoint2DBasics circumcenterToPack)
   {
      circumcenterToPack.checkReferenceFrameMatch(A, B, C);
      return EuclidGeometryTools.triangleCircumcenter(A, B, C, circumcenterToPack);
   }

   /**
    * Computes the coordinates of the center of the circumscribed circle of the triangle ABC.
    * <p>
    * Edge-case, if the problem is degenerate, i.e. the three points are on a line or all equal, this
    * method fails and returns {@code false}.
    * </p>
    * <p>
    * Algorithm from
    * <a href="https://en.wikipedia.org/wiki/Circumscribed_circle#Cartesian_coordinates">Wikipedia
    * article</a>.
    * </p>
    *
    * @param A                  the position of the first vertex of the triangle. Not modified.
    * @param B                  the position of the second vertex of the triangle. Not modified.
    * @param C                  the position of the third vertex of the triangle. Not modified.
    * @param circumcenterToPack the coordinates of the circumscribed circle's center.
    * @return {@code true} if the calculation was successful, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if {@code A}, {@code B}, and {@code C} are not all
    *                                         expressed in the same reference frame.
    */
   public static boolean triangleCircumcenter(FramePoint2DReadOnly A, FramePoint2DReadOnly B, FramePoint2DReadOnly C, FramePoint2DBasics circumcenterToPack)
   {
      A.checkReferenceFrameMatch(B, C);
      circumcenterToPack.setReferenceFrame(A.getReferenceFrame());
      return EuclidGeometryTools.triangleCircumcenter(A, B, C, circumcenterToPack);
   }

   /**
    * Computes the coordinates of the center of the circumscribed circle of the triangle ABC.
    * <p>
    * Edge-case, if the problem is degenerate, i.e. the three points are on a line or all equal, this
    * method fails and returns {@code false}.
    * </p>
    * <p>
    * Algorithm from <a href=
    * "https://en.wikipedia.org/wiki/Circumscribed_circle#Cartesian_coordinates_from_cross-_and_dot-products">Wikipedia
    * article</a>.
    * </p>
    *
    * @param A                  the position of the first vertex of the triangle. Not modified.
    * @param B                  the position of the second vertex of the triangle. Not modified.
    * @param C                  the position of the third vertex of the triangle. Not modified.
    * @param circumcenterToPack the coordinates of the circumscribed circle's center.
    * @return {@code true} if the calculation was successful, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static boolean triangleCircumcenter(FramePoint3DReadOnly A,
                                              FramePoint3DReadOnly B,
                                              FramePoint3DReadOnly C,
                                              FixedFramePoint3DBasics circumcenterToPack)
   {
      circumcenterToPack.checkReferenceFrameMatch(A, B, C);
      return EuclidGeometryTools.triangleCircumcenter(A, B, C, circumcenterToPack);
   }

   /**
    * Computes the coordinates of the center of the circumscribed circle of the triangle ABC.
    * <p>
    * Edge-case, if the problem is degenerate, i.e. the three points are on a line or all equal, this
    * method fails and returns {@code false}.
    * </p>
    * <p>
    * Algorithm from <a href=
    * "https://en.wikipedia.org/wiki/Circumscribed_circle#Cartesian_coordinates_from_cross-_and_dot-products">Wikipedia
    * article</a>.
    * </p>
    *
    * @param A                  the position of the first vertex of the triangle. Not modified.
    * @param B                  the position of the second vertex of the triangle. Not modified.
    * @param C                  the position of the third vertex of the triangle. Not modified.
    * @param circumcenterToPack the coordinates of the circumscribed circle's center.
    * @return {@code true} if the calculation was successful, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if {@code A}, {@code B}, and {@code C} are not all
    *                                         expressed in the same reference frame.
    */
   public static boolean triangleCircumcenter(FramePoint3DReadOnly A, FramePoint3DReadOnly B, FramePoint3DReadOnly C, FramePoint3DBasics circumcenterToPack)
   {
      A.checkReferenceFrameMatch(B, C);
      circumcenterToPack.setReferenceFrame(A.getReferenceFrame());
      return EuclidGeometryTools.triangleCircumcenter(A, B, C, circumcenterToPack);
   }

   /**
    * This methods computes the minimum distance between the two infinitely long 3D lines.
    * <a href="http://geomalgorithms.com/a07-_distance.html"> Useful link</a>.
    *
    * @param pointOnLine1   a 3D point on the first line. Not modified.
    * @param lineDirection1 the 3D direction of the first line. Not modified.
    * @param pointOnLine2   a 3D point on the second line. Not modified.
    * @param lineDirection2 the 3D direction of the second line. Not modified.
    * @return the minimum distance between the two lines.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static double distanceBetweenTwoLine3Ds(FramePoint3DReadOnly pointOnLine1,
                                                  FrameVector3DReadOnly lineDirection1,
                                                  FramePoint3DReadOnly pointOnLine2,
                                                  FrameVector3DReadOnly lineDirection2)
   {
      pointOnLine1.checkReferenceFrameMatch(lineDirection1, pointOnLine2, lineDirection2);
      return EuclidGeometryTools.closestPoint3DsBetweenTwoLine3Ds(pointOnLine1, lineDirection1, pointOnLine2, lineDirection2, null, null);
   }

   /**
    * This methods computes the minimum distance between the two 2D line segments with finite length.
    * <a href="http://geomalgorithms.com/a07-_distance.html"> Useful link</a>.
    *
    * @param lineSegmentStart1 the first endpoint of the first line segment. Not modified.
    * @param lineSegmentEnd1   the second endpoint of the first line segment. Not modified.
    * @param lineSegmentStart2 the first endpoint of the second line segment. Not modified.
    * @param lineSegmentEnd2   the second endpoint of the second line segment. Not modified.
    * @return the minimum distance between the two line segments.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static double distanceBetweenTwoLineSegment2Ds(FramePoint2DReadOnly lineSegmentStart1,
                                                         FramePoint2DReadOnly lineSegmentEnd1,
                                                         FramePoint2DReadOnly lineSegmentStart2,
                                                         FramePoint2DReadOnly lineSegmentEnd2)
   {
      lineSegmentStart1.checkReferenceFrameMatch(lineSegmentEnd1, lineSegmentStart2, lineSegmentEnd2);
      return EuclidGeometryTools.closestPoint2DsBetweenTwoLineSegment2Ds(lineSegmentStart1, lineSegmentEnd1, lineSegmentStart2, lineSegmentEnd2, null, null);
   }

   /**
    * This methods computes the minimum distance between the two 3D line segments with finite length.
    * <a href="http://geomalgorithms.com/a07-_distance.html"> Useful link</a>.
    *
    * @param lineSegmentStart1 the first endpoint of the first line segment. Not modified.
    * @param lineSegmentEnd1   the second endpoint of the first line segment. Not modified.
    * @param lineSegmentStart2 the first endpoint of the second line segment. Not modified.
    * @param lineSegmentEnd2   the second endpoint of the second line segment. Not modified.
    * @return the minimum distance between the two line segments.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static double distanceBetweenTwoLineSegment3Ds(FramePoint3DReadOnly lineSegmentStart1,
                                                         FramePoint3DReadOnly lineSegmentEnd1,
                                                         FramePoint3DReadOnly lineSegmentStart2,
                                                         FramePoint3DReadOnly lineSegmentEnd2)
   {
      lineSegmentStart1.checkReferenceFrameMatch(lineSegmentEnd1, lineSegmentStart2, lineSegmentEnd2);
      return EuclidGeometryTools.closestPoint3DsBetweenTwoLineSegment3Ds(lineSegmentStart1, lineSegmentEnd1, lineSegmentStart2, lineSegmentEnd2, null, null);
   }

   /**
    * Returns the minimum distance between a 2D point and an infinitely long 2D line defined by two
    * points.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if
    * {@code firstPointOnLine.distance(secondPointOnLine) < }{@link EuclidGeometryTools#ONE_TRILLIONTH},
    * this method returns the distance between {@code firstPointOnLine} and the given {@code point}.
    * </ul>
    * </p>
    *
    * @param pointX            x-coordinate of the query.
    * @param pointY            y-coordinate of the query.
    * @param firstPointOnLine  a first point located on the line. Not modified.
    * @param secondPointOnLine a second point located on the line. Not modified.
    * @return the minimum distance between the 2D point and the 2D line.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static double distanceFromPoint2DToLine2D(double pointX, double pointY, FramePoint2DReadOnly firstPointOnLine, FramePoint2DReadOnly secondPointOnLine)
   {
      firstPointOnLine.checkReferenceFrameMatch(secondPointOnLine);
      return EuclidGeometryTools.distanceFromPoint2DToLine2D(pointX, pointY, firstPointOnLine, secondPointOnLine);
   }

   /**
    * Returns the minimum distance between a 2D point and an infinitely long 2D line defined by a point
    * and a direction.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if {@code lineDirection.length() < }{@link EuclidGeometryTools#ONE_TRILLIONTH}, this method
    * returns the distance between {@code pointOnLine} and the given {@code point}.
    * </ul>
    * </p>
    *
    * @param pointX        x-coordinate of the query.
    * @param pointY        y-coordinate of the query.
    * @param pointOnLine   a point located on the line. Not modified.
    * @param lineDirection the direction of the line. Not modified.
    * @return the minimum distance between the 2D point and the 2D line.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static double distanceFromPoint2DToLine2D(double pointX, double pointY, FramePoint2DReadOnly pointOnLine, FrameVector2DReadOnly lineDirection)
   {
      pointOnLine.checkReferenceFrameMatch(lineDirection);
      return EuclidGeometryTools.distanceFromPoint2DToLine2D(pointX, pointY, pointOnLine, lineDirection);
   }

   /**
    * Returns the minimum distance between a 2D point and an infinitely long 2D line defined by two
    * points.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if
    * {@code firstPointOnLine.distance(secondPointOnLine) < }{@link EuclidGeometryTools#ONE_TRILLIONTH},
    * this method returns the distance between {@code firstPointOnLine} and the given {@code point}.
    * </ul>
    * </p>
    *
    * @param point             2D point to compute the distance from the line. Not modified.
    * @param firstPointOnLine  a first point located on the line. Not modified.
    * @param secondPointOnLine a second point located on the line. Not modified.
    * @return the minimum distance between the 2D point and the 2D line.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static double distanceFromPoint2DToLine2D(FramePoint2DReadOnly point, FramePoint2DReadOnly firstPointOnLine, FramePoint2DReadOnly secondPointOnLine)
   {
      point.checkReferenceFrameMatch(firstPointOnLine, secondPointOnLine);
      return EuclidGeometryTools.distanceFromPoint2DToLine2D(point, firstPointOnLine, secondPointOnLine);
   }

   /**
    * Returns the minimum distance between a 2D point and an infinitely long 2D line defined by a point
    * and a direction.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if {@code lineDirection.length() < }{@link EuclidGeometryTools#ONE_TRILLIONTH}, this method
    * returns the distance between {@code pointOnLine} and the given {@code point}.
    * </ul>
    * </p>
    *
    * @param point         2D point to compute the distance from the line. Not modified.
    * @param pointOnLine   a point located on the line. Not modified.
    * @param lineDirection the direction of the line. Not modified.
    * @return the minimum distance between the 2D point and the 2D line.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static double distanceFromPoint2DToLine2D(FramePoint2DReadOnly point, FramePoint2DReadOnly pointOnLine, FrameVector2DReadOnly lineDirection)
   {
      point.checkReferenceFrameMatch(pointOnLine, lineDirection);
      return EuclidGeometryTools.distanceFromPoint2DToLine2D(point, pointOnLine, lineDirection);
   }

   /**
    * Returns the minimum distance between a point and a given line segment.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if
    * {@code lineSegmentStart.distanceSquared(lineSegmentEnd) < }{@link EuclidGeometryTools#ONE_TRILLIONTH},
    * this method returns the distance between {@code lineSegmentStart} and the given {@code point}.
    * </ul>
    * </p>
    *
    * @param pointX           x coordinate of point to be tested.
    * @param pointY           y coordinate of point to be tested.
    * @param lineSegmentStart first endpoint of the line segment. Not modified.
    * @param lineSegmentEnd   second endpoint of the line segment. Not modified.
    * @return the minimum distance between the 2D point and the 2D line segment.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static double distanceFromPoint2DToLineSegment2D(double pointX,
                                                           double pointY,
                                                           FramePoint2DReadOnly lineSegmentStart,
                                                           FramePoint2DReadOnly lineSegmentEnd)
   {
      lineSegmentStart.checkReferenceFrameMatch(lineSegmentEnd);
      return EuclidGeometryTools.distanceFromPoint2DToLineSegment2D(pointX, pointY, lineSegmentStart, lineSegmentEnd);
   }

   /**
    * Returns the minimum distance between a point and a given line segment.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if
    * {@code lineSegmentStart.distanceSquared(lineSegmentEnd) < }{@link EuclidGeometryTools#ONE_TRILLIONTH},
    * this method returns the distance between {@code lineSegmentStart} and the given {@code point}.
    * </ul>
    * </p>
    *
    * @param point            2D point to compute the distance from the line segment. Not modified.
    * @param lineSegmentStart first endpoint of the line segment. Not modified.
    * @param lineSegmentEnd   second endpoint of the line segment. Not modified.
    * @return the minimum distance between the 2D point and the 2D line segment.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static double distanceFromPoint2DToLineSegment2D(FramePoint2DReadOnly point,
                                                           FramePoint2DReadOnly lineSegmentStart,
                                                           FramePoint2DReadOnly lineSegmentEnd)
   {
      point.checkReferenceFrameMatch(lineSegmentStart, lineSegmentEnd);
      return EuclidGeometryTools.distanceFromPoint2DToLineSegment2D(point, lineSegmentStart, lineSegmentEnd);
   }

   /**
    * Returns the minimum distance between a 2D point and a 2D ray defined by its origin and a
    * direction.
    * <p>
    * When the query is located in front of the ray, this is equivalent to calculating the distance
    * from the query to the line that is collinear with the ray. When the query is located behind the
    * ray's origin, this is equivalent to calculating the distance between the query and the origin of
    * the ray.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>if {@code rayDirection.length() < }{@link EuclidGeometryTools#ONE_TRILLIONTH}, this method
    * returns the distance between {@code rayOrigin} and the given {@code point}.
    * </ul>
    * </p>
    *
    * @param pointX       x-coordinate of the query.
    * @param pointY       y-coordinate of the query.
    * @param rayOrigin    a point located on the line. Not modified.
    * @param rayDirection the direction of the line. Not modified.
    * @return the minimum distance between the 2D point and the 2D line.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static double distanceFromPoint2DToRay2D(double pointX, double pointY, FramePoint2DReadOnly rayOrigin, FrameVector2DReadOnly rayDirection)
   {
      rayOrigin.checkReferenceFrameMatch(rayDirection);
      return EuclidGeometryTools.distanceFromPoint2DToRay2D(pointX, pointY, rayOrigin, rayDirection);
   }

   /**
    * Returns the minimum distance between a 2D point and a 2D ray defined by its origin and a
    * direction.
    * <p>
    * When the query is located in front of the ray, this is equivalent to calculating the distance
    * from the query to the line that is collinear with the ray. When the query is located behind the
    * ray's origin, this is equivalent to calculating the distance between the query and the origin of
    * the ray.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>if {@code rayDirection.length() < }{@link EuclidGeometryTools#ONE_TRILLIONTH}, this method
    * returns the distance between {@code rayOrigin} and the given {@code point}.
    * </ul>
    * </p>
    *
    * @param point        the coordinates of the query.
    * @param rayOrigin    a point located on the line. Not modified.
    * @param rayDirection the direction of the line. Not modified.
    * @return the minimum distance between the 2D point and the 2D line.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static double distanceFromPoint2DToRay2D(FramePoint2DReadOnly point, FramePoint2DReadOnly rayOrigin, FrameVector2DReadOnly rayDirection)
   {
      point.checkReferenceFrameMatch(rayOrigin, rayDirection);
      return EuclidGeometryTools.distanceFromPoint2DToRay2D(point, rayOrigin, rayDirection);
   }

   /**
    * Computes the minimum distance between a 3D point and an infinitely long 3D line defined by two
    * points. <a href="http://mathworld.wolfram.com/Point-LineDistance3-Dimensional.html"> Useful
    * link</a>.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if
    * {@code firstPointOnLine.distance(secondPointOnLine) < }{@link EuclidGeometryTools#ONE_TRILLIONTH},
    * this method returns the distance between {@code firstPointOnLine} and the given {@code point}.
    * </ul>
    * </p>
    *
    * @param point             3D point to compute the distance from the line. Not modified.
    * @param firstPointOnLine  a first point located on the line. Not modified.
    * @param secondPointOnLine a second point located on the line. Not modified.
    * @return the minimum distance between the 3D point and the 3D line.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static double distanceFromPoint3DToLine3D(FramePoint3DReadOnly point, FramePoint3DReadOnly firstPointOnLine, FramePoint3DReadOnly secondPointOnLine)
   {
      point.checkReferenceFrameMatch(firstPointOnLine, secondPointOnLine);
      return EuclidGeometryTools.distanceFromPoint3DToLine3D(point, firstPointOnLine, secondPointOnLine);
   }

   /**
    * Computes the minimum distance between a 3D point and an infinitely long 3D line defined by a
    * point and a direction.
    * <a href="http://mathworld.wolfram.com/Point-LineDistance3-Dimensional.html"> Useful link</a>.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if {@code lineDirection.length() < }{@link EuclidGeometryTools#ONE_TRILLIONTH}, this method
    * returns the distance between {@code pointOnLine} and the given {@code point}.
    * </ul>
    * </p>
    *
    * @param point         3D point to compute the distance from the line. Not modified.
    * @param pointOnLine   point located on the line. Not modified.
    * @param lineDirection direction of the line. Not modified.
    * @return the minimum distance between the 3D point and the 3D line.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static double distanceFromPoint3DToLine3D(FramePoint3DReadOnly point, FramePoint3DReadOnly pointOnLine, FrameVector3DReadOnly lineDirection)
   {
      point.checkReferenceFrameMatch(pointOnLine, lineDirection);
      return EuclidGeometryTools.distanceFromPoint3DToLine3D(point, pointOnLine, lineDirection);
   }

   /**
    * Returns the minimum distance between a point and a given line segment.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if
    * {@code lineSegmentStart.distanceSquared(lineSegmentEnd) < }{@link EuclidGeometryTools#ONE_TRILLIONTH},
    * this method returns the distance between {@code lineSegmentStart} and the given {@code point}.
    * </ul>
    * </p>
    *
    * @param pointX           x-coordinate of point to be tested.
    * @param pointY           y-coordinate of point to be tested.
    * @param pointZ           z-coordinate of point to be tested.
    * @param lineSegmentStart first endpoint of the line segment. Not modified.
    * @param lineSegmentEnd   second endpoint of the line segment. Not modified.
    * @return the minimum distance between the 3D point and the 3D line segment.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static double distanceFromPoint3DToLineSegment3D(double pointX,
                                                           double pointY,
                                                           double pointZ,
                                                           FramePoint3DReadOnly lineSegmentStart,
                                                           FramePoint3DReadOnly lineSegmentEnd)
   {
      lineSegmentStart.checkReferenceFrameMatch(lineSegmentEnd);
      return EuclidGeometryTools.distanceFromPoint3DToLineSegment3D(pointX, pointY, pointZ, lineSegmentStart, lineSegmentEnd);
   }

   /**
    * Returns the minimum distance between a point and a given line segment.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if
    * {@code lineSegmentStart.distanceSquared(lineSegmentEnd) < }{@link EuclidGeometryTools#ONE_TRILLIONTH},
    * this method returns the distance between {@code lineSegmentStart} and the given {@code point}.
    * </ul>
    * </p>
    *
    * @param point            3D point to compute the distance from the line segment. Not modified.
    * @param lineSegmentStart first endpoint of the line segment. Not modified.
    * @param lineSegmentEnd   second endpoint of the line segment. Not modified.
    * @return the minimum distance between the 3D point and the 3D line segment.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static double distanceFromPoint3DToLineSegment3D(FramePoint3DReadOnly point,
                                                           FramePoint3DReadOnly lineSegmentStart,
                                                           FramePoint3DReadOnly lineSegmentEnd)
   {
      point.checkReferenceFrameMatch(lineSegmentStart, lineSegmentEnd);
      return EuclidGeometryTools.distanceFromPoint3DToLineSegment3D(point, lineSegmentStart, lineSegmentEnd);
   }

   /**
    * Computes the minimum distance between a given point and a plane.
    *
    * @param pointX       the x-coordinate of the query. Not modified.
    * @param pointY       the y-coordinate of the query. Not modified.
    * @param pointZ       the z-coordinate of the query. Not modified.
    * @param pointOnPlane a point located on the plane. Not modified.
    * @param planeNormal  the normal of the plane. Not modified.
    * @return the distance between the point and the plane.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static double distanceFromPoint3DToPlane3D(double pointX,
                                                     double pointY,
                                                     double pointZ,
                                                     FramePoint3DReadOnly pointOnPlane,
                                                     FrameVector3DReadOnly planeNormal)
   {
      pointOnPlane.checkReferenceFrameMatch(planeNormal);
      return EuclidGeometryTools.distanceFromPoint3DToPlane3D(pointX, pointY, pointZ, pointOnPlane, planeNormal);
   }

   /**
    * Computes the minimum distance between a given point and a plane.
    *
    * @param point        the 3D query. Not modified.
    * @param pointOnPlane a point located on the plane. Not modified.
    * @param planeNormal  the normal of the plane. Not modified.
    * @return the distance between the point and the plane.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static double distanceFromPoint3DToPlane3D(FramePoint3DReadOnly point, FramePoint3DReadOnly pointOnPlane, FrameVector3DReadOnly planeNormal)
   {
      point.checkReferenceFrameMatch(pointOnPlane, planeNormal);
      return EuclidGeometryTools.distanceFromPoint3DToPlane3D(point, pointOnPlane, planeNormal);
   }

   /**
    * Computes the minimum signed distance between a given point and a plane.
    * <p>
    * The returned value is negative when the query is located below the plane, positive otherwise.
    * </p>
    *
    * @param pointX       the x-coordinate of the query. Not modified.
    * @param pointY       the y-coordinate of the query. Not modified.
    * @param pointZ       the z-coordinate of the query. Not modified.
    * @param pointOnPlane a point located on the plane. Not modified.
    * @param planeNormal  the normal of the plane. Not modified.
    * @return the signed distance between the point and the plane.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static double signedDistanceFromPoint3DToPlane3D(double pointX,
                                                           double pointY,
                                                           double pointZ,
                                                           FramePoint3DReadOnly pointOnPlane,
                                                           FrameVector3DReadOnly planeNormal)
   {
      pointOnPlane.checkReferenceFrameMatch(planeNormal);
      return EuclidGeometryTools.signedDistanceFromPoint3DToPlane3D(pointX, pointY, pointZ, pointOnPlane, planeNormal);
   }

   /**
    * Computes the minimum signed distance between a given point and a plane.
    * <p>
    * The returned value is negative when the query is located below the plane, positive otherwise.
    * </p>
    *
    * @param point        the query. Not modified.
    * @param pointOnPlane a point located on the plane. Not modified.
    * @param planeNormal  the normal of the plane. Not modified.
    * @return the signed distance between the point and the plane.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static double signedDistanceFromPoint3DToPlane3D(FramePoint3DReadOnly point, FramePoint3DReadOnly pointOnPlane, FrameVector3DReadOnly planeNormal)
   {
      point.checkReferenceFrameMatch(pointOnPlane, planeNormal);
      return EuclidGeometryTools.signedDistanceFromPoint3DToPlane3D(point, pointOnPlane, planeNormal);
   }

   /**
    * Computes the minimum signed distance between a given point and a plane.
    * <p>
    * The returned value is negative when the query is located below the plane, positive otherwise.
    * </p>
    *
    * @param pointX             the x-coordinate of the query. Not modified.
    * @param pointY             the y-coordinate of the query. Not modified.
    * @param pointZ             the z-coordinate of the query. Not modified.
    * @param pointOnPlane       a point located on the plane. Not modified.
    * @param planeFirstTangent  a first tangent of the infinite plane. Not modified.
    * @param planeSecondTangent a second tangent of the infinite plane. Not modified.
    * @return the signed distance between the point and the plane.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static double signedDistanceFromPoint3DToPlane3D(double pointX,
                                                           double pointY,
                                                           double pointZ,
                                                           FramePoint3DReadOnly pointOnPlane,
                                                           FrameVector3DReadOnly planeFirstTangent,
                                                           FrameVector3DReadOnly planeSecondTangent)
   {
      pointOnPlane.checkReferenceFrameMatch(planeFirstTangent, planeSecondTangent);
      return EuclidGeometryTools.signedDistanceFromPoint3DToPlane3D(pointX, pointY, pointZ, pointOnPlane, planeFirstTangent, planeSecondTangent);
   }

   /**
    * Computes the minimum signed distance between a given point and a plane.
    * <p>
    * The returned value is negative when the query is located below the plane, positive otherwise.
    * </p>
    *
    * @param point              the query. Not modified.
    * @param pointOnPlane       a point located on the plane. Not modified.
    * @param planeFirstTangent  a first tangent of the infinite plane. Not modified.
    * @param planeSecondTangent a second tangent of the infinite plane. Not modified.
    * @return the signed distance between the point and the plane.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static double signedDistanceFromPoint3DToPlane3D(FramePoint3DReadOnly point,
                                                           FramePoint3DReadOnly pointOnPlane,
                                                           FrameVector3DReadOnly planeFirstTangent,
                                                           FrameVector3DReadOnly planeSecondTangent)
   {
      point.checkReferenceFrameMatch(pointOnPlane, planeFirstTangent, planeSecondTangent);
      return EuclidGeometryTools.signedDistanceFromPoint3DToPlane3D(point, pointOnPlane, planeFirstTangent, planeSecondTangent);
   }

   /**
    * Returns the square of the minimum distance between a point and a given line segment.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if
    * {@code lineSegmentStart.distanceSquared(lineSegmentEnd) < }{@link EuclidGeometryTools#ONE_TRILLIONTH},
    * this method returns the distance between {@code lineSegmentStart} and the given {@code point}.
    * </ul>
    * </p>
    *
    * @param pointX           x coordinate of point to be tested.
    * @param pointY           y coordinate of point to be tested.
    * @param lineSegmentStart first endpoint of the line segment. Not modified.
    * @param lineSegmentEnd   second endpoint of the line segment. Not modified.
    * @return the square of the minimum distance between the 2D point and the 2D line segment.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static double distanceSquaredFromPoint2DToLineSegment2D(double pointX,
                                                                  double pointY,
                                                                  FramePoint2DReadOnly lineSegmentStart,
                                                                  FramePoint2DReadOnly lineSegmentEnd)
   {
      lineSegmentStart.checkReferenceFrameMatch(lineSegmentEnd);
      return EuclidGeometryTools.distanceSquaredFromPoint2DToLineSegment2D(pointX, pointY, lineSegmentStart, lineSegmentEnd);
   }

   /**
    * Returns the square of the minimum distance between a point and a given line segment.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if
    * {@code lineSegmentStart.distanceSquared(lineSegmentEnd) < }{@link EuclidGeometryTools#ONE_TRILLIONTH},
    * this method returns the distance between {@code lineSegmentStart} and the given {@code point}.
    * </ul>
    * </p>
    *
    * @param point            coordinates of point to be tested.
    * @param lineSegmentStart first endpoint of the line segment. Not modified.
    * @param lineSegmentEnd   second endpoint of the line segment. Not modified.
    * @return the square of the minimum distance between the 2D point and the 2D line segment.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static double distanceSquaredFromPoint2DToLineSegment2D(FramePoint2DReadOnly point,
                                                                  FramePoint2DReadOnly lineSegmentStart,
                                                                  FramePoint2DReadOnly lineSegmentEnd)
   {
      point.checkReferenceFrameMatch(lineSegmentStart, lineSegmentEnd);
      return EuclidGeometryTools.distanceSquaredFromPoint2DToLineSegment2D(point, lineSegmentStart, lineSegmentEnd);
   }

   /**
    * Returns the square of the minimum distance between a point and a given line segment.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if
    * {@code lineSegmentStart.distanceSquared(lineSegmentEnd) < }{@link EuclidGeometryTools#ONE_TRILLIONTH},
    * this method returns the distance between {@code lineSegmentStart} and the given {@code point}.
    * </ul>
    * </p>
    *
    * @param pointX           x-coordinate of point to be tested.
    * @param pointY           y-coordinate of point to be tested.
    * @param pointZ           z-coordinate of point to be tested.
    * @param lineSegmentStart first endpoint of the line segment. Not modified.
    * @param lineSegmentEnd   second endpoint of the line segment. Not modified.
    * @return the square of the minimum distance between the 3D point and the 3D line segment.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static double distanceSquaredFromPoint3DToLineSegment3D(double pointX,
                                                                  double pointY,
                                                                  double pointZ,
                                                                  FramePoint3DReadOnly lineSegmentStart,
                                                                  FramePoint3DReadOnly lineSegmentEnd)
   {
      lineSegmentStart.checkReferenceFrameMatch(lineSegmentEnd);
      return EuclidGeometryTools.distanceSquaredFromPoint3DToLineSegment3D(pointX, pointY, pointZ, lineSegmentStart, lineSegmentEnd);
   }

   /**
    * Returns the square of the minimum distance between a point and a given line segment.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if
    * {@code lineSegmentStart.distanceSquared(lineSegmentEnd) < }{@link EuclidGeometryTools#ONE_TRILLIONTH},
    * this method returns the distance between {@code lineSegmentStart} and the given {@code point}.
    * </ul>
    * </p>
    *
    * @param point            3D point to compute the distance from the line segment. Not modified.
    * @param lineSegmentStart first endpoint of the line segment. Not modified.
    * @param lineSegmentEnd   second endpoint of the line segment. Not modified.
    * @return the square of the minimum distance between the 3D point and the 3D line segment.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static double distanceSquaredFromPoint3DToLineSegment3D(FramePoint3DReadOnly point,
                                                                  FramePoint3DReadOnly lineSegmentStart,
                                                                  FramePoint3DReadOnly lineSegmentEnd)
   {
      point.checkReferenceFrameMatch(lineSegmentStart, lineSegmentEnd);
      return EuclidGeometryTools.distanceSquaredFromPoint3DToLineSegment3D(point, lineSegmentStart, lineSegmentEnd);
   }

   /**
    * Test if a given line segment intersects a given plane.
    * <p>
    * Edge cases:
    * <ul>
    * <li>the line segment endpoints are equal, this method returns {@code false} whether the endpoints
    * are on the plane or not.
    * <li>one of the line segment endpoints is exactly on the plane, this method returns false.
    * </ul>
    * </p>
    *
    * @param pointOnPlane     a point located on the plane. Not modified.
    * @param planeNormal      the normal of the plane. Not modified.
    * @param lineSegmentStart first endpoint of the line segment. Not modified.
    * @param lineSegmentEnd   second endpoint of the line segment. Not modified.
    * @return {@code true} if an intersection line segment - plane exists, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static boolean doesLineSegment3DIntersectPlane3D(FramePoint3DReadOnly pointOnPlane,
                                                           FrameVector3DReadOnly planeNormal,
                                                           FramePoint3DReadOnly lineSegmentStart,
                                                           FramePoint3DReadOnly lineSegmentEnd)
   {
      pointOnPlane.checkReferenceFrameMatch(planeNormal, lineSegmentStart, lineSegmentEnd);
      return EuclidGeometryTools.doesLineSegment3DIntersectPlane3D(pointOnPlane, planeNormal, lineSegmentStart, lineSegmentEnd);
   }

   /**
    * Tests if an intersection exists between an infinitely long 2D line (defined by a 2D point and a
    * 2D direction) and a 2D line segment (defined by its two 2D endpoints).
    * <p>
    * Edge cases:
    * <ul>
    * <li>When the line and the line segment are parallel but not collinear, they do not intersect.
    * <li>When the line and the line segment are collinear, they are assumed to intersect.
    * <li>When the line intersects the line segment at one of its endpoints, this method returns
    * {@code true} and the endpoint is the intersection.
    * </ul>
    * </p>
    *
    * @param pointOnLineX     the x-coordinate of a point located on the line.
    * @param pointOnLineY     the y-coordinate of a point located on the line.
    * @param lineDirectionX   the x-component of the line direction.
    * @param lineDirectionY   the y-component of the line direction.
    * @param lineSegmentStart the first endpoint of the line segment. Not modified.
    * @param lineSegmentEnd   the second endpoint of the line segment. Not modified.
    * @return {@code true} if the line intersects the line segment, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static boolean doLine2DAndLineSegment2DIntersect(double pointOnLineX,
                                                           double pointOnLineY,
                                                           double lineDirectionX,
                                                           double lineDirectionY,
                                                           FramePoint2DReadOnly lineSegmentStart,
                                                           FramePoint2DReadOnly lineSegmentEnd)
   {
      lineSegmentStart.checkReferenceFrameMatch(lineSegmentEnd);
      return EuclidGeometryTools.doLine2DAndLineSegment2DIntersect(pointOnLineX,
                                                                   pointOnLineY,
                                                                   lineDirectionX,
                                                                   lineDirectionY,
                                                                   lineSegmentStart,
                                                                   lineSegmentEnd);
   }

   /**
    * Tests if an intersection exists between an infinitely long 2D line (defined by a 2D point and a
    * 2D direction) and a 2D line segment (defined by its two 2D endpoints).
    * <p>
    * Edge cases:
    * <ul>
    * <li>When the line and the line segment are parallel but not collinear, they do not intersect.
    * <li>When the line and the line segment are collinear, they are assumed to intersect.
    * <li>When the line intersects the line segment at one of its endpoints, this method returns
    * {@code true} and the endpoint is the intersection.
    * </ul>
    * </p>
    *
    * @param pointOnLine      a point located on the line. Not modified.
    * @param lineDirection    the line direction. Not modified.
    * @param lineSegmentStart the first endpoint of the line segment. Not modified.
    * @param lineSegmentEnd   the second endpoint of the line segment. Not modified.
    * @return {@code true} if the line intersects the line segment, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static boolean doLine2DAndLineSegment2DIntersect(FramePoint2DReadOnly pointOnLine,
                                                           FrameVector2DReadOnly lineDirection,
                                                           FramePoint2DReadOnly lineSegmentStart,
                                                           FramePoint2DReadOnly lineSegmentEnd)
   {
      pointOnLine.checkReferenceFrameMatch(lineDirection, lineSegmentStart, lineSegmentEnd);
      return EuclidGeometryTools.doLine2DAndLineSegment2DIntersect(pointOnLine, lineDirection, lineSegmentStart, lineSegmentEnd);
   }

   /**
    * Test if two line segments intersect each other.
    * <p>
    * Edge cases:
    * <ul>
    * <li>When the two line segments are parallel but not collinear, this method returns false.
    * <li>When the two line segments are collinear, this methods returns {@code true} only if the two
    * line segments overlap or have at least one common endpoint.
    * <li>When the two line segments have a common endpoint, this method returns true.
    * </ul>
    * </p>
    *
    * @param lineSegmentStart1 first endpoint of the first line segment. Not modified.
    * @param lineSegmentEnd1   second endpoint of the first line segment. Not modified.
    * @param lineSegmentStart2 first endpoint of the second line segment. Not modified.
    * @param lineSegmentEnd2   second endpoint of the second line segment. Not modified.
    * @return {@code true} if the two line segments intersect, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static boolean doLineSegment2DsIntersect(FramePoint2DReadOnly lineSegmentStart1,
                                                   FramePoint2DReadOnly lineSegmentEnd1,
                                                   FramePoint2DReadOnly lineSegmentStart2,
                                                   FramePoint2DReadOnly lineSegmentEnd2)
   {
      lineSegmentStart1.checkReferenceFrameMatch(lineSegmentEnd1, lineSegmentStart2, lineSegmentEnd2);
      return EuclidGeometryTools.doLineSegment2DsIntersect(lineSegmentStart1, lineSegmentEnd1, lineSegmentStart2, lineSegmentEnd2);
   }

   /**
    * Tests if an intersection exists between a 2D ray and a 2D line segment.
    * <p>
    * Edge cases:
    * <ul>
    * <li>When the ray and the line segment are parallel but not collinear, they do not intersect.
    * <li>When the ray and the line segment are collinear, they are assumed to intersect.
    * <li>When the ray intersects the line segment at one of its endpoints, this method returns
    * {@code true} and the endpoint is the intersection.
    * </ul>
    * </p>
    *
    * @param rayOrigin        a point located on the ray. Not modified.
    * @param rayDirection     the direction of the ray. Not modified.
    * @param lineSegmentStart first endpoint of the line segment. Not modified.
    * @param lineSegmentEnd   second endpoint of the line segment. Not modified.
    * @return {@code true} if the ray and line segment intersect, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static boolean doRay2DAndLineSegment2DIntersect(FramePoint2DReadOnly rayOrigin,
                                                          FrameVector2DReadOnly rayDirection,
                                                          FramePoint2DReadOnly lineSegmentStart,
                                                          FramePoint2DReadOnly lineSegmentEnd)
   {
      rayOrigin.checkReferenceFrameMatch(rayDirection, lineSegmentStart, lineSegmentEnd);
      return EuclidGeometryTools.doRay2DAndLineSegment2DIntersect(rayOrigin, rayDirection, lineSegmentStart, lineSegmentEnd);
   }

   /**
    * Computes the dot product between two vectors each defined by two points:
    * <ul>
    * <li>{@code vector1 = end1 - start1}
    * <li>{@code vector2 = end2 - start2}
    * </ul>
    *
    * @param start1 the origin of the first vector. Not modified.
    * @param end1   the end of the first vector. Not modified.
    * @param start2 the origin of the second vector. Not modified.
    * @param end2   the end of the second vector. Not modified.
    * @return the value of the dot product of the two vectors.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static double dotProduct(FramePoint2DReadOnly start1, FramePoint2DReadOnly end1, FramePoint2DReadOnly start2, FramePoint2DReadOnly end2)
   {
      start1.checkReferenceFrameMatch(end1, start2, end2);
      return EuclidGeometryTools.dotProduct(start1, end1, start2, end2);
   }

   /**
    * Computes the dot product between two vectors each defined by two points:
    * <ul>
    * <li>{@code vector1 = end1 - start1}
    * <li>{@code vector2 = end2 - start2}
    * </ul>
    *
    * @param start1 the origin of the first vector. Not modified.
    * @param end1   the end of the first vector. Not modified.
    * @param start2 the origin of the second vector. Not modified.
    * @param end2   the end of the second vector. Not modified.
    * @return the value of the dot product of the two vectors.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static double dotProduct(FramePoint3DReadOnly start1, FramePoint3DReadOnly end1, FramePoint3DReadOnly start2, FramePoint3DReadOnly end2)
   {
      start1.checkReferenceFrameMatch(end1, start2, end2);
      return EuclidGeometryTools.dotProduct(start1, end1, start2, end2);
   }

   /**
    * Computes the coordinates of the possible intersections between a line and an axis-aligned
    * bounding box.
    * <p>
    * <a href=
    * "https://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-box-intersection">Useful
    * link</a>.
    * </p>
    * <p>
    * Intersections between the line and the bounding box are not restricted to exist between the two
    * given points defining the line.
    * <p>
    * In the case the line and the bounding box do not intersect, this method returns {@code 0} and
    * {@code firstIntersectionToPack} and {@code secondIntersectionToPack} are set to
    * {@link Double#NaN}.
    * </p>
    *
    * @param boundingBoxMin           the minimum coordinate of the bounding box. Not modified.
    * @param boundingBoxMax           the maximum coordinate of the bounding box. Not modified.
    * @param firstPointOnLine         a first point located on the infinitely long line. Not modified.
    * @param secondPointOnLine        a second point located on the infinitely long line. Not modified.
    * @param firstIntersectionToPack  the coordinate of the first intersection. Can be {@code null}.
    *                                 Modified.
    * @param secondIntersectionToPack the coordinate of the second intersection. Can be {@code null}.
    *                                 Modified.
    * @return the number of intersections between the line and the bounding box. It is either equal to
    *         0 or 2.
    * @throws BoundingBoxException            if any of the minimum coordinates of the bounding box is
    *                                         strictly greater than the maximum coordinate of the
    *                                         bounding box on the same axis.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static int intersectionBetweenLine2DAndBoundingBox2D(FramePoint2DReadOnly boundingBoxMin,
                                                               FramePoint2DReadOnly boundingBoxMax,
                                                               FramePoint2DReadOnly firstPointOnLine,
                                                               FramePoint2DReadOnly secondPointOnLine,
                                                               FixedFramePoint2DBasics firstIntersectionToPack,
                                                               FixedFramePoint2DBasics secondIntersectionToPack)
   {
      boundingBoxMin.checkReferenceFrameMatch(boundingBoxMax, firstPointOnLine, secondPointOnLine);
      if (firstIntersectionToPack != null)
         boundingBoxMin.checkReferenceFrameMatch(firstIntersectionToPack);
      if (secondIntersectionToPack != null)
         boundingBoxMin.checkReferenceFrameMatch(secondIntersectionToPack);
      int numberOfIntersections = EuclidGeometryTools.intersectionBetweenLine2DAndBoundingBox2D(boundingBoxMin,
                                                                                                boundingBoxMax,
                                                                                                firstPointOnLine,
                                                                                                secondPointOnLine,
                                                                                                firstIntersectionToPack,
                                                                                                secondIntersectionToPack);

      return numberOfIntersections;
   }

   /**
    * Computes the coordinates of the possible intersections between a line and an axis-aligned
    * bounding box.
    * <p>
    * <a href=
    * "https://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-box-intersection">Useful
    * link</a>.
    * </p>
    * <p>
    * Intersections between the line and the bounding box are not restricted to exist between the two
    * given points defining the line.
    * <p>
    * In the case the line and the bounding box do not intersect, this method returns {@code 0} and
    * {@code firstIntersectionToPack} and {@code secondIntersectionToPack} are set to
    * {@link Double#NaN}.
    * </p>
    *
    * @param boundingBoxMin           the minimum coordinate of the bounding box. Not modified.
    * @param boundingBoxMax           the maximum coordinate of the bounding box. Not modified.
    * @param firstPointOnLine         a first point located on the infinitely long line. Not modified.
    * @param secondPointOnLine        a second point located on the infinitely long line. Not modified.
    * @param firstIntersectionToPack  the coordinate of the first intersection. Can be {@code null}.
    *                                 Modified.
    * @param secondIntersectionToPack the coordinate of the second intersection. Can be {@code null}.
    *                                 Modified.
    * @return the number of intersections between the line and the bounding box. It is either equal to
    *         0 or 2.
    * @throws BoundingBoxException            if any of the minimum coordinates of the bounding box is
    *                                         strictly greater than the maximum coordinate of the
    *                                         bounding box on the same axis.
    * @throws ReferenceFrameMismatchException if the read-only arguments are not all expressed in the
    *                                         same reference frame.
    */
   public static int intersectionBetweenLine2DAndBoundingBox2D(FramePoint2DReadOnly boundingBoxMin,
                                                               FramePoint2DReadOnly boundingBoxMax,
                                                               FramePoint2DReadOnly firstPointOnLine,
                                                               FramePoint2DReadOnly secondPointOnLine,
                                                               FramePoint2DBasics firstIntersectionToPack,
                                                               FramePoint2DBasics secondIntersectionToPack)
   {
      boundingBoxMin.checkReferenceFrameMatch(boundingBoxMax, firstPointOnLine, secondPointOnLine);
      int numberOfIntersections = EuclidGeometryTools.intersectionBetweenLine2DAndBoundingBox2D(boundingBoxMin,
                                                                                                boundingBoxMax,
                                                                                                firstPointOnLine,
                                                                                                secondPointOnLine,
                                                                                                firstIntersectionToPack,
                                                                                                secondIntersectionToPack);

      // Set the correct reference frame.
      if (firstIntersectionToPack != null)
         firstIntersectionToPack.setReferenceFrame(firstPointOnLine.getReferenceFrame());
      if (secondIntersectionToPack != null)
         secondIntersectionToPack.setReferenceFrame(firstPointOnLine.getReferenceFrame());

      return numberOfIntersections;
   }

   /**
    * Computes the coordinates of the possible intersections between a line and an axis-aligned
    * bounding box.
    * <p>
    * <a href=
    * "https://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-box-intersection">Useful
    * link</a>.
    * </p>
    * <p>
    * In the case the line and the bounding box do not intersect, this method returns {@code 0} and
    * {@code firstIntersectionToPack} and {@code secondIntersectionToPack} are set to
    * {@link Double#NaN}.
    * </p>
    *
    * @param boundingBoxMin           the minimum coordinate of the bounding box. Not modified.
    * @param boundingBoxMax           the maximum coordinate of the bounding box. Not modified.
    * @param pointOnLine              a point located on the infinitely long line. Not modified.
    * @param lineDirection            the direction of the line. Not modified.
    * @param firstIntersectionToPack  the coordinate of the first intersection. Can be {@code null}.
    *                                 Modified.
    * @param secondIntersectionToPack the coordinate of the second intersection. Can be {@code null}.
    *                                 Modified.
    * @return the number of intersections between the line and the bounding box. It is either equal to
    *         0, 1, or 2.
    * @throws BoundingBoxException            if any of the minimum coordinates of the bounding box is
    *                                         strictly greater than the maximum coordinate of the
    *                                         bounding box on the same axis.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static int intersectionBetweenLine2DAndBoundingBox2D(FramePoint2DReadOnly boundingBoxMin,
                                                               FramePoint2DReadOnly boundingBoxMax,
                                                               FramePoint2DReadOnly pointOnLine,
                                                               FrameVector2DReadOnly lineDirection,
                                                               FixedFramePoint2DBasics firstIntersectionToPack,
                                                               FixedFramePoint2DBasics secondIntersectionToPack)
   {
      boundingBoxMin.checkReferenceFrameMatch(boundingBoxMax, pointOnLine, lineDirection);
      if (firstIntersectionToPack != null)
         boundingBoxMin.checkReferenceFrameMatch(firstIntersectionToPack);
      if (secondIntersectionToPack != null)
         boundingBoxMin.checkReferenceFrameMatch(secondIntersectionToPack);
      int numberOfIntersections = EuclidGeometryTools.intersectionBetweenLine2DAndBoundingBox2D(boundingBoxMin,
                                                                                                boundingBoxMax,
                                                                                                pointOnLine,
                                                                                                lineDirection,
                                                                                                firstIntersectionToPack,
                                                                                                secondIntersectionToPack);

      return numberOfIntersections;
   }

   /**
    * Computes the coordinates of the possible intersections between a line and an axis-aligned
    * bounding box.
    * <p>
    * <a href=
    * "https://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-box-intersection">Useful
    * link</a>.
    * </p>
    * <p>
    * In the case the line and the bounding box do not intersect, this method returns {@code 0} and
    * {@code firstIntersectionToPack} and {@code secondIntersectionToPack} are set to
    * {@link Double#NaN}.
    * </p>
    *
    * @param boundingBoxMin           the minimum coordinate of the bounding box. Not modified.
    * @param boundingBoxMax           the maximum coordinate of the bounding box. Not modified.
    * @param pointOnLine              a point located on the infinitely long line. Not modified.
    * @param lineDirection            the direction of the line. Not modified.
    * @param firstIntersectionToPack  the coordinate of the first intersection. Can be {@code null}.
    *                                 Modified.
    * @param secondIntersectionToPack the coordinate of the second intersection. Can be {@code null}.
    *                                 Modified.
    * @return the number of intersections between the line and the bounding box. It is either equal to
    *         0, 1, or 2.
    * @throws BoundingBoxException            if any of the minimum coordinates of the bounding box is
    *                                         strictly greater than the maximum coordinate of the
    *                                         bounding box on the same axis.
    * @throws ReferenceFrameMismatchException if the read-only arguments are not all expressed in the
    *                                         same reference frame.
    */
   public static int intersectionBetweenLine2DAndBoundingBox2D(FramePoint2DReadOnly boundingBoxMin,
                                                               FramePoint2DReadOnly boundingBoxMax,
                                                               FramePoint2DReadOnly pointOnLine,
                                                               FrameVector2DReadOnly lineDirection,
                                                               FramePoint2DBasics firstIntersectionToPack,
                                                               FramePoint2DBasics secondIntersectionToPack)
   {
      boundingBoxMin.checkReferenceFrameMatch(boundingBoxMax, pointOnLine, lineDirection);
      int numberOfIntersections = EuclidGeometryTools.intersectionBetweenLine2DAndBoundingBox2D(boundingBoxMin,
                                                                                                boundingBoxMax,
                                                                                                pointOnLine,
                                                                                                lineDirection,
                                                                                                firstIntersectionToPack,
                                                                                                secondIntersectionToPack);

      // Set the correct reference frame.
      if (firstIntersectionToPack != null)
         firstIntersectionToPack.setReferenceFrame(pointOnLine.getReferenceFrame());
      if (secondIntersectionToPack != null)
         secondIntersectionToPack.setReferenceFrame(pointOnLine.getReferenceFrame());

      return numberOfIntersections;
   }

   /**
    * Computes the intersection between an infinitely long 2D line (defined by a 2D point and a 2D
    * direction) and a 2D line segment (defined by its two 2D endpoints).
    * <p>
    * Edge cases:
    * <ul>
    * <li>When the line and the line segment are parallel but not collinear, they do not intersect,
    * this method returns {@code null}.
    * <li>When the line and the line segment are collinear, they are assumed to intersect at
    * {@code lineSegmentStart}.
    * <li>When the line intersects the line segment at one of its endpoints, this method returns that
    * same endpoint.
    * </ul>
    * </p>
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param pointOnLine      a point located on the line. Not modified.
    * @param lineDirection    the line direction. Not modified.
    * @param lineSegmentStart the first endpoint of the line segment. Not modified.
    * @param lineSegmentEnd   the second endpoint of the line segment. Not modified.
    * @return the 2D point of intersection if it exist, {@code null} otherwise.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static FramePoint2D intersectionBetweenLine2DAndLineSegment2D(FramePoint2DReadOnly pointOnLine,
                                                                        FrameVector2DReadOnly lineDirection,
                                                                        FramePoint2DReadOnly lineSegmentStart,
                                                                        FramePoint2DReadOnly lineSegmentEnd)
   {
      pointOnLine.checkReferenceFrameMatch(lineDirection, lineSegmentStart, lineSegmentEnd);
      Point2D intersection = EuclidGeometryTools.intersectionBetweenLine2DAndLineSegment2D(pointOnLine, lineDirection, lineSegmentStart, lineSegmentEnd);
      if (intersection == null)
         return null;
      else
         return new FramePoint2D(pointOnLine.getReferenceFrame(), intersection);
   }

   /**
    * Computes the intersection between an infinitely long 2D line (defined by a 2D point and a 2D
    * direction) and a 2D line segment (defined by its two 2D endpoints).
    * <p>
    * Edge cases:
    * <ul>
    * <li>When the line and the line segment are parallel but not collinear, they do not intersect.
    * <li>When the line and the line segment are collinear, they are assumed to intersect at
    * {@code lineSegmentStart}.
    * <li>When the line intersects the line segment at one of its endpoints, this method returns
    * {@code true} and the endpoint is the intersection.
    * <li>When there is no intersection, this method returns {@code false} and
    * {@code intersectionToPack} is set to {@link Double#NaN}.
    * </ul>
    * </p>
    *
    * @param pointOnLine        a point located on the line. Not modified.
    * @param lineDirection      the line direction. Not modified.
    * @param lineSegmentStart   the first endpoint of the line segment. Not modified.
    * @param lineSegmentEnd     the second endpoint of the line segment. Not modified.
    * @param intersectionToPack the 2D point in which the result is stored. Can be {@code null}.
    *                           Modified.
    * @return {@code true} if the line intersects the line segment, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static boolean intersectionBetweenLine2DAndLineSegment2D(FramePoint2DReadOnly pointOnLine,
                                                                   FrameVector2DReadOnly lineDirection,
                                                                   FramePoint2DReadOnly lineSegmentStart,
                                                                   FramePoint2DReadOnly lineSegmentEnd,
                                                                   FixedFramePoint2DBasics intersectionToPack)
   {
      pointOnLine.checkReferenceFrameMatch(lineDirection, lineSegmentStart, lineSegmentEnd);
      if (intersectionToPack != null)
         pointOnLine.checkReferenceFrameMatch(intersectionToPack);
      boolean success = EuclidGeometryTools.intersectionBetweenLine2DAndLineSegment2D(pointOnLine,
                                                                                      lineDirection,
                                                                                      lineSegmentStart,
                                                                                      lineSegmentEnd,
                                                                                      intersectionToPack);

      return success;
   }

   /**
    * Computes the intersection between an infinitely long 2D line (defined by a 2D point and a 2D
    * direction) and a 2D line segment (defined by its two 2D endpoints).
    * <p>
    * Edge cases:
    * <ul>
    * <li>When the line and the line segment are parallel but not collinear, they do not intersect.
    * <li>When the line and the line segment are collinear, they are assumed to intersect at
    * {@code lineSegmentStart}.
    * <li>When the line intersects the line segment at one of its endpoints, this method returns
    * {@code true} and the endpoint is the intersection.
    * <li>When there is no intersection, this method returns {@code false} and
    * {@code intersectionToPack} is set to {@link Double#NaN}.
    * </ul>
    * </p>
    *
    * @param pointOnLine        a point located on the line. Not modified.
    * @param lineDirection      the line direction. Not modified.
    * @param lineSegmentStart   the first endpoint of the line segment. Not modified.
    * @param lineSegmentEnd     the second endpoint of the line segment. Not modified.
    * @param intersectionToPack the 2D point in which the result is stored. Can be {@code null}.
    *                           Modified.
    * @return {@code true} if the line intersects the line segment, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if the read-only arguments are not all expressed in the
    *                                         same reference frame.
    */
   public static boolean intersectionBetweenLine2DAndLineSegment2D(FramePoint2DReadOnly pointOnLine,
                                                                   FrameVector2DReadOnly lineDirection,
                                                                   FramePoint2DReadOnly lineSegmentStart,
                                                                   FramePoint2DReadOnly lineSegmentEnd,
                                                                   FramePoint2DBasics intersectionToPack)
   {
      pointOnLine.checkReferenceFrameMatch(lineDirection, lineSegmentStart, lineSegmentEnd);
      boolean success = EuclidGeometryTools.intersectionBetweenLine2DAndLineSegment2D(pointOnLine,
                                                                                      lineDirection,
                                                                                      lineSegmentStart,
                                                                                      lineSegmentEnd,
                                                                                      intersectionToPack);

      if (intersectionToPack != null)
         intersectionToPack.setReferenceFrame(pointOnLine.getReferenceFrame());

      return success;
   }

   /**
    * Computes the coordinates of the possible intersections between a line and an axis-aligned
    * bounding box.
    * <p>
    * <a href=
    * "https://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-box-intersection">Useful
    * link</a>.
    * </p>
    * <p>
    * Intersections between the line and the bounding box are not restricted to exist between the two
    * given points defining the line.
    * <p>
    * In the case the line and the bounding box do not intersect, this method returns {@code 0} and
    * {@code firstIntersectionToPack} and {@code secondIntersectionToPack} are set to
    * {@link Double#NaN}.
    * </p>
    *
    * @param boundingBoxMin           the minimum coordinate of the bounding box. Not modified.
    * @param boundingBoxMax           the maximum coordinate of the bounding box. Not modified.
    * @param firstPointOnLine         a first point located on the infinitely long line. Not modified.
    * @param secondPointOnLine        a second point located on the infinitely long line. Not modified.
    * @param firstIntersectionToPack  the coordinate of the first intersection. Can be {@code null}.
    *                                 Modified.
    * @param secondIntersectionToPack the coordinate of the second intersection. Can be {@code null}.
    *                                 Modified.
    * @return the number of intersections between the line and the bounding box. It is either equal to
    *         0, 1, or 2.
    * @throws BoundingBoxException            if any of the minimum coordinates of the bounding box is
    *                                         strictly greater than the maximum coordinate of the
    *                                         bounding box on the same axis.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static int intersectionBetweenLine3DAndBoundingBox3D(FramePoint3DReadOnly boundingBoxMin,
                                                               FramePoint3DReadOnly boundingBoxMax,
                                                               FramePoint3DReadOnly firstPointOnLine,
                                                               FramePoint3DReadOnly secondPointOnLine,
                                                               FixedFramePoint3DBasics firstIntersectionToPack,
                                                               FixedFramePoint3DBasics secondIntersectionToPack)
   {
      boundingBoxMin.checkReferenceFrameMatch(boundingBoxMax, firstPointOnLine, secondPointOnLine);
      if (firstIntersectionToPack != null)
         boundingBoxMin.checkReferenceFrameMatch(firstIntersectionToPack);
      if (secondIntersectionToPack != null)
         boundingBoxMin.checkReferenceFrameMatch(secondIntersectionToPack);
      int numberOfIntersections = EuclidGeometryTools.intersectionBetweenLine3DAndBoundingBox3D(boundingBoxMin,
                                                                                                boundingBoxMax,
                                                                                                firstPointOnLine,
                                                                                                secondPointOnLine,
                                                                                                firstIntersectionToPack,
                                                                                                secondIntersectionToPack);

      return numberOfIntersections;
   }

   /**
    * Computes the coordinates of the possible intersections between a line and an axis-aligned
    * bounding box.
    * <p>
    * <a href=
    * "https://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-box-intersection">Useful
    * link</a>.
    * </p>
    * <p>
    * Intersections between the line and the bounding box are not restricted to exist between the two
    * given points defining the line.
    * <p>
    * In the case the line and the bounding box do not intersect, this method returns {@code 0} and
    * {@code firstIntersectionToPack} and {@code secondIntersectionToPack} are set to
    * {@link Double#NaN}.
    * </p>
    *
    * @param boundingBoxMin           the minimum coordinate of the bounding box. Not modified.
    * @param boundingBoxMax           the maximum coordinate of the bounding box. Not modified.
    * @param firstPointOnLine         a first point located on the infinitely long line. Not modified.
    * @param secondPointOnLine        a second point located on the infinitely long line. Not modified.
    * @param firstIntersectionToPack  the coordinate of the first intersection. Can be {@code null}.
    *                                 Modified.
    * @param secondIntersectionToPack the coordinate of the second intersection. Can be {@code null}.
    *                                 Modified.
    * @return the number of intersections between the line and the bounding box. It is either equal to
    *         0, 1, or 2.
    * @throws BoundingBoxException            if any of the minimum coordinates of the bounding box is
    *                                         strictly greater than the maximum coordinate of the
    *                                         bounding box on the same axis.
    * @throws ReferenceFrameMismatchException if the read-only arguments are not all expressed in the
    *                                         same reference frame.
    */
   public static int intersectionBetweenLine3DAndBoundingBox3D(FramePoint3DReadOnly boundingBoxMin,
                                                               FramePoint3DReadOnly boundingBoxMax,
                                                               FramePoint3DReadOnly firstPointOnLine,
                                                               FramePoint3DReadOnly secondPointOnLine,
                                                               FramePoint3DBasics firstIntersectionToPack,
                                                               FramePoint3DBasics secondIntersectionToPack)
   {
      boundingBoxMin.checkReferenceFrameMatch(boundingBoxMax, firstPointOnLine, secondPointOnLine);
      int numberOfIntersections = EuclidGeometryTools.intersectionBetweenLine3DAndBoundingBox3D(boundingBoxMin,
                                                                                                boundingBoxMax,
                                                                                                firstPointOnLine,
                                                                                                secondPointOnLine,
                                                                                                firstIntersectionToPack,
                                                                                                secondIntersectionToPack);

      // Set the correct reference frame.
      if (firstIntersectionToPack != null)
         firstIntersectionToPack.setReferenceFrame(firstPointOnLine.getReferenceFrame());
      if (secondIntersectionToPack != null)
         secondIntersectionToPack.setReferenceFrame(firstPointOnLine.getReferenceFrame());

      return numberOfIntersections;
   }

   /**
    * Computes the coordinates of the possible intersections between a line and an axis-aligned
    * bounding box.
    * <p>
    * <a href=
    * "https://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-box-intersection">Useful
    * link</a>.
    * </p>
    * <p>
    * In the case the line and the bounding box do not intersect, this method returns {@code 0} and
    * {@code firstIntersectionToPack} and {@code secondIntersectionToPack} remain unmodified.
    * </p>
    *
    * @param boundingBoxMin           the minimum coordinate of the bounding box. Not modified.
    * @param boundingBoxMax           the maximum coordinate of the bounding box. Not modified.
    * @param pointOnLine              a point located on the infinitely long line. Not modified.
    * @param lineDirection            the direction of the line. Not modified.
    * @param firstIntersectionToPack  the coordinate of the first intersection. Can be {@code null}.
    *                                 Modified.
    * @param secondIntersectionToPack the coordinate of the second intersection. Can be {@code null}.
    *                                 Modified.
    * @return the number of intersections between the line and the bounding box. It is either equal to
    *         0, 1, or 2.
    * @throws BoundingBoxException            if any of the minimum coordinates of the bounding box is
    *                                         strictly greater than the maximum coordinate of the
    *                                         bounding box on the same axis.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static int intersectionBetweenLine3DAndBoundingBox3D(FramePoint3DReadOnly boundingBoxMin,
                                                               FramePoint3DReadOnly boundingBoxMax,
                                                               FramePoint3DReadOnly pointOnLine,
                                                               FrameVector3DReadOnly lineDirection,
                                                               FixedFramePoint3DBasics firstIntersectionToPack,
                                                               FixedFramePoint3DBasics secondIntersectionToPack)
   {
      boundingBoxMin.checkReferenceFrameMatch(boundingBoxMax, pointOnLine, lineDirection);
      if (firstIntersectionToPack != null)
         boundingBoxMin.checkReferenceFrameMatch(firstIntersectionToPack);
      if (secondIntersectionToPack != null)
         boundingBoxMin.checkReferenceFrameMatch(secondIntersectionToPack);
      int numberOfIntersections = EuclidGeometryTools.intersectionBetweenLine3DAndBoundingBox3D(boundingBoxMin,
                                                                                                boundingBoxMax,
                                                                                                pointOnLine,
                                                                                                lineDirection,
                                                                                                firstIntersectionToPack,
                                                                                                secondIntersectionToPack);

      return numberOfIntersections;
   }

   /**
    * Computes the coordinates of the possible intersections between a line and an axis-aligned
    * bounding box.
    * <p>
    * <a href=
    * "https://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-box-intersection">Useful
    * link</a>.
    * </p>
    * <p>
    * In the case the line and the bounding box do not intersect, this method returns {@code 0} and
    * {@code firstIntersectionToPack} and {@code secondIntersectionToPack} remain unmodified.
    * </p>
    *
    * @param boundingBoxMin           the minimum coordinate of the bounding box. Not modified.
    * @param boundingBoxMax           the maximum coordinate of the bounding box. Not modified.
    * @param pointOnLine              a point located on the infinitely long line. Not modified.
    * @param lineDirection            the direction of the line. Not modified.
    * @param firstIntersectionToPack  the coordinate of the first intersection. Can be {@code null}.
    *                                 Modified.
    * @param secondIntersectionToPack the coordinate of the second intersection. Can be {@code null}.
    *                                 Modified.
    * @return the number of intersections between the line and the bounding box. It is either equal to
    *         0, 1, or 2.
    * @throws BoundingBoxException            if any of the minimum coordinates of the bounding box is
    *                                         strictly greater than the maximum coordinate of the
    *                                         bounding box on the same axis.
    * @throws ReferenceFrameMismatchException if the read-only arguments are not all expressed in the
    *                                         same reference frame.
    */
   public static int intersectionBetweenLine3DAndBoundingBox3D(FramePoint3DReadOnly boundingBoxMin,
                                                               FramePoint3DReadOnly boundingBoxMax,
                                                               FramePoint3DReadOnly pointOnLine,
                                                               FrameVector3DReadOnly lineDirection,
                                                               FramePoint3DBasics firstIntersectionToPack,
                                                               FramePoint3DBasics secondIntersectionToPack)
   {
      boundingBoxMin.checkReferenceFrameMatch(boundingBoxMax, pointOnLine, lineDirection);
      int numberOfIntersections = EuclidGeometryTools.intersectionBetweenLine3DAndBoundingBox3D(boundingBoxMin,
                                                                                                boundingBoxMax,
                                                                                                pointOnLine,
                                                                                                lineDirection,
                                                                                                firstIntersectionToPack,
                                                                                                secondIntersectionToPack);

      // Set the correct reference frame.
      if (firstIntersectionToPack != null)
         firstIntersectionToPack.setReferenceFrame(pointOnLine.getReferenceFrame());
      if (secondIntersectionToPack != null)
         secondIntersectionToPack.setReferenceFrame(pointOnLine.getReferenceFrame());

      return numberOfIntersections;
   }

   /**
    * Computes the coordinates of the possible intersections between a line and an axis-aligned
    * bounding box.
    * <p>
    * <a href=
    * "https://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-box-intersection">Useful
    * link</a>.
    * </p>
    * <p>
    * In the case the line and the bounding box do not intersect, this method returns {@code 0} and
    * {@code firstIntersectionToPack} and {@code secondIntersectionToPack} are set to
    * {@link Double#NaN}.
    * </p>
    *
    * @param boundingBoxMinX          the minimum x-coordinate of the bounding box.
    * @param boundingBoxMinY          the minimum y-coordinate of the bounding box.
    * @param boundingBoxMinZ          the minimum z-coordinate of the bounding box.
    * @param boundingBoxMaxX          the maximum x-coordinate of the bounding box.
    * @param boundingBoxMaxY          the maximum y-coordinate of the bounding box.
    * @param boundingBoxMaxZ          the maximum z-coordinate of the bounding box.
    * @param pointOnLine              a point located on the infinitely long line. Not modified.
    * @param lineDirection            the direction of the line. Not modified.
    * @param firstIntersectionToPack  the coordinate of the first intersection. Can be {@code null}.
    *                                 Modified.
    * @param secondIntersectionToPack the coordinate of the second intersection. Can be {@code null}.
    *                                 Modified.
    * @return the number of intersections between the line and the bounding box. It is either equal to
    *         0, 1, or 2.
    * @throws BoundingBoxException            if any of the minimum coordinates of the bounding box is
    *                                         strictly greater than the maximum coordinate of the
    *                                         bounding box on the same axis.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static int intersectionBetweenLine3DAndBoundingBox3D(double boundingBoxMinX,
                                                               double boundingBoxMinY,
                                                               double boundingBoxMinZ,
                                                               double boundingBoxMaxX,
                                                               double boundingBoxMaxY,
                                                               double boundingBoxMaxZ,
                                                               FramePoint3DReadOnly pointOnLine,
                                                               FrameVector3DReadOnly lineDirection,
                                                               FixedFramePoint3DBasics firstIntersectionToPack,
                                                               FixedFramePoint3DBasics secondIntersectionToPack)
   {
      pointOnLine.checkReferenceFrameMatch(lineDirection);
      if (firstIntersectionToPack != null)
         pointOnLine.checkReferenceFrameMatch(firstIntersectionToPack);
      if (secondIntersectionToPack != null)
         pointOnLine.checkReferenceFrameMatch(secondIntersectionToPack);
      int numberOfIntersections = EuclidGeometryTools.intersectionBetweenLine3DAndBoundingBox3D(boundingBoxMinX,
                                                                                                boundingBoxMinY,
                                                                                                boundingBoxMinZ,
                                                                                                boundingBoxMaxX,
                                                                                                boundingBoxMaxY,
                                                                                                boundingBoxMaxZ,
                                                                                                pointOnLine,
                                                                                                lineDirection,
                                                                                                firstIntersectionToPack,
                                                                                                secondIntersectionToPack);

      return numberOfIntersections;
   }

   /**
    * Computes the coordinates of the possible intersections between a line and an axis-aligned
    * bounding box.
    * <p>
    * <a href=
    * "https://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-box-intersection">Useful
    * link</a>.
    * </p>
    * <p>
    * In the case the line and the bounding box do not intersect, this method returns {@code 0} and
    * {@code firstIntersectionToPack} and {@code secondIntersectionToPack} are set to
    * {@link Double#NaN}.
    * </p>
    *
    * @param boundingBoxMinX          the minimum x-coordinate of the bounding box.
    * @param boundingBoxMinY          the minimum y-coordinate of the bounding box.
    * @param boundingBoxMinZ          the minimum z-coordinate of the bounding box.
    * @param boundingBoxMaxX          the maximum x-coordinate of the bounding box.
    * @param boundingBoxMaxY          the maximum y-coordinate of the bounding box.
    * @param boundingBoxMaxZ          the maximum z-coordinate of the bounding box.
    * @param pointOnLine              a point located on the infinitely long line. Not modified.
    * @param lineDirection            the direction of the line. Not modified.
    * @param firstIntersectionToPack  the coordinate of the first intersection. Can be {@code null}.
    *                                 Modified.
    * @param secondIntersectionToPack the coordinate of the second intersection. Can be {@code null}.
    *                                 Modified.
    * @return the number of intersections between the line and the bounding box. It is either equal to
    *         0, 1, or 2.
    * @throws BoundingBoxException            if any of the minimum coordinates of the bounding box is
    *                                         strictly greater than the maximum coordinate of the
    *                                         bounding box on the same axis.
    * @throws ReferenceFrameMismatchException if the read-only arguments are not all expressed in the
    *                                         same reference frame.
    */
   public static int intersectionBetweenLine3DAndBoundingBox3D(double boundingBoxMinX,
                                                               double boundingBoxMinY,
                                                               double boundingBoxMinZ,
                                                               double boundingBoxMaxX,
                                                               double boundingBoxMaxY,
                                                               double boundingBoxMaxZ,
                                                               FramePoint3DReadOnly pointOnLine,
                                                               FrameVector3DReadOnly lineDirection,
                                                               FramePoint3DBasics firstIntersectionToPack,
                                                               FramePoint3DBasics secondIntersectionToPack)
   {
      pointOnLine.checkReferenceFrameMatch(lineDirection);
      int numberOfIntersections = EuclidGeometryTools.intersectionBetweenLine3DAndBoundingBox3D(boundingBoxMinX,
                                                                                                boundingBoxMinY,
                                                                                                boundingBoxMinZ,
                                                                                                boundingBoxMaxX,
                                                                                                boundingBoxMaxY,
                                                                                                boundingBoxMaxZ,
                                                                                                pointOnLine,
                                                                                                lineDirection,
                                                                                                firstIntersectionToPack,
                                                                                                secondIntersectionToPack);

      // Set the correct reference frame.
      if (firstIntersectionToPack != null)
         firstIntersectionToPack.setReferenceFrame(pointOnLine.getReferenceFrame());
      if (secondIntersectionToPack != null)
         secondIntersectionToPack.setReferenceFrame(pointOnLine.getReferenceFrame());

      return numberOfIntersections;
   }

   /**
    * Computes the coordinates of the possible intersections between a line and a cylinder.
    * <p>
    * <a href= "http://mrl.nyu.edu/~dzorin/rend05/lecture2.pdf">Useful link</a>.
    * </p>
    * <p>
    * In the case the line and the cylinder do not intersect, this method returns {@code 0} and
    * {@code firstIntersectionToPack} and {@code secondIntersectionToPack} are set {@link Double#NaN}.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>if either {@code cylinderLength == 0} or {@code cylinderRadius == 0}, this method fails and
    * return {@code 0}.
    * </ul>
    * </p>
    *
    * @param cylinderLength           length of the cylinder.
    * @param cylinderRadius           radius of the cylinder.
    * @param cylinderPosition         the center of the cylinder.
    * @param cylinderAxis             the cylinder's axis.
    * @param firstPointOnLine         a first point located on the infinitely long line. Not modified.
    * @param secondPointOnLine        a second point located on the infinitely long line. Not modified.
    * @param firstIntersectionToPack  the coordinate of the first intersection. Can be {@code null}.
    *                                 Modified.
    * @param secondIntersectionToPack the coordinate of the second intersection. Can be {@code null}.
    *                                 Modified.
    * @return the number of intersections between the line and the cylinder. It is either equal to 0,
    *         1, or 2.
    * @throws IllegalArgumentException        if either {@code cylinderLength < 0} or
    *                                         {@code cylinderRadius < 0}.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static int intersectionBetweenLine3DAndCylinder3D(double cylinderLength,
                                                            double cylinderRadius,
                                                            FramePoint3DReadOnly cylinderPosition,
                                                            FrameVector3DReadOnly cylinderAxis,
                                                            FramePoint3DReadOnly firstPointOnLine,
                                                            FramePoint3DReadOnly secondPointOnLine,
                                                            FixedFramePoint3DBasics firstIntersectionToPack,
                                                            FixedFramePoint3DBasics secondIntersectionToPack)
   {
      cylinderPosition.checkReferenceFrameMatch(cylinderAxis, firstPointOnLine, secondPointOnLine);

      if (firstIntersectionToPack != null)
         firstPointOnLine.checkReferenceFrameMatch(firstIntersectionToPack);
      if (secondIntersectionToPack != null)
         firstPointOnLine.checkReferenceFrameMatch(secondIntersectionToPack);
      int numberOfIntersections = EuclidGeometryTools.intersectionBetweenLine3DAndCylinder3D(cylinderLength,
                                                                                             cylinderRadius,
                                                                                             cylinderPosition,
                                                                                             cylinderAxis,
                                                                                             firstPointOnLine,
                                                                                             secondPointOnLine,
                                                                                             firstIntersectionToPack,
                                                                                             secondIntersectionToPack);

      return numberOfIntersections;
   }

   /**
    * Computes the coordinates of the possible intersections between a line and a cylinder.
    * <p>
    * <a href= "http://mrl.nyu.edu/~dzorin/rend05/lecture2.pdf">Useful link</a>.
    * </p>
    * <p>
    * In the case the line and the cylinder do not intersect, this method returns {@code 0} and
    * {@code firstIntersectionToPack} and {@code secondIntersectionToPack} are set {@link Double#NaN}.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>if either {@code cylinderLength == 0} or {@code cylinderRadius == 0}, this method fails and
    * return {@code 0}.
    * </ul>
    * </p>
    *
    * @param cylinderLength           length of the cylinder.
    * @param cylinderRadius           radius of the cylinder.
    * @param cylinderPosition         the center of the cylinder.
    * @param cylinderAxis             the cylinder's axis.
    * @param firstPointOnLine         a first point located on the infinitely long line. Not modified.
    * @param secondPointOnLine        a second point located on the infinitely long line. Not modified.
    * @param firstIntersectionToPack  the coordinate of the first intersection. Can be {@code null}.
    *                                 Modified.
    * @param secondIntersectionToPack the coordinate of the second intersection. Can be {@code null}.
    *                                 Modified.
    * @return the number of intersections between the line and the cylinder. It is either equal to 0,
    *         1, or 2.
    * @throws IllegalArgumentException        if either {@code cylinderLength < 0} or
    *                                         {@code cylinderRadius < 0}.
    * @throws ReferenceFrameMismatchException if the read-only arguments are not all expressed in the
    *                                         same reference frame.
    */
   public static int intersectionBetweenLine3DAndCylinder3D(double cylinderLength,
                                                            double cylinderRadius,
                                                            FramePoint3DReadOnly cylinderPosition,
                                                            FrameVector3DReadOnly cylinderAxis,
                                                            FramePoint3DReadOnly firstPointOnLine,
                                                            FramePoint3DReadOnly secondPointOnLine,
                                                            FramePoint3DBasics firstIntersectionToPack,
                                                            FramePoint3DBasics secondIntersectionToPack)
   {
      cylinderPosition.checkReferenceFrameMatch(cylinderAxis, firstPointOnLine, secondPointOnLine);
      int numberOfIntersections = EuclidGeometryTools.intersectionBetweenLine3DAndCylinder3D(cylinderLength,
                                                                                             cylinderRadius,
                                                                                             cylinderPosition,
                                                                                             cylinderAxis,
                                                                                             firstPointOnLine,
                                                                                             secondPointOnLine,
                                                                                             firstIntersectionToPack,
                                                                                             secondIntersectionToPack);

      // Set the correct reference frame.
      if (firstIntersectionToPack != null)
         firstIntersectionToPack.setReferenceFrame(firstPointOnLine.getReferenceFrame());
      if (secondIntersectionToPack != null)
         secondIntersectionToPack.setReferenceFrame(firstPointOnLine.getReferenceFrame());

      return numberOfIntersections;
   }

   /**
    * Computes the coordinates of the possible intersections between a line and a cylinder.
    * <p>
    * <a href= "http://mrl.nyu.edu/~dzorin/rend05/lecture2.pdf">Useful link</a>.
    * </p>
    * <p>
    * In the case the line and the cylinder do not intersect, this method returns {@code 0} and
    * {@code firstIntersectionToPack} and {@code secondIntersectionToPack} are set {@link Double#NaN}.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>if either {@code cylinderLength == 0} or {@code cylinderRadius == 0}, this method fails and
    * return {@code 0}.
    * </ul>
    * </p>
    *
    * @param cylinderLength           length of the cylinder.
    * @param cylinderRadius           radius of the cylinder.
    * @param cylinderPosition         the center of the cylinder.
    * @param cylinderAxis             the cylinder's axis.
    * @param pointOnLine              a point located on the infinitely long line. Not modified.
    * @param lineDirection            the direction of the line. Not modified.
    * @param firstIntersectionToPack  the coordinate of the first intersection. Can be {@code null}.
    *                                 Modified.
    * @param secondIntersectionToPack the coordinate of the second intersection. Can be {@code null}.
    *                                 Modified.
    * @return the number of intersections between the line and the cylinder. It is either equal to 0,
    *         1, or 2.
    * @throws IllegalArgumentException        if either {@code cylinderLength < 0} or
    *                                         {@code cylinderRadius < 0}.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static int intersectionBetweenLine3DAndCylinder3D(double cylinderLength,
                                                            double cylinderRadius,
                                                            FramePoint3DReadOnly cylinderPosition,
                                                            FrameVector3DReadOnly cylinderAxis,
                                                            FramePoint3DReadOnly pointOnLine,
                                                            FrameVector3DReadOnly lineDirection,
                                                            FixedFramePoint3DBasics firstIntersectionToPack,
                                                            FixedFramePoint3DBasics secondIntersectionToPack)
   {
      cylinderPosition.checkReferenceFrameMatch(cylinderAxis, pointOnLine, lineDirection);
      if (firstIntersectionToPack != null)
         pointOnLine.checkReferenceFrameMatch(firstIntersectionToPack);
      if (secondIntersectionToPack != null)
         pointOnLine.checkReferenceFrameMatch(secondIntersectionToPack);
      int numberOfIntersections = EuclidGeometryTools.intersectionBetweenLine3DAndCylinder3D(cylinderLength,
                                                                                             cylinderRadius,
                                                                                             cylinderPosition,
                                                                                             cylinderAxis,
                                                                                             pointOnLine,
                                                                                             lineDirection,
                                                                                             firstIntersectionToPack,
                                                                                             secondIntersectionToPack);

      return numberOfIntersections;
   }

   /**
    * Computes the coordinates of the possible intersections between a line and a cylinder.
    * <p>
    * <a href= "http://mrl.nyu.edu/~dzorin/rend05/lecture2.pdf">Useful link</a>.
    * </p>
    * <p>
    * In the case the line and the cylinder do not intersect, this method returns {@code 0} and
    * {@code firstIntersectionToPack} and {@code secondIntersectionToPack} are set {@link Double#NaN}.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>if either {@code cylinderLength == 0} or {@code cylinderRadius == 0}, this method fails and
    * return {@code 0}.
    * </ul>
    * </p>
    *
    * @param cylinderLength           length of the cylinder.
    * @param cylinderRadius           radius of the cylinder.
    * @param cylinderPosition         the center of the cylinder.
    * @param cylinderAxis             the cylinder's axis.
    * @param pointOnLine              a point located on the infinitely long line. Not modified.
    * @param lineDirection            the direction of the line. Not modified.
    * @param firstIntersectionToPack  the coordinate of the first intersection. Can be {@code null}.
    *                                 Modified.
    * @param secondIntersectionToPack the coordinate of the second intersection. Can be {@code null}.
    *                                 Modified.
    * @return the number of intersections between the line and the cylinder. It is either equal to 0,
    *         1, or 2.
    * @throws IllegalArgumentException        if either {@code cylinderLength < 0} or
    *                                         {@code cylinderRadius < 0}.
    * @throws ReferenceFrameMismatchException if the read-only arguments are not all expressed in the
    *                                         same reference frame.
    */
   public static int intersectionBetweenLine3DAndCylinder3D(double cylinderLength,
                                                            double cylinderRadius,
                                                            FramePoint3DReadOnly cylinderPosition,
                                                            FrameVector3DReadOnly cylinderAxis,
                                                            FramePoint3DReadOnly pointOnLine,
                                                            FrameVector3DReadOnly lineDirection,
                                                            FramePoint3DBasics firstIntersectionToPack,
                                                            FramePoint3DBasics secondIntersectionToPack)
   {
      cylinderPosition.checkReferenceFrameMatch(cylinderAxis, pointOnLine, lineDirection);
      int numberOfIntersections = EuclidGeometryTools.intersectionBetweenLine3DAndCylinder3D(cylinderLength,
                                                                                             cylinderRadius,
                                                                                             cylinderPosition,
                                                                                             cylinderAxis,
                                                                                             pointOnLine,
                                                                                             lineDirection,
                                                                                             firstIntersectionToPack,
                                                                                             secondIntersectionToPack);

      // Set the correct reference frame.
      if (firstIntersectionToPack != null)
         firstIntersectionToPack.setReferenceFrame(pointOnLine.getReferenceFrame());
      if (secondIntersectionToPack != null)
         secondIntersectionToPack.setReferenceFrame(pointOnLine.getReferenceFrame());

      return numberOfIntersections;
   }

   /**
    * Computes the coordinates of the possible intersections between a line and an ellipsoid.
    * <p>
    * The ellipsoid is center at (0, 0, 0).
    * </p>
    * <p>
    * In the case the line and the ellipsoid do not intersect, this method returns {@code 0} and
    * {@code firstIntersectionToPack} and {@code secondIntersectionToPack} are set to
    * {@link Double#NaN}.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>if either {@code radiusX}, {@code radiusY}, or {@code radiusZ} is equal to {@code 0}, this
    * method fails and return {@code 0}.
    * </ul>
    * </p>
    *
    * @param radiusX                  radius of the ellipsoid along the x-axis.
    * @param radiusY                  radius of the ellipsoid along the y-axis.
    * @param radiusZ                  radius of the ellipsoid along the z-axis.
    * @param firstPointOnLine         a first point located on the infinitely long line. Not modified.
    * @param secondPointOnLine        a second point located on the infinitely long line. Not modified.
    * @param firstIntersectionToPack  the coordinate of the first intersection. Can be {@code null}.
    *                                 Modified.
    * @param secondIntersectionToPack the coordinate of the second intersection. Can be {@code null}.
    *                                 Modified.
    * @return the number of intersections between the line/line-segment/ray and the ellipsoid. It is
    *         either equal to 0, 1, or 2.
    * @throws IllegalArgumentException        if either {@code radiusX}, {@code radiusY}, or
    *                                         {@code radiusZ} is negative.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static int intersectionBetweenLine3DAndEllipsoid3D(double radiusX,
                                                             double radiusY,
                                                             double radiusZ,
                                                             double positionX,
                                                             double positionY,
                                                             double positionZ,
                                                             FramePoint3DReadOnly firstPointOnLine,
                                                             FramePoint3DReadOnly secondPointOnLine,
                                                             FixedFramePoint3DBasics firstIntersectionToPack,
                                                             FixedFramePoint3DBasics secondIntersectionToPack)
   {
      firstPointOnLine.checkReferenceFrameMatch(secondPointOnLine);
      if (firstIntersectionToPack != null)
         firstPointOnLine.checkReferenceFrameMatch(firstIntersectionToPack);
      if (secondIntersectionToPack != null)
         firstPointOnLine.checkReferenceFrameMatch(secondIntersectionToPack);
      int numberOfIntersections = EuclidGeometryTools.intersectionBetweenLine3DAndEllipsoid3D(radiusX,
                                                                                              radiusY,
                                                                                              radiusZ,
                                                                                              positionX,
                                                                                              positionY,
                                                                                              positionZ,
                                                                                              firstPointOnLine,
                                                                                              secondPointOnLine,
                                                                                              firstIntersectionToPack,
                                                                                              secondIntersectionToPack);

      return numberOfIntersections;
   }

   /**
    * Computes the coordinates of the possible intersections between a line and an ellipsoid.
    * <p>
    * The ellipsoid is center at (0, 0, 0).
    * </p>
    * <p>
    * In the case the line and the ellipsoid do not intersect, this method returns {@code 0} and
    * {@code firstIntersectionToPack} and {@code secondIntersectionToPack} are set to
    * {@link Double#NaN}.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>if either {@code radiusX}, {@code radiusY}, or {@code radiusZ} is equal to {@code 0}, this
    * method fails and return {@code 0}.
    * </ul>
    * </p>
    *
    * @param radiusX                  radius of the ellipsoid along the x-axis.
    * @param radiusY                  radius of the ellipsoid along the y-axis.
    * @param radiusZ                  radius of the ellipsoid along the z-axis.
    * @param firstPointOnLine         a first point located on the infinitely long line. Not modified.
    * @param secondPointOnLine        a second point located on the infinitely long line. Not modified.
    * @param firstIntersectionToPack  the coordinate of the first intersection. Can be {@code null}.
    *                                 Modified.
    * @param secondIntersectionToPack the coordinate of the second intersection. Can be {@code null}.
    *                                 Modified.
    * @return the number of intersections between the line/line-segment/ray and the ellipsoid. It is
    *         either equal to 0, 1, or 2.
    * @throws IllegalArgumentException        if either {@code radiusX}, {@code radiusY}, or
    *                                         {@code radiusZ} is negative.
    * @throws ReferenceFrameMismatchException if the read-only arguments are not all expressed in the
    *                                         same reference frame.
    */
   public static int intersectionBetweenLine3DAndEllipsoid3D(double radiusX,
                                                             double radiusY,
                                                             double radiusZ,
                                                             double positionX,
                                                             double positionY,
                                                             double positionZ,
                                                             FramePoint3DReadOnly firstPointOnLine,
                                                             FramePoint3DReadOnly secondPointOnLine,
                                                             FramePoint3DBasics firstIntersectionToPack,
                                                             FramePoint3DBasics secondIntersectionToPack)
   {
      firstPointOnLine.checkReferenceFrameMatch(secondPointOnLine);
      int numberOfIntersections = EuclidGeometryTools.intersectionBetweenLine3DAndEllipsoid3D(radiusX,
                                                                                              radiusY,
                                                                                              radiusZ,
                                                                                              positionX,
                                                                                              positionY,
                                                                                              positionZ,
                                                                                              firstPointOnLine,
                                                                                              secondPointOnLine,
                                                                                              firstIntersectionToPack,
                                                                                              secondIntersectionToPack);

      // Set the correct reference frame.
      if (firstIntersectionToPack != null)
         firstIntersectionToPack.setReferenceFrame(firstPointOnLine.getReferenceFrame());
      if (secondIntersectionToPack != null)
         secondIntersectionToPack.setReferenceFrame(firstPointOnLine.getReferenceFrame());

      return numberOfIntersections;
   }

   /**
    * Computes the coordinates of the possible intersections between a line and an ellipsoid.
    * <p>
    * The ellipsoid is center at (0, 0, 0).
    * </p>
    * <p>
    * In the case the line and the ellipsoid do not intersect, this method returns {@code 0} and
    * {@code firstIntersectionToPack} and {@code secondIntersectionToPack} are set to
    * {@link Double#NaN}.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>if either {@code radiusX}, {@code radiusY}, or {@code radiusZ} is equal to {@code 0}, this
    * method fails and return {@code 0}.
    * </ul>
    * </p>
    *
    * @param radiusX                  radius of the ellipsoid along the x-axis.
    * @param radiusY                  radius of the ellipsoid along the y-axis.
    * @param radiusZ                  radius of the ellipsoid along the z-axis.
    * @param pointOnLine              a point located on the infinitely long line. Not modified.
    * @param lineDirection            the direction of the line. Not modified.
    * @param firstIntersectionToPack  the coordinate of the first intersection. Can be {@code null}.
    *                                 Modified.
    * @param secondIntersectionToPack the coordinate of the second intersection. Can be {@code null}.
    *                                 Modified.
    * @return the number of intersections between the line/line-segment/ray and the ellipsoid. It is
    *         either equal to 0, 1, or 2.
    * @throws IllegalArgumentException        if either {@code radiusX}, {@code radiusY}, or
    *                                         {@code radiusZ} is negative.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static int intersectionBetweenLine3DAndEllipsoid3D(double radiusX,
                                                             double radiusY,
                                                             double radiusZ,
                                                             double positionX,
                                                             double positionY,
                                                             double positionZ,
                                                             FramePoint3DReadOnly pointOnLine,
                                                             FrameVector3DReadOnly lineDirection,
                                                             FixedFramePoint3DBasics firstIntersectionToPack,
                                                             FixedFramePoint3DBasics secondIntersectionToPack)
   {
      pointOnLine.checkReferenceFrameMatch(lineDirection);
      if (firstIntersectionToPack != null)
         pointOnLine.checkReferenceFrameMatch(firstIntersectionToPack);
      if (secondIntersectionToPack != null)
         pointOnLine.checkReferenceFrameMatch(secondIntersectionToPack);
      int numberOfIntersections = EuclidGeometryTools.intersectionBetweenLine3DAndEllipsoid3D(radiusX,
                                                                                              radiusY,
                                                                                              radiusZ,
                                                                                              positionX,
                                                                                              positionY,
                                                                                              positionZ,
                                                                                              pointOnLine,
                                                                                              lineDirection,
                                                                                              firstIntersectionToPack,
                                                                                              secondIntersectionToPack);

      return numberOfIntersections;
   }

   /**
    * Computes the coordinates of the possible intersections between a line and an ellipsoid.
    * <p>
    * The ellipsoid is center at (0, 0, 0).
    * </p>
    * <p>
    * In the case the line and the ellipsoid do not intersect, this method returns {@code 0} and
    * {@code firstIntersectionToPack} and {@code secondIntersectionToPack} are set to
    * {@link Double#NaN}.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>if either {@code radiusX}, {@code radiusY}, or {@code radiusZ} is equal to {@code 0}, this
    * method fails and return {@code 0}.
    * </ul>
    * </p>
    *
    * @param radiusX                  radius of the ellipsoid along the x-axis.
    * @param radiusY                  radius of the ellipsoid along the y-axis.
    * @param radiusZ                  radius of the ellipsoid along the z-axis.
    * @param pointOnLine              a point located on the infinitely long line. Not modified.
    * @param lineDirection            the direction of the line. Not modified.
    * @param firstIntersectionToPack  the coordinate of the first intersection. Can be {@code null}.
    *                                 Modified.
    * @param secondIntersectionToPack the coordinate of the second intersection. Can be {@code null}.
    *                                 Modified.
    * @return the number of intersections between the line/line-segment/ray and the ellipsoid. It is
    *         either equal to 0, 1, or 2.
    * @throws IllegalArgumentException        if either {@code radiusX}, {@code radiusY}, or
    *                                         {@code radiusZ} is negative.
    * @throws ReferenceFrameMismatchException if the read-only arguments are not all expressed in the
    *                                         same reference frame.
    */
   public static int intersectionBetweenLine3DAndEllipsoid3D(double radiusX,
                                                             double radiusY,
                                                             double radiusZ,
                                                             double positionX,
                                                             double positionY,
                                                             double positionZ,
                                                             FramePoint3DReadOnly pointOnLine,
                                                             FrameVector3DReadOnly lineDirection,
                                                             FramePoint3DBasics firstIntersectionToPack,
                                                             FramePoint3DBasics secondIntersectionToPack)
   {
      pointOnLine.checkReferenceFrameMatch(lineDirection);
      int numberOfIntersections = EuclidGeometryTools.intersectionBetweenLine3DAndEllipsoid3D(radiusX,
                                                                                              radiusY,
                                                                                              radiusZ,
                                                                                              positionX,
                                                                                              positionY,
                                                                                              positionZ,
                                                                                              pointOnLine,
                                                                                              lineDirection,
                                                                                              firstIntersectionToPack,
                                                                                              secondIntersectionToPack);

      // Set the correct reference frame.
      if (firstIntersectionToPack != null)
         firstIntersectionToPack.setReferenceFrame(pointOnLine.getReferenceFrame());
      if (secondIntersectionToPack != null)
         secondIntersectionToPack.setReferenceFrame(pointOnLine.getReferenceFrame());

      return numberOfIntersections;
   }

   /**
    * Computes the coordinates of the intersection between a plane and an infinitely long line.
    * <a href="https://en.wikipedia.org/wiki/Line%E2%80%93plane_intersection"> Useful link </a>.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>If the line is parallel to the plane, this methods fails and returns {@code null}.
    * </ul>
    * </p>
    *
    * @param pointOnPlane  a point located on the plane. Not modified.
    * @param planeNormal   the normal of the plane. Not modified.
    * @param pointOnLine   a point located on the line. Not modified.
    * @param lineDirection the direction of the line. Not modified.
    * @return the coordinates of the intersection, or {@code null} if the line is parallel to the
    *         plane.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static FramePoint3D intersectionBetweenLine3DAndPlane3D(FramePoint3DReadOnly pointOnPlane,
                                                                  FrameVector3DReadOnly planeNormal,
                                                                  FramePoint3DReadOnly pointOnLine,
                                                                  FrameVector3DReadOnly lineDirection)
   {
      pointOnPlane.checkReferenceFrameMatch(planeNormal, pointOnLine, lineDirection);

      FramePoint3D intersection = new FramePoint3D(pointOnLine.getReferenceFrame());
      boolean success = EuclidGeometryTools.intersectionBetweenLine3DAndPlane3D(pointOnPlane, planeNormal, pointOnLine, lineDirection, intersection);
      if (success)
         return intersection;
      else
         return null;
   }

   /**
    * Computes the coordinates of the intersection between a plane and an infinitely long line.
    * <a href="https://en.wikipedia.org/wiki/Line%E2%80%93plane_intersection"> Useful link </a>.
    * <p>
    * Edge cases:
    * <ul>
    * <li>If the line is parallel to the plane, this methods fails and returns {@code false}.
    * </ul>
    * </p>
    *
    * @param pointOnPlane       a point located on the plane. Not modified.
    * @param planeNormal        the normal of the plane. Not modified.
    * @param pointOnLine        a point located on the line. Not modified.
    * @param lineDirection      the direction of the line. Not modified.
    * @param intersectionToPack point in which the coordinates of the intersection are stored.
    * @return {@code true} if the method succeeds, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static boolean intersectionBetweenLine3DAndPlane3D(FramePoint3DReadOnly pointOnPlane,
                                                             FrameVector3DReadOnly planeNormal,
                                                             FramePoint3DReadOnly pointOnLine,
                                                             FrameVector3DReadOnly lineDirection,
                                                             FixedFramePoint3DBasics intersectionToPack)
   {
      pointOnPlane.checkReferenceFrameMatch(planeNormal, pointOnLine, lineDirection);
      if (intersectionToPack != null)
         pointOnPlane.checkReferenceFrameMatch(intersectionToPack);
      return EuclidGeometryTools.intersectionBetweenLine3DAndPlane3D(pointOnPlane, planeNormal, pointOnLine, lineDirection, intersectionToPack);
   }

   /**
    * Computes the coordinates of the intersection between a plane and an infinitely long line.
    * <a href="https://en.wikipedia.org/wiki/Line%E2%80%93plane_intersection"> Useful link </a>.
    * <p>
    * Edge cases:
    * <ul>
    * <li>If the line is parallel to the plane, this methods fails and returns {@code false}.
    * </ul>
    * </p>
    *
    * @param pointOnPlane       a point located on the plane. Not modified.
    * @param planeNormal        the normal of the plane. Not modified.
    * @param pointOnLine        a point located on the line. Not modified.
    * @param lineDirection      the direction of the line. Not modified.
    * @param intersectionToPack point in which the coordinates of the intersection are stored.
    * @return {@code true} if the method succeeds, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if the read-only arguments are not all expressed in the
    *                                         same reference frame.
    */
   public static boolean intersectionBetweenLine3DAndPlane3D(FramePoint3DReadOnly pointOnPlane,
                                                             FrameVector3DReadOnly planeNormal,
                                                             FramePoint3DReadOnly pointOnLine,
                                                             FrameVector3DReadOnly lineDirection,
                                                             FramePoint3DBasics intersectionToPack)
   {
      pointOnPlane.checkReferenceFrameMatch(planeNormal, pointOnLine, lineDirection);
      boolean success = EuclidGeometryTools.intersectionBetweenLine3DAndPlane3D(pointOnPlane, planeNormal, pointOnLine, lineDirection, intersectionToPack);

      if (success && intersectionToPack != null)
         intersectionToPack.setReferenceFrame(pointOnLine.getReferenceFrame());

      return success;
   }

   /**
    * Computes the coordinates of the possible intersections between a line segment and an axis-aligned
    * bounding box.
    * <p>
    * <a href=
    * "https://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-box-intersection">Useful
    * link</a>.
    * </p>
    * <p>
    * Intersection(s) between the line segment and the bounding box can only exist between the
    * endpoints of the line segment.
    * </p>
    * <p>
    * In the case the line segment and the bounding box do not intersect, this method returns {@code 0}
    * and {@code firstIntersectionToPack} and {@code secondIntersectionToPack} are set to
    * {@link Double#NaN}.
    * </p>
    * <p>
    * In the case only one intersection exists between the line segment and the bounding box,
    * {@code firstIntersectionToPack} will contain the coordinate of the intersection and
    * {@code secondIntersectionToPack} will be set to contain only {@link Double#NaN}.
    * </p>
    *
    * @param boundingBoxMin           the minimum coordinate of the bounding box. Not modified.
    * @param boundingBoxMax           the maximum coordinate of the bounding box. Not modified.
    * @param lineSegmentStart         the first endpoint of the line segment. Not modified.
    * @param lineSegmentEnd           the second endpoint of the line segment. Not modified.
    * @param firstIntersectionToPack  the coordinate of the first intersection. Can be {@code null}.
    *                                 Modified.
    * @param secondIntersectionToPack the coordinate of the second intersection. Can be {@code null}.
    *                                 Modified.
    * @return the number of intersections between the line segment and the bounding box. It is either
    *         equal to 0, 1, or 2.
    * @throws BoundingBoxException            if any of the minimum coordinates of the bounding box is
    *                                         strictly greater than the maximum coordinate of the
    *                                         bounding box on the same axis.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static int intersectionBetweenLineSegment2DAndBoundingBox2D(FramePoint2DReadOnly boundingBoxMin,
                                                                      FramePoint2DReadOnly boundingBoxMax,
                                                                      FramePoint2DReadOnly lineSegmentStart,
                                                                      FramePoint2DReadOnly lineSegmentEnd,
                                                                      FixedFramePoint2DBasics firstIntersectionToPack,
                                                                      FixedFramePoint2DBasics secondIntersectionToPack)
   {
      boundingBoxMin.checkReferenceFrameMatch(boundingBoxMax, lineSegmentStart, lineSegmentEnd);
      if (firstIntersectionToPack != null)
         boundingBoxMin.checkReferenceFrameMatch(firstIntersectionToPack);
      if (secondIntersectionToPack != null)
         boundingBoxMin.checkReferenceFrameMatch(secondIntersectionToPack);
      int numberOfIntersections = EuclidGeometryTools.intersectionBetweenLineSegment2DAndBoundingBox2D(boundingBoxMin,
                                                                                                       boundingBoxMax,
                                                                                                       lineSegmentStart,
                                                                                                       lineSegmentEnd,
                                                                                                       firstIntersectionToPack,
                                                                                                       secondIntersectionToPack);

      return numberOfIntersections;
   }

   /**
    * Computes the coordinates of the possible intersections between a line segment and an axis-aligned
    * bounding box.
    * <p>
    * <a href=
    * "https://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-box-intersection">Useful
    * link</a>.
    * </p>
    * <p>
    * Intersection(s) between the line segment and the bounding box can only exist between the
    * endpoints of the line segment.
    * </p>
    * <p>
    * In the case the line segment and the bounding box do not intersect, this method returns {@code 0}
    * and {@code firstIntersectionToPack} and {@code secondIntersectionToPack} are set to
    * {@link Double#NaN}.
    * </p>
    * <p>
    * In the case only one intersection exists between the line segment and the bounding box,
    * {@code firstIntersectionToPack} will contain the coordinate of the intersection and
    * {@code secondIntersectionToPack} will be set to contain only {@link Double#NaN}.
    * </p>
    *
    * @param boundingBoxMin           the minimum coordinate of the bounding box. Not modified.
    * @param boundingBoxMax           the maximum coordinate of the bounding box. Not modified.
    * @param lineSegmentStart         the first endpoint of the line segment. Not modified.
    * @param lineSegmentEnd           the second endpoint of the line segment. Not modified.
    * @param firstIntersectionToPack  the coordinate of the first intersection. Can be {@code null}.
    *                                 Modified.
    * @param secondIntersectionToPack the coordinate of the second intersection. Can be {@code null}.
    *                                 Modified.
    * @return the number of intersections between the line segment and the bounding box. It is either
    *         equal to 0, 1, or 2.
    * @throws BoundingBoxException            if any of the minimum coordinates of the bounding box is
    *                                         strictly greater than the maximum coordinate of the
    *                                         bounding box on the same axis.
    * @throws ReferenceFrameMismatchException if the read-only arguments are not all expressed in the
    *                                         same reference frame.
    */
   public static int intersectionBetweenLineSegment2DAndBoundingBox2D(FramePoint2DReadOnly boundingBoxMin,
                                                                      FramePoint2DReadOnly boundingBoxMax,
                                                                      FramePoint2DReadOnly lineSegmentStart,
                                                                      FramePoint2DReadOnly lineSegmentEnd,
                                                                      FramePoint2DBasics firstIntersectionToPack,
                                                                      FramePoint2DBasics secondIntersectionToPack)
   {
      boundingBoxMin.checkReferenceFrameMatch(boundingBoxMax, lineSegmentStart, lineSegmentEnd);
      int numberOfIntersections = EuclidGeometryTools.intersectionBetweenLineSegment2DAndBoundingBox2D(boundingBoxMin,
                                                                                                       boundingBoxMax,
                                                                                                       lineSegmentStart,
                                                                                                       lineSegmentEnd,
                                                                                                       firstIntersectionToPack,
                                                                                                       secondIntersectionToPack);

      // Set the correct reference frame.
      if (firstIntersectionToPack != null)
         firstIntersectionToPack.setReferenceFrame(lineSegmentStart.getReferenceFrame());
      if (secondIntersectionToPack != null)
         secondIntersectionToPack.setReferenceFrame(lineSegmentStart.getReferenceFrame());

      return numberOfIntersections;
   }

   /**
    * Computes the coordinates of the possible intersections between a line segment and an axis-aligned
    * bounding box.
    * <p>
    * <a href=
    * "https://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-box-intersection">Useful
    * link</a>.
    * </p>
    * <p>
    * Intersection(s) between the line segment and the bounding box can only exist between the
    * endpoints of the line segment.
    * </p>
    * <p>
    * In the case the line segment and the bounding box do not intersect, this method returns {@code 0}
    * and {@code firstIntersectionToPack} and {@code secondIntersectionToPack} are set to
    * {@link Double#NaN}.
    * </p>
    * <p>
    * In the case only one intersection exists between the line segment and the bounding box,
    * {@code firstIntersectionToPack} will contain the coordinate of the intersection and
    * {@code secondIntersectionToPack} will be set to contain only {@link Double#NaN}.
    * </p>
    *
    * @param boundingBoxMin           the minimum coordinate of the bounding box. Not modified.
    * @param boundingBoxMax           the maximum coordinate of the bounding box. Not modified.
    * @param lineSegmentStart         the first endpoint of the line segment. Not modified.
    * @param lineSegmentEnd           the second endpoint of the line segment. Not modified.
    * @param firstIntersectionToPack  the coordinate of the first intersection. Can be {@code null}.
    *                                 Modified.
    * @param secondIntersectionToPack the coordinate of the second intersection. Can be {@code null}.
    *                                 Modified.
    * @return the number of intersections between the line segment and the bounding box. It is either
    *         equal to 0, 1, or 2.
    * @throws BoundingBoxException            if any of the minimum coordinates of the bounding box is
    *                                         strictly greater than the maximum coordinate of the
    *                                         bounding box on the same axis.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static int intersectionBetweenLineSegment3DAndBoundingBox3D(FramePoint3DReadOnly boundingBoxMin,
                                                                      FramePoint3DReadOnly boundingBoxMax,
                                                                      FramePoint3DReadOnly lineSegmentStart,
                                                                      FramePoint3DReadOnly lineSegmentEnd,
                                                                      FixedFramePoint3DBasics firstIntersectionToPack,
                                                                      FixedFramePoint3DBasics secondIntersectionToPack)
   {
      boundingBoxMin.checkReferenceFrameMatch(boundingBoxMax, lineSegmentStart, lineSegmentEnd);
      if (firstIntersectionToPack != null)
         boundingBoxMin.checkReferenceFrameMatch(firstIntersectionToPack);
      if (secondIntersectionToPack != null)
         boundingBoxMin.checkReferenceFrameMatch(secondIntersectionToPack);
      int numberOfIntersections = EuclidGeometryTools.intersectionBetweenLineSegment3DAndBoundingBox3D(boundingBoxMin,
                                                                                                       boundingBoxMax,
                                                                                                       lineSegmentStart,
                                                                                                       lineSegmentEnd,
                                                                                                       firstIntersectionToPack,
                                                                                                       secondIntersectionToPack);

      return numberOfIntersections;
   }

   /**
    * Computes the coordinates of the possible intersections between a line segment and an axis-aligned
    * bounding box.
    * <p>
    * <a href=
    * "https://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-box-intersection">Useful
    * link</a>.
    * </p>
    * <p>
    * Intersection(s) between the line segment and the bounding box can only exist between the
    * endpoints of the line segment.
    * </p>
    * <p>
    * In the case the line segment and the bounding box do not intersect, this method returns {@code 0}
    * and {@code firstIntersectionToPack} and {@code secondIntersectionToPack} are set to
    * {@link Double#NaN}.
    * </p>
    * <p>
    * In the case only one intersection exists between the line segment and the bounding box,
    * {@code firstIntersectionToPack} will contain the coordinate of the intersection and
    * {@code secondIntersectionToPack} will be set to contain only {@link Double#NaN}.
    * </p>
    *
    * @param boundingBoxMin           the minimum coordinate of the bounding box. Not modified.
    * @param boundingBoxMax           the maximum coordinate of the bounding box. Not modified.
    * @param lineSegmentStart         the first endpoint of the line segment. Not modified.
    * @param lineSegmentEnd           the second endpoint of the line segment. Not modified.
    * @param firstIntersectionToPack  the coordinate of the first intersection. Can be {@code null}.
    *                                 Modified.
    * @param secondIntersectionToPack the coordinate of the second intersection. Can be {@code null}.
    *                                 Modified.
    * @return the number of intersections between the line segment and the bounding box. It is either
    *         equal to 0, 1, or 2.
    * @throws BoundingBoxException            if any of the minimum coordinates of the bounding box is
    *                                         strictly greater than the maximum coordinate of the
    *                                         bounding box on the same axis.
    * @throws ReferenceFrameMismatchException if the read-only arguments are not all expressed in the
    *                                         same reference frame.
    */
   public static int intersectionBetweenLineSegment3DAndBoundingBox3D(FramePoint3DReadOnly boundingBoxMin,
                                                                      FramePoint3DReadOnly boundingBoxMax,
                                                                      FramePoint3DReadOnly lineSegmentStart,
                                                                      FramePoint3DReadOnly lineSegmentEnd,
                                                                      FramePoint3DBasics firstIntersectionToPack,
                                                                      FramePoint3DBasics secondIntersectionToPack)
   {
      boundingBoxMin.checkReferenceFrameMatch(boundingBoxMax, lineSegmentStart, lineSegmentEnd);
      int numberOfIntersections = EuclidGeometryTools.intersectionBetweenLineSegment3DAndBoundingBox3D(boundingBoxMin,
                                                                                                       boundingBoxMax,
                                                                                                       lineSegmentStart,
                                                                                                       lineSegmentEnd,
                                                                                                       firstIntersectionToPack,
                                                                                                       secondIntersectionToPack);

      // Set the correct reference frame.
      if (firstIntersectionToPack != null)
         firstIntersectionToPack.setReferenceFrame(lineSegmentStart.getReferenceFrame());
      if (secondIntersectionToPack != null)
         secondIntersectionToPack.setReferenceFrame(lineSegmentStart.getReferenceFrame());

      return numberOfIntersections;
   }

   /**
    * Computes the coordinates of the possible intersections between a line segment and a cylinder.
    * <p>
    * <a href= "http://mrl.nyu.edu/~dzorin/rend05/lecture2.pdf">Useful link</a>.
    * </p>
    * <p>
    * In the case the line segment and the cylinder do not intersect, this method returns {@code 0} and
    * {@code firstIntersectionToPack} and {@code secondIntersectionToPack} are set {@link Double#NaN}.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>if either {@code cylinderLength == 0} or {@code cylinderRadius == 0}, this method fails and
    * return {@code 0}.
    * </ul>
    * </p>
    *
    * @param cylinderLength           length of the cylinder.
    * @param cylinderRadius           radius of the cylinder.
    * @param cylinderPosition         the center of the cylinder.
    * @param cylinderAxis             the cylinder's axis.
    * @param lineSegmentStart         the first endpoint of the line segment. Not modified.
    * @param lineSegmentEnd           the second endpoint of the line segment. Not modified.
    * @param firstIntersectionToPack  the coordinate of the first intersection. Can be {@code null}.
    *                                 Modified.
    * @param secondIntersectionToPack the coordinate of the second intersection. Can be {@code null}.
    *                                 Modified.
    * @return the number of intersections between the line segment and the cylinder. It is either equal
    *         to 0, 1, or 2.
    * @throws IllegalArgumentException        if either {@code cylinderLength < 0} or
    *                                         {@code cylinderRadius < 0}.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static int intersectionBetweenLineSegment3DAndCylinder3D(double cylinderLength,
                                                                   double cylinderRadius,
                                                                   FramePoint3DReadOnly cylinderPosition,
                                                                   FrameVector3DReadOnly cylinderAxis,
                                                                   FramePoint3DReadOnly lineSegmentStart,
                                                                   FramePoint3DReadOnly lineSegmentEnd,
                                                                   FixedFramePoint3DBasics firstIntersectionToPack,
                                                                   FixedFramePoint3DBasics secondIntersectionToPack)
   {
      cylinderPosition.checkReferenceFrameMatch(cylinderAxis, lineSegmentStart, lineSegmentEnd);
      if (firstIntersectionToPack != null)
         lineSegmentStart.checkReferenceFrameMatch(firstIntersectionToPack);
      if (secondIntersectionToPack != null)
         lineSegmentStart.checkReferenceFrameMatch(secondIntersectionToPack);
      int numberOfIntersections = EuclidGeometryTools.intersectionBetweenLineSegment3DAndCylinder3D(cylinderLength,
                                                                                                    cylinderRadius,
                                                                                                    cylinderPosition,
                                                                                                    cylinderAxis,
                                                                                                    lineSegmentStart,
                                                                                                    lineSegmentEnd,
                                                                                                    firstIntersectionToPack,
                                                                                                    secondIntersectionToPack);

      return numberOfIntersections;
   }

   /**
    * Computes the coordinates of the possible intersections between a line segment and a cylinder.
    * <p>
    * <a href= "http://mrl.nyu.edu/~dzorin/rend05/lecture2.pdf">Useful link</a>.
    * </p>
    * <p>
    * In the case the line segment and the cylinder do not intersect, this method returns {@code 0} and
    * {@code firstIntersectionToPack} and {@code secondIntersectionToPack} are set {@link Double#NaN}.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>if either {@code cylinderLength == 0} or {@code cylinderRadius == 0}, this method fails and
    * return {@code 0}.
    * </ul>
    * </p>
    *
    * @param cylinderLength           length of the cylinder.
    * @param cylinderRadius           radius of the cylinder.
    * @param cylinderPosition         the center of the cylinder.
    * @param cylinderAxis             the cylinder's axis.
    * @param lineSegmentStart         the first endpoint of the line segment. Not modified.
    * @param lineSegmentEnd           the second endpoint of the line segment. Not modified.
    * @param firstIntersectionToPack  the coordinate of the first intersection. Can be {@code null}.
    *                                 Modified.
    * @param secondIntersectionToPack the coordinate of the second intersection. Can be {@code null}.
    *                                 Modified.
    * @return the number of intersections between the line segment and the cylinder. It is either equal
    *         to 0, 1, or 2.
    * @throws IllegalArgumentException        if either {@code cylinderLength < 0} or
    *                                         {@code cylinderRadius < 0}.
    * @throws ReferenceFrameMismatchException if the read-only arguments are not all expressed in the
    *                                         same reference frame.
    */
   public static int intersectionBetweenLineSegment3DAndCylinder3D(double cylinderLength,
                                                                   double cylinderRadius,
                                                                   FramePoint3DReadOnly cylinderPosition,
                                                                   FrameVector3DReadOnly cylinderAxis,
                                                                   FramePoint3DReadOnly lineSegmentStart,
                                                                   FramePoint3DReadOnly lineSegmentEnd,
                                                                   FramePoint3DBasics firstIntersectionToPack,
                                                                   FramePoint3DBasics secondIntersectionToPack)
   {
      cylinderPosition.checkReferenceFrameMatch(cylinderAxis, lineSegmentStart, lineSegmentEnd);
      int numberOfIntersections = EuclidGeometryTools.intersectionBetweenLineSegment3DAndCylinder3D(cylinderLength,
                                                                                                    cylinderRadius,
                                                                                                    cylinderPosition,
                                                                                                    cylinderAxis,
                                                                                                    lineSegmentStart,
                                                                                                    lineSegmentEnd,
                                                                                                    firstIntersectionToPack,
                                                                                                    secondIntersectionToPack);

      // Set the correct reference frame.
      if (firstIntersectionToPack != null)
         firstIntersectionToPack.setReferenceFrame(lineSegmentStart.getReferenceFrame());
      if (secondIntersectionToPack != null)
         secondIntersectionToPack.setReferenceFrame(lineSegmentStart.getReferenceFrame());

      return numberOfIntersections;
   }

   /**
    * Computes the coordinates of the possible intersections between a line segment and an ellipsoid.
    * <p>
    * The ellipsoid is center at (0, 0, 0).
    * </p>
    * <p>
    * In the case the line segment and the ellipsoid do not intersect, this method returns {@code 0}
    * and {@code firstIntersectionToPack} and {@code secondIntersectionToPack} are set to
    * {@link Double#NaN}.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>if either {@code radiusX}, {@code radiusY}, or {@code radiusZ} is equal to {@code 0}, this
    * method fails and return {@code 0}.
    * </ul>
    * </p>
    *
    * @param radiusX                  radius of the ellipsoid along the x-axis.
    * @param radiusY                  radius of the ellipsoid along the y-axis.
    * @param radiusZ                  radius of the ellipsoid along the z-axis.
    * @param lineSegmentStart         the first endpoint of the line segment. Not modified.
    * @param lineSegmentEnd           the second endpoint of the line segment. Not modified.
    * @param firstIntersectionToPack  the coordinate of the first intersection. Can be {@code null}.
    *                                 Modified.
    * @param secondIntersectionToPack the coordinate of the second intersection. Can be {@code null}.
    *                                 Modified.
    * @return the number of intersections between the line/line-segment/ray and the ellipsoid. It is
    *         either equal to 0, 1, or 2.
    * @throws IllegalArgumentException        if either {@code radiusX}, {@code radiusY}, or
    *                                         {@code radiusZ} is negative.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static int intersectionBetweenLineSegment3DAndEllipsoid3D(double radiusX,
                                                                    double radiusY,
                                                                    double radiusZ,
                                                                    double positionX,
                                                                    double positionY,
                                                                    double positionZ,
                                                                    FramePoint3DReadOnly lineSegmentStart,
                                                                    FramePoint3DReadOnly lineSegmentEnd,
                                                                    FixedFramePoint3DBasics firstIntersectionToPack,
                                                                    FixedFramePoint3DBasics secondIntersectionToPack)
   {
      lineSegmentStart.checkReferenceFrameMatch(lineSegmentEnd);
      if (firstIntersectionToPack != null)
         lineSegmentStart.checkReferenceFrameMatch(firstIntersectionToPack);
      if (secondIntersectionToPack != null)
         lineSegmentStart.checkReferenceFrameMatch(secondIntersectionToPack);
      int numberOfIntersections = EuclidGeometryTools.intersectionBetweenLineSegment3DAndEllipsoid3D(radiusX,
                                                                                                     radiusY,
                                                                                                     radiusZ,
                                                                                                     positionX,
                                                                                                     positionY,
                                                                                                     positionZ,
                                                                                                     lineSegmentStart,
                                                                                                     lineSegmentEnd,
                                                                                                     firstIntersectionToPack,
                                                                                                     secondIntersectionToPack);

      return numberOfIntersections;
   }

   /**
    * Computes the coordinates of the possible intersections between a line segment and an ellipsoid.
    * <p>
    * The ellipsoid is center at (0, 0, 0).
    * </p>
    * <p>
    * In the case the line segment and the ellipsoid do not intersect, this method returns {@code 0}
    * and {@code firstIntersectionToPack} and {@code secondIntersectionToPack} are set to
    * {@link Double#NaN}.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>if either {@code radiusX}, {@code radiusY}, or {@code radiusZ} is equal to {@code 0}, this
    * method fails and return {@code 0}.
    * </ul>
    * </p>
    *
    * @param radiusX                  radius of the ellipsoid along the x-axis.
    * @param radiusY                  radius of the ellipsoid along the y-axis.
    * @param radiusZ                  radius of the ellipsoid along the z-axis.
    * @param lineSegmentStart         the first endpoint of the line segment. Not modified.
    * @param lineSegmentEnd           the second endpoint of the line segment. Not modified.
    * @param firstIntersectionToPack  the coordinate of the first intersection. Can be {@code null}.
    *                                 Modified.
    * @param secondIntersectionToPack the coordinate of the second intersection. Can be {@code null}.
    *                                 Modified.
    * @return the number of intersections between the line/line-segment/ray and the ellipsoid. It is
    *         either equal to 0, 1, or 2.
    * @throws IllegalArgumentException        if either {@code radiusX}, {@code radiusY}, or
    *                                         {@code radiusZ} is negative.
    * @throws ReferenceFrameMismatchException if the read-only arguments are not all expressed in the
    *                                         same reference frame.
    */
   public static int intersectionBetweenLineSegment3DAndEllipsoid3D(double radiusX,
                                                                    double radiusY,
                                                                    double radiusZ,
                                                                    double positionX,
                                                                    double positionY,
                                                                    double positionZ,
                                                                    FramePoint3DReadOnly lineSegmentStart,
                                                                    FramePoint3DReadOnly lineSegmentEnd,
                                                                    FramePoint3DBasics firstIntersectionToPack,
                                                                    FramePoint3DBasics secondIntersectionToPack)
   {
      lineSegmentStart.checkReferenceFrameMatch(lineSegmentEnd);
      int numberOfIntersections = EuclidGeometryTools.intersectionBetweenLineSegment3DAndEllipsoid3D(radiusX,
                                                                                                     radiusY,
                                                                                                     radiusZ,
                                                                                                     positionX,
                                                                                                     positionY,
                                                                                                     positionZ,
                                                                                                     lineSegmentStart,
                                                                                                     lineSegmentEnd,
                                                                                                     firstIntersectionToPack,
                                                                                                     secondIntersectionToPack);

      // Set the correct reference frame.
      if (firstIntersectionToPack != null)
         firstIntersectionToPack.setReferenceFrame(lineSegmentStart.getReferenceFrame());
      if (secondIntersectionToPack != null)
         secondIntersectionToPack.setReferenceFrame(lineSegmentStart.getReferenceFrame());

      return numberOfIntersections;
   }

   /**
    * Computes the coordinates of the intersection between a plane and a finite length line segment.
    * <p>
    * This method returns {@code null} for the following cases:
    * <ul>
    * <li>the line segment is parallel to the plane,
    * <li>the line segment endpoints are on one side of the plane,
    * <li>the line segment length is equal to zero ({@code lineSegmentStart == lineSegmentEnd}),
    * <li>one of the line segment endpoints lies on the plane.
    * </ul>
    * </p>
    * Once the existence of an intersection is verified, this method calls
    * {@link EuclidGeometryTools#intersectionBetweenLine3DAndPlane3D(Point3DReadOnly, Vector3DReadOnly, Point3DReadOnly, Vector3DReadOnly)}
    * to perform the actual computation.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param pointOnPlane     a point located on the plane. Not modified.
    * @param planeNormal      the normal of the plane. Not modified.
    * @param lineSegmentStart first endpoint of the line segment. Not modified.
    * @param lineSegmentEnd   second endpoint of the line segment. Not modified.
    * @return the intersection, or {@code null} if there is no intersection.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static FramePoint3D intersectionBetweenLineSegment3DAndPlane3D(FramePoint3DReadOnly pointOnPlane,
                                                                         FrameVector3DReadOnly planeNormal,
                                                                         FramePoint3DReadOnly lineSegmentStart,
                                                                         FramePoint3DReadOnly lineSegmentEnd)
   {
      pointOnPlane.checkReferenceFrameMatch(planeNormal, lineSegmentStart, lineSegmentEnd);

      Point3D intersection = EuclidGeometryTools.intersectionBetweenLineSegment3DAndPlane3D(pointOnPlane, planeNormal, lineSegmentStart, lineSegmentEnd);
      if (intersection != null)
      {
         return new FramePoint3D(pointOnPlane.getReferenceFrame(), intersection);
      }
      else
      {
         return null;
      }
   }

   /**
    * Computes the coordinates of the possible intersections between a ray and an axis-aligned bounding
    * box.
    * <p>
    * <a href=
    * "https://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-box-intersection">Useful
    * link</a>.
    * </p>
    * <p>
    * Intersection(s) between the ray and the bounding box cannot exist before the origin of the ray.
    * </p>
    * </p>
    * In the case the ray and the bounding box do not intersect, this method returns {@code 0} and
    * {@code firstIntersectionToPack} and {@code secondIntersectionToPack} are set to
    * {@link Double#NaN}.
    * </p>
    * <p>
    * In the case only one intersection exists between the ray and the bounding box,
    * {@code firstIntersectionToPack} will contain the coordinate of the intersection and
    * {@code secondIntersectionToPack} will be set to contain only {@link Double#NaN}.
    * </p>
    *
    * @param boundingBoxMin           the minimum coordinate of the bounding box. Not modified.
    * @param boundingBoxMax           the maximum coordinate of the bounding box. Not modified.
    * @param rayOrigin                the coordinate of the ray origin. Not modified.
    * @param rayDirection             the direction of the ray. Not modified.
    * @param firstIntersectionToPack  the coordinate of the first intersection. Can be {@code null}.
    *                                 Modified.
    * @param secondIntersectionToPack the coordinate of the second intersection. Can be {@code null}.
    *                                 Modified.
    * @return the number of intersections between the ray and the bounding box. It is either equal to
    *         0, 1, or 2.
    * @throws BoundingBoxException            if any of the minimum coordinates of the bounding box is
    *                                         strictly greater than the maximum coordinate of the
    *                                         bounding box on the same axis.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static int intersectionBetweenRay2DAndBoundingBox2D(FramePoint2DReadOnly boundingBoxMin,
                                                              FramePoint2DReadOnly boundingBoxMax,
                                                              FramePoint2DReadOnly rayOrigin,
                                                              FrameVector2DReadOnly rayDirection,
                                                              FixedFramePoint2DBasics firstIntersectionToPack,
                                                              FixedFramePoint2DBasics secondIntersectionToPack)
   {
      boundingBoxMin.checkReferenceFrameMatch(boundingBoxMax, rayOrigin, rayDirection);
      if (firstIntersectionToPack != null)
         boundingBoxMin.checkReferenceFrameMatch(firstIntersectionToPack);
      if (secondIntersectionToPack != null)
         boundingBoxMin.checkReferenceFrameMatch(secondIntersectionToPack);
      int numberOfIntersections = EuclidGeometryTools.intersectionBetweenRay2DAndBoundingBox2D(boundingBoxMin,
                                                                                               boundingBoxMax,
                                                                                               rayOrigin,
                                                                                               rayDirection,
                                                                                               firstIntersectionToPack,
                                                                                               secondIntersectionToPack);

      return numberOfIntersections;
   }

   /**
    * Computes the coordinates of the possible intersections between a ray and an axis-aligned bounding
    * box.
    * <p>
    * <a href=
    * "https://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-box-intersection">Useful
    * link</a>.
    * </p>
    * <p>
    * Intersection(s) between the ray and the bounding box cannot exist before the origin of the ray.
    * </p>
    * </p>
    * In the case the ray and the bounding box do not intersect, this method returns {@code 0} and
    * {@code firstIntersectionToPack} and {@code secondIntersectionToPack} are set to
    * {@link Double#NaN}.
    * </p>
    * <p>
    * In the case only one intersection exists between the ray and the bounding box,
    * {@code firstIntersectionToPack} will contain the coordinate of the intersection and
    * {@code secondIntersectionToPack} will be set to contain only {@link Double#NaN}.
    * </p>
    *
    * @param boundingBoxMin           the minimum coordinate of the bounding box. Not modified.
    * @param boundingBoxMax           the maximum coordinate of the bounding box. Not modified.
    * @param rayOrigin                the coordinate of the ray origin. Not modified.
    * @param rayDirection             the direction of the ray. Not modified.
    * @param firstIntersectionToPack  the coordinate of the first intersection. Can be {@code null}.
    *                                 Modified.
    * @param secondIntersectionToPack the coordinate of the second intersection. Can be {@code null}.
    *                                 Modified.
    * @return the number of intersections between the ray and the bounding box. It is either equal to
    *         0, 1, or 2.
    * @throws BoundingBoxException            if any of the minimum coordinates of the bounding box is
    *                                         strictly greater than the maximum coordinate of the
    *                                         bounding box on the same axis.
    * @throws ReferenceFrameMismatchException if the read-only arguments are not all expressed in the
    *                                         same reference frame.
    */
   public static int intersectionBetweenRay2DAndBoundingBox2D(FramePoint2DReadOnly boundingBoxMin,
                                                              FramePoint2DReadOnly boundingBoxMax,
                                                              FramePoint2DReadOnly rayOrigin,
                                                              FrameVector2DReadOnly rayDirection,
                                                              FramePoint2DBasics firstIntersectionToPack,
                                                              FramePoint2DBasics secondIntersectionToPack)
   {
      boundingBoxMin.checkReferenceFrameMatch(boundingBoxMax, rayOrigin, rayDirection);
      int numberOfIntersections = EuclidGeometryTools.intersectionBetweenRay2DAndBoundingBox2D(boundingBoxMin,
                                                                                               boundingBoxMax,
                                                                                               rayOrigin,
                                                                                               rayDirection,
                                                                                               firstIntersectionToPack,
                                                                                               secondIntersectionToPack);

      // Set the correct reference frame.
      if (firstIntersectionToPack != null)
         firstIntersectionToPack.setReferenceFrame(rayOrigin.getReferenceFrame());
      if (secondIntersectionToPack != null)
         secondIntersectionToPack.setReferenceFrame(rayOrigin.getReferenceFrame());

      return numberOfIntersections;
   }

   /**
    * Computes the intersection between a 2D and a 2D line segment.
    * <p>
    * Edge cases:
    * <ul>
    * <li>When the ray and the line segment are parallel but not collinear, they do not intersect, this
    * method returns {@code null}.
    * <li>When the ray and the line segment are collinear, they are assumed to intersect at
    * {@code lineSegmentStart}.
    * <li>When the ray intersects the line segment at one of its endpoints, this method returns that
    * same endpoint.
    * </ul>
    * </p>
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param rayOrigin        a point located on the ray. Not modified.
    * @param rayDirection     the direction of the ray. Not modified.
    * @param lineSegmentStart the first endpoint of the line segment. Not modified.
    * @param lineSegmentEnd   the second endpoint of the line segment. Not modified.
    * @return the 2D point of intersection if it exist, {@code null} otherwise.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static FramePoint2D intersectionBetweenRay2DAndLineSegment2D(FramePoint2DReadOnly rayOrigin,
                                                                       FrameVector2DReadOnly rayDirection,
                                                                       FramePoint2DReadOnly lineSegmentStart,
                                                                       FramePoint2DReadOnly lineSegmentEnd)
   {
      rayOrigin.checkReferenceFrameMatch(rayDirection, lineSegmentStart, lineSegmentEnd);

      Point2D intersection = EuclidGeometryTools.intersectionBetweenRay2DAndLineSegment2D(rayOrigin, rayDirection, lineSegmentStart, lineSegmentEnd);
      if (intersection == null)
         return null;
      else
         return new FramePoint2D(rayOrigin.getReferenceFrame(), intersection);
   }

   /**
    * Computes the intersection between a 2D ray and a 2D line segment.
    * <p>
    * Edge cases:
    * <ul>
    * <li>When the ray and the line segment are parallel but not collinear, they do not intersect.
    * <li>When the ray and the line segment are collinear, they are assumed to intersect at
    * {@code lineSegmentStart}.
    * <li>When the ray intersects the line segment at one of its endpoints, this method returns
    * {@code true} and the endpoint is the intersection.
    * <li>When there is no intersection, this method returns {@code false} and
    * {@code intersectionToPack} is set to {@link Double#NaN}.
    * </ul>
    * </p>
    *
    * @param rayOrigin          a point located on the ray. Not modified.
    * @param rayDirection       the direction of the ray. Not modified.
    * @param lineSegmentStart   the first endpoint of the line segment. Not modified.
    * @param lineSegmentEnd     the second endpoint of the line segment. Not modified.
    * @param intersectionToPack the 2D point in which the result is stored. Can be {@code null}.
    *                           Modified.
    * @return {@code true} if the ray intersects the line segment, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static boolean intersectionBetweenRay2DAndLineSegment2D(FramePoint2DReadOnly rayOrigin,
                                                                  FrameVector2DReadOnly rayDirection,
                                                                  FramePoint2DReadOnly lineSegmentStart,
                                                                  FramePoint2DReadOnly lineSegmentEnd,
                                                                  FixedFramePoint2DBasics intersectionToPack)
   {
      rayOrigin.checkReferenceFrameMatch(rayDirection, lineSegmentStart, lineSegmentEnd);
      if (intersectionToPack != null)
         rayOrigin.checkReferenceFrameMatch(intersectionToPack);

      boolean success = EuclidGeometryTools.intersectionBetweenRay2DAndLineSegment2D(rayOrigin,
                                                                                     rayDirection,
                                                                                     lineSegmentStart,
                                                                                     lineSegmentEnd,
                                                                                     intersectionToPack);

      return success;
   }

   /**
    * Computes the intersection between a 2D ray and a 2D line segment.
    * <p>
    * Edge cases:
    * <ul>
    * <li>When the ray and the line segment are parallel but not collinear, they do not intersect.
    * <li>When the ray and the line segment are collinear, they are assumed to intersect at
    * {@code lineSegmentStart}.
    * <li>When the ray intersects the line segment at one of its endpoints, this method returns
    * {@code true} and the endpoint is the intersection.
    * <li>When there is no intersection, this method returns {@code false} and
    * {@code intersectionToPack} is set to {@link Double#NaN}.
    * </ul>
    * </p>
    *
    * @param rayOrigin          a point located on the ray. Not modified.
    * @param rayDirection       the direction of the ray. Not modified.
    * @param lineSegmentStart   the first endpoint of the line segment. Not modified.
    * @param lineSegmentEnd     the second endpoint of the line segment. Not modified.
    * @param intersectionToPack the 2D point in which the result is stored. Can be {@code null}.
    *                           Modified.
    * @return {@code true} if the ray intersects the line segment, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if the read-only arguments are not all expressed in the
    *                                         same reference frame.
    */
   public static boolean intersectionBetweenRay2DAndLineSegment2D(FramePoint2DReadOnly rayOrigin,
                                                                  FrameVector2DReadOnly rayDirection,
                                                                  FramePoint2DReadOnly lineSegmentStart,
                                                                  FramePoint2DReadOnly lineSegmentEnd,
                                                                  FramePoint2DBasics intersectionToPack)
   {
      rayOrigin.checkReferenceFrameMatch(rayDirection, lineSegmentStart, lineSegmentEnd);

      boolean success = EuclidGeometryTools.intersectionBetweenRay2DAndLineSegment2D(rayOrigin,
                                                                                     rayDirection,
                                                                                     lineSegmentStart,
                                                                                     lineSegmentEnd,
                                                                                     intersectionToPack);

      if (intersectionToPack != null)
         intersectionToPack.setReferenceFrame(rayOrigin.getReferenceFrame());

      return success;
   }

   /**
    * Computes the coordinates of the possible intersections between a ray and an axis-aligned bounding
    * box.
    * <p>
    * <a href=
    * "https://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-box-intersection">Useful
    * link</a>.
    * </p>
    * <p>
    * Intersection(s) between the ray and the bounding box cannot exist before the origin of the ray.
    * </p>
    * </p>
    * In the case the ray and the bounding box do not intersect, this method returns {@code 0} and
    * {@code firstIntersectionToPack} and {@code secondIntersectionToPack} are set to
    * {@link Double#NaN}.
    * </p>
    * <p>
    * In the case only one intersection exists between the ray and the bounding box,
    * {@code firstIntersectionToPack} will contain the coordinate of the intersection and
    * {@code secondIntersectionToPack} will be set to contain only {@link Double#NaN}.
    * </p>
    *
    * @param boundingBoxMin           the minimum coordinate of the bounding box. Not modified.
    * @param boundingBoxMax           the maximum coordinate of the bounding box. Not modified.
    * @param rayOrigin                the coordinate of the ray origin. Not modified.
    * @param rayDirection             the direction of the ray. Not modified.
    * @param firstIntersectionToPack  the coordinate of the first intersection. Can be {@code null}.
    *                                 Modified.
    * @param secondIntersectionToPack the coordinate of the second intersection. Can be {@code null}.
    *                                 Modified.
    * @return the number of intersections between the ray and the bounding box. It is either equal to
    *         0, 1, or 2.
    * @throws BoundingBoxException            if any of the minimum coordinates of the bounding box is
    *                                         strictly greater than the maximum coordinate of the
    *                                         bounding box on the same axis.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static int intersectionBetweenRay3DAndBoundingBox3D(FramePoint3DReadOnly boundingBoxMin,
                                                              FramePoint3DReadOnly boundingBoxMax,
                                                              FramePoint3DReadOnly rayOrigin,
                                                              FrameVector3DReadOnly rayDirection,
                                                              FixedFramePoint3DBasics firstIntersectionToPack,
                                                              FixedFramePoint3DBasics secondIntersectionToPack)
   {
      boundingBoxMin.checkReferenceFrameMatch(boundingBoxMax, rayOrigin, rayDirection);
      if (firstIntersectionToPack != null)
         boundingBoxMin.checkReferenceFrameMatch(firstIntersectionToPack);
      if (secondIntersectionToPack != null)
         boundingBoxMin.checkReferenceFrameMatch(secondIntersectionToPack);
      int numberOfIntersections = EuclidGeometryTools.intersectionBetweenRay3DAndBoundingBox3D(boundingBoxMin,
                                                                                               boundingBoxMax,
                                                                                               rayOrigin,
                                                                                               rayDirection,
                                                                                               firstIntersectionToPack,
                                                                                               secondIntersectionToPack);

      return numberOfIntersections;
   }

   /**
    * Computes the coordinates of the possible intersections between a 3D ray and a 3D box
    * <p>
    * <a href=
    * "https://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-box-intersection">Useful
    * link</a>.
    * </p>
    * <p>
    * In the case the ray and the bounding box do not intersect, this method returns {@code 0} and
    * {@code firstIntersectionToPack} and {@code secondIntersectionToPack} are set to
    * {@link Double#NaN}.
    * </p>
    *
    * @param boxPosition              the coordinates of the box position. Not modified.
    * @param boxOrientation           the orientation of the box. Not modified.
    * @param boxSize                  the size of the box. Not modified.
    * @param rayOrigin                the origin point of the 3D ray. Not modified.
    * @param rayDirection             the direction of the 3D ray. Not modified.
    * @param firstIntersectionToPack  the coordinate of the first intersection. Can be {@code null}.
    *                                 Modified.
    * @param secondIntersectionToPack the coordinate of the second intersection. Can be {@code null}.
    *                                 Modified.
    * @return the number of intersections between the line and the 3D box. It is either equal to 0, 1,
    *         or 2. If the ray origin is on the surface of the 3D box it is considered an intersection.
    * @throws IllegalArgumentException if {@code boxSize} contains values <= 0.0
    * @throws ReferenceFrameMismatchException if the read-only arguments are not all expressed in the
    *                                         same reference frame.
    * @see #intersectionBetweenRay3DAndBoundingBox3D(Point3DReadOnly, Point3DReadOnly, Point3DReadOnly,
    *      Vector3DReadOnly, Point3DBasics, Point3DBasics)
    */
   public static int intersectionBetweenRay3DAndBox3D(FramePoint3DReadOnly boxPosition,
                                                      FrameOrientation3DReadOnly boxOrientation,
                                                      FrameVector3DReadOnly boxSize,
                                                      FramePoint3DReadOnly rayOrigin,
                                                      FrameVector3DReadOnly rayDirection,
                                                      FramePoint3DBasics firstIntersectionToPack,
                                                      FramePoint3DBasics secondIntersectionToPack)
   {
      boxPosition.checkReferenceFrameMatch(boxSize, rayOrigin, rayDirection, boxOrientation);
      int numberOfIntersections = EuclidGeometryTools.intersectionBetweenRay3DAndBox3D(boxPosition,
                                                                                       boxOrientation,
                                                                                       boxSize,
                                                                                       rayOrigin,
                                                                                       rayDirection,
                                                                                       firstIntersectionToPack,
                                                                                       secondIntersectionToPack);

      // Set the correct reference frame.
      if (firstIntersectionToPack != null)
         firstIntersectionToPack.setReferenceFrame(rayOrigin.getReferenceFrame());
      if (secondIntersectionToPack != null)
         secondIntersectionToPack.setReferenceFrame(rayOrigin.getReferenceFrame());

      return numberOfIntersections;
   }

   /**
    * Computes the coordinates of the possible intersections between a 3D ray and a 3D box
    * <p>
    * <a href=
    * "https://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-box-intersection">Useful
    * link</a>.
    * </p>
    * <p>
    * In the case the ray and the bounding box do not intersect, this method returns {@code 0} and
    * {@code firstIntersectionToPack} and {@code secondIntersectionToPack} are set to
    * {@link Double#NaN}.
    * </p>
    *
    * @param boxPosition              the coordinates of the box position. Not modified.
    * @param boxOrientation           the orientation of the box. Not modified.
    * @param boxSize                  the size of the box. Not modified.
    * @param rayOrigin                the origin point of the 3D ray. Not modified.
    * @param rayDirection             the direction of the 3D ray. Not modified.
    * @param firstIntersectionToPack  the coordinate of the first intersection. Can be {@code null}.
    *                                 Modified.
    * @param secondIntersectionToPack the coordinate of the second intersection. Can be {@code null}.
    *                                 Modified.
    * @return the number of intersections between the line and the 3D box. It is either equal to 0, 1,
    *         or 2. If the ray origin is on the surface of the 3D box it is considered an intersection.
    * @throws IllegalArgumentException        if {@code boxSize} contains values <= 0.0
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    * @see #intersectionBetweenRay3DAndBoundingBox3D(Point3DReadOnly, Point3DReadOnly, Point3DReadOnly,
    *      Vector3DReadOnly, Point3DBasics, Point3DBasics)
    */
   public static int intersectionBetweenRay3DAndBox3D(FramePoint3DReadOnly boxPosition,
                                                      FrameOrientation3DReadOnly boxOrientation,
                                                      FrameVector3DReadOnly boxSize,
                                                      FramePoint3DReadOnly rayOrigin,
                                                      FrameVector3DReadOnly rayDirection,
                                                      FixedFramePoint3DBasics firstIntersectionToPack,
                                                      FixedFramePoint3DBasics secondIntersectionToPack)
   {
      boxPosition.checkReferenceFrameMatch(boxSize, rayOrigin, rayDirection, boxOrientation);
      if (firstIntersectionToPack != null)
         rayOrigin.checkReferenceFrameMatch(firstIntersectionToPack);
      if (secondIntersectionToPack != null)
         rayOrigin.checkReferenceFrameMatch(secondIntersectionToPack);

      int numberOfIntersections = EuclidGeometryTools.intersectionBetweenRay3DAndBox3D(boxPosition,
                                                                                       boxOrientation,
                                                                                       boxSize,
                                                                                       rayOrigin,
                                                                                       rayDirection,
                                                                                       firstIntersectionToPack,
                                                                                       secondIntersectionToPack);

      return numberOfIntersections;
   }

   /**
    * Computes the coordinates of the possible intersections between a ray and an axis-aligned bounding
    * box.
    * <p>
    * <a href=
    * "https://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-box-intersection">Useful
    * link</a>.
    * </p>
    * <p>
    * Intersection(s) between the ray and the bounding box cannot exist before the origin of the ray.
    * </p>
    * </p>
    * In the case the ray and the bounding box do not intersect, this method returns {@code 0} and
    * {@code firstIntersectionToPack} and {@code secondIntersectionToPack} are set to
    * {@link Double#NaN}.
    * </p>
    * <p>
    * In the case only one intersection exists between the ray and the bounding box,
    * {@code firstIntersectionToPack} will contain the coordinate of the intersection and
    * {@code secondIntersectionToPack} will be set to contain only {@link Double#NaN}.
    * </p>
    *
    * @param boundingBoxMin           the minimum coordinate of the bounding box. Not modified.
    * @param boundingBoxMax           the maximum coordinate of the bounding box. Not modified.
    * @param rayOrigin                the coordinate of the ray origin. Not modified.
    * @param rayDirection             the direction of the ray. Not modified.
    * @param firstIntersectionToPack  the coordinate of the first intersection. Can be {@code null}.
    *                                 Modified.
    * @param secondIntersectionToPack the coordinate of the second intersection. Can be {@code null}.
    *                                 Modified.
    * @return the number of intersections between the ray and the bounding box. It is either equal to
    *         0, 1, or 2.
    * @throws BoundingBoxException            if any of the minimum coordinates of the bounding box is
    *                                         strictly greater than the maximum coordinate of the
    *                                         bounding box on the same axis.
    * @throws ReferenceFrameMismatchException if the read-only arguments are not all expressed in the
    *                                         same reference frame.
    */
   public static int intersectionBetweenRay3DAndBoundingBox3D(FramePoint3DReadOnly boundingBoxMin,
                                                              FramePoint3DReadOnly boundingBoxMax,
                                                              FramePoint3DReadOnly rayOrigin,
                                                              FrameVector3DReadOnly rayDirection,
                                                              FramePoint3DBasics firstIntersectionToPack,
                                                              FramePoint3DBasics secondIntersectionToPack)
   {
      boundingBoxMin.checkReferenceFrameMatch(boundingBoxMax, rayOrigin, rayDirection);
      int numberOfIntersections = EuclidGeometryTools.intersectionBetweenRay3DAndBoundingBox3D(boundingBoxMin,
                                                                                               boundingBoxMax,
                                                                                               rayOrigin,
                                                                                               rayDirection,
                                                                                               firstIntersectionToPack,
                                                                                               secondIntersectionToPack);

      // Set the correct reference frame.
      if (firstIntersectionToPack != null)
         firstIntersectionToPack.setReferenceFrame(rayOrigin.getReferenceFrame());
      if (secondIntersectionToPack != null)
         secondIntersectionToPack.setReferenceFrame(rayOrigin.getReferenceFrame());

      return numberOfIntersections;
   }

   /**
    * Computes the coordinates of the possible intersections between a ray and a cylinder.
    * <p>
    * <a href= "http://mrl.nyu.edu/~dzorin/rend05/lecture2.pdf">Useful link</a>.
    * </p>
    * <p>
    * In the case the ray and the cylinder do not intersect, this method returns {@code 0} and
    * {@code firstIntersectionToPack} and {@code secondIntersectionToPack} are set {@link Double#NaN}.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>if either {@code cylinderLength == 0} or {@code cylinderRadius == 0}, this method fails and
    * return {@code 0}.
    * </ul>
    * </p>
    *
    * @param cylinderLength           length of the cylinder.
    * @param cylinderRadius           radius of the cylinder.
    * @param cylinderPosition         the center of the cylinder.
    * @param cylinderAxis             the cylinder's axis.
    * @param rayOrigin                the coordinate of the ray origin. Not modified.
    * @param rayDirection             the direction of the ray. Not modified.
    * @param firstIntersectionToPack  the coordinate of the first intersection. Can be {@code null}.
    *                                 Modified.
    * @param secondIntersectionToPack the coordinate of the second intersection. Can be {@code null}.
    *                                 Modified.
    * @return the number of intersections between the ray and the bounding box. It is either equal to
    *         0, 1, or 2.
    * @throws IllegalArgumentException        if either {@code cylinderLength < 0} or
    *                                         {@code cylinderRadius < 0}.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static int intersectionBetweenRay3DAndCylinder3D(double cylinderLength,
                                                           double cylinderRadius,
                                                           FramePoint3DReadOnly cylinderPosition,
                                                           FrameVector3DReadOnly cylinderAxis,
                                                           FramePoint3DReadOnly rayOrigin,
                                                           FrameVector3DReadOnly rayDirection,
                                                           FixedFramePoint3DBasics firstIntersectionToPack,
                                                           FixedFramePoint3DBasics secondIntersectionToPack)
   {
      cylinderPosition.checkReferenceFrameMatch(cylinderAxis, rayOrigin, rayDirection);
      if (firstIntersectionToPack != null)
         rayOrigin.checkReferenceFrameMatch(firstIntersectionToPack);
      if (secondIntersectionToPack != null)
         rayOrigin.checkReferenceFrameMatch(secondIntersectionToPack);
      int numberOfIntersections = EuclidGeometryTools.intersectionBetweenRay3DAndCylinder3D(cylinderLength,
                                                                                            cylinderRadius,
                                                                                            cylinderPosition,
                                                                                            cylinderAxis,
                                                                                            rayOrigin,
                                                                                            rayDirection,
                                                                                            firstIntersectionToPack,
                                                                                            secondIntersectionToPack);

      return numberOfIntersections;
   }

   /**
    * Computes the coordinates of the possible intersections between a ray and a cylinder.
    * <p>
    * <a href= "http://mrl.nyu.edu/~dzorin/rend05/lecture2.pdf">Useful link</a>.
    * </p>
    * <p>
    * In the case the ray and the cylinder do not intersect, this method returns {@code 0} and
    * {@code firstIntersectionToPack} and {@code secondIntersectionToPack} are set {@link Double#NaN}.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>if either {@code cylinderLength == 0} or {@code cylinderRadius == 0}, this method fails and
    * return {@code 0}.
    * </ul>
    * </p>
    *
    * @param cylinderLength           length of the cylinder.
    * @param cylinderRadius           radius of the cylinder.
    * @param cylinderPosition         the center of the cylinder.
    * @param cylinderAxis             the cylinder's axis.
    * @param rayOrigin                the coordinate of the ray origin. Not modified.
    * @param rayDirection             the direction of the ray. Not modified.
    * @param firstIntersectionToPack  the coordinate of the first intersection. Can be {@code null}.
    *                                 Modified.
    * @param secondIntersectionToPack the coordinate of the second intersection. Can be {@code null}.
    *                                 Modified.
    * @return the number of intersections between the ray and the bounding box. It is either equal to
    *         0, 1, or 2.
    * @throws IllegalArgumentException        if either {@code cylinderLength < 0} or
    *                                         {@code cylinderRadius < 0}.
    * @throws ReferenceFrameMismatchException if the read-only arguments are not all expressed in the
    *                                         same reference frame.
    */
   public static int intersectionBetweenRay3DAndCylinder3D(double cylinderLength,
                                                           double cylinderRadius,
                                                           FramePoint3DReadOnly cylinderPosition,
                                                           FrameVector3DReadOnly cylinderAxis,
                                                           FramePoint3DReadOnly rayOrigin,
                                                           FrameVector3DReadOnly rayDirection,
                                                           FramePoint3DBasics firstIntersectionToPack,
                                                           FramePoint3DBasics secondIntersectionToPack)
   {
      cylinderPosition.checkReferenceFrameMatch(cylinderAxis, rayOrigin, rayDirection);
      int numberOfIntersections = EuclidGeometryTools.intersectionBetweenRay3DAndCylinder3D(cylinderLength,
                                                                                            cylinderRadius,
                                                                                            cylinderPosition,
                                                                                            cylinderAxis,
                                                                                            rayOrigin,
                                                                                            rayDirection,
                                                                                            firstIntersectionToPack,
                                                                                            secondIntersectionToPack);

      // Set the correct reference frame.
      if (firstIntersectionToPack != null)
         firstIntersectionToPack.setReferenceFrame(rayOrigin.getReferenceFrame());
      if (secondIntersectionToPack != null)
         secondIntersectionToPack.setReferenceFrame(rayOrigin.getReferenceFrame());

      return numberOfIntersections;
   }

   /**
    * Computes the coordinates of the possible intersections between a ray and a ellipsoid.
    * <p>
    * The ellipsoid is center at (0, 0, 0).
    * </p>
    * <p>
    * In the case the ray and the ellipsoid do not intersect, this method returns {@code 0} and
    * {@code firstIntersectionToPack} and {@code secondIntersectionToPack} are set to
    * {@link Double#NaN}.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>if either {@code radiusX}, {@code radiusY}, or {@code radiusZ} is equal to {@code 0}, this
    * method fails and return {@code 0}.
    * </ul>
    * </p>
    *
    * @param radiusX                  radius of the ellipsoid along the x-axis.
    * @param radiusY                  radius of the ellipsoid along the y-axis.
    * @param radiusZ                  radius of the ellipsoid along the z-axis.
    * @param rayOrigin                the coordinate of the ray origin. Not modified.
    * @param rayDirection             the direction of the ray. Not modified.
    * @param firstIntersectionToPack  the coordinate of the first intersection. Can be {@code null}.
    *                                 Modified.
    * @param secondIntersectionToPack the coordinate of the second intersection. Can be {@code null}.
    *                                 Modified.
    * @return the number of intersections between the line/line-segment/ray and the ellipsoid. It is
    *         either equal to 0, 1, or 2.
    * @throws IllegalArgumentException        if either {@code radiusX}, {@code radiusY}, or
    *                                         {@code radiusZ} is negative.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static int intersectionBetweenRay3DAndEllipsoid3D(double radiusX,
                                                            double radiusY,
                                                            double radiusZ,
                                                            double positionX,
                                                            double positionY,
                                                            double positionZ,
                                                            FramePoint3DReadOnly rayOrigin,
                                                            FrameVector3DReadOnly rayDirection,
                                                            FixedFramePoint3DBasics firstIntersectionToPack,
                                                            FixedFramePoint3DBasics secondIntersectionToPack)
   {
      rayOrigin.checkReferenceFrameMatch(rayDirection);
      if (firstIntersectionToPack != null)
         rayOrigin.checkReferenceFrameMatch(firstIntersectionToPack);
      if (secondIntersectionToPack != null)
         rayOrigin.checkReferenceFrameMatch(secondIntersectionToPack);
      int numberOfIntersections = EuclidGeometryTools.intersectionBetweenRay3DAndEllipsoid3D(radiusX,
                                                                                             radiusY,
                                                                                             radiusZ,
                                                                                             positionX,
                                                                                             positionY,
                                                                                             positionZ,
                                                                                             rayOrigin,
                                                                                             rayDirection,
                                                                                             firstIntersectionToPack,
                                                                                             secondIntersectionToPack);

      return numberOfIntersections;
   }

   /**
    * Computes the coordinates of the possible intersections between a ray and a ellipsoid.
    * <p>
    * The ellipsoid is center at (0, 0, 0).
    * </p>
    * <p>
    * In the case the ray and the ellipsoid do not intersect, this method returns {@code 0} and
    * {@code firstIntersectionToPack} and {@code secondIntersectionToPack} are set to
    * {@link Double#NaN}.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>if either {@code radiusX}, {@code radiusY}, or {@code radiusZ} is equal to {@code 0}, this
    * method fails and return {@code 0}.
    * </ul>
    * </p>
    *
    * @param radiusX                  radius of the ellipsoid along the x-axis.
    * @param radiusY                  radius of the ellipsoid along the y-axis.
    * @param radiusZ                  radius of the ellipsoid along the z-axis.
    * @param rayOrigin                the coordinate of the ray origin. Not modified.
    * @param rayDirection             the direction of the ray. Not modified.
    * @param firstIntersectionToPack  the coordinate of the first intersection. Can be {@code null}.
    *                                 Modified.
    * @param secondIntersectionToPack the coordinate of the second intersection. Can be {@code null}.
    *                                 Modified.
    * @return the number of intersections between the line/line-segment/ray and the ellipsoid. It is
    *         either equal to 0, 1, or 2.
    * @throws IllegalArgumentException        if either {@code radiusX}, {@code radiusY}, or
    *                                         {@code radiusZ} is negative.
    * @throws ReferenceFrameMismatchException if the read-only arguments are not all expressed in the
    *                                         same reference frame.
    */
   public static int intersectionBetweenRay3DAndEllipsoid3D(double radiusX,
                                                            double radiusY,
                                                            double radiusZ,
                                                            double positionX,
                                                            double positionY,
                                                            double positionZ,
                                                            FramePoint3DReadOnly rayOrigin,
                                                            FrameVector3DReadOnly rayDirection,
                                                            FramePoint3DBasics firstIntersectionToPack,
                                                            FramePoint3DBasics secondIntersectionToPack)
   {
      rayOrigin.checkReferenceFrameMatch(rayDirection);
      int numberOfIntersections = EuclidGeometryTools.intersectionBetweenRay3DAndEllipsoid3D(radiusX,
                                                                                             radiusY,
                                                                                             radiusZ,
                                                                                             positionX,
                                                                                             positionY,
                                                                                             positionZ,
                                                                                             rayOrigin,
                                                                                             rayDirection,
                                                                                             firstIntersectionToPack,
                                                                                             secondIntersectionToPack);

      // Set the correct reference frame.
      if (firstIntersectionToPack != null)
         firstIntersectionToPack.setReferenceFrame(rayOrigin.getReferenceFrame());
      if (secondIntersectionToPack != null)
         secondIntersectionToPack.setReferenceFrame(rayOrigin.getReferenceFrame());

      return numberOfIntersections;
   }

   /**
    * Computes the intersection between two infinitely long 2D lines each defined by two 2D points.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the two lines are parallel but not collinear, the two lines do not intersect and this
    * method returns {@code null}.
    * <li>if the two lines are collinear, the two lines are assumed to be intersecting at
    * {@code pointOnLine1}.
    * </ul>
    * </p>
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param firstPointOnLine1  a first point located on the first line. Not modified.
    * @param secondPointOnLine1 a second point located on the first line. Not modified.
    * @param firstPointOnLine2  a first point located on the second line. Not modified.
    * @param secondPointOnLine2 a second point located on the second line. Not modified.
    * @return the 2D point of intersection if the two lines intersect, {@code null} otherwise.
    * @throws ReferenceFrameMismatchException if the read-only arguments are not all expressed in the
    *                                         same reference frame.
    */
   public static FramePoint2D intersectionBetweenTwoLine2Ds(FramePoint2DReadOnly firstPointOnLine1,
                                                            FramePoint2DReadOnly secondPointOnLine1,
                                                            FramePoint2DReadOnly firstPointOnLine2,
                                                            FramePoint2DReadOnly secondPointOnLine2)
   {
      firstPointOnLine1.checkReferenceFrameMatch(secondPointOnLine1, firstPointOnLine2, secondPointOnLine2);
      Point2D intersection = EuclidGeometryTools.intersectionBetweenTwoLine2Ds(firstPointOnLine1, secondPointOnLine1, firstPointOnLine2, secondPointOnLine2);

      if (intersection == null)
         return null;
      else
         return new FramePoint2D(firstPointOnLine1.getReferenceFrame(), intersection);
   }

   /**
    * Computes the intersection between two infinitely long 2D lines each defined by two 2D points.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the two lines are parallel but not collinear, the two lines do not intersect.
    * <li>if the two lines are collinear, the two lines are assumed to be intersecting at
    * {@code pointOnLine1}.
    * <li>When there is no intersection, this method returns {@code false} and
    * {@code intersectionToPack} is set to {@link Double#NaN}.
    * </ul>
    * </p>
    *
    * @param firstPointOnLine1  a first point located on the first line. Not modified.
    * @param secondPointOnLine1 a second point located on the first line. Not modified.
    * @param firstPointOnLine2  a first point located on the second line. Not modified.
    * @param secondPointOnLine2 a second point located on the second line. Not modified.
    * @param intersectionToPack 2D point in which the result is stored. Can be {@code null}. Modified.
    * @return {@code true} if the two lines intersect, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static boolean intersectionBetweenTwoLine2Ds(FramePoint2DReadOnly firstPointOnLine1,
                                                       FramePoint2DReadOnly secondPointOnLine1,
                                                       FramePoint2DReadOnly firstPointOnLine2,
                                                       FramePoint2DReadOnly secondPointOnLine2,
                                                       FixedFramePoint2DBasics intersectionToPack)
   {
      firstPointOnLine1.checkReferenceFrameMatch(secondPointOnLine1, firstPointOnLine2, secondPointOnLine2);
      if (intersectionToPack != null)
         firstPointOnLine1.checkReferenceFrameMatch(intersectionToPack);
      return EuclidGeometryTools.intersectionBetweenTwoLine2Ds(firstPointOnLine1,
                                                               secondPointOnLine1,
                                                               firstPointOnLine2,
                                                               secondPointOnLine2,
                                                               intersectionToPack);
   }

   /**
    * Computes the intersection between two infinitely long 2D lines each defined by two 2D points.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the two lines are parallel but not collinear, the two lines do not intersect.
    * <li>if the two lines are collinear, the two lines are assumed to be intersecting at
    * {@code pointOnLine1}.
    * <li>When there is no intersection, this method returns {@code false} and
    * {@code intersectionToPack} is set to {@link Double#NaN}.
    * </ul>
    * </p>
    *
    * @param firstPointOnLine1  a first point located on the first line. Not modified.
    * @param secondPointOnLine1 a second point located on the first line. Not modified.
    * @param firstPointOnLine2  a first point located on the second line. Not modified.
    * @param secondPointOnLine2 a second point located on the second line. Not modified.
    * @param intersectionToPack 2D point in which the result is stored. Can be {@code null}. Modified.
    * @return {@code true} if the two lines intersect, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if the read-only arguments are not all expressed in the
    *                                         same reference frame.
    */
   public static boolean intersectionBetweenTwoLine2Ds(FramePoint2DReadOnly firstPointOnLine1,
                                                       FramePoint2DReadOnly secondPointOnLine1,
                                                       FramePoint2DReadOnly firstPointOnLine2,
                                                       FramePoint2DReadOnly secondPointOnLine2,
                                                       FramePoint2DBasics intersectionToPack)
   {
      firstPointOnLine1.checkReferenceFrameMatch(secondPointOnLine1, firstPointOnLine2, secondPointOnLine2);
      if (intersectionToPack != null)
         intersectionToPack.setReferenceFrame(firstPointOnLine1.getReferenceFrame());
      return EuclidGeometryTools.intersectionBetweenTwoLine2Ds(firstPointOnLine1,
                                                               secondPointOnLine1,
                                                               firstPointOnLine2,
                                                               secondPointOnLine2,
                                                               intersectionToPack);
   }

   /**
    * Computes the intersection between two infinitely long 2D lines each defined by a 2D point and a
    * 2D direction.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the two lines are parallel but not collinear, the two lines do not intersect and this
    * method returns {@code null}.
    * <li>if the two lines are collinear, the two lines are assumed to be intersecting at
    * {@code pointOnLine1}.
    * </ul>
    * </p>
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param pointOnLine1   point located on the first line. Not modified.
    * @param lineDirection1 the first line direction. Not modified.
    * @param pointOnLine2   point located on the second line. Not modified.
    * @param lineDirection2 the second line direction. Not modified.
    * @return the 2D point of intersection if the two lines intersect, {@code null} otherwise.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static FramePoint2D intersectionBetweenTwoLine2Ds(FramePoint2DReadOnly pointOnLine1,
                                                            FrameVector2DReadOnly lineDirection1,
                                                            FramePoint2DReadOnly pointOnLine2,
                                                            FrameVector2DReadOnly lineDirection2)
   {
      pointOnLine1.checkReferenceFrameMatch(lineDirection1, pointOnLine2, lineDirection2);

      Point2D intersection = EuclidGeometryTools.intersectionBetweenTwoLine2Ds(pointOnLine1, lineDirection1, pointOnLine2, lineDirection2);
      if (intersection != null)
         return new FramePoint2D(pointOnLine1.getReferenceFrame(), intersection);
      else
         return null;
   }

   /**
    * Computes the intersection between two infinitely long 2D lines each defined by a 2D point and a
    * 2D direction.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the two lines are parallel but not collinear, the two lines do not intersect.
    * <li>if the two lines are collinear, the two lines are assumed to be intersecting at
    * {@code pointOnLine1}.
    * <li>When there is no intersection, this method returns {@code false} and
    * {@code intersectionToPack} is set to {@link Double#NaN}.
    * </ul>
    * </p>
    *
    * @param pointOnLine1       point located on the first line. Not modified.
    * @param lineDirection1     the first line direction. Not modified.
    * @param pointOnLine2       point located on the second line. Not modified.
    * @param lineDirection2     the second line direction. Not modified.
    * @param intersectionToPack 2D point in which the result is stored. Can be {@code null}. Modified.
    * @return {@code true} if the two lines intersect, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static boolean intersectionBetweenTwoLine2Ds(FramePoint2DReadOnly pointOnLine1,
                                                       FrameVector2DReadOnly lineDirection1,
                                                       FramePoint2DReadOnly pointOnLine2,
                                                       FrameVector2DReadOnly lineDirection2,
                                                       FixedFramePoint2DBasics intersectionToPack)
   {
      pointOnLine1.checkReferenceFrameMatch(lineDirection1, pointOnLine2, lineDirection2);
      if (intersectionToPack != null)
         pointOnLine1.checkReferenceFrameMatch(intersectionToPack);
      boolean success = EuclidGeometryTools.intersectionBetweenTwoLine2Ds(pointOnLine1, lineDirection1, pointOnLine2, lineDirection2, intersectionToPack);

      return success;
   }

   /**
    * Computes the intersection between two infinitely long 2D lines each defined by a 2D point and a
    * 2D direction.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the two lines are parallel but not collinear, the two lines do not intersect.
    * <li>if the two lines are collinear, the two lines are assumed to be intersecting at
    * {@code pointOnLine1}.
    * <li>When there is no intersection, this method returns {@code false} and
    * {@code intersectionToPack} is set to {@link Double#NaN}.
    * </ul>
    * </p>
    *
    * @param pointOnLine1       point located on the first line. Not modified.
    * @param lineDirection1     the first line direction. Not modified.
    * @param pointOnLine2       point located on the second line. Not modified.
    * @param lineDirection2     the second line direction. Not modified.
    * @param intersectionToPack 2D point in which the result is stored. Can be {@code null}. Modified.
    * @return {@code true} if the two lines intersect, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if the read-only arguments are not all expressed in the
    *                                         same reference frame.
    */
   public static boolean intersectionBetweenTwoLine2Ds(FramePoint2DReadOnly pointOnLine1,
                                                       FrameVector2DReadOnly lineDirection1,
                                                       FramePoint2DReadOnly pointOnLine2,
                                                       FrameVector2DReadOnly lineDirection2,
                                                       FramePoint2DBasics intersectionToPack)
   {
      pointOnLine1.checkReferenceFrameMatch(lineDirection1, pointOnLine2, lineDirection2);
      boolean success = EuclidGeometryTools.intersectionBetweenTwoLine2Ds(pointOnLine1, lineDirection1, pointOnLine2, lineDirection2, intersectionToPack);

      if (intersectionToPack != null)
         intersectionToPack.setReferenceFrame(pointOnLine1.getReferenceFrame());

      return success;
   }

   /**
    * Computes the intersection between two 2D line segments each defined by their two 2D endpoints.
    * <p>
    * Edge cases:
    * <ul>
    * <li>When the two line segments are parallel but not collinear, the two line segments do not
    * intersect, this method returns {@code null}.
    * <li>When the two line segments are collinear, if the two line segments do not overlap do not have
    * at least one common endpoint, this method returns {@code null}.
    * <li>When the two line segments have a common endpoint, this method returns the common endpoint as
    * the intersection.
    * </ul>
    * </p>
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param lineSegmentStart1 the first endpoint of the first line segment. Not modified.
    * @param lineSegmentEnd1   the second endpoint of the first line segment. Not modified.
    * @param lineSegmentStart2 the first endpoint of the second line segment. Not modified.
    * @param lineSegmentEnd2   the second endpoint of the second line segment. Not modified.
    * @return the intersection point if it exists, {@code null} otherwise.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.s
    */
   public static FramePoint2D intersectionBetweenTwoLineSegment2Ds(FramePoint2DReadOnly lineSegmentStart1,
                                                                   FramePoint2DReadOnly lineSegmentEnd1,
                                                                   FramePoint2DReadOnly lineSegmentStart2,
                                                                   FramePoint2DReadOnly lineSegmentEnd2)
   {
      lineSegmentStart1.checkReferenceFrameMatch(lineSegmentEnd1, lineSegmentStart2, lineSegmentEnd2);
      Point2D intersection = EuclidGeometryTools.intersectionBetweenTwoLineSegment2Ds(lineSegmentStart1, lineSegmentEnd1, lineSegmentStart2, lineSegmentEnd2);
      if (intersection != null)
         return new FramePoint2D(lineSegmentEnd1.getReferenceFrame(), intersection);
      else
         return null;
   }

   /**
    * Computes the intersection between two 2D line segments each defined by their two 2D endpoints.
    * <p>
    * Edge cases:
    * <ul>
    * <li>When the two line segments are parallel but not collinear, the two line segments do not
    * intersect.
    * <li>When the two line segments are collinear, this methods returns {@code true} only if the two
    * line segments overlap or have at least one common endpoint.
    * <li>When the two line segments have a common endpoint, this method returns {@code true}.
    * <li>When there is no intersection, this method returns {@code false} and
    * {@code intersectionToPack} is set to {@link Double#NaN}.
    * </ul>
    * </p>
    *
    * @param lineSegmentStart1  the first endpoint of the first line segment. Not modified.
    * @param lineSegmentEnd1    the second endpoint of the first line segment. Not modified.
    * @param lineSegmentStart2  the first endpoint of the second line segment. Not modified.
    * @param lineSegmentEnd2    the second endpoint of the second line segment. Not modified.
    * @param intersectionToPack the 2D point in which the result is stored. Modified.
    * @return {@code true} if the two line segments intersect, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static boolean intersectionBetweenTwoLineSegment2Ds(FramePoint2DReadOnly lineSegmentStart1,
                                                              FramePoint2DReadOnly lineSegmentEnd1,
                                                              FramePoint2DReadOnly lineSegmentStart2,
                                                              FramePoint2DReadOnly lineSegmentEnd2,
                                                              FixedFramePoint2DBasics intersectionToPack)
   {
      lineSegmentStart1.checkReferenceFrameMatch(lineSegmentEnd1, lineSegmentStart2, lineSegmentEnd2);
      if (intersectionToPack != null)
         lineSegmentStart1.checkReferenceFrameMatch(intersectionToPack);
      boolean success = EuclidGeometryTools.intersectionBetweenTwoLineSegment2Ds(lineSegmentStart1,
                                                                                 lineSegmentEnd1,
                                                                                 lineSegmentStart2,
                                                                                 lineSegmentEnd2,
                                                                                 intersectionToPack);

      return success;
   }

   /**
    * Computes the intersection between two 2D line segments each defined by their two 2D endpoints.
    * <p>
    * Edge cases:
    * <ul>
    * <li>When the two line segments are parallel but not collinear, the two line segments do not
    * intersect.
    * <li>When the two line segments are collinear, this methods returns {@code true} only if the two
    * line segments overlap or have at least one common endpoint.
    * <li>When the two line segments have a common endpoint, this method returns {@code true}.
    * <li>When there is no intersection, this method returns {@code false} and
    * {@code intersectionToPack} is set to {@link Double#NaN}.
    * </ul>
    * </p>
    *
    * @param lineSegmentStart1  the first endpoint of the first line segment. Not modified.
    * @param lineSegmentEnd1    the second endpoint of the first line segment. Not modified.
    * @param lineSegmentStart2  the first endpoint of the second line segment. Not modified.
    * @param lineSegmentEnd2    the second endpoint of the second line segment. Not modified.
    * @param intersectionToPack the 2D point in which the result is stored. Modified.
    * @return {@code true} if the two line segments intersect, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if the read-only arguments are not all expressed in the
    *                                         same reference frame.
    */
   public static boolean intersectionBetweenTwoLineSegment2Ds(FramePoint2DReadOnly lineSegmentStart1,
                                                              FramePoint2DReadOnly lineSegmentEnd1,
                                                              FramePoint2DReadOnly lineSegmentStart2,
                                                              FramePoint2DReadOnly lineSegmentEnd2,
                                                              FramePoint2DBasics intersectionToPack)
   {
      lineSegmentStart1.checkReferenceFrameMatch(lineSegmentEnd1, lineSegmentStart2, lineSegmentEnd2);
      boolean success = EuclidGeometryTools.intersectionBetweenTwoLineSegment2Ds(lineSegmentStart1,
                                                                                 lineSegmentEnd1,
                                                                                 lineSegmentStart2,
                                                                                 lineSegmentEnd2,
                                                                                 intersectionToPack);

      if (intersectionToPack != null)
         intersectionToPack.setReferenceFrame(lineSegmentStart1.getReferenceFrame());

      return success;
   }

   /**
    * This methods calculates the line of intersection between two planes each defined by a point and a
    * normal. The result is packed in a 3D point located on the intersection line and the 3D direction
    * of the intersection.
    * <p>
    * <a href="http://mathworld.wolfram.com/Plane-PlaneIntersection.html"> Useful link 1</a>,
    * <a href="http://paulbourke.net/geometry/pointlineplane/"> useful link 2</a>.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>When the length of either the plane normal is below
    * {@link EuclidGeometryTools#ONE_TRILLIONTH}, this methods fails and returns {@code false}.
    * <li>When the angle between the two planes is below {@code angleThreshold}, this methods fails and
    * returns {@code false}.
    * <li>When there is no intersection, this method returns {@code false} and
    * {@code pointOnIntersectionToPack} and {@code intersectionDirectionToPack} are set to
    * {@link Double#NaN}.
    * </ul>
    * </p>
    *
    * @param pointOnPlane1               a point on the first plane. Not modified.
    * @param planeNormal1                the normal of the first plane. Not modified.
    * @param pointOnPlane2               a point on the second plane. Not modified.
    * @param planeNormal2                the normal of the second plane. Not modified.
    * @param angleThreshold              the minimum angle between the two planes required to do the
    *                                    calculation.
    * @param pointOnIntersectionToPack   a 3D point that is set such that it belongs to the line of
    *                                    intersection between the two planes. Modified.
    * @param intersectionDirectionToPack a 3D vector that is set to the direction of the line of
    *                                    intersection between the two planes. Modified.
    * @return {@code true} if the intersection was calculated properly, {@code false} otherwise.
    * @throws IllegalArgumentException        if <tt>angleThreshold</tt> &notin; [0; <i>pi</i>/2]
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static boolean intersectionBetweenTwoPlane3Ds(FramePoint3DReadOnly pointOnPlane1,
                                                        FrameVector3DReadOnly planeNormal1,
                                                        FramePoint3DReadOnly pointOnPlane2,
                                                        FrameVector3DReadOnly planeNormal2,
                                                        double angleThreshold,
                                                        FixedFramePoint3DBasics pointOnIntersectionToPack,
                                                        FixedFrameVector3DBasics intersectionDirectionToPack)
   {
      pointOnPlane1.checkReferenceFrameMatch(planeNormal1, pointOnPlane2, planeNormal2);
      if (pointOnIntersectionToPack != null)
         pointOnPlane1.checkReferenceFrameMatch(pointOnIntersectionToPack);
      if (intersectionDirectionToPack != null)
         pointOnPlane1.checkReferenceFrameMatch(intersectionDirectionToPack);
      boolean success = EuclidGeometryTools.intersectionBetweenTwoPlane3Ds(pointOnPlane1,
                                                                           planeNormal1,
                                                                           pointOnPlane2,
                                                                           planeNormal2,
                                                                           angleThreshold,
                                                                           pointOnIntersectionToPack,
                                                                           intersectionDirectionToPack);

      return success;
   }

   /**
    * This methods calculates the line of intersection between two planes each defined by a point and a
    * normal. The result is packed in a 3D point located on the intersection line and the 3D direction
    * of the intersection.
    * <p>
    * <a href="http://mathworld.wolfram.com/Plane-PlaneIntersection.html"> Useful link 1</a>,
    * <a href="http://paulbourke.net/geometry/pointlineplane/"> useful link 2</a>.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>When the length of either the plane normal is below
    * {@link EuclidGeometryTools#ONE_TRILLIONTH}, this methods fails and returns {@code false}.
    * <li>When the angle between the two planes is below {@code angleThreshold}, this methods fails and
    * returns {@code false}.
    * <li>When there is no intersection, this method returns {@code false} and
    * {@code pointOnIntersectionToPack} and {@code intersectionDirectionToPack} are set to
    * {@link Double#NaN}.
    * </ul>
    * </p>
    *
    * @param pointOnPlane1               a point on the first plane. Not modified.
    * @param planeNormal1                the normal of the first plane. Not modified.
    * @param pointOnPlane2               a point on the second plane. Not modified.
    * @param planeNormal2                the normal of the second plane. Not modified.
    * @param angleThreshold              the minimum angle between the two planes required to do the
    *                                    calculation.
    * @param pointOnIntersectionToPack   a 3D point that is set such that it belongs to the line of
    *                                    intersection between the two planes. Modified.
    * @param intersectionDirectionToPack a 3D vector that is set to the direction of the line of
    *                                    intersection between the two planes. Modified.
    * @return {@code true} if the intersection was calculated properly, {@code false} otherwise.
    * @throws IllegalArgumentException        if <tt>angleThreshold</tt> &notin; [0; <i>pi</i>/2]
    * @throws ReferenceFrameMismatchException if the read-only arguments are not all expressed in the
    *                                         same reference frame.
    */
   public static boolean intersectionBetweenTwoPlane3Ds(FramePoint3DReadOnly pointOnPlane1,
                                                        FrameVector3DReadOnly planeNormal1,
                                                        FramePoint3DReadOnly pointOnPlane2,
                                                        FrameVector3DReadOnly planeNormal2,
                                                        double angleThreshold,
                                                        FramePoint3DBasics pointOnIntersectionToPack,
                                                        FrameVector3DBasics intersectionDirectionToPack)
   {
      pointOnPlane1.checkReferenceFrameMatch(planeNormal1, pointOnPlane2, planeNormal2);
      boolean success = EuclidGeometryTools.intersectionBetweenTwoPlane3Ds(pointOnPlane1,
                                                                           planeNormal1,
                                                                           pointOnPlane2,
                                                                           planeNormal2,
                                                                           angleThreshold,
                                                                           pointOnIntersectionToPack,
                                                                           intersectionDirectionToPack);

      if (pointOnIntersectionToPack != null)
         pointOnIntersectionToPack.setReferenceFrame(planeNormal1.getReferenceFrame());
      if (intersectionDirectionToPack != null)
         intersectionDirectionToPack.setReferenceFrame(planeNormal1.getReferenceFrame());

      return success;
   }

   /**
    * This methods calculates the line of intersection between two planes each defined by a point and a
    * normal. The result is packed in a 3D point located on the intersection line and the 3D direction
    * of the intersection.
    * <p>
    * <a href="http://mathworld.wolfram.com/Plane-PlaneIntersection.html"> Useful link 1</a>,
    * <a href="http://paulbourke.net/geometry/pointlineplane/"> useful link 2</a>.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>When the length of either the plane normal is below
    * {@link EuclidGeometryTools#ONE_TRILLIONTH}, this methods fails and returns {@code false}.
    * <li>When the angle between the two planes is below {@link EuclidGeometryTools#ONE_MILLIONTH},
    * this methods fails and returns {@code false}.
    * </ul>
    * </p>
    *
    * @param pointOnPlane1               a point on the first plane. Not modified.
    * @param planeNormal1                the normal of the first plane. Not modified.
    * @param pointOnPlane2               a point on the second plane. Not modified.
    * @param planeNormal2                the normal of the second plane. Not modified.
    * @param pointOnIntersectionToPack   a 3D point that is set such that it belongs to the line of
    *                                    intersection between the two planes. Modified.
    * @param intersectionDirectionToPack a 3D vector that is set to the direction of the line of
    *                                    intersection between the two planes. Modified.
    * @return {@code true} if the intersection was calculated properly, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static boolean intersectionBetweenTwoPlane3Ds(FramePoint3DReadOnly pointOnPlane1,
                                                        FrameVector3DReadOnly planeNormal1,
                                                        FramePoint3DReadOnly pointOnPlane2,
                                                        FrameVector3DReadOnly planeNormal2,
                                                        FixedFramePoint3DBasics pointOnIntersectionToPack,
                                                        FixedFrameVector3DBasics intersectionDirectionToPack)
   {
      pointOnPlane1.checkReferenceFrameMatch(planeNormal1, pointOnPlane2, planeNormal2);
      if (pointOnIntersectionToPack != null)
         pointOnPlane1.checkReferenceFrameMatch(pointOnIntersectionToPack);
      if (intersectionDirectionToPack != null)
         pointOnPlane1.checkReferenceFrameMatch(intersectionDirectionToPack);
      return EuclidGeometryTools.intersectionBetweenTwoPlane3Ds(pointOnPlane1,
                                                                planeNormal1,
                                                                pointOnPlane2,
                                                                planeNormal2,
                                                                pointOnIntersectionToPack,
                                                                intersectionDirectionToPack);
   }

   /**
    * This methods calculates the line of intersection between two planes each defined by a point and a
    * normal. The result is packed in a 3D point located on the intersection line and the 3D direction
    * of the intersection.
    * <p>
    * <a href="http://mathworld.wolfram.com/Plane-PlaneIntersection.html"> Useful link 1</a>,
    * <a href="http://paulbourke.net/geometry/pointlineplane/"> useful link 2</a>.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>When the length of either the plane normal is below
    * {@link EuclidGeometryTools#ONE_TRILLIONTH}, this methods fails and returns {@code false}.
    * <li>When the angle between the two planes is below {@link EuclidGeometryTools#ONE_MILLIONTH},
    * this methods fails and returns {@code false}.
    * </ul>
    * </p>
    *
    * @param pointOnPlane1               a point on the first plane. Not modified.
    * @param planeNormal1                the normal of the first plane. Not modified.
    * @param pointOnPlane2               a point on the second plane. Not modified.
    * @param planeNormal2                the normal of the second plane. Not modified.
    * @param pointOnIntersectionToPack   a 3D point that is set such that it belongs to the line of
    *                                    intersection between the two planes. Modified.
    * @param intersectionDirectionToPack a 3D vector that is set to the direction of the line of
    *                                    intersection between the two planes. Modified.
    * @return {@code true} if the intersection was calculated properly, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if the read-only arguments are not all expressed in the
    *                                         same reference frame.
    */
   public static boolean intersectionBetweenTwoPlane3Ds(FramePoint3DReadOnly pointOnPlane1,
                                                        FrameVector3DReadOnly planeNormal1,
                                                        FramePoint3DReadOnly pointOnPlane2,
                                                        FrameVector3DReadOnly planeNormal2,
                                                        FramePoint3DBasics pointOnIntersectionToPack,
                                                        FrameVector3DBasics intersectionDirectionToPack)
   {
      pointOnPlane1.checkReferenceFrameMatch(planeNormal1, pointOnPlane2, planeNormal2);
      boolean success = EuclidGeometryTools.intersectionBetweenTwoPlane3Ds(pointOnPlane1,
                                                                           planeNormal1,
                                                                           pointOnPlane2,
                                                                           planeNormal2,
                                                                           pointOnIntersectionToPack,
                                                                           intersectionDirectionToPack);
      if (success)
      {
         if (pointOnIntersectionToPack != null)
            pointOnIntersectionToPack.setReferenceFrame(planeNormal1.getReferenceFrame());
         if (intersectionDirectionToPack != null)
            intersectionDirectionToPack.setReferenceFrame(planeNormal1.getReferenceFrame());
      }
      return success;
   }

   /**
    * Determines if the query is: ahead of the ray, i.e. the projection onto ray lies in front of the
    * ray's origin, behind the ray, or neither, i.e. the projection is equal to the ray origin.
    *
    * @param point        the query. Not modified.
    * @param rayOrigin    the ray's origin. Not modified.
    * @param rayDirection the ray's direction. Not modified.
    * @return {@link Location#AHEAD} if the query is located in front of the ray,
    *         {@link Location#BEHIND} if the query is behind the ray, and {@code null} if the query's
    *         projection onto the ray is exactly equal to the ray origin.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static Location whichPartOfRay2DIsPoint2DOn(FramePoint2DReadOnly point, FramePoint2DReadOnly rayOrigin, FrameVector2DReadOnly rayDirection)
   {
      point.checkReferenceFrameMatch(rayOrigin, rayDirection);
      return EuclidGeometryTools.whichPartOfRay2DIsPoint2DOn(point, rayOrigin, rayDirection);
   }

   /**
    * Determines if the query is exactly on or on the right side of the infinitely long line that goes
    * through the ray origin and which direction is perpendicular to the ray and directed towards the
    * left side.
    *
    * @param point        the query. Not modified.
    * @param rayOrigin    the ray's origin. Not modified.
    * @param rayDirection the ray's direction. Not modified.
    * @return {@code true} if the query is located in front of the ray.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static boolean isPoint2DInFrontOfRay2D(FramePoint2DReadOnly point, FramePoint2DReadOnly rayOrigin, FrameVector2DReadOnly rayDirection)
   {
      return whichPartOfRay2DIsPoint2DOn(point, rayOrigin, rayDirection) != Location.BEHIND;
   }

   /**
    * Returns {@code true} only if the point is inside the triangle defined by the vertices a, b, and
    * c. The triangle can be clockwise or counter-clockwise ordered.
    *
    * @param point the point to check if lying inside the triangle. Not modified.
    * @param a     first vertex of the triangle. Not modified.
    * @param b     second vertex of the triangle. Not modified.
    * @param c     third vertex of the triangle. Not modified.
    * @return {@code true} if the query is exactly inside the triangle. {@code false} if the query
    *         point is outside triangle or exactly on an edge of the triangle.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static boolean isPoint2DInsideTriangleABC(FramePoint2DReadOnly point, FramePoint2DReadOnly a, FramePoint2DReadOnly b, FramePoint2DReadOnly c)
   {
      point.checkReferenceFrameMatch(a, b, c);
      return EuclidGeometryTools.isPoint2DInsideTriangleABC(point, a, b, c);
   }

   /**
    * Tests if the point 2D is located on the infinitely long line 2D.
    * <p>
    * The test is performed by computing the distance between the point and the line, if that distance
    * is below {@link EuclidGeometryTools#IS_POINT_ON_LINE_EPS} this method returns {@code true}.
    * </p>
    *
    * @param pointX        the x-coordinate of the query.
    * @param pointY        the y-coordinate of the query.
    * @param pointOnLine   a point located on the line. Not modified.
    * @param lineDirection the direction of the line. Not modified.
    * @return {@code true} if the query is considered to be lying on the line, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static boolean isPoint2DOnLine2D(double pointX, double pointY, FramePoint2DReadOnly pointOnLine, FrameVector2DReadOnly lineDirection)
   {
      pointOnLine.checkReferenceFrameMatch(lineDirection);
      return EuclidGeometryTools.isPoint2DOnLine2D(pointX, pointY, pointOnLine, lineDirection);
   }

   /**
    * Tests if the point 2D is located on the infinitely long line 2D.
    * <p>
    * The test is performed by computing the distance between the point and the line, if that distance
    * is below {@link EuclidGeometryTools#IS_POINT_ON_LINE_EPS} this method returns {@code true}.
    * </p>
    *
    * @param point         the coordinates of the query. Not modified.
    * @param pointOnLine   a point located on the line. Not modified.
    * @param lineDirection the direction of the line. Not modified.
    * @return {@code true} if the query is considered to be lying on the line, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static boolean isPoint2DOnLine2D(FramePoint2DReadOnly point, FramePoint2DReadOnly pointOnLine, FrameVector2DReadOnly lineDirection)
   {
      point.checkReferenceFrameMatch(pointOnLine, lineDirection);
      return EuclidGeometryTools.isPoint2DOnLine2D(point, pointOnLine, lineDirection);
   }

   /**
    * Tests if the point 2D is located on the infinitely long line 2D.
    * <p>
    * The test is performed by computing the distance between the point and the line, if that distance
    * is below {@link EuclidGeometryTools#IS_POINT_ON_LINE_EPS} this method returns {@code true}.
    * </p>
    *
    * @param point            the coordinates of the query. Not modified.
    * @param lineSegmentStart the first endpoint of the line segment. Not modified.
    * @param lineSegmentEnd   the second endpoint of the line segment. Not modified.
    * @return {@code true} if the query is considered to be lying on the line, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static boolean isPoint2DOnLineSegment2D(FramePoint2DReadOnly point, FramePoint2DReadOnly lineSegmentStart, FramePoint2DReadOnly lineSegmentEnd)
   {
      point.checkReferenceFrameMatch(lineSegmentStart, lineSegmentEnd);
      return EuclidGeometryTools.isPoint2DOnLineSegment2D(point, lineSegmentStart, lineSegmentEnd);
   }

   /**
    * Returns whether a 2D point is on the left or right side of an infinitely long line defined by two
    * points. The idea of "side" is determined based on order of {@code firstPointOnLine} and
    * {@code secondPointOnLine}.
    * <p>
    * For instance, given the {@code firstPointOnLine} coordinates x = 0, and y = 0, and the
    * {@code secondPointOnLine} coordinates x = 0, y = 1, a point located on:
    * <ul>
    * <li>the left side of this line has a negative x coordinate.
    * <li>the right side of this line has a positive x coordinate.
    * </ul>
    * </p>
    * This method will return {@code null} if the point is on the line.
    *
    * @param pointX            the x-coordinate of the query point.
    * @param pointY            the y-coordinate of the query point.
    * @param firstPointOnLine  a first point located on the line. Not modified.
    * @param secondPointOnLine a second point located on the line. Not modified.
    * @return {@link Location.LEFT}/{@link Location.RIGHT} if the point is on the left/right side of
    *         the line, or {@code null} if the point is exactly on the line.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static Location whichSideOfLine2DIsPoint2DOn(double pointX,
                                                       double pointY,
                                                       FramePoint2DReadOnly firstPointOnLine,
                                                       FramePoint2DReadOnly secondPointOnLine)
   {
      firstPointOnLine.checkReferenceFrameMatch(secondPointOnLine);
      return EuclidGeometryTools.whichSideOfLine2DIsPoint2DOn(pointX, pointY, firstPointOnLine, secondPointOnLine);
   }

   /**
    * Returns whether a 2D point is on the left or right side of an infinitely long line. The idea of
    * "side" is determined based on the direction of the line.
    * <p>
    * For instance, given the {@code lineDirection} components x = 0, and y = 1, and the
    * {@code pointOnLine} coordinates x = 0, and y = 0, a point located on:
    * <ul>
    * <li>the left side of this line has a negative x coordinate.
    * <li>the right side of this line has a positive x coordinate.
    * </ul>
    * </p>
    * This method will return {@code null} if the point is on the line.
    *
    * @param pointX        the x-coordinate of the query point.
    * @param pointY        the y-coordinate of the query point.
    * @param pointOnLine   a point positioned on the infinite line. Not modified.
    * @param lineDirection the direction of the infinite line. Not modified.
    * @return {@link Location.LEFT}/{@link Location.RIGHT} if the point is on the left/right side of
    *         the line, or {@code null} if the point is exactly on the line.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static Location whichSideOfLine2DIsPoint2DOn(double pointX, double pointY, FramePoint2DReadOnly pointOnLine, FrameVector2DReadOnly lineDirection)
   {
      pointOnLine.checkReferenceFrameMatch(lineDirection);
      return EuclidGeometryTools.whichSideOfLine2DIsPoint2DOn(pointX, pointY, pointOnLine, lineDirection);
   }

   /**
    * Returns whether a 2D point is on the left or right side of an infinitely long line defined by two
    * points. The idea of "side" is determined based on order of {@code firstPointOnLine} and
    * {@code secondPointOnLine}.
    * <p>
    * For instance, given the {@code firstPointOnLine} coordinates x = 0, and y = 0, and the
    * {@code secondPointOnLine} coordinates x = 0, y = 1, a point located on:
    * <ul>
    * <li>the left side of this line has a negative x coordinate.
    * <li>the right side of this line has a positive x coordinate.
    * </ul>
    * </p>
    * This method will return {@code null} if the point is on the line.
    *
    * @param point             the query point. Not modified.
    * @param firstPointOnLine  a first point located on the line. Not modified.
    * @param secondPointOnLine a second point located on the line. Not modified.
    * @return {@link Location.LEFT}/{@link Location.RIGHT} if the point is on the left/right side of
    *         the line, or {@code null} if the point is exactly on the line.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static Location whichSideOfLine2DIsPoint2DOn(FramePoint2DReadOnly point,
                                                       FramePoint2DReadOnly firstPointOnLine,
                                                       FramePoint2DReadOnly secondPointOnLine)
   {
      point.checkReferenceFrameMatch(firstPointOnLine, secondPointOnLine);
      return EuclidGeometryTools.whichSideOfLine2DIsPoint2DOn(point, firstPointOnLine, secondPointOnLine);
   }

   /**
    * Returns whether a 2D point is on the left or right side of an infinitely long line. The idea of
    * "side" is determined based on the direction of the line.
    * <p>
    * For instance, given the {@code lineDirection} components x = 0, and y = 1, and the
    * {@code pointOnLine} coordinates x = 0, and y = 0, a point located on:
    * <ul>
    * <li>the left side of this line has a negative x coordinate.
    * <li>the right side of this line has a positive x coordinate.
    * </ul>
    * </p>
    * This method will return {@code null} if the point is on the line.
    *
    * @param point         the query point. Not modified.
    * @param pointOnLine   a point positioned on the infinite line. Not modified.
    * @param lineDirection the direction of the infinite line. Not modified.
    * @return {@link Location.LEFT}/{@link Location.RIGHT} if the point is on the left/right side of
    *         the line, or {@code null} if the point is exactly on the line.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static Location whichSideOfLine2DIsPoint2DOn(FramePoint2DReadOnly point, FramePoint2DReadOnly pointOnLine, FrameVector2DReadOnly lineDirection)
   {
      point.checkReferenceFrameMatch(pointOnLine, lineDirection);
      return EuclidGeometryTools.whichSideOfLine2DIsPoint2DOn(point, pointOnLine, lineDirection);
   }

   /**
    * Returns a boolean value, stating whether a 2D point is on the left side of an infinitely long
    * line defined by two points. "Left side" is determined based on order of {@code lineStart} and
    * {@code lineEnd}.
    * <p>
    * For instance, given the {@code lineStart} coordinates x = 0, and y = 0, and the {@code lineEnd}
    * coordinates x = 1, y = 0, a point located on the left side of this line has a negative y
    * coordinate.
    * </p>
    * This method will return {@code false} if the point is on the line.
    *
    * @param point             the query point. Not modified.
    * @param firstPointOnLine  a first point located on the line. Not modified.
    * @param secondPointOnLine a second point located on the line. Not modified.
    * @return {@code true} if the point is on the left side of the line, {@code false} if the point is
    *         on the right side or exactly on the line.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static boolean isPoint2DOnLeftSideOfLine2D(FramePoint2DReadOnly point, FramePoint2DReadOnly firstPointOnLine, FramePoint2DReadOnly secondPointOnLine)
   {
      return whichSideOfLine2DIsPoint2DOn(point, firstPointOnLine, secondPointOnLine) == Location.LEFT;
   }

   /**
    * Returns a boolean value, stating whether a 2D point is on the right side of an infinitely long
    * line defined by two points. "Right side" is determined based on order of {@code lineStart} and
    * {@code lineEnd}.
    * <p>
    * For instance, given the {@code lineStart} coordinates x = 0, and y = 0, and the {@code lineEnd}
    * coordinates x = 1, y = 0, a point located on the right side of this line has a positive y
    * coordinate.
    * </p>
    * This method will return {@code false} if the point is on the line.
    *
    * @param point             the query point. Not modified.
    * @param firstPointOnLine  a first point located on the line. Not modified.
    * @param secondPointOnLine a second point located on the line. Not modified.
    * @return {@code true} if the point is on the right side of the line, {@code false} if the point is
    *         on the left side or exactly on the line.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static boolean isPoint2DOnRightSideOfLine2D(FramePoint2DReadOnly point, FramePoint2DReadOnly firstPointOnLine, FramePoint2DReadOnly secondPointOnLine)
   {
      return whichSideOfLine2DIsPoint2DOn(point, firstPointOnLine, secondPointOnLine) == Location.RIGHT;
   }

   /**
    * Returns a boolean value, stating whether a 2D point is on the left or right side of an infinitely
    * long line defined by two points. The idea of "side" is determined based on order of
    * {@code lineStart} and {@code lineEnd}.
    * <p>
    * For instance, given the {@code lineStart} coordinates x = 0, and y = 0, and the {@code lineEnd}
    * coordinates x = 1, y = 0, a point located on:
    * <ul>
    * <li>the left side of this line has a negative y coordinate.
    * <li>the right side of this line has a positive y coordinate.
    * </ul>
    * </p>
    * This method will return {@code false} if the point is on the line.
    *
    * @param pointX            the x-coordinate of the query point.
    * @param pointY            the y-coordinate of the query point.
    * @param firstPointOnLine  a first point located on the line. Not modified.
    * @param secondPointOnLine a second point located on the line. Not modified.
    * @param testLeftSide      the query of the side, when equal to {@code true} this will test for the
    *                          left side, {@code false} this will test for the right side.
    * @return {@code true} if the point is on the query side of the line, {@code false} if the point is
    *         on the opposite side or exactly on the line.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    * @deprecated Use
    *             {@link #whichSideOfLine2DIsPoint2DOn(double, double, FramePoint2DReadOnly, FramePoint2DReadOnly)}
    *             instead.
    */
   @Deprecated
   public static boolean isPoint2DOnSideOfLine2D(double pointX,
                                                 double pointY,
                                                 FramePoint2DReadOnly firstPointOnLine,
                                                 FramePoint2DReadOnly secondPointOnLine,
                                                 boolean testLeftSide)
   {
      firstPointOnLine.checkReferenceFrameMatch(secondPointOnLine);
      return EuclidGeometryTools.isPoint2DOnSideOfLine2D(pointX, pointY, firstPointOnLine, secondPointOnLine, testLeftSide);
   }

   /**
    * Returns a boolean value, stating whether a 2D point is on the left or right side of an infinitely
    * long line. The idea of "side" is determined based on the direction of the line.
    * <p>
    * For instance, given the {@code lineDirection} components x = 0, and y = 1, and the
    * {@code pointOnLine} coordinates x = 0, and y = 0, a point located on:
    * <ul>
    * <li>the left side of this line has a negative y coordinate.
    * <li>the right side of this line has a positive y coordinate.
    * </ul>
    * </p>
    * This method will return {@code false} if the point is on the line.
    *
    * @param pointX        the x-coordinate of the query point.
    * @param pointY        the y-coordinate of the query point.
    * @param pointOnLine   a point positioned on the infinite line. Not modified.
    * @param lineDirection the direction of the infinite line. Not modified.
    * @param testLeftSide  the query of the side, when equal to {@code true} this will test for the
    *                      left side, {@code false} this will test for the right side.
    * @return {@code true} if the point is on the query side of the line, {@code false} if the point is
    *         on the opposite side or exactly on the line.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    * @deprecated Use
    *             {@link #whichSideOfLine2DIsPoint2DOn(double, double, FramePoint2DReadOnly, FrameVector2DReadOnly)}
    *             instead.
    */
   @Deprecated
   public static boolean isPoint2DOnSideOfLine2D(double pointX,
                                                 double pointY,
                                                 FramePoint2DReadOnly pointOnLine,
                                                 FrameVector2DReadOnly lineDirection,
                                                 boolean testLeftSide)
   {
      pointOnLine.checkReferenceFrameMatch(lineDirection);
      return EuclidGeometryTools.isPoint2DOnSideOfLine2D(pointX, pointY, pointOnLine, lineDirection, testLeftSide);
   }

   /**
    * Returns a boolean value, stating whether a 2D point is on the left or right side of an infinitely
    * long line defined by two points. The idea of "side" is determined based on order of
    * {@code lineStart} and {@code lineEnd}.
    * <p>
    * For instance, given the {@code lineStart} coordinates x = 0, and y = 0, and the {@code lineEnd}
    * coordinates x = 1, y = 0, a point located on:
    * <ul>
    * <li>the left side of this line has a negative y coordinate.
    * <li>the right side of this line has a positive y coordinate.
    * </ul>
    * </p>
    * This method will return {@code false} if the point is on the line.
    *
    * @param point             the query point. Not modified.
    * @param firstPointOnLine  a first point located on the line. Not modified.
    * @param secondPointOnLine a second point located on the line. Not modified.
    * @param testLeftSide      the query of the side, when equal to {@code true} this will test for the
    *                          left side, {@code false} this will test for the right side.
    * @return {@code true} if the point is on the query side of the line, {@code false} if the point is
    *         on the opposite side or exactly on the line.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    * @deprecated Use
    *             {@link #whichSideOfLine2DIsPoint2DOn(FramePoint2DReadOnly, FramePoint2DReadOnly, FramePoint2DReadOnly)}
    *             instead.
    */
   @Deprecated
   public static boolean isPoint2DOnSideOfLine2D(FramePoint2DReadOnly point,
                                                 FramePoint2DReadOnly firstPointOnLine,
                                                 FramePoint2DReadOnly secondPointOnLine,
                                                 boolean testLeftSide)
   {
      point.checkReferenceFrameMatch(firstPointOnLine, secondPointOnLine);
      return EuclidGeometryTools.isPoint2DOnSideOfLine2D(point, firstPointOnLine, secondPointOnLine, testLeftSide);
   }

   /**
    * Returns a boolean value, stating whether a 2D point is on the left or right side of an infinitely
    * long line. The idea of "side" is determined based on the direction of the line.
    * <p>
    * For instance, given the {@code lineDirection} components x = 0, and y = 1, and the
    * {@code pointOnLine} coordinates x = 0, and y = 0, a point located on:
    * <ul>
    * <li>the left side of this line has a negative y coordinate.
    * <li>the right side of this line has a positive y coordinate.
    * </ul>
    * </p>
    * This method will return {@code false} if the point is on the line.
    *
    * @param point         the query point. Not modified.
    * @param pointOnLine   a point positioned on the infinite line. Not modified.
    * @param lineDirection the direction of the infinite line. Not modified.
    * @param testLeftSide  the query of the side, when equal to {@code true} this will test for the
    *                      left side, {@code false} this will test for the right side.
    * @return {@code true} if the point is on the query side of the line, {@code false} if the point is
    *         on the opposite side or exactly on the line.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    * @deprecated Use
    *             {@link #whichSideOfLine2DIsPoint2DOn(FramePoint2DReadOnly, FramePoint2DReadOnly, FrameVector2DReadOnly)}
    *             instead.
    */
   @Deprecated
   public static boolean isPoint2DOnSideOfLine2D(FramePoint2DReadOnly point,
                                                 FramePoint2DReadOnly pointOnLine,
                                                 FrameVector2DReadOnly lineDirection,
                                                 boolean testLeftSide)
   {
      point.checkReferenceFrameMatch(pointOnLine, lineDirection);
      return EuclidGeometryTools.isPoint2DOnSideOfLine2D(point, pointOnLine, lineDirection, testLeftSide);
   }

   /**
    * Returns whether a 3D point is above or below of an infinitely large 3D plane. The idea of "above"
    * and "below" is determined based on the normal of the plane.
    * <p>
    * For instance, given the {@code planeNormal} components x = 0, y = 0, and z = 1, and the
    * {@code pointOnPlane} coordinates x = 0, y = 0, and z = 0, a point located:
    * <ul>
    * <li>above this plane has a positive z coordinate.
    * <li>below this plane has a negative z coordinate.
    * </ul>
    * </p>
    * This method will return {@code null} if the point is on the plane.
    *
    * @param pointX       the x-coordinate of the query point.
    * @param pointY       the y-coordinate of the query point.
    * @param pointZ       the z-coordinate of the query point.
    * @param pointOnPlane the coordinates of a point positioned on the infinite plane. Not modified.
    * @param planeNormal  the normal of the infinite plane. Not modified.
    * @return {@link Location#ABOVE}/{@link Location#BELOW} if the point is above/below the plane,
    *         {@code null} if the point is exactly on the plane.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static Location whichSideOfPlane3DIsPoint3DOn(double pointX,
                                                        double pointY,
                                                        double pointZ,
                                                        FramePoint3DReadOnly pointOnPlane,
                                                        FrameVector3DReadOnly planeNormal)
   {
      pointOnPlane.checkReferenceFrameMatch(planeNormal);
      return EuclidGeometryTools.whichSideOfPlane3DIsPoint3DOn(pointX, pointY, pointZ, pointOnPlane, planeNormal);
   }

   /**
    * Returns whether a 3D point is above or below of an infinitely large 3D plane. The idea of "above"
    * and "below" is determined based on the normal of the plane.
    * <p>
    * For instance, given the {@code planeNormal} components x = 0, y = 0, and z = 1, and the
    * {@code pointOnPlane} coordinates x = 0, y = 0, and z = 0, a point located:
    * <ul>
    * <li>above this plane has a positive z coordinate.
    * <li>below this plane has a negative z coordinate.
    * </ul>
    * </p>
    * This method will return {@code null} if the point is on the plane.
    *
    * @param point        the coordinates of the query point.
    * @param pointOnPlane the coordinates of a point positioned on the infinite plane. Not modified.
    * @param planeNormal  the normal of the infinite plane. Not modified.
    * @return {@link Location#ABOVE}/{@link Location#BELOW} if the point is above/below the plane,
    *         {@code null} if the point is exactly on the plane.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static Location whichSideOfPlane3DIsPoint3DOn(FramePoint3DReadOnly point, FramePoint3DReadOnly pointOnPlane, FrameVector3DReadOnly planeNormal)
   {
      point.checkReferenceFrameMatch(pointOnPlane, planeNormal);
      return EuclidGeometryTools.whichSideOfPlane3DIsPoint3DOn(point, pointOnPlane, planeNormal);
   }

   /**
    * Returns whether a 3D point is above or below of an infinitely large 3D plane. The idea of "above"
    * and "below" is determined based on the normal of the plane.
    * <p>
    * The plane's normal is retrieved using the two given tangents:<br>
    * <tt>planeNormal = planeFirstTangent &times; planeSecondTangent</tt><br>
    * Given the plane's normal, this method then calls
    * {@link EuclidGeometryTools#whichSideOfPlane3DIsPoint3DOn(double, double, double, double, double, double, double, double, double)}.
    * </p>
    * <p>
    * This method will fail if the two given tangents are parallel.
    * </p>
    *
    * @param pointX             the x-coordinate of the query point.
    * @param pointY             the y-coordinate of the query point.
    * @param pointZ             the z-coordinate of the query point.
    * @param pointOnPlane       the coordinates of a point positioned on the infinite plane. Not
    *                           modified.
    * @param planeFirstTangent  a first tangent of the infinite plane. Not modified.
    * @param planeSecondTangent a second tangent of the infinite plane. Not modified.
    * @return {@link Location#ABOVE}/{@link Location#BELOW} if the point is above/below the plane,
    *         {@code null} if the point is exactly on the plane.
    * @see EuclidGeometryTools#whichSideOfPlane3DIsPoint3DOn(double, double, double, double, double,
    *      double, double, double, double)
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static Location whichSideOfPlane3DIsPoint3DOn(double pointX,
                                                        double pointY,
                                                        double pointZ,
                                                        FramePoint3DReadOnly pointOnPlane,
                                                        FrameVector3DReadOnly planeFirstTangent,
                                                        FrameVector3DReadOnly planeSecondTangent)
   {
      pointOnPlane.checkReferenceFrameMatch(planeFirstTangent, planeSecondTangent);
      return EuclidGeometryTools.whichSideOfPlane3DIsPoint3DOn(pointX, pointY, pointZ, pointOnPlane, planeFirstTangent, planeSecondTangent);
   }

   /**
    * Returns whether a 3D point is above or below of an infinitely large 3D plane. The idea of "above"
    * and "below" is determined based on the normal of the plane.
    * <p>
    * The plane's normal is retrieved using the two given tangents:<br>
    * <tt>planeNormal = planeFirstTangent &times; planeSecondTangent</tt><br>
    * Given the plane's normal, this method then calls
    * {@link EuclidGeometryTools#whichSideOfPlane3DIsPoint3DOn(double, double, double, double, double, double, double, double, double)}.
    * </p>
    * <p>
    * This method will fail if the two given tangents are parallel.
    * </p>
    *
    * @param point              the coordinates of the query point. Not modified.
    * @param pointOnPlane       the coordinates of a point positioned on the infinite plane. Not
    *                           modified.
    * @param planeFirstTangent  a first tangent of the infinite plane. Not modified.
    * @param planeSecondTangent a second tangent of the infinite plane. Not modified.
    * @return {@link Location#ABOVE}/{@link Location#BELOW} if the point is above/below the plane,
    *         {@code null} if the point is exactly on the plane.
    * @see EuclidGeometryTools#whichSideOfPlane3DIsPoint3DOn(double, double, double, double, double,
    *      double, double, double, double)
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static Location whichSideOfPlane3DIsPoint3DOn(FramePoint3DReadOnly point,
                                                        FramePoint3DReadOnly pointOnPlane,
                                                        FrameVector3DReadOnly planeFirstTangent,
                                                        FrameVector3DReadOnly planeSecondTangent)
   {
      point.checkReferenceFrameMatch(pointOnPlane, planeFirstTangent, planeSecondTangent);
      return EuclidGeometryTools.whichSideOfPlane3DIsPoint3DOn(point, pointOnPlane, planeFirstTangent, planeSecondTangent);
   }

   /**
    * Returns a boolean value, stating whether a 3D point is strictly above or below of an infinitely
    * large 3D plane. The idea of "above" and "below" is determined based on the normal of the plane.
    * <p>
    * For instance, given the {@code planeNormal} components x = 0, y = 0, and z = 1, and the
    * {@code pointOnPlane} coordinates x = 0, y = 0, and z = 0, a point located:
    * <ul>
    * <li>above this plane has a positive z coordinate.
    * <li>below this plane has a negative z coordinate.
    * </ul>
    * </p>
    * This method will return {@code false} if the point is on the plane.
    *
    * @param pointX       the x-coordinate of the query point.
    * @param pointY       the y-coordinate of the query point.
    * @param pointZ       the z-coordinate of the query point.
    * @param pointOnPlane the coordinates of a point positioned on the infinite plane. Not modified.
    * @param planeNormal  the normal of the infinite plane. Not modified.
    * @param testForAbove the query of the side, when equal to {@code true} this will test for the
    *                     above side, {@code false} this will test for the below side.
    * @return {@code true} if the point is on the query side of the plane, {@code false} if the point
    *         is on the opposite side or exactly on the plane.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    * @deprecated Use
    *             {@link #whichSideOfPlane3DIsPoint3DOn(double, double, double, FramePoint3DReadOnly, FrameVector3DReadOnly, boolean)}
    *             instead.
    */
   @Deprecated
   public static boolean isPoint3DAboveOrBelowPlane3D(double pointX,
                                                      double pointY,
                                                      double pointZ,
                                                      FramePoint3DReadOnly pointOnPlane,
                                                      FrameVector3DReadOnly planeNormal,
                                                      boolean testForAbove)
   {
      Location side = whichSideOfPlane3DIsPoint3DOn(pointX, pointY, pointZ, pointOnPlane, planeNormal);
      return testForAbove ? side == Location.ABOVE : side == Location.BELOW;
   }

   /**
    * Returns a boolean value, stating whether a 3D point is strictly above or below of an infinitely
    * large 3D plane. The idea of "above" and "below" is determined based on the normal of the plane.
    * <p>
    * For instance, given the {@code planeNormal} components x = 0, y = 0, and z = 1, and the
    * {@code pointOnPlane} coordinates x = 0, y = 0, and z = 0, a point located:
    * <ul>
    * <li>above this plane has a positive z coordinate.
    * <li>below this plane has a negative z coordinate.
    * </ul>
    * </p>
    * This method will return {@code false} if the point is on the plane.
    *
    * @param point        the coordinates of the query point.
    * @param pointOnPlane the coordinates of a point positioned on the infinite plane. Not modified.
    * @param planeNormal  the normal of the infinite plane. Not modified.
    * @param testForAbove the query of the side, when equal to {@code true} this will test for the
    *                     above side, {@code false} this will test for the below side.
    * @return {@code true} if the point is on the query side of the plane, {@code false} if the point
    *         is on the opposite side or exactly on the plane.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    * @deprecated Use
    *             {@link #whichSideOfPlane3DIsPoint3DOn(FramePoint3DReadOnly, FramePoint3DReadOnly, FrameVector3DReadOnly)}
    *             instead.
    */
   @Deprecated
   public static boolean isPoint3DAboveOrBelowPlane3D(FramePoint3DReadOnly point,
                                                      FramePoint3DReadOnly pointOnPlane,
                                                      FrameVector3DReadOnly planeNormal,
                                                      boolean testForAbove)
   {
      Location side = whichSideOfPlane3DIsPoint3DOn(point, pointOnPlane, planeNormal);
      return testForAbove ? side == Location.ABOVE : side == Location.BELOW;
   }

   /**
    * Returns a boolean value, stating if a 3D point is strictly above an infinitely large 3D plane.
    * The idea of "above" and "below" is determined based on the normal of the plane.
    * <p>
    * For instance, given the {@code planeNormal} components x = 0, y = 0, and z = 1, and the
    * {@code pointOnPlane} coordinates x = 0, y = 0, and z = 0, a point located:
    * <ul>
    * <li>above this plane has a positive z coordinate.
    * <li>below this plane has a negative z coordinate.
    * </ul>
    * </p>
    * This method will return {@code false} if the point is on the plane.
    *
    * @param pointX       the x-coordinate of the query point.
    * @param pointY       the y-coordinate of the query point.
    * @param pointZ       the z-coordinate of the query point.
    * @param pointOnPlane the coordinates of a point positioned on the infinite plane. Not modified.
    * @param planeNormal  the normal of the infinite plane. Not modified.
    * @return {@code true} if the point is strictly above the plane, {@code false} if the point is
    *         below or exactly on the plane.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static boolean isPoint3DAbovePlane3D(double pointX,
                                               double pointY,
                                               double pointZ,
                                               FramePoint3DReadOnly pointOnPlane,
                                               FrameVector3DReadOnly planeNormal)
   {
      return whichSideOfPlane3DIsPoint3DOn(pointX, pointY, pointZ, pointOnPlane, planeNormal) == Location.ABOVE;
   }

   /**
    * Returns a boolean value, stating if a 3D point is strictly above an infinitely large 3D plane.
    * The idea of "above" and "below" is determined based on the normal of the plane.
    * <p>
    * For instance, given the {@code planeNormal} components x = 0, y = 0, and z = 1, and the
    * {@code pointOnPlane} coordinates x = 0, y = 0, and z = 0, a point located:
    * <ul>
    * <li>above this plane has a positive z coordinate.
    * <li>below this plane has a negative z coordinate.
    * </ul>
    * </p>
    * This method will return {@code false} if the point is on the plane.
    *
    * @param point        the coordinates of the query point.
    * @param pointOnPlane the coordinates of a point positioned on the infinite plane. Not modified.
    * @param planeNormal  the normal of the infinite plane. Not modified.
    * @return {@code true} if the point is strictly above the plane, {@code false} if the point is
    *         below or exactly on the plane.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static boolean isPoint3DAbovePlane3D(FramePoint3DReadOnly point, FramePoint3DReadOnly pointOnPlane, FrameVector3DReadOnly planeNormal)
   {
      return whichSideOfPlane3DIsPoint3DOn(point, pointOnPlane, planeNormal) == Location.ABOVE;
   }

   /**
    * Returns a boolean value, stating if a 3D point is strictly below an infinitely large 3D plane.
    * The idea of "above" and "below" is determined based on the normal of the plane.
    * <p>
    * For instance, given the {@code planeNormal} components x = 0, y = 0, and z = 1, and the
    * {@code pointOnPlane} coordinates x = 0, y = 0, and z = 0, a point located:
    * <ul>
    * <li>above this plane has a positive z coordinate.
    * <li>below this plane has a negative z coordinate.
    * </ul>
    * </p>
    * This method will return {@code false} if the point is on the plane.
    *
    * @param pointX       the x-coordinate of the query point.
    * @param pointY       the y-coordinate of the query point.
    * @param pointZ       the z-coordinate of the query point.
    * @param pointOnPlane the coordinates of a point positioned on the infinite plane. Not modified.
    * @param planeNormal  the normal of the infinite plane. Not modified.
    * @return {@code true} if the point is strictly below the plane, {@code false} if the point is
    *         above or exactly on the plane.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static boolean isPoint3DBelowPlane3D(double pointX,
                                               double pointY,
                                               double pointZ,
                                               FramePoint3DReadOnly pointOnPlane,
                                               FrameVector3DReadOnly planeNormal)
   {
      return whichSideOfPlane3DIsPoint3DOn(pointX, pointY, pointZ, pointOnPlane, planeNormal) == Location.BELOW;
   }

   /**
    * Returns a boolean value, stating if a 3D point is strictly below an infinitely large 3D plane.
    * The idea of "above" and "below" is determined based on the normal of the plane.
    * <p>
    * For instance, given the {@code planeNormal} components x = 0, y = 0, and z = 1, and the
    * {@code pointOnPlane} coordinates x = 0, y = 0, and z = 0, a point located:
    * <ul>
    * <li>above this plane has a positive z coordinate.
    * <li>below this plane has a negative z coordinate.
    * </ul>
    * </p>
    * This method will return {@code false} if the point is on the plane.
    *
    * @param point        the coordinates of the query point.
    * @param pointOnPlane the coordinates of a point positioned on the infinite plane. Not modified.
    * @param planeNormal  the normal of the infinite plane. Not modified.
    * @return {@code true} if the point is strictly below the plane, {@code false} if the point is
    *         above or exactly on the plane.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static boolean isPoint3DBelowPlane3D(FramePoint3DReadOnly point, FramePoint3DReadOnly pointOnPlane, FrameVector3DReadOnly planeNormal)
   {
      return whichSideOfPlane3DIsPoint3DOn(point, pointOnPlane, planeNormal) == Location.BELOW;
   }

   /**
    * Returns a boolean value, stating whether a 3D point is strictly above or below of an infinitely
    * large 3D plane. The idea of "above" and "below" is determined based on the normal of the plane.
    * <p>
    * The plane's normal is retrieved using the two given tangents:<br>
    * <tt>planeNormal = planeFirstTangent &times; planeSecondTangent</tt><br>
    * Given the plane's normal, this method then calls
    * {@link EuclidGeometryTools#isPoint3DAboveOrBelowPlane3D(double, double, double, double, double, double, double, double, double, boolean)}.
    * </p>
    * <p>
    * This method will fail if the two given tangents are parallel.
    * </p>
    *
    * @param pointX             the x-coordinate of the query point.
    * @param pointY             the y-coordinate of the query point.
    * @param pointZ             the z-coordinate of the query point.
    * @param pointOnPlane       the coordinates of a point positioned on the infinite plane. Not
    *                           modified.
    * @param planeFirstTangent  a first tangent of the infinite plane. Not modified.
    * @param planeSecondTangent a second tangent of the infinite plane. Not modified.
    * @param testForAbove       the query of the side, when equal to {@code true} this will test for
    *                           the above side, {@code false} this will test for the below side.
    * @return {@code true} if the point is on the query side of the plane, {@code false} if the point
    *         is on the opposite side or exactly on the plane.
    * @see EuclidGeometryTools#isPoint3DAboveOrBelowPlane3D(double, double, double, double, double,
    *      double, double, double, double, boolean)
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    * @deprecated Use
    *             {@link #whichSideOfPlane3DIsPoint3DOn(double, double, double, FramePoint3DReadOnly, FrameVector3DReadOnly, FrameVector3DReadOnly)}
    *             instead.
    */
   @Deprecated
   public static boolean isPoint3DAboveOrBelowPlane3D(double pointX,
                                                      double pointY,
                                                      double pointZ,
                                                      FramePoint3DReadOnly pointOnPlane,
                                                      FrameVector3DReadOnly planeFirstTangent,
                                                      FrameVector3DReadOnly planeSecondTangent,
                                                      boolean testForAbove)
   {
      Location side = whichSideOfPlane3DIsPoint3DOn(pointX, pointY, pointZ, pointOnPlane, planeFirstTangent, planeSecondTangent);
      return testForAbove ? side == Location.ABOVE : side == Location.BELOW;
   }

   /**
    * Returns a boolean value, stating whether a 3D point is strictly above or below of an infinitely
    * large 3D plane. The idea of "above" and "below" is determined based on the normal of the plane.
    * <p>
    * The plane's normal is retrieved using the two given tangents:<br>
    * <tt>planeNormal = planeFirstTangent &times; planeSecondTangent</tt><br>
    * Given the plane's normal, this method then calls
    * {@link EuclidGeometryTools#isPoint3DAboveOrBelowPlane3D(double, double, double, double, double, double, double, double, double, boolean)}.
    * </p>
    * <p>
    * This method will fail if the two given tangents are parallel.
    * </p>
    *
    * @param point              the coordinates of the query point. Not modified.
    * @param pointOnPlane       the coordinates of a point positioned on the infinite plane. Not
    *                           modified.
    * @param planeFirstTangent  a first tangent of the infinite plane. Not modified.
    * @param planeSecondTangent a second tangent of the infinite plane. Not modified.
    * @param testForAbove       the query of the side, when equal to {@code true} this will test for
    *                           the above side, {@code false} this will test for the below side.
    * @return {@code true} if the point is on the query side of the plane, {@code false} if the point
    *         is on the opposite side or exactly on the plane.
    * @see EuclidGeometryTools#isPoint3DAboveOrBelowPlane3D(double, double, double, double, double,
    *      double, double, double, double, boolean)
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    * @deprecated Use
    *             {@link #whichSideOfPlane3DIsPoint3DOn(FramePoint3DReadOnly, FramePoint3DReadOnly, FrameVector3DReadOnly, FrameVector3DReadOnly)}
    *             instead.
    */
   public static boolean isPoint3DAboveOrBelowPlane3D(FramePoint3DReadOnly point,
                                                      FramePoint3DReadOnly pointOnPlane,
                                                      FrameVector3DReadOnly planeFirstTangent,
                                                      FrameVector3DReadOnly planeSecondTangent,
                                                      boolean testForAbove)
   {
      Location side = whichSideOfPlane3DIsPoint3DOn(point, pointOnPlane, planeFirstTangent, planeSecondTangent);
      return testForAbove ? side == Location.ABOVE : side == Location.BELOW;
   }

   /**
    * Returns a boolean value, stating if a 3D point is strictly above an infinitely large 3D plane.
    * The idea of "above" and "below" is determined based on the normal of the plane.
    * <p>
    * The plane's normal is retrieved using the two given tangents:<br>
    * <tt>planeNormal = planeFirstTangent &times; planeSecondTangent</tt><br>
    * Given the plane's normal, this method then calls
    * {@link EuclidGeometryTools#isPoint3DAboveOrBelowPlane3D(double, double, double, double, double, double, double, double, double, boolean)}.
    * </p>
    * <p>
    * This method will fail if the two given tangents are parallel.
    * </p>
    *
    * @param pointX             the x-coordinate of the query point.
    * @param pointY             the y-coordinate of the query point.
    * @param pointZ             the z-coordinate of the query point.
    * @param pointOnPlane       the coordinates of a point positioned on the infinite plane. Not
    *                           modified.
    * @param planeFirstTangent  a first tangent of the infinite plane. Not modified.
    * @param planeSecondTangent a second tangent of the infinite plane. Not modified.
    * @return {@code true} if the point is strictly above the plane, {@code false} if the point is
    *         below or exactly on the plane.
    * @see EuclidGeometryTools#isPoint3DAboveOrBelowPlane3D(double, double, double, double, double,
    *      double, double, double, double, boolean)
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static boolean isPoint3DAbovePlane3D(double pointX,
                                               double pointY,
                                               double pointZ,
                                               FramePoint3DReadOnly pointOnPlane,
                                               FrameVector3DReadOnly planeFirstTangent,
                                               FrameVector3DReadOnly planeSecondTangent)
   {
      return whichSideOfPlane3DIsPoint3DOn(pointX, pointY, pointZ, pointOnPlane, planeFirstTangent, planeSecondTangent) == Location.ABOVE;
   }

   /**
    * Returns a boolean value, stating if a 3D point is strictly above an infinitely large 3D plane.
    * The idea of "above" and "below" is determined based on the normal of the plane.
    * <p>
    * The plane's normal is retrieved using the two given tangents:<br>
    * <tt>planeNormal = planeFirstTangent &times; planeSecondTangent</tt><br>
    * Given the plane's normal, this method then calls
    * {@link EuclidGeometryTools#isPoint3DAboveOrBelowPlane3D(double, double, double, double, double, double, double, double, double, boolean)}.
    * </p>
    * <p>
    * This method will fail if the two given tangents are parallel.
    * </p>
    *
    * @param point              the coordinates of the query point.
    * @param pointOnPlane       the coordinates of a point positioned on the infinite plane. Not
    *                           modified.
    * @param planeFirstTangent  a first tangent of the infinite plane. Not modified.
    * @param planeSecondTangent a second tangent of the infinite plane. Not modified.
    * @return {@code true} if the point is strictly above the plane, {@code false} if the point is
    *         below or exactly on the plane.
    * @see EuclidGeometryTools#isPoint3DAboveOrBelowPlane3D(double, double, double, double, double,
    *      double, double, double, double, boolean)
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static boolean isPoint3DAbovePlane3D(FramePoint3DReadOnly point,
                                               FramePoint3DReadOnly pointOnPlane,
                                               FrameVector3DReadOnly planeFirstTangent,
                                               FrameVector3DReadOnly planeSecondTangent)
   {
      return whichSideOfPlane3DIsPoint3DOn(point, pointOnPlane, planeFirstTangent, planeSecondTangent) == Location.ABOVE;
   }

   /**
    * Returns a boolean value, stating if a 3D point is strictly below an infinitely large 3D plane.
    * The idea of "above" and "below" is determined based on the normal of the plane.
    * <p>
    * The plane's normal is retrieved using the two given tangents:<br>
    * <tt>planeNormal = planeFirstTangent &times; planeSecondTangent</tt><br>
    * Given the plane's normal, this method then calls
    * {@link EuclidGeometryTools#isPoint3DAboveOrBelowPlane3D(double, double, double, double, double, double, double, double, double, boolean)}.
    * </p>
    * <p>
    * This method will fail if the two given tangents are parallel.
    * </p>
    *
    * @param pointX             the x-coordinate of the query point.
    * @param pointY             the y-coordinate of the query point.
    * @param pointZ             the z-coordinate of the query point.
    * @param pointOnPlane       the coordinates of a point positioned on the infinite plane. Not
    *                           modified.
    * @param planeFirstTangent  a first tangent of the infinite plane. Not modified.
    * @param planeSecondTangent a second tangent of the infinite plane. Not modified.
    * @return {@code true} if the point is strictly below the plane, {@code false} if the point is
    *         above or exactly on the plane.
    * @see EuclidGeometryTools#isPoint3DAboveOrBelowPlane3D(double, double, double, double, double,
    *      double, double, double, double, boolean)
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static boolean isPoint3DBelowPlane3D(double pointX,
                                               double pointY,
                                               double pointZ,
                                               FramePoint3DReadOnly pointOnPlane,
                                               FrameVector3DReadOnly planeFirstTangent,
                                               FrameVector3DReadOnly planeSecondTangent)
   {
      return whichSideOfPlane3DIsPoint3DOn(pointX, pointY, pointZ, pointOnPlane, planeFirstTangent, planeSecondTangent) == Location.BELOW;
   }

   /**
    * Returns a boolean value, stating if a 3D point is strictly below an infinitely large 3D plane.
    * The idea of "above" and "below" is determined based on the normal of the plane.
    * <p>
    * The plane's normal is retrieved using the two given tangents:<br>
    * <tt>planeNormal = planeFirstTangent &times; planeSecondTangent</tt><br>
    * Given the plane's normal, this method then calls
    * {@link EuclidGeometryTools#isPoint3DAboveOrBelowPlane3D(double, double, double, double, double, double, double, double, double, boolean)}.
    * </p>
    * <p>
    * This method will fail if the two given tangents are parallel.
    * </p>
    *
    * @param point              the coordinates of the query point.
    * @param pointOnPlane       the coordinates of a point positioned on the infinite plane. Not
    *                           modified.
    * @param planeFirstTangent  a first tangent of the infinite plane. Not modified.
    * @param planeSecondTangent a second tangent of the infinite plane. Not modified.
    * @return {@code true} if the point is strictly below the plane, {@code false} if the point is
    *         above or exactly on the plane.
    * @see EuclidGeometryTools#isPoint3DAboveOrBelowPlane3D(double, double, double, double, double,
    *      double, double, double, double, boolean)
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static boolean isPoint3DBelowPlane3D(FramePoint3DReadOnly point,
                                               FramePoint3DReadOnly pointOnPlane,
                                               FrameVector3DReadOnly planeFirstTangent,
                                               FrameVector3DReadOnly planeSecondTangent)
   {
      return whichSideOfPlane3DIsPoint3DOn(point, pointOnPlane, planeFirstTangent, planeSecondTangent) == Location.BELOW;
   }

   /**
    * Computes the normal of a plane that is defined by three points.
    * <p>
    * Edge cases:
    * <ul>
    * <li>Returns a {@code null} if the three points are on a line.
    * <li>Returns {@code null} if two or three points are equal.
    * </ul>
    * </p>
    * <p>
    * The normal is computed such that the points' winding around it is counter-clockwise.
    * </p>
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param firstPointOnPlane  first point on the plane. Not modified.
    * @param secondPointOnPlane second point on the plane. Not modified.
    * @param thirdPointOnPlane  third point on the plane. Not modified.
    * @return the plane normal or {@code null} when the normal could not be determined.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static FrameVector3D normal3DFromThreePoint3Ds(FramePoint3DReadOnly firstPointOnPlane,
                                                         FramePoint3DReadOnly secondPointOnPlane,
                                                         FramePoint3DReadOnly thirdPointOnPlane)
   {
      firstPointOnPlane.checkReferenceFrameMatch(secondPointOnPlane, thirdPointOnPlane);
      Vector3D normal = EuclidGeometryTools.normal3DFromThreePoint3Ds(firstPointOnPlane, secondPointOnPlane, thirdPointOnPlane);
      if (normal == null)
         return null;
      else
         return new FrameVector3D(firstPointOnPlane.getReferenceFrame(), normal);
   }

   /**
    * Computes the normal of a plane that is defined by three points.
    * <p>
    * Edge cases:
    * <ul>
    * <li>Fails and returns {@code false} if the three points are on a line.
    * <li>Fails and returns {@code false} if two or three points are equal.
    * </ul>
    * </p>
    * <p>
    * The normal is computed such that the points' winding around it is counter-clockwise.
    * </p>
    *
    * @param firstPointOnPlane  first point on the plane. Not modified.
    * @param secondPointOnPlane second point on the plane. Not modified.
    * @param thirdPointOnPlane  third point on the plane. Not modified.
    * @param normalToPack       the vector in which the result is stored. Modified.
    * @return whether the plane normal is properly determined.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static boolean normal3DFromThreePoint3Ds(FramePoint3DReadOnly firstPointOnPlane,
                                                   FramePoint3DReadOnly secondPointOnPlane,
                                                   FramePoint3DReadOnly thirdPointOnPlane,
                                                   FixedFrameVector3DBasics normalToPack)
   {
      firstPointOnPlane.checkReferenceFrameMatch(secondPointOnPlane, thirdPointOnPlane, normalToPack);
      return EuclidGeometryTools.normal3DFromThreePoint3Ds(firstPointOnPlane, secondPointOnPlane, thirdPointOnPlane, normalToPack);
   }

   /**
    * Computes the normal of a plane that is defined by three points.
    * <p>
    * Edge cases:
    * <ul>
    * <li>Fails and returns {@code false} if the three points are on a line.
    * <li>Fails and returns {@code false} if two or three points are equal.
    * </ul>
    * </p>
    *
    * @param firstPointOnPlane  first point on the plane. Not modified.
    * @param secondPointOnPlane second point on the plane. Not modified.
    * @param thirdPointOnPlane  third point on the plane. Not modified.
    * @param normalToPack       the vector in which the result is stored. Modified.
    * @return whether the plane normal is properly determined.
    * @throws ReferenceFrameMismatchException if the read-only arguments are not all expressed in the
    *                                         same reference frame.
    */
   public static boolean normal3DFromThreePoint3Ds(FramePoint3DReadOnly firstPointOnPlane,
                                                   FramePoint3DReadOnly secondPointOnPlane,
                                                   FramePoint3DReadOnly thirdPointOnPlane,
                                                   FrameVector3DBasics normalToPack)
   {
      firstPointOnPlane.checkReferenceFrameMatch(secondPointOnPlane, thirdPointOnPlane);
      normalToPack.setReferenceFrame(firstPointOnPlane.getReferenceFrame());
      return EuclidGeometryTools.normal3DFromThreePoint3Ds(firstPointOnPlane, secondPointOnPlane, thirdPointOnPlane, normalToPack);
   }

   /**
    * Computes the orthogonal projection of a 2D point on an infinitely long 2D line defined by a 2D
    * line segment.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the two given points on the line are too close, i.e.
    * {@code firstPointOnLine.distanceSquared(secondPointOnLine) < }{@link EuclidGeometryTools#ONE_TRILLIONTH},
    * this method fails and returns {@code null}.
    * </ul>
    * </p>
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param pointToProject    the point to compute the projection of. Not modified.
    * @param firstPointOnLine  a first point located on the line. Not modified.
    * @param secondPointOnLine a second point located on the line. Not modified.
    * @return the projection of the point onto the line or {@code null} if the method failed.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static FramePoint2D orthogonalProjectionOnLine2D(FramePoint2DReadOnly pointToProject,
                                                           FramePoint2DReadOnly firstPointOnLine,
                                                           FramePoint2DReadOnly secondPointOnLine)
   {
      pointToProject.checkReferenceFrameMatch(firstPointOnLine, secondPointOnLine);
      Point2D projection = EuclidGeometryTools.orthogonalProjectionOnLine2D(pointToProject, firstPointOnLine, secondPointOnLine);
      if (projection == null)
         return null;
      else
         return new FramePoint2D(pointToProject.getReferenceFrame(), projection);
   }

   /**
    * Computes the orthogonal projection of a 2D point on an infinitely long 2D line defined by a 2D
    * line segment.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the two given points on the line are too close, i.e.
    * {@code firstPointOnLine.distanceSquared(secondPointOnLine) < }{@link EuclidGeometryTools#ONE_TRILLIONTH},
    * this method fails and returns {@code false}.
    * </ul>
    * </p>
    *
    * @param pointToProject    the point to compute the projection of. Not modified.
    * @param firstPointOnLine  a first point located on the line. Not modified.
    * @param secondPointOnLine a second point located on the line. Not modified.
    * @param projectionToPack  point in which the projection of the point onto the line is stored.
    *                          Modified.
    * @return whether the method succeeded or not.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static boolean orthogonalProjectionOnLine2D(FramePoint2DReadOnly pointToProject,
                                                      FramePoint2DReadOnly firstPointOnLine,
                                                      FramePoint2DReadOnly secondPointOnLine,
                                                      FixedFramePoint2DBasics projectionToPack)
   {
      pointToProject.checkReferenceFrameMatch(firstPointOnLine, secondPointOnLine, projectionToPack);
      return EuclidGeometryTools.orthogonalProjectionOnLine2D(pointToProject, firstPointOnLine, secondPointOnLine, projectionToPack);
   }

   /**
    * Computes the orthogonal projection of a 2D point on an infinitely long 2D line defined by a 2D
    * line segment.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the two given points on the line are too close, i.e.
    * {@code firstPointOnLine.distanceSquared(secondPointOnLine) < }{@link EuclidGeometryTools#ONE_TRILLIONTH},
    * this method fails and returns {@code false}.
    * </ul>
    * </p>
    *
    * @param pointToProject    the point to compute the projection of. Not modified.
    * @param firstPointOnLine  a first point located on the line. Not modified.
    * @param secondPointOnLine a second point located on the line. Not modified.
    * @param projectionToPack  point in which the projection of the point onto the line is stored.
    *                          Modified.
    * @return whether the method succeeded or not.
    * @throws ReferenceFrameMismatchException if the read-only arguments are not all expressed in the
    *                                         same reference frame.
    */
   public static boolean orthogonalProjectionOnLine2D(FramePoint2DReadOnly pointToProject,
                                                      FramePoint2DReadOnly firstPointOnLine,
                                                      FramePoint2DReadOnly secondPointOnLine,
                                                      FramePoint2DBasics projectionToPack)
   {
      pointToProject.checkReferenceFrameMatch(firstPointOnLine, secondPointOnLine);
      projectionToPack.setReferenceFrame(pointToProject.getReferenceFrame());
      return EuclidGeometryTools.orthogonalProjectionOnLine2D(pointToProject, firstPointOnLine, secondPointOnLine, projectionToPack);
   }

   /**
    * Computes the orthogonal projection of a 2D point on an infinitely long 2D line defined by a 2D
    * point and a 2D direction.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the given line direction is too small, i.e.
    * {@code lineDirection.lengthSquared() < }{@link EuclidGeometryTools#ONE_TRILLIONTH}, this method
    * fails and returns {@code null}.
    * </ul>
    * </p>
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param pointToProject the point to compute the projection of. Not modified.
    * @param pointOnLine    a point located on the line. Not modified.
    * @param lineDirection  the direction of the line. Not modified.
    * @return the projection of the point onto the line or {@code null} if the method failed.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static FramePoint2D orthogonalProjectionOnLine2D(FramePoint2DReadOnly pointToProject,
                                                           FramePoint2DReadOnly pointOnLine,
                                                           FrameVector2DReadOnly lineDirection)
   {
      pointToProject.checkReferenceFrameMatch(pointOnLine, lineDirection);

      Point2D projection = EuclidGeometryTools.orthogonalProjectionOnLine2D(pointToProject, pointOnLine, lineDirection);
      if (projection == null)
         return null;
      else
         return new FramePoint2D(pointToProject.getReferenceFrame(), projection);
   }

   /**
    * Computes the orthogonal projection of a 2D point on an infinitely long 2D line defined by a 2D
    * point and a 2D direction.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the given line direction is too small, i.e.
    * {@code lineDirection.lengthSquared() < }{@link EuclidGeometryTools#ONE_TRILLIONTH}, this method
    * fails and returns {@code false}.
    * </ul>
    * </p>
    *
    * @param pointToProject   the point to compute the projection of. Not modified.
    * @param pointOnLine      a point located on the line. Not modified.
    * @param lineDirection    the direction of the line. Not modified.
    * @param projectionToPack point in which the projection of the point onto the line is stored.
    *                         Modified.
    * @return whether the method succeeded or not.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static boolean orthogonalProjectionOnLine2D(FramePoint2DReadOnly pointToProject,
                                                      FramePoint2DReadOnly pointOnLine,
                                                      FrameVector2DReadOnly lineDirection,
                                                      FixedFramePoint2DBasics projectionToPack)
   {
      pointToProject.checkReferenceFrameMatch(pointOnLine, lineDirection, projectionToPack);
      return EuclidGeometryTools.orthogonalProjectionOnLine2D(pointToProject, pointOnLine, lineDirection, projectionToPack);
   }

   /**
    * Computes the orthogonal projection of a 2D point on an infinitely long 2D line defined by a 2D
    * point and a 2D direction.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the given line direction is too small, i.e.
    * {@code lineDirection.lengthSquared() < }{@link EuclidGeometryTools#ONE_TRILLIONTH}, this method
    * fails and returns {@code false}.
    * </ul>
    * </p>
    *
    * @param pointToProject   the point to compute the projection of. Not modified.
    * @param pointOnLine      a point located on the line. Not modified.
    * @param lineDirection    the direction of the line. Not modified.
    * @param projectionToPack point in which the projection of the point onto the line is stored.
    *                         Modified.
    * @return whether the method succeeded or not.
    * @throws ReferenceFrameMismatchException if the read-only arguments are not all expressed in the
    *                                         same reference frame.
    */
   public static boolean orthogonalProjectionOnLine2D(FramePoint2DReadOnly pointToProject,
                                                      FramePoint2DReadOnly pointOnLine,
                                                      FrameVector2DReadOnly lineDirection,
                                                      FramePoint2DBasics projectionToPack)
   {
      pointToProject.checkReferenceFrameMatch(pointOnLine, lineDirection);
      projectionToPack.setReferenceFrame(pointToProject.getReferenceFrame());
      return EuclidGeometryTools.orthogonalProjectionOnLine2D(pointToProject, pointOnLine, lineDirection, projectionToPack);
   }

   /**
    * Computes the orthogonal projection of a 3D point on an infinitely long 3D line defined by a 3D
    * point and a 3D direction.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the given line direction is too small, i.e.
    * {@code lineDirection.lengthSquared() < }{@link EuclidGeometryTools#ONE_TRILLIONTH}, this method
    * fails and returns {@code null}.
    * </ul>
    * </p>
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param pointToProject the point to compute the projection of. Not modified.
    * @param pointOnLine    point located on the line. Not modified.
    * @param lineDirection  direction of the line. Not modified.
    * @return the projection of the point onto the line or {@code null} if the method failed.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static FramePoint3D orthogonalProjectionOnLine3D(FramePoint3DReadOnly pointToProject,
                                                           FramePoint3DReadOnly pointOnLine,
                                                           FrameVector3DReadOnly lineDirection)
   {
      pointToProject.checkReferenceFrameMatch(pointOnLine, lineDirection);

      Point3D projection = EuclidGeometryTools.orthogonalProjectionOnLine3D(pointToProject, pointOnLine, lineDirection);
      if (projection == null)
         return null;
      else
         return new FramePoint3D(pointToProject.getReferenceFrame(), projection);
   }

   /**
    * Computes the orthogonal projection of a 3D point on an infinitely long 3D line defined by a 3D
    * point and a 3D direction.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the given line direction is too small, i.e.
    * {@code lineDirection.lengthSquared() < }{@link EuclidGeometryTools#ONE_TRILLIONTH}, this method
    * fails and returns {@code false}.
    * </ul>
    * </p>
    *
    * @param pointToProject   the point to compute the projection of. Not modified.
    * @param pointOnLine      point located on the line. Not modified.
    * @param lineDirection    direction of the line. Not modified.
    * @param projectionToPack point in which the projection of the point onto the line is stored.
    *                         Modified.
    * @return whether the method succeeded or not.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static boolean orthogonalProjectionOnLine3D(FramePoint3DReadOnly pointToProject,
                                                      FramePoint3DReadOnly pointOnLine,
                                                      FrameVector3DReadOnly lineDirection,
                                                      FixedFramePoint3DBasics projectionToPack)
   {
      pointToProject.checkReferenceFrameMatch(pointOnLine, lineDirection, projectionToPack);
      return EuclidGeometryTools.orthogonalProjectionOnLine3D(pointToProject, pointOnLine, lineDirection, projectionToPack);
   }

   /**
    * Computes the orthogonal projection of a 3D point on an infinitely long 3D line defined by a 3D
    * point and a 3D direction.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the given line direction is too small, i.e.
    * {@code lineDirection.lengthSquared() < }{@link EuclidGeometryTools#ONE_TRILLIONTH}, this method
    * fails and returns {@code false}.
    * </ul>
    * </p>
    *
    * @param pointToProject   the point to compute the projection of. Not modified.
    * @param pointOnLine      point located on the line. Not modified.
    * @param lineDirection    direction of the line. Not modified.
    * @param projectionToPack point in which the projection of the point onto the line is stored.
    *                         Modified.
    * @return whether the method succeeded or not.
    * @throws ReferenceFrameMismatchException if the read-only arguments are not all expressed in the
    *                                         same reference frame.
    */
   public static boolean orthogonalProjectionOnLine3D(FramePoint3DReadOnly pointToProject,
                                                      FramePoint3DReadOnly pointOnLine,
                                                      FrameVector3DReadOnly lineDirection,
                                                      FramePoint3DBasics projectionToPack)
   {
      pointToProject.checkReferenceFrameMatch(pointOnLine, lineDirection);
      projectionToPack.setReferenceFrame(pointToProject.getReferenceFrame());
      return EuclidGeometryTools.orthogonalProjectionOnLine3D(pointToProject, pointOnLine, lineDirection, projectionToPack);
   }

   /**
    * Computes the orthogonal projection of a 2D point on a given 2D line segment defined by its two 2D
    * endpoints.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the length of the given line segment is too small, i.e.
    * {@code lineSegmentStart.distanceSquared(lineSegmentEnd) < }{@link EuclidGeometryTools#ONE_TRILLIONTH},
    * this method returns {@code lineSegmentStart}.
    * <li>the projection can not be outside the line segment. When the projection on the corresponding
    * line is outside the line segment, the result is the closest of the two endpoints.
    * </ul>
    * </p>
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param pointToProject   the point to compute the projection of. Not modified.
    * @param lineSegmentStart the line segment first endpoint. Not modified.
    * @param lineSegmentEnd   the line segment second endpoint. Not modified.
    * @return the projection of the point onto the line segment or {@code null} if the method failed.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static FramePoint2D orthogonalProjectionOnLineSegment2D(FramePoint2DReadOnly pointToProject,
                                                                  FramePoint2DReadOnly lineSegmentStart,
                                                                  FramePoint2DReadOnly lineSegmentEnd)
   {
      pointToProject.checkReferenceFrameMatch(lineSegmentStart, lineSegmentEnd);

      Point2D projection = EuclidGeometryTools.orthogonalProjectionOnLineSegment2D(pointToProject, lineSegmentStart, lineSegmentEnd);
      if (projection == null)
         return null;
      else
         return new FramePoint2D(pointToProject.getReferenceFrame(), projection);
   }

   /**
    * Computes the orthogonal projection of a 2D point on a given 2D line segment defined by its two 2D
    * endpoints.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the length of the given line segment is too small, i.e.
    * {@code lineSegmentStart.distanceSquared(lineSegmentEnd) < }{@link EuclidGeometryTools#ONE_TRILLIONTH},
    * this method returns {@code lineSegmentStart}.
    * <li>the projection can not be outside the line segment. When the projection on the corresponding
    * line is outside the line segment, the result is the closest of the two endpoints.
    * </ul>
    * </p>
    *
    * @param pointToProjectX  the x-coordinate of the point to compute the projection of.
    * @param pointToProjectY  the y-coordinate of the point to compute the projection of.
    * @param lineSegmentStart the line segment first endpoint. Not modified.
    * @param lineSegmentEnd   the line segment second endpoint. Not modified.
    * @param projectionToPack point in which the projection of the point onto the line segment is
    *                         stored. Modified.
    * @return whether the method succeeded or not.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static boolean orthogonalProjectionOnLineSegment2D(double pointToProjectX,
                                                             double pointToProjectY,
                                                             FramePoint2DReadOnly lineSegmentStart,
                                                             FramePoint2DReadOnly lineSegmentEnd,
                                                             FixedFramePoint2DBasics projectionToPack)
   {
      lineSegmentStart.checkReferenceFrameMatch(lineSegmentEnd, projectionToPack);
      return EuclidGeometryTools.orthogonalProjectionOnLineSegment2D(pointToProjectX, pointToProjectY, lineSegmentStart, lineSegmentEnd, projectionToPack);
   }

   /**
    * Computes the orthogonal projection of a 2D point on a given 2D line segment defined by its two 2D
    * endpoints.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the length of the given line segment is too small, i.e.
    * {@code lineSegmentStart.distanceSquared(lineSegmentEnd) < }{@link EuclidGeometryTools#ONE_TRILLIONTH},
    * this method returns {@code lineSegmentStart}.
    * <li>the projection can not be outside the line segment. When the projection on the corresponding
    * line is outside the line segment, the result is the closest of the two endpoints.
    * </ul>
    * </p>
    *
    * @param pointToProjectX  the x-coordinate of the point to compute the projection of.
    * @param pointToProjectY  the y-coordinate of the point to compute the projection of.
    * @param lineSegmentStart the line segment first endpoint. Not modified.
    * @param lineSegmentEnd   the line segment second endpoint. Not modified.
    * @param projectionToPack point in which the projection of the point onto the line segment is
    *                         stored. Modified.
    * @return whether the method succeeded or not.
    * @throws ReferenceFrameMismatchException if the read-only arguments are not all expressed in the
    *                                         same reference frame.
    */
   public static boolean orthogonalProjectionOnLineSegment2D(double pointToProjectX,
                                                             double pointToProjectY,
                                                             FramePoint2DReadOnly lineSegmentStart,
                                                             FramePoint2DReadOnly lineSegmentEnd,
                                                             FramePoint2DBasics projectionToPack)
   {
      lineSegmentStart.checkReferenceFrameMatch(lineSegmentEnd);
      projectionToPack.setReferenceFrame(lineSegmentStart.getReferenceFrame());
      return EuclidGeometryTools.orthogonalProjectionOnLineSegment2D(pointToProjectX, pointToProjectY, lineSegmentStart, lineSegmentEnd, projectionToPack);
   }

   /**
    * Computes the orthogonal projection of a 2D point on a given 2D line segment defined by its two 2D
    * endpoints.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the length of the given line segment is too small, i.e.
    * {@code lineSegmentStart.distanceSquared(lineSegmentEnd) < }{@link EuclidGeometryTools#ONE_TRILLIONTH},
    * this method returns {@code lineSegmentStart}.
    * <li>the projection can not be outside the line segment. When the projection on the corresponding
    * line is outside the line segment, the result is the closest of the two endpoints.
    * </ul>
    * </p>
    *
    * @param pointToProject   the point to compute the projection of. Not modified.
    * @param lineSegmentStart the line segment first endpoint. Not modified.
    * @param lineSegmentEnd   the line segment second endpoint. Not modified.
    * @param projectionToPack point in which the projection of the point onto the line segment is
    *                         stored. Modified.
    * @return whether the method succeeded or not.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static boolean orthogonalProjectionOnLineSegment2D(FramePoint2DReadOnly pointToProject,
                                                             FramePoint2DReadOnly lineSegmentStart,
                                                             FramePoint2DReadOnly lineSegmentEnd,
                                                             FixedFramePoint2DBasics projectionToPack)
   {
      pointToProject.checkReferenceFrameMatch(lineSegmentStart, lineSegmentEnd, projectionToPack);
      return EuclidGeometryTools.orthogonalProjectionOnLineSegment2D(pointToProject, lineSegmentStart, lineSegmentEnd, projectionToPack);
   }

   /**
    * Computes the orthogonal projection of a 2D point on a given 2D line segment defined by its two 2D
    * endpoints.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the length of the given line segment is too small, i.e.
    * {@code lineSegmentStart.distanceSquared(lineSegmentEnd) < }{@link EuclidGeometryTools#ONE_TRILLIONTH},
    * this method returns {@code lineSegmentStart}.
    * <li>the projection can not be outside the line segment. When the projection on the corresponding
    * line is outside the line segment, the result is the closest of the two endpoints.
    * </ul>
    * </p>
    *
    * @param pointToProject   the point to compute the projection of. Not modified.
    * @param lineSegmentStart the line segment first endpoint. Not modified.
    * @param lineSegmentEnd   the line segment second endpoint. Not modified.
    * @param projectionToPack point in which the projection of the point onto the line segment is
    *                         stored. Modified.
    * @return whether the method succeeded or not.
    * @throws ReferenceFrameMismatchException if the read-only arguments are not all expressed in the
    *                                         same reference frame.
    */
   public static boolean orthogonalProjectionOnLineSegment2D(FramePoint2DReadOnly pointToProject,
                                                             FramePoint2DReadOnly lineSegmentStart,
                                                             FramePoint2DReadOnly lineSegmentEnd,
                                                             FramePoint2DBasics projectionToPack)
   {
      pointToProject.checkReferenceFrameMatch(lineSegmentStart, lineSegmentEnd);
      projectionToPack.setReferenceFrame(pointToProject.getReferenceFrame());
      return EuclidGeometryTools.orthogonalProjectionOnLineSegment2D(pointToProject, lineSegmentStart, lineSegmentEnd, projectionToPack);
   }

   /**
    * Computes the orthogonal projection of a 3D point on a given 3D line segment defined by its two 3D
    * endpoints.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the length of the given line segment is too small, i.e.
    * {@code lineSegmentStart.distanceSquared(lineSegmentEnd) < }{@link EuclidGeometryTools#ONE_TRILLIONTH},
    * this method returns {@code lineSegmentStart}.
    * <li>the projection can not be outside the line segment. When the projection on the corresponding
    * line is outside the line segment, the result is the closest of the two endpoints.
    * </ul>
    * </p>
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param pointToProject   the point to compute the projection of. Not modified.
    * @param lineSegmentStart the line segment first endpoint. Not modified.
    * @param lineSegmentEnd   the line segment second endpoint. Not modified.
    * @return the projection of the point onto the line segment or {@code null} if the method failed.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static FramePoint3D orthogonalProjectionOnLineSegment3D(FramePoint3DReadOnly pointToProject,
                                                                  FramePoint3DReadOnly lineSegmentStart,
                                                                  FramePoint3DReadOnly lineSegmentEnd)
   {
      pointToProject.checkReferenceFrameMatch(lineSegmentStart, lineSegmentEnd);
      Point3D projection = EuclidGeometryTools.orthogonalProjectionOnLineSegment3D(pointToProject, lineSegmentStart, lineSegmentEnd);
      if (projection == null)
         return null;
      else
         return new FramePoint3D(pointToProject.getReferenceFrame(), projection);
   }

   /**
    * Computes the orthogonal projection of a 3D point on a given 3D line segment defined by its two 3D
    * endpoints.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the length of the given line segment is too small, i.e.
    * {@code lineSegmentStart.distanceSquared(lineSegmentEnd) < }{@link EuclidGeometryTools#ONE_TRILLIONTH},
    * this method returns {@code lineSegmentStart}.
    * <li>the projection can not be outside the line segment. When the projection on the corresponding
    * line is outside the line segment, the result is the closest of the two endpoints.
    * </ul>
    * </p>
    *
    * @param pointToProject   the point to compute the projection of. Not modified.
    * @param lineSegmentStart the line segment first endpoint. Not modified.
    * @param lineSegmentEnd   the line segment second endpoint. Not modified.
    * @param projectionToPack point in which the projection of the point onto the line segment is
    *                         stored. Modified.
    * @return whether the method succeeded or not.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static boolean orthogonalProjectionOnLineSegment3D(FramePoint3DReadOnly pointToProject,
                                                             FramePoint3DReadOnly lineSegmentStart,
                                                             FramePoint3DReadOnly lineSegmentEnd,
                                                             FixedFramePoint3DBasics projectionToPack)
   {
      pointToProject.checkReferenceFrameMatch(lineSegmentStart, lineSegmentEnd, projectionToPack);
      return EuclidGeometryTools.orthogonalProjectionOnLineSegment3D(pointToProject, lineSegmentStart, lineSegmentEnd, projectionToPack);
   }

   /**
    * Computes the orthogonal projection of a 3D point on a given 3D line segment defined by its two 3D
    * endpoints.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the length of the given line segment is too small, i.e.
    * {@code lineSegmentStart.distanceSquared(lineSegmentEnd) < }{@link EuclidGeometryTools#ONE_TRILLIONTH},
    * this method returns {@code lineSegmentStart}.
    * <li>the projection can not be outside the line segment. When the projection on the corresponding
    * line is outside the line segment, the result is the closest of the two endpoints.
    * </ul>
    * </p>
    *
    * @param pointToProject   the point to compute the projection of. Not modified.
    * @param lineSegmentStart the line segment first endpoint. Not modified.
    * @param lineSegmentEnd   the line segment second endpoint. Not modified.
    * @param projectionToPack point in which the projection of the point onto the line segment is
    *                         stored. Modified.
    * @return whether the method succeeded or not.
    * @throws ReferenceFrameMismatchException if the read-only arguments are not all expressed in the
    *                                         same reference frame.
    */
   public static boolean orthogonalProjectionOnLineSegment3D(FramePoint3DReadOnly pointToProject,
                                                             FramePoint3DReadOnly lineSegmentStart,
                                                             FramePoint3DReadOnly lineSegmentEnd,
                                                             FramePoint3DBasics projectionToPack)
   {
      pointToProject.checkReferenceFrameMatch(lineSegmentStart, lineSegmentEnd);
      projectionToPack.setReferenceFrame(pointToProject.getReferenceFrame());
      return EuclidGeometryTools.orthogonalProjectionOnLineSegment3D(pointToProject, lineSegmentStart, lineSegmentEnd, projectionToPack);
   }

   /**
    * Computes the orthogonal projection of a 3D point on a given 3D plane defined by a 3D point and 3D
    * normal.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the length of the plane normal is too small, i.e. less than
    * {@link EuclidGeometryTools#ONE_TRILLIONTH}, this method fails and returns {@code false}.
    * </ul>
    * </p>
    *
    * @param pointToProject the point to compute the projection of. Not modified.
    * @param pointOnPlane   a point on the plane. Not modified.
    * @param planeNormal    the normal of the plane. Not modified.
    * @return the projection of the point onto the plane, or {@code null} if the method failed.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static FramePoint3D orthogonalProjectionOnPlane3D(FramePoint3DReadOnly pointToProject,
                                                            FramePoint3DReadOnly pointOnPlane,
                                                            FrameVector3DReadOnly planeNormal)
   {
      pointToProject.checkReferenceFrameMatch(pointOnPlane, planeNormal);
      Point3D projection = EuclidGeometryTools.orthogonalProjectionOnPlane3D(pointToProject, pointOnPlane, planeNormal);
      if (projection == null)
         return null;
      else
         return new FramePoint3D(pointOnPlane.getReferenceFrame(), projection);
   }

   /**
    * Computes the orthogonal projection of a 3D point on a given 3D plane defined by a 3D point and 3D
    * normal.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the length of the plane normal is too small, i.e. less than
    * {@link EuclidGeometryTools#ONE_TRILLIONTH}, this method fails and returns {@code false}.
    * </ul>
    * </p>
    *
    * @param pointToProject   the point to compute the projection of. Not modified.
    * @param pointOnPlane     a point on the plane. Not modified.
    * @param planeNormal      the normal of the plane. Not modified.
    * @param projectionToPack point in which the projection of the point onto the plane is stored.
    *                         Modified.
    * @return whether the method succeeded or not.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static boolean orthogonalProjectionOnPlane3D(FramePoint3DReadOnly pointToProject,
                                                       FramePoint3DReadOnly pointOnPlane,
                                                       FrameVector3DReadOnly planeNormal,
                                                       FixedFramePoint3DBasics projectionToPack)
   {
      pointToProject.checkReferenceFrameMatch(pointOnPlane, planeNormal, projectionToPack);
      return EuclidGeometryTools.orthogonalProjectionOnPlane3D(pointToProject, pointOnPlane, planeNormal, projectionToPack);
   }

   /**
    * Computes the orthogonal projection of a 3D point on a given 3D plane defined by a 3D point and 3D
    * normal.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the length of the plane normal is too small, i.e. less than
    * {@link EuclidGeometryTools#ONE_TRILLIONTH}, this method fails and returns {@code false}.
    * </ul>
    * </p>
    *
    * @param pointToProject   the point to compute the projection of. Not modified.
    * @param pointOnPlane     a point on the plane. Not modified.
    * @param planeNormal      the normal of the plane. Not modified.
    * @param projectionToPack point in which the projection of the point onto the plane is stored.
    *                         Modified.
    * @return whether the method succeeded or not.
    * @throws ReferenceFrameMismatchException if the read-only arguments are not all expressed in the
    *                                         same reference frame.
    */
   public static boolean orthogonalProjectionOnPlane3D(FramePoint3DReadOnly pointToProject,
                                                       FramePoint3DReadOnly pointOnPlane,
                                                       FrameVector3DReadOnly planeNormal,
                                                       FramePoint3DBasics projectionToPack)
   {
      pointToProject.checkReferenceFrameMatch(pointOnPlane, planeNormal);
      projectionToPack.setReferenceFrame(pointToProject.getReferenceFrame());
      return EuclidGeometryTools.orthogonalProjectionOnPlane3D(pointToProject, pointOnPlane, planeNormal, projectionToPack);
   }

   /**
    * Computes the orthogonal projection of a 3D point on a given 3D plane defined by a 3D point and 3D
    * normal.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the length of the plane normal is too small, i.e. less than
    * {@link EuclidGeometryTools#ONE_TRILLIONTH}, this method fails and returns {@code false}.
    * </ul>
    * </p>
    *
    * @param x                the x-coordinate of the point to compute the projection of. Not modified.
    * @param y                the y-coordinate of the point to compute the projection of. Not modified.
    * @param z                the z-coordinate of the point to compute the projection of. Not modified.
    * @param pointOnPlane     a point on the plane. Not modified.
    * @param planeNormal      the normal of the plane. Not modified.
    * @param projectionToPack point in which the projection of the point onto the plane is stored.
    *                         Modified.
    * @return whether the method succeeded or not.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static boolean orthogonalProjectionOnPlane3D(double x,
                                                       double y,
                                                       double z,
                                                       FramePoint3DReadOnly pointOnPlane,
                                                       FrameVector3DReadOnly planeNormal,
                                                       FixedFramePoint3DBasics projectionToPack)
   {
      pointOnPlane.checkReferenceFrameMatch(planeNormal, projectionToPack);
      return EuclidGeometryTools.orthogonalProjectionOnPlane3D(x, y, z, pointOnPlane, planeNormal, projectionToPack);
   }

   /**
    * Computes the orthogonal projection of a 3D point on a given 3D plane defined by a 3D point and 3D
    * normal.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the length of the plane normal is too small, i.e. less than
    * {@link EuclidGeometryTools#ONE_TRILLIONTH}, this method fails and returns {@code false}.
    * </ul>
    * </p>
    *
    * @param x                the x-coordinate of the point to compute the projection of. Not modified.
    * @param y                the y-coordinate of the point to compute the projection of. Not modified.
    * @param z                the z-coordinate of the point to compute the projection of. Not modified.
    * @param pointOnPlane     a point on the plane. Not modified.
    * @param planeNormal      the normal of the plane. Not modified.
    * @param projectionToPack point in which the projection of the point onto the plane is stored.
    *                         Modified.
    * @return whether the method succeeded or not.
    * @throws ReferenceFrameMismatchException if the read-only arguments are not all expressed in the
    *                                         same reference frame.
    */
   public static boolean orthogonalProjectionOnPlane3D(double x,
                                                       double y,
                                                       double z,
                                                       FramePoint3DReadOnly pointOnPlane,
                                                       FrameVector3DReadOnly planeNormal,
                                                       FramePoint3DBasics projectionToPack)
   {
      pointOnPlane.checkReferenceFrameMatch(planeNormal);
      projectionToPack.setReferenceFrame(pointOnPlane.getReferenceFrame());
      return EuclidGeometryTools.orthogonalProjectionOnPlane3D(x, y, z, pointOnPlane, planeNormal, projectionToPack);
   }

   /**
    * Computes the intersection between two infinitely long 2D lines each defined by a 2D point and a
    * 2D direction and returns a percentage {@code alpha} along the first line such that the
    * intersection coordinates can be computed as follows: <br>
    * {@code intersection = pointOnLine1 + alpha * lineDirection1}
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the two lines are parallel but not collinear, the two lines do not intersect and the
    * returned value is {@link Double#NaN}.
    * <li>if the two lines are collinear, this method returns {@link Double#POSITIVE_INFINITY}.
    * </ul>
    * </p>
    *
    * @param pointOnLine1   a point located on the first line. Not modified.
    * @param lineDirection1 the first line direction. Not modified.
    * @param pointOnLine2   a point located on the second line. Not modified.
    * @param lineDirection2 the second line direction. Not modified.
    * @return {@code alpha} the percentage along the first line of the intersection location. This
    *         method returns {@link Double#NaN} if the lines do not intersect.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static double percentageOfIntersectionBetweenTwoLine2Ds(FramePoint2DReadOnly pointOnLine1,
                                                                  FrameVector2DReadOnly lineDirection1,
                                                                  FramePoint2DReadOnly pointOnLine2,
                                                                  FrameVector2DReadOnly lineDirection2)
   {
      pointOnLine1.checkReferenceFrameMatch(lineDirection1, pointOnLine2, lineDirection2);
      return EuclidGeometryTools.percentageOfIntersectionBetweenTwoLine2Ds(pointOnLine1, lineDirection1, pointOnLine2, lineDirection2);
   }

   /**
    * Computes the intersection between a 2D line segment and an infinitely long 2D line and returns a
    * percentage {@code alpha} along the line segment such that the intersection coordinates can be
    * computed as follows: <br>
    * {@code intersection = (1.0 - alpha) * lineSegmentStart + alpha * lineSegmentEnd}
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the line segment and the line do not intersect, the method returns {@link Double#NaN}.
    * <li>if the intersection is outside the line segment's endpoints, the line segment and the line do
    * not intersect.
    * <li>if the line segment and the line are parallel but not collinear, they do not intersect and
    * the returned value is {@link Double#NaN}.
    * <li>if the line segment and the line are collinear, this method returns
    * {@link Double#POSITIVE_INFINITY}.
    * </ul>
    * </p>
    *
    * @param lineSegmentStart the line segment first endpoint. Not modified.
    * @param lineSegmentEnd   the line segment second endpoint. Not modified.
    * @param pointOnLine      a point located on the line. Not modified.
    * @param lineDirection    the line direction. Not modified.
    * @return {@code alpha} the percentage along the line segment of the intersection location. This
    *         method returns {@link Double#NaN} if the line segment and the line do not intersect.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static double percentageOfIntersectionBetweenLineSegment2DAndLine2D(FramePoint2DReadOnly lineSegmentStart,
                                                                              FramePoint2DReadOnly lineSegmentEnd,
                                                                              FramePoint2DReadOnly pointOnLine,
                                                                              FrameVector2DReadOnly lineDirection)
   {
      lineSegmentStart.checkReferenceFrameMatch(lineSegmentEnd, pointOnLine, lineDirection);
      return EuclidGeometryTools.percentageOfIntersectionBetweenLineSegment2DAndLine2D(lineSegmentStart, lineSegmentEnd, pointOnLine, lineDirection);
   }

   /**
    * Computes a percentage along the line representing the location of the given point once projected
    * onto the line. The returned percentage is in ] -&infin;; &infin; [, {@code 0.0} representing
    * {@code pointOnLine}, and for any given {@code point} the percentage {@code alpha} is computed
    * such that:<br>
    * {@code point = pointOnLine + alpha * lineDirection}.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the length of the line direction is too small, i.e.
    * {@code lineDirection.leangthSquared() < }{@value EuclidGeometryTools#ONE_TRILLIONTH}, this method
    * fails and returns {@code 0.0}.
    * </ul>
    * </p>
    *
    * @param point         the coordinates of the query point.
    * @param pointOnLine   a point located on the line. Not modified.
    * @param lineDirection the direction of the line. Not modified.
    * @return the computed percentage along the line representing where the point projection is
    *         located.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static double percentageAlongLine2D(FramePoint2DReadOnly point, FramePoint2DReadOnly pointOnLine, FrameVector2DReadOnly lineDirection)
   {
      point.checkReferenceFrameMatch(pointOnLine, lineDirection);
      return EuclidGeometryTools.percentageAlongLine2D(point, pointOnLine, lineDirection);
   }

   /**
    * Computes a percentage along the line representing the location of the given point once projected
    * onto the line. The returned percentage is in ] -&infin;; &infin; [, {@code 0.0} representing
    * {@code pointOnLine}, and for any given {@code point} the percentage {@code alpha} is computed
    * such that:<br>
    * {@code point = pointOnLine + alpha * lineDirection}.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the length of the line direction is too small, i.e.
    * {@code lineDirection.leangthSquared() < }{@value EuclidGeometryTools#ONE_TRILLIONTH}, this method
    * fails and returns {@code 0.0}.
    * </ul>
    * </p>
    *
    * @param pointX        the x-coordinate of the query point.
    * @param pointY        the y-coordinate of the query point.
    * @param pointOnLine   a point located on the line. Not modified.
    * @param lineDirection the direction of the line. Not modified.
    * @return the computed percentage along the line representing where the point projection is
    *         located.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static double percentageAlongLine2D(double pointX, double pointY, FramePoint2DReadOnly pointOnLine, FrameVector2DReadOnly lineDirection)
   {
      pointOnLine.checkReferenceFrameMatch(lineDirection);
      return EuclidGeometryTools.percentageAlongLine2D(pointX, pointY, pointOnLine, lineDirection);
   }

   /**
    * Computes a percentage along the line segment representing the location of the projection onto the
    * line segment of the given point. The returned percentage is in ] -&infin;; &infin; [, {@code 0.0}
    * representing {@code lineSegmentStart}, and {@code 1.0} representing {@code lineSegmentEnd}.
    * <p>
    * For example, if the returned percentage is {@code 0.5}, it means that the projection of the given
    * point is located at the middle of the line segment. The coordinates of the projection of the
    * point can be computed from the {@code percentage} as follows: <code>
    * Point2d projection = new Point2d(); </br>
    * projection.interpolate(lineSegmentStart, lineSegmentEnd, percentage); </br>
    * </code>
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the length of the given line segment is too small, i.e.
    * {@code lineSegmentStart.distanceSquared(lineSegmentEnd) < }{@link EuclidGeometryTools#ONE_TRILLIONTH},
    * this method fails and returns {@code 0.0}.
    * </ul>
    * </p>
    *
    * @param pointX           the x-coordinate of the query point.
    * @param pointY           the y-coordinate of the query point.
    * @param lineSegmentStart the line segment first endpoint. Not modified.
    * @param lineSegmentEnd   the line segment second endpoint. Not modified.
    * @return the computed percentage along the line segment representing where the point projection is
    *         located.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static double percentageAlongLineSegment2D(double pointX, double pointY, FramePoint2DReadOnly lineSegmentStart, FramePoint2DReadOnly lineSegmentEnd)
   {
      lineSegmentStart.checkReferenceFrameMatch(lineSegmentEnd);
      return EuclidGeometryTools.percentageAlongLineSegment2D(pointX, pointY, lineSegmentStart, lineSegmentEnd);
   }

   /**
    * Computes a percentage along the line segment representing the location of the projection onto the
    * line segment of the given point. The returned percentage is in ] -&infin;; &infin; [, {@code 0.0}
    * representing {@code lineSegmentStart}, and {@code 1.0} representing {@code lineSegmentEnd}.
    * <p>
    * For example, if the returned percentage is {@code 0.5}, it means that the projection of the given
    * point is located at the middle of the line segment. The coordinates of the projection of the
    * point can be computed from the {@code percentage} as follows: <code>
    * Point2d projection = new Point2d(); </br>
    * projection.interpolate(lineSegmentStart, lineSegmentEnd, percentage); </br>
    * </code>
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the length of the given line segment is too small, i.e.
    * {@code lineSegmentStart.distanceSquared(lineSegmentEnd) < }{@link EuclidGeometryTools#ONE_TRILLIONTH},
    * this method fails and returns {@code 0.0}.
    * </ul>
    * </p>
    *
    * @param point            the query. Not modified.
    * @param lineSegmentStart the line segment first endpoint. Not modified.
    * @param lineSegmentEnd   the line segment second endpoint. Not modified.
    * @return the computed percentage along the line segment representing where the point projection is
    *         located.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static double percentageAlongLineSegment2D(FramePoint2DReadOnly point, FramePoint2DReadOnly lineSegmentStart, FramePoint2DReadOnly lineSegmentEnd)
   {
      point.checkReferenceFrameMatch(lineSegmentStart, lineSegmentEnd);
      return EuclidGeometryTools.percentageAlongLineSegment2D(point, lineSegmentStart, lineSegmentEnd);
   }

   /**
    * Computes a percentage along the line representing the location of the given point once projected
    * onto the line. The returned percentage is in ] -&infin;; &infin; [, {@code 0.0} representing
    * {@code pointOnLine}, and for any given {@code point} the percentage {@code alpha} is computed
    * such that:<br>
    * {@code point = pointOnLine + alpha * lineDirection}.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the length of the line direction is too small, i.e.
    * {@code lineDirection.leangthSquared() < }{@value EuclidGeometryTools#ONE_TRILLIONTH}, this method
    * fails and returns {@code 0.0}.
    * </ul>
    * </p>
    *
    * @param point         the coordinates of the query point.
    * @param pointOnLine   a point located on the line. Not modified.
    * @param lineDirection the direction of the line. Not modified.
    * @return the computed percentage along the line representing where the point projection is
    *         located.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static double percentageAlongLine3D(FramePoint3DReadOnly point, FramePoint3DReadOnly pointOnLine, FrameVector3DReadOnly lineDirection)
   {
      point.checkReferenceFrameMatch(pointOnLine, lineDirection);
      return EuclidGeometryTools.percentageAlongLine3D(point, pointOnLine, lineDirection);
   }

   /**
    * Computes a percentage along the line representing the location of the given point once projected
    * onto the line. The returned percentage is in ] -&infin;; &infin; [, {@code 0.0} representing
    * {@code pointOnLine}, and for any given {@code point} the percentage {@code alpha} is computed
    * such that:<br>
    * {@code point = pointOnLine + alpha * lineDirection}.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the length of the line direction is too small, i.e.
    * {@code lineDirection.leangthSquared() < }{@value EuclidGeometryTools#ONE_TRILLIONTH}, this method
    * fails and returns {@code 0.0}.
    * </ul>
    * </p>
    *
    * @param pointX        the x-coordinate of the query point.
    * @param pointY        the y-coordinate of the query point.
    * @param pointZ        the z-coordinate of the query point.
    * @param pointOnLine   a point located on the line. Not modified.
    * @param lineDirection the direction of the line. Not modified.
    * @return the computed percentage along the line representing where the point projection is
    *         located.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static double percentageAlongLine3D(double pointX,
                                              double pointY,
                                              double pointZ,
                                              FramePoint3DReadOnly pointOnLine,
                                              FrameVector3DReadOnly lineDirection)
   {
      pointOnLine.checkReferenceFrameMatch(lineDirection);
      return EuclidGeometryTools.percentageAlongLine3D(pointX, pointY, pointZ, pointOnLine, lineDirection);
   }

   /**
    * Computes a percentage along the line segment representing the location of the projection onto the
    * line segment of the given point. The returned percentage is in ] -&infin;; &infin; [, {@code 0.0}
    * representing {@code lineSegmentStart}, and {@code 1.0} representing {@code lineSegmentEnd}.
    * <p>
    * For example, if the returned percentage is {@code 0.5}, it means that the projection of the given
    * point is located at the middle of the line segment. The coordinates of the projection of the
    * point can be computed from the {@code percentage} as follows: <code>
    * FramePoint3DReadOnly projection = new Point3D(); </br>
    * projection.interpolate(lineSegmentStart, lineSegmentEnd, percentage); </br>
    * </code>
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the length of the given line segment is too small, i.e.
    * {@code lineSegmentStart.distanceSquared(lineSegmentEnd) < }{@link EuclidGeometryTools#ONE_TRILLIONTH},
    * this method fails and returns {@code 0.0}.
    * </ul>
    * </p>
    *
    * @param pointX           the x-coordinate of the query point.
    * @param pointY           the y-coordinate of the query point.
    * @param pointZ           the z-coordinate of the query point.
    * @param lineSegmentStart the line segment first endpoint. Not modified.
    * @param lineSegmentEnd   the line segment second endpoint. Not modified.
    * @return the computed percentage along the line segment representing where the point projection is
    *         located.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static double percentageAlongLineSegment3D(double pointX,
                                                     double pointY,
                                                     double pointZ,
                                                     FramePoint3DReadOnly lineSegmentStart,
                                                     FramePoint3DReadOnly lineSegmentEnd)
   {
      lineSegmentStart.checkReferenceFrameMatch(lineSegmentEnd);
      return EuclidGeometryTools.percentageAlongLineSegment3D(pointX, pointY, pointZ, lineSegmentStart, lineSegmentEnd);
   }

   /**
    * Computes a percentage along the line segment representing the location of the projection onto the
    * line segment of the given point. The returned percentage is in ] -&infin;; &infin; [, {@code 0.0}
    * representing {@code lineSegmentStart}, and {@code 1.0} representing {@code lineSegmentEnd}.
    * <p>
    * For example, if the returned percentage is {@code 0.5}, it means that the projection of the given
    * point is located at the middle of the line segment. The coordinates of the projection of the
    * point can be computed from the {@code percentage} as follows: <code>
    * FramePoint3DReadOnly projection = new Point3D(); </br>
    * projection.interpolate(lineSegmentStart, lineSegmentEnd, percentage); </br>
    * </code>
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the length of the given line segment is too small, i.e.
    * {@code lineSegmentStart.distanceSquared(lineSegmentEnd) < }{@link EuclidGeometryTools#ONE_TRILLIONTH},
    * this method fails and returns {@code 0.0}.
    * </ul>
    * </p>
    *
    * @param point            the query. Not modified.
    * @param lineSegmentStart the line segment first endpoint. Not modified.
    * @param lineSegmentEnd   the line segment second endpoint. Not modified.
    * @return the computed percentage along the line segment representing where the point projection is
    *         located.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static double percentageAlongLineSegment3D(FramePoint3DReadOnly point, FramePoint3DReadOnly lineSegmentStart, FramePoint3DReadOnly lineSegmentEnd)
   {
      point.checkReferenceFrameMatch(lineSegmentStart, lineSegmentEnd);
      return EuclidGeometryTools.percentageAlongLineSegment3D(point, lineSegmentStart, lineSegmentEnd);
   }

   /**
    * Computes the perpendicular bisector of line segment defined by its two endpoints. The bisector
    * starts off the the middle of the line segment and points toward the left side of the line
    * segment.
    * <p>
    * Edge cases:
    * <ul>
    * <li>when the line segment endpoints are equal, more precisely when
    * {@code lineSegmentStart.distance(lineSegmentEnd) < }{@link EuclidGeometryTools#ONE_TRILLIONTH},
    * the method fails and returns {@code false}.
    * </ul>
    * </p>
    *
    * @param lineSegmentStart        the first endpoint of the line segment. Not modified.
    * @param lineSegmentEnd          the second endpoint of the line segment. Not modified.
    * @param bisectorStartToPack     a 2D point in which the origin of the bisector is stored.
    *                                Modified.
    * @param bisectorDirectionToPack a 2D vector in which the direction of the bisector is stored.
    *                                Modified.
    * @return whether the perpendicular bisector could be determined or not.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static boolean perpendicularBisector2D(FramePoint2DReadOnly lineSegmentStart,
                                                 FramePoint2DReadOnly lineSegmentEnd,
                                                 FixedFramePoint2DBasics bisectorStartToPack,
                                                 FixedFrameVector2DBasics bisectorDirectionToPack)
   {
      lineSegmentStart.checkReferenceFrameMatch(lineSegmentEnd, bisectorStartToPack, bisectorDirectionToPack);
      return EuclidGeometryTools.perpendicularBisector2D(lineSegmentStart, lineSegmentEnd, bisectorStartToPack, bisectorDirectionToPack);
   }

   /**
    * Computes the perpendicular bisector of line segment defined by its two endpoints. The bisector
    * starts off the the middle of the line segment and points toward the left side of the line
    * segment.
    * <p>
    * Edge cases:
    * <ul>
    * <li>when the line segment endpoints are equal, more precisely when
    * {@code lineSegmentStart.distance(lineSegmentEnd) < }{@link EuclidGeometryTools#ONE_TRILLIONTH},
    * the method fails and returns {@code false}.
    * </ul>
    * </p>
    *
    * @param lineSegmentStart        the first endpoint of the line segment. Not modified.
    * @param lineSegmentEnd          the second endpoint of the line segment. Not modified.
    * @param bisectorStartToPack     a 2D point in which the origin of the bisector is stored.
    *                                Modified.
    * @param bisectorDirectionToPack a 2D vector in which the direction of the bisector is stored.
    *                                Modified.
    * @return whether the perpendicular bisector could be determined or not.
    * @throws ReferenceFrameMismatchException if the read-only arguments are not all expressed in the
    *                                         same reference frame.
    */
   public static boolean perpendicularBisector2D(FramePoint2DReadOnly lineSegmentStart,
                                                 FramePoint2DReadOnly lineSegmentEnd,
                                                 FramePoint2DBasics bisectorStartToPack,
                                                 FrameVector2DBasics bisectorDirectionToPack)
   {
      lineSegmentStart.checkReferenceFrameMatch(lineSegmentEnd, bisectorStartToPack, bisectorDirectionToPack);
      return EuclidGeometryTools.perpendicularBisector2D(lineSegmentStart, lineSegmentEnd, bisectorStartToPack, bisectorDirectionToPack);
   }

   /**
    * Computes the endpoints of the perpendicular bisector segment to a line segment defined by its
    * endpoints, such that:
    * <ul>
    * <li>each endpoint of the perpendicular bisector is at a distance of
    * {@code bisectorSegmentHalfLength} from the line segment.
    * <li>the first perpendicular bisector endpoint is located on the left side on the line segment.
    * <li>the second perpendicular bisector endpoint is located on the right side on the line segment.
    * </ul>
    * <p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>when the line segment endpoints are equal, more precisely when
    * {@code lineSegmentStart.distance(lineSegmentEnd) < }{@link EuclidGeometryTools#ONE_TRILLIONTH},
    * the method fails and returns {@code null}.
    * </ul>
    * </p>
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param lineSegmentStart          the first endpoint of the line segment from which the
    *                                  perpendicular bisector is to be computed. Not modified.
    * @param lineSegmentEnd            the second endpoint of the line segment from which the
    *                                  perpendicular bisector is to be computed. Not modified.
    * @param bisectorSegmentHalfLength distance from the line segment each endpoint of the
    *                                  perpendicular bisector segment will be positioned.
    * @return a list containing the two endpoints of the perpendicular bisector segment.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static List<Point2D> perpendicularBisectorSegment2D(FramePoint2DReadOnly lineSegmentStart,
                                                              FramePoint2DReadOnly lineSegmentEnd,
                                                              double bisectorSegmentHalfLength)
   {
      lineSegmentStart.checkReferenceFrameMatch(lineSegmentEnd);
      return EuclidGeometryTools.perpendicularBisectorSegment2D(lineSegmentStart, lineSegmentEnd, bisectorSegmentHalfLength);
   }

   /**
    * Computes the endpoints of the perpendicular bisector segment to a line segment defined by its
    * endpoints, such that:
    * <ul>
    * <li>each endpoint of the perpendicular bisector is at a distance of
    * {@code bisectorSegmentHalfLength} from the line segment.
    * <li>the first perpendicular bisector endpoint is located on the left side on the line segment.
    * <li>the second perpendicular bisector endpoint is located on the right side on the line segment.
    * </ul>
    * <p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>when the line segment endpoints are equal, more precisely when
    * {@code lineSegmentStart.distance(lineSegmentEnd) < }{@link EuclidGeometryTools#ONE_TRILLIONTH},
    * the method fails and returns false.
    * </ul>
    * </p>
    *
    * @param lineSegmentStart           the first endpoint of the line segment from which the
    *                                   perpendicular bisector is to be computed. Not modified.
    * @param lineSegmentEnd             the second endpoint of the line segment from which the
    *                                   perpendicular bisector is to be computed. Not modified.
    * @param bisectorSegmentHalfLength  distance from the line segment each endpoint of the
    *                                   perpendicular bisector segment will be positioned.
    * @param bisectorSegmentStartToPack the first endpoint of the perpendicular bisector segment to be
    *                                   computed. Modified.
    * @param bisectorSegmentEndToPack   the second endpoint of the perpendicular bisector segment to be
    *                                   computed. Modified.
    * @return whether the perpendicular bisector could be determined or not.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static boolean perpendicularBisectorSegment2D(FramePoint2DReadOnly lineSegmentStart,
                                                        FramePoint2DReadOnly lineSegmentEnd,
                                                        double bisectorSegmentHalfLength,
                                                        FixedFramePoint2DBasics bisectorSegmentStartToPack,
                                                        FixedFramePoint2DBasics bisectorSegmentEndToPack)
   {
      lineSegmentStart.checkReferenceFrameMatch(lineSegmentEnd, bisectorSegmentStartToPack, bisectorSegmentEndToPack);
      return EuclidGeometryTools.perpendicularBisectorSegment2D(lineSegmentStart,
                                                                lineSegmentEnd,
                                                                bisectorSegmentHalfLength,
                                                                bisectorSegmentStartToPack,
                                                                bisectorSegmentEndToPack);
   }

   /**
    * Computes the endpoints of the perpendicular bisector segment to a line segment defined by its
    * endpoints, such that:
    * <ul>
    * <li>each endpoint of the perpendicular bisector is at a distance of
    * {@code bisectorSegmentHalfLength} from the line segment.
    * <li>the first perpendicular bisector endpoint is located on the left side on the line segment.
    * <li>the second perpendicular bisector endpoint is located on the right side on the line segment.
    * </ul>
    * <p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>when the line segment endpoints are equal, more precisely when
    * {@code lineSegmentStart.distance(lineSegmentEnd) < }{@link EuclidGeometryTools#ONE_TRILLIONTH},
    * the method fails and returns false.
    * </ul>
    * </p>
    *
    * @param lineSegmentStart           the first endpoint of the line segment from which the
    *                                   perpendicular bisector is to be computed. Not modified.
    * @param lineSegmentEnd             the second endpoint of the line segment from which the
    *                                   perpendicular bisector is to be computed. Not modified.
    * @param bisectorSegmentHalfLength  distance from the line segment each endpoint of the
    *                                   perpendicular bisector segment will be positioned.
    * @param bisectorSegmentStartToPack the first endpoint of the perpendicular bisector segment to be
    *                                   computed. Modified.
    * @param bisectorSegmentEndToPack   the second endpoint of the perpendicular bisector segment to be
    *                                   computed. Modified.
    * @return whether the perpendicular bisector could be determined or not.
    * @throws ReferenceFrameMismatchException if the read-only arguments are not all expressed in the
    *                                         same reference frame.
    */
   public static boolean perpendicularBisectorSegment2D(FramePoint2DReadOnly lineSegmentStart,
                                                        FramePoint2DReadOnly lineSegmentEnd,
                                                        double bisectorSegmentHalfLength,
                                                        FramePoint2DBasics bisectorSegmentStartToPack,
                                                        FramePoint2DBasics bisectorSegmentEndToPack)
   {
      lineSegmentStart.checkReferenceFrameMatch(lineSegmentEnd);
      bisectorSegmentStartToPack.setReferenceFrame(lineSegmentStart.getReferenceFrame());
      bisectorSegmentEndToPack.setReferenceFrame(lineSegmentStart.getReferenceFrame());
      return EuclidGeometryTools.perpendicularBisectorSegment2D(lineSegmentStart,
                                                                lineSegmentEnd,
                                                                bisectorSegmentHalfLength,
                                                                bisectorSegmentStartToPack,
                                                                bisectorSegmentEndToPack);
   }

   /**
    * Computes the vector perpendicular to the given {@code vector} such that:
    * <ul>
    * <li>{@code vector.dot(perpendicularVector) == 0.0}.
    * <li>{@code vector.angle(perpendicularVector) == Math.PI / 2.0}.
    * </ul>
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param vector the vector to compute the perpendicular of. Not modified.
    * @return the perpendicular vector.
    */
   public static FrameVector2D perpendicularVector2D(FrameVector2DReadOnly vector)
   {
      return new FrameVector2D(vector.getReferenceFrame(), EuclidGeometryTools.perpendicularVector2D(vector));
   }

   /**
    * Computes the vector perpendicular to the given {@code vector} such that:
    * <ul>
    * <li>{@code vector.dot(perpendicularVector) == 0.0}.
    * <li>{@code vector.angle(perpendicularVector) == Math.PI / 2.0}.
    * </ul>
    *
    * @param vector                    the vector to compute the perpendicular of. Not modified.
    * @param perpendicularVectorToPack a 2D vector in which the perpendicular vector is stored.
    *                                  Modified.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static void perpendicularVector2D(FrameVector2DReadOnly vector, FixedFrameVector2DBasics perpendicularVectorToPack)
   {
      perpendicularVectorToPack.set(vector.getReferenceFrame(), -vector.getY(), vector.getX());
   }

   /**
    * Computes the vector perpendicular to the given {@code vector} such that:
    * <ul>
    * <li>{@code vector.dot(perpendicularVector) == 0.0}.
    * <li>{@code vector.angle(perpendicularVector) == Math.PI / 2.0}.
    * </ul>
    *
    * @param vector                    the vector to compute the perpendicular of. Not modified.
    * @param perpendicularVectorToPack a 2D vector in which the perpendicular vector is stored.
    *                                  Modified.
    */
   public static void perpendicularVector2D(FrameVector2DReadOnly vector, FrameVector2DBasics perpendicularVectorToPack)
   {
      perpendicularVectorToPack.setToZero(vector.getReferenceFrame());
      perpendicularVectorToPack.set(-vector.getY(), vector.getX());
   }

   /**
    * Computes the perpendicular defined by an infinitely long 3D line (defined by two 3D points) and a
    * 3D point. To do so, the orthogonal projection of the {@code point} on line is first computed. The
    * perpendicular vector is computed as follows:
    * {@code perpendicularVector = point - orthogonalProjection}, resulting in a vector going from the
    * computed projection to the given {@code point} with a length equal to the distance between the
    * point and the line.
    * <p>
    * Edge cases:
    * <ul>
    * <li>when the distance between the two points defining the line is below
    * {@value EuclidGeometryTools#ONE_TRILLIONTH}, the method fails and returns {@code null}.
    * </ul>
    * </p>
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param point                      the 3D point towards which the perpendicular vector should be
    *                                   pointing at. Not modified.
    * @param firstPointOnLine           a first point on the line. Not modified.
    * @param secondPointOnLine          a second point on the line. Not modified.
    * @param orthogonalProjectionToPack a 3D point in which the projection of {@code point} onto the
    *                                   line is stored. Modified. Can be {@code null}.
    * @return the vector perpendicular to the line and pointing to the {@code point}, or {@code null}
    *         when the method fails.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static FrameVector3D perpendicularVector3DFromLine3DToPoint3D(FramePoint3DReadOnly point,
                                                                        FramePoint3DReadOnly firstPointOnLine,
                                                                        FramePoint3DReadOnly secondPointOnLine,
                                                                        FixedFramePoint3DBasics orthogonalProjectionToPack)
   {
      point.checkReferenceFrameMatch(firstPointOnLine, secondPointOnLine);
      if (orthogonalProjectionToPack != null)
         point.checkReferenceFrameMatch(orthogonalProjectionToPack);
      Vector3D perpendicularVector = EuclidGeometryTools.perpendicularVector3DFromLine3DToPoint3D(point,
                                                                                                  firstPointOnLine,
                                                                                                  secondPointOnLine,
                                                                                                  orthogonalProjectionToPack);
      if (perpendicularVector == null)
         return null;
      else
         return new FrameVector3D(point.getReferenceFrame(), perpendicularVector);
   }

   /**
    * Computes the perpendicular defined by an infinitely long 3D line (defined by two 3D points) and a
    * 3D point. To do so, the orthogonal projection of the {@code point} on line is first computed. The
    * perpendicular vector is computed as follows:
    * {@code perpendicularVector = point - orthogonalProjection}, resulting in a vector going from the
    * computed projection to the given {@code point} with a length equal to the distance between the
    * point and the line.
    * <p>
    * Edge cases:
    * <ul>
    * <li>when the distance between the two points defining the line is below
    * {@value EuclidGeometryTools#ONE_TRILLIONTH}, the method fails and returns {@code null}.
    * </ul>
    * </p>
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param point                      the 3D point towards which the perpendicular vector should be
    *                                   pointing at. Not modified.
    * @param firstPointOnLine           a first point on the line. Not modified.
    * @param secondPointOnLine          a second point on the line. Not modified.
    * @param orthogonalProjectionToPack a 3D point in which the projection of {@code point} onto the
    *                                   line is stored. Modified. Can be {@code null}.
    * @return the vector perpendicular to the line and pointing to the {@code point}, or {@code null}
    *         when the method fails.
    * @throws ReferenceFrameMismatchException if the read-only arguments are not all expressed in the
    *                                         same reference frame.
    */
   public static FrameVector3D perpendicularVector3DFromLine3DToPoint3D(FramePoint3DReadOnly point,
                                                                        FramePoint3DReadOnly firstPointOnLine,
                                                                        FramePoint3DReadOnly secondPointOnLine,
                                                                        FramePoint3DBasics orthogonalProjectionToPack)
   {
      point.checkReferenceFrameMatch(firstPointOnLine, secondPointOnLine);
      if (orthogonalProjectionToPack != null)
         orthogonalProjectionToPack.setReferenceFrame(point.getReferenceFrame());
      Vector3D perpendicularVector = EuclidGeometryTools.perpendicularVector3DFromLine3DToPoint3D(point,
                                                                                                  firstPointOnLine,
                                                                                                  secondPointOnLine,
                                                                                                  orthogonalProjectionToPack);
      if (perpendicularVector == null)
         return null;
      else
         return new FrameVector3D(point.getReferenceFrame(), perpendicularVector);
   }

   /**
    * Computes the perpendicular defined by an infinitely long 3D line (defined by two 3D points) and a
    * 3D point. To do so, the orthogonal projection of the {@code point} on line is first computed. The
    * perpendicular vector is computed as follows:
    * {@code perpendicularVector = point - orthogonalProjection}, resulting in a vector going from the
    * computed projection to the given {@code point} with a length equal to the distance between the
    * point and the line.
    * <p>
    * Edge cases:
    * <ul>
    * <li>when the distance between the two points defining the line is below
    * {@value EuclidGeometryTools#ONE_TRILLIONTH}, the method fails and returns {@code false}.
    * </ul>
    * </p>
    *
    * @param point                      the 3D point towards which the perpendicular vector should be
    *                                   pointing at. Not modified.
    * @param firstPointOnLine           a first point on the line. Not modified.
    * @param secondPointOnLine          a second point on the line. Not modified.
    * @param orthogonalProjectionToPack a 3D point in which the projection of {@code point} onto the
    *                                   line is stored. Modified. Can be {@code null}.
    * @param perpendicularVectorToPack  a 3D vector in which the vector perpendicular to the line and
    *                                   pointing to the {@code point} is stored. Modified. Can NOT be
    *                                   {@code null}.
    * @return {@code true} if the method succeeded, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static boolean perpendicularVector3DFromLine3DToPoint3D(FramePoint3DReadOnly point,
                                                                  FramePoint3DReadOnly firstPointOnLine,
                                                                  FramePoint3DReadOnly secondPointOnLine,
                                                                  FixedFramePoint3DBasics orthogonalProjectionToPack,
                                                                  FixedFrameVector3DBasics perpendicularVectorToPack)
   {
      point.checkReferenceFrameMatch(firstPointOnLine, secondPointOnLine, perpendicularVectorToPack);
      if (orthogonalProjectionToPack != null)
         point.checkReferenceFrameMatch(orthogonalProjectionToPack);
      return EuclidGeometryTools.perpendicularVector3DFromLine3DToPoint3D(point,
                                                                          firstPointOnLine,
                                                                          secondPointOnLine,
                                                                          orthogonalProjectionToPack,
                                                                          perpendicularVectorToPack);
   }

   /**
    * Computes the perpendicular defined by an infinitely long 3D line (defined by two 3D points) and a
    * 3D point. To do so, the orthogonal projection of the {@code point} on line is first computed. The
    * perpendicular vector is computed as follows:
    * {@code perpendicularVector = point - orthogonalProjection}, resulting in a vector going from the
    * computed projection to the given {@code point} with a length equal to the distance between the
    * point and the line.
    * <p>
    * Edge cases:
    * <ul>
    * <li>when the distance between the two points defining the line is below
    * {@value EuclidGeometryTools#ONE_TRILLIONTH}, the method fails and returns {@code false}.
    * </ul>
    * </p>
    *
    * @param point                      the 3D point towards which the perpendicular vector should be
    *                                   pointing at. Not modified.
    * @param firstPointOnLine           a first point on the line. Not modified.
    * @param secondPointOnLine          a second point on the line. Not modified.
    * @param orthogonalProjectionToPack a 3D point in which the projection of {@code point} onto the
    *                                   line is stored. Modified. Can be {@code null}.
    * @param perpendicularVectorToPack  a 3D vector in which the vector perpendicular to the line and
    *                                   pointing to the {@code point} is stored. Modified. Can NOT be
    *                                   {@code null}.
    * @return {@code true} if the method succeeded, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if the read-only arguments are not all expressed in the
    *                                         same reference frame.
    */
   public static boolean perpendicularVector3DFromLine3DToPoint3D(FramePoint3DReadOnly point,
                                                                  FramePoint3DReadOnly firstPointOnLine,
                                                                  FramePoint3DReadOnly secondPointOnLine,
                                                                  FramePoint3DBasics orthogonalProjectionToPack,
                                                                  FrameVector3DBasics perpendicularVectorToPack)
   {
      point.checkReferenceFrameMatch(firstPointOnLine, secondPointOnLine);
      if (orthogonalProjectionToPack != null)
         orthogonalProjectionToPack.setReferenceFrame(point.getReferenceFrame());
      perpendicularVectorToPack.setReferenceFrame(point.getReferenceFrame());
      return EuclidGeometryTools.perpendicularVector3DFromLine3DToPoint3D(point,
                                                                          firstPointOnLine,
                                                                          secondPointOnLine,
                                                                          orthogonalProjectionToPack,
                                                                          perpendicularVectorToPack);
   }

   /**
    * Returns the minimum signed distance between a 2D point and an infinitely long 2D line defined by
    * a point and a direction.
    * <p>
    * The calculated distance is negative if the query is located on the right side of the line.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>if
    * {@code firstPointOnLine.distance(secondPointOnLine) < }{@link EuclidGeometryTools#ONE_TRILLIONTH},
    * this method returns the distance between {@code firstPointOnLine} and the given {@code point}.
    * </ul>
    * </p>
    *
    * @param pointX            x-coordinate of the query.
    * @param pointY            y-coordinate of the query.
    * @param firstPointOnLine  a first point located on the line. Not modified.
    * @param secondPointOnLine a second point located on the line. Not modified.
    * @return the minimum distance between the 2D point and the 2D line. The distance is negative if
    *         the query is located on the right side of the line.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static double signedDistanceFromPoint2DToLine2D(double pointX,
                                                          double pointY,
                                                          FramePoint2DReadOnly firstPointOnLine,
                                                          FramePoint2DReadOnly secondPointOnLine)
   {
      firstPointOnLine.checkReferenceFrameMatch(secondPointOnLine);
      return EuclidGeometryTools.signedDistanceFromPoint2DToLine2D(pointX, pointY, firstPointOnLine, secondPointOnLine);
   }

   /**
    * Returns the minimum signed distance between a 2D point and an infinitely long 2D line defined by
    * a point and a direction.
    * <p>
    * The calculated distance is negative if the query is located on the right side of the line.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>if {@code lineDirection.length() < }{@link EuclidGeometryTools#ONE_TRILLIONTH}, this method
    * returns the distance between {@code pointOnLine} and the given {@code point}.
    * </ul>
    * </p>
    *
    * @param pointX        x-coordinate of the query.
    * @param pointY        y-coordinate of the query.
    * @param pointOnLine   a point located on the line. Not modified.
    * @param lineDirection the direction of the line. Not modified.
    * @return the minimum distance between the 2D point and the 2D line. The distance is negative if
    *         the query is located on the right side of the line.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static double signedDistanceFromPoint2DToLine2D(double pointX, double pointY, FramePoint2DReadOnly pointOnLine, FrameVector2DReadOnly lineDirection)
   {
      pointOnLine.checkReferenceFrameMatch(lineDirection);
      return EuclidGeometryTools.signedDistanceFromPoint2DToLine2D(pointX, pointY, pointOnLine, lineDirection);
   }

   /**
    * Returns the minimum signed distance between a 2D point and an infinitely long 2D line defined by
    * a point and a direction.
    * <p>
    * The calculated distance is negative if the query is located on the right side of the line.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>if
    * {@code firstPointOnLine.distance(secondPointOnLine) < }{@link EuclidGeometryTools#ONE_TRILLIONTH},
    * this method returns the distance between {@code firstPointOnLine} and the given {@code point}.
    * </ul>
    * </p>
    *
    * @param point             the coordinates of the query. Not modified.
    * @param firstPointOnLine  a first point located on the line. Not modified.
    * @param secondPointOnLine a second point located on the line. Not modified.
    * @return the minimum distance between the 2D point and the 2D line. The distance is negative if
    *         the query is located on the right side of the line.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static double signedDistanceFromPoint2DToLine2D(FramePoint2DReadOnly point,
                                                          FramePoint2DReadOnly firstPointOnLine,
                                                          FramePoint2DReadOnly secondPointOnLine)
   {
      point.checkReferenceFrameMatch(firstPointOnLine, secondPointOnLine);
      return EuclidGeometryTools.signedDistanceFromPoint2DToLine2D(point, firstPointOnLine, secondPointOnLine);
   }

   /**
    * Returns the minimum signed distance between a 2D point and an infinitely long 2D line defined by
    * a point and a direction.
    * <p>
    * The calculated distance is negative if the query is located on the right side of the line.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>if {@code lineDirection.length() < }{@link EuclidGeometryTools#ONE_TRILLIONTH}, this method
    * returns the distance between {@code pointOnLine} and the given {@code point}.
    * </ul>
    * </p>
    *
    * @param point         the coordinates of the query. Not modified.
    * @param pointOnLine   a point located on the line. Not modified.
    * @param lineDirection the direction of the line. Not modified.
    * @return the minimum distance between the 2D point and the 2D line. The distance is negative if
    *         the query is located on the right side of the line.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static double signedDistanceFromPoint2DToLine2D(FramePoint2DReadOnly point, FramePoint2DReadOnly pointOnLine, FrameVector2DReadOnly lineDirection)
   {
      point.checkReferenceFrameMatch(pointOnLine, lineDirection);
      return EuclidGeometryTools.signedDistanceFromPoint2DToLine2D(point, pointOnLine, lineDirection);
   }

   /**
    * Computes the position of the sphere given its radius and three points that lie on its surface.
    * <p>
    * There two possible solutions to this problem. The solution returned is located "above" the
    * triangle's plane. "Above" is defined by the direction given by the normal of the three points
    * such as their winding is counter-clockwise.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>In the case the radius of the sphere is too small to reach all three points, i.e. the radius
    * is smaller than the circumradius of the triangle, this method fails and returns {@code false}.
    * <li>If the problem is degenerate, i.e. any of the three lengths of the triangle is zero, this
    * method fails and returns {@code false}.
    * </ul>
    * </p>
    *
    * @param p1                     the first point that belongs to the sphere. Not modified.
    * @param p2                     the second point that belongs to the sphere. Not modified.
    * @param p3                     the third point that belongs to the sphere. Not modified.
    * @param sphere3DRadius         the radius of the sphere.
    * @param sphere3DPositionToPack the point used to store the result. Modified.
    * @return whether the sphere position was successfully or not.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static boolean sphere3DPositionFromThreePoints(FramePoint3DReadOnly p1,
                                                         FramePoint3DReadOnly p2,
                                                         FramePoint3DReadOnly p3,
                                                         double sphere3DRadius,
                                                         FixedFramePoint3DBasics sphere3DPositionToPack)
   {
      p1.checkReferenceFrameMatch(p2, p3, sphere3DPositionToPack);
      return EuclidGeometryTools.sphere3DPositionFromThreePoints(p1, p2, p3, sphere3DRadius, sphere3DPositionToPack);
   }

   /**
    * Computes the position of the sphere given its radius and three points that lie on its surface.
    * <p>
    * There two possible solutions to this problem. The solution returned is located "above" the
    * triangle's plane. "Above" is defined by the direction given by the normal of the three points
    * such as their winding is counter-clockwise.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>In the case the radius of the sphere is too small to reach all three points, i.e. the radius
    * is smaller than the circumradius of the triangle, this method fails and returns {@code false}.
    * <li>If the problem is degenerate, i.e. any of the three lengths of the triangle is zero, this
    * method fails and returns {@code false}.
    * </ul>
    * </p>
    *
    * @param p1                     the first point that belongs to the sphere. Not modified.
    * @param p2                     the second point that belongs to the sphere. Not modified.
    * @param p3                     the third point that belongs to the sphere. Not modified.
    * @param sphere3DRadius         the radius of the sphere.
    * @param sphere3DPositionToPack the point used to store the result. Modified.
    * @return whether the sphere position was successfully or not.
    * @throws ReferenceFrameMismatchException if the three points are not all expressed in the same
    *                                         reference frame.
    */
   public static boolean sphere3DPositionFromThreePoints(FramePoint3DReadOnly p1,
                                                         FramePoint3DReadOnly p2,
                                                         FramePoint3DReadOnly p3,
                                                         double sphere3DRadius,
                                                         FramePoint3DBasics sphere3DPositionToPack)
   {
      p1.checkReferenceFrameMatch(p2, p3);
      sphere3DPositionToPack.setReferenceFrame(p1.getReferenceFrame());
      return EuclidGeometryTools.sphere3DPositionFromThreePoints(p1, p2, p3, sphere3DRadius, sphere3DPositionToPack);
   }

   /**
    * Assuming an isosceles triangle defined by three vertices A, B, and C, with |AB| == |BC|, this
    * methods computes the missing vertex B given the vertices A and C, the normal of the triangle, the
    * angle ABC that is equal to the angle at B from the the leg BA to the leg BC.
    * <a href="https://en.wikipedia.org/wiki/Isosceles_triangle"> Useful link</a>.
    *
    * @param baseVertexA                    the first base vertex of the isosceles triangle ABC. Not
    *                                       modified.
    * @param baseVertexC                    the second base vertex of the isosceles triangle ABC. Not
    *                                       modified.
    * @param trianglePlaneNormal            the normal of the plane on which is lying. Not modified.
    * @param ccwAngleAboutNormalAtTopVertex the angle at B from the the leg BA to the leg BC.
    * @param topVertexBToPack               the missing vertex B. Modified.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static void topVertex3DOfIsoscelesTriangle3D(FramePoint3DReadOnly baseVertexA,
                                                       FramePoint3DReadOnly baseVertexC,
                                                       FrameVector3DReadOnly trianglePlaneNormal,
                                                       double ccwAngleAboutNormalAtTopVertex,
                                                       FixedFramePoint3DBasics topVertexBToPack)
   {
      baseVertexA.checkReferenceFrameMatch(baseVertexC, trianglePlaneNormal, topVertexBToPack);
      EuclidGeometryTools.topVertex3DOfIsoscelesTriangle3D(baseVertexA, baseVertexC, trianglePlaneNormal, ccwAngleAboutNormalAtTopVertex, topVertexBToPack);
   }

   /**
    * Assuming an isosceles triangle defined by three vertices A, B, and C, with |AB| == |BC|, this
    * methods computes the missing vertex B given the vertices A and C, the normal of the triangle, the
    * angle ABC that is equal to the angle at B from the the leg BA to the leg BC.
    * <a href="https://en.wikipedia.org/wiki/Isosceles_triangle"> Useful link</a>.
    *
    * @param baseVertexA                    the first base vertex of the isosceles triangle ABC. Not
    *                                       modified.
    * @param baseVertexC                    the second base vertex of the isosceles triangle ABC. Not
    *                                       modified.
    * @param trianglePlaneNormal            the normal of the plane on which is lying. Not modified.
    * @param ccwAngleAboutNormalAtTopVertex the angle at B from the the leg BA to the leg BC.
    * @param topVertexBToPack               the missing vertex B. Modified.
    * @throws ReferenceFrameMismatchException if the read-only arguments are not all expressed in the
    *                                         same reference frame.
    */
   public static void topVertex3DOfIsoscelesTriangle3D(FramePoint3DReadOnly baseVertexA,
                                                       FramePoint3DReadOnly baseVertexC,
                                                       FrameVector3DReadOnly trianglePlaneNormal,
                                                       double ccwAngleAboutNormalAtTopVertex,
                                                       FramePoint3DBasics topVertexBToPack)
   {
      baseVertexA.checkReferenceFrameMatch(baseVertexC, trianglePlaneNormal);
      topVertexBToPack.setReferenceFrame(baseVertexA.getReferenceFrame());
      EuclidGeometryTools.topVertex3DOfIsoscelesTriangle3D(baseVertexA, baseVertexC, trianglePlaneNormal, ccwAngleAboutNormalAtTopVertex, topVertexBToPack);
   }

   /**
    * Given a triangle defined by three points (A,B,C), this methods the point X &in; AC such that the
    * line (B, X) is the angle bisector of B. As a result, the two angles CBX and XBA are equal.
    * <a href="https://en.wikipedia.org/wiki/Angle_bisector_theorem"> Useful link</a>.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if any the triangle's edge is shorter than {@link EuclidGeometryTools#ONE_TRILLIONTH}, this
    * method fails and returns {@code null}.
    * </ul>
    * </p>
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param A the first vertex of the triangle. Not modified.
    * @param B the second vertex of the triangle, this is the first endpoint of the bisector. Not
    *          modified.
    * @param C the third vertex of the triangle. Not modified.
    * @return the second endpoint of the bisector, or {@code null} if the method failed.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static FramePoint2D triangleBisector2D(FramePoint2DReadOnly A, FramePoint2DReadOnly B, FramePoint2DReadOnly C)
   {
      A.checkReferenceFrameMatch(B, C);
      Point2D X = EuclidGeometryTools.triangleBisector2D(A, B, C);
      if (X != null)
         return new FramePoint2D(A.getReferenceFrame(), X);
      else
         return null;
   }

   /**
    * Given a triangle defined by three points (A,B,C), this methods the point X &in; AC such that the
    * line (B, X) is the angle bisector of B. As a result, the two angles CBX and XBA are equal.
    * <a href="https://en.wikipedia.org/wiki/Angle_bisector_theorem"> Useful link</a>.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if any the triangle's edge is shorter than {@link EuclidGeometryTools#ONE_TRILLIONTH}, this
    * method fails and returns {@code false}.
    * </ul>
    * </p>
    *
    * @param A       the first vertex of the triangle. Not modified.
    * @param B       the second vertex of the triangle, this is the first endpoint of the bisector. Not
    *                modified.
    * @param C       the third vertex of the triangle. Not modified.
    * @param XToPack point in which the second endpoint of the bisector is stored. Modified.
    * @return whether the bisector could be calculated or not.
    * @throws ReferenceFrameMismatchException if the arguments are not all expressed in the same
    *                                         reference frame.
    */
   public static boolean triangleBisector2D(FramePoint2DReadOnly A, FramePoint2DReadOnly B, FramePoint2DReadOnly C, FixedFramePoint2DBasics XToPack)
   {
      A.checkReferenceFrameMatch(B, C, XToPack);
      return EuclidGeometryTools.triangleBisector2D(A, B, C, XToPack);
   }

   /**
    * Given a triangle defined by three points (A,B,C), this methods the point X &in; AC such that the
    * line (B, X) is the angle bisector of B. As a result, the two angles CBX and XBA are equal.
    * <a href="https://en.wikipedia.org/wiki/Angle_bisector_theorem"> Useful link</a>.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if any the triangle's edge is shorter than {@link EuclidGeometryTools#ONE_TRILLIONTH}, this
    * method fails and returns {@code false}.
    * </ul>
    * </p>
    *
    * @param A       the first vertex of the triangle. Not modified.
    * @param B       the second vertex of the triangle, this is the first endpoint of the bisector. Not
    *                modified.
    * @param C       the third vertex of the triangle. Not modified.
    * @param XToPack point in which the second endpoint of the bisector is stored. Modified.
    * @return whether the bisector could be calculated or not.
    * @throws ReferenceFrameMismatchException if the read-only arguments are not all expressed in the
    *                                         same reference frame.
    */
   public static boolean triangleBisector2D(FramePoint2DReadOnly A, FramePoint2DReadOnly B, FramePoint2DReadOnly C, FramePoint2DBasics XToPack)
   {
      A.checkReferenceFrameMatch(B, C);
      XToPack.setReferenceFrame(A.getReferenceFrame());
      return EuclidGeometryTools.triangleBisector2D(A, B, C, XToPack);
   }

   /**
    * Tests if the two arguments are equal on a per component basis.
    * <p>
    * If both arguments are {@code null}, {@code true} is returned and if exactly one argument is
    * {@code null}, {@code false} is returned. Otherwise, equality is determined by using the
    * {@link EuclidFrameGeometry#equals equals} method of the first argument.
    * </p>
    *
    * @param a the first geometry in the comparison. Not modified.
    * @param b the second geometry in the comparison. Not modified.
    * @return {@code true} if the arguments are equal to each other and {@code false} otherwise
    * @see EuclidFrameGeometry#equals(EuclidFrameGeometry)
    */
   public static boolean equals(EuclidFrameGeometry a, EuclidFrameGeometry b)
   {
      return (a == b) || (a != null && a.equals(b));
   }

   /**
    * Tests if the two arguments are approximately equal on a per component basis.
    * <p>
    * If both arguments are {@code null}, {@code true} is returned and if exactly one argument is
    * {@code null}, {@code false} is returned. Otherwise, equality is determined by using the
    * {@link EuclidFrameGeometry#epsilonEquals epsilonEquals} method of the first argument.
    * </p>
    *
    * @param a       the first geometry in the comparison. Not modified.
    * @param b       the second geometry in the comparison. Not modified.
    * @param epsilon tolerance to use when comparing each component.
    * @return {@code true} if the arguments are equal to each other and {@code false} otherwise
    * @see EuclidFrameGeometry#epsilonEquals(EuclidFrameGeometry, double)
    */
   public static boolean epsilonEquals(EuclidFrameGeometry a, EuclidFrameGeometry b, double epsilon)
   {
      return (a == b) || (a != null && a.epsilonEquals(b, epsilon));
   }

   /**
    * Tests if the two arguments represent the same geometry to an {@code epsilon}.
    * <p>
    * If both arguments are {@code null}, {@code true} is returned and if exactly one argument is
    * {@code null}, {@code false} is returned. Otherwise, equality is determined by using the
    * {@link EuclidFrameGeometry#geometricallyEquals geometricallyEquals} method of the first argument.
    * </p>
    *
    * @param a       the first geometry in the comparison. Not modified.
    * @param b       the second geometry in the comparison. Not modified.
    * @param epsilon tolerance to use when comparing each component.
    * @return {@code true} if the arguments are equal to each other and {@code false} otherwise
    * @see EuclidFrameGeometry#geometricallyEquals(EuclidFrameGeometry, double)
    */
   public static boolean geometricallyEquals(EuclidFrameGeometry a, EuclidFrameGeometry b, double epsilon)
   {
      return (a == b) || (a != null && a.geometricallyEquals(b, epsilon));
   }
}
