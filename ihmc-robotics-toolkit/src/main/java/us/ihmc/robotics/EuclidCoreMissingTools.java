package us.ihmc.robotics;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

public class EuclidCoreMissingTools
{

   public static void floorToGivenPrecision(Tuple3DBasics tuple3d, double precision)
   {
      tuple3d.setX(MathTools.floorToPrecision(tuple3d.getX(), precision));
      tuple3d.setY(MathTools.floorToPrecision(tuple3d.getY(), precision));
      tuple3d.setZ(MathTools.floorToPrecision(tuple3d.getZ(), precision));

   }

   public static void roundToGivenPrecision(Tuple3DBasics tuple3d, double precision)
   {
      tuple3d.setX(MathTools.roundToPrecision(tuple3d.getX(), precision));
      tuple3d.setY(MathTools.roundToPrecision(tuple3d.getY(), precision));
      tuple3d.setZ(MathTools.roundToPrecision(tuple3d.getZ(), precision));
   }

   public static boolean isFinite(Tuple3DBasics tuple)
   {
      return Double.isFinite(tuple.getX()) && Double.isFinite(tuple.getY()) && Double.isFinite(tuple.getZ());
   }

   /**
    * Projects the provided {@code rotation} onto {@code axis} such that the original rotation can be decomposed
    * into a rotation around {@code axis} and one around an orthogonal axis.
    * <p>
    * rotation = orthogonalRotation * result
    * </p>
    * @param rotation is the original rotation to be projected onto {@code axis}
    * @param axis is the desired rotation axis of the result.
    * @param result will be modified to contain the component of {@code rotation} that is around {@code axis}
    */
   public static void projectRotationOnAxis(QuaternionReadOnly rotation, Vector3DReadOnly axis, QuaternionBasics result)
   {
      double dotProduct = rotation.getX() * axis.getX() + rotation.getY() * axis.getY() + rotation.getZ() * axis.getZ();

      double scale = dotProduct / axis.lengthSquared();
      double projectedX = scale * axis.getX();
      double projectedY = scale * axis.getY();
      double projectedZ = scale * axis.getZ();

      result.set(projectedX, projectedY, projectedZ, rotation.getS());
      result.normalize();
   }

   public static boolean intersectionBetweenTwoLine2Ds(Point2DReadOnly firstPointOnLine1, Point2DReadOnly secondPointOnLine1, Point2DReadOnly firstPointOnLine2,
                                                       Point2DReadOnly secondPointOnLine2, Point2DBasics intersectionToPack)
   {
      double pointOnLine1x = firstPointOnLine1.getX();
      double pointOnLine1y = firstPointOnLine1.getY();
      double lineDirection1x = secondPointOnLine1.getX() - firstPointOnLine1.getX();
      double lineDirection1y = secondPointOnLine1.getY() - firstPointOnLine1.getY();
      double pointOnLine2x = firstPointOnLine2.getX();
      double pointOnLine2y = firstPointOnLine2.getY();
      double lineDirection2x = secondPointOnLine2.getX() - firstPointOnLine2.getX();
      double lineDirection2y = secondPointOnLine2.getY() - firstPointOnLine2.getY();
      return EuclidGeometryTools.intersectionBetweenTwoLine2Ds(pointOnLine1x, pointOnLine1y, lineDirection1x, lineDirection1y, pointOnLine2x, pointOnLine2y,
                                                               lineDirection2x, lineDirection2y, intersectionToPack);
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
    * <li>if the two lines are collinear, the two lines are assumed to be intersecting infinitely the returned value {@code Double.POSITIVE_INFINITY}.
    * </ul>
    * </p>
    *
    * @param pointOnLine1x   x-coordinate of a point located on the first line.
    * @param pointOnLine1y   y-coordinate of a point located on the first line.
    * @param lineDirection1x x-component of the first line direction.
    * @param lineDirection1y y-component of the first line direction.
    * @param pointOnLine2x   x-coordinate of a point located on the second line.
    * @param pointOnLine2y   y-coordinate of a point located on the second line.
    * @param lineDirection2x x-component of the second line direction.
    * @param lineDirection2y y-component of the second line direction.
    * @return {@code alpha} the percentage along the first line of the intersection location. This
    *         method returns {@link Double#NaN} if the lines do not intersect.
    */
   public static double percentageOfIntersectionBetweenTwoLine2DsInfCase(double pointOnLine1x, double pointOnLine1y, double lineDirection1x, double lineDirection1y,
                                                                         double pointOnLine2x, double pointOnLine2y, double lineDirection2x, double lineDirection2y)
   {
      //      We solve for x the problem of the form: A * x = b
      //            A      *     x     =      b
      //      / lineDirection1x -lineDirection2x \   / alpha \   / pointOnLine2x - pointOnLine1x \
      //      |                                  | * |       | = |                               |
      //      \ lineDirection1y -lineDirection2y /   \ beta  /   \ pointOnLine2y - pointOnLine1y /
      // Here, only alpha or beta is needed.

      double determinant = -lineDirection1x * lineDirection2y + lineDirection1y * lineDirection2x;

      double dx = pointOnLine2x - pointOnLine1x;
      double dy = pointOnLine2y - pointOnLine1y;

      if (Math.abs(determinant) < EuclidGeometryTools.ONE_TRILLIONTH)
      { // The lines are parallel
         // Check if they are collinear
         double cross = dx * lineDirection1y - dy * lineDirection1x;
         if (Math.abs(cross) < EuclidGeometryTools.ONE_TRILLIONTH)
         {
            /*
             * The two lines are collinear. There's an infinite number of intersection. Let's set the
             * result to infinity, i.e. alpha = infinity so it can be handled.
             */
            return Double.POSITIVE_INFINITY;
         }
         else
         {
            return Double.NaN;
         }
      }
      else
      {
         double oneOverDeterminant = 1.0 / determinant;
         double AInverse00 = oneOverDeterminant * -lineDirection2y;
         double AInverse01 = oneOverDeterminant * lineDirection2x;

         double alpha = AInverse00 * dx + AInverse01 * dy;

         return alpha;
      }
   }

}
