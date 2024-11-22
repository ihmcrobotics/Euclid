package us.ihmc.euclid.tools;

import org.ejml.MatrixDimensionException;
import org.ejml.data.Matrix;
import us.ihmc.euclid.axisAngle.interfaces.AxisAngleBasics;
import us.ihmc.euclid.axisAngle.interfaces.AxisAngleReadOnly;
import us.ihmc.euclid.interfaces.EuclidGeometry;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixBasics;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation2DBasics;
import us.ihmc.euclid.orientation.interfaces.Orientation2DReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DBasics;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformBasics;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.*;
import us.ihmc.euclid.tuple3D.interfaces.*;
import us.ihmc.euclid.tuple4D.interfaces.*;
import us.ihmc.euclid.yawPitchRoll.interfaces.YawPitchRollBasics;
import us.ihmc.euclid.yawPitchRoll.interfaces.YawPitchRollReadOnly;

import java.util.Collections;
import java.util.List;
import java.util.Objects;

/**
 * This class provides a variety of generic tools such as fast square-root algorithm
 * {@link #fastSquareRoot(double)}, or also an linear interpolation algorithm
 * {@link #interpolate(double, double, double)}.
 *
 * @author Sylvain Bertrand
 */
public class EuclidCoreTools
{
   /**
    * This field is used to save unnecessary computation and represents the number
    * {@code 2.0 * Math.PI}.
    */
   public static final double TwoPI = 2.0 * Math.PI;

   /**
    * Magic number used as a tolerance for switching to the fast square-root formula.
    */
   public static final double EPS_NORM_FAST_SQRT = 2.107342e-08;
   /**
    * An epsilon that is just slightly bigger than numerical inaccuracies used in
    * {@link #shiftAngleInRange(double, double)}.
    */
   public static final double EPS_ANGLE_SHIFT = 1.0e-12;
   /**
    * Tolerance used in {@link #clamp(double, double, double)} to verify the bounds are sane.
    */
   public static final double CLAMP_EPS = 1.0e-10;

   /**
    * Constant representing the coordinates (0, 0) of the origin in the 2D plane.
    */
   public static final Point2DReadOnly origin2D = new Point2DReadOnly()
   {
      @Override
      public double getX()
      {
         return 0.0;
      }

      @Override
      public double getY()
      {
         return 0.0;
      }

      @Override
      public boolean equals(Object object)
      {
         if (object instanceof Tuple2DReadOnly)
            return Point2DReadOnly.super.equals((Tuple2DReadOnly) object);
         else
            return false;
      }

      @Override
      public int hashCode()
      {
         return 1;
      }

      @Override
      public String toString()
      {
         return toString(EuclidCoreIOTools.DEFAULT_FORMAT);
      }
   };

   /**
    * Constant representing the coordinates (0, 0, 0) of the origin in 3D.
    */
   public static final Point3DReadOnly origin3D = new Point3DReadOnly()
   {
      @Override
      public double getX()
      {
         return 0.0;
      }

      @Override
      public double getY()
      {
         return 0.0;
      }

      @Override
      public double getZ()
      {
         return 0.0;
      }

      @Override
      public boolean equals(Object object)
      {
         if (object instanceof Tuple3DReadOnly)
            return Point3DReadOnly.super.equals((Tuple3DReadOnly) object);
         else
            return false;
      }

      @Override
      public int hashCode()
      {
         return 1;
      }

      @Override
      public String toString()
      {
         return toString(EuclidCoreIOTools.DEFAULT_FORMAT);
      }
   };

   /**
    * Constant representing the zero vector 2D: (0, 0).
    */
   public static final Vector2DReadOnly zeroVector2D = new Vector2DReadOnly()
   {
      @Override
      public double getX()
      {
         return 0.0;
      }

      @Override
      public double getY()
      {
         return 0.0;
      }

      @Override
      public boolean equals(Object object)
      {
         if (object instanceof Tuple2DReadOnly)
            return Vector2DReadOnly.super.equals((Tuple2DReadOnly) object);
         else
            return false;
      }

      @Override
      public int hashCode()
      {
         return 1;
      }

      @Override
      public String toString()
      {
         return toString(EuclidCoreIOTools.DEFAULT_FORMAT);
      }
   };

   /**
    * Constant representing the zero vector 3D: (0, 0, 0).
    */
   public static final Vector3DReadOnly zeroVector3D = new Vector3DReadOnly()
   {
      @Override
      public double getX()
      {
         return 0.0;
      }

      @Override
      public double getY()
      {
         return 0.0;
      }

      @Override
      public double getZ()
      {
         return 0.0;
      }

      @Override
      public boolean equals(Object object)
      {
         if (object instanceof Tuple3DReadOnly)
            return Vector3DReadOnly.super.equals((Tuple3DReadOnly) object);
         else
            return false;
      }

      @Override
      public int hashCode()
      {
         return 1;
      }

      @Override
      public String toString()
      {
         return toString(EuclidCoreIOTools.DEFAULT_FORMAT);
      }
   };

   /**
    * Constant representing the neutral quaternion: (x=0, y=0, z=0, s=0).
    */
   public static final QuaternionReadOnly neutralQuaternion = new QuaternionReadOnly()
   {
      @Override
      public double getX()
      {
         return 0;
      }

      @Override
      public double getY()
      {
         return 0.0;
      }

      @Override
      public double getZ()
      {
         return 0.0;
      }

      @Override
      public double getS()
      {
         return 1.0;
      }

      @Override
      public int hashCode()
      {
         return -1106247679;
      }

      @Override
      public boolean equals(Object object)
      {
         if (object instanceof QuaternionReadOnly)
            return equals((QuaternionReadOnly) object);
         else
            return false;
      }

      @Override
      public String toString()
      {
         return toString(EuclidCoreIOTools.DEFAULT_FORMAT);
      }
   };

   /**
    * Constant representing the zero-matrix 3D, i.e. the 9 elements are zero.
    */
   public static final Matrix3DReadOnly zeroMatrix3D = new Matrix3DReadOnly()
   {
      @Override
      public double getM00()
      {
         return 0;
      }

      @Override
      public double getM01()
      {
         return 0;
      }

      @Override
      public double getM02()
      {
         return 0;
      }

      @Override
      public double getM10()
      {
         return 0;
      }

      @Override
      public double getM11()
      {
         return 0;
      }

      @Override
      public double getM12()
      {
         return 0;
      }

      @Override
      public double getM20()
      {
         return 0;
      }

      @Override
      public double getM21()
      {
         return 0;
      }

      @Override
      public double getM22()
      {
         return 0;
      }

      @Override
      public boolean equals(Object object)
      {
         if (object instanceof Matrix3DReadOnly)
            return Matrix3DReadOnly.super.equals((Matrix3DReadOnly) object);
         else
            return false;
      }

      @Override
      public String toString()
      {
         return toString(EuclidCoreIOTools.DEFAULT_FORMAT);
      }

      @Override
      public int hashCode()
      {
         return 1;
      }
   };

   /**
    * Constant representing the identity matrix 3D, i.e. the 3 diagonal elements are equal to one and
    * the other equal to zero.
    */
   public static final Matrix3DReadOnly identityMatrix3D = new Matrix3DReadOnly()
   {
      @Override
      public double getM00()
      {
         return 1;
      }

      @Override
      public double getM01()
      {
         return 0;
      }

      @Override
      public double getM02()
      {
         return 0;
      }

      @Override
      public double getM10()
      {
         return 0;
      }

      @Override
      public double getM11()
      {
         return 1;
      }

      @Override
      public double getM12()
      {
         return 0;
      }

      @Override
      public double getM20()
      {
         return 0;
      }

      @Override
      public double getM21()
      {
         return 0;
      }

      @Override
      public double getM22()
      {
         return 1;
      }

      @Override
      public boolean equals(Object object)
      {
         if (object instanceof Matrix3DReadOnly)
            return Matrix3DReadOnly.super.equals((Matrix3DReadOnly) object);
         else
            return false;
      }

      @Override
      public String toString()
      {
         return toString(EuclidCoreIOTools.DEFAULT_FORMAT);
      }

      @Override
      public int hashCode()
      {
         return 976224257;
      }
   };

   private EuclidCoreTools()
   {
      // Suppresses default constructor, ensuring non-instantiability.
   }

   /**
    * Calculates and returns the square root of the given value.
    *
    * @param value the value to calculate the square root of.
    * @return the value of the square root.
    * @see Math#sqrt(double)
    */
   public static double squareRoot(double value)
   {
      return Math.sqrt(value);
   }

   /**
    * Calculates and returns the square root of the given value.
    * <p>
    * This method is optimized when {@code squaredValueClosedToOne} is equal to
    * 1+/-{@value #EPS_NORM_FAST_SQRT} by using an approximation of the square root.
    *
    * @param squaredValueClosedToOne the value to calculate the square root of.
    * @return the value of the square root.
    */
   public static double fastSquareRoot(double squaredValueClosedToOne)
   {
      if (Math.abs(1.0 - squaredValueClosedToOne) < EPS_NORM_FAST_SQRT)
         squaredValueClosedToOne = 0.5 * (1.0 + squaredValueClosedToOne);
      else
         squaredValueClosedToOne = EuclidCoreTools.squareRoot(squaredValueClosedToOne);

      return squaredValueClosedToOne;
   }

   /**
    * Tests if at least one of the two given elements is equal to {@linkplain Double#NaN}.
    *
    * @param a the first element.
    * @param b the second element.
    * @return {@code true} if at least one element is equal to {@link Double#NaN}, {@code false}
    *       otherwise.
    */
   public static boolean containsNaN(double a, double b)
   {
      return Double.isNaN(a) || Double.isNaN(b);
   }

   /**
    * Tests if at least one of the three given elements is equal to {@linkplain Double#NaN}.
    *
    * @param a the first element.
    * @param b the second element.
    * @param c the third element.
    * @return {@code true} if at least one element is equal to {@link Double#NaN}, {@code false}
    *       otherwise.
    */
   public static boolean containsNaN(double a, double b, double c)
   {
      return Double.isNaN(a) || Double.isNaN(b) || Double.isNaN(c);
   }

   /**
    * Tests if at least one of the four given elements is equal to {@linkplain Double#NaN}.
    *
    * @param a the first element.
    * @param b the second element.
    * @param c the third element.
    * @param d the fourth element.
    * @return {@code true} if at least one element is equal to {@link Double#NaN}, {@code false}
    *       otherwise.
    */
   public static boolean containsNaN(double a, double b, double c, double d)
   {
      return Double.isNaN(a) || Double.isNaN(b) || Double.isNaN(c) || Double.isNaN(d);
   }

   /**
    * Tests if at least one of the nine given elements is equal to {@linkplain Double#NaN}.
    *
    * @param a0 the first element.
    * @param a1 the second element.
    * @param a2 the third element.
    * @param a3 the fourth element.
    * @param a4 the fifth element.
    * @param a5 the sixth element.
    * @param a6 the seventh element.
    * @param a7 the eighth element.
    * @param a8 the ninth element.
    * @return {@code true} if at least one element is equal to {@link Double#NaN}, {@code false}
    *       otherwise.
    */
   public static boolean containsNaN(double a0, double a1, double a2, double a3, double a4, double a5, double a6, double a7, double a8)
   {
      if (Double.isNaN(a0) || Double.isNaN(a1) || Double.isNaN(a2))
         return true;
      if (Double.isNaN(a3) || Double.isNaN(a4) || Double.isNaN(a5))
         return true;
      return Double.isNaN(a6) || Double.isNaN(a7) || Double.isNaN(a8);
   }

   /**
    * Tests if at least one element in the given array is equal to {@link Double#NaN}.
    *
    * @param array the array containing the elements to test for {@link Double#NaN}. Not modified.
    * @return {@code true} if at least one element is equal to {@link Double#NaN}, {@code false}
    *       otherwise.
    */
   public static boolean containsNaN(double[] array)
   {
      for (int i = 0; i < array.length; i++)
      {
         if (Double.isNaN(array[i]))
            return true;
      }
      return false;
   }

   /**
    * Calculates and returns {@code value}<sup>2</sup>
    *
    * @param value the value to compute the square of.
    * @return {@code value * value}.
    */
   public static double square(double value)
   {
      return value * value;
   }

   /**
    * Calculates and returns the norm squared of the given two elements.
    * <p>
    * norm<sup>2</sup> = x<sup>2</sup> + y<sup>2</sup>
    * </p>
    *
    * @param x the first element.
    * @param y the second element.
    * @return the value of the square of the norm.
    */
   public static double normSquared(double x, double y)
   {
      return x * x + y * y;
   }

   /**
    * Calculates and returns the norm squared of the given three elements.
    * <p>
    * norm<sup>2</sup> = x<sup>2</sup> + y<sup>2</sup> + z<sup>2</sup>
    * </p>
    *
    * @param x the first element.
    * @param y the second element.
    * @param z the third element.
    * @return the value of the square of the norm.
    */
   public static double normSquared(double x, double y, double z)
   {
      return x * x + y * y + z * z;
   }

   /**
    * Calculates and returns the norm squared of the given four elements.
    * <p>
    * norm<sup>2</sup> = x<sup>2</sup> + y<sup>2</sup> + z<sup>2</sup> + s<sup>2</sup>
    * </p>
    *
    * @param x the first element.
    * @param y the second element.
    * @param z the third element.
    * @param s the fourth element.
    * @return the value of the square of the norm.
    */
   public static double normSquared(double x, double y, double z, double s)
   {
      return x * x + y * y + z * z + s * s;
   }

   /**
    * Calculates and returns the norm of the given two elements.
    * <p>
    * norm = &radic;(x<sup>2</sup> + y<sup>2</sup>)
    * </p>
    *
    * @param x the first element.
    * @param y the second element.
    * @return the value of the norm.
    */
   public static double norm(double x, double y)
   {
      return squareRoot(normSquared(x, y));
   }

   /**
    * Calculates and returns the norm of the given three elements.
    * <p>
    * norm = &radic;(x<sup>2</sup> + y<sup>2</sup> + z<sup>2</sup>)
    * </p>
    *
    * @param x the first element.
    * @param y the second element.
    * @param z the third element.
    * @return the value of the norm.
    */
   public static double norm(double x, double y, double z)
   {
      return squareRoot(normSquared(x, y, z));
   }

   /**
    * Calculates and returns the norm of the given four elements.
    * <p>
    * norm = &radic;(x<sup>2</sup> + y<sup>2</sup> + z<sup>2</sup> + s<sup>2</sup>)
    * </p>
    *
    * @param x the first element.
    * @param y the second element.
    * @param z the third element.
    * @param s the fourth element.
    * @return the value of the norm.
    */
   public static double norm(double x, double y, double z, double s)
   {
      return squareRoot(normSquared(x, y, z, s));
   }

   /**
    * Calculates and returns the norm of the given two elements.
    * <p>
    * norm = &radic;(x<sup>2</sup> + y<sup>2</sup>)
    * </p>
    * <p>
    * This method is optimized for calculating norms closed to 1 by using
    * {@link #fastSquareRoot(double)}. For computing norms that are not closed to 1, prefer using
    * {@link #norm(double, double)}.
    * </p>
    *
    * @param x the first element.
    * @param y the second element.
    * @return the value of the norm.
    */
   public static double fastNorm(double x, double y)
   {
      return fastSquareRoot(normSquared(x, y));
   }

   /**
    * Calculates and returns the norm of the given three elements.
    * <p>
    * norm = &radic;(x<sup>2</sup> + y<sup>2</sup> + z<sup>2</sup>)
    * </p>
    * <p>
    * This method is optimized for calculating norms closed to 1 by using
    * {@link #fastSquareRoot(double)}. For computing norms that are not closed to 1, prefer using
    * {@link #norm(double, double, double)}.
    * </p>
    *
    * @param x the first element.
    * @param y the second element.
    * @param z the third element.
    * @return the value of the norm.
    */
   public static double fastNorm(double x, double y, double z)
   {
      return fastSquareRoot(normSquared(x, y, z));
   }

   /**
    * Calculates and returns the norm of the given four elements.
    * <p>
    * norm = &radic;(x<sup>2</sup> + y<sup>2</sup> + z<sup>2</sup> + s<sup>2</sup>)
    * </p>
    * <p>
    * This method is optimized for calculating norms closed to 1 by using
    * {@link #fastSquareRoot(double)}. For computing norms that are not closed to 1, prefer using
    * {@link #norm(double, double, double, double)}.
    * </p>
    *
    * @param x the first element.
    * @param y the second element.
    * @param z the third element.
    * @param s the fourth element.
    * @return the value of the norm.
    */
   public static double fastNorm(double x, double y, double z, double s)
   {
      return fastSquareRoot(normSquared(x, y, z, s));
   }

   /**
    * Recomputes the angle value {@code angleToShift} such that the result is in [ -<i>pi</i>,
    * <i>pi</i> [ and still represent the same physical angle as {@code angleToShift}.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if {@code Math.abs(angleToShift + Math.PI) <} {@link #EPS_ANGLE_SHIFT}, the returned angle is
    * {@code -Math.PI}.
    * <li>if {@code Math.abs(angleToShift - Math.PI) <} {@link #EPS_ANGLE_SHIFT}, the returned angle is
    * {@code -Math.PI}.
    * </ul>
    * </p>
    *
    * @param angleToShift the angle to shift.
    * @return the result that is in [ -<i>pi</i>, <i>pi</i> [
    */
   public static double trimAngleMinusPiToPi(double angleToShift)
   {
      return shiftAngleInRange(angleToShift, -Math.PI);
   }

   /**
    * Computes the angle difference:<br>
    * {@code difference = angleA - angleB}<br>
    * and shift the result to be contained in [ -<i>pi</i>, <i>pi</i> [.
    *
    * @param angleA the first angle in the difference.
    * @param angleB the second angle in the difference.
    * @return the result of the subtraction contained in [ -<i>pi</i>, <i>pi</i> [.
    */
   public static double angleDifferenceMinusPiToPi(double angleA, double angleB)
   {
      return trimAngleMinusPiToPi(angleA - angleB);
   }

   /**
    * Recomputes the angle value {@code angleToShift} such that the result is in [{@code angleStart},
    * {@code angleStart} + 2<i>pi</i>[ and still represent the same physical angle as
    * {@code angleToShift}.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if {@code Math.abs(angleToShift - angleStart) <} {@link #EPS_ANGLE_SHIFT}, the returned angle
    * is {@code angleStart}.
    * <li>if {@code Math.abs(angleToShift - angleStart + 2.0 * Math.PI) <} {@link #EPS_ANGLE_SHIFT},
    * the returned angle is {@code angleStart}.
    * </ul>
    * </p>
    *
    * @param angleToShift the angle to shift.
    * @param angleStart   the lowest admissible angle value.
    * @return the result that is in [{@code angleStart}, {@code angleStart} + 2<i>pi</i>[
    */
   public static double shiftAngleInRange(double angleToShift, double angleStart)
   {
      angleStart = angleStart - EPS_ANGLE_SHIFT;

      double deltaFromStart = (angleToShift - angleStart) % TwoPI;

      if (deltaFromStart < 0)
         deltaFromStart += TwoPI;

      return angleStart + deltaFromStart;
   }

   /**
    * Find and return the argument with the maximum value.
    *
    * @param a the first argument to compare.
    * @param b the second argument to compare.
    * @param c the third argument to compare.
    * @return the maximum value of the three arguments.
    */
   public static double max(double a, double b, double c)
   {
      if (a > b)
         return a > c ? a : c;
      else
         return b > c ? b : c;
   }

   /**
    * Find and return the argument with the maximum value.
    *
    * @param a the first argument to compare.
    * @param b the second argument to compare.
    * @param c the third argument to compare.
    * @return the maximum value of the three arguments.
    */
   public static int max(int a, int b, int c)
   {
      if (a > b)
         return a > c ? a : c;
      else
         return b > c ? b : c;
   }

   /**
    * Find and return the argument with the maximum value.
    *
    * @param a the first argument to compare.
    * @param b the second argument to compare.
    * @param c the third argument to compare.
    * @param d the fourth argument to compare.
    * @return the maximum value of the four arguments.
    */
   public static double max(double a, double b, double c, double d)
   {
      if (a > b)
      {
         if (a > c)
            return a > d ? a : d;
         else
            return c > d ? c : d;
      }
      else
      {
         if (b > c)
            return b > d ? b : d;
         else
            return c > d ? c : d;
      }
   }

   /**
    * Find and return the argument with the maximum value.
    *
    * @param a the first argument to compare.
    * @param b the second argument to compare.
    * @param c the third argument to compare.
    * @param d the fourth argument to compare.
    * @return the maximum value of the four arguments.
    */
   public static int max(int a, int b, int c, int d)
   {
      if (a > b)
      {
         if (a > c)
            return a > d ? a : d;
         else
            return c > d ? c : d;
      }
      else
      {
         if (b > c)
            return b > d ? b : d;
         else
            return c > d ? c : d;
      }
   }

   /**
    * Find and return the argument with the minimum value.
    *
    * @param a the first argument to compare.
    * @param b the second argument to compare.
    * @param c the third argument to compare.
    * @return the minimum value of the three arguments.
    */
   public static double min(double a, double b, double c)
   {
      if (a < b)
         return a < c ? a : c;
      else
         return b < c ? b : c;
   }

   /**
    * Find and return the argument with the minimum value.
    *
    * @param a the first argument to compare.
    * @param b the second argument to compare.
    * @param c the third argument to compare.
    * @return the minimum value of the three arguments.
    */
   public static int min(int a, int b, int c)
   {
      if (a < b)
         return a < c ? a : c;
      else
         return b < c ? b : c;
   }

   /**
    * Find and return the argument with the minimum value.
    *
    * @param a the first argument to compare.
    * @param b the second argument to compare.
    * @param c the third argument to compare.
    * @param d the fourth argument to compare.
    * @return the minimum value of the four arguments.
    */
   public static double min(double a, double b, double c, double d)
   {
      if (a < b)
      {
         if (a < c)
            return a < d ? a : d;
         else
            return c < d ? c : d;
      }
      else
      {
         if (b < c)
            return b < d ? b : d;
         else
            return c < d ? c : d;
      }
   }

   /**
    * Find and return the argument with the minimum value.
    *
    * @param a the first argument to compare.
    * @param b the second argument to compare.
    * @param c the third argument to compare.
    * @param d the fourth argument to compare.
    * @return the minimum value of the four arguments.
    */
   public static int min(int a, int b, int c, int d)
   {
      if (a < b)
      {
         if (a < c)
            return a < d ? a : d;
         else
            return c < d ? c : d;
      }
      else
      {
         if (b < c)
            return b < d ? b : d;
         else
            return c < d ? c : d;
      }
   }

   /**
    * Find and return the argument with the value in between the two others.
    *
    * @param a the first argument to compare.
    * @param b the second argument to compare.
    * @param c the third argument to compare.
    * @return the value in between the two other arguments.
    */
   public static double med(double a, double b, double c)
   {
      if (a > b)
      {
         if (a > c)
            return b > c ? b : c;
         else
            return a;
      }
      else
      {
         if (b > c)
            return a > c ? a : c;
         else
            return b;
      }
   }

   /**
    * Find and return the argument with the value in between the two others.
    *
    * @param a the first argument to compare.
    * @param b the second argument to compare.
    * @param c the third argument to compare.
    * @return the value in between the two other arguments.
    */
   public static int med(int a, int b, int c)
   {
      if (a > b)
      {
         if (a > c)
            return b > c ? b : c;
         else
            return a;
      }
      else
      {
         if (b > c)
            return a > c ? a : c;
         else
            return b;
      }
   }

   /**
    * Test if the two values are equal with the special case of considering
    * {@code Double.NaN == Double.NaN}.
    *
    * @param expectedValue the first value to compare.
    * @param actualValue   the second value to compare.
    * @return {@code true} if the two values are considered to be equal, {@code false otherwise}.
    */
   public static boolean equals(double expectedValue, double actualValue)
   {
      if (expectedValue == actualValue)
         return true;
      else
         return Double.isNaN(expectedValue) && Double.isNaN(actualValue);
   }

   /**
    * Tests if the two values are equal to an {@code epsilon}:<br>
    * |{@code expectedValue} - {@code actualValue}| &leq; {@code epsilon}
    * <p>
    * If any of the two values is equal to {@link Double#NaN}, this method fails and returns
    * {@code false}.
    * </p>
    *
    * @param expectedValue the first value to compare.
    * @param actualValue   the second value to compare.
    * @param epsilon       the tolerance to use for the test.
    * @return {@code true} if the two values are considered to be equal, {@code false otherwise}.
    */
   public static boolean epsilonEquals(double expectedValue, double actualValue, double epsilon)
   {
      return expectedValue == actualValue || Math.abs(expectedValue - actualValue) <= epsilon;
   }

   /**
    * Tests if the two arguments are equal on a per component basis.
    * <p>
    * If both arguments are {@code null}, {@code true} is returned and if exactly one argument is
    * {@code null}, {@code false} is returned. Otherwise, equality is determined by using the
    * {@link EuclidGeometry#equals equals} method of the first argument.
    * </p>
    *
    * @param a the first geometry in the comparison. Not modified.
    * @param b the second geometry in the comparison. Not modified.
    * @return {@code true} if the arguments are equal to each other and {@code false} otherwise
    * @see EuclidGeometry#equals(EuclidGeometry)
    */
   public static boolean equals(EuclidGeometry a, EuclidGeometry b)
   {
      return Objects.equals(a, b);
   }

   /**
    * Tests if the two arguments are approximately equal on a per component basis.
    * <p>
    * If both arguments are {@code null}, {@code true} is returned and if exactly one argument is
    * {@code null}, {@code false} is returned. Otherwise, equality is determined by using the
    * {@link EuclidGeometry#epsilonEquals epsilonEquals} method of the first argument.
    * </p>
    *
    * @param a       the first geometry in the comparison. Not modified.
    * @param b       the second geometry in the comparison. Not modified.
    * @param epsilon tolerance to use when comparing each component.
    * @return {@code true} if the arguments are equal to each other and {@code false} otherwise
    * @see EuclidGeometry#epsilonEquals(EuclidGeometry, double)
    */
   public static boolean epsilonEquals(EuclidGeometry a, EuclidGeometry b, double epsilon)
   {
      return (a == b) || (a != null && a.epsilonEquals(b, epsilon));
   }

   /**
    * Tests if the two arguments represent the same geometry to an {@code epsilon}.
    * <p>
    * If both arguments are {@code null}, {@code true} is returned and if exactly one argument is
    * {@code null}, {@code false} is returned. Otherwise, equality is determined by using the
    * {@link EuclidGeometry#geometricallyEquals geometricallyEquals} method of the first argument.
    * </p>
    *
    * @param a       the first geometry in the comparison. Not modified.
    * @param b       the second geometry in the comparison. Not modified.
    * @param epsilon tolerance to use when comparing each component.
    * @return {@code true} if the arguments are equal to each other and {@code false} otherwise
    * @see EuclidGeometry#geometricallyEquals(EuclidGeometry, double)
    */
   public static boolean geometricallyEquals(EuclidGeometry a, EuclidGeometry b, double epsilon)
   {
      return (a == b) || (a != null && a.geometricallyEquals(b, epsilon));
   }

   /**
    * Tests whether the value is equal to zero to an epsilon.
    * <p>
    * This is equivalent to {@code epsilonEquals(value, 0.0, epsilon)}.
    * </p>
    *
    * @param value   the query.
    * @param epsilon the tolerance to use for the test.
    * @return {@code true} if the value can be considered zero, {@code false} otherwise.
    */
   public static boolean isZero(double value, double epsilon)
   {
      return Math.abs(value) <= epsilon;
   }

   /**
    * Tests whether both values are equal to zero to an epsilon.
    *
    * @param x       the first value.
    * @param y       the second value.
    * @param epsilon the tolerance to use for the test.
    * @return {@code true} if both values can be considered zero, {@code false} otherwise.
    */
   public static boolean areAllZero(double x, double y, double epsilon)
   {
      return isZero(x, epsilon) && isZero(y, epsilon);
   }

   /**
    * Tests whether all values are equal to zero to an epsilon.
    *
    * @param x       the first value.
    * @param y       the second value.
    * @param z       the third value.
    * @param epsilon the tolerance to use for the test.
    * @return {@code true} if all values can be considered zero, {@code false} otherwise.
    */
   public static boolean areAllZero(double x, double y, double z, double epsilon)
   {
      return isZero(x, epsilon) && isZero(y, epsilon) && isZero(z, epsilon);
   }

   /**
    * Tests whether all values are equal to zero to an epsilon.
    *
    * @param x       the first value.
    * @param y       the second value.
    * @param z       the third value.
    * @param s       the fourth value.
    * @param epsilon the tolerance to use for the test.
    * @return {@code true} if all values can be considered zero, {@code false} otherwise.
    */
   public static boolean areAllZero(double x, double y, double z, double s, double epsilon)
   {
      return isZero(x, epsilon) && isZero(y, epsilon) && isZero(z, epsilon) && isZero(s, epsilon);
   }

   /**
    * Tests whether the to angles represent the same physical angle to an {@code epsilon}.
    * <p>
    * If any of the two angles is equal to {@link Double#NaN}, this method fails and returns
    * {@code false}.
    * </p>
    *
    * @param expectedAngle the first angle in the comparison.
    * @param actualAngle   the second angle in the comparison.
    * @param epsilon       the tolerance to use for the test.
    * @return {@code true} if the two angles are considered to be geometrically equal, {@code false}
    *       otherwise.
    */
   public static boolean angleGeometricallyEquals(double expectedAngle, double actualAngle, double epsilon)
   {
      return Math.abs(angleDifferenceMinusPiToPi(expectedAngle, actualAngle)) <= epsilon;
   }

   /**
    * Tests whether the angle is geometrically equal to the zero angle to an epsilon.
    * <p>
    * This is equivalent to {@code angleGeometricallyEquals(angle, 0.0, epsilon)}.
    * </p>
    *
    * @param angle   the query.
    * @param epsilon the tolerance to use for the test.
    * @return {@code true} if the angle can be considered zero, {@code false} otherwise.
    */
   public static boolean isAngleZero(double angle, double epsilon)
   {
      angle = Math.abs(angle) % TwoPI;
      if (angle > Math.PI)
         angle -= TwoPI;
      return Math.abs(angle) <= epsilon;
   }

   /**
    * Clamps value to the given range, defined by {@code -minMax} and {@code minMax}, inclusive.
    *
    * @param value  value
    * @param minMax inclusive absolute boundary
    * @return <li>{@code -minMax} if {@code value} is less than {@code -minMax}</li>
    *       <li>{@code minMax} if {@code value} is greater than {@code minMax}</li>
    *       <li>{@code value} if {@code value} is between or equal to {@code -minMax} and
    *       {@code minMax}</li>
    */
   public static double clamp(double value, double minMax)
   {
      return clamp(value, -minMax, minMax);
   }

   /**
    * Clamps value to the given range, inclusive.
    *
    * @param value value
    * @param min   inclusive boundary start
    * @param max   inclusive boundary end
    * @return <li>{@code min} if {@code value} is less than {@code min}</li>
    *       <li>{@code max} if {@code value} is greater than {@code max}</li>
    *       <li>{@code value} if {@code value} is between or equal to {@code min} and
    *       {@code max}</li>
    */
   public static double clamp(double value, double min, double max)
   {
      if (min > max + CLAMP_EPS)
      {
         throw new RuntimeException(EuclidCoreTools.class.getSimpleName() + ".clamp(double, double, double): min > max (" + min + " > " + max + ")");
      }

      return Math.min(max, Math.max(value, min));
   }

   /**
    * Performs a linear interpolation from {@code a} to {@code b} given the percentage {@code alpha}.
    * <p>
    * result = (1.0 - alpha) * a + alpha * b
    * </p>
    *
    * @param a     the first value used in the interpolation.
    * @param b     the second value used in the interpolation.
    * @param alpha the percentage to use for the interpolation. A value of 0 will return {@code a},
    *              while a value of 1 will return {@code b}.
    * @return the interpolated value.
    */
   public static double interpolate(double a, double b, double alpha)
   {
      return (1.0 - alpha) * a + alpha * b;
   }

   /**
    * Returns the trigonometric tangent of an angle.
    *
    * @param a an angle, in radians.
    * @return the tangent of the argument.
    * @see Math#tan(double)
    */
   public static double tan(double a)
   {
      return StrictMath.tan(a);
   }

   /**
    * Returns the arc tangent of a value; the returned angle is in the range -<i>pi</i>/2 through
    * <i>pi</i>/2.
    *
    * @param a the value whose arc tangent is to be returned.
    * @return the arc tangent of the argument.
    * @see Math#atan(double)
    */
   public static double atan(double a)
   {
      return StrictMath.atan(a);
   }

   /**
    * Returns the angle <i>theta</i> from the conversion of rectangular coordinates
    * ({@code x},&nbsp;{@code y}) to polar coordinates (r,&nbsp;<i>theta</i>).
    *
    * @param y the ordinate coordinate
    * @param x the abscissa coordinate
    * @return the <i>theta</i> component of the point (<i>r</i>,&nbsp;<i>theta</i>) in polar
    *       coordinates that corresponds to the point (<i>x</i>,&nbsp;<i>y</i>) in Cartesian
    *       coordinates.
    * @see Math#atan2(double, double)
    */
   public static double atan2(double y, double x)
   {
      return StrictMath.atan2(y, x);
   }

   /**
    * Returns the trigonometric cosine of an angle.
    *
    * @param a an angle, in radians.
    * @return the cosine of the argument.
    * @see Math#cos(double)
    */
   public static double cos(double a)
   {
      return StrictMath.cos(a);
   }

   /**
    * Returns the trigonometric sine of an angle.
    *
    * @param a an angle, in radians.
    * @return the sine of the argument.
    * @see Math#sin(double)
    */
   public static double sin(double a)
   {
      return StrictMath.sin(a);
   }

   /**
    * Returns the arc cosine of a value; the returned angle is in the range 0.0 through <i>pi</i>.
    *
    * @param a the value whose arc cosine is to be returned.
    * @return the arc cosine of the argument.
    * @see Math#acos(double)
    */
   public static double acos(double a)
   {
      return StrictMath.acos(a);
   }

   /**
    * Variation of {@link Math#acos(double)} function that relies on the following identity:
    *
    * <pre>
    * acos(cosX) = 2 atan2(&Sqrt;(1 - cosX<sup>2</sup>), 1 + cosX)
    * </pre>
    *
    * @param cosX the value whose arc cosine is to be returned. The value is clamped to be within [-1,
    *             1].
    * @return the arc cosine of the argument.
    */
   public static double fastAcos(double cosX)
   {
      if (cosX == -1.0)
         return Math.PI;
      else
         return 2.0 * atan2(squareRoot(1.0 - cosX * cosX), 1.0 + cosX);
   }

   /**
    * Returns the arc sine of a value; the returned angle is in the range -<i>pi</i>/2 through
    * <i>pi</i>/2.
    *
    * @param a the value whose arc sine is to be returned.
    * @return the arc sine of the argument.
    * @see Math#asin(double)
    */
   public static double asin(double a)
   {
      return StrictMath.asin(a);
   }

   /**
    * Checks that the given {@code matrixToTest} has a minimum size of [{@code minRows},
    * {@code minColumns}].
    *
    * @param minRows      the minimum number of rows that the matrix should have.
    * @param minColumns   the minimum number of columns that the matrix should have.
    * @param matrixToTest the matrix which size is to be tested.
    * @throws MatrixDimensionException if the matrix does not meet the minimum size.
    */
   public static void checkMatrixMinimumSize(int minRows, int minColumns, Matrix matrixToTest)
   {
      if (matrixToTest.getNumCols() < minColumns || matrixToTest.getNumRows() < minRows)
      {
         throw new MatrixDimensionException(
               "The matrix is too small, expected: [nRows >= " + minRows + ", nColumns >= " + minColumns + "], was: [nRows = " + matrixToTest.getNumRows()
               + ", nCols = " + matrixToTest.getNumCols() + "].");
      }
   }

   /**
    * Reverses in the specified list the order of the elements that are in the range
    * [{@code fromIndex}, {@code toIndex}[.
    * <p>
    * This method is garbage free and is equivalent to
    * {@code Collections.reverse(list.subList(fromIndex, toIndex))}.
    * </p>
    *
    * @param list      the list whose elements are to be reversed. Modified.
    * @param fromIndex low endpoint (inclusive) of the range to be reversed.
    * @param toIndex   high endpoint (exclusive) of the range to be reversed.
    */
   public static void reverse(List<?> list, int fromIndex, int toIndex)
   {
      int start = fromIndex;
      int end = toIndex - 1;

      while (start < end)
      {
         Collections.swap(list, start++, end--);
      }
   }

   /**
    * Rotates the elements of the {@code list} in the range [{@code fromIndex}, {@code toIndex}[ by the
    * given {@code distance}.
    * <p>
    * This method is garbage free and is equivalent to
    * {@code Collections.rotate(list.subList(fromIndex, toIndex), shift)}.
    * </p>
    * <p>
    * Here are few examples:
    * <ul>
    * <li>{@code list=[0, 1, 2, 3, 4]}, {@code rotate(list, 0, list.size(), -1)} gives:
    * {@code [1, 2, 3, 4, 0]}.
    * <li>{@code list=[0, 1, 2, 3, 4]}, {@code rotate(list, 0, list.size(), +1)} gives:
    * {@code [4, 0, 1, 2, 3]}.
    * <li>{@code list=[9, 0, 1, 2, 9]}, {@code rotate(list, 1, 4, -1)} gives:
    * {@code [9, 1, 2, 0, 9]}.
    * <li>{@code list=[9, 0, 1, 2, 9]}, {@code rotate(list, 1, 4, +1)} gives:
    * {@code [9, 2, 0, 1, 9]}.
    * </ul>
    * </p>
    *
    * @param <T>       the element type.
    * @param list      the list whose elements are to be rotated. Modified.
    * @param fromIndex low endpoint (inclusive) of the range to be rotated.
    * @param toIndex   high endpoint (exclusive) of the range to be rotated.
    * @param distance  the distance by which the elements are to be rotated. There are no constraints
    *                  on this value; it may be zero, negative, or greater than {@code list.size()}.
    * @see Collections#rotate(List, int)
    */
   public static <T> void rotate(List<T> list, int fromIndex, int toIndex, int distance)
   {
      int size = toIndex - fromIndex;

      if (size <= 1)
         return;

      distance = wrap(distance, size);

      if (distance == 0)
         return;

      for (int cycleStart = fromIndex, nMoved = 0; nMoved != size; cycleStart++)
      {
         T shifted = list.get(cycleStart);
         int i = cycleStart;
         do
         {
            i += distance;
            if (i >= toIndex)
               i -= size;
            shifted = list.set(i, shifted);
            nMoved++;
         }
         while (i != cycleStart);
      }
   }

   /**
    * Recomputes the given {@code index} such that it is &in; [0, {@code listSize}[.
    * <p>
    * The {@code index} remains unchanged if already &in; [0, {@code listSize}[.
    * <p>
    * Examples:
    * <ul>
    * <li>{@code wrap(-1, 10)} returns 9.
    * <li>{@code wrap(10, 10)} returns 0.
    * <li>{@code wrap( 5, 10)} returns 5.
    * <li>{@code wrap(15, 10)} returns 5.
    * </ul>
    * </p>
    *
    * @param index    the index to be wrapped if necessary.
    * @param listSize the size of the list around which the index is to be wrapped.
    * @return the wrapped index.
    */
   public static int wrap(int index, int listSize)
   {
      index %= listSize;
      if (index < 0)
         index += listSize;
      return index;
   }

   /**
    * Increments the given {@code index} such that it is &in; [0, {@code listSize}[.
    * Examples:
    * <ul>
    * <li>{@code next(-1, 10)} returns 0.
    * <li>{@code next(10, 10)} returns 1.
    * <li>{@code next( 5, 10)} returns 6.
    * <li>{@code next(15, 10)} returns 6.
    * </ul>
    * </p>
    *
    * @param index    the index to be incremented and wrapped if necessary.
    * @param listSize the size of the list around which the index is to be wrapped.
    * @return the wrapped incremented index.
    */
   public static int next(int index, int listSize)
   {
      return wrap(index + 1, listSize);
   }

   /**
    * Decrements the given {@code index} such that it is &in; [0, {@code listSize}[.
    * Examples:
    * <ul>
    * <li>{@code next(-1, 10)} returns 10.
    * <li>{@code next(10, 10)} returns 9.
    * <li>{@code next( 5, 10)} returns 4.
    * <li>{@code next(15, 10)} returns 4.
    * </ul>
    * </p>
    *
    * @param index    the index to be decremented and wrapped if necessary.
    * @param listSize the size of the list around which the index is to be wrapped.
    * @return the wrapped decremented index.
    */
   public static int previous(int index, int listSize)
   {
      return wrap(index - 1, listSize);
   }

   /**
    * Computes the finite difference of the two given values.
    *
    * @param previousValue the value at the previous time step.
    * @param currentValue  the value at the current time step.
    * @param dt            the time step.
    * @return the derivative.
    */
   public static double finiteDifference(double previousValue, double currentValue, double dt)
   {
      return (currentValue - previousValue) / dt;
   }

   /**
    * Computes the finite difference of the two given angles.
    * <p>
    * This method handles <tt>2&pi;</tt> wrap around.
    * </p>
    *
    * @param previousAngle the angle (in radians) at the previous time step.
    * @param currentAngle  the angle (in radians) at the current time step.
    * @param dt            the time step.
    * @return the angular velocity in radians per second.
    */
   public static double finiteDifferenceAngle(double previousAngle, double currentAngle, double dt)
   {
      return angleDifferenceMinusPiToPi(currentAngle, previousAngle) / dt;
   }

   /**
    * Computes the finite difference of the two given values.
    *
    * @param previousValue    the value at the previous time step.
    * @param currentValue     the value at the current time step.
    * @param dt               the time step.
    * @param derivativeToPack the vector used to store the derivative. Modified.
    */
   public static void finiteDifference(Tuple2DReadOnly previousValue, Tuple2DReadOnly currentValue, double dt, Vector2DBasics derivativeToPack)
   {
      derivativeToPack.sub(currentValue, previousValue);
      derivativeToPack.scale(1.0 / dt);
   }

   /**
    * Computes the finite difference of the two given values.
    *
    * @param previousValue    the value at the previous time step.
    * @param currentValue     the value at the current time step.
    * @param dt               the time step.
    * @param derivativeToPack the vector used to store the derivative. Modified.
    */
   public static void finiteDifference(Tuple3DReadOnly previousValue, Tuple3DReadOnly currentValue, double dt, Vector3DBasics derivativeToPack)
   {
      derivativeToPack.sub(currentValue, previousValue);
      derivativeToPack.scale(1.0 / dt);
   }

   /**
    * Computes the finite difference of the two given values.
    *
    * @param previousValue    the value at the previous time step.
    * @param currentValue     the value at the current time step.
    * @param dt               the time step.
    * @param derivativeToPack the vector used to store the derivative. Modified.
    */
   public static void finiteDifference(Tuple4DReadOnly previousValue, Tuple4DReadOnly currentValue, double dt, Vector4DBasics derivativeToPack)
   {
      derivativeToPack.sub(currentValue, previousValue);
      derivativeToPack.scale(1.0 / dt);
   }

   /**
    * Computes the angular velocity from the finite difference of the two given orientations.
    *
    * @param previousOrientation the orientation at the previous time step. Not modified.
    * @param currentOrientation  the orientation at the current time step. Not modified.
    * @param dt                  the time step.
    * @return the angular velocity in radians per second.
    */
   public static double finiteDifference(Orientation2DReadOnly previousOrientation, Orientation2DReadOnly currentOrientation, double dt)
   {
      return finiteDifferenceAngle(previousOrientation.getYaw(), currentOrientation.getYaw(), dt);
   }

   /**
    * Computes the angular velocity from the finite difference of the two given orientations.
    * <p>
    * The resulting angular velocity is expressed in the local coordinates of the orientation. Note that the angular velocity can be considered to be expressed
    * the local frame described by either the previous or current orientation, i.e. <tt>&omega; = R<sub>current</sub><sup>previous</sup> &omega;</tt>.
    * </p>
    * <p>
    * The method identifies the type of orientation used to perform the finite difference and calls the appropriate method. Note though that to get best
    * performance, it is recommended to compute the finite difference using two quaternions.
    * </p>
    *
    * @param previousOrientation   the orientation at the previous time step. Not modified.
    * @param currentOrientation    the orientation at the current time step. Not modified.
    * @param dt                    the time step.
    * @param angularVelocityToPack the vector used to store the angular velocity expressed in the orientation's local coordinates. Modified.
    */
   public static void finiteDifference(Orientation3DReadOnly previousOrientation,
                                       Orientation3DReadOnly currentOrientation,
                                       double dt,
                                       Vector3DBasics angularVelocityToPack)
   {
      if (previousOrientation instanceof QuaternionReadOnly qPrev)
      {
         if (currentOrientation instanceof QuaternionReadOnly qCurr)
            QuaternionTools.finiteDifference(qPrev, qCurr, dt, angularVelocityToPack);
         else if (currentOrientation instanceof RotationMatrixReadOnly rCurr)
            RotationMatrixTools.finiteDifference(qPrev, rCurr, dt, angularVelocityToPack);
         else if (currentOrientation instanceof AxisAngleReadOnly aaCurr)
            QuaternionTools.finiteDifference(qPrev, aaCurr, dt, angularVelocityToPack);
         else if (currentOrientation instanceof YawPitchRollReadOnly yprCurr)
            QuaternionTools.finiteDifference(qPrev, yprCurr, dt, angularVelocityToPack);
         else
            throw newUnsupportedOrientationException(previousOrientation, currentOrientation);
      }
      else if (previousOrientation instanceof RotationMatrixReadOnly rPrev)
      {
         if (currentOrientation instanceof RotationMatrixReadOnly rCurr)
            RotationMatrixTools.finiteDifference(rPrev, rCurr, dt, angularVelocityToPack);
         else if (currentOrientation instanceof QuaternionReadOnly qCurr)
            RotationMatrixTools.finiteDifference(rPrev, qCurr, dt, angularVelocityToPack);
         else if (currentOrientation instanceof AxisAngleReadOnly aaCurr)
            RotationMatrixTools.finiteDifference(rPrev, aaCurr, dt, angularVelocityToPack);
         else if (currentOrientation instanceof YawPitchRollReadOnly yprCurr)
            RotationMatrixTools.finiteDifference(rPrev, yprCurr, dt, angularVelocityToPack);
         else
            throw newUnsupportedOrientationException(previousOrientation, currentOrientation);
      }
      else if (previousOrientation instanceof AxisAngleReadOnly aaPrev)
      {
         if (currentOrientation instanceof AxisAngleReadOnly aaCurr)
            AxisAngleTools.finiteDifference(aaPrev, aaCurr, dt, angularVelocityToPack);
         else if (currentOrientation instanceof QuaternionReadOnly qCurr)
            QuaternionTools.finiteDifference(aaPrev, qCurr, dt, angularVelocityToPack);
         else if (currentOrientation instanceof RotationMatrixReadOnly rCurr)
            RotationMatrixTools.finiteDifference(aaPrev, rCurr, dt, angularVelocityToPack);
         else if (currentOrientation instanceof YawPitchRollReadOnly yprCurr)
            YawPitchRollTools.finiteDifference(aaPrev, yprCurr, dt, angularVelocityToPack);
         else
            throw newUnsupportedOrientationException(previousOrientation, currentOrientation);
      }
      else if (previousOrientation instanceof YawPitchRollReadOnly yprPrev)
      {
         if (currentOrientation instanceof YawPitchRollReadOnly yprCurr)
            YawPitchRollTools.finiteDifference(yprPrev, yprCurr, dt, angularVelocityToPack);
         else if (currentOrientation instanceof RotationMatrixReadOnly rCurr)
            RotationMatrixTools.finiteDifference(yprPrev, rCurr, dt, angularVelocityToPack);
         else if (currentOrientation instanceof QuaternionReadOnly qCurr)
            QuaternionTools.finiteDifference(yprPrev, qCurr, dt, angularVelocityToPack);
         else if (currentOrientation instanceof AxisAngleReadOnly aaCurr)
            YawPitchRollTools.finiteDifference(yprPrev, aaCurr, dt, angularVelocityToPack);
         else
            throw newUnsupportedOrientationException(previousOrientation, currentOrientation);
      }
      else
         throw newUnsupportedOrientationException(previousOrientation, currentOrientation);
   }

   /**
    * Computes the linear and angular velocities from the finite difference of the two given transforms.
    * <p>
    * Note that:
    * <ul>
    *    <li>the linear velocity is expressed in the base coordinates (world frame).
    *    <li>the angular velocity is expressed in the local coordinates of the transform.
    * </ul>
    * </p>
    * <p>
    *  @param previousTransform     the transform at the previous time step. Not modified.
    *
    * @param currentTransform      the transform at the current time step. Not modified.
    * @param dt                    the time step.
    * @param angularVelocityToPack the vector used to store the angular velocity. Modified.
    * @param linearVelocityToPack  the vector used to store the linear velocity. Modified.
    */
   public static void finiteDifference(RigidBodyTransformReadOnly previousTransform,
                                       RigidBodyTransformReadOnly currentTransform,
                                       double dt,
                                       Vector3DBasics angularVelocityToPack,
                                       Vector3DBasics linearVelocityToPack)
   {
      finiteDifference(previousTransform.getRotation(), currentTransform.getRotation(), dt, angularVelocityToPack);
      finiteDifference(previousTransform.getTranslation(), currentTransform.getTranslation(), dt, linearVelocityToPack);
   }

   /**
    * First order integration of the given derivative to compute the current value.
    *
    * @param previousValue the value at the previous time step.
    * @param derivative    the time derivative.
    * @param dt            the time step.
    * @return the current value.
    */
   public static double integrate(double previousValue, double derivative, double dt)
   {
      return previousValue + dt * derivative;
   }

   /**
    * First order integration of the given derivative to compute the current angle.
    *
    * @param previousAngle   the angle (in radians) at the previous time step.
    * @param angularVelocity the angular velocity (in radians per second).
    * @param dt              the time step.
    * @return the current angle (in radians).
    */
   public static double integrateAngle(double previousAngle, double angularVelocity, double dt)
   {
      return trimAngleMinusPiToPi(previousAngle + dt * angularVelocity);
   }

   /**
    * First order integration of the given derivative to compute the current value.
    *
    * @param previousValue      the value at the previous time step.
    * @param derivative         the time derivative. Not modified.
    * @param dt                 the time step.
    * @param currentValueToPack the vector used to store the current value. Modified.
    */
   public static void integrate(Tuple2DReadOnly previousValue, Vector2DReadOnly derivative, double dt, Tuple2DBasics currentValueToPack)
   {
      currentValueToPack.scaleAdd(dt, derivative, previousValue);
   }

   /**
    * First order integration of the given derivative to compute the current value.
    *
    * @param previousValue      the value at the previous time step.
    * @param derivative         the time derivative. Not modified.
    * @param dt                 the time step.
    * @param currentValueToPack the vector used to store the current value. Modified.
    */
   public static void integrate(Tuple3DReadOnly previousValue, Vector3DReadOnly derivative, double dt, Tuple3DBasics currentValueToPack)
   {
      currentValueToPack.scaleAdd(dt, derivative, previousValue);
   }

   /**
    * First order integration of the given derivative to compute the current value.
    *
    * @param previousValue      the value at the previous time step.
    * @param derivative         the time derivative. Not modified.
    * @param dt                 the time step.
    * @param currentValueToPack the vector used to store the current value. Modified.
    */
   public static void integrate(Tuple4DReadOnly previousValue, Vector4DReadOnly derivative, double dt, Tuple4DBasics currentValueToPack)
   {
      currentValueToPack.set(previousValue.getX() + dt * derivative.getX(),
                             previousValue.getY() + dt * derivative.getY(),
                             previousValue.getZ() + dt * derivative.getZ(),
                             previousValue.getS() + dt * derivative.getS());
   }

   /**
    * First order integration of the given derivative to compute the current orientation.
    *
    * @param previousOrientation      the orientation at the previous time step. Not modified.
    * @param angularVelocity          the angular velocity (in radians per second). Not modified.
    * @param dt                       the time step.
    * @param currentOrientationToPack the orientation used to store the current orientation. Modified.
    */
   public static void integrate(Orientation2DReadOnly previousOrientation, double angularVelocity, double dt, Orientation2DBasics currentOrientationToPack)
   {
      currentOrientationToPack.setYaw(integrateAngle(previousOrientation.getYaw(), angularVelocity, dt));
   }

   /**
    * First order integration of the given derivative to compute the current orientation.
    * <p>
    * Note that the angular velocity is expected to be expressed in the local coordinates of the orientation. If not, perform the following operation before
    * calling this method: {@code previousOrientation.inverseTransform(angularVelocity)}.
    * </p>
    *
    * @param previousOrientation      the orientation at the previous time step. Not modified.
    * @param angularVelocity          the angular velocity (in radians per second) expressed in the local coordinates of the orientation. Not modified.
    * @param dt                       the time step.
    * @param currentOrientationToPack the orientation used to store the current orientation. Modified.
    */
   public static void integrate(Orientation3DReadOnly previousOrientation,
                                Vector3DReadOnly angularVelocity,
                                double dt,
                                Orientation3DBasics currentOrientationToPack)
   {
      double rx = angularVelocity.getX() * dt;
      double ry = angularVelocity.getY() * dt;
      double rz = angularVelocity.getZ() * dt;

      if (currentOrientationToPack instanceof QuaternionBasics qCurr)
      {
         QuaternionTools.appendRotationVector(previousOrientation, rx, ry, rz, qCurr);
      }
      else if (currentOrientationToPack instanceof RotationMatrixBasics rCurr)
      {
         RotationMatrixTools.appendRotationVector(previousOrientation, rx, ry, rz, rCurr);
      }
      else if (currentOrientationToPack instanceof AxisAngleBasics aaCurr)
      {
         AxisAngleTools.appendRotationVector(previousOrientation, rx, ry, rz, aaCurr);
      }
      else if (currentOrientationToPack instanceof YawPitchRollBasics yprCurr)
      {
         YawPitchRollTools.appendRotationVector(previousOrientation, rx, ry, rz, yprCurr);
      }
      else
      {
         throw newUnsupportedOrientationException(previousOrientation, currentOrientationToPack);
      }
   }

   /**
    * First order integration of the given derivative to compute the current transform.
    * <p>
    * Note that:
    * <ul>
    *    <li>the linear velocity is expressed in the base coordinates (world frame).
    *    <li>the angular velocity is expressed in the local coordinates of the transform.
    * </ul>
    * </p>
    *
    * @param previousTransform      the transform at the previous time step. Not modified.
    * @param angularVelocity        the angular velocity. Not modified.
    * @param linearVelocity         the linear velocity. Not modified.
    * @param dt                     the time step.
    * @param currentTransformToPack the transform used to store the current transform. Modified.
    */
   public static void integrate(RigidBodyTransformReadOnly previousTransform,
                                Vector3DReadOnly angularVelocity,
                                Vector3DReadOnly linearVelocity,
                                double dt,
                                RigidBodyTransformBasics currentTransformToPack)
   {
      integrate(previousTransform.getRotation(), angularVelocity, dt, currentTransformToPack.getRotation());
      integrate(previousTransform.getTranslation(), linearVelocity, dt, currentTransformToPack.getTranslation());
   }

   private static UnsupportedOperationException newUnsupportedOrientationException(Orientation3DReadOnly previousOrientation,
                                                                                   Orientation3DReadOnly currentOrientation)
   {
      return new UnsupportedOperationException("Unsupported orientation type: [currentOrientation = %s, previousOrientation = %s].".formatted(currentOrientation.getClass()
                                                                                                                                                                .getSimpleName(),
                                                                                                                                              previousOrientation.getClass()
                                                                                                                                                                 .getSimpleName()));
   }

   /**
    * Checks whether all the elements contained within this tuple are finite.
    *
    * @param tuple Tuple to inspect.
    * @return {@code true} if all elements are finite, {@code false} if any is not.
    */
   public static boolean isFinite(Tuple3DReadOnly tuple)
   {
      return Double.isFinite(tuple.getX()) && Double.isFinite(tuple.getY()) && Double.isFinite(tuple.getZ());
   }

   /**
    * Checks whether two rigid body transforms are equal within some epislon.
    *
    * @param a                  first transform to check.
    * @param b                  first transform to check.
    * @param rotationEpsilon    epsilon tolerance for the rotation between the two.
    * @param translationEpsilon epsilon tolerance for the translation between the two.
    * @return {@code true} if the two transforms are equal, {@code false} if not.
    */
   public static boolean epsilonEquals(RigidBodyTransformReadOnly a, RigidBodyTransformReadOnly b, double rotationEpsilon, double translationEpsilon)
   {
      return a.getRotation().geometricallyEquals(b.getRotation(), rotationEpsilon) && a.getTranslation()
                                                                                       .geometricallyEquals(b.getTranslation(), translationEpsilon);
   }

   /**
    * Computes the angle in radians from the first 3D vector to the second 3D vector. The computed
    * angle is in the range [0; <i>pi</i>].
    *
    * @param firstVectorX          x value of the first vector.
    * @param firstVectorY          y value of the first vector.
    * @param firstVectorZ          z value of the first vector.
    * @param isFirstVectorUnitary  whether the first vector is a unit vector.
    * @param secondVectorX         x value of the second vector.
    * @param secondVectorY         y value of the second vector.
    * @param secondVectorZ         z value of the second vector.
    * @param isSecondVectorUnitary whether the second vector is a unit vector.
    * @return the angle in radians from the first vector to the second vector.
    */
   public static double angleFromFirstToSecondVector3D(double firstVectorX,
                                                       double firstVectorY,
                                                       double firstVectorZ,
                                                       boolean isFirstVectorUnitary,
                                                       double secondVectorX,
                                                       double secondVectorY,
                                                       double secondVectorZ,
                                                       boolean isSecondVectorUnitary)
   {
      double firstVectorLength = isFirstVectorUnitary ? 1.0 : norm(firstVectorX, firstVectorY, firstVectorZ);
      double secondVectorLength = isSecondVectorUnitary ? 1.0 : norm(secondVectorX, secondVectorY, secondVectorZ);

      double dotProduct = firstVectorX * secondVectorX + firstVectorY * secondVectorY + firstVectorZ * secondVectorZ;
      dotProduct /= firstVectorLength * secondVectorLength;

      if (dotProduct > 1.0)
         dotProduct = 1.0;
      else if (dotProduct < -1.0)
         dotProduct = -1.0;

      return acos(dotProduct);
   }

   /**
    * Computes the angle in radians from the first 3D vector to the second 3D vector. The computed
    * angle is in the range [0; <i>pi</i>].
    *
    * @param firstVector  the first vector. Not modified.
    * @param secondVector the second vector. Not modified.
    * @return the angle in radians from the first vector to the second vector.
    */
   public static double angleFromFirstToSecondVector3D(Vector3DReadOnly firstVector, Vector3DReadOnly secondVector)
   {
      return angleFromFirstToSecondVector3D(firstVector.getX(),
                                            firstVector.getY(),
                                            firstVector.getZ(),
                                            firstVector instanceof UnitVector3DReadOnly,
                                            secondVector.getX(),
                                            secondVector.getY(),
                                            secondVector.getZ(),
                                            secondVector instanceof UnitVector3DReadOnly);
   }

   /**
    * Calculate the angular velocity by differentiating orientation.
    *
    * @param qStart                the initial orientation at time t.
    * @param qEnd                  the final orientation at time t + duration.
    * @param duration              the time interval between the 2 orientations.
    * @param angularVelocityToPack the angular velocity.
    */
   public static void differentiateOrientation(QuaternionReadOnly qStart, QuaternionReadOnly qEnd, double duration, Vector3DBasics angularVelocityToPack)
   {
      double q1x = qStart.getX();
      double q1y = qStart.getY();
      double q1z = qStart.getZ();
      double q1s = qStart.getS();

      double q2x = qEnd.getX();
      double q2y = qEnd.getY();
      double q2z = qEnd.getZ();
      double q2s = qEnd.getS();

      double diffx = q1s * q2x - q1x * q2s - q1y * q2z + q1z * q2y;
      double diffy = q1s * q2y + q1x * q2z - q1y * q2s - q1z * q2x;
      double diffz = q1s * q2z - q1x * q2y + q1y * q2x - q1z * q2s;
      double diffs = q1s * q2s + q1x * q2x + q1y * q2y + q1z * q2z;

      if (diffs < 0.0)
      {
         diffx = -diffx;
         diffy = -diffy;
         diffz = -diffz;
         diffs = -diffs;
      }

      double sinHalfTheta = norm(diffx, diffy, diffz);

      double angle;
      if (epsilonEquals(1.0, diffs, 1.0e-12))
         angle = 2.0 * sinHalfTheta / diffs;
      else
         angle = 2.0 * atan2(sinHalfTheta, diffs);
      angularVelocityToPack.set(diffx, diffy, diffz);
      angularVelocityToPack.scale(angle / (sinHalfTheta * duration));
   }

   /**
    * Sets the yaw pitch roll but the doubles are given in degrees.
    */
   public static void setYawPitchRollDegrees(Orientation3DBasics orientation3DBasics, double yaw, double pitch, double roll)
   {
      orientation3DBasics.setYawPitchRoll(Math.toRadians(yaw), Math.toRadians(pitch), Math.toRadians(roll));
   }

   /**
    * Projects the provided {@code rotation} onto {@code axis} such that the original rotation can be
    * decomposed into a rotation around {@code axis} and one around an orthogonal axis.
    * <p>
    * rotation = orthogonalRotation * result
    * </p>
    *
    * @param rotation is the original rotation to be projected onto {@code axis}
    * @param axis     is the desired rotation axis of the result.
    * @param result   will be modified to contain the component of {@code rotation} that is around
    *                 {@code axis}
    */
   public static void projectRotationOnAxis(QuaternionReadOnly rotation, Vector3DReadOnly axis, QuaternionBasics result)
   {
      double dotProduct = rotation.getX() * axis.getX() + rotation.getY() * axis.getY() + rotation.getZ() * axis.getZ();

      double scale = dotProduct / axis.normSquared();
      double projectedX = scale * axis.getX();
      double projectedY = scale * axis.getY();
      double projectedZ = scale * axis.getZ();

      result.set(projectedX, projectedY, projectedZ, rotation.getS());
      result.normalize();
   }

   /**
    * Calculates the 3D part of the given {@code input} that is parallel to the given
    * {@code normalAxis} and stores the result in {@code inputNormalPartToPack}.
    * <p>
    * The result has the following properties:
    * <ul>
    * <li><tt>x.n = x<sub>n</sub>.n</tt>
    * <li><tt>|x<sub>n</sub>&times;n| = 0</tt>
    * </ul>
    * where:
    * <ul>
    * <li><tt>x</tt> is {@code input}.
    * <li></tt>n</tt> is {@code normalAxis}.
    * <li><tt>x<sub>n</sub></tt> is {@code inputNormalPartToPack}.
    * </ul>
    * </p>
    *
    * @param input                 the tuple to extract the normal part of. Not modified.
    * @param normalAxis            the normal vector. It is normalized internally if needed. Not
    *                              modified.
    * @param inputNormalPartToPack the tuple used to store the normal part of the input. Modified.
    */
   public static void extractNormalPart(Tuple3DReadOnly input, Vector3DReadOnly normalAxis, Tuple3DBasics inputNormalPartToPack)
   {
      double normalX = normalAxis.getX();
      double normalY = normalAxis.getY();
      double normalZ = normalAxis.getZ();
      double normalLengthSquared = normSquared(normalX, normalY, normalZ);

      double dot = TupleTools.dot(normalAxis, input) / normalLengthSquared;
      inputNormalPartToPack.setAndScale(dot, normalAxis);
   }

   /**
    * Calculates the 3D part of the given {@code input} that is orthogonal to the given
    * {@code normalAxis} and stores the result in {@code inputTangentialPartToPack}.
    * <p>
    * The result has the following properties:
    * <ul>
    * <li><tt>x<sub>t</sub>.n = 0</tt>
    * <li><tt>|x - (x.n)n| = |x<sub>t</sub>|</tt>
    * </ul>
    * where:
    * <ul>
    * <li><tt>x</tt> is {@code input}.
    * <li></tt>n</tt> is {@code normalAxis}.
    * <li><tt>x<sub>t</sub></tt> is {@code inputTangentialPartToPack}.
    * </ul>
    * </p>
    *
    * @param input                    the tuple to extract the tangential part of. Not modified.
    * @param normalAxis               the normal vector. It is normalized internally if needed. Not
    *                                 modified.
    * @param inputTagentialPartToPack the tuple used to store the tangential part of the input.
    *                                 Modified.
    */
   public static void extractTangentialPart(Tuple3DReadOnly input, Vector3DReadOnly normalAxis, Tuple3DBasics inputTagentialPartToPack)
   {
      double normalX = normalAxis.getX();
      double normalY = normalAxis.getY();
      double normalZ = normalAxis.getZ();
      double normalLengthSquared = normSquared(normalX, normalY, normalZ);

      double dot = TupleTools.dot(normalX, normalY, normalZ, input) / normalLengthSquared;
      double normalPartX = dot * normalX;
      double normalPartY = dot * normalY;
      double normalPartZ = dot * normalZ;

      inputTagentialPartToPack.set(input);
      inputTagentialPartToPack.sub(normalPartX, normalPartY, normalPartZ);
   }

   /**
    * Modifies {@code tupleToModify} such that its normal part is equal to the normal part of
    * {@code input}.
    * <p>
    * This method performs the following calculation: <tt>x = x - (x.n)n + (y.n)n</tt>, where:
    * <ul>
    * <li><tt>x</tt> is {@code tupleToModify}.
    * <li></tt>n</tt> is {@code normalAxis}.
    * <li><tt>y</tt> is {@code input}.
    * </ul>
    * </p>
    *
    * @param input         the tuple containing the normal part used to update {@code tupleToModify}.
    *                      Not modified.
    * @param normalAxis    the normal vector. It is normalized internally if needed. Not modified.
    * @param tupleToModify the tuple to modify the normal part of. Modified.
    */
   public static void setNormalPart(Tuple3DReadOnly input, Vector3DReadOnly normalAxis, Tuple3DBasics tupleToModify)
   {
      double normalX = normalAxis.getX();
      double normalY = normalAxis.getY();
      double normalZ = normalAxis.getZ();
      double normalLengthSquared = normSquared(normalX, normalY, normalZ);
      double dot = (TupleTools.dot(normalAxis, input) - TupleTools.dot(normalAxis, tupleToModify)) / normalLengthSquared;
      tupleToModify.scaleAdd(dot, normalAxis, tupleToModify);
   }

   /**
    * Generic transform operation to apply a rotation matrix {@code matrix} to a tuple 3d.
    *
    * @param matrix           rotation matrix to apply
    * @param xOriginal        x value of the tuple
    * @param yOriginal        y value of the tuple
    * @param zOriginal        z value of the tuple
    * @param tupleTransformed transformed tuple 3D to compute.
    */
   public static void transform(Matrix3DReadOnly matrix, double xOriginal, double yOriginal, double zOriginal, Tuple3DBasics tupleTransformed)
   {
      double x = matrix.getM00() * xOriginal + matrix.getM01() * yOriginal + matrix.getM02() * zOriginal;
      double y = matrix.getM10() * xOriginal + matrix.getM11() * yOriginal + matrix.getM12() * zOriginal;
      double z = matrix.getM20() * xOriginal + matrix.getM21() * yOriginal + matrix.getM22() * zOriginal;
      tupleTransformed.set(x, y, z);
   }

   /**
    * Assert on a component basis is the {@code tuple} is equal to (0, 0, 0) given the tolerance
    * {@code epsilon}.
    *
    * @param tuple   the query. Not modified.
    * @param epsilon the tolerance.
    * @return {@code true} if the tuple's component are all equal to zero, {@code false} otherwise.
    */
   public static boolean isZero(Tuple3DReadOnly tuple, double epsilon)
   {
      if (!epsilonEquals(tuple.getX(), 0.0, epsilon))
         return false;
      if (!epsilonEquals(tuple.getY(), 0.0, epsilon))
         return false;
      if (!epsilonEquals(tuple.getZ(), 0.0, epsilon))
         return false;
      return true;
   }

   /**
    * Assert on a component basis is the {@code tuple} is equal to (0, 0) given the tolerance
    * {@code epsilon}.
    *
    * @param tuple   the query. Not modified.
    * @param epsilon the tolerance.
    * @return {@code true} if the tuple's component are all equal to zero, {@code false} otherwise.
    */
   public static boolean isZero(Tuple2DReadOnly tuple, double epsilon)
   {
      if (!epsilonEquals(tuple.getX(), 0.0, epsilon))
         return false;
      if (!epsilonEquals(tuple.getY(), 0.0, epsilon))
         return false;
      return true;
   }
}
