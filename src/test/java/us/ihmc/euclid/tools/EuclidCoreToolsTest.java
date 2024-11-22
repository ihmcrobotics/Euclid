package us.ihmc.euclid.tools;

import org.junit.jupiter.api.Test;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.orientation.Orientation2D;
import us.ihmc.euclid.orientation.interfaces.Orientation2DReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DBasics;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.UnitVector3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.Vector4D;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.Tuple4DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.Vector4DBasics;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;
import static us.ihmc.euclid.EuclidTestConstants.ITERATIONS;
import static us.ihmc.euclid.tools.EuclidCoreTools.EPS_NORM_FAST_SQRT;
import static us.ihmc.euclid.tools.EuclidCoreTools.wrap;

public class EuclidCoreToolsTest
{
   @Test
   public void testConstants()
   {
      Point2D expectedOrigin2D = new Point2D();
      assertEquals(expectedOrigin2D, EuclidCoreTools.origin2D);
      assertEquals(expectedOrigin2D.hashCode(), EuclidCoreTools.origin2D.hashCode());
      assertEquals(expectedOrigin2D.toString(), EuclidCoreTools.origin2D.toString());

      Point3D expectedOrigin3D = new Point3D();
      assertEquals(expectedOrigin3D, EuclidCoreTools.origin3D);
      assertEquals(expectedOrigin3D.hashCode(), EuclidCoreTools.origin3D.hashCode());
      assertEquals(expectedOrigin3D.toString(), EuclidCoreTools.origin3D.toString());

      Vector2D expectedZeroVector2D = new Vector2D();
      assertEquals(expectedZeroVector2D, EuclidCoreTools.zeroVector2D);
      assertEquals(expectedZeroVector2D.hashCode(), EuclidCoreTools.zeroVector2D.hashCode());
      assertEquals(expectedZeroVector2D.toString(), EuclidCoreTools.zeroVector2D.toString());

      Vector3D expectedZeroVector3D = new Vector3D();
      assertEquals(expectedZeroVector3D, EuclidCoreTools.zeroVector3D);
      assertEquals(expectedZeroVector3D.hashCode(), EuclidCoreTools.zeroVector3D.hashCode());
      assertEquals(expectedZeroVector3D.toString(), EuclidCoreTools.zeroVector3D.toString());

      Vector4D expectedQuaternion = new Vector4D();
      expectedQuaternion.setElement(3, 1);
      assertEquals(expectedQuaternion, EuclidCoreTools.neutralQuaternion);
      assertEquals(expectedQuaternion.hashCode(), EuclidCoreTools.neutralQuaternion.hashCode());
      assertEquals(expectedQuaternion.toString(), EuclidCoreTools.neutralQuaternion.toString());

      Matrix3D expectedZeroMatrix3D = new Matrix3D();
      expectedZeroMatrix3D.setToZero();
      assertEquals(expectedZeroMatrix3D, EuclidCoreTools.zeroMatrix3D);
      assertEquals(expectedZeroMatrix3D.toString(), EuclidCoreTools.zeroMatrix3D.toString());
      assertEquals(expectedZeroMatrix3D.hashCode(), EuclidCoreTools.zeroMatrix3D.hashCode());

      Matrix3D expectedIdentity3D = new Matrix3D();
      expectedIdentity3D.setIdentity();
      assertEquals(expectedIdentity3D, EuclidCoreTools.identityMatrix3D);
      assertEquals(expectedIdentity3D.toString(), EuclidCoreTools.identityMatrix3D.toString());
      assertEquals(expectedIdentity3D.hashCode(), EuclidCoreTools.identityMatrix3D.hashCode());
   }

   @Test
   public void testFastSquareRoot()
   {
      Random random = new Random(2342L);

      for (int i = 0; i < 1000; i++)
      {
         double squaredValue = 1.5 * random.nextDouble();
         squaredValue = Math.min(squaredValue, 1.0 - EPS_NORM_FAST_SQRT);

         double actualValue = EuclidCoreTools.fastSquareRoot(squaredValue);
         double expectedValue = EuclidCoreTools.squareRoot(squaredValue);
         assertEquals(expectedValue, actualValue, Double.MIN_VALUE);
      }

      for (int i = 0; i < 1000; i++)
      {
         double squaredValue = 0.8 + 1.5 * random.nextDouble();
         squaredValue = Math.max(squaredValue, 1.0 + EPS_NORM_FAST_SQRT);

         double actualValue = EuclidCoreTools.fastSquareRoot(squaredValue);
         double expectedValue = EuclidCoreTools.squareRoot(squaredValue);
         assertEquals(expectedValue, actualValue, Double.MIN_VALUE);
      }

      for (int i = 0; i < 1000; i++)
      {
         double squaredValue = EuclidCoreRandomTools.nextDouble(random, 1.0 - EPS_NORM_FAST_SQRT, 1.0 + EPS_NORM_FAST_SQRT);
         squaredValue = Math.max(squaredValue, 1.0 + EPS_NORM_FAST_SQRT);

         double actualValue = EuclidCoreTools.fastSquareRoot(squaredValue);
         double expectedValue = EuclidCoreTools.squareRoot(squaredValue);
         assertEquals(expectedValue, actualValue, Double.MIN_VALUE);
      }
   }

   @Test
   public void testContainsNaNWith2Elements()
   {
      assertFalse(EuclidCoreTools.containsNaN(0.0, 0.0));
      assertTrue(EuclidCoreTools.containsNaN(Double.NaN, 0.0));
      assertTrue(EuclidCoreTools.containsNaN(0.0, Double.NaN));
   }

   @Test
   public void testContainsNaNWith3Elements()
   {
      assertFalse(EuclidCoreTools.containsNaN(0.0, 0.0, 0.0));
      assertTrue(EuclidCoreTools.containsNaN(Double.NaN, 0.0, 0.0));
      assertTrue(EuclidCoreTools.containsNaN(0.0, Double.NaN, 0.0));
      assertTrue(EuclidCoreTools.containsNaN(0.0, 0.0, Double.NaN));
   }

   @Test
   public void testContainsNaNWith4Elements()
   {
      assertFalse(EuclidCoreTools.containsNaN(0.0, 0.0, 0.0, 0.0));
      assertTrue(EuclidCoreTools.containsNaN(Double.NaN, 0.0, 0.0, 0.0));
      assertTrue(EuclidCoreTools.containsNaN(0.0, Double.NaN, 0.0, 0.0));
      assertTrue(EuclidCoreTools.containsNaN(0.0, 0.0, Double.NaN, 0.0));
      assertTrue(EuclidCoreTools.containsNaN(0.0, 0.0, 0.0, Double.NaN));
   }

   @Test
   public void testContainsNaNWith9Elements()
   {
      assertFalse(EuclidCoreTools.containsNaN(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
      assertTrue(EuclidCoreTools.containsNaN(Double.NaN, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
      assertTrue(EuclidCoreTools.containsNaN(0.0, Double.NaN, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
      assertTrue(EuclidCoreTools.containsNaN(0.0, 0.0, Double.NaN, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
      assertTrue(EuclidCoreTools.containsNaN(0.0, 0.0, 0.0, Double.NaN, 0.0, 0.0, 0.0, 0.0, 0.0));
      assertTrue(EuclidCoreTools.containsNaN(0.0, 0.0, 0.0, 0.0, Double.NaN, 0.0, 0.0, 0.0, 0.0));
      assertTrue(EuclidCoreTools.containsNaN(0.0, 0.0, 0.0, 0.0, 0.0, Double.NaN, 0.0, 0.0, 0.0));
      assertTrue(EuclidCoreTools.containsNaN(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, Double.NaN, 0.0, 0.0));
      assertTrue(EuclidCoreTools.containsNaN(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, Double.NaN, 0.0));
      assertTrue(EuclidCoreTools.containsNaN(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, Double.NaN));
   }

   @Test
   public void testContainsNaNWithArray()
   {
      assertFalse(EuclidCoreTools.containsNaN(new double[0]));
      assertFalse(EuclidCoreTools.containsNaN(new double[] {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}));
      assertTrue(EuclidCoreTools.containsNaN(new double[] {Double.NaN, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}));
      assertTrue(EuclidCoreTools.containsNaN(new double[] {0.0, Double.NaN, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}));
      assertTrue(EuclidCoreTools.containsNaN(new double[] {0.0, 0.0, Double.NaN, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}));
      assertTrue(EuclidCoreTools.containsNaN(new double[] {0.0, 0.0, 0.0, Double.NaN, 0.0, 0.0, 0.0, 0.0, 0.0}));
      assertTrue(EuclidCoreTools.containsNaN(new double[] {0.0, 0.0, 0.0, 0.0, Double.NaN, 0.0, 0.0, 0.0, 0.0}));
      assertTrue(EuclidCoreTools.containsNaN(new double[] {0.0, 0.0, 0.0, 0.0, 0.0, Double.NaN, 0.0, 0.0, 0.0}));
      assertTrue(EuclidCoreTools.containsNaN(new double[] {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, Double.NaN, 0.0, 0.0}));
      assertTrue(EuclidCoreTools.containsNaN(new double[] {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, Double.NaN, 0.0}));
      assertTrue(EuclidCoreTools.containsNaN(new double[] {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, Double.NaN}));
   }

   @Test
   public void testNormSquaredWith2Elements()
   {
      assertEquals(1.0, EuclidCoreTools.normSquared(1.0, 0.0));
      assertEquals(1.0, EuclidCoreTools.normSquared(0.0, 1.0));
      assertEquals(0.0, EuclidCoreTools.normSquared(0.0, 0.0));

      assertEquals(1.0, EuclidCoreTools.normSquared(-1.0, 0.0));
      assertEquals(1.0, EuclidCoreTools.normSquared(0.0, -1.0));

      assertEquals(4.0, EuclidCoreTools.normSquared(2.0, 0.0));
      assertEquals(4.0, EuclidCoreTools.normSquared(0.0, 2.0));

      assertEquals(4.0, EuclidCoreTools.normSquared(-2.0, 0.0));
      assertEquals(4.0, EuclidCoreTools.normSquared(0.0, -2.0));
   }

   @Test
   public void testNormSquaredWith3Elements()
   {
      assertEquals(1.0, EuclidCoreTools.normSquared(1.0, 0.0, 0.0));
      assertEquals(1.0, EuclidCoreTools.normSquared(0.0, 1.0, 0.0));
      assertEquals(1.0, EuclidCoreTools.normSquared(0.0, 0.0, 1.0));
      assertEquals(0.0, EuclidCoreTools.normSquared(0.0, 0.0, 0.0));

      assertEquals(1.0, EuclidCoreTools.normSquared(-1.0, 0.0, 0.0));
      assertEquals(1.0, EuclidCoreTools.normSquared(0.0, -1.0, 0.0));
      assertEquals(1.0, EuclidCoreTools.normSquared(0.0, 0.0, -1.0));

      assertEquals(4.0, EuclidCoreTools.normSquared(2.0, 0.0, 0.0));
      assertEquals(4.0, EuclidCoreTools.normSquared(0.0, 2.0, 0.0));
      assertEquals(4.0, EuclidCoreTools.normSquared(0.0, 0.0, 2.0));

      assertEquals(4.0, EuclidCoreTools.normSquared(-2.0, 0.0, 0.0));
      assertEquals(4.0, EuclidCoreTools.normSquared(0.0, -2.0, 0.0));
      assertEquals(4.0, EuclidCoreTools.normSquared(0.0, 0.0, -2.0));
   }

   @Test
   public void testNormSquaredWith4Elements()
   {
      assertEquals(1.0, EuclidCoreTools.normSquared(1.0, 0.0, 0.0, 0.0));
      assertEquals(1.0, EuclidCoreTools.normSquared(0.0, 1.0, 0.0, 0.0));
      assertEquals(1.0, EuclidCoreTools.normSquared(0.0, 0.0, 1.0, 0.0));
      assertEquals(1.0, EuclidCoreTools.normSquared(0.0, 0.0, 0.0, 1.0));
      assertEquals(0.0, EuclidCoreTools.normSquared(0.0, 0.0, 0.0, 0.0));

      assertEquals(1.0, EuclidCoreTools.normSquared(-1.0, 0.0, 0.0, 0.0));
      assertEquals(1.0, EuclidCoreTools.normSquared(0.0, -1.0, 0.0, 0.0));
      assertEquals(1.0, EuclidCoreTools.normSquared(0.0, 0.0, -1.0, 0.0));
      assertEquals(1.0, EuclidCoreTools.normSquared(0.0, 0.0, 0.0, -1.0));

      assertEquals(4.0, EuclidCoreTools.normSquared(2.0, 0.0, 0.0, 0.0));
      assertEquals(4.0, EuclidCoreTools.normSquared(0.0, 2.0, 0.0, 0.0));
      assertEquals(4.0, EuclidCoreTools.normSquared(0.0, 0.0, 2.0, 0.0));
      assertEquals(4.0, EuclidCoreTools.normSquared(0.0, 0.0, 0.0, 2.0));

      assertEquals(4.0, EuclidCoreTools.normSquared(-2.0, 0.0, 0.0, 0.0));
      assertEquals(4.0, EuclidCoreTools.normSquared(0.0, -2.0, 0.0, 0.0));
      assertEquals(4.0, EuclidCoreTools.normSquared(0.0, 0.0, -2.0, 0.0));
      assertEquals(4.0, EuclidCoreTools.normSquared(0.0, 0.0, 0.0, -2.0));
   }

   @Test
   public void testNormWith2Elements()
   {
      assertEquals(1.0, EuclidCoreTools.norm(1.0, 0.0));
      assertEquals(1.0, EuclidCoreTools.norm(0.0, 1.0));
      assertEquals(0.0, EuclidCoreTools.norm(0.0, 0.0));
      assertEquals(1.0, EuclidCoreTools.norm(-1.0, 0.0));
      assertEquals(1.0, EuclidCoreTools.norm(0.0, -1.0));
      assertEquals(2.0, EuclidCoreTools.norm(2.0, 0.0));
      assertEquals(2.0, EuclidCoreTools.norm(0.0, 2.0));
      assertEquals(2.0, EuclidCoreTools.norm(-2.0, 0.0));
      assertEquals(2.0, EuclidCoreTools.norm(0.0, -2.0));

      assertEquals(1.0, EuclidCoreTools.fastNorm(1.0, 0.0));
      assertEquals(1.0, EuclidCoreTools.fastNorm(0.0, 1.0));
      assertEquals(0.0, EuclidCoreTools.fastNorm(0.0, 0.0));
      assertEquals(1.0, EuclidCoreTools.fastNorm(-1.0, 0.0));
      assertEquals(1.0, EuclidCoreTools.fastNorm(0.0, -1.0));
      assertEquals(2.0, EuclidCoreTools.fastNorm(2.0, 0.0));
      assertEquals(2.0, EuclidCoreTools.fastNorm(0.0, 2.0));
      assertEquals(2.0, EuclidCoreTools.fastNorm(-2.0, 0.0));
      assertEquals(2.0, EuclidCoreTools.fastNorm(0.0, -2.0));
   }

   @Test
   public void testNormWith3Elements()
   {
      assertEquals(1.0, EuclidCoreTools.norm(1.0, 0.0, 0.0));
      assertEquals(1.0, EuclidCoreTools.norm(0.0, 1.0, 0.0));
      assertEquals(1.0, EuclidCoreTools.norm(0.0, 0.0, 1.0));
      assertEquals(0.0, EuclidCoreTools.norm(0.0, 0.0, 0.0));
      assertEquals(1.0, EuclidCoreTools.norm(-1.0, 0.0, 0.0));
      assertEquals(1.0, EuclidCoreTools.norm(0.0, -1.0, 0.0));
      assertEquals(1.0, EuclidCoreTools.norm(0.0, 0.0, -1.0));
      assertEquals(2.0, EuclidCoreTools.norm(2.0, 0.0, 0.0));
      assertEquals(2.0, EuclidCoreTools.norm(0.0, 2.0, 0.0));
      assertEquals(2.0, EuclidCoreTools.norm(0.0, 0.0, 2.0));
      assertEquals(2.0, EuclidCoreTools.norm(-2.0, 0.0, 0.0));
      assertEquals(2.0, EuclidCoreTools.norm(0.0, -2.0, 0.0));
      assertEquals(2.0, EuclidCoreTools.norm(0.0, 0.0, -2.0));

      assertEquals(1.0, EuclidCoreTools.fastNorm(1.0, 0.0, 0.0));
      assertEquals(1.0, EuclidCoreTools.fastNorm(0.0, 1.0, 0.0));
      assertEquals(1.0, EuclidCoreTools.fastNorm(0.0, 0.0, 1.0));
      assertEquals(0.0, EuclidCoreTools.fastNorm(0.0, 0.0, 0.0));
      assertEquals(1.0, EuclidCoreTools.fastNorm(-1.0, 0.0, 0.0));
      assertEquals(1.0, EuclidCoreTools.fastNorm(0.0, -1.0, 0.0));
      assertEquals(1.0, EuclidCoreTools.fastNorm(0.0, 0.0, -1.0));
      assertEquals(2.0, EuclidCoreTools.fastNorm(2.0, 0.0, 0.0));
      assertEquals(2.0, EuclidCoreTools.fastNorm(0.0, 2.0, 0.0));
      assertEquals(2.0, EuclidCoreTools.fastNorm(0.0, 0.0, 2.0));
      assertEquals(2.0, EuclidCoreTools.fastNorm(-2.0, 0.0, 0.0));
      assertEquals(2.0, EuclidCoreTools.fastNorm(0.0, -2.0, 0.0));
      assertEquals(2.0, EuclidCoreTools.fastNorm(0.0, 0.0, -2.0));
   }

   @Test
   public void testNormWith4Elements()
   {
      assertEquals(1.0, EuclidCoreTools.norm(1.0, 0.0, 0.0, 0.0));
      assertEquals(1.0, EuclidCoreTools.norm(0.0, 1.0, 0.0, 0.0));
      assertEquals(1.0, EuclidCoreTools.norm(0.0, 0.0, 1.0, 0.0));
      assertEquals(1.0, EuclidCoreTools.norm(0.0, 0.0, 0.0, 1.0));
      assertEquals(0.0, EuclidCoreTools.norm(0.0, 0.0, 0.0, 0.0));
      assertEquals(1.0, EuclidCoreTools.norm(-1.0, 0.0, 0.0, 0.0));
      assertEquals(1.0, EuclidCoreTools.norm(0.0, -1.0, 0.0, 0.0));
      assertEquals(1.0, EuclidCoreTools.norm(0.0, 0.0, -1.0, 0.0));
      assertEquals(1.0, EuclidCoreTools.norm(0.0, 0.0, 0.0, -1.0));
      assertEquals(2.0, EuclidCoreTools.norm(2.0, 0.0, 0.0, 0.0));
      assertEquals(2.0, EuclidCoreTools.norm(0.0, 2.0, 0.0, 0.0));
      assertEquals(2.0, EuclidCoreTools.norm(0.0, 0.0, 2.0, 0.0));
      assertEquals(2.0, EuclidCoreTools.norm(0.0, 0.0, 0.0, 2.0));
      assertEquals(2.0, EuclidCoreTools.norm(-2.0, 0.0, 0.0, 0.0));
      assertEquals(2.0, EuclidCoreTools.norm(0.0, -2.0, 0.0, 0.0));
      assertEquals(2.0, EuclidCoreTools.norm(0.0, 0.0, -2.0, 0.0));
      assertEquals(2.0, EuclidCoreTools.norm(0.0, 0.0, 0.0, -2.0));

      assertEquals(1.0, EuclidCoreTools.fastNorm(1.0, 0.0, 0.0, 0.0));
      assertEquals(1.0, EuclidCoreTools.fastNorm(0.0, 1.0, 0.0, 0.0));
      assertEquals(1.0, EuclidCoreTools.fastNorm(0.0, 0.0, 1.0, 0.0));
      assertEquals(1.0, EuclidCoreTools.fastNorm(0.0, 0.0, 0.0, 1.0));
      assertEquals(0.0, EuclidCoreTools.fastNorm(0.0, 0.0, 0.0, 0.0));
      assertEquals(1.0, EuclidCoreTools.fastNorm(-1.0, 0.0, 0.0, 0.0));
      assertEquals(1.0, EuclidCoreTools.fastNorm(0.0, -1.0, 0.0, 0.0));
      assertEquals(1.0, EuclidCoreTools.fastNorm(0.0, 0.0, -1.0, 0.0));
      assertEquals(1.0, EuclidCoreTools.fastNorm(0.0, 0.0, 0.0, -1.0));
      assertEquals(2.0, EuclidCoreTools.fastNorm(2.0, 0.0, 0.0, 0.0));
      assertEquals(2.0, EuclidCoreTools.fastNorm(0.0, 2.0, 0.0, 0.0));
      assertEquals(2.0, EuclidCoreTools.fastNorm(0.0, 0.0, 2.0, 0.0));
      assertEquals(2.0, EuclidCoreTools.fastNorm(0.0, 0.0, 0.0, 2.0));
      assertEquals(2.0, EuclidCoreTools.fastNorm(-2.0, 0.0, 0.0, 0.0));
      assertEquals(2.0, EuclidCoreTools.fastNorm(0.0, -2.0, 0.0, 0.0));
      assertEquals(2.0, EuclidCoreTools.fastNorm(0.0, 0.0, -2.0, 0.0));
      assertEquals(2.0, EuclidCoreTools.fastNorm(0.0, 0.0, 0.0, -2.0));
   }

   @Test
   public void testTrimAngleMinusPiToPi()
   {
      Random random = new Random(2323L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         double startOfRange = -Math.PI;
         double endOfRange = Math.PI;
         double expectedAngle = EuclidCoreRandomTools.nextDouble(random, startOfRange, endOfRange);
         double angleToShift = expectedAngle + (random.nextInt(21) - 10) * 2.0 * Math.PI;
         double actualAngle = EuclidCoreTools.trimAngleMinusPiToPi(angleToShift);
         assertEquals(expectedAngle, actualAngle, 1.0e-12, "iteration: " + i);

         expectedAngle = startOfRange;
         angleToShift = expectedAngle + (random.nextInt(21) - 10) * 2.0 * Math.PI;
         actualAngle = EuclidCoreTools.trimAngleMinusPiToPi(angleToShift);
         assertEquals(expectedAngle, actualAngle, 1.0e-12);

         expectedAngle = endOfRange - 1.0e-9;
         angleToShift = expectedAngle + (random.nextInt(21) - 10) * 2.0 * Math.PI;
         actualAngle = EuclidCoreTools.trimAngleMinusPiToPi(angleToShift);
         assertEquals(expectedAngle, actualAngle, 1.0e-12);
      }
   }

   @Test
   public void testAngleDifferenceMinusPiToPi()
   {
      Random random = new Random(2323L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         double startOfRange = -Math.PI;
         double endOfRange = Math.PI;
         double expectedDifference = EuclidCoreRandomTools.nextDouble(random, startOfRange, endOfRange);
         double untrimmedDifference = expectedDifference + (random.nextInt(21) - 10) * 2.0 * Math.PI;
         double angleA = EuclidCoreRandomTools.nextDouble(random, 4.0 * Math.PI);
         double angleB = angleA - untrimmedDifference;
         double actualDifference = EuclidCoreTools.angleDifferenceMinusPiToPi(angleA, angleB);
         assertEquals(expectedDifference, actualDifference, 1.0e-12, "iteration: " + i);
      }
   }

   @Test
   public void testShiftAngleInRange()
   {
      Random random = new Random(23423L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         double startOfRange = EuclidCoreRandomTools.nextDouble(random, 2.0 * Math.PI);
         double endOfRange = startOfRange + 2.0 * Math.PI;
         double expectedAngle = EuclidCoreRandomTools.nextDouble(random, startOfRange, endOfRange);
         double angleToShift = expectedAngle + (random.nextInt(21) - 10) * 2.0 * Math.PI;
         double actualAngle = EuclidCoreTools.shiftAngleInRange(angleToShift, startOfRange);
         assertEquals(expectedAngle, actualAngle, 1.0e-12, "iteration: " + i);

         expectedAngle = startOfRange;
         angleToShift = expectedAngle + (random.nextInt(21) - 10) * 2.0 * Math.PI;
         actualAngle = EuclidCoreTools.shiftAngleInRange(angleToShift, startOfRange);
         assertEquals(expectedAngle, actualAngle, 1.0e-12);

         expectedAngle = endOfRange - 1.0e-9;
         angleToShift = expectedAngle + (random.nextInt(21) - 10) * 2.0 * Math.PI;
         actualAngle = EuclidCoreTools.shiftAngleInRange(angleToShift, startOfRange);
         assertEquals(expectedAngle, actualAngle, 1.0e-12);
      }
   }

   @Test
   public void testAngleGeometricallyEquals()
   {
      Random random = new Random(35635);

      for (int i = 0; i < ITERATIONS; i++)
      { // Test with angle that are close-ish to each other
         double epsilon = EuclidCoreRandomTools.nextDouble(random, 1.0e-12, 0.5);
         double angleA = EuclidCoreRandomTools.nextDouble(random, 10.0 * Math.PI);
         double angleNotEqual = angleA + (random.nextBoolean() ? -1.01 : 1.01) * epsilon;
         double angleEqual = angleA + (random.nextBoolean() ? -0.99 : 0.99) * epsilon;

         assertFalse(EuclidCoreTools.angleGeometricallyEquals(angleA, angleNotEqual, epsilon));
         assertTrue(EuclidCoreTools.angleGeometricallyEquals(angleA, angleEqual, epsilon));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test with angle that are far-ish to each other
         double epsilon = EuclidCoreRandomTools.nextDouble(random, 1.0e-12, 0.5);
         double angleA = EuclidCoreRandomTools.nextDouble(random, 10.0 * Math.PI);
         double twoPIMutiple = random.nextInt(15) * 2.0 * Math.PI;
         if (random.nextBoolean())
            twoPIMutiple = -twoPIMutiple;
         double angleNotEqual = angleA + (random.nextBoolean() ? -1.01 : 1.01) * epsilon + twoPIMutiple;
         double angleEqual = angleA + (random.nextBoolean() ? -0.99 : 0.99) * epsilon + twoPIMutiple;

         assertFalse(EuclidCoreTools.angleGeometricallyEquals(angleA, angleNotEqual, epsilon));
         assertTrue(EuclidCoreTools.angleGeometricallyEquals(angleA, angleEqual, epsilon));
      }
   }

   @Test
   public void testIsAngleZero()
   {
      Random random = new Random(35635);

      for (int i = 0; i < ITERATIONS; i++)
      {
         double epsilon = EuclidCoreRandomTools.nextDouble(random, 1.0e-12, 0.5);
         double twoPIMutiple = random.nextInt(15) * 2.0 * Math.PI;
         double zeroAngle = 0.99 * EuclidCoreRandomTools.nextDouble(random, epsilon);
         double nonZeroAngle = EuclidCoreRandomTools.nextDouble(random, 1.01 * epsilon, Math.PI);
         if (random.nextBoolean())
            nonZeroAngle = -nonZeroAngle;

         assertTrue(EuclidCoreTools.isAngleZero(zeroAngle, epsilon));
         assertTrue(EuclidCoreTools.isAngleZero(zeroAngle + twoPIMutiple, epsilon));

         assertFalse(EuclidCoreTools.isAngleZero(nonZeroAngle, epsilon));
         assertFalse(EuclidCoreTools.isAngleZero(nonZeroAngle + twoPIMutiple, epsilon));
      }
   }

   @Test
   public void testMax()
   {
      Random random = new Random(45645L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         double a = EuclidCoreRandomTools.nextDouble(random, 10.0);
         double b = EuclidCoreRandomTools.nextDouble(random, 10.0);
         double c = EuclidCoreRandomTools.nextDouble(random, 10.0);
         double expected = Math.max(a, Math.max(b, c));
         double actual = EuclidCoreTools.max(a, b, c);
         assertEquals(expected, actual);
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         double a = EuclidCoreRandomTools.nextDouble(random, 10.0);
         double b = EuclidCoreRandomTools.nextDouble(random, 10.0);
         double c = EuclidCoreRandomTools.nextDouble(random, 10.0);
         double d = EuclidCoreRandomTools.nextDouble(random, 10.0);
         double expected = Math.max(a, Math.max(b, Math.max(c, d)));
         double actual = EuclidCoreTools.max(a, b, c, d);
         assertEquals(expected, actual);
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         int a = random.nextInt();
         int b = random.nextInt();
         int c = random.nextInt();
         int expected = Math.max(a, Math.max(b, c));
         int actual = EuclidCoreTools.max(a, b, c);
         assertEquals(expected, actual);
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         int a = random.nextInt();
         int b = random.nextInt();
         int c = random.nextInt();
         int d = random.nextInt();
         int expected = Math.max(a, Math.max(b, Math.max(c, d)));
         int actual = EuclidCoreTools.max(a, b, c, d);
         assertEquals(expected, actual);
      }
   }

   @Test
   public void testMin()
   {
      Random random = new Random(45645L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         double a = EuclidCoreRandomTools.nextDouble(random, 10.0);
         double b = EuclidCoreRandomTools.nextDouble(random, 10.0);
         double c = EuclidCoreRandomTools.nextDouble(random, 10.0);
         double expected = Math.min(a, Math.min(b, c));
         double actual = EuclidCoreTools.min(a, b, c);
         assertEquals(expected, actual);
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         double a = EuclidCoreRandomTools.nextDouble(random, 10.0);
         double b = EuclidCoreRandomTools.nextDouble(random, 10.0);
         double c = EuclidCoreRandomTools.nextDouble(random, 10.0);
         double d = EuclidCoreRandomTools.nextDouble(random, 10.0);
         double expected = Math.min(a, Math.min(b, Math.min(c, d)));
         double actual = EuclidCoreTools.min(a, b, c, d);
         assertEquals(expected, actual);
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         int a = random.nextInt();
         int b = random.nextInt();
         int c = random.nextInt();
         int expected = Math.min(a, Math.min(b, c));
         int actual = EuclidCoreTools.min(a, b, c);
         assertEquals(expected, actual);
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         int a = random.nextInt();
         int b = random.nextInt();
         int c = random.nextInt();
         int d = random.nextInt();
         int expected = Math.min(a, Math.min(b, Math.min(c, d)));
         int actual = EuclidCoreTools.min(a, b, c, d);
         assertEquals(expected, actual);
      }
   }

   @Test
   public void testMed()
   {
      Random random = new Random(45645L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         double a = EuclidCoreRandomTools.nextDouble(random, 10.0);
         double b = EuclidCoreRandomTools.nextDouble(random, 10.0);
         double c = EuclidCoreRandomTools.nextDouble(random, 10.0);
         double[] sorted = {a, b, c};
         Arrays.sort(sorted);
         double expected = sorted[1];
         double actual = EuclidCoreTools.med(a, b, c);
         if (expected != actual)
            EuclidCoreTools.med(a, b, c);
         assertEquals(expected, actual, "a = " + a + ", b = " + b + ", c = " + c);
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         int a = random.nextInt();
         int b = random.nextInt();
         int c = random.nextInt();
         int[] sorted = {a, b, c};
         Arrays.sort(sorted);
         int expected = sorted[1];
         int actual = EuclidCoreTools.med(a, b, c);
         if (expected != actual)
            EuclidCoreTools.med(a, b, c);
         assertEquals(expected, actual, "a = " + a + ", b = " + b + ", c = " + c);
      }
   }

   @Test
   public void testEpsilonEquals()
   {
      Random random = new Random(34235);

      for (int i = 0; i < ITERATIONS; i++)
      { // Tests with epsilon == 0
         double epsilon = 0;
         double value = EuclidCoreRandomTools.nextDouble(random, -1.0e32, 1.0e32);
         assertTrue(EuclidCoreTools.epsilonEquals(value, value, epsilon));
         assertFalse(EuclidCoreTools.epsilonEquals(value, Math.nextUp(value), epsilon));
         assertFalse(EuclidCoreTools.epsilonEquals(value, Math.nextDown(value), epsilon));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Typical use test
         double epsilon = random.nextDouble();
         double expectedValue = EuclidCoreRandomTools.nextDouble(random, -1.0e32, 1.0e32);
         double actualValue = expectedValue + EuclidCoreRandomTools.nextDouble(random, 0, epsilon);
         assertTrue(EuclidCoreTools.epsilonEquals(expectedValue, actualValue, epsilon));
         actualValue = expectedValue - EuclidCoreRandomTools.nextDouble(random, 0, epsilon);
         assertTrue(EuclidCoreTools.epsilonEquals(expectedValue, actualValue, epsilon));

         actualValue = expectedValue + EuclidCoreRandomTools.nextDouble(random, epsilon, 10.0 * epsilon);
         assertTrue(EuclidCoreTools.epsilonEquals(expectedValue, actualValue, epsilon));
         actualValue = expectedValue - EuclidCoreRandomTools.nextDouble(random, epsilon, 10.0 * epsilon);
         assertTrue(EuclidCoreTools.epsilonEquals(expectedValue, actualValue, epsilon));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test with infinity and NaN
         double epsilon = random.nextDouble();

         assertFalse(EuclidCoreTools.epsilonEquals(Double.NaN, Double.NaN, epsilon));
         assertFalse(EuclidCoreTools.epsilonEquals(Double.NaN, EuclidCoreRandomTools.nextDouble(random, -1.0e32, 1.0e32), epsilon));
         assertFalse(EuclidCoreTools.epsilonEquals(EuclidCoreRandomTools.nextDouble(random, -1.0e32, 1.0e32), Double.NaN, epsilon));

         assertTrue(EuclidCoreTools.epsilonEquals(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, epsilon));
         assertFalse(EuclidCoreTools.epsilonEquals(Double.POSITIVE_INFINITY, EuclidCoreRandomTools.nextDouble(random, -1.0e32, 1.0e32), epsilon));
         assertFalse(EuclidCoreTools.epsilonEquals(EuclidCoreRandomTools.nextDouble(random, -1.0e32, 1.0e32), Double.POSITIVE_INFINITY, epsilon));

         assertTrue(EuclidCoreTools.epsilonEquals(Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY, epsilon));
         assertFalse(EuclidCoreTools.epsilonEquals(Double.NEGATIVE_INFINITY, EuclidCoreRandomTools.nextDouble(random, -1.0e32, 1.0e32), epsilon));
         assertFalse(EuclidCoreTools.epsilonEquals(EuclidCoreRandomTools.nextDouble(random, -1.0e32, 1.0e32), Double.NEGATIVE_INFINITY, epsilon));
      }
   }

   @Test
   public void testInterpolate()
   {
      Random random = new Random(3665L);

      for (int i = 0; i < ITERATIONS; i++)
      { // Test Tuple.interpolate(double a, double b, double alpha)
         double a = random.nextDouble();
         double b = random.nextDouble();
         double alpha = random.nextDouble();

         double result = EuclidCoreTools.interpolate(a, b, alpha);
         double expected = a + alpha * (b - a);
         assertEquals(result, expected, 1.0e-10);

         alpha = 0.5;
         result = EuclidCoreTools.interpolate(a, b, alpha);
         assertEquals(result, 0.5 * a + 0.5 * b);
         alpha = 0.0;
         result = EuclidCoreTools.interpolate(a, b, alpha);
         assertEquals(result, a);
         alpha = 1.0;
         result = EuclidCoreTools.interpolate(a, b, alpha);
         assertEquals(result, b);

         for (alpha = -2.0; alpha <= 2.0; alpha += 0.1)
         {
            result = EuclidCoreTools.interpolate(a, b, alpha);
            assertEquals(result, a + alpha * (b - a), 1.0e-10);
         }
      }
   }

   @Test
   public void testClamp()
   {
      Random random = new Random(3453);

      { // Test clamp(double value, double minMax)

         for (int i = 0; i < ITERATIONS; i++)
         {
            double minMax = EuclidCoreRandomTools.nextDouble(random, 0.0, 100.0);
            double valueInside = EuclidCoreRandomTools.nextDouble(random, minMax);
            double valueUnder = EuclidCoreRandomTools.nextDouble(random, -100.0, 0.0) - minMax;
            double valueOver = EuclidCoreRandomTools.nextDouble(random, 0.0, 100.0) + minMax;

            assertEquals(valueInside, EuclidCoreTools.clamp(valueInside, minMax));
            assertEquals(-minMax, EuclidCoreTools.clamp(valueUnder, minMax));
            assertEquals(minMax, EuclidCoreTools.clamp(valueOver, minMax));
         }

         EuclidCoreTestTools.assertExceptionIsThrown(() -> EuclidCoreTools.clamp(0.0, -EuclidCoreTools.CLAMP_EPS - 1.0e-12), RuntimeException.class);
      }

      { // Test clamp(double value, double minMax)

         for (int i = 0; i < ITERATIONS; i++)
         {
            double min = EuclidCoreRandomTools.nextDouble(random, -100.0, 100.0);
            double max = min + EuclidCoreRandomTools.nextDouble(random, 0.0, 100.0);
            double valueInside = EuclidCoreRandomTools.nextDouble(random, min, max);
            double valueUnder = min - EuclidCoreRandomTools.nextDouble(random, 0.0, 100.0);
            double valueOver = max + EuclidCoreRandomTools.nextDouble(random, 0.0, 100.0);

            assertEquals(valueInside, EuclidCoreTools.clamp(valueInside, min, max));
            assertEquals(min, EuclidCoreTools.clamp(valueUnder, min, max));
            assertEquals(max, EuclidCoreTools.clamp(valueOver, min, max));
         }

         double min = EuclidCoreRandomTools.nextDouble(random, -100.0, 100.0);
         EuclidCoreTestTools.assertExceptionIsThrown(() -> EuclidCoreTools.clamp(0.0, min, min - EuclidCoreTools.CLAMP_EPS - 1.0e-12), RuntimeException.class);
      }
   }

   @Test
   public void testFastAcos()
   {
      double epsilon = 1.0e-14;
      double maxError = 0.0;
      double maxErrorX = 0.0;
      double error;
      double expected;
      double actual;

      for (double x = -1.0; x < 1.0; x += 1.0e-6)
      {
         expected = Math.acos(x);
         actual = EuclidCoreTools.fastAcos(x);
         assertEquals(expected, actual, epsilon, "x=" + x);

         error = Math.abs(actual - expected);
         if (error > maxError)
         {
            maxError = error;
            maxErrorX = x;
         }
      }

      double x = -1.0;
      expected = Math.acos(x);
      actual = EuclidCoreTools.fastAcos(x);
      assertEquals(expected, actual, epsilon, "x=" + x);
      error = Math.abs(actual - expected);
      if (error > maxError)
      {
         maxError = error;
         maxErrorX = x;
      }

      x = 1.0;
      expected = Math.acos(x);
      actual = EuclidCoreTools.fastAcos(x);
      assertEquals(expected, actual, epsilon, "x=" + x);
      error = Math.abs(actual - expected);
      if (error > maxError)
      {
         maxError = error;
         maxErrorX = x;
      }

      System.out.println("EuclidCoreToolsTest.testFastAcos: max error=" + maxError + " at x=" + maxErrorX);
   }

   @Test
   public void testReverse()
   {
      Random random = new Random(234234);

      for (int i = 0; i < ITERATIONS; i++)
      {
         List<Integer> originalList = new ArrayList<>();

         int size = random.nextInt(20);
         while (originalList.size() < size)
            originalList.add(Integer.valueOf(originalList.size()));

         int fromIndex = size == 0 ? 0 : random.nextInt(size);
         int toIndex = size == 0 ? 0 : fromIndex + random.nextInt(originalList.size() - fromIndex);

         List<Integer> expected = new ArrayList<>(originalList);
         Collections.reverse(expected.subList(fromIndex, toIndex));
         List<Integer> actual = new ArrayList<>(originalList);
         EuclidCoreTools.reverse(actual, fromIndex, toIndex);

         assertEquals(expected, actual);
      }
   }

   @Test
   public void testRotate()
   {
      Random random = new Random(3453);

      for (int i = 0; i < ITERATIONS; i++)
      {
         List<Integer> originalList = new ArrayList<>();

         int shift = random.nextInt(100);

         int size = random.nextInt(100);
         while (originalList.size() < size)
            originalList.add(Integer.valueOf(originalList.size()));

         int fromIndex = size == 0 ? 0 : random.nextInt(size);
         int toIndex = size == 0 ? 0 : fromIndex + random.nextInt(originalList.size() - fromIndex);
         int subSize = toIndex - fromIndex;

         List<Integer> expectedList = new ArrayList<>(originalList);

         for (int j = fromIndex; j < toIndex; j++)
         {
            int originalIndex = wrap(j - fromIndex - shift, subSize) + fromIndex;
            expectedList.set(j, originalList.get(originalIndex));
         }

         List<Integer> actualList = new ArrayList<>(originalList);
         EuclidCoreTools.rotate(actualList, fromIndex, toIndex, shift);

         assertEquals(expectedList, actualList);

         expectedList = new ArrayList<>(originalList);
         Collections.rotate(expectedList.subList(fromIndex, toIndex), shift);
         assertEquals(expectedList, actualList);
      }

      { // Example 1
         List<Integer> list = Arrays.asList(0, 1, 2, 3, 4);
         EuclidCoreTools.rotate(list, 0, list.size(), -1);
         assertEquals(Arrays.asList(1, 2, 3, 4, 0), list);
      }

      { // Example 2
         List<Integer> list = Arrays.asList(0, 1, 2, 3, 4);
         EuclidCoreTools.rotate(list, 0, list.size(), +1);
         assertEquals(Arrays.asList(4, 0, 1, 2, 3), list);
      }

      { // Example 3
         List<Integer> list = Arrays.asList(9, 0, 1, 2, 9);
         EuclidCoreTools.rotate(list, 1, 4, -1);
         assertEquals(Arrays.asList(9, 1, 2, 0, 9), list);
      }

      { // Example 4
         List<Integer> list = Arrays.asList(9, 0, 1, 2, 9);
         EuclidCoreTools.rotate(list, 1, 4, +1);
         assertEquals(Arrays.asList(9, 2, 0, 1, 9), list);
      }
   }

   @Test
   public void testFiniteDifference()
   {
      Random random = new Random(234234);

      // double:
      for (int i = 0; i < ITERATIONS; i++)
      {
         double dt = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0);
         double prevDouble = random.nextDouble();
         double expectedVelocity = EuclidCoreRandomTools.nextDouble(random);
         double nextDouble = prevDouble + dt * expectedVelocity;

         double actualVelocity = EuclidCoreTools.finiteDifference(prevDouble, nextDouble, dt);
         assertEquals(expectedVelocity, actualVelocity, 1.0e-12);
      }

      // angles:
      for (int i = 0; i < ITERATIONS; i++)
      {
         double dt = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0);
         double prevAngle = EuclidCoreRandomTools.nextDouble(random, -Math.PI, Math.PI);
         double expectedVelocity = EuclidCoreRandomTools.nextDouble(random, -Math.PI, Math.PI);
         double nextAngle = prevAngle + dt * expectedVelocity;

         double actualVelocity = EuclidCoreTools.finiteDifference(prevAngle, nextAngle, dt);
         assertEquals(expectedVelocity, actualVelocity, 1.0e-12);
      }

      // Tuple2DReadOnly:
      for (int i = 0; i < ITERATIONS; i++)
      {
         double dt = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0);
         Tuple2DReadOnly prevTuple = EuclidCoreRandomTools.nextPoint2D(random);
         Tuple2DReadOnly expectedVelocity = EuclidCoreRandomTools.nextVector2D(random);
         Tuple2DBasics nextTuple = new Point2D();
         nextTuple.scaleAdd(dt, expectedVelocity, prevTuple);

         Vector2DBasics actualVelocity = new Vector2D();
         EuclidCoreTools.finiteDifference(prevTuple, nextTuple, dt, actualVelocity);
         EuclidCoreTestTools.assertEquals(expectedVelocity, actualVelocity, 1.0e-12);
      }

      // Tuple3DReadOnly:
      for (int i = 0; i < ITERATIONS; i++)
      {
         double dt = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0);
         Tuple3DReadOnly prevTuple = EuclidCoreRandomTools.nextPoint3D(random);
         Tuple3DReadOnly expectedVelocity = EuclidCoreRandomTools.nextVector3D(random);
         Tuple3DBasics nextTuple = new Point3D();
         nextTuple.scaleAdd(dt, expectedVelocity, prevTuple);

         Vector3DBasics actualVelocity = new Vector3D();
         EuclidCoreTools.finiteDifference(prevTuple, nextTuple, dt, actualVelocity);
         EuclidCoreTestTools.assertEquals(expectedVelocity, actualVelocity, 1.0e-12);
      }

      // Tuple4DReadOnly:
      for (int i = 0; i < ITERATIONS; i++)
      {
         double dt = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0);
         Tuple4DReadOnly prevTuple = EuclidCoreRandomTools.nextVector4D(random);
         Tuple4DReadOnly expectedVelocity = EuclidCoreRandomTools.nextVector4D(random);
         Vector4D nextTuple = new Vector4D();
         nextTuple.scaleAdd(dt, expectedVelocity, prevTuple);

         Vector4DBasics actualVelocity = new Vector4D();
         EuclidCoreTools.finiteDifference(prevTuple, nextTuple, dt, actualVelocity);
         EuclidCoreTestTools.assertEquals(expectedVelocity, actualVelocity, 1.0e-12);
      }

      // Orientation2DReadOnly:
      for (int i = 0; i < ITERATIONS; i++)
      { // Let's first verify that the AxisAngle finite difference is correct.
         double dt = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0);
         Orientation2DReadOnly previousOrientation = EuclidCoreRandomTools.nextOrientation2D(random);
         Orientation2DReadOnly currentOrientation = EuclidCoreRandomTools.nextOrientation2D(random);

         double expectedAngularVelocity = EuclidCoreTools.finiteDifferenceAngle(previousOrientation.getYaw(), currentOrientation.getYaw(), dt);
         double actualAngularVelocity = EuclidCoreTools.finiteDifference(previousOrientation, currentOrientation, dt);

         assertEquals(expectedAngularVelocity, actualAngularVelocity, 1.0e-12);
      }

      // Orientation3DReadOnly:
      for (int i = 0; i < ITERATIONS; i++)
      { // Let's first verify that the Quaternion finite difference is correct.
         double dt = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0);

         // We initialize the current orientation to a random orientation.
         QuaternionReadOnly currentOrientation = EuclidCoreRandomTools.nextQuaternion(random);
         Quaternion nextOrientation = new Quaternion(); // We're going to compute this one.

         // We build the next orientation from a desired angular velocity that we're going to integrate and append.
         Vector3D expectedAngularVelocityWorld = EuclidCoreRandomTools.nextVector3D(random);
         Vector3D expectedAngularVelocityLocalCurrent = new Vector3D();
         currentOrientation.inverseTransform(expectedAngularVelocityWorld, expectedAngularVelocityLocalCurrent);
         AxisAngle diff = new AxisAngle();
         diff.setAngle(expectedAngularVelocityLocalCurrent.norm() * dt);
         diff.getAxis().set(expectedAngularVelocityLocalCurrent);
         nextOrientation.set(currentOrientation);
         nextOrientation.append(diff);

         Vector3D expectedAngularVelocityLocalNext = new Vector3D();
         nextOrientation.inverseTransform(expectedAngularVelocityWorld, expectedAngularVelocityLocalNext);

         // Showing that the angular velocity is the same in both frames.
         // This is a sanity check to make sure the test is correct.
         EuclidCoreTestTools.assertEquals(expectedAngularVelocityLocalNext, expectedAngularVelocityLocalCurrent, 1.0e-12);

         Vector3DBasics actualAngularVelocity = new Vector3D();
         QuaternionTools.finiteDifference(currentOrientation, nextOrientation, dt, actualAngularVelocity);
         EuclidCoreTestTools.assertEquals("Iteration: " + i + ", angular distance: " + nextOrientation.distance(currentOrientation) + ", velocity error: "
                                          + actualAngularVelocity.differenceNorm(expectedAngularVelocityLocalNext),
                                          expectedAngularVelocityLocalNext,
                                          actualAngularVelocity,
                                          2.0e-10);
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         double dt = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0);
         // Quaternion and others:
         compareFiniteDifference(EuclidCoreRandomTools.nextQuaternion(random), EuclidCoreRandomTools.nextQuaternion(random), dt, 1.0e-11, i);
         compareFiniteDifference(EuclidCoreRandomTools.nextQuaternion(random), EuclidCoreRandomTools.nextRotationMatrix(random), dt, 1.0e-11, i);
         compareFiniteDifference(EuclidCoreRandomTools.nextQuaternion(random), EuclidCoreRandomTools.nextAxisAngle(random), dt, 1.0e-11, i);
         compareFiniteDifference(EuclidCoreRandomTools.nextQuaternion(random), EuclidCoreRandomTools.nextYawPitchRoll(random), dt, 1.0e-11, i);

         // RotationMatrix and others:
         compareFiniteDifference(EuclidCoreRandomTools.nextRotationMatrix(random), EuclidCoreRandomTools.nextRotationMatrix(random), dt, 1.0e-11, i);
         compareFiniteDifference(EuclidCoreRandomTools.nextRotationMatrix(random), EuclidCoreRandomTools.nextQuaternion(random), dt, 1.0e-11, i);
         compareFiniteDifference(EuclidCoreRandomTools.nextRotationMatrix(random), EuclidCoreRandomTools.nextAxisAngle(random), dt, 1.0e-11, i);
         compareFiniteDifference(EuclidCoreRandomTools.nextRotationMatrix(random), EuclidCoreRandomTools.nextYawPitchRoll(random), dt, 1.0e-11, i);

         // AxisAngle and others:
         compareFiniteDifference(EuclidCoreRandomTools.nextAxisAngle(random), EuclidCoreRandomTools.nextAxisAngle(random), dt, 1.0e-11, i);
         compareFiniteDifference(EuclidCoreRandomTools.nextAxisAngle(random), EuclidCoreRandomTools.nextQuaternion(random), dt, 1.0e-11, i);
         compareFiniteDifference(EuclidCoreRandomTools.nextAxisAngle(random), EuclidCoreRandomTools.nextRotationMatrix(random), dt, 1.0e-11, i);
         compareFiniteDifference(EuclidCoreRandomTools.nextAxisAngle(random), EuclidCoreRandomTools.nextYawPitchRoll(random), dt, 1.0e-11, i);

         // YawPitchRoll and others:
         compareFiniteDifference(EuclidCoreRandomTools.nextYawPitchRoll(random), EuclidCoreRandomTools.nextYawPitchRoll(random), dt, 1.0e-11, i);
         compareFiniteDifference(EuclidCoreRandomTools.nextYawPitchRoll(random), EuclidCoreRandomTools.nextQuaternion(random), dt, 1.0e-11, i);
         compareFiniteDifference(EuclidCoreRandomTools.nextYawPitchRoll(random), EuclidCoreRandomTools.nextRotationMatrix(random), dt, 1.0e-11, i);
         compareFiniteDifference(EuclidCoreRandomTools.nextYawPitchRoll(random), EuclidCoreRandomTools.nextAxisAngle(random), dt, 1.0e-11, i);

         // To make sure we're not missing anything:
         compareFiniteDifference(EuclidCoreRandomTools.nextOrientation3D(random), EuclidCoreRandomTools.nextOrientation3D(random), dt, 1.0e-11, i);
      }

      // RigidityTransform:
      for (int i = 0; i < ITERATIONS; i++)
      {
         double dt = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0);
         RigidBodyTransform prevTransform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform diff = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform currTransform = new RigidBodyTransform();
         currTransform.set(prevTransform);
         currTransform.multiply(diff);

         Vector3DBasics expectedLinearVelocity = new Vector3D();
         expectedLinearVelocity.set(diff.getTranslation());
         expectedLinearVelocity.scale(1.0 / dt);
         prevTransform.transform(expectedLinearVelocity);

         Vector3DBasics expectedAngularVelocity = new Vector3D();
         diff.getRotation().getRotationVector(expectedAngularVelocity);
         expectedAngularVelocity.scale(1.0 / dt);

         Vector3DBasics actualLinearVelocity = new Vector3D();
         Vector3DBasics actualAngularVelocity = new Vector3D();
         EuclidCoreTools.finiteDifference(prevTransform, currTransform, dt, actualAngularVelocity, actualLinearVelocity);

         EuclidCoreTestTools.assertEquals(expectedLinearVelocity, actualLinearVelocity, 1.0e-11);
      }
   }

   private static void compareFiniteDifference(Orientation3DReadOnly prevOrientation,
                                               Orientation3DReadOnly currOrientation,
                                               double dt,
                                               double epsilon,
                                               int iteration)
   {
      Vector3DBasics actualAngularVelocity = new Vector3D();
      Vector3DBasics expectedAngularVelocity = new Vector3D();

      Quaternion prevQuaternion = new Quaternion(prevOrientation);
      Quaternion currQuaternion = new Quaternion(currOrientation);
      if (prevQuaternion.distance(currQuaternion) > Math.PI)
      {
         if (prevOrientation.distance(currOrientation) < Math.PI)
         { // We're dealing with an implementation of Orientation3DReadOnly that is limited to [-pi, pi], need to restrict the quaternion to the same range.
            prevQuaternion.negate();
         }
      }
      QuaternionTools.finiteDifference(prevQuaternion, currQuaternion, dt, actualAngularVelocity);
      EuclidCoreTools.finiteDifference(prevOrientation, currOrientation, dt, expectedAngularVelocity);
      double toleranceScale = Math.max(1.0, expectedAngularVelocity.norm());
      EuclidCoreTestTools.assertEquals("Iteration: %d, prev type: %s, curr type: %s, error norm rel: %s".formatted(iteration,
                                                                                                                   prevOrientation.getClass().getSimpleName(),
                                                                                                                   currOrientation.getClass().getSimpleName(),
                                                                                                                   actualAngularVelocity.differenceNorm(
                                                                                                                         expectedAngularVelocity)
                                                                                                                   / toleranceScale),
                                       expectedAngularVelocity,
                                       actualAngularVelocity,
                                       epsilon * toleranceScale);
   }

   @Test
   public void testIntegrate()
   {
      Random random = new Random(234234);

      // double:
      for (int i = 0; i < ITERATIONS; i++)
      {
         double dt = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0);
         double prevValue = EuclidCoreRandomTools.nextDouble(random, 10.0);
         double expectedCurrValue = EuclidCoreRandomTools.nextDouble(random, 10.0);
         double derivative = EuclidCoreTools.finiteDifference(prevValue, expectedCurrValue, dt);

         double actualCurrValue = EuclidCoreTools.integrate(prevValue, derivative, dt);
         assertEquals(expectedCurrValue, actualCurrValue, 1.0e-12);
      }

      // angles:
      for (int i = 0; i < ITERATIONS; i++)
      {
         double dt = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0);
         double prevAngle = EuclidCoreRandomTools.nextDouble(random, -Math.PI, Math.PI);
         double expectedCurrAngle = EuclidCoreRandomTools.nextDouble(random, -Math.PI, Math.PI);
         double derivative = EuclidCoreTools.finiteDifferenceAngle(prevAngle, expectedCurrAngle, dt);

         double actualCurrAngle = EuclidCoreTools.integrateAngle(prevAngle, derivative, dt);
         assertEquals(expectedCurrAngle, actualCurrAngle, 1.0e-12);
      }

      // Tuple2D:
      for (int i = 0; i < ITERATIONS; i++)
      {
         double dt = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0);
         Tuple2DReadOnly prevTuple = EuclidCoreRandomTools.nextPoint2D(random);
         Tuple2DReadOnly expectedCurrTuple = EuclidCoreRandomTools.nextPoint2D(random);
         Vector2D derivative = new Vector2D();
         EuclidCoreTools.finiteDifference(prevTuple, expectedCurrTuple, dt, derivative);

         Point2D actualCurrTuple = new Point2D();
         EuclidCoreTools.integrate(prevTuple, derivative, dt, actualCurrTuple);
         EuclidCoreTestTools.assertEquals(expectedCurrTuple, actualCurrTuple, 1.0e-12);
      }

      // Tuple3D:
      for (int i = 0; i < ITERATIONS; i++)
      {
         double dt = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0);
         Tuple3DReadOnly prevTuple = EuclidCoreRandomTools.nextPoint3D(random);
         Tuple3DReadOnly expectedCurrTuple = EuclidCoreRandomTools.nextPoint3D(random);
         Vector3D derivative = new Vector3D();
         EuclidCoreTools.finiteDifference(prevTuple, expectedCurrTuple, dt, derivative);

         Point3D actualCurrTuple = new Point3D();
         EuclidCoreTools.integrate(prevTuple, derivative, dt, actualCurrTuple);
         EuclidCoreTestTools.assertEquals(expectedCurrTuple, actualCurrTuple, 1.0e-12);
      }

      // Tuple4D:
      for (int i = 0; i < ITERATIONS; i++)
      {
         double dt = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0);
         Tuple4DReadOnly prevTuple = EuclidCoreRandomTools.nextVector4D(random);
         Tuple4DReadOnly expectedCurrTuple = EuclidCoreRandomTools.nextVector4D(random);
         Vector4D derivative = new Vector4D();
         EuclidCoreTools.finiteDifference(prevTuple, expectedCurrTuple, dt, derivative);

         Vector4D actualCurrTuple = new Vector4D();
         EuclidCoreTools.integrate(prevTuple, derivative, dt, actualCurrTuple);
         EuclidCoreTestTools.assertEquals(expectedCurrTuple, actualCurrTuple, 1.0e-12);
      }

      // Orientation2D:
      for (int i = 0; i < ITERATIONS; i++)
      { // Let's first verify that the AxisAngle finite difference is correct.
         double dt = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0);
         Orientation2DReadOnly prevOrientation = EuclidCoreRandomTools.nextOrientation2D(random);
         Orientation2DReadOnly expectedCurrOrientation = EuclidCoreRandomTools.nextOrientation2D(random);
         double derivative = EuclidCoreTools.finiteDifference(prevOrientation.getYaw(), expectedCurrOrientation.getYaw(), dt);

         Orientation2D actualCurrOrientation = new Orientation2D();
         EuclidCoreTools.integrate(prevOrientation, derivative, dt, actualCurrOrientation);

         EuclidCoreTestTools.assertEquals(expectedCurrOrientation, actualCurrOrientation, 1.0e-12);
      }

      // Orientation3D:
      for (int i = 0; i < ITERATIONS; i++)
      {
         double dt = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0);
         Orientation3DReadOnly prevOrientation = EuclidCoreRandomTools.nextOrientation3D(random);
         Orientation3DReadOnly expectedCurrOrientation = EuclidCoreRandomTools.nextOrientation3D(random);
         Vector3D derivative = new Vector3D();
         EuclidCoreTools.finiteDifference(prevOrientation, expectedCurrOrientation, dt, derivative);

         Orientation3DBasics actualCurrOrientation = EuclidCoreRandomTools.nextOrientation3D(random);
         EuclidCoreTools.integrate(prevOrientation, derivative, dt, actualCurrOrientation);

         EuclidCoreTestTools.assertGeometricallyEquals(expectedCurrOrientation, actualCurrOrientation, 1.0e-12);
      }

      // RigidBodyTransform:
      for (int i = 0; i < ITERATIONS; i++)
      {
         double dt = EuclidCoreRandomTools.nextDouble(random, 0.1, 1.0);
         RigidBodyTransform prevTransform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform diff = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform expectedCurrTransform = new RigidBodyTransform();
         expectedCurrTransform.set(prevTransform);
         expectedCurrTransform.multiply(diff);

         Vector3D expectedLinearVelocity = new Vector3D();
         expectedLinearVelocity.set(diff.getTranslation());
         expectedLinearVelocity.scale(1.0 / dt);
         prevTransform.transform(expectedLinearVelocity);

         Vector3D expectedAngularVelocity = new Vector3D();
         diff.getRotation().getRotationVector(expectedAngularVelocity);
         expectedAngularVelocity.scale(1.0 / dt);

         RigidBodyTransform actualCurrTransform = new RigidBodyTransform();
         EuclidCoreTools.integrate(prevTransform, expectedAngularVelocity, expectedLinearVelocity, dt, actualCurrTransform);

         EuclidCoreTestTools.assertEquals(expectedCurrTransform, actualCurrTransform, 1.0e-12);
      }
   }

   private static final int iters = 1000;


   @Test
   public void testProjectRotationOnAxis()
   {
      Random random = new Random(9429424L);
      Quaternion fullRotation = new Quaternion();
      Quaternion actualRotation = new Quaternion();

      for (int i = 0; i < 10000; i++)
      {
         // Create random axis and a rotation around that axis.
         Vector3D axis = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0);
         double angle = EuclidCoreRandomTools.nextDouble(random, -Math.PI, Math.PI);
         Quaternion expectedRotation = new Quaternion(new AxisAngle(axis, angle));

         // Create an orthogonal rotation.
         Vector3D orthogonalAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, axis, true);
         double orthogonalAngle = EuclidCoreRandomTools.nextDouble(random, -Math.PI, Math.PI);
         Quaternion orthogonalRotation = new Quaternion(new AxisAngle(orthogonalAxis, orthogonalAngle));

         // From the combined rotation and the original axis back out the rotation around the original axis.
         fullRotation.multiply(orthogonalRotation, expectedRotation);
         EuclidCoreTools.projectRotationOnAxis(fullRotation, axis, actualRotation);
         EuclidCoreTestTools.assertOrientation3DGeometricallyEquals(expectedRotation, actualRotation, 1.0e-10);
      }
   }

   @Test
   public void testRotationMatrix3DFromFirstToSecondVector3D()
   {
      Random random = new Random(43634);

      for (int i = 0; i < 10000; i++)
      {
         Vector3D firstVector = EuclidCoreRandomTools.nextVector3D(random);
         Vector3D secondVector = EuclidCoreRandomTools.nextVector3D(random);

         RotationMatrix actualRotationMatrix = new RotationMatrix();
         AxisAngle actualAxisAngle = new AxisAngle();
         AxisAngle expectedAxisAngle = new AxisAngle();

         EuclidGeometryTools.rotationMatrix3DFromFirstToSecondVector3D(firstVector, secondVector, actualRotationMatrix);
         actualAxisAngle.set(actualRotationMatrix);
         EuclidGeometryTools.orientation3DFromFirstToSecondVector3D(firstVector, secondVector, expectedAxisAngle);
         EuclidCoreTestTools.assertOrientation3DGeometricallyEquals(expectedAxisAngle, actualAxisAngle, 1.0e-7);
      }
   }

   @Test
   public void testDistanceBetweenTwoLineSegment2Ds()
   {
      Point2D closestPointOnLineSegment1 = new Point2D();
      Point2D closestPointOnLineSegment2 = new Point2D();

      Vector2D lineSegmentDirection1 = new Vector2D();
      Vector2D lineSegmentDirection2 = new Vector2D();

      Random random = new Random(11762L);

      // Parallel case, expecting expectedPointOnLineSegment1 =
      // lineSegmentStart1
      for (int i = 0; i < iters; i++)
      {
         Point2D lineSegmentStart1 = EuclidCoreRandomTools.nextPoint2D(random);
         lineSegmentStart1.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         Point2D lineSegmentEnd1 = EuclidCoreRandomTools.nextPoint2D(random);
         lineSegmentEnd1.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));

         lineSegmentDirection1.sub(lineSegmentEnd1, lineSegmentStart1);
         lineSegmentDirection1.normalize();

         // expectedPointOnLineSegment1 = lineSegmentStart1
         closestPointOnLineSegment1.set(lineSegmentStart1);

         // Create the closest point of line segment 2
         Vector2D orthogonalToLineSegment1 = nextOrthogonalVector2D(random, lineSegmentDirection1, true);
         double expectedMinimumDistance = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);
         closestPointOnLineSegment2.scaleAdd(expectedMinimumDistance, orthogonalToLineSegment1, closestPointOnLineSegment1);

         // Set the lineSegmentDirection2 = lineSegmentDirection1
         lineSegmentDirection2.set(lineSegmentDirection1);

         // Set the end points of the line segment 2 around the expected
         // closest point.
         Point2D lineSegmentStart2 = new Point2D();
         Point2D lineSegmentEnd2 = new Point2D();
         lineSegmentStart2.scaleAdd(EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0), lineSegmentDirection2, closestPointOnLineSegment2);
         lineSegmentEnd2.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), lineSegmentDirection2, closestPointOnLineSegment2);

         double actualMinimumDistance = EuclidGeometryTools.distanceBetweenTwoLineSegment2Ds(lineSegmentStart1,
                                                                                             lineSegmentEnd1,
                                                                                             lineSegmentStart2,
                                                                                             lineSegmentEnd2);
         assertEquals(expectedMinimumDistance, actualMinimumDistance, 1e-12);

         // Set the end points of the line segment 2 before the expected
         // closest point, so we have expectedClosestPointOnLineSegment2 =
         // lineSegmentEnd2
         double shiftStartFromExpected = EuclidCoreRandomTools.nextDouble(random, -20.0, -10.0);
         double shiftEndFromExpected = EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0);
         lineSegmentStart2.scaleAdd(shiftStartFromExpected, lineSegmentDirection2, closestPointOnLineSegment2);
         lineSegmentEnd2.scaleAdd(shiftEndFromExpected, lineSegmentDirection2, closestPointOnLineSegment2);
         closestPointOnLineSegment2.set(lineSegmentEnd2);
         expectedMinimumDistance = closestPointOnLineSegment1.distance(closestPointOnLineSegment2);

         actualMinimumDistance = EuclidGeometryTools.distanceBetweenTwoLineSegment2Ds(lineSegmentStart1,
                                                                                      lineSegmentEnd1,
                                                                                      lineSegmentStart2,
                                                                                      lineSegmentEnd2);
         assertEquals(expectedMinimumDistance, actualMinimumDistance, 1e-12);

         actualMinimumDistance = EuclidGeometryTools.distanceBetweenTwoLineSegment2Ds(lineSegmentStart1,
                                                                                      lineSegmentEnd1,
                                                                                      lineSegmentEnd2,
                                                                                      lineSegmentStart2);
         assertEquals(expectedMinimumDistance, actualMinimumDistance, 1e-12);
      }

      // Case: on closest point on lineSegment1 outside end points.
      for (int i = 0; i < iters; i++)
      {
         Point2D lineSegmentStart1 = EuclidCoreRandomTools.nextPoint2D(random);
         lineSegmentStart1.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         Point2D lineSegmentEnd1 = EuclidCoreRandomTools.nextPoint2D(random);
         lineSegmentEnd1.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));

         lineSegmentDirection1.sub(lineSegmentEnd1, lineSegmentStart1);
         lineSegmentDirection1.normalize();

         // Put the first closest to the start of line segment 1
         closestPointOnLineSegment1.set(lineSegmentStart1);

         // Create the closest point of line segment 2 such that it reaches
         // out of line segment 1
         Vector2D oppositeOflineSegmentDirection1 = new Vector2D();
         oppositeOflineSegmentDirection1.setAndNegate(lineSegmentDirection1);
         Vector2D orthogonalToLineSegment1 = nextOrthogonalVector2D(random, lineSegmentDirection1, true);
         Vector2D shiftVector = new Vector2D();
         shiftVector.interpolate(orthogonalToLineSegment1, oppositeOflineSegmentDirection1, EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));
         closestPointOnLineSegment2.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), shiftVector, closestPointOnLineSegment1);

         // Set the line direction 2 to orthogonal to the shift vector
         lineSegmentDirection2 = nextOrthogonalVector2D(random, shiftVector, true);

         // Set the end points of the line segment 2 around the expected
         // closest point.
         Point2D lineSegmentStart2 = new Point2D();
         Point2D lineSegmentEnd2 = new Point2D();
         lineSegmentStart2.scaleAdd(EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0), lineSegmentDirection2, closestPointOnLineSegment2);
         lineSegmentEnd2.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), lineSegmentDirection2, closestPointOnLineSegment2);
         double expectedMinimumDistance = closestPointOnLineSegment1.distance(closestPointOnLineSegment2);

         double actualMinimumDistance = EuclidGeometryTools.distanceBetweenTwoLineSegment2Ds(lineSegmentStart1,
                                                                                             lineSegmentEnd1,
                                                                                             lineSegmentStart2,
                                                                                             lineSegmentEnd2);
         assertEquals(expectedMinimumDistance, actualMinimumDistance, 1e-12);
         actualMinimumDistance = EuclidGeometryTools.distanceBetweenTwoLineSegment2Ds(lineSegmentStart1,
                                                                                      lineSegmentEnd1,
                                                                                      lineSegmentEnd2,
                                                                                      lineSegmentStart2);
         assertEquals(expectedMinimumDistance, actualMinimumDistance, 1e-12);
         actualMinimumDistance = EuclidGeometryTools.distanceBetweenTwoLineSegment2Ds(lineSegmentEnd1,
                                                                                      lineSegmentStart1,
                                                                                      lineSegmentStart2,
                                                                                      lineSegmentEnd2);
         assertEquals(expectedMinimumDistance, actualMinimumDistance, 1e-12);
         actualMinimumDistance = EuclidGeometryTools.distanceBetweenTwoLineSegment2Ds(lineSegmentEnd1,
                                                                                      lineSegmentStart1,
                                                                                      lineSegmentEnd2,
                                                                                      lineSegmentStart2);
         assertEquals(expectedMinimumDistance, actualMinimumDistance, 1e-12);
      }

      // Edge case: both closest points are outside bounds of each line
      // segment
      for (int i = 0; i < iters; i++)
      {
         Point2D lineSegmentStart1 = EuclidCoreRandomTools.nextPoint2D(random);
         lineSegmentStart1.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         Point2D lineSegmentEnd1 = EuclidCoreRandomTools.nextPoint2D(random);
         lineSegmentEnd1.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));

         lineSegmentDirection1.sub(lineSegmentEnd1, lineSegmentStart1);
         lineSegmentDirection1.normalize();

         // Put the first closest to the start of line segment 1
         closestPointOnLineSegment1.set(lineSegmentStart1);

         // Create the closest point of line segment 2 such that it reaches
         // out of line segment 1
         Vector2D oppositeOflineSegmentDirection1 = new Vector2D();
         oppositeOflineSegmentDirection1.setAndNegate(lineSegmentDirection1);
         Vector2D orthogonalToLineSegment1 = nextOrthogonalVector2D(random, lineSegmentDirection1, true);
         Vector2D shiftVector = new Vector2D();
         double alpha = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0);
         shiftVector.interpolate(orthogonalToLineSegment1, oppositeOflineSegmentDirection1, alpha);
         closestPointOnLineSegment2.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), shiftVector, closestPointOnLineSegment1);

         // set the start of the second line segment to the expected closest
         // point
         Point2D lineSegmentStart2 = new Point2D(closestPointOnLineSegment2);

         // Set the line direction 2 to point somewhat in the same direction
         // as the shift vector
         Vector2D orthogonalToShiftVector = nextOrthogonalVector2D(random, shiftVector, true);
         lineSegmentDirection2.interpolate(shiftVector, orthogonalToShiftVector, EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));

         // Set the end points of the line segment 2 around the expected
         // closest point.
         Point2D lineSegmentEnd2 = new Point2D();
         double alpha2 = EuclidCoreRandomTools.nextDouble(random, 0.1, 10.0);
         lineSegmentEnd2.scaleAdd(alpha2, lineSegmentDirection2, closestPointOnLineSegment2);

         double expectedMinimumDistance = closestPointOnLineSegment1.distance(closestPointOnLineSegment2);
         double actualMinimumDistance = EuclidGeometryTools.distanceBetweenTwoLineSegment2Ds(lineSegmentStart1,
                                                                                             lineSegmentEnd1,
                                                                                             lineSegmentStart2,
                                                                                             lineSegmentEnd2);
         assertEquals(expectedMinimumDistance, actualMinimumDistance, 1e-12);
         actualMinimumDistance = EuclidGeometryTools.distanceBetweenTwoLineSegment2Ds(lineSegmentStart1,
                                                                                      lineSegmentEnd1,
                                                                                      lineSegmentEnd2,
                                                                                      lineSegmentStart2);
         assertEquals(expectedMinimumDistance, actualMinimumDistance, 1e-12);
         actualMinimumDistance = EuclidGeometryTools.distanceBetweenTwoLineSegment2Ds(lineSegmentEnd1,
                                                                                      lineSegmentStart1,
                                                                                      lineSegmentStart2,
                                                                                      lineSegmentEnd2);
         assertEquals(expectedMinimumDistance, actualMinimumDistance, 1e-12);
         actualMinimumDistance = EuclidGeometryTools.distanceBetweenTwoLineSegment2Ds(lineSegmentEnd1,
                                                                                      lineSegmentStart1,
                                                                                      lineSegmentEnd2,
                                                                                      lineSegmentStart2);
         assertEquals(expectedMinimumDistance, actualMinimumDistance, 1e-12);
      }
   }

   @Test
   public void testExtractNormalPart()
   {
      Random random = new Random(6457);

      for (int i = 0; i < iters; i++)
      { // We build the normal and tangential parts of the vector and then assemble it and expect to get the normal part pack.
         Vector3D normalAxis = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0);
         Vector3D tangentialAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, normalAxis, true);
         Vector3D normalPart = new Vector3D();
         Vector3D tangentialPart = new Vector3D();

         double normalMagnitude = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);
         double tangentialMagnitude = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);
         normalPart.setAndScale(normalMagnitude, normalAxis);
         tangentialPart.setAndScale(tangentialMagnitude, tangentialAxis);

         Vector3D input = new Vector3D();
         input.add(normalPart, tangentialPart);

         Vector3D actualNormalPart = new Vector3D();
         EuclidCoreTools.extractNormalPart(input, normalAxis, actualNormalPart);
         EuclidCoreTestTools.assertEquals(normalPart, actualNormalPart, 1e-12);

         // Randomize the axis
         normalAxis.scale(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0));
         EuclidCoreTools.extractNormalPart(input, normalAxis, actualNormalPart);
         EuclidCoreTestTools.assertEquals(normalPart, actualNormalPart, 1e-12);
      }
   }

   @Test
   public void testExtractTangentialPart()
   {
      Random random = new Random(63457);

      for (int i = 0; i < iters; i++)
      { // We build the normal and tangential parts of the vector and then assemble it and expect to get the normal part pack.
         // Setting trivial setup using Axis3D
         Axis3D normalAxis = EuclidCoreRandomTools.nextAxis3D(random);
         Axis3D tangentialAxis = random.nextBoolean() ? normalAxis.next() : normalAxis.previous();
         Vector3D normalPart = new Vector3D();
         Vector3D tangentialPart = new Vector3D();

         double normalMagnitude = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);
         double tangentialMagnitude = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);
         normalPart.setAndScale(normalMagnitude, normalAxis);
         tangentialPart.setAndScale(tangentialMagnitude, tangentialAxis);

         Vector3D input = new Vector3D();
         input.add(normalPart, tangentialPart);

         Vector3D actualTangentialPart = new Vector3D();
         EuclidCoreTools.extractTangentialPart(input, normalAxis, actualTangentialPart);
         EuclidCoreTestTools.assertEquals("Iteration: " + i, tangentialPart, actualTangentialPart, 1e-12);
      }

      for (int i = 0; i < iters; i++)
      { // We build the normal and tangential parts of the vector and then assemble it and expect to get the normal part pack.
         Vector3D normalAxis = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0);
         Vector3D tangentialAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, normalAxis, true);
         Vector3D normalPart = new Vector3D();
         Vector3D tangentialPart = new Vector3D();

         double normalMagnitude = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);
         double tangentialMagnitude = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);
         normalPart.setAndScale(normalMagnitude, normalAxis);
         tangentialPart.setAndScale(tangentialMagnitude, tangentialAxis);

         Vector3D input = new Vector3D();
         input.add(normalPart, tangentialPart);

         Vector3D actualTangentialPart = new Vector3D();
         EuclidCoreTools.extractTangentialPart(input, normalAxis, actualTangentialPart);
         EuclidCoreTestTools.assertEquals(tangentialPart, actualTangentialPart, 1e-12);

         // Randomize the axis
         normalAxis.scale(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0));
         EuclidCoreTools.extractTangentialPart(input, normalAxis, actualTangentialPart);
         EuclidCoreTestTools.assertEquals(tangentialPart, actualTangentialPart, 1e-12);
      }
   }

   @Test
   public void testIntersectionBetweenRay2DAndLine2D()
   {
      Point2D rayOrigin = new Point2D(8.689, 0.5687);
      Point2D pointOnRay = new Point2D(8.6432, 0.4951);
      Vector2D rayDirection = new Vector2D();
      rayDirection.sub(pointOnRay, rayOrigin);

      Point2D lineStart = new Point2D(8.4521, 0.4323);
      Point2D lineEnd = new Point2D(8.776, 0.5267);
      Vector2D lineDirection = new Vector2D();
      lineDirection.sub(lineEnd, lineStart);

      Point2D intersectionToPac = new Point2D();
      assertTrue(EuclidGeometryTools.intersectionBetweenRay2DAndLine2D(rayOrigin, rayDirection, lineStart, lineDirection, intersectionToPac));

      Point2D intersectionExpected = new Point2D();
      EuclidGeometryTools.intersectionBetweenLine2DAndLineSegment2D(rayOrigin, rayDirection, lineStart, lineEnd, intersectionExpected);

      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(intersectionExpected, intersectionToPac, 1e-5);
   }

   @Test
   public void testSetNormalPart()
   {
      Random random = new Random(897632);

      for (int i = 0; i < iters; i++)
      {
         // Setting trivial setup using Axis3D
         Axis3D normalAxis = EuclidCoreRandomTools.nextAxis3D(random);
         Axis3D tangentialAxis = random.nextBoolean() ? normalAxis.next() : normalAxis.previous();
         Vector3D normalPart = new Vector3D();
         Vector3D tangentialPart = new Vector3D();
         double normalMagnitude = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);
         double tangentialMagnitude = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);
         normalPart.setAndScale(normalMagnitude, normalAxis);
         tangentialPart.setAndScale(tangentialMagnitude, tangentialAxis);

         Point3D input = new Point3D(normalPart);
         input.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), tangentialPart, input);

         Vector3D tupleToModify = new Vector3D(tangentialPart);
         tupleToModify.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), normalPart, tupleToModify);

         Vector3D expected = new Vector3D();
         expected.add(normalPart, tangentialPart);

         EuclidCoreTools.setNormalPart(input, normalAxis, tupleToModify);
         EuclidCoreTestTools.assertEquals(expected, tupleToModify, 1e-12);
      }

      for (int i = 0; i < iters; i++)
      {
         Vector3D normalAxis = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0);
         Vector3D tangentialAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, normalAxis, true);
         Vector3D normalPart = new Vector3D();
         Vector3D tangentialPart = new Vector3D();
         double normalMagnitude = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);
         double tangentialMagnitude = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);
         normalPart.setAndScale(normalMagnitude, normalAxis);
         tangentialPart.setAndScale(tangentialMagnitude, tangentialAxis);

         Point3D input = new Point3D(normalPart);
         input.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), tangentialPart, input);

         Vector3D tupleToModify = new Vector3D(tangentialPart);
         tupleToModify.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), normalPart, tupleToModify);

         Vector3D expected = new Vector3D();
         expected.add(normalPart, tangentialPart);

         EuclidCoreTools.setNormalPart(input, normalAxis, tupleToModify);
         EuclidCoreTestTools.assertEquals(expected, tupleToModify, 1e-12);

         // Randomize the axis
         normalAxis.scale(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0));
         EuclidCoreTools.setNormalPart(input, normalAxis, tupleToModify);
         EuclidCoreTestTools.assertEquals(expected, tupleToModify, 1e-12);
      }
   }

   @Test
   public void testDifferentiateOrientation()
   {
      Random random = new Random(23423);

      for (int i = 0; i < iters; i++)
      {
         Quaternion qStart = EuclidCoreRandomTools.nextQuaternion(random);
         double duration = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0e-2);
         double angle = EuclidCoreRandomTools.nextDouble(random, 0.0, Math.PI);
         UnitVector3D velocityAxis = EuclidCoreRandomTools.nextUnitVector3D(random);
         Vector3D expectedAngularVelocity = new Vector3D();
         expectedAngularVelocity.setAndScale(angle / duration, velocityAxis);

         Quaternion qEnd = new Quaternion();
         qEnd.setRotationVector(velocityAxis.getX() * angle, velocityAxis.getY() * angle, velocityAxis.getZ() * angle);
         qEnd.prepend(qStart);

         Vector3D actualAngularVelocity = new Vector3D();
         EuclidCoreTools.differentiateOrientation(qStart, qEnd, duration, actualAngularVelocity);

         EuclidCoreTestTools.assertEquals(expectedAngularVelocity, actualAngularVelocity, 1e-12 * Math.max(1.0, expectedAngularVelocity.norm()));
      }
   }

   /**
    * Generates a random vector that is perpendicular to {@code vectorToBeOrthogonalTo}.
    *
    * @param random                 the random generator to use.
    * @param vectorToBeOrthogonalTo the vector to be orthogonal to. Not modified.
    * @param normalize              whether to normalize the generated vector or not.
    * @return the random vector.
    */
   public static Vector2D nextOrthogonalVector2D(Random random, Vector2DReadOnly vectorToBeOrthogonalTo, boolean normalize)
   {
      Vector2D v1 = new Vector2D(vectorToBeOrthogonalTo.getY(), -vectorToBeOrthogonalTo.getX());

      Vector2D randomPerpendicular = new Vector2D();
      double a = EuclidCoreRandomTools.nextDouble(random, 1.0);
      randomPerpendicular.scaleAdd(a, v1, randomPerpendicular);

      if (normalize)
         randomPerpendicular.normalize();

      return randomPerpendicular;
   }
}
