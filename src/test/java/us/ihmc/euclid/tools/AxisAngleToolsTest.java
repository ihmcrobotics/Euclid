package us.ihmc.euclid.tools;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static us.ihmc.euclid.EuclidTestConstants.ITERATIONS;
import static us.ihmc.euclid.tools.EuclidCoreRandomTools.nextAxisAngle;
import static us.ihmc.euclid.tools.EuclidCoreRandomTools.nextDouble;
import static us.ihmc.euclid.tools.EuclidCoreRandomTools.nextVector3DWithFixedLength;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.axisAngle.interfaces.AxisAngleReadOnly;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;

public class AxisAngleToolsTest
{
   private static final double EPSILON = 1.0e-14;

   @Test
   public void testDistance()
   {
      Random random = new Random(32434L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         AxisAngleReadOnly aa1 = nextAxisAngle(random);
         AxisAngleReadOnly aa2 = nextAxisAngle(random);

         Quaternion q1 = new Quaternion(aa1);
         Quaternion q2 = new Quaternion(aa2);

         double actualDistance = AxisAngleTools.distance(aa1, aa2);
         double expectedDistance = q1.distance(q2);
         assertEquals(expectedDistance, actualDistance, EPSILON);
         assertEquals(0.0, aa1.distance(aa1), EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         Vector3D u1 = nextVector3DWithFixedLength(random, nextDouble(random, 0.1, 1.0));
         Vector3D u2 = nextVector3DWithFixedLength(random, nextDouble(random, 0.1, 1.0));
         AxisAngleReadOnly aa1 = new AxisAngle(u1, nextDouble(random, Math.PI));
         AxisAngleReadOnly aa2 = new AxisAngle(u2, nextDouble(random, Math.PI));

         Quaternion q1 = new Quaternion(aa1);
         Quaternion q2 = new Quaternion(aa2);

         double actualDistance = AxisAngleTools.distance(aa1, aa2);
         double expectedDistance = q1.distance(q2);
         assertEquals(expectedDistance, actualDistance, EPSILON);
         assertEquals(0.0, aa1.distance(aa1), EPSILON);
      }

      // test distance from self(axis angle) to other(quaternion)

      for (int i = 0; i < ITERATIONS; ++i)
      {
         System.out.println("iter = " + i);
         AxisAngle axisAngle = EuclidCoreRandomTools.nextAxisAngle(random);
         Quaternion quaternion = EuclidCoreRandomTools.nextQuaternion(random);
         Quaternion converted = new Quaternion(axisAngle);

         double actualDistance = AxisAngleTools.distance(axisAngle, quaternion);
         double expectedDistance = QuaternionTools.distance(converted, quaternion, false);
         System.out.println("actual = " + actualDistance + "\nexpected = " + expectedDistance);
         assertEquals(actualDistance, expectedDistance, EPSILON);
         
//         actual = 1.0740802829893035
//         expected = 1.212024412428833
      }

      // test distance from self(axis angle) to other (rotation matrix)
      for (int i = 0; i < ITERATIONS; ++i)
      {
         AxisAngle axisAngle = EuclidCoreRandomTools.nextAxisAngle(random);
         RotationMatrix rotationMatrix = EuclidCoreRandomTools.nextRotationMatrix(random);
         RotationMatrix converted = new RotationMatrix(axisAngle);

         double actualDistance = AxisAngleTools.distance(axisAngle, rotationMatrix);
         double expectedDistance = RotationMatrixTools.distance(converted, rotationMatrix);

         assertEquals(actualDistance, expectedDistance, EPSILON);

      }

      // test distance from self(axis angle) to other (yaw pitch roll)
      for (int i = 0; i < ITERATIONS; ++i)
      {
         AxisAngle axisAngle = EuclidCoreRandomTools.nextAxisAngle(random);
         YawPitchRoll yawPitchRoll = EuclidCoreRandomTools.nextYawPitchRoll(random);
         AxisAngle converted = new AxisAngle(yawPitchRoll);

         double actualDistance = AxisAngleTools.distance(axisAngle, yawPitchRoll);
         double expectedDistance = AxisAngleTools.distance(converted, axisAngle);
         System.out.println("actual = " + actualDistance + "\nexpected = " + expectedDistance);
         assertEquals(actualDistance, expectedDistance, EPSILON);
      }

   }
}
