package us.ihmc.euclid.tools;

import org.junit.jupiter.api.Test;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.orientation.Orientation2D;
import us.ihmc.euclid.transform.AffineTransform;
import us.ihmc.euclid.transform.QuaternionBasedTransform;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Vector4D;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;

import java.util.Arrays;
import java.util.List;
import java.util.Random;
import java.util.stream.IntStream;

import static org.junit.jupiter.api.Assertions.*;

public class EuclidCoreIOToolsTest
{
   @Test
   public void testGetRigidBodyTransformStringRegression()
   {
      Random random = new Random(345345);
      String expected;
      String randomFormat = EuclidCoreIOTools.getStringFormat(8, 7);

      RigidBodyTransform t = EuclidCoreRandomTools.nextRigidBodyTransform(random);

      expected = "-0.146  0.372  0.917 |  0.969\n" //
                 + " 0.050  0.928 -0.369 |  0.028\n" //
                 + "-0.988 -0.008 -0.154 | -0.619\n" //
                 + " 0.000  0.000  0.000 |  1.000";
      assertEquals(expected, EuclidCoreIOTools.getRigidBodyTransformString(t));
      assertEquals(expected, EuclidCoreIOTools.getRigidBodyTransformString(EuclidCoreIOTools.DEFAULT_FORMAT, t));
      assertEquals("null", EuclidCoreIOTools.getRigidBodyTransformString(null));
      expected = "-0.1460544 0.3719639 0.9166848 | 0.9690608\n" //
                 + "0.0497947 0.9282109 -0.3687071 | 0.0275034\n" //
                 + "-0.9880226 -0.0082053 -0.1540911 | -0.6188257\n"//
                 + "0.0000000 0.0000000 0.0000000 | 1.0000000";
      assertEquals(expected, EuclidCoreIOTools.getRigidBodyTransformString(randomFormat, t));
      expected = "-0.14605438133050594 0.3719638705161967 0.916684785913224 | 0.9690607506715772\n" //
                 + "0.04979465293823307 0.9282109416799752 -0.36870711992628336 | 0.02750342450772747\n" //
                 + "-0.9880225757708799 -0.008205289524678472 -0.154091086669191 | -0.6188257166341338\n"//
                 + "0.0 0.0 0.0 | 1.0";
      assertEquals(expected, EuclidCoreIOTools.getRigidBodyTransformString(null, t));
   }

   @Test
   public void testGetAffineTransformStringRegression()
   {
      Random random = new Random(345345);
      String expected;
      String randomFormat = EuclidCoreIOTools.getStringFormat(8, 7);

      AffineTransform t = EuclidCoreRandomTools.nextAffineTransform(random);

      expected = " 1.639 -5.595  9.691 | -0.618\n" //
                 + " 0.275 -6.188  7.012 | -0.556\n" //
                 + "-0.728  6.787 -4.461 |  0.498\n" //
                 + " 0.000  0.000  0.000 |  1.000";
      assertEquals(expected, EuclidCoreIOTools.getAffineTransformString(t));
      assertEquals(expected, EuclidCoreIOTools.getAffineTransformString(EuclidCoreIOTools.DEFAULT_FORMAT, t));
      assertEquals("null", EuclidCoreIOTools.getAffineTransformString(null));

      expected = "1.6394447 -5.5954184 9.6906075 | -0.6176655\n" //
                 + "0.2750342 -6.1882572 7.0120891 | -0.5556910\n" //
                 + "-0.7281610 6.7873417 -4.4606236 | 0.4979491\n"//
                 + "0.0000000 0.0000000 0.0000000 | 1.0000000";
      assertEquals(expected, EuclidCoreIOTools.getAffineTransformString(randomFormat, t));

      expected = "1.6394446573547388 -5.59541839804184 9.690607506715772 | -0.6176654552449496\n" //
                 + "0.2750342450772756 -6.188257166341337 7.012089076126404 | -0.5556910209492063\n" //
                 + "-0.7281610246766981 6.787341699515743 -4.460623648325692 | 0.4979490960610431\n"//
                 + "0.0 0.0 0.0 | 1.0";
      assertEquals(expected, EuclidCoreIOTools.getAffineTransformString(null, t));
   }

   @Test
   public void testGetQuaternionBasedTransformStringRegression()
   {
      Random random = new Random(345345);
      String expected;
      String randomFormat = EuclidCoreIOTools.getStringFormat(8, 7);

      QuaternionBasedTransform t = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);

      expected = "Quaternion:  ( 0.141,  0.746, -0.126,  0.638 )\nTranslation: ( 0.969,  0.028, -0.619 )";
      assertEquals(expected, EuclidCoreIOTools.getQuaternionBasedTransformString(t));
      assertEquals(expected, EuclidCoreIOTools.getQuaternionBasedTransformString(EuclidCoreIOTools.DEFAULT_FORMAT, t));
      assertEquals("null", EuclidCoreIOTools.getQuaternionBasedTransformString(null));

      expected = "Quaternion:  (0.1412673, 0.7463840, -0.1262461, 0.6379783 )\nTranslation: (0.9690608, 0.0275034, -0.6188257 )";
      assertEquals(expected, EuclidCoreIOTools.getQuaternionBasedTransformString(randomFormat, t));

      expected = "Quaternion:  (0.14126726766904454, 0.7463840180630331, -0.12624614150672073, 0.6379783447892801 )\n"
                 + "Translation: (0.9690607506715772, 0.02750342450772747, -0.6188257166341338 )";
      assertEquals(expected, EuclidCoreIOTools.getQuaternionBasedTransformString(null, t));
   }

   @Test
   public void testGetTuple2DStringRegression()
   {
      Random random = new Random(345345);
      String expected;
      String randomFormat = EuclidCoreIOTools.getStringFormat(8, 7);

      Point2D t = EuclidCoreRandomTools.nextPoint2D(random);

      expected = "( 0.164, -0.560 )";
      assertEquals(expected, EuclidCoreIOTools.getTuple2DString(t));
      assertEquals(expected, EuclidCoreIOTools.getTuple2DString(EuclidCoreIOTools.DEFAULT_FORMAT, t));
      assertEquals("null", EuclidCoreIOTools.getTuple2DString(null));

      expected = "(0.1639445, -0.5595418 )";
      assertEquals(expected, EuclidCoreIOTools.getTuple2DString(randomFormat, t));

      expected = "(0.16394446573547383, -0.559541839804184 )";
      assertEquals(expected, EuclidCoreIOTools.getTuple2DString(null, t));
   }

   @Test
   public void testGetTuple3DStringRegression()
   {
      Random random = new Random(345345);
      String expected;
      String randomFormat = EuclidCoreIOTools.getStringFormat(8, 7);

      Point3D t = EuclidCoreRandomTools.nextPoint3D(random);

      expected = "( 0.164, -0.560,  0.969 )";
      assertEquals(expected, EuclidCoreIOTools.getTuple3DString(t));
      assertEquals(expected, EuclidCoreIOTools.getTuple3DString(EuclidCoreIOTools.DEFAULT_FORMAT, t));
      assertEquals("null", EuclidCoreIOTools.getTuple3DString(null));

      expected = "(0.1639445, -0.5595418, 0.9690608 )";
      assertEquals(expected, EuclidCoreIOTools.getTuple3DString(randomFormat, t));

      expected = "(0.16394446573547383, -0.559541839804184, 0.9690607506715772 )";
      assertEquals(expected, EuclidCoreIOTools.getTuple3DString(null, t));
   }

   @Test
   public void testGetTuple4DStringRegression()
   {
      Random random = new Random(345345);
      String expected;
      String randomFormat = EuclidCoreIOTools.getStringFormat(8, 7);

      Vector4D t = EuclidCoreRandomTools.nextVector4D(random);

      expected = "( 0.164, -0.560,  0.969,  0.028 )";
      assertEquals(expected, EuclidCoreIOTools.getTuple4DString(t));
      assertEquals(expected, EuclidCoreIOTools.getTuple4DString(EuclidCoreIOTools.DEFAULT_FORMAT, t));
      assertEquals("null", EuclidCoreIOTools.getTuple4DString(null));

      expected = "(0.1639445, -0.5595418, 0.9690608, 0.0275034 )";
      assertEquals(expected, EuclidCoreIOTools.getTuple4DString(randomFormat, t));

      expected = "(0.16394446573547383, -0.559541839804184, 0.9690607506715772, 0.02750342450772747 )";
      assertEquals(expected, EuclidCoreIOTools.getTuple4DString(null, t));
   }

   @Test
   public void testGetAxisAngleStringRegression()
   {
      Random random = new Random(345345);
      String expected;
      String randomFormat = EuclidCoreIOTools.getStringFormat(8, 7);

      AxisAngle t = EuclidCoreRandomTools.nextAxisAngle(random);

      expected = "(-0.183, -0.969,  0.164, -1.758 )";
      assertEquals(expected, EuclidCoreIOTools.getAxisAngleString(t));
      assertEquals(expected, EuclidCoreIOTools.getAxisAngleString(EuclidCoreIOTools.DEFAULT_FORMAT, t));
      assertEquals("null", EuclidCoreIOTools.getAxisAngleString(null));
      expected = "(-0.1834510, -0.9692615, 0.1639445, -1.7578525 )";
      assertEquals(expected, EuclidCoreIOTools.getAxisAngleString(randomFormat, t));
      expected = "(-0.18345104608744625, -0.9692615363482303, 0.16394446573547383, -1.7578525333049415 )";
      assertEquals(expected, EuclidCoreIOTools.getAxisAngleString(null, t));
   }

   @Test
   public void testGetOrientation2DStringRegression()
   {
      Random random = new Random(345345);
      String expected;
      String randomFormat = EuclidCoreIOTools.getStringFormat(8, 7);

      Orientation2D t = EuclidCoreRandomTools.nextOrientation2D(random);

      expected = "( 0.515 )";
      assertEquals(expected, EuclidCoreIOTools.getOrientation2DString(t));
      assertEquals(expected, EuclidCoreIOTools.getOrientation2DString(EuclidCoreIOTools.DEFAULT_FORMAT, t));
      assertEquals("null", EuclidCoreIOTools.getOrientation2DString(null));
      expected = "(0.5150467 )";
      assertEquals(expected, EuclidCoreIOTools.getOrientation2DString(randomFormat, t));
      expected = "(0.5150467291512681 )";
      assertEquals(expected, EuclidCoreIOTools.getOrientation2DString(null, t));
   }

   @Test
   public void testGetMatrix3DStringRegression()
   {
      Random random = new Random(345345);
      String expected;
      String randomFormat = EuclidCoreIOTools.getStringFormat(8, 7);

      Matrix3D t = EuclidCoreRandomTools.nextMatrix3D(random);

      expected = "/ 0.164, -0.560,  0.969 \\\n"//
                 + "| 0.028, -0.619,  0.701 |\n"//
                 + "\\-0.073,  0.679, -0.446 /";
      assertEquals(expected, EuclidCoreIOTools.getMatrix3DString(t));
      assertEquals(expected, EuclidCoreIOTools.getMatrix3DString(EuclidCoreIOTools.DEFAULT_FORMAT, t));
      assertEquals("null", EuclidCoreIOTools.getMatrix3DString(null));
      expected = "/ 0.1639445, -0.5595418,  0.9690608 \\\n"//
                 + "| 0.0275034, -0.6188257,  0.7012089 |\n"//
                 + "\\-0.0728161,  0.6787342, -0.4460624 /";
      assertEquals(expected, EuclidCoreIOTools.getMatrix3DString(randomFormat, t));
      expected = "/0.16394446573547383,  -0.559541839804184,  0.9690607506715772 \\\n"//
                 + "|0.02750342450772747, -0.6188257166341338,  0.7012089076126404 |\n"//
                 + "\\-0.0728161024676699,  0.6787341699515741, -0.4460623648325692 /";
      assertEquals(expected, EuclidCoreIOTools.getMatrix3DString(null, t));
   }

   @Test
   public void testGetYawPitchRollStringRegression()
   {
      Random random = new Random(345345);
      String expected;
      String randomFormat = EuclidCoreIOTools.getStringFormat(8, 7);

      YawPitchRoll t = EuclidCoreRandomTools.nextYawPitchRoll(random);

      expected = "yaw-pitch-roll: ( 0.515, -0.861,  3.044)";
      assertEquals(expected, EuclidCoreIOTools.getYawPitchRollString(t));
      assertEquals(expected, EuclidCoreIOTools.getYawPitchRollString(EuclidCoreIOTools.DEFAULT_FORMAT, t));
      assertEquals("null", EuclidCoreIOTools.getYawPitchRollString(null));
      expected = "yaw-pitch-roll: (0.5150467, -0.8611524, 3.0443941)";
      assertEquals(expected, EuclidCoreIOTools.getYawPitchRollString(randomFormat, t));
      expected = "yaw-pitch-roll: (0.5150467291512681, -0.8611524243712763, 3.0443941351920367)";
      assertEquals(expected, EuclidCoreIOTools.getYawPitchRollString(null, t));
   }

   @Test
   public void testGetStringOf()
   {
      Random random = new Random(345345);
      String expected;
      String separator = "-p-";
      String randomFormat = EuclidCoreIOTools.getStringFormat(8, 7);

      { // Test generators for double values
         double[] values = random.doubles(3).map(d -> d - 0.5).toArray();
         expected = " 0.082-p--0.280-p- 0.485";
         assertEquals(expected, EuclidCoreIOTools.getStringOf(separator, values));
         assertEquals(expected, EuclidCoreIOTools.getStringOf(separator, EuclidCoreIOTools.DEFAULT_FORMAT, values));
         assertEquals(expected, EuclidCoreIOTools.getStringOf(null, null, separator, values));
         assertEquals(expected, EuclidCoreIOTools.getStringOf(null, null, separator, EuclidCoreIOTools.DEFAULT_FORMAT, values));

         assertEquals("null", EuclidCoreIOTools.getStringOf(separator, (double[]) null));
         assertEquals("null", EuclidCoreIOTools.getStringOf(separator, EuclidCoreIOTools.DEFAULT_FORMAT, (double[]) null));
         assertEquals("null", EuclidCoreIOTools.getStringOf(null, null, separator, (double[]) null));
         assertEquals("null", EuclidCoreIOTools.getStringOf(null, null, separator, EuclidCoreIOTools.DEFAULT_FORMAT, (double[]) null));

         assertEquals("", EuclidCoreIOTools.getStringOf(separator, new double[0]));
         assertEquals("", EuclidCoreIOTools.getStringOf(separator, EuclidCoreIOTools.DEFAULT_FORMAT, new double[0]));
         assertEquals("", EuclidCoreIOTools.getStringOf(null, null, separator, new double[0]));
         assertEquals("", EuclidCoreIOTools.getStringOf(null, null, separator, EuclidCoreIOTools.DEFAULT_FORMAT, new double[0]));

         expected = "0.0819722-p--0.2797709-p-0.4845304";
         assertEquals(expected, EuclidCoreIOTools.getStringOf(separator, randomFormat, values));
         assertEquals(expected, EuclidCoreIOTools.getStringOf(null, null, separator, randomFormat, values));

         expected = "0.08197223286773692-p--0.279770919902092-p-0.4845303753357886";
         assertEquals(expected, EuclidCoreIOTools.getStringOf(separator, null, values));
         assertEquals(expected, EuclidCoreIOTools.getStringOf(null, null, separator, null, values));

         String prefix = "!prefix!";
         String suffix = "?suffix?";
         expected = "!prefix! 0.082-p--0.280-p- 0.485?suffix?";
         assertEquals(expected, EuclidCoreIOTools.getStringOf(prefix, suffix, separator, values));
         assertEquals(expected, EuclidCoreIOTools.getStringOf(prefix, suffix, separator, EuclidCoreIOTools.DEFAULT_FORMAT, values));

         assertEquals("null", EuclidCoreIOTools.getStringOf(prefix, suffix, separator, (double[]) null));
         assertEquals("null", EuclidCoreIOTools.getStringOf(prefix, suffix, separator, EuclidCoreIOTools.DEFAULT_FORMAT, (double[]) null));

         assertEquals("!prefix!?suffix?", EuclidCoreIOTools.getStringOf(prefix, suffix, separator, new double[0]));
         assertEquals("!prefix!?suffix?", EuclidCoreIOTools.getStringOf(prefix, suffix, separator, EuclidCoreIOTools.DEFAULT_FORMAT, new double[0]));

         expected = "!prefix!0.0819722-p--0.2797709-p-0.4845304?suffix?";
         assertEquals(expected, EuclidCoreIOTools.getStringOf(prefix, suffix, separator, randomFormat, values));

         expected = "!prefix!0.08197223286773692-p--0.279770919902092-p-0.4845303753357886?suffix?";
         assertEquals(expected, EuclidCoreIOTools.getStringOf(prefix, suffix, separator, null, values));
      }

      { // Test generators for float values
         float[] values = {random.nextFloat() - 0.5f, random.nextFloat() - 0.5f, random.nextFloat() - 0.5f};
         expected = " 0.014-p- 0.287-p--0.309";
         assertEquals(expected, EuclidCoreIOTools.getStringOf(separator, values));
         assertEquals(expected, EuclidCoreIOTools.getStringOf(separator, EuclidCoreIOTools.DEFAULT_FORMAT, values));
         assertEquals(expected, EuclidCoreIOTools.getStringOf(null, null, separator, values));
         assertEquals(expected, EuclidCoreIOTools.getStringOf(null, null, separator, EuclidCoreIOTools.DEFAULT_FORMAT, values));

         assertEquals("null", EuclidCoreIOTools.getStringOf(separator, (float[]) null));
         assertEquals("null", EuclidCoreIOTools.getStringOf(separator, EuclidCoreIOTools.DEFAULT_FORMAT, (float[]) null));
         assertEquals("null", EuclidCoreIOTools.getStringOf(null, null, separator, (float[]) null));
         assertEquals("null", EuclidCoreIOTools.getStringOf(null, null, separator, EuclidCoreIOTools.DEFAULT_FORMAT, (float[]) null));

         assertEquals("", EuclidCoreIOTools.getStringOf(separator, new float[0]));
         assertEquals("", EuclidCoreIOTools.getStringOf(separator, EuclidCoreIOTools.DEFAULT_FORMAT, new float[0]));
         assertEquals("", EuclidCoreIOTools.getStringOf(null, null, separator, new float[0]));
         assertEquals("", EuclidCoreIOTools.getStringOf(null, null, separator, EuclidCoreIOTools.DEFAULT_FORMAT, new float[0]));

         expected = "0.0137517-p-0.2874116-p--0.3094129";
         assertEquals(expected, EuclidCoreIOTools.getStringOf(separator, randomFormat, values));
         assertEquals(expected, EuclidCoreIOTools.getStringOf(null, null, separator, randomFormat, values));

         expected = "0.013751686-p-0.28741163-p--0.3094129";
         assertEquals(expected, EuclidCoreIOTools.getStringOf(separator, null, values));
         assertEquals(expected, EuclidCoreIOTools.getStringOf(null, null, separator, null, values));

         String prefix = "!prefix!";
         String suffix = "?suffix?";
         expected = "!prefix! 0.014-p- 0.287-p--0.309?suffix?";
         assertEquals(expected, EuclidCoreIOTools.getStringOf(prefix, suffix, separator, values));
         assertEquals(expected, EuclidCoreIOTools.getStringOf(prefix, suffix, separator, EuclidCoreIOTools.DEFAULT_FORMAT, values));

         assertEquals("null", EuclidCoreIOTools.getStringOf(prefix, suffix, separator, (float[]) null));
         assertEquals("null", EuclidCoreIOTools.getStringOf(prefix, suffix, separator, EuclidCoreIOTools.DEFAULT_FORMAT, (float[]) null));

         assertEquals("!prefix!?suffix?", EuclidCoreIOTools.getStringOf(prefix, suffix, separator, new float[0]));
         assertEquals("!prefix!?suffix?", EuclidCoreIOTools.getStringOf(prefix, suffix, separator, EuclidCoreIOTools.DEFAULT_FORMAT, new float[0]));

         expected = "!prefix!0.0137517-p-0.2874116-p--0.3094129?suffix?";
         assertEquals(expected, EuclidCoreIOTools.getStringOf(prefix, suffix, separator, randomFormat, values));

         expected = "!prefix!0.013751686-p-0.28741163-p--0.3094129?suffix?";
         assertEquals(expected, EuclidCoreIOTools.getStringOf(prefix, suffix, separator, null, values));
      }

      { // Test generators for boolean values
         boolean[] values = {random.nextBoolean(), random.nextBoolean(), random.nextBoolean(), random.nextBoolean()};
         expected = "true-p-true-p-true-p-false";
         assertEquals(expected, EuclidCoreIOTools.getStringOf(separator, values));
         assertEquals(expected, EuclidCoreIOTools.getStringOf(null, null, separator, values));

         assertEquals("null", EuclidCoreIOTools.getStringOf(separator, (boolean[]) null));
         assertEquals("null", EuclidCoreIOTools.getStringOf(null, null, separator, (boolean[]) null));

         assertEquals("", EuclidCoreIOTools.getStringOf(separator, new boolean[0]));
         assertEquals("", EuclidCoreIOTools.getStringOf(null, null, separator, new boolean[0]));

         String prefix = "!prefix!";
         String suffix = "?suffix?";
         expected = "!prefix!true-p-true-p-true-p-false?suffix?";
         assertEquals(expected, EuclidCoreIOTools.getStringOf(prefix, suffix, separator, values));

         assertEquals("null", EuclidCoreIOTools.getStringOf(prefix, suffix, separator, (boolean[]) null));

         assertEquals("!prefix!?suffix?", EuclidCoreIOTools.getStringOf(prefix, suffix, separator, new boolean[0]));
      }

      { // Test generators for int values
         int[] values = {random.nextInt(1000) - 500, random.nextInt(1000) - 500, random.nextInt(1000) - 500, random.nextInt(1000) - 500};
         expected = "325-p--400-p--255-p--494";
         assertEquals(expected, EuclidCoreIOTools.getStringOf(separator, values));
         assertEquals(expected, EuclidCoreIOTools.getStringOf(null, null, separator, values));

         assertEquals("null", EuclidCoreIOTools.getStringOf(separator, (int[]) null));
         assertEquals("null", EuclidCoreIOTools.getStringOf(null, null, separator, (int[]) null));

         assertEquals("", EuclidCoreIOTools.getStringOf(separator, new int[0]));
         assertEquals("", EuclidCoreIOTools.getStringOf(null, null, separator, new int[0]));

         String prefix = "!prefix!";
         String suffix = "?suffix?";
         expected = "!prefix!325-p--400-p--255-p--494?suffix?";
         assertEquals(expected, EuclidCoreIOTools.getStringOf(prefix, suffix, separator, values));

         assertEquals("null", EuclidCoreIOTools.getStringOf(prefix, suffix, separator, (int[]) null));

         assertEquals("!prefix!?suffix?", EuclidCoreIOTools.getStringOf(prefix, suffix, separator, new int[0]));
      }

      { // Test generators for int values
         long[] values = {random.nextInt(1000) - 500, random.nextInt(1000) - 500, random.nextInt(1000) - 500, random.nextInt(1000) - 500};
         expected = "-107-p-104-p-246-p--373";
         assertEquals(expected, EuclidCoreIOTools.getStringOf(separator, values));
         assertEquals(expected, EuclidCoreIOTools.getStringOf(null, null, separator, values));

         assertEquals("null", EuclidCoreIOTools.getStringOf(separator, (long[]) null));
         assertEquals("null", EuclidCoreIOTools.getStringOf(null, null, separator, (long[]) null));

         assertEquals("", EuclidCoreIOTools.getStringOf(separator, new long[0]));
         assertEquals("", EuclidCoreIOTools.getStringOf(null, null, separator, new long[0]));

         String prefix = "!prefix!";
         String suffix = "?suffix?";
         expected = "!prefix!-107-p-104-p-246-p--373?suffix?";
         assertEquals(expected, EuclidCoreIOTools.getStringOf(prefix, suffix, separator, values));

         assertEquals("null", EuclidCoreIOTools.getStringOf(prefix, suffix, separator, (long[]) null));

         assertEquals("!prefix!?suffix?", EuclidCoreIOTools.getStringOf(prefix, suffix, separator, new long[0]));
      }
   }

   @Test
   public void testGetArrayString()
   {
      Random random = new Random(345345);
      String expected;
      String separator = "-p-";
      TestObject[] array = nextTestObjectArray(random, 3);

      { // Testing without any null value
         expected = "[blop=4, choux=-0.0727027174796302]-p-[blop=-4, choux=0.4845303753357886]-p-[blop=44, choux=0.2874116776971093]";
         assertEquals(expected, EuclidCoreIOTools.getArrayString(separator, array));
         assertEquals(expected, EuclidCoreIOTools.getArrayString(null, null, separator, array));
         assertEquals(expected, EuclidCoreIOTools.getArrayString(separator, array, Object::toString));
         assertEquals(expected, EuclidCoreIOTools.getArrayString(null, null, separator, array, Object::toString));

         String prefix = "!prefix!";
         String suffix = "?suffix?";
         expected = "!prefix![blop=4, choux=-0.0727027174796302]-p-[blop=-4, choux=0.4845303753357886]-p-[blop=44, choux=0.2874116776971093]?suffix?";
         assertEquals(expected, EuclidCoreIOTools.getArrayString(prefix, suffix, separator, array));
         assertEquals(expected, EuclidCoreIOTools.getArrayString(prefix, suffix, separator, array, Object::toString));

         expected = "blop=4-p-blop=-4-p-blop=44";
         assertEquals(expected, EuclidCoreIOTools.getArrayString(separator, array, e -> "blop=" + e.blop));
         assertEquals(expected, EuclidCoreIOTools.getArrayString(null, null, separator, array, e -> "blop=" + e.blop));
      }

      { // Testing with a null value
         array[random.nextInt(array.length)] = null;
         expected = "[blop=4, choux=-0.0727027174796302]-p-[blop=-4, choux=0.4845303753357886]-p-null";
         assertEquals(expected, EuclidCoreIOTools.getArrayString(separator, array));
         assertEquals(expected, EuclidCoreIOTools.getArrayString(null, null, separator, array));
         assertEquals(expected, EuclidCoreIOTools.getArrayString(separator, array, Object::toString));
         assertEquals(expected, EuclidCoreIOTools.getArrayString(null, null, separator, array, Object::toString));

         String prefix = "!prefix!";
         String suffix = "?suffix?";
         expected = "!prefix![blop=4, choux=-0.0727027174796302]-p-[blop=-4, choux=0.4845303753357886]-p-null?suffix?";
         assertEquals(expected, EuclidCoreIOTools.getArrayString(prefix, suffix, separator, array));
         assertEquals(expected, EuclidCoreIOTools.getArrayString(prefix, suffix, separator, array, Object::toString));

         expected = "blop=4-p-blop=-4-p-null";
         assertEquals(expected, EuclidCoreIOTools.getArrayString(separator, array, e -> "blop=" + e.blop));
         assertEquals(expected, EuclidCoreIOTools.getArrayString(null, null, separator, array, e -> "blop=" + e.blop));
      }
   }

   @Test
   public void testGetCollectionString()
   {
      Random random = new Random(345345);
      String expected;
      String separator = "-p-";
      List<TestObject> list = Arrays.asList(nextTestObjectArray(random, 3));

      { // Testing without any null value
         expected = "[blop=4, choux=-0.0727027174796302]-p-[blop=-4, choux=0.4845303753357886]-p-[blop=44, choux=0.2874116776971093]";
         assertEquals(expected, EuclidCoreIOTools.getCollectionString(separator, list));
         assertEquals(expected, EuclidCoreIOTools.getCollectionString(null, null, separator, list));
         assertEquals(expected, EuclidCoreIOTools.getCollectionString(separator, list, Object::toString));
         assertEquals(expected, EuclidCoreIOTools.getCollectionString(null, null, separator, list, Object::toString));

         String prefix = "!prefix!";
         String suffix = "?suffix?";
         expected = "!prefix![blop=4, choux=-0.0727027174796302]-p-[blop=-4, choux=0.4845303753357886]-p-[blop=44, choux=0.2874116776971093]?suffix?";
         assertEquals(expected, EuclidCoreIOTools.getCollectionString(prefix, suffix, separator, list));
         assertEquals(expected, EuclidCoreIOTools.getCollectionString(prefix, suffix, separator, list, Object::toString));

         expected = "blop=4-p-blop=-4-p-blop=44";
         assertEquals(expected, EuclidCoreIOTools.getCollectionString(separator, list, e -> "blop=" + e.blop));
         assertEquals(expected, EuclidCoreIOTools.getCollectionString(null, null, separator, list, e -> "blop=" + e.blop));
      }

      { // Testing with a null value
         list.set(random.nextInt(list.size()), null);
         expected = "[blop=4, choux=-0.0727027174796302]-p-[blop=-4, choux=0.4845303753357886]-p-null";
         assertEquals(expected, EuclidCoreIOTools.getCollectionString(separator, list));
         assertEquals(expected, EuclidCoreIOTools.getCollectionString(null, null, separator, list));
         assertEquals(expected, EuclidCoreIOTools.getCollectionString(separator, list, Object::toString));
         assertEquals(expected, EuclidCoreIOTools.getCollectionString(null, null, separator, list, Object::toString));

         String prefix = "!prefix!";
         String suffix = "?suffix?";
         expected = "!prefix![blop=4, choux=-0.0727027174796302]-p-[blop=-4, choux=0.4845303753357886]-p-null?suffix?";
         assertEquals(expected, EuclidCoreIOTools.getCollectionString(prefix, suffix, separator, list));
         assertEquals(expected, EuclidCoreIOTools.getCollectionString(prefix, suffix, separator, list, Object::toString));

         expected = "blop=4-p-blop=-4-p-null";
         assertEquals(expected, EuclidCoreIOTools.getCollectionString(separator, list, e -> "blop=" + e.blop));
         assertEquals(expected, EuclidCoreIOTools.getCollectionString(null, null, separator, list, e -> "blop=" + e.blop));
      }
   }

   private static TestObject[] nextTestObjectArray(Random random, int length)
   {
      return IntStream.range(0, length).mapToObj(i -> new TestObject(random)).toArray(TestObject[]::new);
   }

   private static class TestObject
   {
      private int blop;
      private double choux;

      public TestObject(Random random)
      {
         blop = random.nextInt(100) - 50;
         choux = random.nextDouble() - 0.5;
      }

      @Override
      public String toString()
      {
         return "[blop=" + blop + ", choux=" + choux + "]";
      }
   }
}
