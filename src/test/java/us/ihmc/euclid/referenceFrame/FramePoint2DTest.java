package us.ihmc.euclid.referenceFrame;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.fail;
import static us.ihmc.euclid.EuclidTestConstants.ITERATIONS;

import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.function.Predicate;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.EuclidTestConstants;
import us.ihmc.euclid.referenceFrame.api.EuclidFrameAPIDefaultConfiguration;
import us.ihmc.euclid.referenceFrame.api.EuclidFrameAPITester;
import us.ihmc.euclid.referenceFrame.api.MethodSignature;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple2DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameTestTools;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.euclid.tuple3D.Point3D;

public class FramePoint2DTest extends FrameTuple2DBasicsTest<FramePoint2D>
{
   public static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   @Override
   public Tuple2DBasics createRandomFramelessTuple(Random random)
   {
      return EuclidCoreRandomTools.nextPoint2D(random);
   }

   @Override
   public FramePoint2D createFrameTuple(ReferenceFrame referenceFrame, double x, double y)
   {
      return new FramePoint2D(referenceFrame, x, y);
   }

   @Test
   @Override
   public void testConsistencyWithTuple2D() throws Exception
   {
      // TODO Auto-generated method stub
      super.testConsistencyWithTuple2D();
   }

   @Test
   public void testConstructors() throws Exception
   {
      Random random = new Random(435345);

      { // Test FramePoint2D()
         FramePoint2D framePoint2D = new FramePoint2D();
         assertTrue(framePoint2D.getReferenceFrame() == worldFrame);
         EuclidCoreTestTools.assertTuple2DIsSetToZero(framePoint2D);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test FramePoint2D(ReferenceFrame referenceFrame)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         FramePoint2D framePoint2D = new FramePoint2D(randomFrame);
         assertTrue(framePoint2D.getReferenceFrame() == randomFrame);
         EuclidCoreTestTools.assertTuple2DIsSetToZero(framePoint2D);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test FramePoint2D(ReferenceFrame referenceFrame, double x, double y)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         Point2D randomTuple = EuclidCoreRandomTools.nextPoint2D(random);
         FramePoint2D framePoint2D = new FramePoint2D(randomFrame, randomTuple.getX(), randomTuple.getY());
         assertTrue(framePoint2D.getReferenceFrame() == randomFrame);
         EuclidCoreTestTools.assertEquals(randomTuple, framePoint2D, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test FramePoint2D(ReferenceFrame referenceFrame, double[] pointArray)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         Point2D randomTuple = EuclidCoreRandomTools.nextPoint2D(random);
         double[] array = new double[3];
         randomTuple.get(array);
         FramePoint2D framePoint2D = new FramePoint2D(randomFrame, array);
         assertTrue(framePoint2D.getReferenceFrame() == randomFrame);
         EuclidCoreTestTools.assertEquals(randomTuple, framePoint2D, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test FramePoint2D(ReferenceFrame referenceFrame, Tuple2DReadOnly tuple2DReadOnly)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         Point2D randomTuple = EuclidCoreRandomTools.nextPoint2D(random);
         FramePoint2D framePoint2D = new FramePoint2D(randomFrame, randomTuple);
         assertTrue(framePoint2D.getReferenceFrame() == randomFrame);
         EuclidCoreTestTools.assertEquals(randomTuple, framePoint2D, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test FramePoint2D(ReferenceFrame referenceFrame, Tuple3DReadOnly tuple3DReadOnly)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         Point3D randomTuple3D = EuclidCoreRandomTools.nextPoint3D(random);
         FramePoint2D framePoint2D = new FramePoint2D(randomFrame, randomTuple3D);
         assertTrue(framePoint2D.getReferenceFrame() == randomFrame);
         EuclidCoreTestTools.assertEquals(new Point2D(randomTuple3D), framePoint2D, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test FramePoint3D(FrameTuple3DReadOnly frameTuple3DReadOnly)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         FramePoint3D randomFrameTuple3D = EuclidFrameRandomTools.nextFramePoint3D(random, randomFrame);
         FramePoint2D framePoint2D = new FramePoint2D(randomFrameTuple3D);
         assertTrue(framePoint2D.getReferenceFrame() == randomFrame);
         EuclidCoreTestTools.assertEquals(new Point2D(randomFrameTuple3D), framePoint2D, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test FramePoint2D(FrameTuple2DReadOnly other)
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         FramePoint2D randomTuple = EuclidFrameRandomTools.nextFramePoint2D(random, randomFrame);
         FramePoint2D framePoint2D = new FramePoint2D(randomTuple);
         assertTrue(framePoint2D.getReferenceFrame() == randomFrame);
         EuclidCoreTestTools.assertEquals(randomTuple, framePoint2D, EPSILON);
         EuclidFrameTestTools.assertEquals(randomTuple, framePoint2D, EPSILON);
      }
   }

   @Test
   public void testSetMatchingFrame() throws Exception
   {
      EuclidFrameAPITester tester = new EuclidFrameAPITester(new EuclidFrameAPIDefaultConfiguration());
      tester.assertSetMatchingFramePreserveFunctionality(EuclidFrameRandomTools::nextFramePoint2D, EuclidTestConstants.API_FUNCTIONALITY_TEST_ITERATIONS);

      Random random = new Random(544354);

      for (int i = 0; i < ITERATIONS; i++)
      { // Test setMatchingFrame(FrameTuple2DReadOnly other)
         ReferenceFrame sourceFrame = EuclidFrameRandomTools.nextReferenceFrame(random, true);
         ReferenceFrame destinationFrame = EuclidFrameRandomTools.nextReferenceFrame(random, true);

         FrameTuple2DReadOnly source = EuclidFrameRandomTools.nextFramePoint2D(random, sourceFrame);
         FramePoint2D actual = createEmptyFrameTuple(destinationFrame);

         actual.setMatchingFrame(source);

         FramePoint2D expected = new FramePoint2D(source);
         expected.changeFrame(destinationFrame);

         EuclidFrameTestTools.assertEquals(expected, actual, EPSILON);
      }
      
      for (int i = 0; i < ITERATIONS; i++)
      { // Test setMatchingFrame(ReferenceFrame referenceFrame, double x, double y)
         ReferenceFrame sourceFrame = EuclidFrameRandomTools.nextReferenceFrame(random, true);
         ReferenceFrame destinationFrame = EuclidFrameRandomTools.nextReferenceFrame(random, true);

         double x = EuclidCoreRandomTools.nextDouble(random);
         double y = EuclidCoreRandomTools.nextDouble(random);
         FramePoint2D actual = createEmptyFrameTuple(destinationFrame);

         actual.setMatchingFrame(sourceFrame, x, y);

         FramePoint2D expected = new FramePoint2D();
         expected.setIncludingFrame(sourceFrame, x, y);
         expected.changeFrame(destinationFrame);

         EuclidFrameTestTools.assertEquals(expected, actual, EPSILON);
      }
   }

   @Test
   public void testChangeFrame() throws Exception
   {
      Random random = new Random(43563);

      for (int i = 0; i < ITERATIONS; i++)
      {
         ReferenceFrame[] referenceFrames = EuclidFrameRandomTools.nextReferenceFrameTree(random, true);
         ReferenceFrame initialFrame = referenceFrames[random.nextInt(referenceFrames.length)];
         ReferenceFrame anotherFrame = referenceFrames[random.nextInt(referenceFrames.length)];

         Point2D expected = EuclidCoreRandomTools.nextPoint2D(random);
         FramePoint2D actual = new FramePoint2D(initialFrame, expected);

         RigidBodyTransform transform = initialFrame.getTransformToDesiredFrame(anotherFrame);
         expected.applyTransform(transform);

         actual.changeFrame(anotherFrame);
         assertTrue(anotherFrame == actual.getReferenceFrame());
         EuclidCoreTestTools.assertEquals(expected, actual, EPSILON);

         ReferenceFrame differentRootFrame = ReferenceFrameTools.constructARootFrame("anotherRootFrame");
         try
         {
            actual.changeFrame(differentRootFrame);
            fail("Should have thrown a RuntimeException");
         }
         catch (RuntimeException e)
         {
            // good
         }
      }
   }

   @Test
   public void testChangeFrameAndProjectToXYPlane() throws Exception
   {
      Random random = new Random(345345);

      for (int i = 0; i < ITERATIONS; i++)
      {
         ReferenceFrame initialFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
         ReferenceFrame newFrame = EuclidFrameRandomTools.nextReferenceFrame(random);

         Point2D expected = EuclidCoreRandomTools.nextPoint2D(random);
         FramePoint2D actual = new FramePoint2D(initialFrame, expected);

         expected.applyTransform(initialFrame.getTransformToDesiredFrame(newFrame), false);
         actual.changeFrameAndProjectToXYPlane(newFrame);

         assertTrue(newFrame == actual.getReferenceFrame());
         EuclidCoreTestTools.assertEquals(expected, actual, EPSILON);
      }
   }

   @Test
   public void testSetFromReferenceFrame() throws Exception
   {
      Random random = new Random(6572);

      for (int i = 0; i < ITERATIONS; i++)
      {
         ReferenceFrame[] referenceFrames = EuclidFrameRandomTools.nextReferenceFrameTree(random, true);
         ReferenceFrame initialFrame = referenceFrames[random.nextInt(referenceFrames.length)];
         ReferenceFrame anotherFrame = referenceFrames[random.nextInt(referenceFrames.length)];

         FramePoint2D expected = createEmptyFrameTuple(anotherFrame);
         expected.changeFrame(initialFrame);

         FramePoint2D actual = createRandomFrameTuple(random, initialFrame);
         actual.setFromReferenceFrame(anotherFrame);
         assertTrue(initialFrame == actual.getReferenceFrame());
         EuclidCoreTestTools.assertEquals(expected, actual, EPSILON);
      }
   }

   @Test
   public void testGeometricallyEquals() throws Exception
   {
      Random random = new Random(32120);

      for (int i = 0; i < ITERATIONS; i++)
      {
         FramePoint2D framePoint1 = EuclidFrameRandomTools.nextFramePoint2D(random, worldFrame);
         FramePoint2D framePoint2 = new FramePoint2D(worldFrame);
         double epsilon = random.nextDouble();
         Vector2D difference;

         difference = EuclidCoreRandomTools.nextVector2DWithFixedLength(random, 0.99 * epsilon);
         framePoint2.add(framePoint1, difference);
         assertTrue(framePoint1.geometricallyEquals(framePoint2, epsilon));

         difference = EuclidCoreRandomTools.nextVector2DWithFixedLength(random, 1.01 * epsilon);
         framePoint2.add(framePoint1, difference);
         assertFalse(framePoint1.geometricallyEquals(framePoint2, epsilon));
      }
   }

   @Test
   public void testHashCode() throws Exception
   {
      Random random = new Random(621541L);
      ReferenceFrame[] frames = EuclidFrameRandomTools.nextReferenceFrameTree(random, 100);

      FramePoint2D tuple = new FramePoint2D();

      tuple.setX(random.nextDouble());
      tuple.setY(random.nextDouble());
      tuple.setReferenceFrame(frames[random.nextInt(frames.length)]);

      int newHashCode, previousHashCode;
      newHashCode = tuple.hashCode();
      assertEquals(newHashCode, tuple.hashCode());

      previousHashCode = tuple.hashCode();

      for (int i = 0; i < ITERATIONS; i++)
      {
         tuple.setElement(i % 2, random.nextDouble());
         newHashCode = tuple.hashCode();
         assertNotEquals(newHashCode, previousHashCode);
         previousHashCode = newHashCode;

         ReferenceFrame oldFrame = tuple.getReferenceFrame();
         ReferenceFrame newFrame = frames[random.nextInt(frames.length)];
         tuple.setReferenceFrame(newFrame);
         newHashCode = tuple.hashCode();
         if (oldFrame != newFrame)
            assertNotEquals(newHashCode, previousHashCode);
         previousHashCode = newHashCode;
      }
   }

   @Override
   public void testOverloading() throws Exception
   {
      super.testOverloading();
      List<MethodSignature> signaturesToIgnore = new ArrayList<>();
      signaturesToIgnore.add(new MethodSignature("set", Point2D.class));
      signaturesToIgnore.add(new MethodSignature("epsilonEquals", Point2D.class, Double.TYPE));
      signaturesToIgnore.add(new MethodSignature("geometricallyEquals", Point2D.class, Double.TYPE));
      Predicate<Method> methodFilter = EuclidFrameAPITester.methodFilterFromSignature(signaturesToIgnore);

      EuclidFrameAPITester tester = new EuclidFrameAPITester(new EuclidFrameAPIDefaultConfiguration());
      tester.assertOverloadingWithFrameObjects(FramePoint2D.class, Point2D.class, true, 1, methodFilter);
   }

   @Override
   @Test
   public void testSetIncludingFrame()
   {
      EuclidFrameAPITester tester = new EuclidFrameAPITester(new EuclidFrameAPIDefaultConfiguration());
      tester.assertSetIncludingFramePreserveFunctionality(EuclidFrameRandomTools::nextFramePoint2D, EuclidTestConstants.API_FUNCTIONALITY_TEST_ITERATIONS);
   }
}
