package us.ihmc.euclid.matrix;

import org.ejml.data.DMatrix;
import org.ejml.data.DMatrixRMaj;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.exceptions.NotAMatrix2DException;
import us.ihmc.euclid.exceptions.NotARotationMatrixException;
import us.ihmc.euclid.exceptions.NotAnOrientation2DException;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tools.Matrix3DFeatures;
import us.ihmc.euclid.tools.Matrix3DTools;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Vector4D;

import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;
import static us.ihmc.euclid.EuclidTestConstants.ITERATIONS;

public abstract class Matrix3DReadOnlyTest<T extends Matrix3DReadOnly>
{
   public static final double SMALL_EPS = 1.0e-12;

   public abstract T createEmptyMatrix();

   public abstract T createMatrix(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22);

   public abstract T createRandomMatrix(Random random);

   @Test
   public void testGettersForIndividualComponents() throws Exception
   {
      Random random = new Random(32424L);

      double m00 = random.nextDouble();
      double m01 = random.nextDouble();
      double m02 = random.nextDouble();
      double m10 = random.nextDouble();
      double m11 = random.nextDouble();
      double m12 = random.nextDouble();
      double m20 = random.nextDouble();
      double m21 = random.nextDouble();
      double m22 = random.nextDouble();
      T matrix = createMatrix(m00, m01, m02, m10, m11, m12, m20, m21, m22);

      assertEquals(m00, matrix.getM00(), SMALL_EPS);
      assertEquals(m01, matrix.getM01(), SMALL_EPS);
      assertEquals(m02, matrix.getM02(), SMALL_EPS);

      assertEquals(m10, matrix.getM10(), SMALL_EPS);
      assertEquals(m11, matrix.getM11(), SMALL_EPS);
      assertEquals(m12, matrix.getM12(), SMALL_EPS);

      assertEquals(m20, matrix.getM20(), SMALL_EPS);
      assertEquals(m21, matrix.getM21(), SMALL_EPS);
      assertEquals(m22, matrix.getM22(), SMALL_EPS);

      assertEquals(m00, matrix.getElement(0, 0), SMALL_EPS);
      assertEquals(m01, matrix.getElement(0, 1), SMALL_EPS);
      assertEquals(m02, matrix.getElement(0, 2), SMALL_EPS);
      assertEquals(m10, matrix.getElement(1, 0), SMALL_EPS);
      assertEquals(m11, matrix.getElement(1, 1), SMALL_EPS);
      assertEquals(m12, matrix.getElement(1, 2), SMALL_EPS);
      assertEquals(m20, matrix.getElement(2, 0), SMALL_EPS);
      assertEquals(m21, matrix.getElement(2, 1), SMALL_EPS);
      assertEquals(m22, matrix.getElement(2, 2), SMALL_EPS);

      for (int j = 0; j < 3; j++)
      {
         final int jFinal = j;
         EuclidCoreTestTools.assertExceptionIsThrown(() -> matrix.getElement(jFinal, 3), ArrayIndexOutOfBoundsException.class);
         EuclidCoreTestTools.assertExceptionIsThrown(() -> matrix.getElement(3, jFinal), ArrayIndexOutOfBoundsException.class);
      }
   }

   @Test
   public void testGetArray() throws Exception
   {
      Random random = new Random(4356L);
      T matrix = createRandomMatrix(random);
      double[] matrixArray = new double[9];
      matrix.get(matrixArray);

      for (int row = 0; row < 3; row++)
      {
         for (int column = 0; column < 3; column++)
         {
            assertTrue(matrix.getElement(row, column) == matrixArray[row * 3 + column]);
         }
      }

      int startIndex = random.nextInt(10);
      matrixArray = new double[9 + startIndex];
      matrix.get(startIndex, matrixArray);

      for (int row = 0; row < 3; row++)
      {
         for (int column = 0; column < 3; column++)
         {
            assertTrue(matrix.getElement(row, column) == matrixArray[row * 3 + column + startIndex]);
         }
      }
   }

   @Test
   public void testGetDenseMatrix() throws Exception
   {
      Random random = new Random(4356L);
      Matrix3D matrix = EuclidCoreRandomTools.nextMatrix3D(random);

      DMatrix denseMatrix = new DMatrixRMaj(3, 3);
      matrix.get(denseMatrix);

      for (int row = 0; row < 3; row++)
      {
         for (int column = 0; column < 3; column++)
         {
            assertTrue(matrix.getElement(row, column) == denseMatrix.get(row, column));
         }
      }

      int startRow = random.nextInt(10);
      int startColumn = random.nextInt(10);
      denseMatrix = new DMatrixRMaj(3 + startRow, 3 + startColumn);
      matrix.get(startRow, startColumn, denseMatrix);

      for (int row = 0; row < 3; row++)
      {
         for (int column = 0; column < 3; column++)
         {
            assertTrue(matrix.getElement(row, column) == denseMatrix.get(row + startRow, column + startColumn));
         }
      }
   }

   @Test
   public void testGetColumn() throws Exception
   {
      Random random = new Random(655L);
      T matrix = createRandomMatrix(random);
      double[] columnArray = new double[3];

      matrix.getColumn(0, columnArray);
      assertTrue(matrix.getM00() == columnArray[0]);
      assertTrue(matrix.getM10() == columnArray[1]);
      assertTrue(matrix.getM20() == columnArray[2]);

      matrix.getColumn(1, columnArray);
      assertTrue(matrix.getM01() == columnArray[0]);
      assertTrue(matrix.getM11() == columnArray[1]);
      assertTrue(matrix.getM21() == columnArray[2]);

      matrix.getColumn(2, columnArray);
      assertTrue(matrix.getM02() == columnArray[0]);
      assertTrue(matrix.getM12() == columnArray[1]);
      assertTrue(matrix.getM22() == columnArray[2]);

      EuclidCoreTestTools.assertExceptionIsThrown(() -> matrix.getColumn(-1, columnArray), ArrayIndexOutOfBoundsException.class);
      EuclidCoreTestTools.assertExceptionIsThrown(() -> matrix.getColumn(3, columnArray), ArrayIndexOutOfBoundsException.class);

      Vector3D columnVector = new Vector3D();
      matrix.getColumn(0, columnVector);
      assertTrue(matrix.getM00() == columnVector.getElement(0));
      assertTrue(matrix.getM10() == columnVector.getElement(1));
      assertTrue(matrix.getM20() == columnVector.getElement(2));

      matrix.getColumn(1, columnVector);
      assertTrue(matrix.getM01() == columnVector.getElement(0));
      assertTrue(matrix.getM11() == columnVector.getElement(1));
      assertTrue(matrix.getM21() == columnVector.getElement(2));

      matrix.getColumn(2, columnVector);
      assertTrue(matrix.getM02() == columnVector.getElement(0));
      assertTrue(matrix.getM12() == columnVector.getElement(1));
      assertTrue(matrix.getM22() == columnVector.getElement(2));

      EuclidCoreTestTools.assertExceptionIsThrown(() -> matrix.getColumn(-1, columnVector), ArrayIndexOutOfBoundsException.class);
      EuclidCoreTestTools.assertExceptionIsThrown(() -> matrix.getColumn(3, columnVector), ArrayIndexOutOfBoundsException.class);
   }

   @Test
   public void testGetRow() throws Exception
   {
      Random random = new Random(6465L);
      Matrix3D matrix = EuclidCoreRandomTools.nextMatrix3D(random);

      double[] rowArray = new double[3];
      matrix.getRow(0, rowArray);
      assertTrue(matrix.getM00() == rowArray[0]);
      assertTrue(matrix.getM01() == rowArray[1]);
      assertTrue(matrix.getM02() == rowArray[2]);

      matrix.getRow(1, rowArray);
      assertTrue(matrix.getM10() == rowArray[0]);
      assertTrue(matrix.getM11() == rowArray[1]);
      assertTrue(matrix.getM12() == rowArray[2]);

      matrix.getRow(2, rowArray);
      assertTrue(matrix.getM20() == rowArray[0]);
      assertTrue(matrix.getM21() == rowArray[1]);
      assertTrue(matrix.getM22() == rowArray[2]);

      EuclidCoreTestTools.assertExceptionIsThrown(() -> matrix.getRow(-1, rowArray), ArrayIndexOutOfBoundsException.class);
      EuclidCoreTestTools.assertExceptionIsThrown(() -> matrix.getRow(3, rowArray), ArrayIndexOutOfBoundsException.class);

      Vector3D rowVector = new Vector3D();
      matrix.getRow(0, rowVector);
      assertTrue(matrix.getM00() == rowVector.getElement(0));
      assertTrue(matrix.getM01() == rowVector.getElement(1));
      assertTrue(matrix.getM02() == rowVector.getElement(2));

      matrix.getRow(1, rowVector);
      assertTrue(matrix.getM10() == rowVector.getElement(0));
      assertTrue(matrix.getM11() == rowVector.getElement(1));
      assertTrue(matrix.getM12() == rowVector.getElement(2));

      matrix.getRow(2, rowVector);
      assertTrue(matrix.getM20() == rowVector.getElement(0));
      assertTrue(matrix.getM21() == rowVector.getElement(1));
      assertTrue(matrix.getM22() == rowVector.getElement(2));

      EuclidCoreTestTools.assertExceptionIsThrown(() -> matrix.getRow(-1, rowVector), ArrayIndexOutOfBoundsException.class);
      EuclidCoreTestTools.assertExceptionIsThrown(() -> matrix.getRow(3, rowVector), ArrayIndexOutOfBoundsException.class);
   }

   @Test
   public void testContainsNaN() throws Exception
   {
      assertFalse(createMatrix(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0).containsNaN());
      assertTrue(createMatrix(Double.NaN, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0).containsNaN());
      assertTrue(createMatrix(0.0, Double.NaN, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0).containsNaN());
      assertTrue(createMatrix(0.0, 0.0, Double.NaN, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0).containsNaN());
      assertTrue(createMatrix(0.0, 0.0, 0.0, Double.NaN, 0.0, 0.0, 0.0, 0.0, 0.0).containsNaN());
      assertTrue(createMatrix(0.0, 0.0, 0.0, 0.0, Double.NaN, 0.0, 0.0, 0.0, 0.0).containsNaN());
      assertTrue(createMatrix(0.0, 0.0, 0.0, 0.0, 0.0, Double.NaN, 0.0, 0.0, 0.0).containsNaN());
      assertTrue(createMatrix(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, Double.NaN, 0.0, 0.0).containsNaN());
      assertTrue(createMatrix(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, Double.NaN, 0.0).containsNaN());
      assertTrue(createMatrix(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, Double.NaN).containsNaN());
   }

   @Test
   public void testDeterminant() throws Exception
   {
      Random random = new Random(51665L);

      T matrix;

      for (int i = 0; i < ITERATIONS; i++)
      {
         matrix = createRandomMatrix(random);
         double actualDeterminant = matrix.determinant();
         double expectedDeterminant = Matrix3DFeatures.determinant(matrix.getM00(),
                                                                   matrix.getM01(),
                                                                   matrix.getM02(),
                                                                   matrix.getM10(),
                                                                   matrix.getM11(),
                                                                   matrix.getM12(),
                                                                   matrix.getM20(),
                                                                   matrix.getM21(),
                                                                   matrix.getM22());
         assertEquals(expectedDeterminant, actualDeterminant, SMALL_EPS);
      }
   }

   @Test
   public void testCheckIfRotationMatrix() throws Exception
   {
      // Let's just do a trivial test here. A more thorough test is done in Matrix3DFeaturesTest

      T matrix1 = createMatrix(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      EuclidCoreTestTools.assertExceptionIsThrown(() -> matrix1.checkIfRotationMatrix(),
                                                  "The matrix is not a rotation matrix: \n" + matrix1.toString(null),
                                                  NotARotationMatrixException.class);

      T matrix2 = createMatrix(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
      matrix2.checkIfRotationMatrix();
   }

   @Test
   public void testCheckIfMatrix2D() throws Exception
   {
      Random random = new Random(3242L);
      // Let's just do a trivial test here. A more thorough test is done in Matrix3DFeaturesTest

      double d = EuclidCoreRandomTools.nextDouble(random, 5.0);
      T matrix1 = createMatrix(0.0, 0.0, d, 0.0, 0.0, d, d, d, d);
      EuclidCoreTestTools.assertExceptionIsThrown(() -> matrix1.checkIfMatrix2D(),
                                                  "The matrix is not in XY plane: \n" + matrix1.toString(null),
                                                  NotAMatrix2DException.class);

      T matrix2 = createMatrix(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
      matrix2.checkIfMatrix2D();
      matrix2 = createMatrix(d, d, 0.0, d, d, 0.0, 0.0, 0.0, 1.0);
      matrix2.checkIfMatrix2D();
   }

   @Test
   public void testIsIdentity() throws Exception
   {
      // Let's just do a trivial test here. A more thorough test is done in Matrix3DFeaturesTest

      T matrix;
      matrix = createMatrix(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
      assertTrue(matrix.isIdentity());
      assertTrue(matrix.isIdentity(SMALL_EPS));

      matrix = createMatrix(1.0, 0.1, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
      assertFalse(matrix.isIdentity());
      assertFalse(matrix.isIdentity(SMALL_EPS));
   }

   @Test
   public void testIsRotationMatrix() throws Exception
   {
      // Let's just do a trivial test here. A more thorough test is done in Matrix3DFeaturesTest

      T matrix;
      matrix = createMatrix(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
      assertTrue(matrix.isRotationMatrix());
      assertTrue(matrix.isRotationMatrix(SMALL_EPS));

      matrix = createMatrix(1.0, 0.1, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
      assertFalse(matrix.isRotationMatrix());
      assertFalse(matrix.isRotationMatrix(SMALL_EPS));
   }

   @Test
   public void testIsMatrix2D() throws Exception
   {
      Random random = new Random(3242L);
      // Let's just do a trivial test here. A more thorough test is done in Matrix3DFeaturesTest

      T matrix;
      double d = EuclidCoreRandomTools.nextDouble(random, 5.0);
      matrix = createMatrix(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
      assertTrue(matrix.isMatrix2D());
      assertTrue(matrix.isMatrix2D(SMALL_EPS));

      matrix = createMatrix(0.0, 0.0, d, 0.0, 0.0, d, d, d, d);
      assertFalse(matrix.isMatrix2D());
      assertFalse(matrix.isMatrix2D(SMALL_EPS));
      matrix = createMatrix(d, d, 0.0, d, d, 0.0, 0.0, 0.0, 1.0);
      assertTrue(matrix.isMatrix2D());
      assertTrue(matrix.isMatrix2D(SMALL_EPS));
   }

   @Test
   public void testMatrixSkewSymmetric() throws Exception
   {
      Random random = new Random(3242L);
      // Let's just do a trivial test here. A more thorough test is done in Matrix3DFeaturesTest

      T matrix;
      double d = EuclidCoreRandomTools.nextDouble(random, 5.0);
      matrix = createMatrix(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
      assertFalse(matrix.isMatrixSkewSymmetric());
      assertFalse(matrix.isMatrixSkewSymmetric(SMALL_EPS));

      matrix = createMatrix(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      assertTrue(matrix.isMatrixSkewSymmetric());
      assertTrue(matrix.isMatrixSkewSymmetric(SMALL_EPS));
      matrix = createMatrix(0.0, d, 0.0, -d, 0.0, 0.0, 0.0, 0.0, 0.0);
      assertTrue(matrix.isMatrixSkewSymmetric());
      assertTrue(matrix.isMatrixSkewSymmetric(SMALL_EPS));
   }

   @Test
   public void testTransform() throws Exception
   {
      Random random = new Random(43535L);

      for (int i = 0; i < ITERATIONS; i++)
      { // transform(Tuple3DBasics tupleToTransform)
         Vector3D original = EuclidCoreRandomTools.nextVector3D(random);
         Vector3D actual = new Vector3D();
         Vector3D expected = new Vector3D();

         T matrix = createRandomMatrix(random);

         Matrix3DTools.transform(matrix, original, expected);
         actual.set(original);
         matrix.transform(actual);
         EuclidCoreTestTools.assertEquals(expected, actual, SMALL_EPS);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // transform(Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
         Vector3D original = EuclidCoreRandomTools.nextVector3D(random);
         Vector3D actual = new Vector3D();
         Vector3D expected = new Vector3D();

         T matrix = createRandomMatrix(random);

         Matrix3DTools.transform(matrix, original, expected);
         matrix.transform(original, actual);
         EuclidCoreTestTools.assertEquals(expected, actual, SMALL_EPS);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // addTransform(Tuple3DBasics tupleToTransform)
         Vector3D original = EuclidCoreRandomTools.nextVector3D(random);
         Vector3D actual = new Vector3D();
         Vector3D expected = new Vector3D();

         T matrix = createRandomMatrix(random);

         expected.set(original);
         Matrix3DTools.addTransform(matrix, original, expected);
         actual.set(original);
         matrix.addTransform(actual);
         EuclidCoreTestTools.assertEquals(expected, actual, SMALL_EPS);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // addTransform(Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
         Vector3D original = EuclidCoreRandomTools.nextVector3D(random);
         Vector3D actual = EuclidCoreRandomTools.nextVector3D(random);
         Vector3D expected = new Vector3D(actual);

         T matrix = createRandomMatrix(random);

         Matrix3DTools.addTransform(matrix, original, expected);
         matrix.addTransform(original, actual);
         EuclidCoreTestTools.assertEquals(expected, actual, SMALL_EPS);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // transform(Tuple2DBasics tupleToTransform)
         Vector2D original = EuclidCoreRandomTools.nextVector2D(random);
         Vector2D actual = new Vector2D();
         Vector2D expected = new Vector2D();

         double yaw = EuclidCoreRandomTools.nextDouble(random, Math.PI);
         double cYaw = EuclidCoreTools.cos(yaw);
         double sYaw = EuclidCoreTools.sin(yaw);

         T matrix = createMatrix(cYaw, -sYaw, 0.0, sYaw, cYaw, 0.0, 0.0, 0.0, 1.0);

         Matrix3DTools.transform(matrix, original, expected, true);
         actual.set(original);
         matrix.transform(actual);
         EuclidCoreTestTools.assertEquals(expected, actual, SMALL_EPS);

         EuclidCoreTestTools.assertExceptionIsThrown(() -> createRandomMatrix(random).transform(actual),
                                                     NotAMatrix2DException.class,
                                                     NotAnOrientation2DException.class);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // transform(Tuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed)
         Vector2D original = EuclidCoreRandomTools.nextVector2D(random);
         Vector2D actual = new Vector2D();
         Vector2D expected = new Vector2D();

         double yaw = EuclidCoreRandomTools.nextDouble(random, Math.PI);
         double cYaw = EuclidCoreTools.cos(yaw);
         double sYaw = EuclidCoreTools.sin(yaw);

         T matrix = createMatrix(cYaw, -sYaw, 0.0, sYaw, cYaw, 0.0, 0.0, 0.0, 1.0);

         Matrix3DTools.transform(matrix, original, expected, true);
         matrix.transform(original, actual);
         EuclidCoreTestTools.assertEquals(expected, actual, SMALL_EPS);

         EuclidCoreTestTools.assertExceptionIsThrown(() -> createRandomMatrix(random).transform(original, actual),
                                                     NotAMatrix2DException.class,
                                                     NotAnOrientation2DException.class);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // transform(Tuple2DBasics tupleToTransform, boolean checkIfTransformInXYPlane)
         Vector2D original = EuclidCoreRandomTools.nextVector2D(random);
         Vector2D actual = new Vector2D();
         Vector2D expected = new Vector2D();

         double yaw = EuclidCoreRandomTools.nextDouble(random, Math.PI);
         double cYaw = EuclidCoreTools.cos(yaw);
         double sYaw = EuclidCoreTools.sin(yaw);

         T matrix = createMatrix(cYaw, -sYaw, 0.0, sYaw, cYaw, 0.0, 0.0, 0.0, 1.0);

         Matrix3DTools.transform(matrix, original, expected, true);
         actual.set(original);
         matrix.transform(actual, true);
         EuclidCoreTestTools.assertEquals(expected, actual, SMALL_EPS);

         EuclidCoreTestTools.assertExceptionIsThrown(() -> createRandomMatrix(random).transform(actual, true),
                                                     NotAMatrix2DException.class,
                                                     NotAnOrientation2DException.class);

         matrix = createRandomMatrix(random);
         Matrix3DTools.transform(matrix, original, expected, false);
         actual.set(original);
         matrix.transform(actual, false);
         EuclidCoreTestTools.assertEquals(expected, actual, SMALL_EPS);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // transform(Tuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed, boolean checkIfTransformInXYPlane)
         Vector2D original = EuclidCoreRandomTools.nextVector2D(random);
         Vector2D actual = new Vector2D();
         Vector2D expected = new Vector2D();

         double yaw = EuclidCoreRandomTools.nextDouble(random, Math.PI);
         double cYaw = EuclidCoreTools.cos(yaw);
         double sYaw = EuclidCoreTools.sin(yaw);

         T matrix = createMatrix(cYaw, -sYaw, 0.0, sYaw, cYaw, 0.0, 0.0, 0.0, 1.0);

         Matrix3DTools.transform(matrix, original, expected, true);
         matrix.transform(original, actual, true);
         EuclidCoreTestTools.assertEquals(expected, actual, SMALL_EPS);

         EuclidCoreTestTools.assertExceptionIsThrown(() -> createRandomMatrix(random).transform(original, actual, true),
                                                     NotAMatrix2DException.class,
                                                     NotAnOrientation2DException.class);

         matrix = createRandomMatrix(random);
         Matrix3DTools.transform(matrix, original, expected, false);
         actual.set(original);
         matrix.transform(original, actual, false);
         EuclidCoreTestTools.assertEquals(expected, actual, SMALL_EPS);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // transform(Matrix3D matrixToTransform)
         Matrix3D original = EuclidCoreRandomTools.nextMatrix3D(random);
         Matrix3D actual = new Matrix3D();
         Matrix3D expected = new Matrix3D();

         T matrix = createRandomMatrix(random);

         Matrix3DTools.transform(matrix, original, expected);
         actual.set(original);
         matrix.transform(actual);
         EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, SMALL_EPS);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // transform(Matrix3DReadOnly matrixOriginal, Matrix3D matrixTransformed)
         Matrix3D original = EuclidCoreRandomTools.nextMatrix3D(random);
         Matrix3D actual = new Matrix3D();
         Matrix3D expected = new Matrix3D();

         T matrix = createRandomMatrix(random);

         Matrix3DTools.transform(matrix, original, expected);
         matrix.transform(original, actual);
         EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, 10.0 * SMALL_EPS);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // transform(Vector4DBasics vectorToTransform)
         Vector4D original = EuclidCoreRandomTools.nextVector4D(random);
         Vector4D actual = new Vector4D();
         Vector4D expected = new Vector4D();

         T matrix = createRandomMatrix(random);

         Matrix3DTools.transform(matrix, original, expected);
         actual.set(original);
         matrix.transform(actual);
         EuclidCoreTestTools.assertEquals(expected, actual, SMALL_EPS);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // transform(Vector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed)
         Vector4D original = EuclidCoreRandomTools.nextVector4D(random);
         Vector4D actual = new Vector4D();
         Vector4D expected = new Vector4D();

         T matrix = createRandomMatrix(random);

         Matrix3DTools.transform(matrix, original, expected);
         matrix.transform(original, actual);
         EuclidCoreTestTools.assertEquals(expected, actual, SMALL_EPS);
      }
   }

   @Test
   public void testInverseTransform() throws Exception
   {
      Random random = new Random(43535L);
      double epsilonMatrix = 1.0e-7;

      for (int i = 0; i < ITERATIONS; i++)
      { // inverseTransform(Tuple3DBasics tupleToTransform)
         Vector3D original = EuclidCoreRandomTools.nextVector3D(random);
         Vector3D actual = new Vector3D();
         Vector3D expected = new Vector3D();

         T matrix = createRandomMatrix(random);

         while (Math.abs(matrix.determinant()) < 1.0e-3)
            matrix = createRandomMatrix(random);

         Matrix3DTools.inverseTransform(matrix, original, expected);
         actual.set(original);
         matrix.inverseTransform(actual);
         EuclidCoreTestTools.assertEquals(expected, actual, SMALL_EPS);

         Matrix3D matrixInverse = new Matrix3D();
         matrixInverse.setAndInvert(matrix);
         matrixInverse.transform(original, expected);
         EuclidCoreTestTools.assertEquals(expected, actual, SMALL_EPS);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // inverseTransform(Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
         Vector3D original = EuclidCoreRandomTools.nextVector3D(random);
         Vector3D actual = new Vector3D();
         Vector3D expected = new Vector3D();

         T matrix = createRandomMatrix(random);

         Matrix3DTools.inverseTransform(matrix, original, expected);
         matrix.inverseTransform(original, actual);
         EuclidCoreTestTools.assertEquals(expected, actual, 10.0 * SMALL_EPS);

         Matrix3D matrixInverse = new Matrix3D();
         matrixInverse.setAndInvert(matrix);
         matrixInverse.transform(original, expected);
         EuclidCoreTestTools.assertEquals(expected, actual, SMALL_EPS);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // inverseTransform(Tuple2DBasics tupleToTransform)
         Vector2D original = EuclidCoreRandomTools.nextVector2D(random);
         Vector2D actual = new Vector2D();
         Vector2D expected = new Vector2D();

         double yaw = EuclidCoreRandomTools.nextDouble(random, Math.PI);
         double cYaw = EuclidCoreTools.cos(yaw);
         double sYaw = EuclidCoreTools.sin(yaw);

         T matrix = createMatrix(cYaw, -sYaw, 0.0, sYaw, cYaw, 0.0, 0.0, 0.0, 1.0);

         Matrix3DTools.inverseTransform(matrix, original, expected, true);
         actual.set(original);
         matrix.inverseTransform(actual);
         EuclidCoreTestTools.assertEquals(expected, actual, SMALL_EPS);

         Matrix3D matrixInverse = new Matrix3D();
         matrixInverse.setAndInvert(matrix);
         matrixInverse.transform(original, expected);
         EuclidCoreTestTools.assertEquals(expected, actual, SMALL_EPS);

         EuclidCoreTestTools.assertExceptionIsThrown(() -> createRandomMatrix(random).inverseTransform(actual),
                                                     NotAMatrix2DException.class,
                                                     NotAnOrientation2DException.class);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // inverseTransform(Tuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed)
         Vector2D original = EuclidCoreRandomTools.nextVector2D(random);
         Vector2D actual = new Vector2D();
         Vector2D expected = new Vector2D();

         double yaw = EuclidCoreRandomTools.nextDouble(random, Math.PI);
         double cYaw = EuclidCoreTools.cos(yaw);
         double sYaw = EuclidCoreTools.sin(yaw);

         T matrix = createMatrix(cYaw, -sYaw, 0.0, sYaw, cYaw, 0.0, 0.0, 0.0, 1.0);

         Matrix3DTools.inverseTransform(matrix, original, expected, true);
         matrix.inverseTransform(original, actual);
         EuclidCoreTestTools.assertEquals(expected, actual, SMALL_EPS);

         Matrix3D matrixInverse = new Matrix3D();
         matrixInverse.setAndInvert(matrix);
         matrixInverse.transform(original, expected);
         EuclidCoreTestTools.assertEquals(expected, actual, SMALL_EPS);

         EuclidCoreTestTools.assertExceptionIsThrown(() -> createRandomMatrix(random).inverseTransform(original, actual),
                                                     NotAMatrix2DException.class,
                                                     NotAnOrientation2DException.class);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // inverseTransform(Tuple2DBasics tupleToTransform, boolean checkIfTransformInXYPlane)
         Vector2D original = EuclidCoreRandomTools.nextVector2D(random);
         Vector2D actual = new Vector2D();
         Vector2D expected = new Vector2D();

         double yaw = EuclidCoreRandomTools.nextDouble(random, Math.PI);
         double cYaw = EuclidCoreTools.cos(yaw);
         double sYaw = EuclidCoreTools.sin(yaw);

         T matrix = createMatrix(cYaw, -sYaw, 0.0, sYaw, cYaw, 0.0, 0.0, 0.0, 1.0);

         Matrix3DTools.inverseTransform(matrix, original, expected, true);
         actual.set(original);
         matrix.inverseTransform(actual, true);
         EuclidCoreTestTools.assertEquals(expected, actual, SMALL_EPS);

         Matrix3D matrixInverse = new Matrix3D();
         matrixInverse.setAndInvert(matrix);
         matrixInverse.transform(original, expected);
         EuclidCoreTestTools.assertEquals(expected, actual, SMALL_EPS);

         EuclidCoreTestTools.assertExceptionIsThrown(() -> createRandomMatrix(random).inverseTransform(actual, true),
                                                     NotAMatrix2DException.class,
                                                     NotAnOrientation2DException.class);

         matrix = createRandomMatrix(random);
         Matrix3DTools.inverseTransform(matrix, original, expected, false);
         actual.set(original);
         matrix.inverseTransform(actual, false);
         EuclidCoreTestTools.assertEquals(expected, actual, SMALL_EPS);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // inverseTransform(Tuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed, boolean checkIfTransformInXYPlane)
         Vector2D original = EuclidCoreRandomTools.nextVector2D(random);
         Vector2D actual = new Vector2D();
         Vector2D expected = new Vector2D();

         double yaw = EuclidCoreRandomTools.nextDouble(random, Math.PI);
         double cYaw = EuclidCoreTools.cos(yaw);
         double sYaw = EuclidCoreTools.sin(yaw);

         T matrix = createMatrix(cYaw, -sYaw, 0.0, sYaw, cYaw, 0.0, 0.0, 0.0, 1.0);

         Matrix3DTools.inverseTransform(matrix, original, expected, true);
         matrix.inverseTransform(original, actual, true);
         EuclidCoreTestTools.assertEquals(expected, actual, SMALL_EPS);

         Matrix3D matrixInverse = new Matrix3D();
         matrixInverse.setAndInvert(matrix);
         matrixInverse.transform(original, expected);
         EuclidCoreTestTools.assertEquals(expected, actual, SMALL_EPS);

         EuclidCoreTestTools.assertExceptionIsThrown(() -> createRandomMatrix(random).inverseTransform(original, actual, true),
                                                     NotAMatrix2DException.class,
                                                     NotAnOrientation2DException.class);

         matrix = createRandomMatrix(random);
         Matrix3DTools.inverseTransform(matrix, original, expected, false);
         actual.set(original);
         matrix.inverseTransform(original, actual, false);
         EuclidCoreTestTools.assertEquals(expected, actual, SMALL_EPS);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // inverseTransform(Matrix3D matrixToTransform)
         Matrix3D original = EuclidCoreRandomTools.nextMatrix3D(random);
         Matrix3D actual = new Matrix3D();
         Matrix3D expected = new Matrix3D();

         T matrix = createRandomMatrix(random);

         while (Math.abs(matrix.determinant()) < 1.0e-3)
            matrix = createRandomMatrix(random);

         Matrix3DTools.inverseTransform(matrix, original, expected);
         actual.set(original);
         matrix.inverseTransform(actual);
         EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, epsilonMatrix);

         Matrix3D matrixInverse = new Matrix3D();
         matrixInverse.setAndInvert(matrix);
         matrixInverse.transform(original, expected);
         EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, epsilonMatrix);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // inverseTransform(Matrix3DReadOnly matrixOriginal, Matrix3D matrixTransformed)
         Matrix3D original = EuclidCoreRandomTools.nextMatrix3D(random);
         Matrix3D actual = new Matrix3D();
         Matrix3D expected = new Matrix3D();

         T matrix = createRandomMatrix(random);

         while (Math.abs(matrix.determinant()) < 1.0e-3)
            matrix = createRandomMatrix(random);

         Matrix3DTools.inverseTransform(matrix, original, expected);
         matrix.inverseTransform(original, actual);
         EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, epsilonMatrix);

         Matrix3D matrixInverse = new Matrix3D();
         matrixInverse.setAndInvert(matrix);
         matrixInverse.transform(original, expected);
         EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, Math.max(1, expected.maxAbsElement()) * epsilonMatrix);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // inverseTransform(Vector4DBasics vectorToTransform)
         Vector4D original = EuclidCoreRandomTools.nextVector4D(random);
         Vector4D actual = new Vector4D();
         Vector4D expected = new Vector4D();

         T matrix = createRandomMatrix(random);

         Matrix3DTools.inverseTransform(matrix, original, expected);
         actual.set(original);
         matrix.inverseTransform(actual);
         EuclidCoreTestTools.assertEquals(expected, actual, SMALL_EPS);

         Matrix3D matrixInverse = new Matrix3D();
         matrixInverse.setAndInvert(matrix);
         matrixInverse.transform(original, expected);
         EuclidCoreTestTools.assertEquals(expected, actual, SMALL_EPS);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // inverseTransform(Vector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed)
         Vector4D original = EuclidCoreRandomTools.nextVector4D(random);
         Vector4D actual = new Vector4D();
         Vector4D expected = new Vector4D();

         T matrix = createRandomMatrix(random);

         Matrix3DTools.inverseTransform(matrix, original, expected);
         matrix.inverseTransform(original, actual);
         EuclidCoreTestTools.assertEquals(expected, actual, SMALL_EPS);

         Matrix3D matrixInverse = new Matrix3D();
         matrixInverse.setAndInvert(matrix);
         matrixInverse.transform(original, expected);
         EuclidCoreTestTools.assertEquals(expected, actual, SMALL_EPS);
      }
   }

   @Test
   public void testEpsilonEquals() throws Exception
   {
      Random random = new Random(621541L);
      double epsilon = random.nextDouble();

      T matrix = createRandomMatrix(random);
      double m00 = matrix.getM00();
      double m01 = matrix.getM01();
      double m02 = matrix.getM02();
      double m10 = matrix.getM10();
      double m11 = matrix.getM11();
      double m12 = matrix.getM12();
      double m20 = matrix.getM20();
      double m21 = matrix.getM21();
      double m22 = matrix.getM22();

      double small = 0.999 * epsilon;
      double big = 1.001 * epsilon;

      assertTrue(matrix.epsilonEquals(createMatrix(m00 + small, m01, m02, m10, m11, m12, m20, m21, m22), epsilon));
      assertTrue(matrix.epsilonEquals(createMatrix(m00, m01 + small, m02, m10, m11, m12, m20, m21, m22), epsilon));
      assertTrue(matrix.epsilonEquals(createMatrix(m00, m01, m02 + small, m10, m11, m12, m20, m21, m22), epsilon));
      assertTrue(matrix.epsilonEquals(createMatrix(m00, m01, m02, m10 + small, m11, m12, m20, m21, m22), epsilon));
      assertTrue(matrix.epsilonEquals(createMatrix(m00, m01, m02, m10, m11 + small, m12, m20, m21, m22), epsilon));
      assertTrue(matrix.epsilonEquals(createMatrix(m00, m01, m02, m10, m11, m12 + small, m20, m21, m22), epsilon));
      assertTrue(matrix.epsilonEquals(createMatrix(m00, m01, m02, m10, m11, m12, m20 + small, m21, m22), epsilon));
      assertTrue(matrix.epsilonEquals(createMatrix(m00, m01, m02, m10, m11, m12, m20, m21 + small, m22), epsilon));
      assertTrue(matrix.epsilonEquals(createMatrix(m00, m01, m02, m10, m11, m12, m20, m21, m22 + small), epsilon));

      assertTrue(matrix.epsilonEquals(createMatrix(m00 - small, m01, m02, m10, m11, m12, m20, m21, m22), epsilon));
      assertTrue(matrix.epsilonEquals(createMatrix(m00, m01 - small, m02, m10, m11, m12, m20, m21, m22), epsilon));
      assertTrue(matrix.epsilonEquals(createMatrix(m00, m01, m02 - small, m10, m11, m12, m20, m21, m22), epsilon));
      assertTrue(matrix.epsilonEquals(createMatrix(m00, m01, m02, m10 - small, m11, m12, m20, m21, m22), epsilon));
      assertTrue(matrix.epsilonEquals(createMatrix(m00, m01, m02, m10, m11 - small, m12, m20, m21, m22), epsilon));
      assertTrue(matrix.epsilonEquals(createMatrix(m00, m01, m02, m10, m11, m12 - small, m20, m21, m22), epsilon));
      assertTrue(matrix.epsilonEquals(createMatrix(m00, m01, m02, m10, m11, m12, m20 - small, m21, m22), epsilon));
      assertTrue(matrix.epsilonEquals(createMatrix(m00, m01, m02, m10, m11, m12, m20, m21 - small, m22), epsilon));
      assertTrue(matrix.epsilonEquals(createMatrix(m00, m01, m02, m10, m11, m12, m20, m21, m22 - small), epsilon));

      assertFalse(matrix.epsilonEquals(createMatrix(m00 + big, m01, m02, m10, m11, m12, m20, m21, m22), epsilon));
      assertFalse(matrix.epsilonEquals(createMatrix(m00, m01 + big, m02, m10, m11, m12, m20, m21, m22), epsilon));
      assertFalse(matrix.epsilonEquals(createMatrix(m00, m01, m02 + big, m10, m11, m12, m20, m21, m22), epsilon));
      assertFalse(matrix.epsilonEquals(createMatrix(m00, m01, m02, m10 + big, m11, m12, m20, m21, m22), epsilon));
      assertFalse(matrix.epsilonEquals(createMatrix(m00, m01, m02, m10, m11 + big, m12, m20, m21, m22), epsilon));
      assertFalse(matrix.epsilonEquals(createMatrix(m00, m01, m02, m10, m11, m12 + big, m20, m21, m22), epsilon));
      assertFalse(matrix.epsilonEquals(createMatrix(m00, m01, m02, m10, m11, m12, m20 + big, m21, m22), epsilon));
      assertFalse(matrix.epsilonEquals(createMatrix(m00, m01, m02, m10, m11, m12, m20, m21 + big, m22), epsilon));
      assertFalse(matrix.epsilonEquals(createMatrix(m00, m01, m02, m10, m11, m12, m20, m21, m22 + big), epsilon));

      assertFalse(matrix.epsilonEquals(createMatrix(m00 - big, m01, m02, m10, m11, m12, m20, m21, m22), epsilon));
      assertFalse(matrix.epsilonEquals(createMatrix(m00, m01 - big, m02, m10, m11, m12, m20, m21, m22), epsilon));
      assertFalse(matrix.epsilonEquals(createMatrix(m00, m01, m02 - big, m10, m11, m12, m20, m21, m22), epsilon));
      assertFalse(matrix.epsilonEquals(createMatrix(m00, m01, m02, m10 - big, m11, m12, m20, m21, m22), epsilon));
      assertFalse(matrix.epsilonEquals(createMatrix(m00, m01, m02, m10, m11 - big, m12, m20, m21, m22), epsilon));
      assertFalse(matrix.epsilonEquals(createMatrix(m00, m01, m02, m10, m11, m12 - big, m20, m21, m22), epsilon));
      assertFalse(matrix.epsilonEquals(createMatrix(m00, m01, m02, m10, m11, m12, m20 - big, m21, m22), epsilon));
      assertFalse(matrix.epsilonEquals(createMatrix(m00, m01, m02, m10, m11, m12, m20, m21 - big, m22), epsilon));
      assertFalse(matrix.epsilonEquals(createMatrix(m00, m01, m02, m10, m11, m12, m20, m21, m22 - big), epsilon));
   }
}