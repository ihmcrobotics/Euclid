package us.ihmc.euclid.exceptions;

import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreIOTools;

/**
 * {@code RuntimeException} dedicated to matrix operations in the xy-plane. It is thrown when a
 * transformation matrix refers to a complete 3D operation making impossible the operation with a 2D
 * object restricted to the xy-plane.
 *
 * @author Sylvain Bertrand
 */
public class NotAMatrix2DException extends RuntimeException
{
   private static final long serialVersionUID = 8769481575502495447L;

   /**
    * Constructs an {@code NotAMatrix2DException} with no detail message.
    */
   public NotAMatrix2DException()
   {
      super();
   }

   /**
    * Constructs an {@code NotAMatrix2DException} with the specified detail message.
    *
    * @param message the detail message.
    */
   public NotAMatrix2DException(String message)
   {
      super(message);
   }

   /**
    * Constructs an {@code NotAMatrix2DException} with a default detail message outputting the given
    * matrix coefficients.
    *
    * @param matrix the matrix to be displayed in the detail message. Not modified.
    */
   public NotAMatrix2DException(Matrix3DReadOnly matrix)
   {
      super("The matrix is not in XY plane: \n" + matrix.toString(null));
   }

   /**
    * Constructs an {@code NotAMatrix2DException} with a default detail message outputting the given
    * matrix coefficients.
    *
    * @param m00 the 1st row 1st column coefficient of the matrix to be displayed in the detail
    *            message.
    * @param m01 the 1st row 2nd column coefficient of the matrix to be displayed in the detail
    *            message.
    * @param m02 the 1st row 3rd column coefficient of the matrix to be displayed in the detail
    *            message.
    * @param m10 the 2nd row 1st column coefficient of the matrix to be displayed in the detail
    *            message.
    * @param m11 the 2nd row 2nd column coefficient of the matrix to be displayed in the detail
    *            message.
    * @param m12 the 2nd row 3rd column coefficient of the matrix to be displayed in the detail
    *            message.
    * @param m20 the 3rd row 1st column coefficient of the matrix to be displayed in the detail
    *            message.
    * @param m21 the 3rd row 2nd column coefficient of the matrix to be displayed in the detail
    *            message.
    * @param m22 the 3rd row 3rd column coefficient of the matrix to be displayed in the detail
    *            message.
    */
   public NotAMatrix2DException(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22)
   {
      super("The matrix is not in XY plane: \n" + EuclidCoreIOTools.getMatrix3DString(null, m00, m01, m02, m10, m11, m12, m20, m21, m22));
   }
}
