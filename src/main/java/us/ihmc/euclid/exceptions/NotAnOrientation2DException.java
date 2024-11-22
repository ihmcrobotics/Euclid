package us.ihmc.euclid.exceptions;

import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;

/**
 * {@code RuntimeException} dedicated to operations with orientations in the XY-plane. It is thrown
 * when an orientation refers to a complete 3D operation making impossible the operation with a 2D
 * object restricted to the xy-plane.
 *
 * @author Sylvain Bertrand
 */
public class NotAnOrientation2DException extends RuntimeException
{
   private static final long serialVersionUID = -2112810108449410621L;

   /**
    * Constructs an {@code NotOrientation2DException} with no detail message.
    */
   public NotAnOrientation2DException()
   {
      super();
   }

   /**
    * Constructs an {@code NotOrientation2DException} with the specified detail message.
    *
    * @param message the detail message.
    */
   public NotAnOrientation2DException(String message)
   {
      super(message);
   }

   /**
    * Constructs an {@code NotOrientation2DException} with a default detail message outputting the
    * given matrix coefficients.
    *
    * @param orientation3DReadOnly the orientation to be displayed in the detail message. Not modified.
    */
   public NotAnOrientation2DException(Orientation3DReadOnly orientation3DReadOnly)
   {
      super("The orientation is not in XY plane: \n" + orientation3DReadOnly.toString(null));
   }
}
