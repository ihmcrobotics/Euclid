package us.ihmc.euclid.tools;

import us.ihmc.euclid.axisAngle.interfaces.AxisAngleReadOnly;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation2DReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.transform.QuaternionBasedTransform;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.AffineTransformReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.Tuple4DReadOnly;
import us.ihmc.euclid.yawPitchRoll.interfaces.YawPitchRollReadOnly;

import java.util.Arrays;
import java.util.Collection;
import java.util.Iterator;
import java.util.function.Function;

/**
 * {@code EuclidCoreIOTools} is intended to gather the input & output tools for printing, saving,
 * and loading geometry objects.
 * <p>
 * At this time, only a few print tools are offered, additional features will come in future
 * releases.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public class EuclidCoreIOTools
{
   /** Default format used to print decimal numbers. */
   public static final String DEFAULT_FORMAT = getStringFormat(6, 3);

   private EuclidCoreIOTools()
   {
      // Suppresses default constructor, ensuring non-instantiability.
   }

   /**
    * Gets a representative {@code String} of {@code rigidBodyTransform} as follows:
    *
    * <pre>
    *  0.596  0.630  0.930 | -0.435
    * -0.264  0.763  0.575 | -0.464
    * -0.430 -0.188 -0.048 |  0.611
    *  0.000  0.000  0.000 |  1.000
    * </pre>
    *
    * @param rigidBodyTransform the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getRigidBodyTransformString(RigidBodyTransform rigidBodyTransform)
   {
      return getRigidBodyTransformString(DEFAULT_FORMAT, rigidBodyTransform);
   }

   /**
    * Gets a representative {@code String} of {@code rigidBodyTransform} given a specific format to
    * use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    *
    * <pre>
    *  0.596  0.630  0.930 | -0.435
    * -0.264  0.763  0.575 | -0.464
    * -0.430 -0.188 -0.048 |  0.611
    *  0.000  0.000  0.000 |  1.000
    * </pre>
    * </p>
    *
    * @param format             the format to use for each number.
    * @param rigidBodyTransform the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getRigidBodyTransformString(String format, RigidBodyTransform rigidBodyTransform)
   {
      if (rigidBodyTransform == null)
         return "null";
      return getHomogeneousTransformString(format, rigidBodyTransform.getRotation(), rigidBodyTransform.getTranslation());
   }

   /**
    * Gets a representative {@code String} of {@code affineTransform} as follows:
    *
    * <pre>
    *  0.596  0.630  0.930 | -0.435
    * -0.264  0.763  0.575 | -0.464
    * -0.430 -0.188 -0.048 |  0.611
    *  0.000  0.000  0.000 |  1.000
    * </pre>
    *
    * @param affineTransform the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getAffineTransformString(AffineTransformReadOnly affineTransform)
   {
      return getAffineTransformString(DEFAULT_FORMAT, affineTransform);
   }

   /**
    * Gets a representative {@code String} of {@code affineTransform} given a specific format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    *
    * <pre>
    *  0.596  0.630  0.930 | -0.435
    * -0.264  0.763  0.575 | -0.464
    * -0.430 -0.188 -0.048 |  0.611
    *  0.000  0.000  0.000 |  1.000
    * </pre>
    * </p>
    *
    * @param format          the format to use for each number.
    * @param affineTransform the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getAffineTransformString(String format, AffineTransformReadOnly affineTransform)
   {
      if (affineTransform == null)
         return "null";
      else
         return getHomogeneousTransformString(format, affineTransform.getLinearTransform(), affineTransform.getTranslation());
   }

   private static String getHomogeneousTransformString(String format, Matrix3DReadOnly matrix, Tuple3DReadOnly translation)
   {
      String ret = "";

      for (int i = 0; i < 3; i++)
      {
         ret += getStringOf(null, " ", " ", format, matrix.getElement(i, 0), matrix.getElement(i, 1), matrix.getElement(i, 2));
         ret += "| " + toString(format, translation.getElement(i)) + "\n";
      }

      ret += getStringOf(null, " ", " ", format, 0.0, 0.0, 0.0);
      ret += "| " + toString(format, 1.0);

      return ret;
   }

   /**
    * Gets a representative {@code String} of {@code quaternionBasedTransform} as follows:
    *
    * <pre>
    * Quaternion:  ( 0.174,  0.732, -0.222,  0.620 )
    * Translation: (-0.558, -0.380,  0.130 )
    * </pre>
    *
    * @param quaternionBasedTransform the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getQuaternionBasedTransformString(QuaternionBasedTransform quaternionBasedTransform)
   {
      return getQuaternionBasedTransformString(DEFAULT_FORMAT, quaternionBasedTransform);
   }

   /**
    * Gets a representative {@code String} of {@code quaternionBasedTransform} given a specific format
    * to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    *
    * <pre>
    * Quaternion:  ( 0.174,  0.732, -0.222,  0.620 )
    * Translation: (-0.558, -0.380,  0.130 )
    * </pre>
    * </p>
    *
    * @param format                   the format to use for each number.
    * @param quaternionBasedTransform the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getQuaternionBasedTransformString(String format, QuaternionBasedTransform quaternionBasedTransform)
   {
      if (quaternionBasedTransform == null)
         return "null";
      return getQuaternionBasedTransformString(format, quaternionBasedTransform.getRotation(), quaternionBasedTransform.getTranslation());
   }

   private static String getQuaternionBasedTransformString(String format, QuaternionReadOnly quaternion, Tuple3DReadOnly translation)
   {
      if (quaternion == null)
         return "null";

      String ret = "";

      ret += getStringOf("Quaternion:  (", " )\n", ", ", format, quaternion.getX(), quaternion.getY(), quaternion.getZ(), quaternion.getS());
      ret += getStringOf("Translation: (", " )", ", ", format, translation.getX(), translation.getY(), translation.getZ());
      return ret;
   }

   /**
    * Gets a representative {@code String} of {@code tuple} as follows:
    *
    * <pre>
    * (-0.675, -0.102 )
    * </pre>
    *
    * @param tuple the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getTuple2DString(Tuple2DReadOnly tuple)
   {
      return getTuple2DString(DEFAULT_FORMAT, tuple);
   }

   /**
    * Gets a representative {@code String} of {@code tuple} given a specific format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    *
    * <pre>
    * (-0.675, -0.102 )
    * </pre>
    * </p>
    *
    * @param format the format to use for each number.
    * @param tuple  the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getTuple2DString(String format, Tuple2DReadOnly tuple)
   {
      if (tuple == null)
         return "null";
      else
         return getStringOf("(", " )", ", ", format, tuple.getX(), tuple.getY());
   }

   /**
    * Gets a representative {@code String} of {@code tuple} as follows:
    *
    * <pre>
    * (-0.558, -0.380,  0.130 )
    * </pre>
    *
    * @param tuple the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getTuple3DString(Tuple3DReadOnly tuple)
   {
      return getTuple3DString(DEFAULT_FORMAT, tuple);
   }

   /**
    * Gets a representative {@code String} of {@code tuple} given a specific format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    *
    * <pre>
    * (-0.558, -0.380,  0.130 )
    * </pre>
    * </p>
    *
    * @param format the format to use for each number.
    * @param tuple  the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getTuple3DString(String format, Tuple3DReadOnly tuple)
   {
      if (tuple == null)
         return "null";
      else
         return getStringOf("(", " )", ", ", format, tuple.getX(), tuple.getY(), tuple.getZ());
   }

   /**
    * Gets a representative {@code String} of {@code tuple} as follows:
    *
    * <pre>
    * (-0.052, -0.173, -0.371,  0.087 )
    * </pre>
    *
    * @param tuple the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getTuple4DString(Tuple4DReadOnly tuple)
   {
      return getTuple4DString(DEFAULT_FORMAT, tuple);
   }

   /**
    * Gets a representative {@code String} of {@code tuple} given a specific format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    *
    * <pre>
    * (-0.052, -0.173, -0.371,  0.087 )
    * </pre>
    * </p>
    *
    * @param format the format to use for each number.
    * @param tuple  the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getTuple4DString(String format, Tuple4DReadOnly tuple)
   {
      if (tuple == null)
         return "null";
      else
         return getStringOf("(", " )", ", ", format, tuple.getX(), tuple.getY(), tuple.getZ(), tuple.getS());
   }

   /**
    * Gets a representative {@code String} of {@code axisAngle} as follows:
    *
    * <pre>
    * ( 0.674,  0.455,  0.582,  0.593 )
    * </pre>
    *
    * @param axisAngle the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getAxisAngleString(AxisAngleReadOnly axisAngle)
   {
      return getAxisAngleString(DEFAULT_FORMAT, axisAngle);
   }

   /**
    * Gets a representative {@code String} of {@code axisAngle} given a specific format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    *
    * <pre>
    * ( 0.674,  0.455,  0.582,  0.593 )
    * </pre>
    * </p>
    *
    * @param format    the format to use for each number.
    * @param axisAngle the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getAxisAngleString(String format, AxisAngleReadOnly axisAngle)
   {
      if (axisAngle == null)
         return "null";
      else
         return getStringOf("(", " )", ", ", format, axisAngle.getX(), axisAngle.getY(), axisAngle.getZ(), axisAngle.getAngle());
   }

   /**
    * Gets a representative {@code String} of {@code orientation2D} as follows:
    *
    * <pre>
    * (0.174)
    * </pre>
    *
    * @param orientation2D the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getOrientation2DString(Orientation2DReadOnly orientation2D)
   {
      return getOrientation2DString(DEFAULT_FORMAT, orientation2D);
   }

   /**
    * Gets a representative {@code String} of {@code orientation2D} given a specific format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    *
    * <pre>
    * (0.174)
    * </pre>
    * </p>
    *
    * @param format        the format to use for each number.
    * @param orientation2D the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getOrientation2DString(String format, Orientation2DReadOnly orientation2D)
   {
      if (orientation2D == null)
         return "null";
      else
         return getOrientation2DString(format, orientation2D.getYaw());
   }

   /**
    * Gets a representative {@code String} of {@code orientation2DAngle} given a specific format to
    * use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    *
    * <pre>
    * (0.174)
    * </pre>
    * </p>
    *
    * @param format             the format to use for each number.
    * @param orientation2DAngle the angle of the orientation 2D to get the {@code String} of. Not
    *                           modified.
    * @return the representative {@code String}.
    */
   public static String getOrientation2DString(String format, double orientation2DAngle)
   {
      return getStringOf("(", " )", ", ", format, orientation2DAngle);
   }

   /**
    * Gets a representative {@code String} of {@code matrix} as follows:
    *
    * <pre>
    * /-0.576, -0.784,  0.949 \
    * | 0.649, -0.542, -0.941 |
    * \-0.486, -0.502, -0.619 /
    * </pre>
    *
    * @param matrix the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getMatrix3DString(Matrix3DReadOnly matrix)
   {
      return getMatrix3DString(DEFAULT_FORMAT, matrix);
   }

   /**
    * Gets a representative {@code String} of {@code matrix} given a specific format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    *
    * <pre>
    * /-0.576, -0.784,  0.949 \
    * | 0.649, -0.542, -0.941 |
    * \-0.486, -0.502, -0.619 /
    * </pre>
    * </p>
    *
    * @param format the format to use for each number.
    * @param matrix the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getMatrix3DString(String format, Matrix3DReadOnly matrix)
   {
      if (matrix == null)
         return "null";
      else
         return getMatrix3DString(format,
                                  matrix.getM00(),
                                  matrix.getM01(),
                                  matrix.getM02(),
                                  matrix.getM10(),
                                  matrix.getM11(),
                                  matrix.getM12(),
                                  matrix.getM20(),
                                  matrix.getM21(),
                                  matrix.getM22());
   }

   /**
    * Gets a representative {@code String} of {@code matrix} as follows:
    *
    * <pre>
    * /-0.576, -0.784,  0.949 \
    * | 0.649, -0.542, -0.941 |
    * \-0.486, -0.502, -0.619 /
    * </pre>
    * </p>
    *
    * @param m00 the 1st row 1st column coefficient of the matrix.
    * @param m01 the 1st row 2nd column coefficient of the matrix.
    * @param m02 the 1st row 3rd column coefficient of the matrix.
    * @param m10 the 2nd row 1st column coefficient of the matrix.
    * @param m11 the 2nd row 2nd column coefficient of the matrix.
    * @param m12 the 2nd row 3rd column coefficient of the matrix.
    * @param m20 the 3rd row 1st column coefficient of the matrix.
    * @param m21 the 3rd row 2nd column coefficient of the matrix.
    * @param m22 the 3rd row 3rd column coefficient of the matrix.
    * @return the representative {@code String}.
    */
   public static String getMatrix3DString(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22)
   {
      return getMatrix3DString(DEFAULT_FORMAT, m00, m01, m02, m10, m11, m12, m20, m21, m22);
   }

   /**
    * Gets a representative {@code String} of {@code matrix} given a specific format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    *
    * <pre>
    * /-0.576, -0.784,  0.949 \
    * | 0.649, -0.542, -0.941 |
    * \-0.486, -0.502, -0.619 /
    * </pre>
    * </p>
    *
    * @param format the format to use for each number.
    * @param m00    the 1st row 1st column coefficient of the matrix.
    * @param m01    the 1st row 2nd column coefficient of the matrix.
    * @param m02    the 1st row 3rd column coefficient of the matrix.
    * @param m10    the 2nd row 1st column coefficient of the matrix.
    * @param m11    the 2nd row 2nd column coefficient of the matrix.
    * @param m12    the 2nd row 3rd column coefficient of the matrix.
    * @param m20    the 3rd row 1st column coefficient of the matrix.
    * @param m21    the 3rd row 2nd column coefficient of the matrix.
    * @param m22    the 3rd row 3rd column coefficient of the matrix.
    * @return the representative {@code String}.
    */
   public static String getMatrix3DString(String format,
                                          double m00,
                                          double m01,
                                          double m02,
                                          double m10,
                                          double m11,
                                          double m12,
                                          double m20,
                                          double m21,
                                          double m22)
   {
      // First transform all the numbers into strings with the given format and find the maximum length per column.
      String m00String = toString(format, m00);
      String m01String = toString(format, m01);
      String m02String = toString(format, m02);
      String m10String = toString(format, m10);
      String m11String = toString(format, m11);
      String m12String = toString(format, m12);
      String m20String = toString(format, m20);
      String m21String = toString(format, m21);
      String m22String = toString(format, m22);

      int c0Length = EuclidCoreTools.max(m00String.length(), m10String.length(), m20String.length());
      int c1Length = EuclidCoreTools.max(m01String.length(), m11String.length(), m21String.length());
      int c2Length = EuclidCoreTools.max(m02String.length(), m12String.length(), m22String.length());

      // Then reformat the strings to have the same length per column.
      m00String = centerPadString(m00String, c0Length);
      m01String = centerPadString(m01String, c1Length);
      m02String = centerPadString(m02String, c2Length);
      m10String = centerPadString(m10String, c0Length);
      m11String = centerPadString(m11String, c1Length);
      m12String = centerPadString(m12String, c2Length);
      m20String = centerPadString(m20String, c0Length);
      m21String = centerPadString(m21String, c1Length);
      m22String = centerPadString(m22String, c2Length);

      return "/%s, %s, %s \\\n|%s, %s, %s |\n\\%s, %s, %s /".formatted(m00String,
                                                                       m01String,
                                                                       m02String,
                                                                       m10String,
                                                                       m11String,
                                                                       m12String,
                                                                       m20String,
                                                                       m21String,
                                                                       m22String);
   }

   private static String centerPadString(String in, int desiredLength)
   {
      int padLength = desiredLength - in.length();
      if (padLength < 0)
         return in;
      int padRight = padLength / 2;
      int padLeft = padLength - padRight;
      return " ".repeat(padLeft) + in + " ".repeat(padRight);
   }

   /**
    * Gets a representative {@code String} of {@code yawPitchRoll} as follows:
    *
    * <pre>
    * yaw-pitch-roll: ( 0.674,  0.455,  0.582 )
    * </pre>
    *
    * @param yawPitchRoll the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getYawPitchRollString(YawPitchRollReadOnly yawPitchRoll)
   {
      return getYawPitchRollString(DEFAULT_FORMAT, yawPitchRoll);
   }

   /**
    * Gets a representative {@code String} of {@code yawPitchRoll} given a specific format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    *
    * <pre>
    * yaw-pitch-roll: ( 0.674,  0.455,  0.582 )
    * </pre>
    * </p>
    *
    * @param format       the format to use for each number.
    * @param yawPitchRoll the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getYawPitchRollString(String format, YawPitchRollReadOnly yawPitchRoll)
   {
      if (yawPitchRoll == null)
         return "null";
      else
         return getYawPitchRollString(format, yawPitchRoll.getYaw(), yawPitchRoll.getPitch(), yawPitchRoll.getRoll());
   }

   /**
    * Gets a representative {@code String} of {@code orientation} using a yaw-pitch-roll representation
    * as follows:
    *
    * <pre>
    * yaw-pitch-roll: ( 0.674,  0.455,  0.582 )
    * </pre>
    *
    * @param orientation the orientation to get the {@code String} of using a yaw-pitch-roll
    *                    representation. Not modified.
    * @return the representative {@code String}.
    */
   public static String getStringAsYawPitchRoll(Orientation3DReadOnly orientation)
   {
      return getStringAsYawPitchRoll(DEFAULT_FORMAT, orientation);
   }

   /**
    * Gets a representative {@code String} of {@code orientation} using a yaw-pitch-roll representation
    * and given a specific format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    *
    * <pre>
    * yaw-pitch-roll: ( 0.674,  0.455,  0.582 )
    * </pre>
    * </p>
    *
    * @param format      the format to use for each number.
    * @param orientation the orientation to get the {@code String} of using a yaw-pitch-roll
    *                    representation. Not modified.
    * @return the representative {@code String}.
    */
   public static String getStringAsYawPitchRoll(String format, Orientation3DReadOnly orientation)
   {
      if (orientation == null)
         return "null";
      else
         return getYawPitchRollString(format, orientation.getYaw(), orientation.getPitch(), orientation.getRoll());
   }

   /**
    * Gets a representative {@code String} of {@code yawPitchRoll} as follows:
    *
    * <pre>
    * yaw-pitch-roll: ( 0.674,  0.455,  0.582 )
    * </pre>
    *
    * @param yaw   the first angle representing the rotation around the z-axis.
    * @param pitch the second angle representing the rotation around the y-axis.
    * @param roll  the third angle representing the rotation around the x-axis.
    * @return the representative {@code String}.
    */
   public static String getYawPitchRollString(double yaw, double pitch, double roll)
   {
      return getYawPitchRollString(DEFAULT_FORMAT, yaw, pitch, roll);
   }

   /**
    * Gets a representative {@code String} of {@code yawPitchRoll} given a specific format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    *
    * <pre>
    * yaw-pitch-roll: ( 0.674,  0.455,  0.582 )
    * </pre>
    * </p>
    *
    * @param format the format to use for each number.
    * @param yaw    the first angle representing the rotation around the z-axis.
    * @param pitch  the second angle representing the rotation around the y-axis.
    * @param roll   the third angle representing the rotation around the x-axis.
    * @return the representative {@code String}.
    */
   public static String getYawPitchRollString(String format, double yaw, double pitch, double roll)
   {
      return getStringOf("yaw-pitch-roll: (", ")", ", ", format, yaw, pitch, roll);
   }

   /**
    * Gets a representative {@code String} of a series of doubles given a specific separator.
    * <p>
    * Using {@code separator = ", "}, this provides a {@code String} as follows:
    *
    * <pre>
    *  0.123, -0.480,  1.457
    * </pre>
    * </p>
    *
    * @param separator the {@code String} to insert between two values.
    * @param values    the values to get the {@code String} of.
    * @return the representative {@code String}.
    */
   public static String getStringOf(String separator, double... values)
   {
      return getStringOf(separator, DEFAULT_FORMAT, values);
   }

   /**
    * Gets a representative {@code String} of a series of doubles given a specific separator, and
    * format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT} and {@code separator = ", "}, this provides a
    * {@code String} as follows:
    *
    * <pre>
    *  0.123, -0.480,  1.457
    * </pre>
    * </p>
    *
    * @param separator the {@code String} to insert between two values.
    * @param format    the format to use for each number.
    * @param values    the values to get the {@code String} of.
    * @return the representative {@code String}.
    */
   public static String getStringOf(String separator, String format, double... values)
   {
      return getStringOf(null, null, separator, format, values);
   }

   /**
    * Gets a representative {@code String} of a series of doubles given specific prefix, suffix, and
    * separator.
    * <p>
    * Using {@code prefix = "("}, {@code suffix = ")"}, and {@code separator = ", "}, this provides a
    * {@code String} as follows:
    *
    * <pre>
    * ( 0.123, -0.480,  1.457)
    * </pre>
    * </p>
    *
    * @param prefix    the {@code String} to prepend to the result.
    * @param suffix    the {@code String} to append to the result.
    * @param separator the {@code String} to insert between two values.
    * @param values    the values to get the {@code String} of.
    * @return the representative {@code String}.
    */
   public static String getStringOf(String prefix, String suffix, String separator, double... values)
   {
      return getStringOf(prefix, suffix, separator, DEFAULT_FORMAT, values);
   }

   /**
    * Gets a representative {@code String} of a series of doubles given specific prefix, suffix,
    * separator, and format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, {@code prefix = "("}, {@code suffix = ")"}, and
    * {@code separator = ", "}, this provides a {@code String} as follows:
    *
    * <pre>
    * ( 0.123, -0.480,  1.457)
    * </pre>
    * </p>
    *
    * @param prefix    the {@code String} to prepend to the result.
    * @param suffix    the {@code String} to append to the result.
    * @param separator the {@code String} to insert between two values.
    * @param format    the format to use for each number.
    * @param values    the values to get the {@code String} of.
    * @return the representative {@code String}.
    */
   public static String getStringOf(String prefix, String suffix, String separator, String format, double... values)
   {
      if (values == null)
         return "null";

      StringBuilder sb = new StringBuilder();

      if (prefix != null)
         sb.append(prefix);

      if (values.length > 0)
      {
         sb.append(toString(format, values[0]));
         for (int i = 1; i < values.length; i++)
         {
            sb.append(separator);
            sb.append(toString(format, values[i]));
         }
      }

      if (suffix != null)
         sb.append(suffix);

      return sb.toString();
   }

   /**
    * Gets a representative {@code String} of a series of floats given a specific separator.
    * <p>
    * Using {@code separator = ", "}, this provides a {@code String} as follows:
    *
    * <pre>
    *  0.123, -0.480,  1.457
    * </pre>
    * </p>
    *
    * @param separator the {@code String} to insert between two values.
    * @param values    the values to get the {@code String} of.
    * @return the representative {@code String}.
    */
   public static String getStringOf(String separator, float... values)
   {
      return getStringOf(separator, DEFAULT_FORMAT, values);
   }

   /**
    * Gets a representative {@code String} of a series of floats given a specific separator, and format
    * to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT} and {@code separator = ", "}, this provides a
    * {@code String} as follows:
    *
    * <pre>
    *  0.123, -0.480,  1.457
    * </pre>
    * </p>
    *
    * @param separator the {@code String} to insert between two values.
    * @param format    the format to use for each number.
    * @param values    the values to get the {@code String} of.
    * @return the representative {@code String}.
    */
   public static String getStringOf(String separator, String format, float... values)
   {
      return getStringOf(null, null, separator, format, values);
   }

   /**
    * Gets a representative {@code String} of a series of floats given specific prefix, suffix, and
    * separator.
    * <p>
    * Using {@code prefix = "("}, {@code suffix = ")"}, and {@code separator = ", "}, this provides a
    * {@code String} as follows:
    *
    * <pre>
    * ( 0.123, -0.480,  1.457)
    * </pre>
    * </p>
    *
    * @param prefix    the {@code String} to prepend to the result.
    * @param suffix    the {@code String} to append to the result.
    * @param separator the {@code String} to insert between two values.
    * @param values    the values to get the {@code String} of.
    * @return the representative {@code String}.
    */
   public static String getStringOf(String prefix, String suffix, String separator, float... values)
   {
      return getStringOf(prefix, suffix, separator, DEFAULT_FORMAT, values);
   }

   /**
    * Gets a representative {@code String} of a series of floats given specific prefix, suffix,
    * separator, and format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, {@code prefix = "("}, {@code suffix = ")"}, and
    * {@code separator = ", "}, this provides a {@code String} as follows:
    *
    * <pre>
    * ( 0.123, -0.480,  1.457)
    * </pre>
    * </p>
    *
    * @param prefix    the {@code String} to prepend to the result.
    * @param suffix    the {@code String} to append to the result.
    * @param separator the {@code String} to insert between two values.
    * @param format    the format to use for each number.
    * @param values    the values to get the {@code String} of.
    * @return the representative {@code String}.
    */
   public static String getStringOf(String prefix, String suffix, String separator, String format, float... values)
   {
      if (values == null)
         return "null";

      StringBuilder sb = new StringBuilder();

      if (prefix != null)
         sb.append(prefix);

      if (values.length > 0)
      {
         sb.append(toString(format, values[0]));
         for (int i = 1; i < values.length; i++)
         {
            sb.append(separator);
            sb.append(toString(format, values[i]));
         }
      }

      if (suffix != null)
         sb.append(suffix);

      return sb.toString();
   }

   /**
    * Gets a representative {@code String} of a series of booleans given a specific separator.
    * <p>
    * Using {@code separator = ", "}, this provides a {@code String} as follows:
    *
    * <pre>
    * true, false, false
    * </pre>
    * </p>
    *
    * @param separator the {@code String} to insert between two values.
    * @param values    the values to get the {@code String} of.
    * @return the representative {@code String}.
    */
   public static String getStringOf(String separator, boolean... values)
   {
      return getStringOf(null, null, separator, values);
   }

   /**
    * Gets a representative {@code String} of a series of booleans given specific prefix, suffix, and
    * separator.
    * <p>
    * Using {@code prefix = "("}, {@code suffix = ")"}, and {@code separator = ", "}, this provides a
    * {@code String} as follows:
    *
    * <pre>
    * (true, false, false)
    * </pre>
    * </p>
    *
    * @param prefix    the {@code String} to prepend to the result.
    * @param suffix    the {@code String} to append to the result.
    * @param separator the {@code String} to insert between two values.
    * @param values    the values to get the {@code String} of.
    * @return the representative {@code String}.
    */
   public static String getStringOf(String prefix, String suffix, String separator, boolean... values)
   {
      if (values == null)
         return "null";

      StringBuilder sb = new StringBuilder();

      if (prefix != null)
         sb.append(prefix);

      if (values.length > 0)
      {
         sb.append(Boolean.toString(values[0]));
         for (int i = 1; i < values.length; i++)
         {
            sb.append(separator);
            sb.append(Boolean.toString(values[i]));
         }
      }

      if (suffix != null)
         sb.append(suffix);

      return sb.toString();
   }

   /**
    * Gets a representative {@code String} of a series of integers given a specific separator.
    * <p>
    * Using {@code separator = ", "}, this provides a {@code String} as follows:
    *
    * <pre>
    * 47, -52200, 9874
    * </pre>
    * </p>
    *
    * @param separator the {@code String} to insert between two values.
    * @param values    the values to get the {@code String} of.
    * @return the representative {@code String}.
    */
   public static String getStringOf(String separator, int... values)
   {
      return getStringOf(null, null, separator, values);
   }

   /**
    * Gets a representative {@code String} of a series of integers given specific prefix, suffix, and
    * separator.
    * <p>
    * Using {@code prefix = "("}, {@code suffix = ")"}, and {@code separator = ", "}, this provides a
    * {@code String} as follows:
    *
    * <pre>
    * (47, -52200, 9874)
    * </pre>
    * </p>
    *
    * @param prefix    the {@code String} to prepend to the result.
    * @param suffix    the {@code String} to append to the result.
    * @param separator the {@code String} to insert between two values.
    * @param values    the values to get the {@code String} of.
    * @return the representative {@code String}.
    */
   public static String getStringOf(String prefix, String suffix, String separator, int... values)
   {
      if (values == null)
         return "null";

      StringBuilder sb = new StringBuilder();

      if (prefix != null)
         sb.append(prefix);

      if (values.length > 0)
      {
         sb.append(Integer.toString(values[0]));
         for (int i = 1; i < values.length; i++)
         {
            sb.append(separator);
            sb.append(Integer.toString(values[i]));
         }
      }

      if (suffix != null)
         sb.append(suffix);

      return sb.toString();
   }

   /**
    * Gets a representative {@code String} of a series of longs given a specific separator.
    * <p>
    * Using {@code separator = ", "}, this provides a {@code String} as follows:
    *
    * <pre>
    * 47, -52200, 9874
    * </pre>
    * </p>
    *
    * @param separator the {@code String} to insert between two values.
    * @param values    the values to get the {@code String} of.
    * @return the representative {@code String}.
    */
   public static String getStringOf(String separator, long... values)
   {
      return getStringOf(null, null, separator, values);
   }

   /**
    * Gets a representative {@code String} of a series of longs given specific prefix, suffix, and
    * separator.
    * <p>
    * Using {@code prefix = "("}, {@code suffix = ")"}, and {@code separator = ", "}, this provides a
    * {@code String} as follows:
    *
    * <pre>
    * (47, -52200, 9874)
    * </pre>
    * </p>
    *
    * @param prefix    the {@code String} to prepend to the result.
    * @param suffix    the {@code String} to append to the result.
    * @param separator the {@code String} to insert between two values.
    * @param values    the values to get the {@code String} of.
    * @return the representative {@code String}.
    */
   public static String getStringOf(String prefix, String suffix, String separator, long... values)
   {
      if (values == null)
         return "null";

      StringBuilder sb = new StringBuilder();

      if (prefix != null)
         sb.append(prefix);

      if (values.length > 0)
      {
         sb.append(Long.toString(values[0]));
         for (int i = 1; i < values.length; i++)
         {
            sb.append(separator);
            sb.append(Long.toString(values[i]));
         }
      }

      if (suffix != null)
         sb.append(suffix);

      return sb.toString();
   }

   /**
    * Gets a representative {@code String} of the elements contained in the given array.
    * <p>
    * This provides an alternative to {@link Arrays#toString(Object[])} where the format of the output
    * can be controller by defining a custom {@code separator}. For instance, with
    * {@code separator = \n} the resulting {@code String} is composed of one element per line as
    * opposed to {@link Arrays#toString(Object[])} which outputs all the elements in one line.
    * </p>
    *
    * @param separator the {@code String} used to separate elements of the array.
    * @param array     the array of elements to get the {@code String} of.
    * @return the representative {@code String}.
    */
   public static String getArrayString(String separator, Object[] array)
   {
      return getArrayString(null, null, separator, array);
   }

   /**
    * Gets a representative {@code String} of the elements contained in the given array.
    * <p>
    * This provides an alternative to {@link Arrays#toString(Object[])} where the format of the output
    * can be controlled by defining a custom {@code separator}. For instance, with
    * {@code separator = \n} the resulting {@code String} is composed of one element per line as
    * opposed to {@link Arrays#toString(Object[])} which outputs all the elements in one line.
    * </p>
    *
    * @param prefix    the {@code String} to prepend to the result.
    * @param suffix    the {@code String} to append to the result.
    * @param separator the {@code String} used to separate elements of the array.
    * @param array     the array of elements to get the {@code String} of.
    * @return the representative {@code String}.
    */
   public static String getArrayString(String prefix, String suffix, String separator, Object[] array)
   {
      return getArrayString(prefix, suffix, separator, array, Object::toString);
   }

   /**
    * Gets a representative {@code String} of the elements contained in the given array.
    * <p>
    * This provides an alternative to {@link Arrays#toString(Object[])} where the format of the output
    * can be controller by defining a custom {@code separator}. For instance, with
    * {@code separator = \n} the resulting {@code String} is composed of one element per line as
    * opposed to {@link Arrays#toString(Object[])} which outputs all the elements in one line.
    * </p>
    *
    * @param <T>                     the type of the array elements.
    * @param separator               the {@code String} used to separate elements of the array.
    * @param array                   the array of elements to get the {@code String} of.
    * @param elementToStringFunction the {@code Function} used to generate a representative
    *                                {@code String} for each element.
    * @return the representative {@code String}.
    */
   public static <T> String getArrayString(String separator, T[] array, Function<T, String> elementToStringFunction)
   {
      return getArrayString(null, null, separator, array, elementToStringFunction);
   }

   /**
    * Gets a representative {@code String} of the elements contained in the given array.
    * <p>
    * This provides an alternative to {@link Arrays#toString(Object[])} where the format of the output
    * can be controlled by defining a custom {@code separator}. For instance, with
    * {@code separator = \n} the resulting {@code String} is composed of one element per line as
    * opposed to {@link Arrays#toString(Object[])} which outputs all the elements in one line.
    * </p>
    *
    * @param <T>                     the type of the array elements.
    * @param prefix                  the {@code String} to prepend to the result.
    * @param suffix                  the {@code String} to append to the result.
    * @param separator               the {@code String} used to separate elements of the array.
    * @param array                   the array of elements to get the {@code String} of.
    * @param elementToStringFunction the {@code Function} used to generate a representative
    *                                {@code String} for each element.
    * @return the representative {@code String}.
    */
   public static <T> String getArrayString(String prefix, String suffix, String separator, T[] array, Function<T, String> elementToStringFunction)
   {
      if (array == null)
         return "null";
      else
         return getCollectionString(prefix, suffix, separator, Arrays.asList(array), elementToStringFunction);
   }

   /**
    * Gets a representative {@code String} of the elements contained in the given {@code Collection}.
    * <p>
    * This provides an alternative to {@link Collection#toString()} where the format of the output can
    * be controller by defining a custom {@code separator}. For instance, with {@code separator = \n}
    * the resulting {@code String} is composed of one element per line as opposed to
    * {@link Collection#toString()} which outputs all the elements in one line.
    * </p>
    *
    * @param separator  the {@code String} used to separate elements of the collection.
    * @param collection the series of elements to get the {@code String} of.
    * @return the representative {@code String}.
    */
   public static String getCollectionString(String separator, Collection<?> collection)
   {
      return getCollectionString(null, null, separator, collection);
   }

   /**
    * Gets a representative {@code String} of the elements contained in the given {@code Collection}.
    * <p>
    * This provides an alternative to {@link Collection#toString()} where the format of the output can
    * be controller by defining a custom {@code separator}. For instance, with {@code separator = \n}
    * the resulting {@code String} is composed of one element per line as opposed to
    * {@link Collection#toString()} which outputs all the elements in one line.
    * </p>
    *
    * @param prefix     the {@code String} to prepend to the result.
    * @param suffix     the {@code String} to append to the result.
    * @param separator  the {@code String} used to separate elements of the collection.
    * @param collection the series of elements to get the {@code String} of.
    * @return the representative {@code String}.
    */
   public static String getCollectionString(String prefix, String suffix, String separator, Collection<?> collection)
   {
      return getCollectionString(prefix, suffix, separator, collection, Object::toString);
   }

   /**
    * Gets a representative {@code String} of the elements contained in the given {@code Collection}.
    * <p>
    * This provides an alternative to {@link Collection#toString()} where the format of the output can
    * be controller by defining a custom {@code separator}. For instance, with {@code separator = \n}
    * the resulting {@code String} is composed of one element per line as opposed to
    * {@link Collection#toString()} which outputs all the elements in one line.
    * </p>
    *
    * @param <T>                     the type of the collection elements.
    * @param separator               the {@code String} used to separate elements of the collection.
    * @param collection              the series of elements to get the {@code String} of.
    * @param elementToStringFunction the {@code Function} used to generate a representative
    *                                {@code String} for each element.
    * @return the representative {@code String}.
    */
   public static <T> String getCollectionString(String separator, Collection<? extends T> collection, Function<T, String> elementToStringFunction)
   {
      return getCollectionString(null, null, separator, collection, elementToStringFunction);
   }

   /**
    * Gets a representative {@code String} of the elements contained in the given {@code Collection}.
    * <p>
    * This provides an alternative to {@link Collection#toString()} where the format of the output can
    * be controller by defining a custom {@code separator}. For instance, with {@code separator = \n}
    * the resulting {@code String} is composed of one element per line as opposed to
    * {@link Collection#toString()} which outputs all the elements in one line.
    * </p>
    *
    * @param <T>                     the type of the collection elements.
    * @param prefix                  the {@code String} to prepend to the result.
    * @param suffix                  the {@code String} to append to the result.
    * @param separator               the {@code String} used to separate elements of the collection.
    * @param collection              the series of elements to get the {@code String} of.
    * @param elementToStringFunction the {@code Function} used to generate a representative
    *                                {@code String} for each element.
    * @return the representative {@code String}.
    */
   public static <T> String getCollectionString(String prefix,
                                                String suffix,
                                                String separator,
                                                Collection<? extends T> collection,
                                                Function<T, String> elementToStringFunction)
   {
      if (collection == null)
         return "null";

      StringBuilder sb = new StringBuilder();
      if (prefix != null)
         sb.append(prefix);

      if (!collection.isEmpty())
      {
         Iterator<? extends T> iterator = collection.iterator();
         sb.append(elementToStringFunction.apply(iterator.next()));

         while (iterator.hasNext())
         {
            T next = iterator.next();
            sb.append(separator);
            sb.append(next == null ? "null" : elementToStringFunction.apply(next));
         }
      }

      if (suffix != null)
         sb.append(suffix);

      return sb.toString();
   }

   /**
    * Format the {@code value} using the given {@code format}.
    *
    * @param format the format to use. Can be {@code null}.
    * @param value  the value to get the {@code String} of.
    * @return the formatted {@code String} representing the given {@code value}.
    */
   public static String toString(String format, double value)
   {
      return format != null ? String.format(format, value) : Double.toString(value);
   }

   /**
    * Format the {@code value} using the given {@code format}.
    *
    * @param format the format to use. Can be {@code null}.
    * @param value  the value to get the {@code String} of.
    * @return the formatted {@code String} representing the given {@code value}.
    */
   public static String toString(String format, float value)
   {
      return format != null ? String.format(format, value) : Float.toString(value);
   }

   /**
    * Format the {@code value} using the given {@code format}.
    *
    * @param format the format to use. Can be {@code null}.
    * @param value  the value to get the {@code String} of.
    * @return the formatted {@code String} representing the given {@code value}.
    */
   public static String toString(String format, boolean value)
   {
      return format != null ? String.format(format, value) : Boolean.toString(value);
   }

   /**
    * Format the {@code value} using the given {@code format}.
    *
    * @param format the format to use. Can be {@code null}.
    * @param value  the value to get the {@code String} of.
    * @return the formatted {@code String} representing the given {@code value}.
    */
   public static String toString(String format, int value)
   {
      return format != null ? String.format(format, value) : Integer.toString(value);
   }

   /**
    * Format the {@code value} using the given {@code format}.
    *
    * @param format the format to use. Can be {@code null}.
    * @param value  the value to get the {@code String} of.
    * @return the formatted {@code String} representing the given {@code value}.
    */
   public static String toString(String format, long value)
   {
      return format != null ? String.format(format, value) : Long.toString(value);
   }

   /**
    * Gets the {@code String} for formatting decimal numbers.
    *
    * @param numberOfChar the number of character to print.
    * @param precision    the precision of the output.
    * @return the formatting {@code String}.
    */
   public static String getStringFormat(int numberOfChar, int precision)
   {
      return "%" + numberOfChar + "." + precision + "f";
   }
}
