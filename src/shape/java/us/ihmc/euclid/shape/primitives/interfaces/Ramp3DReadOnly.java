package us.ihmc.euclid.shape.primitives.interfaces;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.interfaces.BoundingBox3DBasics;
import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.interfaces.EuclidGeometry;
import us.ihmc.euclid.interfaces.Transformable;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.shape.tools.EuclidShapeIOTools;
import us.ihmc.euclid.shape.tools.EuclidShapeTools;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * Read-only interface for a ramp 3D.
 * <p>
 * A ramp represents a 3D shape with a triangular section in the XZ-plane. Shape description:
 * <ul>
 * <li>The slope face starts from {@code x=0.0}, {@code z=0.0} to end at {@code x=size.getX()},
 * {@code z=size.getZ()}.
 * <li>The bottom face is horizontal (XY-plane) at {@code z=0.0}.
 * <li>The rear face is vertical (YZ-plane) at {@code x=size.getX()}.
 * <li>The left face is vertical (XZ-plane) at {@code y=-size.getY()/2.0}.
 * <li>The right face is vertical (XZ-plane) at {@code y=size.getY()/2.0}.
 * </ul>
 * </p>
 *
 * @author Sylvain Bertrand
 */
public interface Ramp3DReadOnly extends Shape3DReadOnly
{
   /**
    * Get the read-only reference to the size along the three local axes of this ramp.
    *
    * @return the size of this ramp.
    */
   Vector3DReadOnly getSize();

   /**
    * Gets the read-only reference to the pose of this ramp.
    *
    * @return the pose of this ramp.
    */
   @Override
   Shape3DPoseReadOnly getPose();

   /**
    * Gets the read-only reference to the orientation of this ramp.
    *
    * @return the orientation of this ramp.
    */
   default RotationMatrixReadOnly getOrientation()
   {
      return getPose().getShapeOrientation();
   }

   /**
    * Gets the read-only reference of the position of this ramp.
    *
    * @return the position of this ramp.
    */
   default Point3DReadOnly getPosition()
   {
      return getPose().getShapePosition();
   }

   /** {@inheritDoc} */
   @Override
   default double getVolume()
   {
      return EuclidShapeTools.rampVolume(getSizeX(), getSizeY(), getSizeZ());
   }

   /**
    * Checks that the size component corresponding to the given axis is positive.
    *
    * @param axis to identify the component to check.
    * @throws IllegalArgumentException if the size component is strictly negative.
    */
   default void checkSizePositive(Axis3D axis)
   {
      if (getSize().getElement(axis) < 0.0)
         throw new IllegalArgumentException("The " + axis + "-size of a " + getClass().getSimpleName() + " cannot be negative: " + getSize().getElement(axis));
   }

   /**
    * Gets the intermediate variable supplier that can be used for performing operations in either a
    * garbage-free of thread-safe manner.
    *
    * @return the intermediate variable supplier.
    */
   IntermediateVariableSupplier getIntermediateVariableSupplier();

   /** {@inheritDoc} */
   @Override
   default boolean containsNaN()
   {
      return getPose().containsNaN() || getSize().containsNaN();
   }

   /** {@inheritDoc} */
   @Override
   default boolean evaluatePoint3DCollision(Point3DReadOnly pointToCheck, Point3DBasics closestPointOnSurfaceToPack, Vector3DBasics normalAtClosestPointToPack)
   {
      Point3DBasics pointToCheckInLocal = getIntermediateVariableSupplier().requestPoint3D();
      getPose().inverseTransform(pointToCheck, pointToCheckInLocal);

      double distance = EuclidShapeTools.evaluatePoint3DRamp3DCollision(pointToCheckInLocal,
                                                                        getSize(),
                                                                        closestPointOnSurfaceToPack,
                                                                        normalAtClosestPointToPack);

      transformToWorld(closestPointOnSurfaceToPack);
      transformToWorld(normalAtClosestPointToPack);

      getIntermediateVariableSupplier().releasePoint3D(pointToCheckInLocal);

      return distance <= 0.0;
   }


   /**
    * Computes the coordinates of the possible intersections between a line and this ramp.
    * <p>
    * In the case the line and this ramp do not intersect, this method returns {@code 0} and
    * {@code firstIntersectionToPack} and {@code secondIntersectionToPack} remain unmodified.
    * </p>
    *
    * @param line                     the line expressed in world coordinates that may intersect this
    *                                 ramp. Not modified.
    * @param firstIntersectionToPack  the coordinate in world of the first intersection. Can be
    *                                 {@code null}. Modified.
    * @param secondIntersectionToPack the coordinate in world of the second intersection. Can be
    *                                 {@code null}. Modified.
    * @return the number of intersections between the line and this ramp. It is either equal to 0, 1, or
    *         2.
    */
   default int intersectionWith(Line3DReadOnly line, Point3DBasics firstIntersectionToPack, Point3DBasics secondIntersectionToPack)
   {
      return intersectionWith(line.getPoint(), line.getDirection(), firstIntersectionToPack, secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the possible intersections between a line and this ramp.
    * <p>
    * In the case the line and this ramp do not intersect, this method returns {@code 0} and
    * {@code firstIntersectionToPack} and {@code secondIntersectionToPack} are set to
    * {@link Double#NaN}.
    * </p>
    *
    * @param pointOnLine              a point expressed in world located on the infinitely long line.
    *                                 Not modified.
    * @param lineDirection            the direction expressed in world of the line. Not modified.
    * @param firstIntersectionToPack  the coordinate in world of the first intersection. Can be
    *                                 {@code null}. Modified.
    * @param secondIntersectionToPack the coordinate in world of the second intersection. Can be
    *                                 {@code null}. Modified.
    * @return the number of intersections between the line and this ramp. It is either equal to 0, 1, or
    *         2.
    */
   default int intersectionWith(Point3DReadOnly pointOnLine,
                                Vector3DReadOnly lineDirection,
                                Point3DBasics firstIntersectionToPack,
                                Point3DBasics secondIntersectionToPack)
   {
      
     
      double rampPositionX = getPose().getShapePosition().getX();
      double rampPositionY = getPose().getShapePosition().getY(); 
      double rampPositionZ = getPose().getShapePosition().getZ(); 
      double rampLength = getSizeX();
      double rampWidth = getSizeY();
      double rampHeight = getSizeZ();
      
      Vector3DReadOnly axisX = getPose().getXAxis();
      Vector3DReadOnly axisY = getPose().getYAxis();
      Vector3DReadOnly axisZ = getPose().getZAxis();
      
      double angle = getRampIncline();
      
      double pointOnLineX = pointOnLine.getX();
      double pointOnLineY = pointOnLine.getY();
      double pointOnLineZ = pointOnLine.getZ();
      
      double lineDirectionX = lineDirection.getX();
      double lineDirectionY = lineDirection.getY();
      double lineDirectionZ = lineDirection.getZ();
      
      Point3DBasics pointOnLineInLocal = getIntermediateVariableSupplier().requestPoint3D();
      Vector3DBasics lineDirectionInLocal = getIntermediateVariableSupplier().requestVector3D();
      
      getPose().inverseTransform(pointOnLine, pointOnLineInLocal);
      getPose().inverseTransform(lineDirection, lineDirectionInLocal);


      int numberOfIntersections = EuclidGeometryTools.intersectionBetweenLine3DAndRampImpl(rampPositionX,
                                                                                           rampPositionY,
                                                                                           rampPositionZ,
                                                                                           rampLength,
                                                                                           rampWidth,
                                                                                           rampHeight,
                                                                                           axisX,
                                                                                           axisY,
                                                                                           axisZ,
                                                                                           angle,
                                                                                           pointOnLineX,
                                                                                           pointOnLineY,
                                                                                           pointOnLineZ,
                                                                                           lineDirectionX,
                                                                                           lineDirectionY,
                                                                                           lineDirectionZ,
                                                                                           firstIntersectionToPack,
                                                                                           secondIntersectionToPack);
      
      getIntermediateVariableSupplier().releasePoint3D(pointOnLineInLocal);
      getIntermediateVariableSupplier().releaseVector3D(lineDirectionInLocal);

      if (firstIntersectionToPack != null && numberOfIntersections >= 1)
         transformToWorld(firstIntersectionToPack);
      if (secondIntersectionToPack != null && numberOfIntersections == 2)
         transformToWorld(secondIntersectionToPack);
      return numberOfIntersections;
   }


   
   /** {@inheritDoc} */
   @Override
   default boolean getSupportingVertex(Vector3DReadOnly supportDirection, Point3DBasics supportingVertexToPack)
   {
      if (getOrientation().isIdentity())
      {
         EuclidShapeTools.supportingVectexRamp3D(supportDirection, getSize(), supportingVertexToPack);
         supportingVertexToPack.add(getPosition());
      }
      else
      {
         Vector3DBasics supportDirectionInLocal = getIntermediateVariableSupplier().requestVector3D();
         getPose().inverseTransform(supportDirection, supportDirectionInLocal);

         EuclidShapeTools.supportingVectexRamp3D(supportDirectionInLocal, getSize(), supportingVertexToPack);
         transformToWorld(supportingVertexToPack);

         getIntermediateVariableSupplier().releaseVector3D(supportDirectionInLocal);
      }

      return true;
   }

   /** {@inheritDoc} */
   @Override
   default double signedDistance(Point3DReadOnly point)
   {
      Point3DBasics pointInLocal = getIntermediateVariableSupplier().requestPoint3D();
      getPose().inverseTransform(point, pointInLocal);

      double signedDistance = EuclidShapeTools.signedDistanceBetweenPoint3DAndRamp3D(pointInLocal, getSize());

      getIntermediateVariableSupplier().releasePoint3D(pointInLocal);

      return signedDistance;
   }

   /** {@inheritDoc} */
   @Override
   default boolean isPointInside(Point3DReadOnly query, double epsilon)
   {
      Point3DBasics queryInLocal = getIntermediateVariableSupplier().requestPoint3D();
      getPose().inverseTransform(query, queryInLocal);

      boolean isInside = EuclidShapeTools.isPoint3DInsideRamp3D(queryInLocal, getSize(), epsilon);

      getIntermediateVariableSupplier().releasePoint3D(queryInLocal);

      return isInside;
   }

   /** {@inheritDoc} */
   @Override
   default boolean orthogonalProjection(Point3DReadOnly pointToProject, Point3DBasics projectionToPack)
   {
      Point3DBasics pointToProjectInLocal = getIntermediateVariableSupplier().requestPoint3D();
      getPose().inverseTransform(pointToProject, pointToProjectInLocal);

      boolean hasBeenProjected = EuclidShapeTools.orthogonalProjectionOntoRamp3D(pointToProjectInLocal, getSize(), projectionToPack);

      if (hasBeenProjected)
         transformToWorld(projectionToPack);

      getIntermediateVariableSupplier().releasePoint3D(pointToProjectInLocal);

      return hasBeenProjected;
   }

   /**
    * Gets the size of this ramp along the x-axis.
    *
    * @return this ramp size along the x-axis.
    */
   default double getSizeX()
   {
      return getSize().getX();
   }

   /**
    * Gets the size of this ramp along the y-axis.
    *
    * @return this ramp size along the x-axis.
    */
   default double getSizeY()
   {
      return getSize().getY();
   }

   /**
    * Gets the size of this ramp along the z-axis.
    *
    * @return this ramp size along the x-axis.
    */
   default double getSizeZ()
   {
      return getSize().getZ();
   }

   /**
    * Gets the length of this ramp's slope part.
    * <p>
    * Note that this is different than {@link #getSizeX()}. The returned value is equal to:
    * &radic;(this.length<sup>2</sup> + this.height<sup>2</sup>)
    * </p>
    *
    * @return the length of the slope.
    */
   default double getRampLength()
   {
      return EuclidShapeTools.computeRamp3DLength(getSize());
   }

   /**
    * Gets the angle formed by the slope and the bottom face.
    * <p>
    * The angle is positive and in [0, <i>pi</i>].
    * </p>
    *
    * @return the slope angle.
    */
   default double getRampIncline()
   {
      return EuclidShapeTools.computeRamp3DIncline(getSize());
   }

   /**
    * Computes and returns the surface normal of the slope face of this ramp.
    * <p>
    * WARNING: The default implementation of this method generates garbage.
    * </p>
    *
    * @return the surface normal of the slope.
    */
   default Vector3DReadOnly getRampSurfaceNormal()
   {
      Vector3D surfaceNormal = new Vector3D();
      getRampSurfaceNormal(surfaceNormal);
      return surfaceNormal;
   }

   /**
    * Computes and packs the surface normal of the slope face of this ramp.
    *
    * @param surfaceNormalToPack the surface normal of the slope. Modified.
    */
   default void getRampSurfaceNormal(Vector3DBasics surfaceNormalToPack)
   {
      surfaceNormalToPack.set(-getSizeZ() / getRampLength(), 0.0, getSizeX() / getRampLength());
      transformToWorld(surfaceNormalToPack);
   }

   /**
    * Gets the 6 vertices, expressed in world, of this ramp as an array.
    * <p>
    * WARNING: The default implementation of this method generates garbage.
    * </p>
    *
    * @return an array of 6 {@code Point3D} with this ramp vertices.
    */
   default Point3DBasics[] getVertices()
   {
      Point3D[] vertices = new Point3D[6];
      for (int vertexIndex = 0; vertexIndex < 6; vertexIndex++)
         getVertex(vertexIndex, vertices[vertexIndex] = new Point3D());
      return vertices;
   }

   /**
    * Pack the coordinates in world of the 6 vertices of this ramp in the given array.
    *
    * @param verticesToPack the array in which the coordinates are stored. Modified.
    * @throws IllegalArgumentException if the length of the given array is different than 6.
    * @throws NullPointerException     if any of the 6 first elements of the given array is
    *                                  {@code null}.
    */
   default void getVertices(Point3DBasics[] verticesToPack)
   {
      if (verticesToPack.length < 6)
         throw new IllegalArgumentException("Array is too small, has to be at least 6 element long, was: " + verticesToPack.length);

      for (int vertexIndex = 0; vertexIndex < 6; vertexIndex++)
         getVertex(vertexIndex, verticesToPack[vertexIndex]);
   }

   /**
    * Packs the world coordinates of one of this ramp vertices.
    * <p>
    * WARNING: The default implementation of this method generates garbage.
    * </p>
    *
    * @param vertexIndex the index in [0, 5] of the vertex to pack.
    * @return the coordinates of the vertex.
    * @throws IndexOutOfBoundsException if {@code vertexIndex} is not in [0, 5].
    */
   default Point3DBasics getVertex(int vertexIndex)
   {
      Point3D vertex = new Point3D();
      getVertex(vertexIndex, vertex);
      return vertex;
   }

   /**
    * Packs the world coordinates of one of this ramp vertices.
    *
    * @param vertexIndex  the index in [0, 5] of the vertex to pack.
    * @param vertexToPack point in which the coordinates of the vertex are stored. Modified.
    * @throws IndexOutOfBoundsException if {@code vertexIndex} is not in [0, 5].
    */
   default void getVertex(int vertexIndex, Point3DBasics vertexToPack)
   {
      if (vertexIndex < 0 || vertexIndex >= 6)
         throw new IndexOutOfBoundsException("The vertex index has to be in [0, 5], was: " + vertexIndex);

      vertexToPack.set((vertexIndex & 2) == 0 ? getSizeX() : 0.0,
                       (vertexIndex & 1) == 0 ? 0.5 * getSizeY() : -0.5 * getSizeY(),
                       (vertexIndex & 4) == 0 ? 0.0 : getSizeZ());
      transformToWorld(vertexToPack);
   }

   /** {@inheritDoc} */
   @Override
   default void getBoundingBox(BoundingBox3DBasics boundingBoxToPack)
   {
      boundingBoxToPack.setToNaN();
      Point3DBasics vertex = getIntermediateVariableSupplier().requestPoint3D();

      for (int vertexIndex = 0; vertexIndex < 6; vertexIndex++)
      {
         getVertex(vertexIndex, vertex);
         boundingBoxToPack.updateToIncludePoint(vertex);
      }

      getIntermediateVariableSupplier().releasePoint3D(vertex);
   }

   /**
    * Gets the {@code ConvexPolytope3DReadOnly} view backed this ramp.
    *
    * @return the polytope view of this ramp.
    */
   RampPolytope3DView asConvexPolytope();

   /** {@inheritDoc} */
   @Override
   default boolean isConvex()
   {
      return true;
   }

   /** {@inheritDoc} */
   @Override
   default boolean isPrimitive()
   {
      return true;
   }

   /** {@inheritDoc} */
   @Override
   default boolean isDefinedByPose()
   {
      return true;
   }

   @Override
   Ramp3DBasics copy();

   /** {@inheritDoc} */
   @Override
   default boolean epsilonEquals(EuclidGeometry geometry, double epsilon)
   {
      if (geometry == this)
         return true;
      if (geometry == null)
         return false;
      if (!(geometry instanceof Ramp3DReadOnly))
         return false;
      Ramp3DReadOnly other = (Ramp3DReadOnly) geometry;
      return getSize().epsilonEquals(other.getSize(), epsilon) && getPose().epsilonEquals(other.getPose(), epsilon);
   }

   /** {@inheritDoc} */
   @Override
   default boolean geometricallyEquals(EuclidGeometry geometry, double epsilon)
   {
      if (geometry == this)
         return true;
      if (geometry == null)
         return false;
      if (!(geometry instanceof Ramp3DReadOnly))
         return false;
      Ramp3DReadOnly other = (Ramp3DReadOnly) geometry;
      return getSize().epsilonEquals(other.getSize(), epsilon) && getPose().geometricallyEquals(other.getPose(), epsilon);
   }

   /** {@inheritDoc} */
   @Override
   default boolean equals(EuclidGeometry geometry)
   {
      if (geometry == this)
         return true;
      if (geometry == null)
         return false;
      if (!(geometry instanceof Ramp3DReadOnly))
         return false;
      Ramp3DReadOnly other = (Ramp3DReadOnly) geometry;
      return getPose().equals(other.getPose()) && getSize().equals(other.getSize());
   }

   /**
    * Changes the given {@code transformable} from being expressed in world to being expressed in this
    * shape local coordinates.
    *
    * @param transformable the transformable to change the coordinates in which it is expressed.
    *                      Modified.
    */
   default void transformToLocal(Transformable transformable)
   {
      transformable.applyInverseTransform(getPose());
   }

   /**
    * Changes the given {@code transformable} from being expressed in this shape local coordinates to
    * being expressed in world.
    *
    * @param transformable the transformable to change the coordinates in which it is expressed.
    *                      Modified.
    */
   default void transformToWorld(Transformable transformable)
   {
      transformable.applyTransform(getPose());
   }

   /**
    * Gets the representative {@code String} of this ramp 3D given a specific format to use.
    * <p>
    * Using the default format {@link EuclidCoreIOTools#DEFAULT_FORMAT}, this provides a {@code String}
    * as follows:
    *
    * <pre>
    * Ramp 3D: [position: ( 0.540,  0.110,  0.319 ), yaw-pitch-roll: (-2.061, -0.904, -1.136), size: ( 0.191,  0.719,  0.479 )]
    * </pre>
    * </p>
    */
   @Override
   default String toString(String format)
   {
      return EuclidShapeIOTools.getRamp3DString(format, this);
   }
}
