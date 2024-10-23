package us.ihmc.euclid.interfaces;

import us.ihmc.euclid.tools.EuclidCoreIOTools;

/**
 * Base interface used to represent all the geometry objects in the Euclid library.
 */
public interface EuclidGeometry
{
   /**
    * Tests if {@code this} is exactly equal to the given {@code geometry}.
    * <p>
    * The test is achieved on a per component basis. A failing test does not necessarily mean that the
    * two objects are different in a geometric way.
    * </p>
    *
    * @param geometry the geometry to compare against {@code this}. Not modified.
    * @return {@code true} if the two objects are of same type and are equal component-wise,
    *         {@code false} otherwise.
    */
   boolean equals(EuclidGeometry geometry);

   /**
    * Tests if {@code this} is approximately equal to {@code geometry} using the tolerance
    * {@code epsilon}.
    * <p>
    * Similar to {@link #equals(EuclidGeometry)}, the test is achieved on a per component basis. A
    * failing test does not necessarily mean that the two objects are different in a geometric way.
    * </p>
    *
    * @param geometry the geometry to compare against {@code this}. Not modified.
    * @param epsilon  tolerance to use when comparing each component.
    * @return {@code true} if the two objects are of same type and are approximately equal
    *         component-wise, {@code false} otherwise.
    */
   boolean epsilonEquals(EuclidGeometry geometry, double epsilon);

   /**
    * Tests if {@code this} and {@code geometry} represent the same geometry to an {@code epsilon}.
    * <p>
    * The implementation of this test depends on the type of geometry. For instance, two points will be
    * considered geometrically equal if they are at a distance from each other that is less or equal
    * than {@code epsilon}. The two object must represent the same type of geometry.
    * </p>
    *
    * @param geometry the geometry to compare against {@code this}. Not modified.
    * @param epsilon  tolerance to use when comparing the two geometries, usually refers to a distance.
    * @return {@code true} if the two objects represent the same geometry, {@code false} otherwise.
    */
   boolean geometricallyEquals(EuclidGeometry geometry, double epsilon);

   /**
    * Provides a {@code String} representation of this geometry.
    *
    * @param format the format to use for each number, if {@code null} is passed, raw number format is
    *               used.
    * @return the {@code String} representing this geometry.
    * @see EuclidCoreIOTools#DEFAULT_FORMAT
    */
   String toString(String format);
}