package us.ihmc.euclid.shape.convexPolytope.interfaces;

import java.util.Collection;

import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public interface Vertex3DReadOnly extends Point3DReadOnly
{
   /**
    * Get list of edges that originate at this vertex
    * 
    * @return a list of read only references to the edges that originate at this edge
    */
   Collection<? extends HalfEdge3DReadOnly> getAssociatedEdges();

   /**
    * Get a particular associated edge based on its index in the associated edge list held by the
    * vertex.
    * 
    * @param index must be less than value returned by {@code getNumberOfAssociatedEdges()}
    * @return a read only reference to the edge
    */
   HalfEdge3DReadOnly getAssociatedEdge(int index);

   /**
    * Checks if the edge specified originates at the current vertex. Check is performed by comparing
    * objects and not geometrical closeness
    * 
    * @param the half edge that is to be checked for association
    * @return {@code true} if the specified edge is on the associated edge list, {@code false}
    *         otherwise
    */
   default boolean isAssociatedWithEdge(HalfEdge3DReadOnly edgeToCheck)
   {
      return getAssociatedEdges().contains(edgeToCheck);
   }

   /**
    * Returns the number of references held in the associated edge list
    * 
    * @return integer number of edges that originate at this vertex
    */
   int getNumberOfAssociatedEdges();

   /**
    * Retrieves the half-edge that originates from this vertex and ends at the given
    * {@code destination}.
    * 
    * @param destination the vertex to which the desired half-edge ends at.
    * @return the half-edge starting from {@code this} and ending at {@code destination}, or
    *         {@code null} if no such half-edge exists.
    */
   default HalfEdge3DReadOnly getEdgeTo(Vertex3DReadOnly destination)
   {
      for (int edgeIndex = 0; edgeIndex < getNumberOfAssociatedEdges(); edgeIndex++)
      {
         HalfEdge3DReadOnly candidate = getAssociatedEdge(edgeIndex);

         if (candidate.getDestination() == destination)
            return candidate;
      }

      return null;
   }

   /**
    * Calculates the dot product of the specified vector and the vector from the origin to this vertex
    * 
    * @return the resultant dot product
    */
   default double dot(Vector3DReadOnly vector)
   {
      return getX() * vector.getX() + getY() * vector.getY() + getZ() * vector.getZ();
   }

   @Override
   default double distance(Point3DReadOnly other)
   {
      return Point3DReadOnly.super.distance(other);
   }

   default boolean equals(Vertex3DReadOnly other)
   {
      return Point3DReadOnly.super.equals(other);
   }

   default boolean epsilonEquals(Vertex3DReadOnly other, double epsilon)
   {
      return Point3DReadOnly.super.epsilonEquals(other, epsilon);
   }

   default boolean geometricallyEquals(Vertex3DReadOnly other, double epsilon)
   {
      return Point3DReadOnly.super.geometricallyEquals(other, epsilon);
   }

   /**
    * Get a printable version of the vertex data
    * 
    * @return string indicating the spatial coordinates for this vertex
    */
   String toString();
}