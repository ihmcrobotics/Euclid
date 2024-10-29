package us.ihmc.euclid.shape.collision;

import org.junit.jupiter.api.Test;
import us.ihmc.euclid.geometry.LineSegment3D;
import us.ihmc.euclid.geometry.interfaces.Vertex3DSupplier;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.shape.collision.GilbertJohnsonKeerthiCollisionDetectorTest.AnalyticalShapeCollisionDetection;
import us.ihmc.euclid.shape.collision.GilbertJohnsonKeerthiCollisionDetectorTest.Pair;
import us.ihmc.euclid.shape.collision.epa.ExpandingPolytopeAlgorithm;
import us.ihmc.euclid.shape.collision.gjk.GilbertJohnsonKeerthiCollisionDetector;
import us.ihmc.euclid.shape.collision.interfaces.SupportingVertexHolder;
import us.ihmc.euclid.shape.convexPolytope.ConvexPolytope3D;
import us.ihmc.euclid.shape.convexPolytope.Face3D;
import us.ihmc.euclid.shape.convexPolytope.HalfEdge3D;
import us.ihmc.euclid.shape.convexPolytope.Vertex3D;
import us.ihmc.euclid.shape.convexPolytope.interfaces.ConvexPolytope3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.tools.EuclidPolytopeFactories;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.shape.primitives.Capsule3D;
import us.ihmc.euclid.shape.primitives.Cylinder3D;
import us.ihmc.euclid.shape.primitives.Ellipsoid3D;
import us.ihmc.euclid.shape.primitives.PointShape3D;
import us.ihmc.euclid.shape.primitives.Ramp3D;
import us.ihmc.euclid.shape.primitives.Sphere3D;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DBasics;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DReadOnly;
import us.ihmc.euclid.shape.tools.EuclidShapeRandomTools;
import us.ihmc.euclid.shape.tools.EuclidShapeTestTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple4D.Quaternion;

import java.util.Arrays;
import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;

class ExpandingPolytopeAlgorithmTest
{
   private static final int ITERATIONS = 5000;
   private static final double EPSILON = 1.0e-12;

   @Test
   void testNonCollidingCubeAndTetrahedron()
   {
      Random random = new Random(34534);

      ConvexPolytope3D cube = EuclidPolytopeFactories.newCube(1.0);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Point3D tetrahedronClosest = new Point3D(0.5, 0.0, 0.0);
         Point3D tetrahedronFarthest0 = new Point3D(1.0, 1.0, 0.0);
         Point3D tetrahedronFarthest1 = new Point3D(1.0, 0.0, 1.0);
         Point3D tetrahedronFarthest2 = new Point3D(1.0, -1.0, 0.0);

         Vector3D translation = new Vector3D();
         translation.setX(EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0)); // Positive => no collision
         translation.setY(EuclidCoreRandomTools.nextDouble(random, 0.5));
         translation.setZ(EuclidCoreRandomTools.nextDouble(random, 0.5));

         Arrays.asList(tetrahedronClosest, tetrahedronFarthest0, tetrahedronFarthest1, tetrahedronFarthest2).forEach(p -> p.add(translation));

         ConvexPolytope3D tetrahedron = new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(tetrahedronClosest,
                                                                                                 tetrahedronFarthest0,
                                                                                                 tetrahedronFarthest1,
                                                                                                 tetrahedronFarthest2));

         double distance = Double.POSITIVE_INFINITY;

         for (Vertex3D tetraVertex : tetrahedron.getVertices())
         {
            assertFalse(cube.isPointInside(tetraVertex));

            for (Face3D cubeFace : cube.getFaces())
            {
               distance = Math.min(distance, cubeFace.distance(tetraVertex));
            }
         }

         for (Vertex3D cubeVertex : cube.getVertices())
         {
            assertFalse(tetrahedron.isPointInside(cubeVertex));

            for (Face3D tetraFace : tetrahedron.getFaces())
            {
               distance = Math.min(distance, tetraFace.distance(cubeVertex));
            }
         }

         assertEquals(translation.getX(), distance, EPSILON);

         ExpandingPolytopeAlgorithm epa = new ExpandingPolytopeAlgorithm();
         EuclidShape3DCollisionResult result = epa.evaluateCollision(cube, tetrahedron);
         Point3D pointOnCube = result.getPointOnA();
         Point3D pointOnTetrahedron = result.getPointOnB();

         double separatingDistance = result.getSignedDistance();
         assertEquals(0.0, cube.distance(pointOnCube), EPSILON);
         assertEquals(0.0, tetrahedron.distance(pointOnTetrahedron), EPSILON);
         assertEquals(cube.distance(tetrahedronClosest), separatingDistance, EPSILON);

         EuclidCoreTestTools.assertEquals(tetrahedronClosest, pointOnTetrahedron, EPSILON);
         assertEquals(tetrahedronClosest.getY(), pointOnCube.getY(), EPSILON);
         assertEquals(tetrahedronClosest.getZ(), pointOnCube.getZ(), EPSILON);
      }
   }

   @Test
   void testCollidingCubeAndPoint()
   {
      Random random = new Random(34534);

      ConvexPolytope3D cube = EuclidPolytopeFactories.newCube(1.0);

      { // Predefined example
         Point3D point = new Point3D(0.25, 0.0, 0.0);
         ConvexPolytope3D singleton = new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(point));

         ExpandingPolytopeAlgorithm epa = new ExpandingPolytopeAlgorithm();
         EuclidShape3DCollisionResult result = epa.evaluateCollision(cube, singleton);
         Point3D pointOnCube = result.getPointOnA();
         Point3D pointOnSingleton = result.getPointOnB();

         assertEquals(0.0, cube.distance(pointOnCube), EPSILON);
         assertEquals(0.0, singleton.distance(pointOnSingleton), EPSILON);
         assertEquals(0.25, -result.getSignedDistance(), EPSILON);
         assertEquals(pointOnCube.distance(pointOnSingleton), -result.getSignedDistance(), EPSILON);
         assertEquals(cube.signedDistance(point), result.getSignedDistance(), EPSILON);

         EuclidCoreTestTools.assertEquals(point, pointOnSingleton, EPSILON);
         assertEquals(point.getY(), pointOnCube.getY(), EPSILON);
         assertEquals(point.getZ(), pointOnCube.getZ(), EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         Point3D point = EuclidCoreRandomTools.nextPoint3D(random, -0.5, 0.5);
         ConvexPolytope3D tetrahedron = new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(point));

         ExpandingPolytopeAlgorithm epa = new ExpandingPolytopeAlgorithm();
         EuclidShape3DCollisionResult result = epa.evaluateCollision(cube, tetrahedron);
         Point3D pointOnCube = result.getPointOnA();
         Point3D pointOnSingleton = result.getPointOnB();

         assertEquals(pointOnCube.distance(pointOnSingleton), -result.getSignedDistance(), EPSILON);
         assertEquals(cube.signedDistance(point), result.getSignedDistance(), EPSILON);
         assertEquals(0.0, cube.distance(pointOnCube), EPSILON);
         assertEquals(0.0, tetrahedron.distance(pointOnSingleton), EPSILON);

         EuclidCoreTestTools.assertEquals(point, pointOnSingleton, EPSILON);
      }
   }

   @Test
   void testCollidingCubeAndTetrahedron()
   {
      Random random = new Random(34534);

      ConvexPolytope3D cube = EuclidPolytopeFactories.newCube(1.0);

      for (int i = 0; i < ITERATIONS; i++)
      { // Superficial collision, the result is straightforward
         Point3D tetrahedronClosest = new Point3D(0.5, 0.0, 0.0);
         Point3D tetrahedronFarthest0 = new Point3D(100.0, 0.02, 0.0);
         Point3D tetrahedronFarthest1 = new Point3D(100.0, 0.0, 0.02);
         Point3D tetrahedronFarthest2 = new Point3D(100.0, -0.02, 0.0);

         Vector3D translation = new Vector3D();
         translation.setX(EuclidCoreRandomTools.nextDouble(random, -0.25, 0.0)); // Negative => collision
         translation.setY(EuclidCoreRandomTools.nextDouble(random, 0.25));
         translation.setZ(EuclidCoreRandomTools.nextDouble(random, 0.25));

         Arrays.asList(tetrahedronClosest, tetrahedronFarthest0, tetrahedronFarthest1, tetrahedronFarthest2).forEach(p -> p.add(translation));

         ConvexPolytope3D tetrahedron = new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(tetrahedronClosest,
                                                                                                 tetrahedronFarthest0,
                                                                                                 tetrahedronFarthest1,
                                                                                                 tetrahedronFarthest2));

         ExpandingPolytopeAlgorithm epa = new ExpandingPolytopeAlgorithm();
         EuclidShape3DCollisionResult result = epa.evaluateCollision(cube, tetrahedron);
         Point3D pointOnCube = result.getPointOnA();
         Point3D pointOnTetrahedron = result.getPointOnB();

         assertEquals(pointOnCube.distance(pointOnTetrahedron), -result.getSignedDistance(), EPSILON);
         assertEquals(0.0, cube.distance(pointOnCube), EPSILON);
         assertEquals(0.0, tetrahedron.distance(pointOnTetrahedron), EPSILON);
         EuclidCoreTestTools.assertEquals(tetrahedronClosest, pointOnTetrahedron, EPSILON);

         assertEquals(cube.signedDistance(tetrahedronClosest), result.getSignedDistance(), EPSILON);
         assertEquals(tetrahedronClosest.getY(), pointOnCube.getY(), EPSILON);
         assertEquals(tetrahedronClosest.getZ(), pointOnCube.getZ(), EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Deep collision, check that EPA can be used to resolve collision
         Point3D tetrahedronClosest = EuclidCoreRandomTools.nextPoint3D(random, 0.5);
         Point3D tetrahedronFarthest0 = new Point3D(1.0, 1.2, 0.0);
         Point3D tetrahedronFarthest1 = new Point3D(1.0, 0.0, 1.2);
         Point3D tetrahedronFarthest2 = new Point3D(1.0, -1.2, 0.0);

         ConvexPolytope3D tetrahedron = new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(tetrahedronClosest,
                                                                                                 tetrahedronFarthest0,
                                                                                                 tetrahedronFarthest1,
                                                                                                 tetrahedronFarthest2));

         assertResolvingCollision("Iteration " + i, cube, tetrahedron);
      }
   }

   @Test
   void testNonCollidingConvexPolytope3DWithTetrahedron() throws Exception
   {
      Random random = new Random(45345);

      for (int i = 0; i < ITERATIONS; i++)
      {
         ConvexPolytope3D convexPolytope3D = EuclidShapeRandomTools.nextConvexPolytope3DWithEdgeCases(random);
         { // Create the tetrahedron to have its top vertex closest to a face.
            if (convexPolytope3D.isEmpty())
            {
               performAssertionsOnEPA(random, convexPolytope3D, EuclidShapeRandomTools.nextConeConvexPolytope3D(random), null, null);
               performAssertionsOnEPA(random, EuclidShapeRandomTools.nextConeConvexPolytope3D(random), convexPolytope3D, null, null);
            }
            else
            {
               Face3D face = convexPolytope3D.getFace(random.nextInt(convexPolytope3D.getNumberOfFaces()));
               HalfEdge3D edge = face.getEdge(random.nextInt(face.getNumberOfEdges()));
               Point3D closestOnConvexPolytope3D = EuclidGeometryRandomTools.nextPoint3DInTriangle(random,
                                                                                                   face.getCentroid(),
                                                                                                   edge.getOrigin(),
                                                                                                   edge.getDestination());
               Point3D closestOnTetrahedron = new Point3D(closestOnConvexPolytope3D);
               closestOnTetrahedron.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0), face.getNormal(), closestOnTetrahedron);

               ConvexPolytope3D tetrahedron = GilbertJohnsonKeerthiCollisionDetectorTest.newTetrahedron(random, closestOnTetrahedron, face.getNormal(), 1.0);

               performAssertionsOnEPA(random, convexPolytope3D, tetrahedron, closestOnConvexPolytope3D, closestOnTetrahedron);
            }
         }

         { // Create the tetrahedron to have its top vertex closest to an edge.
            if (convexPolytope3D.isEmpty())
            {
               performAssertionsOnEPA(random, convexPolytope3D, EuclidShapeRandomTools.nextConeConvexPolytope3D(random), null, null);
               performAssertionsOnEPA(random, EuclidShapeRandomTools.nextConeConvexPolytope3D(random), convexPolytope3D, null, null);
            }
            else
            {
               Face3D firstFace = convexPolytope3D.getFace(random.nextInt(convexPolytope3D.getNumberOfFaces()));
               HalfEdge3D closestEdge = firstFace.getEdge(random.nextInt(firstFace.getNumberOfEdges()));

               Vector3D towardOutside = new Vector3D();
               if (closestEdge.getTwin() != null)
               {
                  Face3D secondFace = closestEdge.getTwin().getFace();
                  towardOutside.interpolate(firstFace.getNormal(), secondFace.getNormal(), EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));
               }
               else
               {
                  Vector3D faceNormal = new Vector3D(firstFace.getNormal());
                  towardOutside.cross(faceNormal, closestEdge.getDirection(false));
                  if (random.nextBoolean())
                     faceNormal.negate();
                  towardOutside.interpolate(faceNormal, EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));
               }

               towardOutside.normalize();

               Point3D pointOnEdge = new Point3D();
               pointOnEdge.interpolate(closestEdge.getOrigin(), closestEdge.getDestination(), EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));
               Point3D pointOutside = new Point3D();
               pointOutside.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), towardOutside, pointOnEdge);

               ConvexPolytope3D tetrahedron = GilbertJohnsonKeerthiCollisionDetectorTest.newTetrahedron(random, pointOutside, towardOutside, 1.0);

               performAssertionsOnEPA(random, convexPolytope3D, tetrahedron, pointOnEdge, pointOutside);
            }
         }

         { // Point outside closest to a vertex
            if (convexPolytope3D.isEmpty())
            {
               performAssertionsOnEPA(random, convexPolytope3D, EuclidShapeRandomTools.nextConeConvexPolytope3D(random), null, null);
               performAssertionsOnEPA(random, EuclidShapeRandomTools.nextConeConvexPolytope3D(random), convexPolytope3D, null, null);
            }
            else
            {
               Vertex3D closestVertex = convexPolytope3D.getVertex(random.nextInt(convexPolytope3D.getNumberOfVertices()));

               Vector3D towardOutside = new Vector3D();
               closestVertex.getAssociatedEdges()
                            .stream()
                            .forEach(edge -> towardOutside.scaleAdd(random.nextDouble(), edge.getFace().getNormal(), towardOutside));
               towardOutside.normalize();

               Point3D pointOutside = new Point3D();
               pointOutside.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0), towardOutside, closestVertex);

               ConvexPolytope3D tetrahedron = GilbertJohnsonKeerthiCollisionDetectorTest.newTetrahedron(random, pointOutside, towardOutside, 1.0);

               performAssertionsOnEPA(random, convexPolytope3D, tetrahedron, closestVertex, pointOutside);
            }
         }
      }
   }

   @Test
   void testVertexCollidingConvexPolytope3DWithTetrahedron() throws Exception
   {
      Random random = new Random(453451);

      for (int i = 0; i < ITERATIONS; i++)
      {
         ConvexPolytope3D convexPolytope3D = EuclidShapeRandomTools.nextConvexPolytope3DWithEdgeCases(random);
         { // Create the tetrahedron from its top vertex lying inside the polytope
            ConvexPolytope3D tetrahedron;

            if (convexPolytope3D.isEmpty())
            {
               performAssertionsOnEPA(random, convexPolytope3D, EuclidShapeRandomTools.nextConvexPolytope3D(random), null, null);
            }
            else
            {
               if (convexPolytope3D.getNumberOfVertices() == 1)
               {
                  tetrahedron = EuclidShapeRandomTools.nextTetrahedronContainingPoint3D(random, convexPolytope3D.getVertex(0));
                  assertTrue(tetrahedron.isPointInside(convexPolytope3D.getVertex(0)));
               }
               else if (convexPolytope3D.getNumberOfVertices() == 2)
               {
                  HalfEdge3D edge = convexPolytope3D.getHalfEdge(0);
                  Vector3DBasics edgeDirection = edge.getDirection(true);
                  Vector3D firstOrthogonalToEdge = EuclidCoreRandomTools.nextOrthogonalVector3D(random, edgeDirection, true);
                  Vector3D secondOrthogonalToEdge = new Vector3D();
                  secondOrthogonalToEdge.cross(firstOrthogonalToEdge, edgeDirection);

                  LineSegment3D firstTetraSegment = new LineSegment3D();
                  firstTetraSegment.getFirstEndpoint().scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.1, 10.0), edgeDirection, edge.midpoint());
                  firstTetraSegment.getSecondEndpoint().scaleAdd(-EuclidCoreRandomTools.nextDouble(random, 0.1, 10.0), edgeDirection, edge.midpoint());
                  firstTetraSegment.translate(firstOrthogonalToEdge);
                  LineSegment3D secondTetraSegment = new LineSegment3D();
                  secondTetraSegment.getFirstEndpoint().scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.1, 10.0), secondOrthogonalToEdge, edge.midpoint());
                  secondTetraSegment.getSecondEndpoint()
                                    .scaleAdd(-EuclidCoreRandomTools.nextDouble(random, 0.1, 10.0), secondOrthogonalToEdge, edge.midpoint());
                  firstOrthogonalToEdge.negate();
                  secondTetraSegment.translate(firstOrthogonalToEdge);
                  tetrahedron = new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(firstTetraSegment.getFirstEndpoint(),
                                                                                         firstTetraSegment.getSecondEndpoint(),
                                                                                         secondTetraSegment.getFirstEndpoint(),
                                                                                         secondTetraSegment.getSecondEndpoint()));
               }
               else if (convexPolytope3D.getNumberOfFaces() == 1)
               {
                  Face3D face = convexPolytope3D.getFace(0);
                  Point3D pointOnFace = EuclidShapeRandomTools.nextPoint3DOnFace3D(random, face);
                  tetrahedron = EuclidShapeRandomTools.nextTetrahedronContainingPoint3D(random, pointOnFace);
               }
               else
               {
                  Face3D face = convexPolytope3D.getFace(random.nextInt(convexPolytope3D.getNumberOfFaces()));
                  HalfEdge3D edge = face.getEdge(random.nextInt(face.getNumberOfEdges()));
                  Point3D pointInside = EuclidGeometryRandomTools.nextPoint3DInTetrahedron(random,
                                                                                           convexPolytope3D.getCentroid(),
                                                                                           face.getCentroid(),
                                                                                           edge.getOrigin(),
                                                                                           edge.getDestination());

                  tetrahedron = GilbertJohnsonKeerthiCollisionDetectorTest.newTetrahedron(random, pointInside, face.getNormal(), 1.0);
               }

               assertResolvingCollision("Iteration " + i, convexPolytope3D, tetrahedron);
            }
         }

         { // Create the tetrahedron from its top vertex: 1- make it lie on an edge, 2- go inside slightly
            ConvexPolytope3D tetrahedron;

            if (convexPolytope3D.isEmpty())
            {
               performAssertionsOnEPA(random, convexPolytope3D, EuclidShapeRandomTools.nextConvexPolytope3D(random), null, null);
            }
            else
            {
               if (convexPolytope3D.getNumberOfVertices() == 1)
               {
                  tetrahedron = EuclidShapeRandomTools.nextTetrahedronContainingPoint3D(random, convexPolytope3D.getVertex(0));
                  assertTrue(tetrahedron.isPointInside(convexPolytope3D.getVertex(0)));
               }
               else if (convexPolytope3D.getNumberOfVertices() == 2)
               {
                  HalfEdge3D edge = convexPolytope3D.getHalfEdge(0);
                  Vector3DBasics edgeDirection = edge.getDirection(true);
                  Vector3D firstOrthogonalToEdge = EuclidCoreRandomTools.nextOrthogonalVector3D(random, edgeDirection, true);
                  Vector3D secondOrthogonalToEdge = new Vector3D();
                  secondOrthogonalToEdge.cross(firstOrthogonalToEdge, edgeDirection);

                  LineSegment3D firstTetraSegment = new LineSegment3D();
                  firstTetraSegment.getFirstEndpoint().scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.1, 10.0), edgeDirection, edge.midpoint());
                  firstTetraSegment.getSecondEndpoint().scaleAdd(-EuclidCoreRandomTools.nextDouble(random, 0.1, 10.0), edgeDirection, edge.midpoint());
                  firstTetraSegment.translate(firstOrthogonalToEdge);
                  LineSegment3D secondTetraSegment = new LineSegment3D();
                  secondTetraSegment.getFirstEndpoint().scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.1, 10.0), secondOrthogonalToEdge, edge.midpoint());
                  secondTetraSegment.getSecondEndpoint()
                                    .scaleAdd(-EuclidCoreRandomTools.nextDouble(random, 0.1, 10.0), secondOrthogonalToEdge, edge.midpoint());
                  firstOrthogonalToEdge.negate();
                  secondTetraSegment.translate(firstOrthogonalToEdge);
                  tetrahedron = new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(firstTetraSegment.getFirstEndpoint(),
                                                                                         firstTetraSegment.getSecondEndpoint(),
                                                                                         secondTetraSegment.getFirstEndpoint(),
                                                                                         secondTetraSegment.getSecondEndpoint()));
               }
               else if (convexPolytope3D.getNumberOfFaces() == 1)
               {
                  Face3D face = convexPolytope3D.getFace(0);
                  Point3D pointOnFace = EuclidShapeRandomTools.nextPoint3DOnFace3D(random, face);
                  tetrahedron = EuclidShapeRandomTools.nextTetrahedronContainingPoint3D(random, pointOnFace);
               }
               else
               {
                  Face3D firstFace = convexPolytope3D.getFace(random.nextInt(convexPolytope3D.getNumberOfFaces()));
                  HalfEdge3D closestEdge = firstFace.getEdge(random.nextInt(firstFace.getNumberOfEdges()));

                  Point3D pointOnEdge = new Point3D();
                  pointOnEdge.interpolate(closestEdge.getOrigin(), closestEdge.getDestination(), EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));

                  Vector3D towardInside = new Vector3D();
                  towardInside.sub(convexPolytope3D.getCentroid(), pointOnEdge);
                  towardInside.normalize();

                  Point3D pointInside = new Point3D();
                  double distanceInside = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0e-3);
                  pointInside.scaleAdd(distanceInside, towardInside, pointOnEdge);

                  tetrahedron = GilbertJohnsonKeerthiCollisionDetectorTest.newTetrahedron(random, pointInside, towardInside, 1.0);
               }

               assertResolvingCollision("Iteration " + i, convexPolytope3D, tetrahedron);
            }
         }

         { // Create the tetrahedron from its top vertex: 1- make it to be at a vertex, 2- go inside slightly
            if (convexPolytope3D.isEmpty())
            {
               performAssertionsOnEPA(random, convexPolytope3D, EuclidShapeRandomTools.nextConvexPolytope3D(random), null, null);
            }
            else
            {
               ConvexPolytope3D tetrahedron;

               if (convexPolytope3D.getNumberOfVertices() == 1)
               {
                  tetrahedron = EuclidShapeRandomTools.nextTetrahedronContainingPoint3D(random, convexPolytope3D.getVertex(0));
                  assertTrue(tetrahedron.isPointInside(convexPolytope3D.getVertex(0)));
               }
               else if (convexPolytope3D.getNumberOfVertices() == 2)
               {
                  HalfEdge3D edge = convexPolytope3D.getHalfEdge(0);
                  Vector3DBasics edgeDirection = edge.getDirection(true);
                  Vector3D firstOrthogonalToEdge = EuclidCoreRandomTools.nextOrthogonalVector3D(random, edgeDirection, true);
                  Vector3D secondOrthogonalToEdge = new Vector3D();
                  secondOrthogonalToEdge.cross(firstOrthogonalToEdge, edgeDirection);

                  LineSegment3D firstTetraSegment = new LineSegment3D();
                  firstTetraSegment.getFirstEndpoint().scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.1, 10.0), edgeDirection, edge.midpoint());
                  firstTetraSegment.getSecondEndpoint().scaleAdd(-EuclidCoreRandomTools.nextDouble(random, 0.1, 10.0), edgeDirection, edge.midpoint());
                  firstTetraSegment.translate(firstOrthogonalToEdge);
                  LineSegment3D secondTetraSegment = new LineSegment3D();
                  secondTetraSegment.getFirstEndpoint().scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.1, 10.0), secondOrthogonalToEdge, edge.midpoint());
                  secondTetraSegment.getSecondEndpoint()
                                    .scaleAdd(-EuclidCoreRandomTools.nextDouble(random, 0.1, 10.0), secondOrthogonalToEdge, edge.midpoint());
                  firstOrthogonalToEdge.negate();
                  secondTetraSegment.translate(firstOrthogonalToEdge);
                  tetrahedron = new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(firstTetraSegment.getFirstEndpoint(),
                                                                                         firstTetraSegment.getSecondEndpoint(),
                                                                                         secondTetraSegment.getFirstEndpoint(),
                                                                                         secondTetraSegment.getSecondEndpoint()));
               }
               else if (convexPolytope3D.getNumberOfFaces() == 1)
               {
                  Face3D face = convexPolytope3D.getFace(0);
                  Point3D pointOnFace = EuclidShapeRandomTools.nextPoint3DOnFace3D(random, face);
                  tetrahedron = EuclidShapeRandomTools.nextTetrahedronContainingPoint3D(random, pointOnFace);
               }
               else
               {
                  Vertex3D closestVertex = convexPolytope3D.getVertex(random.nextInt(convexPolytope3D.getNumberOfVertices()));

                  Vector3D towardInside = new Vector3D();
                  towardInside.sub(convexPolytope3D.getCentroid(), closestVertex);

                  Point3D pointInside = new Point3D();
                  double distanceInside = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0e-3);
                  pointInside.scaleAdd(distanceInside, towardInside, closestVertex);

                  tetrahedron = GilbertJohnsonKeerthiCollisionDetectorTest.newTetrahedron(random, pointInside, towardInside, 1.0);
               }

               assertResolvingCollision("Iteration " + i, convexPolytope3D, tetrahedron);
            }
         }
      }
   }

   public static void performAssertionsOnEPA(Random random,
                                             ConvexPolytope3DReadOnly polytopeA,
                                             ConvexPolytope3DReadOnly polytopeB,
                                             Point3DReadOnly expectedPointOnA,
                                             Point3DReadOnly expectedPointOnB)
   {
      ExpandingPolytopeAlgorithm epa = new ExpandingPolytopeAlgorithm();
      EuclidShape3DCollisionResult result = epa.evaluateCollision(polytopeA, polytopeB);
      assertTrue(polytopeA == result.getShapeA());
      assertTrue(polytopeB == result.getShapeB());

      if (polytopeA.isEmpty() || polytopeB.isEmpty())
      {
         assertFalse(result.areShapesColliding());
         assertTrue(Double.isNaN(result.getSignedDistance()));
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(result.getPointOnA());
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(result.getPointOnB());
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(result.getNormalOnA());
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(result.getNormalOnB());
      }
      else
      {
         Point3D actualPointOnA = result.getPointOnA();
         Point3D actualPointOnB = result.getPointOnB();

         assertEquals(0.0, polytopeA.distance(actualPointOnA), EPSILON);
         assertEquals(0.0, polytopeB.distance(actualPointOnB), EPSILON);
         EuclidCoreTestTools.assertEquals(expectedPointOnA, actualPointOnA, 10.0 * EPSILON);
         EuclidCoreTestTools.assertEquals(expectedPointOnB, actualPointOnB, 10.0 * EPSILON);
      }
   }

   public static void assertResolvingCollision(String messagePrefix, ConvexPolytope3DReadOnly polytopeA, ConvexPolytope3DReadOnly polytopeB)
   {
      GilbertJohnsonKeerthiCollisionDetector gjkDetector = new GilbertJohnsonKeerthiCollisionDetector();
      EuclidShape3DCollisionResult result = gjkDetector.evaluateCollision(polytopeA, polytopeB);
      assertTrue(result.areShapesColliding());

      ExpandingPolytopeAlgorithm epa = new ExpandingPolytopeAlgorithm();
      result = epa.evaluateCollision(polytopeA, polytopeB);
      Point3D pointOnA = result.getPointOnA();
      Point3D pointOnB = result.getPointOnB();

      assertEquals(0.0, polytopeA.distance(pointOnA), EPSILON);
      assertEquals(0.0, polytopeB.distance(pointOnB), EPSILON);

      Vector3D collisionVector = new Vector3D();
      collisionVector.sub(pointOnA, pointOnB);
      Vector3D augmentedCollisionVector = new Vector3D();
      augmentedCollisionVector.setAndScale(0.99, collisionVector);
      ConvexPolytope3D polytopeBTranslated = new ConvexPolytope3D(polytopeB);
      polytopeBTranslated.applyTransform(new RigidBodyTransform(new Quaternion(), augmentedCollisionVector));
      // We translate the polytopeB but not enough to resolve the collision
      result = gjkDetector.evaluateCollision(polytopeA, polytopeBTranslated);
      assertTrue(result.areShapesColliding(), messagePrefix);

      augmentedCollisionVector.setAndNormalize(collisionVector);
      augmentedCollisionVector.scale(Math.max(1.0e-4, 0.01 * collisionVector.norm()) + collisionVector.norm());
      polytopeBTranslated = new ConvexPolytope3D(polytopeB);
      polytopeBTranslated.applyTransform(new RigidBodyTransform(new Quaternion(), augmentedCollisionVector));
      // We translate the polytopeB just enough to resolve the collision
      result = gjkDetector.evaluateCollision(polytopeA, polytopeBTranslated);
      assertFalse(result.areShapesColliding(), messagePrefix);
   }

   @Test
   void testSphere3DToSphere3D() throws Exception
   { // This test confirms that GJK-EPA can be used with primitives too, and also serves as benchmark for accuracy.
      Random random = new Random(1382635);

      double distanceMaxEpsilon = 1.0e-7;
      double positionMaxEpsilon = 1.0e-3;

      double distanceMeanEpsilon = 1.0e-12;
      double positionMeanEpsilon = 1.0e-6;

      double depthThresholdCollisionDetection = 0.0;

      AnalyticalShapeCollisionDetection<Sphere3D, Sphere3D> function = new AnalyticalShapeCollisionDetection<>(() ->
                                                                                                               {
                                                                                                                  Sphere3D sphereA = EuclidShapeRandomTools.nextSphere3D(
                                                                                                                        random);
                                                                                                                  Sphere3D sphereB = EuclidShapeRandomTools.nextSphere3D(
                                                                                                                        random);
                                                                                                                  return new Pair<>(sphereA, sphereB);
                                                                                                               },
                                                                                                               EuclidShapeCollisionTools::evaluateSphere3DSphere3DCollision);

      assertAgainstAnalyticalFunction(function,
                                      distanceMaxEpsilon,
                                      positionMaxEpsilon,
                                      distanceMeanEpsilon,
                                      positionMeanEpsilon,
                                      depthThresholdCollisionDetection);
   }

   @Test
   void testPointShape3DBox3DCollisionTest() throws Exception
   { // This test confirms that GJK-EPA can be used with primitives too, and also serves as benchmark for accuracy.
      Random random = new Random(13741);

      double distanceMaxEpsilon = 1.0e-15;
      double positionMaxEpsilon = 1.0e-15;

      double distanceMeanEpsilon = 1.0e-18;
      double positionMeanEpsilon = 1.0e-17;

      double depthThresholdCollisionDetection = 0.0;

      AnalyticalShapeCollisionDetection<PointShape3D, Box3D> function = new AnalyticalShapeCollisionDetection<>(() ->
                                                                                                                {
                                                                                                                   PointShape3D shapeA = EuclidShapeRandomTools.nextPointShape3D(
                                                                                                                         random);
                                                                                                                   Box3D shapeB = EuclidShapeRandomTools.nextBox3D(
                                                                                                                         random);
                                                                                                                   return new Pair<>(shapeA, shapeB);
                                                                                                                },
                                                                                                                EuclidShapeCollisionTools::evaluatePointShape3DBox3DCollision);

      assertAgainstAnalyticalFunction(function,
                                      distanceMaxEpsilon,
                                      positionMaxEpsilon,
                                      distanceMeanEpsilon,
                                      positionMeanEpsilon,
                                      depthThresholdCollisionDetection);
   }

   @Test
   void testSphere3DBox3DCollisionTest() throws Exception
   { // This test confirms that GJK-EPA can be used with primitives too, and also serves as benchmark for accuracy.
      Random random = new Random(13741);

      double distanceMaxEpsilon = 1.0e-8;
      double positionMaxEpsilon = 1.0e-3;

      double distanceMeanEpsilon = 1.0e-10;
      double positionMeanEpsilon = 1.0e-6;

      double depthThresholdCollisionDetection = 0.0;

      AnalyticalShapeCollisionDetection<Sphere3D, Box3D> function = new AnalyticalShapeCollisionDetection<>(() ->
                                                                                                            {
                                                                                                               Sphere3D shapeA = EuclidShapeRandomTools.nextSphere3D(
                                                                                                                     random);
                                                                                                               Box3D shapeB = EuclidShapeRandomTools.nextBox3D(
                                                                                                                     random);
                                                                                                               return new Pair<>(shapeA, shapeB);
                                                                                                            },
                                                                                                            EuclidShapeCollisionTools::evaluateSphere3DBox3DCollision);

      assertAgainstAnalyticalFunction(function,
                                      distanceMaxEpsilon,
                                      positionMaxEpsilon,
                                      distanceMeanEpsilon,
                                      positionMeanEpsilon,
                                      depthThresholdCollisionDetection);
   }

   @Test
   void testPointShape3DCapsule3DCollisionTest() throws Exception
   { // This test confirms that GJK-EPA can be used with primitives too, and also serves as benchmark for accuracy.
      Random random = new Random(13741);

      double distanceMaxEpsilon = 1.0e-8;
      double positionMaxEpsilon = 1.0e-3;

      double distanceMeanEpsilon = 1.0e-11;
      double positionMeanEpsilon = 1.0e-6;

      double depthThresholdCollisionDetection = 0.0;

      AnalyticalShapeCollisionDetection<PointShape3D, Capsule3D> function = new AnalyticalShapeCollisionDetection<>(() ->
                                                                                                                    {
                                                                                                                       PointShape3D shapeA = EuclidShapeRandomTools.nextPointShape3D(
                                                                                                                             random);
                                                                                                                       Capsule3D shapeB = EuclidShapeRandomTools.nextCapsule3D(
                                                                                                                             random);
                                                                                                                       return new Pair<>(shapeA, shapeB);
                                                                                                                    },
                                                                                                                    EuclidShapeCollisionTools::evaluatePointShape3DCapsule3DCollision);

      assertAgainstAnalyticalFunction(function,
                                      distanceMaxEpsilon,
                                      positionMaxEpsilon,
                                      distanceMeanEpsilon,
                                      positionMeanEpsilon,
                                      depthThresholdCollisionDetection);
   }

   @Test
   void testCapsule3DCapsule3DCollisionTest() throws Exception
   { // This test confirms that GJK-EPA can be used with primitives too, and also serves as benchmark for accuracy.
      Random random = new Random(13741);

      double distanceMaxEpsilon = 1.0e-8;
      double positionMaxEpsilon = 1.0e-3;

      double distanceMeanEpsilon = 1.0e-10;
      double positionMeanEpsilon = 1.0e-6;

      double depthThresholdCollisionDetection = 0.0;

      AnalyticalShapeCollisionDetection<Capsule3D, Capsule3D> function = new AnalyticalShapeCollisionDetection<>(() ->
                                                                                                                 {
                                                                                                                    Capsule3D shapeA = EuclidShapeRandomTools.nextCapsule3D(
                                                                                                                          random);
                                                                                                                    Capsule3D shapeB = EuclidShapeRandomTools.nextCapsule3D(
                                                                                                                          random);
                                                                                                                    return new Pair<>(shapeA, shapeB);
                                                                                                                 },
                                                                                                                 EuclidShapeCollisionTools::evaluateCapsule3DCapsule3DCollision);

      assertAgainstAnalyticalFunction(function,
                                      distanceMaxEpsilon,
                                      positionMaxEpsilon,
                                      distanceMeanEpsilon,
                                      positionMeanEpsilon,
                                      depthThresholdCollisionDetection);
   }

   @Test
   void testSphere3DCapsule3DCollisionTest() throws Exception
   { // This test confirms that GJK-EPA can be used with primitives too, and also serves as benchmark for accuracy.
      Random random = new Random(13741);

      double distanceMaxEpsilon = 1.0e-8;
      double positionMaxEpsilon = 1.0e-3;

      double distanceMeanEpsilon = 1.0e-10;
      double positionMeanEpsilon = 1.0e-6;

      double depthThresholdCollisionDetection = 0.0;

      AnalyticalShapeCollisionDetection<Sphere3D, Capsule3D> function = new AnalyticalShapeCollisionDetection<>(() ->
                                                                                                                {
                                                                                                                   Sphere3D shapeA = EuclidShapeRandomTools.nextSphere3D(
                                                                                                                         random);
                                                                                                                   Capsule3D shapeB = EuclidShapeRandomTools.nextCapsule3D(
                                                                                                                         random);
                                                                                                                   return new Pair<>(shapeA, shapeB);
                                                                                                                },
                                                                                                                EuclidShapeCollisionTools::evaluateSphere3DCapsule3DCollision);

      assertAgainstAnalyticalFunction(function,
                                      distanceMaxEpsilon,
                                      positionMaxEpsilon,
                                      distanceMeanEpsilon,
                                      positionMeanEpsilon,
                                      depthThresholdCollisionDetection);
   }

   @Test
   void testPointShape3DCylinder3DCollisionTest() throws Exception
   { // This test confirms that GJK-EPA can be used with primitives too, and also serves as benchmark for accuracy.
      Random random = new Random(13741);

      double distanceMaxEpsilon = 1.0e-7;
      double positionMaxEpsilon = 1.0e-4;

      double distanceMeanEpsilon = 1.0e-10;
      double positionMeanEpsilon = 1.0e-7;

      double depthThresholdCollisionDetection = 0.0;

      AnalyticalShapeCollisionDetection<PointShape3D, Cylinder3D> function = new AnalyticalShapeCollisionDetection<>(() ->
                                                                                                                     {
                                                                                                                        PointShape3D shapeA = EuclidShapeRandomTools.nextPointShape3D(
                                                                                                                              random);
                                                                                                                        Cylinder3D shapeB = EuclidShapeRandomTools.nextCylinder3D(
                                                                                                                              random);
                                                                                                                        return new Pair<>(shapeA, shapeB);
                                                                                                                     },
                                                                                                                     EuclidShapeCollisionTools::evaluatePointShape3DCylinder3DCollision);

      assertAgainstAnalyticalFunction(function,
                                      distanceMaxEpsilon,
                                      positionMaxEpsilon,
                                      distanceMeanEpsilon,
                                      positionMeanEpsilon,
                                      depthThresholdCollisionDetection);
   }

   @Test
      // TODO Sphere <-> Cylinder seems to be inaccurate compared to the other shape combinations.
   void testSphere3DCylinder3DCollisionTest() throws Exception
   { // This test confirms that GJK-EPA can be used with primitives too, and also serves as benchmark for accuracy.
      Random random = new Random(13741);

      double distanceMaxEpsilon = 1.0e-2;
      double positionMaxEpsilon = 3.0e-2;

      double distanceMeanEpsilon = 2.0e-7;
      double positionMeanEpsilon = 5.0e-5;

      double depthThresholdCollisionDetection = 0.0;

      AnalyticalShapeCollisionDetection<Sphere3D, Cylinder3D> function = new AnalyticalShapeCollisionDetection<>(() ->
                                                                                                                 {
                                                                                                                    Sphere3D shapeA = EuclidShapeRandomTools.nextSphere3D(
                                                                                                                          random);
                                                                                                                    Cylinder3D shapeB = EuclidShapeRandomTools.nextCylinder3D(
                                                                                                                          random);
                                                                                                                    return new Pair<>(shapeA, shapeB);
                                                                                                                 },
                                                                                                                 EuclidShapeCollisionTools::evaluateSphere3DCylinder3DCollision);

      assertAgainstAnalyticalFunction(function,
                                      distanceMaxEpsilon,
                                      positionMaxEpsilon,
                                      distanceMeanEpsilon,
                                      positionMeanEpsilon,
                                      depthThresholdCollisionDetection);
   }

   @Test
   void testPointShape3DEllipsoid3DCollisionTest() throws Exception
   { // This test confirms that GJK-EPA can be used with primitives too, and also serves as benchmark for accuracy.
      Random random = new Random(13741);

      double distanceMaxEpsilon = 1.0e-10;
      double positionMaxEpsilon = 1.0e-4;

      double distanceMeanEpsilon = 1.0e-12;
      double positionMeanEpsilon = 1.0e-7;

      double depthThresholdCollisionDetection = 0.0;

      AnalyticalShapeCollisionDetection<PointShape3D, Ellipsoid3D> function = new AnalyticalShapeCollisionDetection<>(() ->
                                                                                                                      {
                                                                                                                         PointShape3D shapeA = EuclidShapeRandomTools.nextPointShape3D(
                                                                                                                               random);
                                                                                                                         Ellipsoid3D shapeB = EuclidShapeRandomTools.nextEllipsoid3D(
                                                                                                                               random);
                                                                                                                         return new Pair<>(shapeA, shapeB);
                                                                                                                      },
                                                                                                                      EuclidShapeCollisionTools::evaluatePointShape3DEllipsoid3DCollision);

      assertAgainstAnalyticalFunction(function,
                                      distanceMaxEpsilon,
                                      positionMaxEpsilon,
                                      distanceMeanEpsilon,
                                      positionMeanEpsilon,
                                      depthThresholdCollisionDetection);
   }

   @Test
   void testSphere3DEllipsoid3DCollisionTest() throws Exception
   { // This test confirms that GJK-EPA can be used with primitives too, and also serves as benchmark for accuracy.
      Random random = new Random(13741);

      double distanceMaxEpsilon = 1.0e-7;
      double positionMaxEpsilon = 1.0e-3;

      double distanceMeanEpsilon = 1.0e-11;
      double positionMeanEpsilon = 1.0e-6;

      double depthThresholdCollisionDetection = 0.0;

      AnalyticalShapeCollisionDetection<Sphere3D, Ellipsoid3D> function = new AnalyticalShapeCollisionDetection<>(() ->
                                                                                                                  {
                                                                                                                     Sphere3D shapeA = EuclidShapeRandomTools.nextSphere3D(
                                                                                                                           random);
                                                                                                                     Ellipsoid3D shapeB = EuclidShapeRandomTools.nextEllipsoid3D(
                                                                                                                           random);
                                                                                                                     return new Pair<>(shapeA, shapeB);
                                                                                                                  },
                                                                                                                  EuclidShapeCollisionTools::evaluateSphere3DEllipsoid3DCollision);

      assertAgainstAnalyticalFunction(function,
                                      distanceMaxEpsilon,
                                      positionMaxEpsilon,
                                      distanceMeanEpsilon,
                                      positionMeanEpsilon,
                                      depthThresholdCollisionDetection);
   }

   @Test
   void testPointShape3DRamp3DCollisionTest() throws Exception
   { // This test confirms that GJK-EPA can be used with primitives too, and also serves as benchmark for accuracy.
      Random random = new Random(13741);

      double distanceMaxEpsilon = 1.0e-14;
      double positionMaxEpsilon = 1.0e-14;

      double distanceMeanEpsilon = 1.0e-18;
      double positionMeanEpsilon = 1.0e-18;

      double depthThresholdCollisionDetection = 0.0;

      AnalyticalShapeCollisionDetection<PointShape3D, Ramp3D> function = new AnalyticalShapeCollisionDetection<>(() ->
                                                                                                                 {
                                                                                                                    PointShape3D shapeA = EuclidShapeRandomTools.nextPointShape3D(
                                                                                                                          random);
                                                                                                                    Ramp3D shapeB = EuclidShapeRandomTools.nextRamp3D(
                                                                                                                          random);
                                                                                                                    return new Pair<>(shapeA, shapeB);
                                                                                                                 },
                                                                                                                 EuclidShapeCollisionTools::evaluatePointShape3DRamp3DCollision);

      assertAgainstAnalyticalFunction(function,
                                      distanceMaxEpsilon,
                                      positionMaxEpsilon,
                                      distanceMeanEpsilon,
                                      positionMeanEpsilon,
                                      depthThresholdCollisionDetection);
   }

   @Test
   void testSphere3DRamp3DCollisionTest() throws Exception
   { // This test confirms that GJK-EPA can be used with primitives too, and also serves as benchmark for accuracy.
      Random random = new Random(13741);

      double distanceMaxEpsilon = 1.0e-8;
      double positionMaxEpsilon = 3.0e-4;

      double distanceMeanEpsilon = 1.0e-11;
      double positionMeanEpsilon = 1.0e-6;

      double depthThresholdCollisionDetection = 0.0;

      AnalyticalShapeCollisionDetection<Sphere3D, Ramp3D> function = new AnalyticalShapeCollisionDetection<>(() ->
                                                                                                             {
                                                                                                                Sphere3D shapeA = EuclidShapeRandomTools.nextSphere3D(
                                                                                                                      random);
                                                                                                                Ramp3D shapeB = EuclidShapeRandomTools.nextRamp3D(
                                                                                                                      random);
                                                                                                                return new Pair<>(shapeA, shapeB);
                                                                                                             },
                                                                                                             EuclidShapeCollisionTools::evaluateSphere3DRamp3DCollision);

      assertAgainstAnalyticalFunction(function,
                                      distanceMaxEpsilon,
                                      positionMaxEpsilon,
                                      distanceMeanEpsilon,
                                      positionMeanEpsilon,
                                      depthThresholdCollisionDetection);
   }

   @Test
   void testShapeTransformOptimization()
   { // Comparing the method with the Shape3DReadOnly arguments against the one with SupportingVertexHolder argument to validate the transformation optimization done in the first one.
      Random random = new Random(3466); // TODO changing this seed to 3456 causes the test to fail
      double distanceEpsilon = 5.0e-5;
      // When 2 "flat-ish" shapes are closest on their part with low curvature, the closest points can shift along the surface.
      double pointTangentialEpsilon = 1.0e-2;

      for (int i = 0; i < 10 * ITERATIONS; i++)
      {
         Shape3DBasics shapeA = EuclidShapeRandomTools.nextConvexShape3D(random);
         Shape3DBasics shapeB = EuclidShapeRandomTools.nextConvexShape3D(random);

         ExpandingPolytopeAlgorithm detector = new ExpandingPolytopeAlgorithm();
         EuclidShape3DCollisionResult expectedResult = detector.evaluateCollision((SupportingVertexHolder) shapeA, (SupportingVertexHolder) shapeB);
         expectedResult.setShapeA(shapeA);
         expectedResult.setShapeB(shapeB);
         EuclidShape3DCollisionResult actualResult = detector.evaluateCollision(shapeA, shapeB);

         EuclidShapeTestTools.assertEuclidShape3DCollisionResultGeometricallyEquals("Iteration " + i,
                                                                                    expectedResult,
                                                                                    actualResult,
                                                                                    distanceEpsilon,
                                                                                    pointTangentialEpsilon,
                                                                                    0.0);
      }
   }

   private static <A extends Shape3DReadOnly, B extends Shape3DReadOnly> void assertAgainstAnalyticalFunction(AnalyticalShapeCollisionDetection<A, B> function,
                                                                                                              double distanceMaxEpsilon,
                                                                                                              double positionMaxEpsilon,
                                                                                                              double distanceMeanEpsilon,
                                                                                                              double positionMeanEpsilon,
                                                                                                              double depthThresholdCollisionDetection)
   {
      boolean verbose = false;
      double meanDistanceError = 0.0;
      double meanPositionError = 0.0;
      double maxDistanceError = 0.0;
      double maxPositionError = 0.0;

      int numberOfCollidingSamples = 0;

      for (int i = 0; i < ITERATIONS; i++)
      {
         String iterationPrefix = "Iteration #" + i;
         Pair<A, B> shapes = function.shapeSupplier.get();
         A shapeA = shapes.a;
         B shapeB = shapes.b;

         EuclidShape3DCollisionResult expectedResult = function.collisionFunction.apply(shapeA, shapeB);
         EuclidShape3DCollisionResult epaResult = new EuclidShape3DCollisionResult();

         ExpandingPolytopeAlgorithm epaDetector = new ExpandingPolytopeAlgorithm();
         epaDetector.evaluateCollision(shapeA, shapeB, epaResult);

         double distanceError = Math.abs(expectedResult.getSignedDistance() - epaResult.getSignedDistance());
         if (verbose && i % 5000 == 0)
         {
            System.out.println(iterationPrefix + " Analytical: " + expectedResult.getSignedDistance() + ", EPA: " + epaResult.getSignedDistance() + ", diff: "
                               + distanceError);
         }

         if (expectedResult.getSignedDistance() > 0.0 || expectedResult.getSignedDistance() < -depthThresholdCollisionDetection)
         {
            assertEquals(expectedResult.areShapesColliding(),
                         epaResult.areShapesColliding(),
                         iterationPrefix + " Analytical: " + expectedResult.getSignedDistance() + ", EPA: " + epaResult.getSignedDistance() + ", diff: "
                         + distanceError);
         }

         if (epaResult.areShapesColliding())
         { // We're only interested on measuring the error for EPA as GJK is tested in its own test class.
            double positionErrorOnA = expectedResult.getPointOnA().distance(epaResult.getPointOnA());
            double positionErrorOnB = expectedResult.getPointOnB().distance(epaResult.getPointOnB());

            assertEquals(expectedResult.getSignedDistance(),
                         epaResult.getSignedDistance(),
                         distanceMaxEpsilon,
                         iterationPrefix + " difference: " + distanceError);

            EuclidCoreTestTools.assertPoint3DGeometricallyEquals(iterationPrefix, expectedResult.getPointOnA(), epaResult.getPointOnA(), positionMaxEpsilon);
            EuclidCoreTestTools.assertPoint3DGeometricallyEquals(iterationPrefix, expectedResult.getPointOnB(), epaResult.getPointOnB(), positionMaxEpsilon);

            numberOfCollidingSamples++;
            meanDistanceError += distanceError;
            meanPositionError += positionErrorOnA;
            meanPositionError += positionErrorOnB;

            maxDistanceError = Math.max(maxDistanceError, distanceError);
            maxPositionError = EuclidCoreTools.max(maxPositionError, positionErrorOnA, positionErrorOnB);
         }

         // GJK-EPA does not estimate the normal on each shape.
         assertTrue(epaResult.getNormalOnA().containsNaN(), iterationPrefix);
         assertTrue(epaResult.getNormalOnB().containsNaN(), iterationPrefix);
      }

      meanDistanceError /= ITERATIONS;
      meanPositionError /= 2.0 * ITERATIONS;

      if (verbose)
      {
         System.out.println("Number of iterations: " + ITERATIONS + ", number of colliding samples: " + numberOfCollidingSamples);
         System.out.println("Max error for the distance: " + maxDistanceError + ", position: " + maxPositionError);
         System.out.println("Average error for the distance: " + meanDistanceError + ", position: " + meanPositionError);
      }

      assertTrue(meanDistanceError < distanceMeanEpsilon, "mean distance error: " + meanDistanceError + " expected less than: " + distanceMeanEpsilon);
      assertTrue(meanPositionError < positionMeanEpsilon, "mean position error: " + meanPositionError + " expected less than: " + positionMeanEpsilon);
   }
}
