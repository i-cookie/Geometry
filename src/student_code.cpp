#include "student_code.h"
#include "mutablePriorityQueue.h"

using namespace std;

namespace CGL
{

  /**
   * Evaluates one step of the de Casteljau's algorithm using the given points and
   * the scalar parameter t (class member).
   *
   * @param points A vector of points in 2D
   * @return A vector containing intermediate points or the final interpolated vector
   */
  std::vector<Vector2D> BezierCurve::evaluateStep(std::vector<Vector2D> const &points)
  { 
    // TODO Task 1.
    if (points.size() == 1)
      return points;

    std::vector<Vector2D> res;
    for (int i = 0; i < points.size()-1; i++) {
      double nx = points[i].x * (1-t) + points[i+1].x * t;
      double ny = points[i].y * (1-t) + points[i+1].y * t;
      res.push_back(Vector2D(nx, ny));
    }
    return res;
  }

  /**
   * Evaluates one step of the de Casteljau's algorithm using the given points and
   * the scalar parameter t (function parameter).
   *
   * @param points    A vector of points in 3D
   * @param t         Scalar interpolation parameter
   * @return A vector containing intermediate points or the final interpolated vector
   */
  std::vector<Vector3D> BezierPatch::evaluateStep(std::vector<Vector3D> const &points, double t) const
  {
    // TODO Task 2.
    if (points.size() == 1)
      return points;

    std::vector<Vector3D> res;
    for (int i = 0; i < points.size()-1; i++) {
      double nx = points[i].x * (1-t) + points[i+1].x * t;
      double ny = points[i].y * (1-t) + points[i+1].y * t;
      double nz = points[i].z * (1-t) + points[i+1].z * t;
      res.push_back(Vector3D(nx, ny, nz));
    }
    return res;
  }

  /**
   * Fully evaluates de Casteljau's algorithm for a vector of points at scalar parameter t
   *
   * @param points    A vector of points in 3D
   * @param t         Scalar interpolation parameter
   * @return Final interpolated vector
   */
  Vector3D BezierPatch::evaluate1D(std::vector<Vector3D> const &points, double t) const
  {
    // TODO Task 2.
    std::vector<Vector3D> res = points;
    while (res.size() > 1)
      res = evaluateStep(res, t);

    return res[0];
  }

  /**
   * Evaluates the Bezier patch at parameter (u, v)
   *
   * @param u         Scalar interpolation parameter
   * @param v         Scalar interpolation parameter (along the other axis)
   * @return Final interpolated vector
   */
  Vector3D BezierPatch::evaluate(double u, double v) const 
  {  
    // TODO Task 2.
    Vector3D res = Vector3D();
    std::vector<Vector3D> columnPoints;
    for (int i = 0; i < controlPoints.size(); i++) {
      Vector3D colPoint;
      colPoint = evaluate1D(controlPoints[i], u);
      columnPoints.push_back(colPoint);
    }
    res = evaluate1D(columnPoints, v);
    return res;
  }

  Vector3D Vertex::normal( void ) const
  {
    // TODO Task 3.
    // Returns an approximate unit normal at this vertex, computed by
    // taking the area-weighted average of the normals of neighboring
    // triangles, then normalizing.
    Vector3D res = Vector3D(0., 0., 0.);

    HalfedgeCIter h = this->halfedge();
    do {
      if (!h->face()->isBoundary()) {
        double area = 0.;
        Vector3D v1 = Vector3D(h->next()->vertex()->position         - this->position);
        Vector3D v2 = Vector3D(h->next()->next()->vertex()->position - this->position);
        area = 0.5 * cross(v1, v2).norm();
        res += area * h->face()->normal();
      }
      h = h->twin()->next();
    } while (h != this->halfedge());

    return res.unit();
  }

  EdgeIter HalfedgeMesh::flipEdge( EdgeIter e0 )
  {
    // TODO Task 4.
    // This method should flip the given edge and return an iterator to the flipped edge.
    if (e0->isBoundary())
      return e0;
    HalfedgeIter h0 = e0->halfedge(),
                 h1 = h0->twin(),
                 h01 = h0->next(),
                 h02 = h01->next(),
                 h11 = h1->next(),
                 h12 = h11->next();
    VertexIter v0 = h0->vertex(),
               v1 = h1->vertex(),
               v02 = h02->vertex(),
               v12 = h12->vertex();
    FaceIter f0 = h0->face(),
             f1 = h1->face();

    // Face settings
    f0->halfedge() = h0;
    f1->halfedge() = h1;

    // Vertex settings
    v0->halfedge() = h11;
    v1->halfedge() = h01;

    // Halfedge settings
    //   setNeighbors(next, twin,       vertex, edge,        face)
    // For h0 triangle
    h0-> setNeighbors(h12, h1,          v02,    e0,          f0);
    h01->setNeighbors(h0,  h01->twin(), v1,     h01->edge(), f0);
    h02->setNeighbors(h11, h02->twin(), v02,    h02->edge(), f1);
    // For h1 triangle
    h1-> setNeighbors(h02, h0,          v12,    e0,          f1);
    h11->setNeighbors(h1,  h11->twin(), v0,     h11->edge(), f1);
    h12->setNeighbors(h01, h12->twin(), v12,    h12->edge(), f0);

    return e0;
  }

  VertexIter HalfedgeMesh::splitEdge( EdgeIter e0 )
  {
    // TODO Task 5.
    // This method should split the given edge and return an iterator to the newly inserted vertex.
    // The halfedge of this vertex should point along the edge that was split, rather than the new edges.
    if (e0->isBoundary()) {
      //return VertexIter();
      HalfedgeIter hA1 = e0->halfedge()->isBoundary() ? e0->halfedge()->twin() : e0->halfedge(),
                   hA2 = hA1->twin(),
                   h0 = hA1->next(),
                   h1 = h0->next(),
                   hB1 = newHalfedge(),
                   hB2 = newHalfedge(),
                   hAB1 = newHalfedge(),
                   hAB2 = newHalfedge();
      VertexIter m = newVertex(),
                 vA = hA2->vertex(),
                 vB = hA1->vertex();
      FaceIter fA = h0->face(),
               fB = newFace();
      EdgeIter eB = newEdge(),
               eAB = newEdge();
      
      // Vertex settings
      m->halfedge() = hA1;
      m->isNew = true;
      m->position = 0.5 * Vector3D(vA->position + vB->position);
      vB->halfedge() = hB2;

      // Face settings
      fA->halfedge() = h0,
      fB->halfedge() = h1;

      // Edge settings
      eB->halfedge() = hB1;
      eB->isNew = false;
      eAB->halfedge() = hAB1;
      eAB->isNew = true;

      // Halfedge settings
      //    setNeighbors(next,        twin,        vertex,       edge,       face)
      h0->  setNeighbors(hAB2,        h0->twin(),  vA,           h0->edge(), fA);
      h1->  setNeighbors(hB2,         h1->twin(),  h1->vertex(), h1->edge(), fB);
      hA1-> setNeighbors(h0,          hA2,         m,            e0,         fA);
      hA2-> setNeighbors(hA2->next(), hA1,         vA,           e0,         hA2->face());
      hB1-> setNeighbors(hB1->next(), hB2,         m,            eB,         hA2->face());
      hB2-> setNeighbors(hAB1,        hB1,         vB,           eB,         fB);
      hAB1->setNeighbors(h1,          hAB2,        m,            eAB,        fB);
      hAB2->setNeighbors(hA1,         hAB1,        h1->vertex(), eAB,        fA);

      return m;
    }

    // Original elements
    HalfedgeIter h0 = e0->halfedge(),
                 h1 = h0->twin(),
                 h01 = h0->next(),
                 h02 = h01->next(),
                 h11 = h1->next(),
                 h12 = h11->next();
    VertexIter v0 = h0->vertex(),
               v1 = h1->vertex(),
               vA = h02->vertex(),
               vB = h12->vertex();
    FaceIter f0 = h0->face(),
             f1 = h1->face();

    // New elements to add
    HalfedgeIter hA1 = newHalfedge(),
                 hA2 = newHalfedge(),
                 hB1 = newHalfedge(),
                 hB2 = newHalfedge(),
                 hAB1 = newHalfedge(),
                 hAB2 = newHalfedge();
    VertexIter m = newVertex();
    FaceIter fA = newFace(),
             fB = newFace();
    EdgeIter eA = newEdge(),
             eAB = newEdge(),
             eB = newEdge();
    
    // Vertex settings
    m->halfedge() = h0;
    m->isNew = true;
    m->position = 0.5 * Vector3D(v0->position + v1->position);
    v0->halfedge() = h11;

    // Face settings
    f0->halfedge() = h01;
    f1->halfedge() = h12;
    fA->halfedge() = h02;
    fB->halfedge() = h11;

    // Edge settings
    eA->halfedge() = hA1;
    eA->isNew = true;
    eB->halfedge() = hB1;
    eB->isNew = true;
    eAB->halfedge() = hAB1;
    eAB->isNew = false;

    // Halfedge settings
    //    setNeighbors(next, twin,        vertex, edge,        face)
    h0->  setNeighbors(h01,  h1,          m,      e0,          f0);
    h1->  setNeighbors(hB1,  h0,          v1,     e0,          f1);
    h01-> setNeighbors(hA2,  h01->twin(), v1,     h01->edge(), f0);
    h02-> setNeighbors(hAB2, h02->twin(), vA,     h02->edge(), fA);
    h11-> setNeighbors(hB2,  h11->twin(), v0,     h11->edge(), fB);
    h12-> setNeighbors(h1,   h12->twin(), vB,     h12->edge(), f1);
    hA1-> setNeighbors(h02,  hA2,         m,      eA,          fA);
    hA2-> setNeighbors(h0,   hA1,         vA,     eA,          f0);
    hB1-> setNeighbors(h12,  hB2,         m,      eB,          f1);
    hB2-> setNeighbors(hAB1, hB1,         vB,     eB,          fB);
    hAB1->setNeighbors(h11,  hAB2,        m,      eAB,         fB);
    hAB2->setNeighbors(hA1,  hAB1,        v0,     eAB,         fA);

    return m;
  }



  void MeshResampler::upsample( HalfedgeMesh& mesh )
  {
    // TODO Task 6.
    // This routine should increase the number of triangles in the mesh using Loop subdivision.
    // One possible solution is to break up the method as listed below.

    // 1. Compute new positions for all the vertices in the input mesh, using the Loop subdivision rule,
    // and store them in Vertex::newPosition. At this point, we also want to mark each vertex as being
    // a vertex of the original mesh.
    for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++) {
      int n = v->degree();
      double u = (n == 3 ? 3.0/16.0 : 3.0/(8.0*n));
      Vector3D neighbor_position_sum;

      HalfedgeCIter h = v->halfedge();
      do {
        neighbor_position_sum += h->twin()->vertex()->position;
        h = h->twin()->next();
      } while (h != v->halfedge());

      v->newPosition = (1.0 - n*u) * v->position + u * neighbor_position_sum;
      v->isNew = false;
    }
    //cout << "debug1" << endl;

    // 2. Compute the updated vertex positions associated with edges, and store it in Edge::newPosition.
    for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++) {
      Vector3D A = e->halfedge()->vertex()                        ->position;
      Vector3D B = e->halfedge()->twin()->vertex()                ->position;
      Vector3D C = e->halfedge()->twin()->next()->next()->vertex()->position;
      Vector3D D = e->halfedge()->next()->next()->vertex()        ->position;
      e->newPosition = 3.0/8.0 * (A+B) + 1.0/8.0 * (C+D);
      e->isNew = false;
    }
    //cout << "debug2" << endl;

    // 3. Split every edge in the mesh, in any order. For future reference, we're also going to store some
    // information about which subdivide edges come from splitting an edge in the original mesh, and which edges
    // are new, by setting the flat Edge::isNew. Note that in this loop, we only want to iterate over edges of
    // the original mesh---otherwise, we'll end up splitting edges that we just split (and the loop will never end!)
    for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++) {
      if (e->isNew) continue;
      if (e->halfedge()->vertex()->isNew |
          e->halfedge()->twin()->vertex()->isNew) 
        continue;
      VertexIter v = mesh.splitEdge(e);
      v->newPosition = e->newPosition;
      //v->newPosition = 0.5 * (e->halfedge()->vertex()->position + e->halfedge()->twin()->vertex()->position);
    }
    //cout << "debug3" << endl;

    // 4. Flip any new edge that connects an old and new vertex.
    for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++) {
      if (!e->isNew) continue;
      if (e->halfedge()->vertex()->isNew ^
          e->halfedge()->twin()->vertex()->isNew)
        mesh.flipEdge(e);
    }
    //cout << "debug4" << endl;

    // 5. Copy the new vertex positions into final Vertex::position.
    for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++) {
      v->position = v->newPosition;
      v->isNew = false;
    }
    //cout << "debug5" << endl;

    for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++) {
      e->isNew = false;
    }
  }
}
