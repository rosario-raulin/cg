#include "Triangle.hpp"
#include "Ray.hpp"
#include <memory>

namespace rt
{
Triangle::Triangle(const Vec3d &v0, const Vec3d &v1, const Vec3d &v2,
                   const Vec3d &uvw0, const Vec3d &uvw1, const Vec3d &uvw2)
{
  mVertices[0] = v0;
  mVertices[1] = v1;
  mVertices[2] = v2;
  mUVW[0] = uvw0;
  mUVW[1] = uvw1;
  mUVW[2] = uvw2;
}

bool
Triangle::closestIntersectionModel(const Ray &ray, double maxLambda, RayIntersection& intersection) const
{
  /*

  What we are basically doing is solving the system of linear equations given by

  (1) o1 + lambda*d1 = (1-v-w)*a1 + v*b1 + w*c1
  (2) o2 + lambda*d2 = (1-v-w)*a2 + v*b2 + w*c2
  (3) o1 + lambda*d3 = (1-v-w)*a3 + v*b3 + w*c3,

  where
    - o = (o1, o2, o3)T is the origin of the ray
    - a = (a1, a2, a3)T, b = (b1, b2, b3)T, c = (c1, c2, c3)T
    are the vertices of the triangle
    - v, w and u = 1 - v - w are the Barycentric coordinates

  for which we check that 0 <= u, v, w <= 1. If so, the ray intersects the triangle.

  We use Cramer's rule as described in [1] to do this:

    lambda = D1 / D
    v = D2 / D
    w = D3 / D

  with
    D = det(A)
    Di = det(Ai) where Ai is A with the i'th row exchanged by lambda

    (A is the matrix representing the system of linear equations).

  We also apply some trickery as described in [1] but this does not change
  the basic idea.

  [1] http://wwwcg.in.tum.de/fileadmin/user_upload/Lehrstuehle/Lehrstuhl_XV/Teaching/SS07/Proseminar/Florian_Ferstl.pdf

  */

  // some helper variables
  Vec3d e1 = mVertices[1] - mVertices[0];
  Vec3d e2 = mVertices[2] - mVertices[0];

  // first we need to determine 
  Vec3d p = cross(ray.direction(), e2);
  double D = dot(p, e1);
  if (fabs(D) < Math::safetyEps()) {
    // no intersection between the plane of the triange and the ray
    return false;
  }

  // Use Dt do "gain some performance" (well, ...)
  double Dt = 1.0 / D;
  Vec3d t = ray.origin() - mVertices[0];
  double alpha = dot(p, t) * Dt;

  if (alpha < 0.0 || alpha > 1.0) return false;

  Vec3d q = cross(t, e1);
  double beta = dot(q, ray.direction()) * Dt;

  // We combine the check for beta and gamma = 1 - alpha - beta,
  // which in turn mean that alpha + beta <= 1 (which implies beta <= 1).
  if (beta < 0.0 || (beta+alpha) > 1.0) return false;

  double lambda = dot(q, e2) * Dt;

  if (lambda < 0.0 || lambda > maxLambda) return false;

  intersection = RayIntersection(ray, shared_from_this(), lambda, ray.pointOnRay(lambda), Vec3d(0,0,0));
  return true;
}

BoundingBox Triangle::computeBoundingBox() const
{
  BoundingBox bbox;
  bbox.expandByPoint(mVertices[0]);
  bbox.expandByPoint(mVertices[1]);
  bbox.expandByPoint(mVertices[2]);
  return bbox;
}

} //namespace rt
