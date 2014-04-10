#include "Sphere.hpp"
#include "Ray.hpp"
#include <memory>

namespace rt
{

double sgn(double x) {
  if (x < 0) return -1;
  else return 1;
}

// This is just a "clever" way of solving quadratic equations
int calcRoots(double a, double b, double c, double& t1, double& t2) {
  double d = b * b - 4 * a * c;
  if (d < 0) {
    t1 = 0.0;
    t2 = 0.0;
    return 0;
  } else {
    d = sqrt(d);
    double q = -0.5 * (b + sgn(b) * d);
    t1 = q / a;
    t2 = c / q;
    if (t1 > t2) {
      q = t2; t2 = t1; t1 = q;
    }
    return t1 == t2 ? 1 : 2;
  }
}

bool
Sphere::closestIntersectionModel(const Ray &ray, double maxLambda, RayIntersection& intersection) const
{
  /*
    A ray and a sphere intersect if and only if
      (o + lambda * d - c)^2 = r^2

    where
      o is the origin of the ray
      d is the direction vector of the ray
      c is the center of the sphere
      r is the radius of the sphere

    Since o and d are known, c = O and r = 1, we have
    a quadratic equation in lambda. We solve it using calcRoots()
    which is a slightly modified version of the classical
    quadratic formula (as described in [1]).

    [1] http://wiki.delphigl.com/index.php/Tutorial_Raytracing_-_Grundlagen_I#Quadratische_Gleichungen
  */

  // a, b and c are the constants in
  // a * lambda^2 + b * lambda + c = 0.
  double a = dot(ray.direction(), ray.direction());
  double b = 2 * dot(ray.direction(), ray.origin());
  double c = dot(ray.origin(), ray.origin()) - 1;

  // t1 and t2 are the solutions of our quadratic equation.
  // W.l.o.g. t1 <= t2 (calcRoots takes care of that)
  // calcRoots return the number of roots it found (zero, one or two).
  double t1, t2;
  int roots = calcRoots(a, b, c, t1, t2);

  bool doIntersect = false;
  if (roots > 0) {
    // There is a solution, we just need the (smallest) positve one.
    double lambda = t1 >= 0 ? t1 : t2;
    if (lambda < maxLambda) {
      doIntersect = true;
      intersection = RayIntersection(ray, shared_from_this(), lambda, ray.pointOnRay(lambda), Vec3d(0,0,0));
    }
  }

  return doIntersect;
}

BoundingBox Sphere::computeBoundingBox() const
{
  BoundingBox box;
  box.setMin(Vec3d(-1,-1,-1));
  box.setMax(Vec3d(1,1,1));
  return box;
}

} //namespace rt
