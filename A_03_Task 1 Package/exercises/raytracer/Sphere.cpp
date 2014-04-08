#include "Sphere.hpp"
#include "Ray.hpp"
#include <memory>

namespace rt
{

double sgn(double x) {
  if (x < 0) return -1;
  else return 1;
}

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
  double a = dot(ray.direction(), ray.direction());
  double b = 2 * dot(ray.direction(), ray.origin());
  double c = dot(ray.origin(), ray.origin()) - 1;

  double t1, t2;
  int roots = calcRoots(a, b, c, t1, t2);

  bool doIntersect = false;
  if (roots > 0) {
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
