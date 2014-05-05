#include "BezierPatchMesh.hpp"
#include <functional>

namespace rt
{

  BezierPatchMesh::BezierPatchMesh(size_t m,    size_t n,
    size_t resu, size_t resv) : BVHIndexedTriangleMesh(),
  mM(m), mN(n), mResU(resu), mResV(resv)
  {
    // Allocate memory for Bezier points
    mControlPoints.resize(m*n);
  }

  void BezierPatchMesh::initialize()
  {
    //this function samples the underlying continuous patch and tessellates it
    //regularly with triangles

    //sample at triangle vertices at uniform uv parameters
    std::vector<BezierPatchSample> samples; samples.reserve(mResU*mResV);

    for(size_t j=0; j<mResV; ++j)
      for(size_t i=0; i<mResU; ++i)
        samples.push_back(this->sample(double(i) / (mResU-1), double(j) / (mResV-1)));

    //construct triangle strips based on the computed samples
    for(size_t j=0; j<mResV-1; ++j)
    {
      for(size_t i=0; i<mResU-1; ++i)
      {
        const size_t i00 = mResU* j    +  i,
          i10 = mResU* j    + (i+1),
          i01 = mResU*(j+1) +  i,
          i11 = mResU*(j+1) + (i+1);

        {
          //construct lower triangle
          const Vec3d &v0 = samples[i00].position,
            &v1 = samples[i10].position,
            &v2 = samples[i01].position;
          const Vec2d &p0 = samples[i00].uv,
            &p1 = samples[i10].uv,
            &p2 = samples[i01].uv;
          Vec3d  n0 = samples[i00].normal,
            n1 = samples[i10].normal,
            n2 = samples[i01].normal;

          if(!n0.lengthSquared() || !n1.lengthSquared() || !n2.lengthSquared())
          {
            //fall back to triangle normals if normals are not defined
            const Vec3d normal = cross((v1-v0),(v2-v0)).normalize();
            n0 = normal; n1 = normal; n2 = normal;
          }

          int i0=this->addVertex(v0,n0,Vec3d(p0,0));
          int i1=this->addVertex(v1,n1,Vec3d(p1,0));
          int i2=this->addVertex(v2,n2,Vec3d(p2,0));
          this->addTriangle(i0,i1,i2);
        }

        {
          //construct upper triangle
          const Vec3d &v0 = samples[i10].position,
            &v1 = samples[i11].position,
            &v2 = samples[i01].position;
          const Vec2d &p0 = samples[i10].uv,
            &p1 = samples[i11].uv,
            &p2 = samples[i01].uv;
          Vec3d n0 = samples[i10].normal,
            n1 = samples[i11].normal,
            n2 = samples[i01].normal;

          if(!n0.lengthSquared() || !n1.lengthSquared() || !n2.lengthSquared())
          {
            //fall back to triangle normals if normals are not defined
            const Vec3d normal = cross((v1-v0),(v2-v0)).normalize();
            n0 = normal; n1 = normal; n2 = normal;
          }
          int i0=this->addVertex(v0,n0,Vec3d(p0,0));
          int i1=this->addVertex(v1,n1,Vec3d(p1,0));
          int i2=this->addVertex(v2,n2,Vec3d(p2,0));
          this->addTriangle(i0,i1,i2);
        }
      }
    }
    BVHIndexedTriangleMesh::initialize();
  }

  BoundingBox BezierPatchMesh::computeBoundingBox() const
  {
    BoundingBox bbox;
    for (size_t i=0;i<mControlPoints.size();++i)
      bbox.expandByPoint(mControlPoints[i]);
    return bbox;
  }

  // This is the special version of De-Casteljau's algorithm.
  // Note that we choose a non-recursive version that stops before the
  // last step so we can calculate both, the tangent and the point.
  // Output: b(1, n-1)(x) and b(0, n-1)(x) stored in points[0] and points[1]
  void BezierPatchMesh::_deCasteljau(std::vector<Vec3d>& points, double x) const {
    while (points.size() != 2) {
      for (size_t i = 0; i < points.size()-1; ++i) {
        points[i] = (1-x)*points[i] + x*points[i+1];
      }
      points.resize(points.size()-1);
    }
  }

  // This is the generic version of De-Casteljau's algorithm that calculates
  // both, the tangent and the point.
  void BezierPatchMesh::deCasteljau(const std::vector<Vec3d>& points, double x, Vec3d& point, Vec3d& tangent) const {
    std::vector<Vec3d> tmp(points);
    _deCasteljau(tmp, x);

    point = (1-x)*tmp[0] + x*tmp[1];
    tangent = (tmp[1] - tmp[0]) * (double)points.size();
  }

  BezierPatchMesh::BezierPatchSample BezierPatchMesh::sample(double u, double v) const
  {
    BezierPatchSample ret;
    ret.uv = Vec2d(u,v);

    std::vector<Vec3d> uPoints(mM), vPoints(mN);
    Vec3d vTangent, uTangent, garbage, point;

    // We do a first pass of De-Casteljau...
    for (size_t j = 0; j < mN; ++j) {
      for (size_t i = 0; i < mM; ++i) {
        uPoints[i] = controlPoint(i,j);
      }

      deCasteljau(uPoints, u, vPoints[j], garbage);
    }
    // ... just to calculate v's tangent.
    deCasteljau(vPoints, v, garbage, vTangent);

    // Next up: calculating u's tangent and the intersection point
    for (size_t i = 0; i < mM; ++i) {
      for (size_t j = 0; j < mN; ++j) {
        vPoints[j] = controlPoint(i,j);
      }

      deCasteljau(vPoints, v, uPoints[i], garbage);
    }
    deCasteljau(uPoints, u, ret.position, uTangent);

    // The normalized cross product of u's tangent and v's tangent
    // is the normalized normal vector we are looking for
    ret.normal = cross(uTangent, vTangent).normalize();
    return ret;
  }
} //namespace rt
