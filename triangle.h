#ifndef TRIANGLE_H
#define TRIANGLE_H

#include "hittable.h"

class triangle : public hittable {
  public:
    triangle(const point3& p0, const point3& p1, const point3& p2, shared_ptr<material> m)
      : p0(p0), p1(p1), p2(p2), mat(m)
    {
        // Compute bounding box from three vertices
        point3 minp(
            std::fmin(std::fmin(p0.x(), p1.x()), p2.x()),
            std::fmin(std::fmin(p0.y(), p1.y()), p2.y()),
            std::fmin(std::fmin(p0.z(), p1.z()), p2.z())
        );

        point3 maxp(
            std::fmax(std::fmax(p0.x(), p1.x()), p2.x()),
            std::fmax(std::fmax(p0.y(), p1.y()), p2.y()),
            std::fmax(std::fmax(p0.z(), p1.z()), p2.z())
        );

        bbox = aabb(minp, maxp);
    }

    aabb bounding_box() const override { return bbox; }

    bool hit(const ray& r, interval ray_t, hit_record& rec) const override {
        // Möller–Trumbore ray-triangle intersection
        const double EPSILON = 1e-8;

        vec3 edge1 = p1 - p0;
        vec3 edge2 = p2 - p0;

        vec3 h = cross(r.direction(), edge2);
        double a = dot(edge1, h);

        if (std::fabs(a) < EPSILON)
            return false; // Ray parallel to triangle

        double f = 1.0 / a;
        vec3 s = r.origin() - p0;
        double u = f * dot(s, h);
        if (u < 0.0 || u > 1.0)
            return false;

        vec3 q = cross(s, edge1);
        double v = f * dot(r.direction(), q);
        if (v < 0.0 || (u + v) > 1.0)
            return false;

        double t = f * dot(edge2, q);
        if (!ray_t.contains(t))
            return false;

        // Fill hit record
        rec.t = t;
        rec.p = r.at(t);
        rec.mat = mat;
        vec3 outward_normal = unit_vector(cross(edge1, edge2));
        rec.set_face_normal(r, outward_normal);

        rec.u = u;
        rec.v = v;

        return true;
    }

  private:
    point3 p0, p1, p2;
    shared_ptr<material> mat;
    aabb bbox;
};

#endif
