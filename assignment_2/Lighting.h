#ifndef _LIGHTING_H_
#define _LIGHTING_H_

#include <Eigen/Geometry>
#include <algorithm>
#include "Color.h"
#include "Point.h"
#include "Canvas.h"

namespace Lighting {

Color lighting(const VectorXd& point_position,
               const VectorXd& n,
               const Material& material,
               const std::vector<Light>& lights,
               const VectorXd& cam_position) {
  using namespace Eigen;
  using std::max;

  auto diffuse = material.diffuse_reflectance;
  auto ambient = material.ambient_reflectance;
  auto specular = material.specular_reflectance;
  auto shininess = material.shininess;

  VectorXd diffuse_sum(3);
  diffuse_sum.setZero(3, 1);
  VectorXd specular_sum(3);
  specular_sum.setZero(3, 1);

  auto cam_direction = (cam_position - point_position).normalized();

  // Get the contribution from each light
  for (const auto& light : lights) {
    auto light_color = light.color.matrix();
    // Attentuate the light_color
    auto distance = (point_position - cam_position).norm();
    light_color *= (1.0 / (1 + (light.attenuation * distance * distance)));

    auto light_position = VectorXd(3);
    light_position << light.x, light.y, light.z;
    auto light_direction = (light_position - point_position).normalized();

    // Get diffuse light
    double dot_product = n.dot(light_direction);
    auto light_diffuse = light_color * max(0.0, dot_product);
    diffuse_sum += light_diffuse;

    // Get specular light
    dot_product = n.dot((cam_direction + light_direction).normalized());
    auto light_specular = light_color *
                          pow(max(0.0, dot_product), shininess);
    specular_sum += light_specular;
  }

  // Add ambient, diffuse, specular light; clip as necessary.
  auto color = Color{
                 VectorXd::Ones(3).cwiseMin(
                   ambient
                   + diffuse_sum.cwiseProduct(diffuse)
                   + specular_sum.cwiseProduct(specular))};
  return color;
}

/** Get barycentric coords given coords and triangle vertices. */
Vector3d getBaryCoords(Point a, Point b, Point c, int x, int y) {
  auto f = [](Point i, Point j, int xi, int yi) {
    auto x = static_cast<double>(xi);
    auto y = static_cast<double>(yi);
    return (i.y - j.y) * x + (j.x - i.x) * y + i.x * j.y - j.x * i.y;
  };

  Vector3d bary_coords(3);
  bary_coords << f(b, c, x, y) / f(b, c, a.x, a.y),
                 f(a, c, x, y) / f(a, c, b.x, b.y),
                 f(a, b, x, y) / f(a, b, c.x, c.y);
  return bary_coords;
}

void
rasterColoredTriangle(const Face& f, const std::vector<Vertex>& vertices,
                      const std::vector<NormalVector>& normals,
                      const Material& material,
                      const Camera& camera, const std::vector<Light>& lights,
                      Canvas& canvas,
                      Eigen::MatrixXd& buffer,
                      int shading_mode) {
  using std::min;
  using std::max;
  
  // Get NDC vertices
  auto a_ndc = camera.world2NDC(vertices[f.v1_idx - 1]);
  auto b_ndc = camera.world2NDC(vertices[f.v2_idx - 1]);
  auto c_ndc = camera.world2NDC(vertices[f.v3_idx - 1]);

  // Backface culling
  auto cross = (c_ndc.matrix() - b_ndc.matrix()).cross(a_ndc.matrix() - b_ndc.matrix());
  if (cross(2) < 0) {
    return;
  }

  // Get points in screen coordinates
  auto pa = canvas.getPointFromNDCVertex(a_ndc);
  auto pb = canvas.getPointFromNDCVertex(b_ndc);
  auto pc = canvas.getPointFromNDCVertex(c_ndc);

  // Get pixel range to iterate over
  auto x_min = min({pa.x, pb.x, pc.x});
  auto x_max = max({pa.x, pb.x, pc.x});
  auto y_min = min({pa.y, pb.y, pc.y});
  auto y_max = max({pa.y, pb.y, pc.y});

  for (int x = x_min; x <= x_max; ++x) {
    for (int y = y_min; y <= y_max; ++y) {
      auto bary_coords = getBaryCoords(pa, pb, pc, x, y);
      auto alpha = bary_coords(0);
      auto beta = bary_coords(1);
      auto gamma = bary_coords(2);
      if (alpha >= 0 && alpha <= 1 &&
          beta  >= 0 && beta  <= 1 &&
          gamma >= 0 && gamma <= 1) {
        // Point is inside the triangle, must color it.
        // Get NDC coords of the point we want to paint
        Vertex v_ndc(alpha * a_ndc.matrix() +
                     beta  * b_ndc.matrix() +
                     gamma * c_ndc.matrix());
        // Check if onscreen and do depth buffering
        if (v_ndc.inNDCCube() && !(v_ndc.z > buffer(x, y))) {
          buffer(x, y) = v_ndc.z;
          if (shading_mode == 0) {  // Do Gourard Shading

            // Get colors of triangle vertices
            Color v1_color = lighting(vertices[f.v1_idx - 1].matrix(),
                                      normals[f.v1_normal_idx - 1].matrix(),
                                      material,
                                      lights,
                                      camera.getPosition()
                                      );
            Color v2_color = lighting(vertices[f.v2_idx - 1].matrix(),
                                      normals[f.v2_normal_idx - 1].matrix(),
                                      material,
                                      lights,
                                      camera.getPosition()
                                      );

            Color v3_color = lighting(vertices[f.v3_idx - 1].matrix(),
                                      normals[f.v3_normal_idx - 1].matrix(),
                                      material,
                                      lights,
                                      camera.getPosition()
                                      );
            // Resulting color is linear combination of vertex colors weighted
            // by barycentric coordinate values
            auto c = Color{alpha * v1_color.matrix() +
                           beta * v2_color.matrix() +
                           gamma * v3_color.matrix()};

            canvas.fill(x, y, c);
          } else {  // Do Phong shading
            // norm is combo of vertex norms, weighted by bary coords
            auto norm = Vector3d{(alpha * normals[f.v1_normal_idx - 1].matrix()) +
                                 (beta  * normals[f.v2_normal_idx - 1].matrix()) +
                                 (gamma * normals[f.v3_normal_idx - 1].matrix())};
            // world position is combo of vertex positions, weighted by bary coords
            auto v_world = Vector3d{(alpha * vertices[f.v1_idx - 1].matrix()) +
                                    (beta * vertices[f.v2_idx - 1].matrix()) +
                                    (gamma * vertices[f.v3_idx - 1].matrix())};
            // get lighting for the new position and normal vector.
            auto color = lighting(v_world,
                                  norm,
                                  material,
                                  lights,
                                  camera.getPosition());

            canvas.fill(x, y, color);
          }
        }
      }
    }
  }
}

} // namespace Lighting

#endif  // _LIGHTING_H_
