// Bulk extraction of a meshed shape's triangulations and edge polylines: what Build123d's ShapeMesh would
// otherwise gather with several P/Invoke calls per node and per triangle.
#include "b3d_shim.h"

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepLib_ToolTriangulatedShape.hxx>
#include <BRep_Tool.hxx>
#include <GCPnts_TangentialDeflection.hxx>
#include <Poly_Polygon3D.hxx>
#include <Poly_PolygonOnTriangulation.hxx>
#include <Poly_Triangulation.hxx>
#include <TopExp.hxx>
#include <NCollection_IndexedDataMap.hxx>
#include <NCollection_List.hxx>
#include <TopoDS_Shape.hxx>
#include <TopTools_ShapeMapHasher.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>

#include <vector>

namespace
{
using EdgeFaces = NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>, TopTools_ShapeMapHasher>;

//! The extracted mesh: every face's nodes and triangles, and every edge's polyline, end to end, with offsets.
struct Mesh
{
  std::vector<float>   positions; // x, y, z per node, all faces
  std::vector<float>   normals;   // per node
  std::vector<int32_t> triangles; // index triples into each face's own nodes
  std::vector<int64_t> faceNodes; // node offset of each face, then the total
  std::vector<int64_t> faceTris;  // triangle offset (in indices) of each face, then the total
  std::vector<int32_t> faceTypes; // GeomAbs_SurfaceType
  std::vector<float>   points;    // x, y, z per polyline point, all edges
  std::vector<int64_t> edgePoints; // point offset of each edge, then the total
  std::vector<int32_t> edgeTypes; // GeomAbs_CurveType
};

void addPoint(std::vector<float>& to, const gp_Pnt& p)
{
  to.push_back(static_cast<float>(p.X()));
  to.push_back(static_cast<float>(p.Y()));
  to.push_back(static_cast<float>(p.Z()));
}

void addFace(Mesh& mesh, const TopoDS_Face& face)
{
  mesh.faceNodes.push_back(static_cast<int64_t>(mesh.positions.size() / 3));
  mesh.faceTris.push_back(static_cast<int64_t>(mesh.triangles.size()));
  mesh.faceTypes.push_back(static_cast<int32_t>(BRepAdaptor_Surface(face, false).GetType()));

  TopLoc_Location                  location;
  const Handle(Poly_Triangulation) tri = BRep_Tool::Triangulation(face, location);
  if (tri.IsNull())
    return;
  if (!tri->HasNormals())
    BRepLib_ToolTriangulatedShape::ComputeNormals(face, tri); // the surface's normals at the nodes

  const gp_Trsf trsf     = location.Transformation();
  const bool    reversed = face.Orientation() == TopAbs_REVERSED;
  // Winding follows the surface's natural normal: flipped for a reversed face, and again by a mirroring location
  const bool flipWinding = reversed != trsf.IsNegative();
  for (int i = 1; i <= tri->NbNodes(); ++i)
  {
    addPoint(mesh.positions, tri->Node(i).Transformed(trsf));
    gp_Dir n = tri->Normal(i);
    n.Transform(trsf);
    if (reversed)
      n.Reverse();
    mesh.normals.push_back(static_cast<float>(n.X()));
    mesh.normals.push_back(static_cast<float>(n.Y()));
    mesh.normals.push_back(static_cast<float>(n.Z()));
  }
  for (int i = 1; i <= tri->NbTriangles(); ++i)
  {
    int a, b, c;
    tri->Triangle(i).Get(a, b, c);
    mesh.triangles.push_back(a - 1);
    mesh.triangles.push_back(flipWinding ? c - 1 : b - 1);
    mesh.triangles.push_back(flipWinding ? b - 1 : c - 1);
  }
}

void addEdge(Mesh&                                            mesh,
             const TopoDS_Edge&                               edge,
             const EdgeFaces&                                 edgeFaces,
             double                                           deflection,
             double                                           angle)
{
  mesh.edgePoints.push_back(static_cast<int64_t>(mesh.points.size() / 3));
  BRepAdaptor_Curve curve(edge);
  mesh.edgeTypes.push_back(static_cast<int32_t>(curve.GetType()));
  const size_t start = mesh.points.size();

  // On a face's triangulation, so the edge sits exactly on the shaded surface
  bool done = false;
  if (const auto* faces = edgeFaces.Seek(edge))
    for (const TopoDS_Shape& shape : *faces)
    {
      TopLoc_Location                  location;
      const TopoDS_Face&               face = TopoDS::Face(shape);
      const Handle(Poly_Triangulation) tri  = BRep_Tool::Triangulation(face, location);
      if (tri.IsNull())
        continue;
      const Handle(Poly_PolygonOnTriangulation) polygon = BRep_Tool::PolygonOnTriangulation(edge, tri, location);
      if (polygon.IsNull())
        continue;
      const gp_Trsf trsf = location.Transformation();
      for (int i = 1; i <= polygon->NbNodes(); ++i)
        addPoint(mesh.points, tri->Node(polygon->Node(i)).Transformed(trsf));
      done = true;
      break;
    }
  // A free edge's own polygon, from the mesher
  if (!done)
  {
    TopLoc_Location              location;
    const Handle(Poly_Polygon3D) polygon = BRep_Tool::Polygon3D(edge, location);
    if (!polygon.IsNull())
    {
      const gp_Trsf trsf = location.Transformation();
      for (int i = 1; i <= polygon->NbNodes(); ++i)
        addPoint(mesh.points, polygon->Nodes().Value(i).Transformed(trsf));
      done = true;
    }
  }
  // Else discretised to the same tolerances
  if (!done)
  {
    GCPnts_TangentialDeflection discretizer(curve, angle, deflection);
    for (int i = 1; i <= discretizer.NbPoints(); ++i)
      addPoint(mesh.points, discretizer.Value(i));
  }
  // Polygons follow the curve's parameter: reversed to run the way the edge does
  if (edge.Orientation() == TopAbs_REVERSED)
  {
    const size_t count = (mesh.points.size() - start) / 3;
    for (size_t i = 0; i < count / 2; ++i)
      for (size_t k = 0; k < 3; ++k)
        std::swap(mesh.points[start + 3 * i + k], mesh.points[start + 3 * (count - 1 - i) + k]);
  }
}
} // namespace

//! The triangulations of `faces` and the polylines of `edges`, sub-shapes of the meshed `root`. Faces without a
//! triangulation give no nodes; edges are taken from a face's triangulation where they lie on one.
B3D_API void* b3d_mesh_extract(const void*        root,
                               const void* const* faces,
                               int32_t            faceCount,
                               const void* const* edges,
                               int32_t            edgeCount,
                               double             deflection,
                               double             angle,
                               b3d_error**        err)
{
  *err = nullptr;
  try
  {
    auto* mesh = new Mesh();
    try
    {
      EdgeFaces edgeFaces;
      TopExp::MapShapesAndAncestors(*static_cast<const TopoDS_Shape*>(root), TopAbs_EDGE, TopAbs_FACE, edgeFaces);
      for (int32_t i = 0; i < faceCount; ++i)
        addFace(*mesh, TopoDS::Face(*static_cast<const TopoDS_Shape*>(faces[i])));
      mesh->faceNodes.push_back(static_cast<int64_t>(mesh->positions.size() / 3));
      mesh->faceTris.push_back(static_cast<int64_t>(mesh->triangles.size()));
      for (int32_t i = 0; i < edgeCount; ++i)
        addEdge(*mesh, TopoDS::Edge(*static_cast<const TopoDS_Shape*>(edges[i])), edgeFaces, deflection, angle);
      mesh->edgePoints.push_back(static_cast<int64_t>(mesh->points.size() / 3));
      return mesh;
    }
    catch (...)
    {
      delete mesh;
      throw;
    }
  }
  B3D_CATCH(err)
  return nullptr;
}

//! The sizes of an extracted mesh: nodes, triangle indices, edge points (in floats: 3 per point).
B3D_API void b3d_mesh_sizes(const void* mesh, int64_t* nodeFloats, int64_t* triangleIndices, int64_t* pointFloats)
{
  const auto* m    = static_cast<const Mesh*>(mesh);
  *nodeFloats      = static_cast<int64_t>(m->positions.size());
  *triangleIndices = static_cast<int64_t>(m->triangles.size());
  *pointFloats     = static_cast<int64_t>(m->points.size());
}

//! Copies an extracted mesh into buffers of the sizes b3d_mesh_sizes gives (and of the face and edge counts, plus
//! one, for the offsets).
B3D_API void b3d_mesh_copy(const void* mesh,
                           float*      positions,
                           float*      normals,
                           int32_t*    triangles,
                           int64_t*    faceNodes,
                           int64_t*    faceTris,
                           int32_t*    faceTypes,
                           float*      points,
                           int64_t*    edgePoints,
                           int32_t*    edgeTypes)
{
  const auto* m    = static_cast<const Mesh*>(mesh);
  auto        copy = [](const auto& from, auto* to) { std::copy(from.begin(), from.end(), to); };
  copy(m->positions, positions);
  copy(m->normals, normals);
  copy(m->triangles, triangles);
  copy(m->faceNodes, faceNodes);
  copy(m->faceTris, faceTris);
  copy(m->faceTypes, faceTypes);
  copy(m->points, points);
  copy(m->edgePoints, edgePoints);
  copy(m->edgeTypes, edgeTypes);
}

//! Frees an extracted mesh.
B3D_API void b3d_mesh_free(void* mesh)
{
  delete static_cast<Mesh*>(mesh);
}
