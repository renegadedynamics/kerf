// Hand-written shim functions for what OcctGen can't bind: operators and internal representations.
#include "b3d_shim.h"

#include <BRep_CurveRepresentation.hxx>
#include <BRep_TEdge.hxx>
#include <BRepTools.hxx>
#include <BRepTools_History.hxx>
#include <BRep_Builder.hxx>
#include <Geom_Curve.hxx>
#include <ShapeAnalysis_Curve.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_XYZ.hxx>

#include <cstdlib>
#include <cstring>
#include <functional>
#include <locale>
#include <sstream>

//! std::hash of a shape: equal for shapes that are IsSame (same TShape and Location).
B3D_API uint64_t b3d_TopoDS_Shape_hash(const void* shape)
{
  return static_cast<uint64_t>(std::hash<TopoDS_Shape>{}(*static_cast<const TopoDS_Shape*>(shape)));
}

//! Whether the edge's own 3D curve lies in a plane normal to `normal`: 1 or 0, or -1 if it has none.
B3D_API int32_t b3d_edge_curve3d_is_planar(const void* edge, const void* normal, double tolerance, b3d_error** err)
{
  *err = nullptr;
  try
  {
    const auto& e = *static_cast<const TopoDS_Edge*>(edge);
    gp_XYZ n = *static_cast<const gp_XYZ*>(normal);
    const auto* tedge = dynamic_cast<const BRep_TEdge*>(e.TShape().get());
    if (tedge == nullptr)
      return -1;
    for (const auto& rep : tedge->Curves())
      if (rep->IsCurve3D())
        return ShapeAnalysis_Curve::IsPlanar(rep->Curve3D(), n, tolerance) ? 1 : 0;
    return -1;
  }
  B3D_CATCH(err)
  return -1;
}

//! A copy of a history: its maps copied as they are (merging into an empty history drops the removal of a shape
//! that also generated or was modified).
B3D_API void* b3d_BRepTools_History_copy(const void* history, b3d_error** err)
{
  *err = nullptr;
  try
  {
    return b3d_retain(new BRepTools_History(*static_cast<const BRepTools_History*>(history)));
  }
  B3D_CATCH(err)
  return nullptr;
}

//! A shape in OCCT's BRep text format, with its triangulations if `with_triangles`: a malloc'd buffer the caller
//! frees with b3d_free_bytes.
B3D_API uint8_t* b3d_brep_write(const void* shape, uint8_t with_triangles, int64_t* size, b3d_error** err)
{
  *err = nullptr;
  *size = 0;
  try
  {
    std::ostringstream out;
    out.imbue(std::locale::classic());
    BRepTools::Write(*static_cast<const TopoDS_Shape*>(shape), out, with_triangles != 0, false,
                     TopTools_FormatVersion_CURRENT);
    const std::string text = out.str();
    auto* data = static_cast<uint8_t*>(std::malloc(text.size() == 0 ? 1 : text.size()));
    if (data == nullptr)
      throw std::bad_alloc();
    std::memcpy(data, text.data(), text.size());
    *size = static_cast<int64_t>(text.size());
    return data;
  }
  B3D_CATCH(err)
  return nullptr;
}

//! The shape read from OCCT's BRep text format.
B3D_API void* b3d_brep_read(const uint8_t* data, int64_t size, b3d_error** err)
{
  *err = nullptr;
  try
  {
    std::istringstream in(std::string(reinterpret_cast<const char*>(data), static_cast<size_t>(size)));
    in.imbue(std::locale::classic());
    TopoDS_Shape shape;
    BRepTools::Read(shape, in, BRep_Builder());
    return new TopoDS_Shape(shape);
  }
  B3D_CATCH(err)
  return nullptr;
}

//! Frees a buffer the shim returned.
B3D_API void b3d_free_bytes(uint8_t* data)
{
  std::free(data);
}
