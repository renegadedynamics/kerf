// Hand-written shim functions for what OcctGen can't bind: operators and internal representations.
#include "b3d_shim.h"

#include <BRep_CurveRepresentation.hxx>
#include <BRep_TEdge.hxx>
#include <BRepTools_History.hxx>
#include <Geom_Curve.hxx>
#include <ShapeAnalysis_Curve.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_XYZ.hxx>

#include <functional>

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
