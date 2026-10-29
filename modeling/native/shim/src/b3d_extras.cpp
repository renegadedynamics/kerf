// Hand-written shim functions for what OcctGen can't bind: operators and internal representations.
#include "b3d_shim.h"

#include <BRepAlgoAPI_Algo.hxx>
#include <BRep_CurveRepresentation.hxx>
#include <BRep_TEdge.hxx>
#include <BRep_TFace.hxx>
#include <Geom2d_BSplineCurve.hxx>
#include <Geom2d_BezierCurve.hxx>
#include <Geom2d_OffsetCurve.hxx>
#include <Geom2d_TrimmedCurve.hxx>
#include <Geom_BSplineCurve.hxx>
#include <Geom_BSplineSurface.hxx>
#include <Geom_BezierCurve.hxx>
#include <Geom_BezierSurface.hxx>
#include <Geom_OffsetCurve.hxx>
#include <Geom_OffsetSurface.hxx>
#include <Geom_RectangularTrimmedSurface.hxx>
#include <Geom_SweptSurface.hxx>
#include <Geom_TrimmedCurve.hxx>
#include <Message_Alert.hxx>
#include <Message_Report.hxx>
#include <Poly_Polygon3D.hxx>
#include <Poly_PolygonOnTriangulation.hxx>
#include <Poly_Triangulation.hxx>
#include <TopoDS_Iterator.hxx>
#include <BRepTools.hxx>
#include <BRepTools_History.hxx>
#include <BRep_Builder.hxx>
#include <Geom_Curve.hxx>
#include <ShapeAnalysis_Curve.hxx>
#include <Storage_StreamFormatError.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_XYZ.hxx>

#include <cctype>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <locale>
#include <sstream>
#include <string>
#include <unordered_set>

//! std::hash of a shape: equal for shapes that are IsSame (same TShape and Location).
B3D_API uint64_t b3d_TopoDS_Shape_hash(const void* shape)
{
  return static_cast<uint64_t>(std::hash<TopoDS_Shape>{}(*static_cast<const TopoDS_Shape*>(shape)));
}

//! Whether the edge's own 3D curve lies in a plane normal to `normal`: 1 or 0, or -1 if it has none.
B3D_API int32_t b3d_edge_curve3d_is_planar(const void* edge, const void* normal, double tolerance, b3d_error** err)
{
  *err = nullptr;
  B3D_TRY
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
  B3D_TRY
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
  B3D_TRY
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

//! The shape read from OCCT's BRep text format. The data may be broken or hostile: OCCT's reader trusts it, so it's
//! checked for what the reader can't survive first, and a read that fails stops rather than carrying on.
B3D_API void* b3d_brep_read(const uint8_t* data, int64_t size, b3d_error** err)
{
  *err = nullptr;
  B3D_TRY
  {
    // The reader reads words into 255-character buffers with no limit (C++17's operator>>), so a longer one would
    // overrun the stack. No BRep word comes near the limit
    constexpr int64_t longestWord = 200;
    int64_t           word        = 0;
    for (int64_t i = 0; i < size; ++i)
    {
      const uint8_t c = data[i];
      word = std::isspace(c) ? 0 : word + 1;
      if (word >= longestWord)
        throw Storage_StreamFormatError("BRep data has a word too long to be BRep");
      if (c == 0)
        throw Storage_StreamFormatError("BRep data is text, without NULs");
    }
    std::istringstream in(std::string(reinterpret_cast<const char*>(data), static_cast<size_t>(size)));
    in.imbue(std::locale::classic());
    // A failed read throws: the reader doesn't check, and at the end of truncated data it would otherwise loop for
    // ever, reusing the last word it read
    in.exceptions(std::ios::failbit | std::ios::badbit);
    TopoDS_Shape shape;
    try
    {
      BRepTools::Read(shape, in, BRep_Builder());
    }
    catch (const std::ios_base::failure&)
    {
      throw Storage_StreamFormatError("BRep data is truncated or malformed");
    }
    if (shape.IsNull())
      throw Storage_StreamFormatError("BRep data holds no shape");
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

namespace
{
//! A rough count of the native memory a shape holds: what the GC is told about, so it collects before native
//! memory balloons. Each TShape and each piece of geometry is counted once however often it's shared.
class MemoryEstimate
{
public:
  int64_t Of(const TopoDS_Shape& shape)
  {
    if (!shape.IsNull())
      visit(shape);
    return bytes;
  }

private:
  std::unordered_set<const void*> seen;
  int64_t                         bytes = 0;

  bool first(const void* p) { return p != nullptr && seen.insert(p).second; }

  void visit(const TopoDS_Shape& shape)
  {
    const TopoDS_TShape* tshape = shape.TShape().get();
    if (!first(tshape))
      return;
    bytes += 96;
    if (const auto* tedge = dynamic_cast<const BRep_TEdge*>(tshape))
      for (const Handle(BRep_CurveRepresentation)& rep : tedge->Curves())
      {
        bytes += 64;
        if (rep->IsCurve3D())
          curve(rep->Curve3D());
        else if (rep->IsCurveOnSurface())
        {
          curve2d(rep->PCurve());
          if (rep->IsCurveOnClosedSurface())
            curve2d(rep->PCurve2());
        }
        else if (rep->IsPolygon3D() && first(rep->Polygon3D().get()))
          bytes += 32 * static_cast<int64_t>(rep->Polygon3D()->NbNodes());
        else if (rep->IsPolygonOnTriangulation() && first(rep->PolygonOnTriangulation().get()))
          bytes += 12 * static_cast<int64_t>(rep->PolygonOnTriangulation()->NbNodes());
      }
    else if (const auto* tface = dynamic_cast<const BRep_TFace*>(tshape))
    {
      surface(tface->Surface());
      for (const Handle(Poly_Triangulation)& tri : tface->Triangulations())
        if (first(tri.get()))
          bytes += static_cast<int64_t>(tri->NbNodes()) * (24 + (tri->HasUVNodes() ? 16 : 0) + (tri->HasNormals() ? 12 : 0))
                   + 12 * static_cast<int64_t>(tri->NbTriangles());
    }
    for (TopoDS_Iterator it(shape, false, false); it.More(); it.Next())
      visit(it.Value());
  }

  void curve(const Handle(Geom_Curve)& c)
  {
    if (!first(c.get()))
      return;
    bytes += 80;
    if (const auto* b = dynamic_cast<const Geom_BSplineCurve*>(c.get()))
      bytes += 32 * static_cast<int64_t>(b->NbPoles()) + 12 * static_cast<int64_t>(b->NbKnots());
    else if (const auto* z = dynamic_cast<const Geom_BezierCurve*>(c.get()))
      bytes += 32 * static_cast<int64_t>(z->NbPoles());
    else if (const auto* t = dynamic_cast<const Geom_TrimmedCurve*>(c.get()))
      curve(t->BasisCurve());
    else if (const auto* o = dynamic_cast<const Geom_OffsetCurve*>(c.get()))
      curve(o->BasisCurve());
  }

  void curve2d(const Handle(Geom2d_Curve)& c)
  {
    if (!first(c.get()))
      return;
    bytes += 64;
    if (const auto* b = dynamic_cast<const Geom2d_BSplineCurve*>(c.get()))
      bytes += 24 * static_cast<int64_t>(b->NbPoles()) + 12 * static_cast<int64_t>(b->NbKnots());
    else if (const auto* z = dynamic_cast<const Geom2d_BezierCurve*>(c.get()))
      bytes += 24 * static_cast<int64_t>(z->NbPoles());
    else if (const auto* t = dynamic_cast<const Geom2d_TrimmedCurve*>(c.get()))
      curve2d(t->BasisCurve());
    else if (const auto* o = dynamic_cast<const Geom2d_OffsetCurve*>(c.get()))
      curve2d(o->BasisCurve());
  }

  void surface(const Handle(Geom_Surface)& s)
  {
    if (!first(s.get()))
      return;
    bytes += 120;
    if (const auto* b = dynamic_cast<const Geom_BSplineSurface*>(s.get()))
      bytes += 32 * static_cast<int64_t>(b->NbUPoles()) * b->NbVPoles() + 12 * static_cast<int64_t>(b->NbUKnots() + b->NbVKnots());
    else if (const auto* z = dynamic_cast<const Geom_BezierSurface*>(s.get()))
      bytes += 32 * static_cast<int64_t>(z->NbUPoles()) * z->NbVPoles();
    else if (const auto* t = dynamic_cast<const Geom_RectangularTrimmedSurface*>(s.get()))
      surface(t->BasisSurface());
    else if (const auto* o = dynamic_cast<const Geom_OffsetSurface*>(s.get()))
      surface(o->BasisSurface());
    else if (const auto* w = dynamic_cast<const Geom_SweptSurface*>(s.get()))
      curve(w->BasisCurve());
  }
};
} // namespace

//! An estimate of the native memory a shape holds, in bytes (see MemoryEstimate).
B3D_API int64_t b3d_shape_memory(const void* shape)
{
  try
  {
    return MemoryEstimate().Of(*static_cast<const TopoDS_Shape*>(shape));
  }
  catch (...)
  {
    return 0; // an estimate: no estimate is no harm
  }
}

//! The names of the errors a boolean (or other BRepAlgoAPI algorithm) reported, e.g. "BOPAlgo_AlertTooFewArguments",
//! comma-separated, each once: a malloc'd string the caller frees with b3d_free_bytes, or null if there are none.
B3D_API char* b3d_algo_errors(const void* algo)
{
  try
  {
    std::string names;
    const Handle(Message_Report)& report = static_cast<const BRepAlgoAPI_Algo*>(algo)->GetReport();
    if (report.IsNull())
      return nullptr;
    for (const Handle(Message_Alert)& alert : report->GetAlerts(Message_Fail))
    {
      const std::string name = alert->DynamicType()->Name();
      if (names.find(name) == std::string::npos)
        names += (names.empty() ? "" : ", ") + name;
    }
    if (names.empty())
      return nullptr;
    auto* text = static_cast<char*>(std::malloc(names.size() + 1));
    if (text != nullptr)
      std::memcpy(text, names.c_str(), names.size() + 1);
    return text;
  }
  catch (...)
  {
    return nullptr; // diagnostics mustn't mask the failure they explain
  }
}
