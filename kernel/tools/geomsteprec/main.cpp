// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
//
// geomsteprec: records what native OCCT's StepToGeom and GeomToStep do, for unit 8D (tools/geomsteprec/build.ps1):
//
//   geomsteprec gen <cases>                writes the constructed cases (curves, 2D curves and surfaces of every
//                                          kind in GeomTools' compact text, and gp values) that "g2s" reads.
//   geomsteprec s2g <list> <out> [full]    list: one "name<TAB>path" per line (the corpus's STEP files). Each file is
//                                          read with STEPControl_Reader (the AP214 protocol) and every geometric
//                                          entity of the model converted by StepToGeom (points, directions, vectors,
//                                          placements, transformation operators, curves in 3D and 2D, surfaces),
//                                          with the default factors ("mm") and with inches and degrees ("in"). Only
//                                          the entities of StepBasic, StepGeom, StepShape and StepRepr, the packages
//                                          the port reads so far; the others converted are counted ("SKIPPED n").
//   geomsteprec g2s <cases> <list> <out> [full]
//                                          the cases, then the geometry of the .brep files listed (surfaces of the
//                                          faces; 3D curves, their trimmed ranges and pcurves of the edges, each
//                                          once), converted by every GeomToStep maker that applies, with both factor
//                                          sets; each result written as STEP entities (a model of its own, the AP214
//                                          protocol's StepData_StepWriter) and converted back by StepToGeom.
//
// Output, per file (or set of cases): "TAG <factors> <lines> <sha256 of the lines>" and sample lines "L <factors>
// <line>" (all of them with "full" or for the cases, otherwise the first 10 and up to 20 exceptions, cut at 200
// characters). The port's
// replay (tests/OcctNet.DataExchange.Tests/GeomToStep/GeomStepNativeTests.cs) computes the same lines.
// Reals are written as C++'s ostream does with precision 17; geometry in GeomTools' compact text on one line.
#include <windows.h>
#include <bcrypt.h>

#include <cmath>
#include <cstdio>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include <BRep_CurveRepresentation.hxx>
#include <BRep_GCurve.hxx>
#include <BRep_TEdge.hxx>
#include <BRep_Tool.hxx>
#include <BRep_Builder.hxx>
#include <BRepTools.hxx>
#include <Geom_Axis1Placement.hxx>
#include <Geom_Axis2Placement.hxx>
#include <Geom_BSplineCurve.hxx>
#include <Geom_BSplineSurface.hxx>
#include <Geom_BezierCurve.hxx>
#include <Geom_BezierSurface.hxx>
#include <Geom_CartesianPoint.hxx>
#include <Geom_Circle.hxx>
#include <Geom_ConicalSurface.hxx>
#include <Geom_CylindricalSurface.hxx>
#include <Geom_Direction.hxx>
#include <Geom_Ellipse.hxx>
#include <Geom_Hyperbola.hxx>
#include <Geom_Line.hxx>
#include <Geom_OffsetCurve.hxx>
#include <Geom_OffsetSurface.hxx>
#include <Geom_Parabola.hxx>
#include <Geom_Plane.hxx>
#include <Geom_RectangularTrimmedSurface.hxx>
#include <Geom_SphericalSurface.hxx>
#include <Geom_SurfaceOfLinearExtrusion.hxx>
#include <Geom_SurfaceOfRevolution.hxx>
#include <Geom_ToroidalSurface.hxx>
#include <Geom_TrimmedCurve.hxx>
#include <Geom_VectorWithMagnitude.hxx>
#include <Geom2d_AxisPlacement.hxx>
#include <Geom2d_BSplineCurve.hxx>
#include <Geom2d_BezierCurve.hxx>
#include <Geom2d_CartesianPoint.hxx>
#include <Geom2d_Circle.hxx>
#include <Geom2d_Direction.hxx>
#include <Geom2d_Ellipse.hxx>
#include <Geom2d_Hyperbola.hxx>
#include <Geom2d_Line.hxx>
#include <Geom2d_OffsetCurve.hxx>
#include <Geom2d_Parabola.hxx>
#include <Geom2d_TrimmedCurve.hxx>
#include <Geom2d_VectorWithMagnitude.hxx>
#include <GeomTools_Curve2dSet.hxx>
#include <GeomTools_CurveSet.hxx>
#include <GeomTools_SurfaceSet.hxx>
#include <GeomToStep_MakeAxis1Placement.hxx>
#include <GeomToStep_MakeAxis2Placement2d.hxx>
#include <GeomToStep_MakeAxis2Placement3d.hxx>
#include <GeomToStep_MakeBSplineCurveWithKnots.hxx>
#include <GeomToStep_MakeBSplineCurveWithKnotsAndRationalBSplineCurve.hxx>
#include <GeomToStep_MakeBSplineSurfaceWithKnots.hxx>
#include <GeomToStep_MakeBSplineSurfaceWithKnotsAndRationalBSplineSurface.hxx>
#include <GeomToStep_MakeBoundedCurve.hxx>
#include <GeomToStep_MakeBoundedSurface.hxx>
#include <GeomToStep_MakeCartesianPoint.hxx>
#include <GeomToStep_MakeCartesianTransformationOperator.hxx>
#include <GeomToStep_MakeCircle.hxx>
#include <GeomToStep_MakeConic.hxx>
#include <GeomToStep_MakeConicalSurface.hxx>
#include <GeomToStep_MakeCurve.hxx>
#include <GeomToStep_MakeCylindricalSurface.hxx>
#include <GeomToStep_MakeDirection.hxx>
#include <GeomToStep_MakeElementarySurface.hxx>
#include <GeomToStep_MakeEllipse.hxx>
#include <GeomToStep_MakeHyperbola.hxx>
#include <GeomToStep_MakeLine.hxx>
#include <GeomToStep_MakeParabola.hxx>
#include <GeomToStep_MakePlane.hxx>
#include <GeomToStep_MakePolyline.hxx>
#include <GeomToStep_MakeRectangularTrimmedSurface.hxx>
#include <GeomToStep_MakeSphericalSurface.hxx>
#include <GeomToStep_MakeSurface.hxx>
#include <GeomToStep_MakeSurfaceOfLinearExtrusion.hxx>
#include <GeomToStep_MakeSurfaceOfRevolution.hxx>
#include <GeomToStep_MakeSweptSurface.hxx>
#include <GeomToStep_MakeToroidalSurface.hxx>
#include <GeomToStep_MakeVector.hxx>
#include <Interface_Static.hxx>
#include <Message.hxx>
#include <Message_Messenger.hxx>
#include <Message_PrinterOStream.hxx>
#include <STEPControl_Reader.hxx>
#include <Standard_Failure.hxx>
#include <StepAP214.hxx>
#include <StepAP214_Protocol.hxx>
#include <StepData_Factors.hxx>
#include <StepData_StepModel.hxx>
#include <StepData_StepWriter.hxx>
#include <StepData_WriterLib.hxx>
#include <StepGeom_Line.hxx>
#include <StepGeom_Conic.hxx>
#include <StepGeom_Circle.hxx>
#include <StepGeom_Ellipse.hxx>
#include <StepGeom_Hyperbola.hxx>
#include <StepGeom_Parabola.hxx>
#include <StepGeom_BoundedCurve.hxx>
#include <StepGeom_BSplineCurveWithKnots.hxx>
#include <StepGeom_BSplineCurveWithKnotsAndRationalBSplineCurve.hxx>
#include <StepGeom_BoundedSurface.hxx>
#include <StepGeom_ElementarySurface.hxx>
#include <StepGeom_SweptSurface.hxx>
#include <StepGeom_Plane.hxx>
#include <StepGeom_CylindricalSurface.hxx>
#include <StepGeom_ConicalSurface.hxx>
#include <StepGeom_SphericalSurface.hxx>
#include <StepGeom_ToroidalSurface.hxx>
#include <StepGeom_SurfaceOfLinearExtrusion.hxx>
#include <StepGeom_SurfaceOfRevolution.hxx>
#include <StepGeom_RectangularTrimmedSurface.hxx>
#include <StepGeom_BSplineSurfaceWithKnots.hxx>
#include <StepGeom_BSplineSurfaceWithKnotsAndRationalBSplineSurface.hxx>
#include <StepGeom_Polyline.hxx>
#include <StepGeom_OffsetSurface.hxx>
#include <StepGeom_Axis1Placement.hxx>
#include <StepGeom_Axis2Placement2d.hxx>
#include <StepGeom_Axis2Placement3d.hxx>
#include <StepGeom_CartesianPoint.hxx>
#include <StepGeom_CartesianTransformationOperator2d.hxx>
#include <StepGeom_CartesianTransformationOperator3d.hxx>
#include <StepGeom_Curve.hxx>
#include <StepGeom_Direction.hxx>
#include <StepGeom_SuParameters.hxx>
#include <StepGeom_Surface.hxx>
#include <StepGeom_Vector.hxx>
#include <StepToGeom.hxx>
#include <TCollection_HAsciiString.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Ax22d.hxx>
#include <gp_Circ.hxx>
#include <gp_Elips.hxx>
#include <gp_Lin.hxx>
#include <gp_Lin2d.hxx>
#include <gp_Pln.hxx>
#include <gp_Ax3.hxx>
#include <gp_Trsf.hxx>
#include <gp_Trsf2d.hxx>

// ---------------------------------------------------------------------------------------------------------------
// Text

static std::string R(double theValue)
{
  std::ostringstream aS;
  aS.precision(17);
  aS << theValue;
  return aS.str();
}

static std::string OneLine(const std::string& theText)
{
  std::string aRes;
  for (char c : theText)
    aRes += (c == '\n' || c == '\r') ? ' ' : c;
  while (!aRes.empty() && aRes.back() == ' ')
    aRes.pop_back();
  return aRes;
}

static std::string Sha256(const std::string& theData)
{
  BCRYPT_ALG_HANDLE  anAlg = nullptr;
  BCRYPT_HASH_HANDLE aHash = nullptr;
  unsigned char      aOut[32];
  BCryptOpenAlgorithmProvider(&anAlg, BCRYPT_SHA256_ALGORITHM, nullptr, 0);
  BCryptCreateHash(anAlg, &aHash, nullptr, 0, nullptr, 0, 0);
  BCryptHashData(aHash, (PUCHAR)theData.data(), (ULONG)theData.size(), 0);
  BCryptFinishHash(aHash, aOut, 32, 0);
  BCryptDestroyHash(aHash);
  BCryptCloseAlgorithmProvider(anAlg, 0);
  std::string aRes;
  char        aBuf[4];
  for (int i = 0; i < 32; i++)
  {
    sprintf(aBuf, "%02x", aOut[i]);
    aRes += aBuf;
  }
  return aRes;
}

static std::string Pnt(const gp_Pnt& P) { return R(P.X()) + " " + R(P.Y()) + " " + R(P.Z()); }
static std::string Pnt(const gp_Pnt2d& P) { return R(P.X()) + " " + R(P.Y()); }
static std::string Dir(const gp_Dir& D) { return R(D.X()) + " " + R(D.Y()) + " " + R(D.Z()); }
static std::string Dir(const gp_Dir2d& D) { return R(D.X()) + " " + R(D.Y()); }

static std::string Curve(const occ::handle<Geom_Curve>& C)
{
  std::ostringstream aS;
  aS.precision(17);
  GeomTools_CurveSet::PrintCurve(C, aS, true);
  return OneLine(aS.str());
}

static std::string Curve2d(const occ::handle<Geom2d_Curve>& C)
{
  std::ostringstream aS;
  aS.precision(17);
  GeomTools_Curve2dSet::PrintCurve2d(C, aS, true);
  return OneLine(aS.str());
}

static std::string Surface(const occ::handle<Geom_Surface>& S)
{
  std::ostringstream aS;
  aS.precision(17);
  GeomTools_SurfaceSet::PrintSurface(S, aS, true);
  return OneLine(aS.str());
}

static std::string Trsf(const gp_Trsf& T)
{
  std::string aRes = "trsf";
  for (int r = 1; r <= 3; r++)
    for (int c = 1; c <= 4; c++)
      aRes += " " + R(T.Value(r, c));
  return aRes + " " + R(T.ScaleFactor()) + " " + std::to_string((int)T.Form());
}

static std::string Trsf2d(const gp_Trsf2d& T)
{
  std::string aRes = "trsf2d";
  for (int r = 1; r <= 2; r++)
    for (int c = 1; c <= 3; c++)
      aRes += " " + R(T.Value(r, c));
  return aRes + " " + R(T.ScaleFactor()) + " " + std::to_string((int)T.Form());
}

// ---------------------------------------------------------------------------------------------------------------
// StepToGeom on one entity: "<prefix><method> <result>" lines

#define TRY(name, expr)                                                                                        \
  {                                                                                                            \
    std::string aRes;                                                                                          \
    try                                                                                                        \
    {                                                                                                          \
      aRes = expr;                                                                                             \
    }                                                                                                          \
    catch (...)                                                                                                \
    {                                                                                                          \
      aRes = "EXC";                                                                                            \
    }                                                                                                          \
    theLines.push_back(thePrefix + name + " " + aRes);                                                         \
  }

template <class T> static std::string Null(const occ::handle<T>& h, const std::string& theText)
{
  return h.IsNull() ? std::string("null") : theText;
}

static void StepToGeomLines(const occ::handle<Standard_Transient>& theEnt,
                            const StepData_Factors&               F,
                            const std::string&                    thePrefix,
                            std::vector<std::string>&             theLines)
{
  if (occ::handle<StepGeom_CartesianPoint> P = occ::down_cast<StepGeom_CartesianPoint>(theEnt))
  {
    TRY("pnt", ([&]() { auto r = StepToGeom::MakeCartesianPoint(P, F); return Null(r, r.IsNull() ? "" : Pnt(r->Pnt())); })());
    TRY("pnt2d", ([&]() { auto r = StepToGeom::MakeCartesianPoint2d(P, F); return Null(r, r.IsNull() ? "" : Pnt(r->Pnt2d())); })());
  }
  else if (occ::handle<StepGeom_Direction> D = occ::down_cast<StepGeom_Direction>(theEnt))
  {
    TRY("dir", ([&]() { auto r = StepToGeom::MakeDirection(D); return Null(r, r.IsNull() ? "" : Dir(r->Dir())); })());
    TRY("dir2d", ([&]() { auto r = StepToGeom::MakeDirection2d(D); return Null(r, r.IsNull() ? "" : Dir(r->Dir2d())); })());
  }
  else if (occ::handle<StepGeom_Vector> V = occ::down_cast<StepGeom_Vector>(theEnt))
  {
    TRY("vec", ([&]() { auto r = StepToGeom::MakeVectorWithMagnitude(V, F); return Null(r, r.IsNull() ? "" : Pnt(gp_Pnt(r->Vec().XYZ()))); })());
    TRY("vec2d", ([&]() { auto r = StepToGeom::MakeVectorWithMagnitude2d(V); return Null(r, r.IsNull() ? "" : Pnt(gp_Pnt2d(r->Vec2d().XY()))); })());
  }
  else if (occ::handle<StepGeom_Axis1Placement> A1 = occ::down_cast<StepGeom_Axis1Placement>(theEnt))
  {
    TRY("ax1", ([&]() { auto r = StepToGeom::MakeAxis1Placement(A1, F); return Null(r, r.IsNull() ? "" : Pnt(r->Ax1().Location()) + " " + Dir(r->Ax1().Direction())); })());
  }
  else if (occ::handle<StepGeom_Axis2Placement3d> A2 = occ::down_cast<StepGeom_Axis2Placement3d>(theEnt))
  {
    TRY("ax2", ([&]() { auto r = StepToGeom::MakeAxis2Placement(A2, F); return Null(r, r.IsNull() ? "" : Pnt(r->Ax2().Location()) + " " + Dir(r->Ax2().Direction()) + " " + Dir(r->Ax2().XDirection())); })());
  }
  else if (occ::handle<StepGeom_Axis2Placement2d> A22 = occ::down_cast<StepGeom_Axis2Placement2d>(theEnt))
  {
    TRY("ax2d", ([&]() { auto r = StepToGeom::MakeAxisPlacement(A22, F); return Null(r, r.IsNull() ? "" : Pnt(r->Ax2d().Location()) + " " + Dir(r->Ax2d().Direction())); })());
  }
  else if (occ::handle<StepGeom_CartesianTransformationOperator3d> T3 = occ::down_cast<StepGeom_CartesianTransformationOperator3d>(theEnt))
  {
    TRY("trsf", ([&]() { gp_Trsf T; bool ok = StepToGeom::MakeTransformation3d(T3, T, F); return ok ? Trsf(T) : std::string("false"); })());
  }
  else if (occ::handle<StepGeom_CartesianTransformationOperator2d> T2 = occ::down_cast<StepGeom_CartesianTransformationOperator2d>(theEnt))
  {
    TRY("trsf2d", ([&]() { gp_Trsf2d T; bool ok = StepToGeom::MakeTransformation2d(T2, T, F); return ok ? Trsf2d(T) : std::string("false"); })());
  }
  else if (occ::handle<StepGeom_SuParameters> SU = occ::down_cast<StepGeom_SuParameters>(theEnt))
  {
    TRY("su", ([&]() { auto r = StepToGeom::MakeAxis2Placement(SU); return Null(r, r.IsNull() ? "" : Pnt(r->Ax2().Location()) + " " + Dir(r->Ax2().Direction()) + " " + Dir(r->Ax2().XDirection())); })());
  }
  if (occ::handle<StepGeom_Curve> C = occ::down_cast<StepGeom_Curve>(theEnt))
  {
    TRY("curve", ([&]() { auto r = StepToGeom::MakeCurve(C, F); return Null(r, r.IsNull() ? "" : Curve(r)); })());
    TRY("curve2d", ([&]() { auto r = StepToGeom::MakeCurve2d(C, F); return Null(r, r.IsNull() ? "" : Curve2d(r)); })());
  }
  if (occ::handle<StepGeom_Surface> S = occ::down_cast<StepGeom_Surface>(theEnt))
  {
    TRY("surface", ([&]() { auto r = StepToGeom::MakeSurface(S, F); return Null(r, r.IsNull() ? "" : Surface(r)); })());
  }
}

static StepData_Factors Inches()
{
  StepData_Factors F;
  F.InitializeFactors(25.4, M_PI / 180., 1.);
  return F;
}

// ---------------------------------------------------------------------------------------------------------------
// Output

static void Block(FILE*                           theOut,
                  const char*                     theTag,
                  const std::vector<std::string>& theLines,
                  bool                            theFull)
{
  std::string aAll;
  for (const std::string& aLine : theLines)
    aAll += aLine + "\n";
  fprintf(theOut, "TAG %s %zu %s\n", theTag, theLines.size(), Sha256(aAll).c_str());
  int aNbExc = 0;
  for (size_t i = 0; i < theLines.size(); i++)
  {
    bool isExc = theLines[i].find(" EXC") != std::string::npos;
    if (theFull)
      fprintf(theOut, "L %s %s\n", theTag, theLines[i].c_str());
    else if (i < 10 || (isExc && aNbExc++ < 20))
      fprintf(theOut, "L %s %s\n", theTag, theLines[i].substr(0, 200).c_str()); // samples cut at 200 characters
  }
}

static std::vector<std::pair<std::string, std::string>> ReadList(const char* theList)
{
  std::vector<std::pair<std::string, std::string>> aRes;
  std::ifstream                                    aList(theList);
  std::string                                      aLine;
  while (std::getline(aList, aLine))
  {
    if (!aLine.empty() && aLine.back() == '\r')
      aLine.pop_back();
    size_t aTab = aLine.find('\t');
    if (aTab != std::string::npos)
      aRes.emplace_back(aLine.substr(0, aTab), aLine.substr(aTab + 1));
  }
  return aRes;
}

// ---------------------------------------------------------------------------------------------------------------
// GeomToStep: a result written as STEP entities, then converted back

static occ::handle<StepAP214_Protocol> theProtocol;

static std::string Written(const occ::handle<Standard_Transient>& theEnt)
{
  occ::handle<StepData_StepModel> aModel = new StepData_StepModel;
  aModel->SetProtocol(theProtocol);
  aModel->AddWithRefs(theEnt, theProtocol);
  StepData_StepWriter aWriter(aModel);
  StepData_WriterLib  aLib(theProtocol);
  for (int i = 1; i <= aModel->NbEntities(); i++)
    aWriter.SendEntity(i, aLib);
  std::string aRes;
  for (int i = 1; i <= aWriter.NbLines(); i++)
    aRes += std::string(i > 1 ? " | " : "") + aWriter.Line(i)->ToCString();
  return aRes;
}

// One maker: "<g> <maker> <entities>" or NOTDONE or EXC, then the entity converted back
template <class TMaker, class TArg>
static void Make(const std::string&        theId,
                 const char*               theName,
                 const TArg&               theArg,
                 const StepData_Factors&   F,
                 std::vector<std::string>& theLines)
{
  occ::handle<Standard_Transient> anEnt;
  std::string                     aRes;
  try
  {
    TMaker aMaker(theArg, F);
    if (!aMaker.IsDone())
      aRes = "NOTDONE";
    else
    {
      anEnt = aMaker.Value();
      aRes  = anEnt.IsNull() ? std::string("null") : Written(anEnt);
    }
  }
  catch (...)
  {
    aRes = "EXC";
  }
  theLines.push_back(theId + " " + theName + " " + aRes);
  if (!anEnt.IsNull())
    StepToGeomLines(anEnt, F, theId + " " + theName + " back ", theLines);
}

// The same for makers without factors
template <class TMaker, class TArg>
static void Make0(const std::string&        theId,
                  const char*               theName,
                  const TArg&               theArg,
                  const StepData_Factors&   F,
                  std::vector<std::string>& theLines)
{
  occ::handle<Standard_Transient> anEnt;
  std::string                     aRes;
  try
  {
    TMaker aMaker(theArg);
    if (!aMaker.IsDone())
      aRes = "NOTDONE";
    else
    {
      anEnt = aMaker.Value();
      aRes  = anEnt.IsNull() ? std::string("null") : Written(anEnt);
    }
  }
  catch (...)
  {
    aRes = "EXC";
  }
  theLines.push_back(theId + " " + theName + " " + aRes);
  if (!anEnt.IsNull())
    StepToGeomLines(anEnt, F, theId + " " + theName + " back ", theLines);
}

static void CurveMakers(const std::string& theId, const occ::handle<Geom_Curve>& C, const StepData_Factors& F, std::vector<std::string>& theLines)
{
  theLines.push_back(theId + " " + C->DynamicType()->Name());
  Make<GeomToStep_MakeCurve>(theId, "curve", C, F, theLines);
  if (auto L = occ::down_cast<Geom_Line>(C))
  {
    Make<GeomToStep_MakeLine>(theId, "line", L, F, theLines);
    Make<GeomToStep_MakeLine>(theId, "lin", L->Lin(), F, theLines);
  }
  if (auto Co = occ::down_cast<Geom_Conic>(C))
    Make<GeomToStep_MakeConic>(theId, "conic", Co, F, theLines);
  if (auto Ci = occ::down_cast<Geom_Circle>(C))
  {
    Make<GeomToStep_MakeCircle>(theId, "circle", Ci, F, theLines);
    Make<GeomToStep_MakeCircle>(theId, "circ", Ci->Circ(), F, theLines);
  }
  if (auto E = occ::down_cast<Geom_Ellipse>(C))
  {
    Make<GeomToStep_MakeEllipse>(theId, "ellipse", E, F, theLines);
    Make<GeomToStep_MakeEllipse>(theId, "elips", E->Elips(), F, theLines);
  }
  if (auto H = occ::down_cast<Geom_Hyperbola>(C))
    Make<GeomToStep_MakeHyperbola>(theId, "hyperbola", H, F, theLines);
  if (auto P = occ::down_cast<Geom_Parabola>(C))
    Make<GeomToStep_MakeParabola>(theId, "parabola", P, F, theLines);
  if (auto B = occ::down_cast<Geom_BoundedCurve>(C))
    Make<GeomToStep_MakeBoundedCurve>(theId, "bounded", B, F, theLines);
  if (auto BS = occ::down_cast<Geom_BSplineCurve>(C))
  {
    Make<GeomToStep_MakeBSplineCurveWithKnots>(theId, "bspline", BS, F, theLines);
    Make<GeomToStep_MakeBSplineCurveWithKnotsAndRationalBSplineCurve>(theId, "rbspline", BS, F, theLines);
  }
}

static void Curve2dMakers(const std::string& theId, const occ::handle<Geom2d_Curve>& C, const StepData_Factors& F, std::vector<std::string>& theLines)
{
  theLines.push_back(theId + " " + C->DynamicType()->Name());
  Make<GeomToStep_MakeCurve>(theId, "curve", C, F, theLines);
  if (auto L = occ::down_cast<Geom2d_Line>(C))
  {
    Make<GeomToStep_MakeLine>(theId, "line", L, F, theLines);
    Make<GeomToStep_MakeLine>(theId, "lin", L->Lin2d(), F, theLines);
  }
  if (auto Co = occ::down_cast<Geom2d_Conic>(C))
    Make<GeomToStep_MakeConic>(theId, "conic", Co, F, theLines);
  if (auto Ci = occ::down_cast<Geom2d_Circle>(C))
    Make<GeomToStep_MakeCircle>(theId, "circle", Ci, F, theLines);
  if (auto E = occ::down_cast<Geom2d_Ellipse>(C))
    Make<GeomToStep_MakeEllipse>(theId, "ellipse", E, F, theLines);
  if (auto H = occ::down_cast<Geom2d_Hyperbola>(C))
    Make<GeomToStep_MakeHyperbola>(theId, "hyperbola", H, F, theLines);
  if (auto P = occ::down_cast<Geom2d_Parabola>(C))
    Make<GeomToStep_MakeParabola>(theId, "parabola", P, F, theLines);
  if (auto B = occ::down_cast<Geom2d_BoundedCurve>(C))
    Make<GeomToStep_MakeBoundedCurve>(theId, "bounded", B, F, theLines);
  if (auto BS = occ::down_cast<Geom2d_BSplineCurve>(C))
  {
    Make<GeomToStep_MakeBSplineCurveWithKnots>(theId, "bspline", BS, F, theLines);
    Make<GeomToStep_MakeBSplineCurveWithKnotsAndRationalBSplineCurve>(theId, "rbspline", BS, F, theLines);
  }
}

static void SurfaceMakers(const std::string& theId, const occ::handle<Geom_Surface>& S, const StepData_Factors& F, std::vector<std::string>& theLines)
{
  theLines.push_back(theId + " " + S->DynamicType()->Name());
  Make<GeomToStep_MakeSurface>(theId, "surface", S, F, theLines);
  if (auto B = occ::down_cast<Geom_BoundedSurface>(S))
    Make<GeomToStep_MakeBoundedSurface>(theId, "bounded", B, F, theLines);
  if (auto E = occ::down_cast<Geom_ElementarySurface>(S))
    Make<GeomToStep_MakeElementarySurface>(theId, "elementary", E, F, theLines);
  if (auto W = occ::down_cast<Geom_SweptSurface>(S))
    Make<GeomToStep_MakeSweptSurface>(theId, "swept", W, F, theLines);
  if (auto P = occ::down_cast<Geom_Plane>(S))
  {
    Make<GeomToStep_MakePlane>(theId, "plane", P, F, theLines);
    Make<GeomToStep_MakePlane>(theId, "pln", P->Pln(), F, theLines);
  }
  if (auto Cy = occ::down_cast<Geom_CylindricalSurface>(S))
    Make<GeomToStep_MakeCylindricalSurface>(theId, "cylinder", Cy, F, theLines);
  if (auto Co = occ::down_cast<Geom_ConicalSurface>(S))
    Make<GeomToStep_MakeConicalSurface>(theId, "cone", Co, F, theLines);
  if (auto Sp = occ::down_cast<Geom_SphericalSurface>(S))
    Make<GeomToStep_MakeSphericalSurface>(theId, "sphere", Sp, F, theLines);
  if (auto To = occ::down_cast<Geom_ToroidalSurface>(S))
    Make<GeomToStep_MakeToroidalSurface>(theId, "torus", To, F, theLines);
  if (auto Ex = occ::down_cast<Geom_SurfaceOfLinearExtrusion>(S))
    Make<GeomToStep_MakeSurfaceOfLinearExtrusion>(theId, "extrusion", Ex, F, theLines);
  if (auto Rv = occ::down_cast<Geom_SurfaceOfRevolution>(S))
    Make<GeomToStep_MakeSurfaceOfRevolution>(theId, "revolution", Rv, F, theLines);
  if (auto RT = occ::down_cast<Geom_RectangularTrimmedSurface>(S))
    Make<GeomToStep_MakeRectangularTrimmedSurface>(theId, "rtrimmed", RT, F, theLines);
  if (auto BS = occ::down_cast<Geom_BSplineSurface>(S))
  {
    Make<GeomToStep_MakeBSplineSurfaceWithKnots>(theId, "bspline", BS, F, theLines);
    Make<GeomToStep_MakeBSplineSurfaceWithKnotsAndRationalBSplineSurface>(theId, "rbspline", BS, F, theLines);
  }
}

// ---------------------------------------------------------------------------------------------------------------
// The cases: one per line, "<kind> <values>"

static std::vector<double> Numbers(std::istringstream& theIn)
{
  std::vector<double> aRes;
  std::string         aTok;
  while (theIn >> aTok)
    aRes.push_back(std::strtod(aTok.c_str(), nullptr));
  return aRes;
}

static gp_Trsf MakeTrsf(const std::vector<double>& v)
{
  // operations of 8 numbers: kind (1 rotation about the axis p, d by angle a; 2 translation by p; 3 scale about p by
  // a; 4 mirror through the point p), then px py pz dx dy dz a; each applied after the previous ones
  gp_Trsf T;
  for (size_t i = 0; i + 7 < v.size(); i += 8)
  {
    gp_Trsf t;
    gp_Pnt  p(v[i + 1], v[i + 2], v[i + 3]);
    switch ((int)v[i])
    {
      case 1: t.SetRotation(gp_Ax1(p, gp_Dir(v[i + 4], v[i + 5], v[i + 6])), v[i + 7]); break;
      case 2: t.SetTranslation(gp_Vec(v[i + 1], v[i + 2], v[i + 3])); break;
      case 3: t.SetScale(p, v[i + 7]); break;
      default: t.SetMirror(p); break;
    }
    T.PreMultiply(t);
  }
  return T;
}

static void CaseLines(const std::string& theId, const std::string& theLine, const StepData_Factors& F, std::vector<std::string>& theLines)
{
  std::istringstream anIn(theLine);
  std::string        aKind;
  anIn >> aKind;
  if (aKind == "C3")
  {
    occ::handle<Geom_Curve> C = GeomTools_CurveSet::ReadCurve(anIn);
    CurveMakers(theId, C, F, theLines);
    return;
  }
  if (aKind == "C2")
  {
    occ::handle<Geom2d_Curve> C = GeomTools_Curve2dSet::ReadCurve2d(anIn);
    Curve2dMakers(theId, C, F, theLines);
    return;
  }
  if (aKind == "S")
  {
    occ::handle<Geom_Surface> S = GeomTools_SurfaceSet::ReadSurface(anIn);
    SurfaceMakers(theId, S, F, theLines);
    return;
  }
  std::vector<double> v = Numbers(anIn);
  theLines.push_back(theId + " " + aKind);
  if (aKind == "PNT")
  {
    gp_Pnt P(v[0], v[1], v[2]);
    try { theLines.push_back(theId + " pnt " + Written(GeomToStep_MakeCartesianPoint(P, F.LengthFactor()).Value())); } catch (...) { theLines.push_back(theId + " pnt EXC"); }
    occ::handle<Geom_CartesianPoint> G = new Geom_CartesianPoint(P);
    try { theLines.push_back(theId + " gpnt " + Written(GeomToStep_MakeCartesianPoint(G, F.LengthFactor()).Value())); } catch (...) { theLines.push_back(theId + " gpnt EXC"); }
  }
  else if (aKind == "PNT2")
  {
    gp_Pnt2d P(v[0], v[1]);
    try { theLines.push_back(theId + " pnt " + Written(GeomToStep_MakeCartesianPoint(P, F.LengthFactor()).Value())); } catch (...) { theLines.push_back(theId + " pnt EXC"); }
    occ::handle<Geom2d_CartesianPoint> G = new Geom2d_CartesianPoint(P);
    try { theLines.push_back(theId + " gpnt " + Written(GeomToStep_MakeCartesianPoint(G).Value())); } catch (...) { theLines.push_back(theId + " gpnt EXC"); }
  }
  else if (aKind == "DIR")
  {
    gp_Dir D(v[0], v[1], v[2]);
    Make0<GeomToStep_MakeDirection>(theId, "dir", D, F, theLines);
    Make0<GeomToStep_MakeDirection>(theId, "gdir", occ::handle<Geom_Direction>(new Geom_Direction(D)), F, theLines);
  }
  else if (aKind == "DIR2")
  {
    gp_Dir2d D(v[0], v[1]);
    Make0<GeomToStep_MakeDirection>(theId, "dir", D, F, theLines);
    Make0<GeomToStep_MakeDirection>(theId, "gdir", occ::handle<Geom2d_Direction>(new Geom2d_Direction(D)), F, theLines);
  }
  else if (aKind == "VEC")
  {
    gp_Vec V(v[0], v[1], v[2]);
    Make<GeomToStep_MakeVector>(theId, "vec", V, F, theLines);
    Make<GeomToStep_MakeVector>(theId, "gvec", occ::handle<Geom_Vector>(new Geom_VectorWithMagnitude(V)), F, theLines);
  }
  else if (aKind == "VEC2")
  {
    gp_Vec2d V(v[0], v[1]);
    Make<GeomToStep_MakeVector>(theId, "vec", V, F, theLines);
    Make<GeomToStep_MakeVector>(theId, "gvec", occ::handle<Geom2d_Vector>(new Geom2d_VectorWithMagnitude(V)), F, theLines);
  }
  else if (aKind == "AX1")
  {
    gp_Ax1 A(gp_Pnt(v[0], v[1], v[2]), gp_Dir(v[3], v[4], v[5]));
    Make<GeomToStep_MakeAxis1Placement>(theId, "ax1", A, F, theLines);
    Make<GeomToStep_MakeAxis1Placement>(theId, "gax1", occ::handle<Geom_Axis1Placement>(new Geom_Axis1Placement(A)), F, theLines);
  }
  else if (aKind == "AX2D")
  {
    gp_Ax2d A(gp_Pnt2d(v[0], v[1]), gp_Dir2d(v[2], v[3]));
    Make<GeomToStep_MakeAxis1Placement>(theId, "ax2d", A, F, theLines);
    Make<GeomToStep_MakeAxis1Placement>(theId, "gax2d", occ::handle<Geom2d_AxisPlacement>(new Geom2d_AxisPlacement(A)), F, theLines);
  }
  else if (aKind == "AX2")
  {
    gp_Ax2 A(gp_Pnt(v[0], v[1], v[2]), gp_Dir(v[3], v[4], v[5]), gp_Dir(v[6], v[7], v[8]));
    Make<GeomToStep_MakeAxis2Placement3d>(theId, "ax2", A, F, theLines);
    Make<GeomToStep_MakeAxis2Placement3d>(theId, "gax2", occ::handle<Geom_Axis2Placement>(new Geom_Axis2Placement(A)), F, theLines);
    Make<GeomToStep_MakeAxis2Placement2d>(theId, "ax2to2d", A, F, theLines);
    gp_Ax3 A3(A);
    if (v.size() > 9 && v[9] == 0.)
      A3.YReverse();
    Make<GeomToStep_MakeAxis2Placement3d>(theId, "ax3", A3, F, theLines);
  }
  else if (aKind == "AX22D")
  {
    gp_Ax22d A(gp_Pnt2d(v[0], v[1]), gp_Dir2d(v[2], v[3]), v[4] != 0.);
    Make<GeomToStep_MakeAxis2Placement2d>(theId, "ax22d", A, F, theLines);
  }
  else if (aKind == "TRSF")
  {
    gp_Trsf T = MakeTrsf(v);
    theLines.push_back(theId + " " + Trsf(T));
    Make<GeomToStep_MakeCartesianTransformationOperator>(theId, "cto", T, F, theLines);
    Make<GeomToStep_MakeAxis2Placement3d>(theId, "ax2trsf", T, F, theLines);
  }
  else if (aKind == "ORIGIN")
  {
    try { theLines.push_back(theId + " origin " + Written(GeomToStep_MakeAxis2Placement3d(F).Value())); } catch (...) { theLines.push_back(theId + " origin EXC"); }
  }
  else if (aKind == "PL3")
  {
    int n = (int)v[0];
    NCollection_Array1<gp_Pnt> P(1, n);
    for (int i = 1; i <= n; i++)
      P.SetValue(i, gp_Pnt(v[3 * i - 2], v[3 * i - 1], v[3 * i]));
    Make<GeomToStep_MakePolyline>(theId, "polyline", P, F, theLines);
  }
  else if (aKind == "PL2")
  {
    int n = (int)v[0];
    NCollection_Array1<gp_Pnt2d> P(1, n);
    for (int i = 1; i <= n; i++)
      P.SetValue(i, gp_Pnt2d(v[2 * i - 1], v[2 * i]));
    Make<GeomToStep_MakePolyline>(theId, "polyline", P, F, theLines);
  }
}

// ---------------------------------------------------------------------------------------------------------------
// The geometry of a shape: surfaces of the faces, then per edge its 3D curves (with their trimmed ranges) and
// pcurves, each geometry once, in the order met

static void ShapeLines(const TopoDS_Shape& theShape, const StepData_Factors& F, std::vector<std::string>& theLines)
{
  std::map<const Standard_Transient*, int> aSeen;
  int                                      aNb = 0;
  for (TopExp_Explorer anExp(theShape, TopAbs_FACE); anExp.More(); anExp.Next())
  {
    TopLoc_Location                  L;
    const occ::handle<Geom_Surface>& S = BRep_Tool::Surface(TopoDS::Face(anExp.Current()), L);
    if (S.IsNull() || aSeen.count(S.get()))
      continue;
    aSeen[S.get()] = ++aNb;
    SurfaceMakers("s" + std::to_string(aNb), S, F, theLines);
  }
  for (TopExp_Explorer anExp(theShape, TopAbs_EDGE); anExp.More(); anExp.Next())
  {
    occ::handle<BRep_TEdge> TE = occ::down_cast<BRep_TEdge>(anExp.Current().TShape());
    for (const occ::handle<BRep_CurveRepresentation>& aRep : TE->Curves())
    {
      if (aRep->IsCurve3D())
      {
        occ::handle<Geom_Curve> C = aRep->Curve3D();
        if (C.IsNull() || aSeen.count(C.get()))
          continue;
        aSeen[C.get()] = ++aNb;
        std::string anId = "c" + std::to_string(aNb);
        CurveMakers(anId, C, F, theLines);
        occ::handle<BRep_GCurve> GC = occ::down_cast<BRep_GCurve>(aRep);
        occ::handle<Geom_Curve>  T;
        try
        {
          T = new Geom_TrimmedCurve(C, GC->First(), GC->Last());
        }
        catch (...)
        {
          theLines.push_back(anId + "t EXC");
        }
        if (!T.IsNull())
          CurveMakers(anId + "t", T, F, theLines);
      }
      else if (aRep->IsCurveOnSurface())
      {
        occ::handle<Geom2d_Curve> P[2] = {aRep->PCurve(), aRep->IsCurveOnClosedSurface() ? aRep->PCurve2() : nullptr};
        for (const occ::handle<Geom2d_Curve>& C : P)
        {
          if (C.IsNull() || aSeen.count(C.get()))
            continue;
          aSeen[C.get()] = ++aNb;
          Curve2dMakers("p" + std::to_string(aNb), C, F, theLines);
        }
      }
    }
  }
}

// ---------------------------------------------------------------------------------------------------------------
// gen: the constructed cases

static std::string W(const occ::handle<Geom_Curve>& C) { return "C3 " + Curve(C); }
static std::string W(const occ::handle<Geom2d_Curve>& C) { return "C2 " + Curve2d(C); }
static std::string W(const occ::handle<Geom_Surface>& S) { return "S " + Surface(S); }

static void Gen(FILE* theOut)
{
  std::vector<std::string> aCases;
  const gp_Ax2 aTilted(gp_Pnt(1.5, -2.25, 3.125), gp_Dir(0.3, -0.4, 0.866), gp_Dir(1., 0., -0.3 / 0.866));
  const gp_Ax2 aZ(gp_Pnt(0., 0., 0.), gp_Dir(0., 0., 1.));

  // 3D curves
  occ::handle<Geom_Line>      aLine = new Geom_Line(gp_Pnt(1, 2, 3), gp_Dir(1, 1, 0));
  occ::handle<Geom_Circle>    aCirc = new Geom_Circle(aTilted, 5.);
  occ::handle<Geom_Ellipse>   anEll = new Geom_Ellipse(aTilted, 7., 3.);
  occ::handle<Geom_Hyperbola> aHyp  = new Geom_Hyperbola(aTilted, 4., 2.);
  occ::handle<Geom_Parabola>  aPar  = new Geom_Parabola(aTilted, 1.5);
  aCases.push_back(W(aLine));
  aCases.push_back(W(occ::handle<Geom_Curve>(new Geom_Circle(aZ, 10.))));
  aCases.push_back(W(aCirc));
  aCases.push_back(W(anEll));
  aCases.push_back(W(aHyp));
  aCases.push_back(W(aPar));

  NCollection_Array1<gp_Pnt> aP4(1, 4);
  aP4(1) = gp_Pnt(0, 0, 0); aP4(2) = gp_Pnt(1, 2, 0.5); aP4(3) = gp_Pnt(3, 2.5, -1); aP4(4) = gp_Pnt(4, 0, 0.25);
  NCollection_Array1<double> aW4(1, 4);
  aW4(1) = 1.; aW4(2) = 0.7; aW4(3) = 1.3; aW4(4) = 1.;
  occ::handle<Geom_BezierCurve> aBez  = new Geom_BezierCurve(aP4);
  occ::handle<Geom_BezierCurve> aRBez = new Geom_BezierCurve(aP4, aW4);
  aCases.push_back(W(aBez));
  aCases.push_back(W(aRBez));

  NCollection_Array1<gp_Pnt> aP7(1, 7);
  for (int i = 1; i <= 7; i++)
    aP7(i) = gp_Pnt(i * 1.25, std::sin(i * 0.9) * 3., std::cos(i * 0.7));
  NCollection_Array1<double> aW7(1, 7);
  for (int i = 1; i <= 7; i++)
    aW7(i) = 1. + 0.1 * (i % 3);
  {
    // non-uniform cubic
    NCollection_Array1<double> K(1, 5); K(1) = 0; K(2) = 1; K(3) = 2.5; K(4) = 3; K(5) = 4.5;
    NCollection_Array1<int>    M(1, 5); M(1) = 4; M(2) = 1; M(3) = 1; M(4) = 1; M(5) = 4;
    occ::handle<Geom_BSplineCurve> B  = new Geom_BSplineCurve(aP7, K, M, 3);
    occ::handle<Geom_BSplineCurve> RB = new Geom_BSplineCurve(aP7, aW7, K, M, 3);
    aCases.push_back(W(B));
    aCases.push_back(W(RB));
    aCases.push_back(W(occ::handle<Geom_Curve>(new Geom_TrimmedCurve(B, 0.5, 3.75))));
    aCases.push_back(W(occ::handle<Geom_Curve>(new Geom_OffsetCurve(B, 0.5, gp_Dir(0, 0, 1)))));
  }
  {
    // uniform quadratic (knots 0..5, interior mults 1, end mults 3)
    NCollection_Array1<double> K(1, 6); for (int i = 1; i <= 6; i++) K(i) = i - 1;
    NCollection_Array1<int>    M(1, 6); for (int i = 1; i <= 6; i++) M(i) = 1;
    M(1) = 3; M(6) = 3;
    aCases.push_back(W(occ::handle<Geom_Curve>(new Geom_BSplineCurve(aP7, K, M, 2))));
    // piecewise Bezier (interior mults = degree)
    NCollection_Array1<double> K2(1, 3); K2(1) = 0; K2(2) = 1; K2(3) = 2;
    NCollection_Array1<int>    M2(1, 3); M2(1) = 4; M2(2) = 3; M2(3) = 4;
    aCases.push_back(W(occ::handle<Geom_Curve>(new Geom_BSplineCurve(aP7, K2, M2, 3))));
  }
  {
    // periodic, rational periodic
    NCollection_Array1<double> K(1, 8); for (int i = 1; i <= 8; i++) K(i) = (i - 1) * 0.5;
    NCollection_Array1<int>    M(1, 8); for (int i = 1; i <= 8; i++) M(i) = 1;
    occ::handle<Geom_BSplineCurve> P  = new Geom_BSplineCurve(aP7, K, M, 3, true);
    occ::handle<Geom_BSplineCurve> RP = new Geom_BSplineCurve(aP7, aW7, K, M, 3, true);
    aCases.push_back(W(P));
    aCases.push_back(W(RP));
    aCases.push_back(W(occ::handle<Geom_Curve>(new Geom_TrimmedCurve(P, 0.25, 2.75))));
  }
  aCases.push_back(W(occ::handle<Geom_Curve>(new Geom_TrimmedCurve(aLine, -2., 7.5))));
  aCases.push_back(W(occ::handle<Geom_Curve>(new Geom_TrimmedCurve(aCirc, 0.5, 4.))));
  aCases.push_back(W(occ::handle<Geom_Curve>(new Geom_TrimmedCurve(anEll, 5., 1., false))));
  aCases.push_back(W(occ::handle<Geom_Curve>(new Geom_TrimmedCurve(aRBez, 0.2, 0.9))));
  aCases.push_back(W(occ::handle<Geom_Curve>(new Geom_TrimmedCurve(aHyp, -1., 1.5))));

  // 2D curves
  const gp_Ax22d aDirect(gp_Pnt2d(1., -2.), gp_Dir2d(0.6, 0.8), true);
  const gp_Ax22d anIndirect(gp_Pnt2d(-3., 0.5), gp_Dir2d(0.8, -0.6), false);
  occ::handle<Geom2d_Line> aLine2d = new Geom2d_Line(gp_Pnt2d(0.5, 1.5), gp_Dir2d(1., -1.));
  aCases.push_back(W(occ::handle<Geom2d_Curve>(aLine2d)));
  aCases.push_back(W(occ::handle<Geom2d_Curve>(new Geom2d_Circle(aDirect, 2.5))));
  aCases.push_back(W(occ::handle<Geom2d_Curve>(new Geom2d_Circle(anIndirect, 2.5))));
  aCases.push_back(W(occ::handle<Geom2d_Curve>(new Geom2d_Ellipse(aDirect, 4., 1.5))));
  aCases.push_back(W(occ::handle<Geom2d_Curve>(new Geom2d_Ellipse(anIndirect, 4., 1.5))));
  aCases.push_back(W(occ::handle<Geom2d_Curve>(new Geom2d_Hyperbola(aDirect, 3., 1.))));
  aCases.push_back(W(occ::handle<Geom2d_Curve>(new Geom2d_Parabola(aDirect, 0.75))));
  NCollection_Array1<gp_Pnt2d> aQ5(1, 5);
  for (int i = 1; i <= 5; i++)
    aQ5(i) = gp_Pnt2d(i * 0.75, std::sin(i * 1.1));
  NCollection_Array1<double> aW5(1, 5);
  for (int i = 1; i <= 5; i++)
    aW5(i) = 1. + 0.2 * (i % 2);
  aCases.push_back(W(occ::handle<Geom2d_Curve>(new Geom2d_BezierCurve(aQ5))));
  aCases.push_back(W(occ::handle<Geom2d_Curve>(new Geom2d_BezierCurve(aQ5, aW5))));
  {
    NCollection_Array1<double> K(1, 4); K(1) = 0; K(2) = 0.3; K(3) = 0.6; K(4) = 1;
    NCollection_Array1<int>    M(1, 4); M(1) = 3; M(2) = 1; M(3) = 1; M(4) = 3;
    occ::handle<Geom2d_BSplineCurve> B = new Geom2d_BSplineCurve(aQ5, K, M, 2);
    aCases.push_back(W(occ::handle<Geom2d_Curve>(B)));
    aCases.push_back(W(occ::handle<Geom2d_Curve>(new Geom2d_BSplineCurve(aQ5, aW5, K, M, 2))));
    NCollection_Array1<double> KP(1, 6); for (int i = 1; i <= 6; i++) KP(i) = i - 1;
    NCollection_Array1<int>    MP(1, 6); for (int i = 1; i <= 6; i++) MP(i) = 1;
    aCases.push_back(W(occ::handle<Geom2d_Curve>(new Geom2d_BSplineCurve(aQ5, aW5, KP, MP, 2, true))));
    aCases.push_back(W(occ::handle<Geom2d_Curve>(new Geom2d_TrimmedCurve(B, 0.1, 0.8))));
    aCases.push_back(W(occ::handle<Geom2d_Curve>(new Geom2d_TrimmedCurve(aLine2d, -1., 2.))));
    aCases.push_back(W(occ::handle<Geom2d_Curve>(new Geom2d_OffsetCurve(B, 0.25))));
  }

  // surfaces
  occ::handle<Geom_Plane>              aPln  = new Geom_Plane(gp_Ax3(aTilted));
  occ::handle<Geom_CylindricalSurface> aCyl  = new Geom_CylindricalSurface(gp_Ax3(aTilted), 2.5);
  occ::handle<Geom_ConicalSurface>     aCone = new Geom_ConicalSurface(gp_Ax3(aTilted), 0.3, 4.);
  occ::handle<Geom_SphericalSurface>   aSph  = new Geom_SphericalSurface(gp_Ax3(aTilted), 3.);
  occ::handle<Geom_ToroidalSurface>    aTor  = new Geom_ToroidalSurface(gp_Ax3(aTilted), 6., 1.5);
  gp_Ax3                               anInd(aTilted);
  anInd.YReverse();
  aCases.push_back(W(occ::handle<Geom_Surface>(aPln)));
  aCases.push_back(W(occ::handle<Geom_Surface>(new Geom_Plane(anInd))));
  aCases.push_back(W(occ::handle<Geom_Surface>(aCyl)));
  aCases.push_back(W(occ::handle<Geom_Surface>(aCone)));
  aCases.push_back(W(occ::handle<Geom_Surface>(new Geom_ConicalSurface(gp_Ax3(aZ), -0.4, 2.))));
  aCases.push_back(W(occ::handle<Geom_Surface>(aSph)));
  aCases.push_back(W(occ::handle<Geom_Surface>(aTor)));
  occ::handle<Geom_BSplineCurve> aProfile;
  {
    NCollection_Array1<double> K(1, 5); K(1) = 0; K(2) = 1; K(3) = 2.5; K(4) = 3; K(5) = 4.5;
    NCollection_Array1<int>    M(1, 5); M(1) = 4; M(2) = 1; M(3) = 1; M(4) = 1; M(5) = 4;
    aProfile = new Geom_BSplineCurve(aP7, K, M, 3);
  }
  occ::handle<Geom_SurfaceOfRevolution> aRev =
    new Geom_SurfaceOfRevolution(aProfile, gp_Ax1(gp_Pnt(0, -5, 0), gp_Dir(1, 0, 0)));
  aCases.push_back(W(occ::handle<Geom_Surface>(aRev)));
  aCases.push_back(W(occ::handle<Geom_Surface>(new Geom_SurfaceOfRevolution(aCirc, gp_Ax1(gp_Pnt(20, 0, 0), gp_Dir(0, 1, 0))))));
  aCases.push_back(W(occ::handle<Geom_Surface>(new Geom_SurfaceOfLinearExtrusion(anEll, gp_Dir(0.1, 0.2, 1.)))));
  aCases.push_back(W(occ::handle<Geom_Surface>(new Geom_SurfaceOfLinearExtrusion(aLine, gp_Dir(0., 0., 1.)))));
  NCollection_Array2<gp_Pnt> aG(1, 4, 1, 3);
  NCollection_Array2<double> aGW(1, 4, 1, 3);
  for (int i = 1; i <= 4; i++)
    for (int j = 1; j <= 3; j++)
    {
      aG(i, j)  = gp_Pnt(i * 2., j * 1.5, std::sin(i + 2. * j));
      aGW(i, j) = 1. + 0.15 * ((i + j) % 3);
    }
  aCases.push_back(W(occ::handle<Geom_Surface>(new Geom_BezierSurface(aG))));
  aCases.push_back(W(occ::handle<Geom_Surface>(new Geom_BezierSurface(aG, aGW))));
  occ::handle<Geom_BSplineSurface> aBSS;
  {
    NCollection_Array1<double> UK(1, 2); UK(1) = 0; UK(2) = 1;
    NCollection_Array1<int>    UM(1, 2); UM(1) = 4; UM(2) = 4;
    NCollection_Array1<double> VK(1, 2); VK(1) = 0; VK(2) = 2;
    NCollection_Array1<int>    VM(1, 2); VM(1) = 3; VM(2) = 3;
    aBSS = new Geom_BSplineSurface(aG, UK, VK, UM, VM, 3, 2);
    aCases.push_back(W(occ::handle<Geom_Surface>(aBSS)));
    aCases.push_back(W(occ::handle<Geom_Surface>(new Geom_BSplineSurface(aG, aGW, UK, VK, UM, VM, 3, 2))));
    NCollection_Array1<double> PK(1, 5); for (int i = 1; i <= 5; i++) PK(i) = i - 1;
    NCollection_Array1<int>    PM(1, 5); for (int i = 1; i <= 5; i++) PM(i) = 1;
    aCases.push_back(W(occ::handle<Geom_Surface>(new Geom_BSplineSurface(aG, aGW, PK, VK, PM, VM, 2, 2, true, false))));
  }
  aCases.push_back(W(occ::handle<Geom_Surface>(new Geom_RectangularTrimmedSurface(aPln, -1., 2., -3., 4.))));
  aCases.push_back(W(occ::handle<Geom_Surface>(new Geom_RectangularTrimmedSurface(aCyl, 0.5, 2.5, -1., 3.))));
  aCases.push_back(W(occ::handle<Geom_Surface>(new Geom_RectangularTrimmedSurface(aCone, 0., 3., 0.5, 2.))));
  aCases.push_back(W(occ::handle<Geom_Surface>(new Geom_RectangularTrimmedSurface(aSph, 0.1, 3., -1., 1.))));
  aCases.push_back(W(occ::handle<Geom_Surface>(new Geom_RectangularTrimmedSurface(aTor, 0., 4., 1., 5.))));
  aCases.push_back(W(occ::handle<Geom_Surface>(new Geom_RectangularTrimmedSurface(aRev, 0., 2., 0.5, 4.))));
  aCases.push_back(W(occ::handle<Geom_Surface>(new Geom_RectangularTrimmedSurface(aBSS, 0.2, 0.8, 0.5, 1.5))));
  aCases.push_back(W(occ::handle<Geom_Surface>(new Geom_OffsetSurface(aPln, 1.25))));
  aCases.push_back(W(occ::handle<Geom_Surface>(new Geom_OffsetSurface(aCyl, -0.5))));
  aCases.push_back(W(occ::handle<Geom_Surface>(new Geom_OffsetSurface(aBSS, 0.3))));

  for (const std::string& aCase : aCases)
    fprintf(theOut, "%s\n", aCase.c_str());

  // gp values (the numbers at precision 17)
  auto N = [](std::initializer_list<double> theValues) {
    std::string aRes;
    for (double v : theValues)
      aRes += " " + R(v);
    return aRes;
  };
  const double PI = M_PI;
  fprintf(theOut, "PNT%s\n", N({1.5, -2.25, 1e-3}).c_str());
  fprintf(theOut, "PNT%s\n", N({123456.789, 0., -98.7654321}).c_str());
  fprintf(theOut, "PNT2%s\n", N({0.25, -7.5}).c_str());
  fprintf(theOut, "DIR%s\n", N({0.3, -0.4, 0.866}).c_str());
  fprintf(theOut, "DIR%s\n", N({0., 0., -1.}).c_str());
  fprintf(theOut, "DIR2%s\n", N({-0.6, 0.8}).c_str());
  fprintf(theOut, "VEC%s\n", N({3., -4., 12.}).c_str());
  fprintf(theOut, "VEC%s\n", N({1e-3, 0., 0.}).c_str());
  fprintf(theOut, "VEC2%s\n", N({-2.5, 6.}).c_str());
  fprintf(theOut, "AX1%s\n", N({1., 2., 3., 0., 1., 1.}).c_str());
  fprintf(theOut, "AX2D%s\n", N({-1., 0.5, 1., 1.}).c_str());
  fprintf(theOut, "AX2%s\n", N({1.5, -2.25, 3.125, 0.3, -0.4, 0.866, 1., 0., -0.3 / 0.866, 1.}).c_str());
  fprintf(theOut, "AX2%s\n", N({0., 0., 0., 0., 0., 1., 1., 0., 0., 0.}).c_str());
  fprintf(theOut, "AX22D%s\n", N({1., -2., 0.6, 0.8, 1.}).c_str());
  fprintf(theOut, "AX22D%s\n", N({-3., 0.5, 0.8, -0.6, 0.}).c_str());
  fprintf(theOut, "TRSF%s\n", N({2., 10., -5., 2.5, 0., 0., 0., 0.}).c_str());
  fprintf(theOut, "TRSF%s\n", N({1., 0., 0., 0., 0., 0., 1., PI / 3., 2., 1., 2., 3., 0., 0., 0., 0.}).c_str());
  fprintf(theOut, "TRSF%s\n", N({1., 1., 1., 0., 1., 1., 1., 0.7, 3., 0., 0., 0., 0., 0., 0., 2.5, 2., -4., 0., 1., 0., 0., 0., 0.}).c_str());
  fprintf(theOut, "TRSF%s\n", N({4., 1., 2., 3., 0., 0., 0., 0., 1., 0., 0., 0., 1., 0., 0., 0.25}).c_str());
  fprintf(theOut, "ORIGIN\n");
  fprintf(theOut, "PL3%s\n", N({4., 0., 0., 0., 1., 0., 0., 1., 1., 0.5, 0., 1., 1.}).c_str());
  fprintf(theOut, "PL2%s\n", N({3., 0., 0., 2., 0.5, -1., 3.}).c_str());
}

// ---------------------------------------------------------------------------------------------------------------

int main(int argc, char** argv)
{
  if (argc < 3)
  {
    fprintf(stderr, "geomsteprec gen <cases> | s2g <list> <out> [full] | g2s <cases> <list> <out> [full]\n");
    return 2;
  }
  // The messages go nowhere (they are OCCT's traces, not compared)
  Message::DefaultMessenger()->RemovePrinters(STANDARD_TYPE(Message_PrinterOStream));
  // STEPControl's controller (its protocol and libraries, Shape Healing's ShapeAlgo container)
  STEPControl_Reader anInit;
  theProtocol = StepAP214::Protocol();
  std::string aMode = argv[1];
  const StepData_Factors aFactors[2] = {StepData_Factors(), Inches()};
  const char* const      aTags[2]    = {"mm", "in"};

  if (aMode == "gen")
  {
    FILE* anOut = fopen(argv[2], "wb");
    Gen(anOut);
    fclose(anOut);
    return 0;
  }
  if (aMode == "s2g" && argc >= 4)
  {
    FILE* anOut = fopen(argv[3], "wb");
    bool  isFull = argc > 4 && std::string(argv[4]) == "full";
    for (const auto& anInput : ReadList(argv[2]))
    {
      fprintf(anOut, "FILE %s\n", anInput.first.c_str());
      STEPControl_Reader aReader;
      if (aReader.ReadFile(anInput.second.c_str()) != IFSelect_RetDone)
      {
        fprintf(anOut, "READFAIL\n");
        continue;
      }
      occ::handle<StepData_StepModel> aModel = aReader.StepModel();
      // Only the entities of the packages the port reads so far (unit 8C1's StepBasic, StepGeom, StepShape and
      // StepRepr): the others (StepFEA's FEA_AXIS2_PLACEMENT_3D...) are counted, not converted
      std::vector<bool> isKept(aModel->NbEntities() + 1, false);
      int               aNbSkipped = 0;
      for (int i = 1; i <= aModel->NbEntities(); i++)
      {
        const std::string aType = aModel->Value(i)->DynamicType()->Name();
        isKept[i] = aType.rfind("StepGeom_", 0) == 0 || aType.rfind("StepShape_", 0) == 0
                    || aType.rfind("StepRepr_", 0) == 0 || aType.rfind("StepBasic_", 0) == 0;
        if (!isKept[i])
        {
          std::vector<std::string> aProbe;
          StepToGeomLines(aModel->Value(i), aFactors[0], "", aProbe);
          aNbSkipped += aProbe.empty() ? 0 : 1;
        }
      }
      if (aNbSkipped > 0)
        fprintf(anOut, "SKIPPED %d\n", aNbSkipped);
      for (int f = 0; f < 2; f++)
      {
        std::vector<std::string> aLines;
        for (int i = 1; i <= aModel->NbEntities(); i++)
          if (isKept[i])
            StepToGeomLines(aModel->Value(i), aFactors[f], std::to_string(i) + " ", aLines);
        Block(anOut, aTags[f], aLines, isFull);
      }
      fflush(anOut);
    }
    fclose(anOut);
    return 0;
  }
  if (aMode == "g2s" && argc >= 5)
  {
    FILE* anOut  = fopen(argv[4], "wb");
    bool  isFull = argc > 5 && std::string(argv[5]) == "full";
    std::vector<std::string> aCases;
    {
      std::ifstream aFile(argv[2]);
      std::string   aLine;
      while (std::getline(aFile, aLine))
      {
        if (!aLine.empty() && aLine.back() == '\r')
          aLine.pop_back();
        if (!aLine.empty())
          aCases.push_back(aLine);
      }
    }
    fprintf(anOut, "CASES %zu\n", aCases.size());
    for (int f = 0; f < 2; f++)
    {
      std::vector<std::string> aLines;
      for (size_t i = 0; i < aCases.size(); i++)
      {
        try
        {
          CaseLines("k" + std::to_string(i + 1), aCases[i], aFactors[f], aLines);
        }
        catch (...)
        {
          aLines.push_back("k" + std::to_string(i + 1) + " EXC");
        }
      }
      Block(anOut, aTags[f], aLines, true);
    }
    for (const auto& anInput : ReadList(argv[3]))
    {
      fprintf(anOut, "FILE %s\n", anInput.first.c_str());
      TopoDS_Shape aShape;
      BRep_Builder aBuilder;
      if (!BRepTools::Read(aShape, anInput.second.c_str(), aBuilder))
      {
        fprintf(anOut, "READFAIL\n");
        continue;
      }
      for (int f = 0; f < 2; f++)
      {
        std::vector<std::string> aLines;
        ShapeLines(aShape, aFactors[f], aLines);
        Block(anOut, aTags[f], aLines, isFull);
      }
      fflush(anOut);
    }
    fclose(anOut);
    return 0;
  }
  return 2;
}
