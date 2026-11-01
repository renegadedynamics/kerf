// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
//
// shcustomrec: records native OCCT's ShapeCustom where build123d.NET's bindings can't reach it (unit 7A):
// ShapeCustom::BSplineRestriction with restriction parameters other than the defaults (the setters aren't bound), and
// the geometry tools ShapeCustom_Surface (ConvertToAnalytical, ConvertToPeriodic), ShapeCustom_Curve
// (ConvertToPeriodic) and ShapeCustom_Curve2d (ConvertToLine2d, SimplifyBSpline2d), on the faces, edges and pcurves of
// the input shapes. Built against build123d.NET's native OCCT V8_0_1 (tools/shcustomrec/build.ps1).
//
// Inputs: the BRep files listed one per line in the file given as the first argument (data/occ's and the heal group's,
// chosen by build.ps1); the recording goes to the file given as the second argument. Per input:
//   FILE <name>                         the file's name (the tests find it under data/occ or the test dataset)
//   BSR <set>                           BSplineRestriction with the parameter set (1, 2), inputs up to 30 kB, then
//   THROW <type>  or  OUT <n>           n lines: the result's BRep text (format 3)
//   SURF <face>                         a B-spline or Bezier surface (BRep_Tool::Surface(F, L)), face index in
//                                       TopExp::MapShapes(FACE)
//   ANA <tol> <gap> <n>                 ConvertToAnalytical(tol, false): gap (hex), n lines of GeomTools text (0: null)
//   PER <n>                             ConvertToPeriodic(false): n lines (0: null)
//   CURV <edge>                         a B-spline 3D curve, edge index in TopExp::MapShapes(EDGE)
//   PER <n>
//   PCURVE <edge> <face>                a B-spline or Bezier pcurve
//   LINE <cf> <cl> <dev> <n>            ConvertToLine2d(c, f, l, 1e-4): the outputs (hex) and the line (0: null)
//   SIMP <0|1> <n>                      (B-splines) SimplifyBSpline2d on a copy, tolerance Precision::Approximation()
//   THROW <type>                        in place of a line above if the call threw
//
//   shcustomrec inputs.txt shcustomrec.txt
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <typeinfo>
#include <vector>

#include <BRepTools.hxx>
#include <BRep_Builder.hxx>
#include <BRep_Tool.hxx>
#include <Geom2d_BSplineCurve.hxx>
#include <Geom2d_BezierCurve.hxx>
#include <Geom2d_Line.hxx>
#include <Geom_BSplineCurve.hxx>
#include <Geom_BSplineSurface.hxx>
#include <Geom_BezierSurface.hxx>
#include <GeomTools.hxx>
#include <OSD.hxx>
#include <Precision.hxx>
#include <ShapeCustom.hxx>
#include <ShapeCustom_Curve.hxx>
#include <ShapeCustom_Curve2d.hxx>
#include <ShapeCustom_RestrictionParameters.hxx>
#include <ShapeCustom_Surface.hxx>
#include <Standard_ErrorHandler.hxx>
#include <Standard_Failure.hxx>
#include <TopExp.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopTools_IndexedMapOfShape.hxx>

static std::string Hex(double theValue)
{
  uint64_t aBits;
  std::memcpy(&aBits, &theValue, 8);
  char aBuf[20];
  std::snprintf(aBuf, sizeof(aBuf), "%016llx", (unsigned long long)aBits);
  return aBuf;
}

static std::ofstream gOut;

// Writes "<head> <n>" and the n lines of the text.
static void Block(const std::string& theHead, const std::string& theText)
{
  int aNb = 0;
  for (char c : theText)
  {
    aNb += c == '\n' ? 1 : 0;
  }
  if (!theText.empty() && theText.back() != '\n')
  {
    ++aNb;
  }
  gOut << theHead << " " << aNb << "\n" << theText;
  if (!theText.empty() && theText.back() != '\n')
  {
    gOut << "\n";
  }
}

static std::string Text(const occ::handle<Geom_Surface>& theS)
{
  if (theS.IsNull())
  {
    return "";
  }
  std::ostringstream aS;
  GeomTools::Write(theS, aS);
  return aS.str();
}

static std::string Text(const occ::handle<Geom_Curve>& theC)
{
  if (theC.IsNull())
  {
    return "";
  }
  std::ostringstream aS;
  GeomTools::Write(theC, aS);
  return aS.str();
}

static std::string Text(const occ::handle<Geom2d_Curve>& theC)
{
  if (theC.IsNull())
  {
    return "";
  }
  std::ostringstream aS;
  GeomTools::Write(theC, aS);
  return aS.str();
}

static void Throw(const Standard_Failure& e)
{
  gOut << "THROW " << e.ExceptionType() << "\n";
}

static occ::handle<ShapeCustom_RestrictionParameters> Parameters(int theSet)
{
  occ::handle<ShapeCustom_RestrictionParameters> aP = new ShapeCustom_RestrictionParameters;
  if (theSet == 1)
  {
    // every elementary surface and planes too
    aP->ConvertPlane()           = true;
    aP->ConvertCylindricalSurf() = true;
    aP->ConvertConicalSurf()     = true;
    aP->ConvertSphericalSurf()   = true;
    aP->ConvertToroidalSurf()    = true;
  }
  else
  {
    // only what exceeds the limits, Beziers, offsets kept as offsets, no segment mode
    aP->ConvertCurve3d()       = false;
    aP->ConvertCurve2d()       = false;
    aP->ConvertOffsetSurf()    = false;
    aP->ConvertOffsetCurv3d()  = false;
    aP->ConvertOffsetCurv2d()  = false;
    aP->SegmentSurfaceMode()   = false;
    aP->ConvertBezierSurf()    = true;
    aP->GMaxDegree()           = 9;
    aP->GMaxSeg()              = 50;
  }
  return aP;
}

static void BSR(const TopoDS_Shape& theShape, int theSet)
{
  gOut << "BSR " << theSet << "\n";
  try
  {
    OCC_CATCH_SIGNALS
    TopoDS_Shape aRes = theSet == 1
                          ? ShapeCustom::BSplineRestriction(theShape, 1e-3, 1e-3, 5, 100, GeomAbs_C1, GeomAbs_C1, true, true, Parameters(1))
                          : ShapeCustom::BSplineRestriction(theShape, 1e-2, 1e-4, 3, 4, GeomAbs_C2, GeomAbs_C1, false, false, Parameters(2));
    std::ostringstream aS;
    BRepTools::Write(aRes, aS, true, false, TopTools_FormatVersion_VERSION_3);
    Block("OUT", aS.str());
  }
  catch (const Standard_Failure& e)
  {
    Throw(e);
  }
}

static void Geometry(const TopoDS_Shape& theShape)
{
  TopTools_IndexedMapOfShape aFaces, anEdges;
  TopExp::MapShapes(theShape, TopAbs_FACE, aFaces);
  TopExp::MapShapes(theShape, TopAbs_EDGE, anEdges);

  int aNbSurf = 0;
  for (int i = 1; i <= aFaces.Extent() && aNbSurf < 30; i++)
  {
    TopLoc_Location           aL;
    occ::handle<Geom_Surface> aS = BRep_Tool::Surface(TopoDS::Face(aFaces(i)), aL);
    if (aS.IsNull() || !(aS->IsKind(STANDARD_TYPE(Geom_BSplineSurface)) || aS->IsKind(STANDARD_TYPE(Geom_BezierSurface))))
    {
      continue;
    }
    ++aNbSurf;
    gOut << "SURF " << i << "\n";
    for (double aTol : {1e-4, 1e-2})
    {
      try
      {
        OCC_CATCH_SIGNALS
        ShapeCustom_Surface       aTool(aS);
        occ::handle<Geom_Surface> aRes = aTool.ConvertToAnalytical(aTol, false);
        Block("ANA " + Hex(aTol) + " " + Hex(aTool.Gap()), Text(aRes));
      }
      catch (const Standard_Failure& e)
      {
        Throw(e);
      }
    }
    try
    {
      OCC_CATCH_SIGNALS
      ShapeCustom_Surface aTool(occ::down_cast<Geom_Surface>(aS->Copy()));
      Block("PER", Text(aTool.ConvertToPeriodic(false)));
    }
    catch (const Standard_Failure& e)
    {
      Throw(e);
    }
  }

  int aNbCurv = 0;
  for (int i = 1; i <= anEdges.Extent() && aNbCurv < 30; i++)
  {
    TopLoc_Location         aL;
    double                  f, l;
    occ::handle<Geom_Curve> aC = BRep_Tool::Curve(TopoDS::Edge(anEdges(i)), aL, f, l);
    if (aC.IsNull() || !aC->IsKind(STANDARD_TYPE(Geom_BSplineCurve)))
    {
      continue;
    }
    ++aNbCurv;
    gOut << "CURV " << i << "\n";
    try
    {
      OCC_CATCH_SIGNALS
      ShapeCustom_Curve aTool(occ::down_cast<Geom_Curve>(aC->Copy()));
      Block("PER", Text(aTool.ConvertToPeriodic(false)));
    }
    catch (const Standard_Failure& e)
    {
      Throw(e);
    }
  }

  int aNbPCurv = 0;
  for (int j = 1; j <= aFaces.Extent() && aNbPCurv < 40; j++)
  {
    const TopoDS_Face&         aF = TopoDS::Face(aFaces(j));
    TopTools_IndexedMapOfShape aFaceEdges;
    TopExp::MapShapes(aF, TopAbs_EDGE, aFaceEdges);
    for (int k = 1; k <= aFaceEdges.Extent() && aNbPCurv < 40; k++)
    {
      const TopoDS_Edge&        anE = TopoDS::Edge(aFaceEdges(k));
      double                    f, l;
      occ::handle<Geom2d_Curve> aC = BRep_Tool::CurveOnSurface(anE, aF, f, l);
      if (aC.IsNull() || !(aC->IsKind(STANDARD_TYPE(Geom2d_BSplineCurve)) || aC->IsKind(STANDARD_TYPE(Geom2d_BezierCurve))))
      {
        continue;
      }
      ++aNbPCurv;
      gOut << "PCURVE " << anEdges.FindIndex(static_cast<const TopoDS_Shape&>(anE)) << " " << j << "\n";
      try
      {
        OCC_CATCH_SIGNALS
        double                   cf = 0, cl = 0, aDev = 0;
        occ::handle<Geom2d_Line> aLine = ShapeCustom_Curve2d::ConvertToLine2d(aC, f, l, 1e-4, cf, cl, aDev);
        Block("LINE " + Hex(cf) + " " + Hex(cl) + " " + Hex(aDev), Text(occ::handle<Geom2d_Curve>(aLine)));
      }
      catch (const Standard_Failure& e)
      {
        Throw(e);
      }
      occ::handle<Geom2d_BSplineCurve> aBS = occ::down_cast<Geom2d_BSplineCurve>(aC);
      if (!aBS.IsNull())
      {
        try
        {
          OCC_CATCH_SIGNALS
          occ::handle<Geom2d_BSplineCurve> aCopy = occ::down_cast<Geom2d_BSplineCurve>(aBS->Copy());
          bool isDone = ShapeCustom_Curve2d::SimplifyBSpline2d(aCopy, Precision::Approximation());
          Block(std::string("SIMP ") + (isDone ? "1" : "0"), Text(occ::handle<Geom2d_Curve>(aCopy)));
        }
        catch (const Standard_Failure& e)
        {
          Throw(e);
        }
      }
    }
  }
}

int main(int argc, char** argv)
{
  if (argc < 3)
  {
    std::fprintf(stderr, "usage: shcustomrec inputs.txt out.txt\n");
    return 2;
  }
  OSD::SetSignal(false);
  std::ifstream aList(argv[1]);
  gOut.open(argv[2], std::ios::binary);
  std::string aPath;
  while (std::getline(aList, aPath))
  {
    if (!aPath.empty() && aPath.back() == '\r')
    {
      aPath.pop_back();
    }
    if (aPath.empty())
    {
      continue;
    }
    TopoDS_Shape aShape;
    BRep_Builder aB;
    try
    {
      OCC_CATCH_SIGNALS
      if (!BRepTools::Read(aShape, aPath.c_str(), aB) || aShape.IsNull())
      {
        continue;
      }
    }
    catch (const Standard_Failure&)
    {
      continue;
    }
    std::string aName = aPath.substr(aPath.find_last_of("/\\") + 1);
    gOut << "FILE " << aName << "\n";
    std::ifstream aFile(aPath, std::ios::binary | std::ios::ate);
    if ((long long)aFile.tellg() <= 30000)
    {
      BSR(aShape, 1);
      BSR(aShape, 2);
    }
    Geometry(aShape);
    gOut.flush();
  }
  return 0;
}
