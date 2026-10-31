// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
//
// piperec: records native OCCT's BRepOffsetAPI_MakePipe and BRepOffsetAPI_MiddlePath (unit 5S2), which build123d.NET
// doesn't bind, on inputs built from a small deterministic generator. Built against build123d.NET's native OCCT V8_0_1
// (tools/piperec/build.ps1).
//
// Pipes: spines (straight, polylines with corners, G1 chains of lines and tangent arcs, interpolated B-splines,
// closed circles and polygons); profiles (vertices, edges, open and closed wires, faces with and without holes,
// shells of two faces, compounds of a wire and a vertex) placed at the spine's start or away from it; the three
// trihedron modes (corrected Frenet, Frenet, discrete) with and without ForceApproxC1. Middle paths: of pipes of
// polygonal faces along polylines and arc chains (the solid, its first and last faces), and of the same given as
// wires.
//
// Each case writes its inputs as the BRep text of a compound and reads it back, so native OCCT runs on exactly the
// shapes the port reads from the recording (tests/OcctNet.ModelingAlgorithms.Tests/BRepOffsetAPI/
// BRepOffsetAPI_MakePipeNativeTests.cs). Per case:
//   CASE <kind> <seed>
//   OPT <mode> <ForceApproxC1>        (pipes)
//   IN <n>          followed by n lines: the input compound (spine, profile; or shape, start, end)
//   THROW <type>    if the maker threw, else
//   STATUS <IsDone> <ErrorOnSurface in hex>   (middle paths: STATUS <IsDone> -)
//   OUT <n>         the result
//   FIRST <n>, LAST <n>                       (pipes) the first and last shapes
//   GEN <list>      (pipes) for each sub-shape of the profile, then of the spine (TopExp::MapShapes order), the
//                   generated shapes as indices in the result's map of all sub-shapes, ',' between, ';' after each
//   GEN2 <list>     (pipes) Generated(spine edge, profile edge or vertex) for each pair, as indices, ';' after each
//
//   piperec > piperec.txt
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cmath>
#include <sstream>
#include <string>
#include <typeinfo>
#include <vector>

#include <BRepAdaptor_Curve.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRep_Tool.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepOffsetAPI_MakePipe.hxx>
#include <BRepOffsetAPI_MiddlePath.hxx>
#include <BRepTools.hxx>
#include <BRep_Builder.hxx>
#include <GC_MakeArcOfCircle.hxx>
#include <GeomAPI_Interpolate.hxx>
#include <Geom_TrimmedCurve.hxx>
#include <NCollection_HArray1.hxx>
#include <OSD.hxx>
#include <Standard_Failure.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Iterator.hxx>
#include <TopoDS_Shell.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopTools_ListOfShape.hxx>
#include <gp_Ax2.hxx>
#include <gp_Circ.hxx>

static std::string Hex(double theValue)
{
  uint64_t aBits;
  std::memcpy(&aBits, &theValue, 8);
  char aBuf[20];
  std::snprintf(aBuf, sizeof(aBuf), "%016llx", (unsigned long long)aBits);
  return aBuf;
}

//! The generator: a 64-bit LCG; U() has 32 significant bits. One draw per statement (MSVC evaluates arguments in no
//! fixed order).
struct Rng
{
  uint64_t s;

  explicit Rng(uint64_t theSeed)
      : s(theSeed * 2862933555777941757ULL + 3037000493ULL)
  {
  }

  uint64_t Next()
  {
    s = s * 6364136223846793005ULL + 1442695040888963407ULL;
    return s;
  }

  double U() { return (double)(Next() >> 32) / 4294967296.0; }

  double R(double a, double b) { return a + (b - a) * U(); }

  int I(int a, int b) { return a + (int)((Next() >> 33) % (uint64_t)(b - a + 1)); }
};

static std::string theOut;

static void Line(const std::string& theLine)
{
  theOut += theLine;
  theOut += "\n";
}

static void Flush()
{
  std::fputs(theOut.c_str(), stdout);
  theOut.clear();
}

static std::string Text(const TopoDS_Shape& theShape)
{
  std::ostringstream aStream;
  BRepTools::Write(theShape, aStream);
  return aStream.str();
}

static void Block(const char* theTag, const std::string& theText)
{
  int aCount = 0;
  for (char aChar : theText)
  {
    if (aChar == '\n')
    {
      aCount++;
    }
  }
  if (!theText.empty() && theText.back() != '\n')
  {
    aCount++;
  }
  Line(std::string(theTag) + " " + std::to_string(aCount));
  theOut += theText;
  if (!theText.empty() && theText.back() != '\n')
  {
    theOut += "\n";
  }
}

static TopoDS_Shape ReadBack(const TopoDS_Shape& theShape)
{
  std::istringstream aStream(Text(theShape));
  TopoDS_Shape       aRes;
  BRep_Builder       aB;
  BRepTools::Read(aRes, aStream, aB);
  return aRes;
}

static std::vector<TopoDS_Shape> Children(const TopoDS_Shape& theShape)
{
  std::vector<TopoDS_Shape> aRes;
  for (TopoDS_Iterator anIt(theShape); anIt.More(); anIt.Next())
  {
    aRes.push_back(anIt.Value());
  }
  return aRes;
}

static TopoDS_Shape Compound(const std::vector<TopoDS_Shape>& theShapes)
{
  BRep_Builder    aB;
  TopoDS_Compound aC;
  aB.MakeCompound(aC);
  for (const TopoDS_Shape& aS : theShapes)
  {
    aB.Add(aC, aS);
  }
  return aC;
}

// ------------------------------------------------------------------------------------------------ inputs

static TopoDS_Wire Polyline(const std::vector<gp_Pnt>& thePoints, bool theClosed)
{
  BRepBuilderAPI_MakePolygon aP;
  for (const gp_Pnt& aQ : thePoints)
  {
    aP.Add(aQ);
  }
  if (theClosed)
  {
    aP.Close();
  }
  return aP.Wire();
}

//! A spine of kind theKind (0 straight, 1 polyline, 2 G1 line and arcs, 3 B-spline, 4 circle, 5 closed polygon).
static TopoDS_Wire Spine(Rng& r, int theKind)
{
  switch (theKind)
  {
    case 0: {
      double aL = r.R(5, 15);
      return Polyline({gp_Pnt(0, 0, 0), gp_Pnt(0, 0, aL)}, false);
    }
    case 1: {
      int                 aNb = r.I(2, 4);
      std::vector<gp_Pnt> aPts{gp_Pnt(0, 0, 0)};
      gp_Vec              aDir(0, 0, 1);
      for (int i = 0; i < aNb; i++)
      {
        double aX = r.R(-0.7, 0.7);
        double aY = r.R(-0.7, 0.7);
        aDir      = (aDir + gp_Vec(aX, aY, 0)).Normalized();
        double aL = r.R(6, 12);
        aPts.push_back(aPts.back().Translated(aDir * aL));
      }
      return Polyline(aPts, false);
    }
    case 2: {
      double                  aL = r.R(5, 10);
      gp_Pnt                  aP1(0, 0, aL);
      BRepBuilderAPI_MakeWire aMW(BRepBuilderAPI_MakeEdge(gp_Pnt(0, 0, 0), aP1).Edge());
      gp_Vec                  aT(0, 0, 1);
      int                     aNb = r.I(1, 2);
      for (int i = 0; i < aNb; i++)
      {
        double aF  = r.R(4, 8);
        double aX  = r.R(-6, 6);
        double aY  = r.R(-6, 6);
        gp_Pnt aP2 = aP1.Translated(aT * aF + gp_Vec(aX, aY, 0));
        occ::handle<Geom_TrimmedCurve> anArc = GC_MakeArcOfCircle(aP1, aT, aP2).Value();
        aMW.Add(BRepBuilderAPI_MakeEdge(anArc).Edge());
        gp_Pnt aQ;
        anArc->D1(anArc->LastParameter(), aQ, aT);
        aT.Normalize();
        aP1 = aP2;
      }
      return aMW.Wire();
    }
    case 3: {
      int                                     aNb = r.I(3, 5);
      occ::handle<NCollection_HArray1<gp_Pnt>> aPts = new NCollection_HArray1<gp_Pnt>(1, aNb);
      for (int i = 1; i <= aNb; i++)
      {
        double aX = r.R(-3, 3);
        double aY = r.R(-3, 3);
        aPts->SetValue(i, gp_Pnt(aX, aY, 6.0 * (i - 1)));
      }
      GeomAPI_Interpolate anInt(aPts, false, 1e-7);
      anInt.Perform();
      return BRepBuilderAPI_MakeWire(BRepBuilderAPI_MakeEdge(anInt.Curve()).Edge()).Wire();
    }
    case 4: {
      double aR = r.R(8, 15);
      gp_Circ aC(gp_Ax2(gp_Pnt(0, 0, 0), gp_Dir(0, -1, 0), gp_Dir(1, 0, 0)), aR);
      return BRepBuilderAPI_MakeWire(BRepBuilderAPI_MakeEdge(aC).Edge()).Wire();
    }
    default: {
      double aS = r.R(8, 15);
      return Polyline({gp_Pnt(0, 0, 0), gp_Pnt(0, 0, aS), gp_Pnt(aS, 0, aS), gp_Pnt(aS, 0, 0)}, true);
    }
  }
}

static gp_Pnt OnPlane(const gp_Ax2& theAx, double theU, double theV)
{
  return theAx.Location().Translated(gp_Vec(theAx.XDirection()) * theU + gp_Vec(theAx.YDirection()) * theV);
}

static TopoDS_Wire Rectangle(const gp_Ax2& theAx, double theW, double theH)
{
  return Polyline({OnPlane(theAx, -theW, -theH), OnPlane(theAx, theW, -theH), OnPlane(theAx, theW, theH), OnPlane(theAx, -theW, theH)}, true);
}

//! A profile of kind theKind in the plane theAx (0 vertex, 1 edge, 2 open wire, 3 closed wire, 4 circle, 5 face,
//! 6 holed face, 7 shell of two faces, 8 compound of a wire and a vertex).
static TopoDS_Shape Profile(Rng& r, const gp_Ax2& theAx, int theKind)
{
  double aS = r.R(0.5, 1.5);
  switch (theKind)
  {
    case 0:
      return BRepBuilderAPI_MakeVertex(OnPlane(theAx, aS, 0)).Vertex();
    case 1:
      return BRepBuilderAPI_MakeEdge(OnPlane(theAx, -aS, 0), OnPlane(theAx, aS, 0)).Edge();
    case 2: {
      double aH = r.R(0.2, 1.0);
      return Polyline({OnPlane(theAx, -aS, 0), OnPlane(theAx, 0, aH), OnPlane(theAx, aS, 0)}, false);
    }
    case 3: {
      double aH = r.R(0.5, 1.5);
      return Rectangle(theAx, aS, aH);
    }
    case 4:
      return BRepBuilderAPI_MakeWire(BRepBuilderAPI_MakeEdge(gp_Circ(theAx, aS)).Edge()).Wire();
    case 5: {
      double aH = r.R(0.5, 1.5);
      return BRepBuilderAPI_MakeFace(Rectangle(theAx, aS, aH), true).Face();
    }
    case 6: {
      BRepBuilderAPI_MakeFace aMF(Rectangle(theAx, aS * 2, aS * 2), true);
      TopoDS_Wire             aHole = BRepBuilderAPI_MakeWire(BRepBuilderAPI_MakeEdge(gp_Circ(theAx, aS)).Edge()).Wire();
      aHole.Reverse();
      aMF.Add(aHole);
      return aMF.Face();
    }
    case 7: {
      // two faces sharing an edge
      gp_Pnt          a = OnPlane(theAx, -aS, -aS), b = OnPlane(theAx, 0, -aS), c = OnPlane(theAx, aS, -aS);
      gp_Pnt          d = OnPlane(theAx, aS, aS), e = OnPlane(theAx, 0, aS), f = OnPlane(theAx, -aS, aS);
      TopoDS_Edge     ab = BRepBuilderAPI_MakeEdge(a, b), bc = BRepBuilderAPI_MakeEdge(b, c), cd = BRepBuilderAPI_MakeEdge(c, d);
      TopoDS_Edge     de = BRepBuilderAPI_MakeEdge(d, e), ef = BRepBuilderAPI_MakeEdge(e, f), fa = BRepBuilderAPI_MakeEdge(f, a);
      TopoDS_Edge     be = BRepBuilderAPI_MakeEdge(TopExp::LastVertex(ab), TopExp::FirstVertex(ef));
      TopoDS_Wire     w1 = BRepBuilderAPI_MakeWire(ab, be, ef, fa);
      TopoDS_Wire     w2 = BRepBuilderAPI_MakeWire(bc, cd, de, TopoDS::Edge(be.Reversed()));
      BRep_Builder    aB;
      TopoDS_Shell    aSh;
      aB.MakeShell(aSh);
      aB.Add(aSh, BRepBuilderAPI_MakeFace(w1, true).Face());
      aB.Add(aSh, BRepBuilderAPI_MakeFace(w2, true).Face());
      return aSh;
    }
    default: {
      double aH = r.R(0.5, 1.5);
      return Compound({Rectangle(theAx, aS, aH), BRepBuilderAPI_MakeVertex(OnPlane(theAx, 3 * aS, 0)).Vertex()});
    }
  }
}

// ------------------------------------------------------------------------------------------------ cases

static std::string Indices(const TopTools_ListOfShape& theList, const TopTools_IndexedMapOfShape& theMap)
{
  std::string aRes;
  bool        isFirst = true;
  for (TopTools_ListOfShape::Iterator anIt(theList); anIt.More(); anIt.Next())
  {
    if (!isFirst)
    {
      aRes += ",";
    }
    isFirst = false;
    aRes += std::to_string(theMap.FindIndex(anIt.Value()));
  }
  return aRes;
}

static void PipeCase(int theKind, int theSeed)
{
  Rng    r(theKind * 1000 + theSeed);
  int    aSpineKind = theKind % 6;
  TopoDS_Wire aSpine0 = Spine(r, aSpineKind);
  // the frame at the spine's start
  TopoDS_Vertex aV1, aV2;
  TopExp::Vertices(aSpine0, aV1, aV2);
  gp_Pnt aStart = BRep_Tool::Pnt(aV1);
  gp_Vec aTangent;
  {
    TopExp_Explorer anExp(aSpine0, TopAbs_EDGE);
    BRepAdaptor_Curve aC(TopoDS::Edge(anExp.Current()));
    gp_Pnt aQ;
    aC.D1(anExp.Current().Orientation() == TopAbs_FORWARD ? aC.FirstParameter() : aC.LastParameter(), aQ, aTangent);
    if (anExp.Current().Orientation() == TopAbs_REVERSED)
    {
      aTangent.Reverse();
    }
  }
  int  anAway = r.I(0, 4);
  double aDX    = r.R(-1, 1);
  double aDY    = r.R(-1, 1);
  gp_Pnt anO    = anAway == 0 ? aStart.Translated(gp_Vec(aDX, aDY, 0)) : aStart;
  gp_Ax2 anAx(anO, gp_Dir(aTangent));
  int    aProfKind = r.I(0, 8);
  TopoDS_Shape aProfile0 = Profile(r, anAx, aProfKind);
  int    aMode   = r.I(0, 2);
  int    aForce  = r.I(0, 3) == 0 ? 1 : 0;

  Line("CASE pipe" + std::to_string(theKind) + " " + std::to_string(theSeed));
  Line("OPT " + std::to_string(aMode) + " " + std::to_string(aForce));
  TopoDS_Shape anIn = Compound({aSpine0, aProfile0});
  Block("IN", Text(anIn));
  std::vector<TopoDS_Shape> aParts = Children(ReadBack(anIn));
  TopoDS_Wire  aSpine   = TopoDS::Wire(aParts[0]);
  TopoDS_Shape aProfile = aParts[1];
  try
  {
    GeomFill_Trihedron aTrihedron = aMode == 0 ? GeomFill_IsCorrectedFrenet
                                    : aMode == 1 ? GeomFill_IsFrenet
                                                 : GeomFill_IsDiscreteTrihedron;
    BRepOffsetAPI_MakePipe aPipe(aSpine, aProfile, aTrihedron, aForce != 0);
    Line(std::string("STATUS ") + (aPipe.IsDone() ? "1 " : "0 ") + Hex(aPipe.ErrorOnSurface()));
    if (!aPipe.IsDone())
    {
      return;
    }
    TopoDS_Shape aRes = aPipe.Shape();
    Block("OUT", Text(aRes));
    Block("FIRST", Text(aPipe.FirstShape()));
    Block("LAST", Text(aPipe.LastShape()));
    TopTools_IndexedMapOfShape aResMap;
    TopExp::MapShapes(aRes, aResMap);
    std::string aGen = "GEN ";
    for (const TopoDS_Shape& anIn2 : {aProfile, TopoDS_Shape(aSpine)})
    {
      TopTools_IndexedMapOfShape aMap;
      TopExp::MapShapes(anIn2, aMap);
      for (int i = 1; i <= aMap.Extent(); i++)
      {
        aGen += Indices(aPipe.Generated(aMap(i)), aResMap) + ";";
      }
    }
    Line(aGen);
    std::string aGen2 = "GEN2 ";
    TopTools_IndexedMapOfShape aSpineEdges, aProfShapes;
    TopExp::MapShapes(aSpine, TopAbs_EDGE, aSpineEdges);
    TopExp::MapShapes(aProfile, TopAbs_EDGE, aProfShapes);
    TopExp::MapShapes(aProfile, TopAbs_VERTEX, aProfShapes);
    for (int i = 1; i <= aSpineEdges.Extent(); i++)
    {
      for (int j = 1; j <= aProfShapes.Extent(); j++)
      {
        TopoDS_Shape aG = aPipe.Generated(aSpineEdges(i), aProfShapes(j));
        aGen2 += std::to_string(aG.IsNull() ? -1 : aResMap.FindIndex(aG)) + ";";
      }
    }
    Line(aGen2);
  }
  catch (const Standard_Failure& e)
  {
    Line(std::string("THROW ") + e.ExceptionType());
  }
  catch (...)
  {
    Line("THROW fault");
  }
}

static void MiddleCase(int theKind, int theSeed)
{
  Rng r(50000 + theKind * 1000 + theSeed);
  // a pipe of a polygonal face along a polyline (kind 0) or a line and arcs (kind 1); as a solid with its end faces,
  // or (theSeed odd) as a shell with its end wires
  TopoDS_Wire aSpine = Spine(r, theKind == 0 ? 1 : 2);
  int         aNb    = r.I(3, 5);
  gp_Ax2      anAx(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1));
  std::vector<gp_Pnt> aPts;
  for (int i = 0; i < aNb; i++)
  {
    double aR = r.R(0.8, 1.6);
    aPts.push_back(OnPlane(anAx, aR * std::cos(2 * M_PI * i / aNb), aR * std::sin(2 * M_PI * i / aNb)));
  }
  TopoDS_Wire  aW = Polyline(aPts, true);
  bool         isShell = (theSeed % 2) == 1;
  TopoDS_Shape aProf = isShell ? TopoDS_Shape(aW) : TopoDS_Shape(BRepBuilderAPI_MakeFace(aW, true).Face());
  Line("CASE middle" + std::to_string(theKind) + " " + std::to_string(theSeed));
  try
  {
    BRepOffsetAPI_MakePipe aPipe(aSpine, aProf);
    TopoDS_Shape anIn = Compound({aPipe.Shape(), aPipe.FirstShape(), aPipe.LastShape()});
    Block("IN", Text(anIn));
    std::vector<TopoDS_Shape> aParts = Children(ReadBack(anIn));
    BRepOffsetAPI_MiddlePath  aMP(aParts[0], aParts[1], aParts[2]);
    aMP.Build();
    Line(std::string("STATUS ") + (aMP.IsDone() ? "1" : "0") + " -");
    if (aMP.IsDone())
    {
      Block("OUT", Text(aMP.Shape()));
    }
  }
  catch (const Standard_Failure& e)
  {
    Line(std::string("THROW ") + e.ExceptionType());
  }
  catch (...)
  {
    Line("THROW fault");
  }
}

int main()
{
  // As DRAWEXE and build123d.NET: access violations arrive as OSD_Exception_ACCESS_VIOLATION (built with /EHa)
  OSD::SetSignal(false);
  for (int k = 0; k < 6; k++)
  {
    for (int s = 1; s <= 40; s++)
    {
      PipeCase(k, s);
      Flush();
    }
  }
  for (int k = 0; k < 2; k++)
  {
    for (int s = 1; s <= 15; s++)
    {
      MiddleCase(k, s);
      Flush();
    }
  }
  return 0;
}
