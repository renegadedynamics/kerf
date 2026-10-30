// Records native BRepAlgo_FaceRestrictor and TopOpeBRepBuild_WireToFace results (unit 5O1): for each case, a
// planar face and closed wires (lines, circles, arcs, B-splines; nested, disjoint, crossing, wrongly oriented) are
// built, written as BRep text (a compound: the face, then the wires) and read back, and the faces made from the read
// shapes are written as BRep text, one by one. Built against build123d.NET's native OCCT V8_0_1 (TKBool's own
// classes, not a copy). The port reads the same input text and compares its faces' text with the recorded one.
//   frrec > FaceRestrictorRecordings.txt
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <sstream>
#include <string>
#include <typeinfo>
#include <vector>

#include <BRepAlgo.hxx>
#include <BRepAlgo_FaceRestrictor.hxx>
#include <BRepAlgo_Loop.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepTools.hxx>
#include <BRep_Builder.hxx>
#include <BRep_Tool.hxx>
#include <Geom_Curve.hxx>
#include <ElSLib.hxx>
#include <GC_MakeArcOfCircle.hxx>
#include <GeomAPI_Interpolate.hxx>
#include <Geom2d_Line.hxx>
#include <Geom_BSplineCurve.hxx>
#include <Geom_Plane.hxx>
#include <Geom_TrimmedCurve.hxx>
#include <NCollection_HArray1.hxx>
#include <NCollection_List.hxx>
#include <Standard_Failure.hxx>
#include <TopOpeBRepBuild_WireToFace.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Iterator.hxx>
#include <TopoDS_Wire.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <gp_Ax3.hxx>
#include <gp_Circ.hxx>
#include <gp_Pln.hxx>
#include <gp_Pnt2d.hxx>

// ---------------------------------------------------------------------------------------------------------------
// Deterministic pseudo-random numbers (64-bit LCG, the top 53 bits)

static uint64_t theSeed = 0x5e01f00dULL;

static double Rand()
{
  theSeed = theSeed * 6364136223846793005ULL + 1442695040888963407ULL;
  return (double)(theSeed >> 11) / 9007199254740992.0;
}

static double Rand(double theMin, double theMax)
{
  return theMin + (theMax - theMin) * Rand();
}

// ---------------------------------------------------------------------------------------------------------------
// Wires on a plane, from 2D descriptions

static gp_Pnt P3(const gp_Pln& thePln, double theU, double theV)
{
  return ElSLib::Value(theU, theV, thePln);
}

// A polygon through the 2D points, in their order (counter-clockwise in the plane's parameters when they turn so)
static TopoDS_Wire Polygon(const gp_Pln& thePln, const std::vector<gp_Pnt2d>& thePts)
{
  BRepBuilderAPI_MakeWire aMW;
  const size_t            n = thePts.size();
  for (size_t i = 0; i < n; ++i)
  {
    const gp_Pnt2d& a = thePts[i];
    const gp_Pnt2d& b = thePts[(i + 1) % n];
    aMW.Add(BRepBuilderAPI_MakeEdge(P3(thePln, a.X(), a.Y()), P3(thePln, b.X(), b.Y())).Edge());
  }
  return aMW.Wire();
}

static TopoDS_Wire Rect(const gp_Pln& thePln, double x0, double y0, double x1, double y1, bool theCCW)
{
  std::vector<gp_Pnt2d> aPts = {gp_Pnt2d(x0, y0), gp_Pnt2d(x1, y0), gp_Pnt2d(x1, y1), gp_Pnt2d(x0, y1)};
  if (!theCCW)
  {
    std::vector<gp_Pnt2d> aRev(aPts.rbegin(), aPts.rend());
    aPts = aRev;
  }
  return Polygon(thePln, aPts);
}

static TopoDS_Wire Circle(const gp_Pln& thePln, double cx, double cy, double r, bool theCCW)
{
  gp_Ax2 anAx(P3(thePln, cx, cy), thePln.Axis().Direction(), thePln.XAxis().Direction());
  if (!theCCW)
  {
    anAx = gp_Ax2(P3(thePln, cx, cy), thePln.Axis().Direction().Reversed(), thePln.XAxis().Direction());
  }
  gp_Circ aC(anAx, r);
  return BRepBuilderAPI_MakeWire(BRepBuilderAPI_MakeEdge(aC).Edge()).Wire();
}

// A rectangle with rounded corners (lines and arcs)
static TopoDS_Wire RoundRect(const gp_Pln& thePln, double x0, double y0, double x1, double y1, double r, bool theCCW)
{
  const double k = r * (1.0 - std::sqrt(0.5));
  // counter-clockwise: bottom, BR arc, right, TR arc, top, TL arc, left, BL arc
  std::vector<gp_Pnt> aP = {P3(thePln, x0 + r, y0), P3(thePln, x1 - r, y0), P3(thePln, x1 - k, y0 + k),
                            P3(thePln, x1, y0 + r), P3(thePln, x1, y1 - r), P3(thePln, x1 - k, y1 - k),
                            P3(thePln, x1 - r, y1), P3(thePln, x0 + r, y1), P3(thePln, x0 + k, y1 - k),
                            P3(thePln, x0, y1 - r), P3(thePln, x0, y0 + r), P3(thePln, x0 + k, y0 + k)};
  if (!theCCW)
  {
    std::vector<gp_Pnt> aRev(aP.rbegin(), aP.rend());
    // keep the triples (line end, arc middle, arc end) in order: reverse then rotate so a line comes first
    aP.clear();
    for (size_t i = 0; i < aRev.size(); ++i)
    {
      aP.push_back(aRev[(i + 11) % aRev.size()]);
    }
  }
  BRepBuilderAPI_MakeWire aMW;
  for (size_t i = 0; i < 12; i += 3)
  {
    const gp_Pnt& a = aP[i];
    const gp_Pnt& b = aP[i + 1];
    const gp_Pnt& m = aP[i + 2];
    const gp_Pnt& c = aP[(i + 3) % 12];
    aMW.Add(BRepBuilderAPI_MakeEdge(a, b).Edge());
    aMW.Add(BRepBuilderAPI_MakeEdge(GC_MakeArcOfCircle(b, m, c).Value()).Edge());
  }
  return aMW.Wire();
}

// A closed wire of a line and a B-spline through points of a circle's upper half
static TopoDS_Wire Drop(const gp_Pln& thePln, double cx, double cy, double r, bool theCCW)
{
  occ::handle<NCollection_HArray1<gp_Pnt>> aPts = new NCollection_HArray1<gp_Pnt>(1, 7);
  for (int i = 0; i < 7; ++i)
  {
    double a = M_PI * i / 6.0;
    double rr = r * (1.0 + 0.2 * std::sin(3.0 * a));
    aPts->SetValue(i + 1, P3(thePln, cx + rr * std::cos(a), cy + rr * std::sin(a)));
  }
  GeomAPI_Interpolate anInt(aPts, false, 1.0e-7);
  anInt.Perform();
  occ::handle<Geom_BSplineCurve> aBS = anInt.Curve();
  TopoDS_Edge                    anArc  = BRepBuilderAPI_MakeEdge(aBS).Edge();
  TopoDS_Edge                    aLine  = BRepBuilderAPI_MakeEdge(aPts->Value(7), aPts->Value(1)).Edge();
  BRepBuilderAPI_MakeWire        aMW(anArc, aLine);
  TopoDS_Wire                    aW = aMW.Wire();
  if (!theCCW)
  {
    aW.Reverse();
  }
  return aW;
}

static gp_Pln Plane(int theKind)
{
  switch (theKind)
  {
    case 0:
      return gp_Pln(gp_Ax3(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1), gp_Dir(1, 0, 0)));
    case 1: {
      gp_Ax3 anAx(gp_Pnt(1.0, 2.0, 3.0), gp_Dir(0, 0, 1), gp_Dir(1, 0, 0));
      anAx.Rotate(gp_Ax1(gp_Pnt(1.0, 2.0, 3.0), gp_Dir(1, 1, 0)), 0.4);
      return gp_Pln(anAx);
    }
    default: {
      // left-handed: the normal is reversed with respect to X^Y
      gp_Ax3 anAx(gp_Pnt(-2.0, 0.5, 1.0), gp_Dir(0, 1, 0), gp_Dir(1, 0, 0));
      anAx.YReverse();
      return gp_Pln(anAx);
    }
  }
}

// ---------------------------------------------------------------------------------------------------------------
// The scenarios: the wires on a plane

static std::vector<TopoDS_Wire> Scenario(int theId, const gp_Pln& p)
{
  std::vector<TopoDS_Wire> w;
  switch (theId)
  {
    case 0: // outer only
      w.push_back(Rect(p, 0, 0, 10, 6, true));
      break;
    case 1: // outer and a hole
      w.push_back(Rect(p, 0, 0, 10, 6, true));
      w.push_back(Circle(p, 5, 3, 2, false));
      break;
    case 2: // outer and three holes of different kinds
      w.push_back(RoundRect(p, 0, 0, 20, 10, 2, true));
      w.push_back(Circle(p, 4, 5, 2, false));
      w.push_back(Rect(p, 8, 2, 12, 8, false));
      w.push_back(Drop(p, 16, 4, 2, false));
      break;
    case 3: // two disjoint outers
      w.push_back(Rect(p, 0, 0, 4, 4, true));
      w.push_back(Circle(p, 10, 2, 2, true));
      break;
    case 4: // outer, hole, island in the hole
      w.push_back(Rect(p, 0, 0, 20, 20, true));
      w.push_back(Rect(p, 4, 4, 16, 16, false));
      w.push_back(Circle(p, 10, 10, 3, true));
      break;
    case 5: // two outers, each with a hole
      w.push_back(Circle(p, 0, 0, 5, true));
      w.push_back(Circle(p, 0, 0, 2, false));
      w.push_back(RoundRect(p, 8, -4, 16, 4, 1, true));
      w.push_back(Drop(p, 12, -1, 1.5, false));
      break;
    case 6: // crossing outers
      w.push_back(Rect(p, 0, 0, 10, 10, true));
      w.push_back(Rect(p, 5, 5, 15, 15, true));
      break;
    case 7: // a hole crossing the outer boundary
      w.push_back(Rect(p, 0, 0, 10, 10, true));
      w.push_back(Circle(p, 10, 5, 3, false));
      break;
    case 8: // every wire counter-clockwise (the hole wrongly oriented)
      w.push_back(Rect(p, 0, 0, 10, 6, true));
      w.push_back(Circle(p, 5, 3, 2, true));
      break;
    case 9: // concentric circles, four levels
      w.push_back(Circle(p, 0, 0, 8, true));
      w.push_back(Circle(p, 0, 0, 6, false));
      w.push_back(Circle(p, 0, 0, 4, true));
      w.push_back(Circle(p, 0, 0, 2, false));
      break;
    case 10: // nested rounded rectangles, given inner first
      w.push_back(RoundRect(p, 3, 3, 7, 7, 1, false));
      w.push_back(RoundRect(p, 0, 0, 10, 10, 2, true));
      break;
    case 11: // every wire clockwise
      w.push_back(Rect(p, 0, 0, 10, 6, false));
      w.push_back(Circle(p, 5, 3, 2, false));
      break;
    case 12: // a hole touching the outer boundary at a point
      w.push_back(Rect(p, 0, 0, 10, 10, true));
      w.push_back(Polygon(p, {gp_Pnt2d(5, 0.0), gp_Pnt2d(3, 4), gp_Pnt2d(7, 4)}));
      break;
    case 13: { // an outer and a hole whose wires aren't flagged closed (native flags them: a write)
      BRep_Builder aB;
      for (int k = 0; k < 2; ++k)
      {
        TopoDS_Wire aSrc = (k == 0) ? Rect(p, 0, 0, 10, 6, true) : Rect(p, 2, 2, 5, 4, false);
        TopoDS_Wire aW;
        aB.MakeWire(aW);
        for (TopExp_Explorer anExp(aSrc, TopAbs_EDGE); anExp.More(); anExp.Next())
        {
          aB.Add(aW, anExp.Current());
        }
        w.push_back(aW);
      }
      break;
    }
    case 14: { // edges with pcurves on another plane (the same geometry) and no 3D curve: ChangePCurve writes
      occ::handle<Geom_Plane> anOther = new Geom_Plane(p);
      const gp_Pnt2d          aPts[4] = {gp_Pnt2d(0, 0), gp_Pnt2d(8, 0), gp_Pnt2d(8, 5), gp_Pnt2d(0, 5)};
      TopoDS_Vertex           aV[4];
      for (int i = 0; i < 4; ++i)
      {
        aV[i] = BRepBuilderAPI_MakeVertex(P3(p, aPts[i].X(), aPts[i].Y())).Vertex();
      }
      BRepBuilderAPI_MakeWire aMW;
      for (int i = 0; i < 4; ++i)
      {
        const gp_Pnt2d& a = aPts[i];
        const gp_Pnt2d& b = aPts[(i + 1) % 4];
        occ::handle<Geom2d_Line> aL = new Geom2d_Line(a, gp_Dir2d(b.X() - a.X(), b.Y() - a.Y()));
        aMW.Add(BRepBuilderAPI_MakeEdge(aL, anOther, aV[i], aV[(i + 1) % 4], 0.0, a.Distance(b)).Edge());
      }
      w.push_back(aMW.Wire());
      break;
    }
    default:
      break;
  }
  return w;
}

static std::string Text(const TopoDS_Shape& theShape)
{
  std::ostringstream aStream;
  BRepTools::Write(theShape, aStream);
  return aStream.str();
}

static void Block(const char* theTag, const std::string& theText)
{
  std::printf("%s %zu\n", theTag, theText.size());
  std::fwrite(theText.data(), 1, theText.size(), stdout);
  std::printf("\n");
}

// The face of a case: 0, an infinite face on the plane; 1, the face of the first wire; 2, the latter reversed
static TopoDS_Face CaseFace(int theKind, const gp_Pln& thePln, const std::vector<TopoDS_Wire>& theWires)
{
  if (theKind == 0)
  {
    return BRepBuilderAPI_MakeFace(thePln).Face();
  }
  TopoDS_Face aF = BRepBuilderAPI_MakeFace(thePln, theWires.front(), true).Face();
  if (theKind == 2)
  {
    aF.Reverse();
  }
  return aF;
}

// Mode: 0, FaceRestrictor; 1, FaceRestrictor with projection; 2, with orientation correction; 3, WireToFace
static void Run(int theCase, const char* theKind, int theMode, const TopoDS_Face& theFace, const std::vector<TopoDS_Wire>& theWires)
{
  TopoDS_Compound aC;
  BRep_Builder    aB;
  aB.MakeCompound(aC);
  aB.Add(aC, theFace);
  for (const TopoDS_Wire& aW : theWires)
  {
    aB.Add(aC, aW);
  }
  const std::string anInput = Text(aC);
  std::printf("case %d %s mode %d\n", theCase, theKind, theMode);
  Block("input", anInput);

  // the shapes read back from the text
  TopoDS_Shape       aRead;
  BRep_Builder       aRB;
  std::istringstream anIn(anInput);
  BRepTools::Read(aRead, anIn, aRB);
  TopoDS_Iterator aIt(aRead);
  TopoDS_Face     aFace = TopoDS::Face(aIt.Value());
  NCollection_List<TopoDS_Shape> aWires;
  for (aIt.Next(); aIt.More(); aIt.Next())
  {
    aWires.Append(aIt.Value());
  }

  try
  {
    NCollection_List<TopoDS_Shape> aFaces;
    bool                           isDone = true;
    if (theMode == 3)
    {
      TopOpeBRepBuild_WireToFace aWTF;
      for (NCollection_List<TopoDS_Shape>::Iterator it(aWires); it.More(); it.Next())
      {
        aWTF.AddWire(TopoDS::Wire(it.Value()));
      }
      aWTF.MakeFaces(aFace, aFaces);
    }
    else
    {
      BRepAlgo_FaceRestrictor aFR;
      aFR.Init(aFace, theMode == 1, theMode == 2);
      for (NCollection_List<TopoDS_Shape>::Iterator it(aWires); it.More(); it.Next())
      {
        TopoDS_Wire aW = TopoDS::Wire(it.Value());
        aFR.Add(aW);
      }
      aFR.Perform();
      isDone = aFR.IsDone();
      for (; aFR.More(); aFR.Next())
      {
        aFaces.Append(aFR.Current());
      }
    }
    std::printf("done %d faces %d\n", isDone ? 1 : 0, aFaces.Extent());
    for (NCollection_List<TopoDS_Shape>::Iterator it(aFaces); it.More(); it.Next())
    {
      Block("face", Text(it.Value()));
    }
  }
  catch (const Standard_Failure& anE)
  {
    std::string aName = typeid(anE).name();
    if (aName.rfind("class ", 0) == 0)
    {
      aName = aName.substr(6);
    }
    std::printf("exception %s\n", aName.c_str());
  }
  std::printf("end\n");
}

// ---------------------------------------------------------------------------------------------------------------
// The other BRepAlgo tools: each case's input is a compound, its outputs are shapes (or values)

static void Result(const char* theTag, const TopoDS_Shape& theShape)
{
  if (theShape.IsNull())
  {
    std::printf("%s null\n", theTag);
  }
  else
  {
    Block(theTag, Text(theShape));
  }
}

static TopoDS_Compound Holder(const std::vector<TopoDS_Shape>& theShapes)
{
  TopoDS_Compound aC;
  BRep_Builder    aB;
  aB.MakeCompound(aC);
  for (const TopoDS_Shape& aS : theShapes)
  {
    aB.Add(aC, aS);
  }
  return aC;
}

// Writes the input, reads it back and returns the read shapes
static std::vector<TopoDS_Shape> AlgoInput(int theId, const char* theName, const std::vector<TopoDS_Shape>& theShapes)
{
  const std::string anInput = Text(Holder(theShapes));
  std::printf("algo %d %s\n", theId, theName);
  Block("input", anInput);
  TopoDS_Shape       aRead;
  BRep_Builder       aRB;
  std::istringstream anIn(anInput);
  BRepTools::Read(aRead, anIn, aRB);
  std::vector<TopoDS_Shape> aShapes;
  for (TopoDS_Iterator aIt(aRead); aIt.More(); aIt.Next())
  {
    aShapes.push_back(aIt.Value());
  }
  return aShapes;
}

static void Algos()
{
  int anId = 0;
  for (int pk = 0; pk < 3; ++pk)
  {
    const gp_Pln aPln = Plane(pk);
    // ConvertWire and ConvertFace on wires with curves
    std::vector<TopoDS_Wire> aWires = {RoundRect(aPln, 0, 0, 10, 6, 1.5, true), Drop(aPln, 3, 2, 2, true),
                                       Circle(aPln, 0, 0, 3, false), Rect(aPln, 0, 0, 4, 3, true)};
    for (const TopoDS_Wire& aW : aWires)
    {
      TopoDS_Face               aF  = BRepBuilderAPI_MakeFace(aPln, aW, true).Face();
      std::vector<TopoDS_Shape> aIn = AlgoInput(anId++, "convertwire", {aF, aW});
      try
      {
        Result("result", BRepAlgo::ConvertWire(TopoDS::Wire(aIn[1]), 0.01, TopoDS::Face(aIn[0])));
        Result("face", BRepAlgo::ConvertFace(TopoDS::Face(aIn[0]), 0.01));
      }
      catch (const Standard_Failure&)
      {
        std::printf("exception\n");
      }
      std::printf("end\n");
    }
    // ConcatenateWire (C1, G1) and ConcatenateWireC0
    for (const TopoDS_Wire& aW : aWires)
    {
      std::vector<TopoDS_Shape> aIn = AlgoInput(anId++, "concatenate", {aW});
      const TopoDS_Wire         aRW = TopoDS::Wire(aIn[0]);
      try
      {
        Result("c1", BRepAlgo::ConcatenateWire(aRW, GeomAbs_C1));
        Result("g1", BRepAlgo::ConcatenateWire(aRW, GeomAbs_G1));
        Result("c0", BRepAlgo::ConcatenateWireC0(aRW));
      }
      catch (const Standard_Failure&)
      {
        std::printf("exception\n");
      }
      std::printf("valid %d %d\n", BRepAlgo::IsValid(aRW) ? 1 : 0, BRepAlgo::IsTopologicallyValid(aRW) ? 1 : 0);
      std::printf("end\n");
    }
    // BRepAlgo_Loop: the edges of a rectangle face, the bottom one cut at a vertex in its middle, and a hole's
    TopoDS_Wire aRect = Rect(aPln, 0, 0, 10, 6, true);
    TopoDS_Wire aHole = Circle(aPln, 5, 3, 1, false);
    TopoDS_Face aFace = BRepBuilderAPI_MakeFace(aPln, aRect, true).Face();
    TopoDS_Edge aBottom;
    for (TopExp_Explorer anExp(aRect, TopAbs_EDGE); anExp.More(); anExp.Next())
    {
      aBottom = TopoDS::Edge(anExp.Current());
      break;
    }
    double             f, l;
    occ::handle<Geom_Curve> aC = BRep_Tool::Curve(aBottom, f, l);
    TopoDS_Vertex      aMid = BRepBuilderAPI_MakeVertex(aC->Value(0.5 * (f + l))).Vertex();
    BRep_Builder().UpdateVertex(aMid, 0.5 * (f + l), aBottom, 1.0e-7);
    std::vector<TopoDS_Shape> aLoopShapes = {aFace, aMid, aHole};
    for (TopExp_Explorer anExp(aRect, TopAbs_EDGE); anExp.More(); anExp.Next())
    {
      aLoopShapes.push_back(anExp.Current());
    }
    for (int withHole = 0; withHole < 2; ++withHole)
    {
      std::vector<TopoDS_Shape> aIn = AlgoInput(anId++, withHole ? "loop-hole" : "loop", aLoopShapes);
      try
      {
        BRepAlgo_Loop aLoop;
        aLoop.Init(TopoDS::Face(aIn[0]));
        TopoDS_Edge                    aCut = TopoDS::Edge(aIn[3]);
        NCollection_List<TopoDS_Shape> aLV;
        aLV.Append(aIn[1].Oriented(TopAbs_REVERSED));
        aLV.Append(aIn[1].Oriented(TopAbs_FORWARD));
        aLoop.AddEdge(aCut, aLV);
        for (size_t i = 4; i < aIn.size(); ++i)
        {
          aLoop.AddConstEdge(TopoDS::Edge(aIn[i]));
        }
        if (withHole)
        {
          for (TopExp_Explorer anExp(aIn[2], TopAbs_EDGE); anExp.More(); anExp.Next())
          {
            aLoop.AddConstEdge(TopoDS::Edge(anExp.Current()));
          }
        }
        aLoop.Perform();
        aLoop.WiresToFaces();
        std::printf("wires %d\n", aLoop.NewWires().Extent());
        for (NCollection_List<TopoDS_Shape>::Iterator it(aLoop.NewWires()); it.More(); it.Next())
        {
          Result("wire", it.Value());
        }
        std::printf("faces %d\n", aLoop.NewFaces().Extent());
        for (NCollection_List<TopoDS_Shape>::Iterator it(aLoop.NewFaces()); it.More(); it.Next())
        {
          Result("face", it.Value());
        }
        std::printf("newedges %d\n", aLoop.NewEdges(aCut).Extent());
        for (NCollection_List<TopoDS_Shape>::Iterator it(aLoop.NewEdges(aCut)); it.More(); it.Next())
        {
          Result("edge", it.Value());
        }
      }
      catch (const Standard_Failure&)
      {
        std::printf("exception\n");
      }
      std::printf("end\n");
    }
  }
}

int main()
{
  int aCase = 0;
  for (int s = 0; s <= 14; ++s)
  {
    for (int pk = 0; pk < 3; ++pk)
    {
      const gp_Pln             aPln   = Plane(pk);
      std::vector<TopoDS_Wire> aWires = Scenario(s, aPln);
      for (int fk = 0; fk < 3; ++fk)
      {
        TopoDS_Face aFace = CaseFace(fk, aPln, aWires);
        for (int mode = 0; mode < 4; ++mode)
        {
          if (s == 14 && mode > 1)
          {
            continue; // native dereferences a null pcurve (correction mode) or crashes in FC2D (WireToFace)
          }
          char aKind[64];
          std::snprintf(aKind, sizeof(aKind), "s%d-p%d-f%d", s, pk, fk);
          Run(aCase++, aKind, mode, aFace, aWires);
        }
      }
    }
  }

  // random sets of circles, rectangles and rounded rectangles: nested, disjoint or crossing
  for (int r = 0; r < 80; ++r)
  {
    const gp_Pln             aPln = Plane(r % 3);
    std::vector<TopoDS_Wire> aWires;
    const int                n = 1 + (int)(Rand() * 5);
    for (int i = 0; i < n; ++i)
    {
      double cx = Rand(-10, 10), cy = Rand(-10, 10), sz = Rand(1, 8);
      bool   ccw = (i == 0) ? true : Rand() < 0.5;
      double k   = Rand();
      if (k < 0.4)
      {
        aWires.push_back(Circle(aPln, cx, cy, sz, ccw));
      }
      else if (k < 0.8)
      {
        aWires.push_back(Rect(aPln, cx - sz, cy - 0.6 * sz, cx + sz, cy + 0.6 * sz, ccw));
      }
      else
      {
        aWires.push_back(RoundRect(aPln, cx - sz, cy - sz, cx + sz, cy + sz, 0.3 * sz, ccw));
      }
    }
    TopoDS_Face aFace = CaseFace(r % 2, aPln, aWires);
    char        aKind[64];
    std::snprintf(aKind, sizeof(aKind), "random%d", r);
    Run(aCase++, aKind, r % 4, aFace, aWires);
  }
  Algos();
  return 0;
}
