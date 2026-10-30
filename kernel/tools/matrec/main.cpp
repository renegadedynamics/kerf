// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
//
// matrec: records native OCCT's bisecting locus (MAT, MAT2d, BRepMAT2d; unit 5M) on planar faces: polygons convex
// and not, with circular arcs, holed regions, full circles, and open wires. Each case builds a face from a text
// description, then runs BRepMAT2d_Explorer, BRepMAT2d_BisectingLocus::Compute and BRepMAT2d_LinkTopoBilo, and writes
// the explorer's contours, the graph (arcs, nodes, basic elements and their links), each arc's bisector (its kind,
// bounds and points along it) and the links to the edges and vertices, every double as its 64 bits in hex. The
// inputs are written as round-trip decimals; the port builds the same face from the same text
// (tests/OcctNet.ModelingAlgorithms.Tests/BRepMAT2d). Built against build123d.NET's native OCCT V8_0_1
// (tools/matrec/build.ps1).
//
//   matrec <random cases>   writes the recording to stdout
//
// One line per case:  <inputs...> ; <tokens...>
//   M <side L|R> <join 0|2> <openresult 0|1> <nwires> <wire>...
//   wire:  W <closed 0|1> <nvertices> (x y k [...])...   k = L (segment to the next vertex), A mx my (arc through
//          (mx, my) to the next vertex) or S ax ay bx by (cubic Bezier curve with the middle poles (ax, ay) and
//          (bx, by)); the last vertex of an open wire has k = -
//          O cx cy r                                          a full circle (one closed edge), counterclockwise
// The face is on the XOY plane: the first closed wire bounds it (BRepBuilderAPI_MakeFace(plane, wire)), the other
// wires are added (BRepBuilderAPI_MakeFace(face, wire)); with an open wire first, the face is the infinite plane.
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <sstream>
#include <string>
#include <typeinfo>
#include <vector>

#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepMAT2d_BisectingLocus.hxx>
#include <BRepMAT2d_Explorer.hxx>
#include <BRepMAT2d_LinkTopoBilo.hxx>
#include <Bisector_Bisec.hxx>
#include <Bisector_BisecAna.hxx>
#include <GC_MakeArcOfCircle.hxx>
#include <Geom2d_CartesianPoint.hxx>
#include <Geom2d_TrimmedCurve.hxx>
#include <Geom_BezierCurve.hxx>
#include <Geom_TrimmedCurve.hxx>
#include <NCollection_Array1.hxx>
#include <MAT_Arc.hxx>
#include <MAT_BasicElt.hxx>
#include <MAT_Graph.hxx>
#include <MAT_Node.hxx>
#include <NCollection_IndexedMap.hxx>
#include <Precision.hxx>
#include <Standard_Failure.hxx>
#include <TopExp.hxx>
#include <TopTools_ShapeMapHasher.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Wire.hxx>
#include <TopoDS_Vertex.hxx>
#include <TopoDS_Edge.hxx>
#include <algorithm>
#include <gp.hxx>
#include <gp_Circ.hxx>
#include <gp_Pln.hxx>

static std::string theOut;

static void Put(const std::string& theTok)
{
  theOut += ' ';
  theOut += theTok;
}

static std::string Hex(double theValue)
{
  uint64_t aBits;
  std::memcpy(&aBits, &theValue, 8);
  char aBuf[20];
  std::snprintf(aBuf, sizeof(aBuf), "%llx", (unsigned long long)aBits);
  return aBuf;
}

static std::string Hex(const gp_Pnt2d& theP) { return Hex(theP.X()) + "," + Hex(theP.Y()); }

static std::string Dec(double theV)
{
  char aBuf[40];
  std::snprintf(aBuf, sizeof(aBuf), "%.17g", theV);
  return aBuf;
}

static std::string TypeName(const std::type_info& theT)
{
  std::string aName = theT.name(); // "class MAT_Arc"
  if (aName.rfind("class ", 0) == 0)
  {
    aName = aName.substr(6);
  }
  return aName;
}

static std::string Fail(const std::function<void()>& theF)
{
  try
  {
    theF();
  }
  catch (const Standard_Failure& theFail)
  {
    return "!" + TypeName(typeid(theFail));
  }
  catch (...)
  {
    return "!SEH";
  }
  return "";
}

// Runs an accessor, writing name:value or name:!Exception
static void Tok(const std::string& theName, const std::function<std::string()>& theF)
{
  std::string aVal;
  std::string aFail = Fail([&]() { aVal = theF(); });
  Put(theName + ":" + (aFail.empty() ? aVal : aFail));
}

//=================================================================================================
// The face (the C# side parses and builds it the same way).

struct Reader
{
  std::istringstream myIn;

  explicit Reader(const std::string& theText)
      : myIn(theText)
  {
  }

  std::string Word()
  {
    std::string aW;
    myIn >> aW;
    return aW;
  }

  double D() { return std::strtod(Word().c_str(), nullptr); }

  int I() { return std::atoi(Word().c_str()); }
};

static TopoDS_Wire ReadWire(Reader& theR, bool& theClosed)
{
  std::string aKind = theR.Word();
  if (aKind == "O")
  {
    double cx = theR.D(), cy = theR.D(), r = theR.D();
    gp_Circ aC(gp_Ax2(gp_Pnt(cx, cy, 0.0), gp::DZ()), r);
    theClosed = true;
    return BRepBuilderAPI_MakeWire(BRepBuilderAPI_MakeEdge(aC).Edge()).Wire();
  }
  theClosed = theR.I() != 0;
  int                        aNb = theR.I();
  std::vector<gp_Pnt>        aPnts;
  std::vector<std::string>   aKinds;
  std::vector<gp_Pnt>        aMids;
  std::vector<gp_Pnt>        aMids2;
  std::vector<TopoDS_Vertex> aVerts;
  for (int i = 0; i < aNb; i++)
  {
    double x = theR.D(), y = theR.D();
    aPnts.push_back(gp_Pnt(x, y, 0.0));
    std::string k = theR.Word();
    aKinds.push_back(k);
    if (k == "A")
    {
      double mx = theR.D(), my = theR.D();
      aMids.push_back(gp_Pnt(mx, my, 0.0));
      aMids2.push_back(gp_Pnt());
    }
    else if (k == "S")
    {
      double mx = theR.D(), my = theR.D(), nx = theR.D(), ny = theR.D();
      aMids.push_back(gp_Pnt(mx, my, 0.0));
      aMids2.push_back(gp_Pnt(nx, ny, 0.0));
    }
    else
    {
      aMids.push_back(gp_Pnt());
      aMids2.push_back(gp_Pnt());
    }
    aVerts.push_back(BRepBuilderAPI_MakeVertex(aPnts.back()).Vertex());
  }
  BRepBuilderAPI_MakeWire aMW;
  int                     aNbEdges = theClosed ? aNb : aNb - 1;
  for (int i = 0; i < aNbEdges; i++)
  {
    int j = (i + 1) % aNb;
    if (aKinds[i] == "A")
    {
      occ::handle<Geom_TrimmedCurve> anArc = GC_MakeArcOfCircle(aPnts[i], aMids[i], aPnts[j]).Value();
      aMW.Add(BRepBuilderAPI_MakeEdge(anArc, aVerts[i], aVerts[j]).Edge());
    }
    else if (aKinds[i] == "S")
    {
      NCollection_Array1<gp_Pnt> aPoles(1, 4);
      aPoles(1)                        = aPnts[i];
      aPoles(2)                        = aMids[i];
      aPoles(3)                        = aMids2[i];
      aPoles(4)                        = aPnts[j];
      occ::handle<Geom_BezierCurve> aB = new Geom_BezierCurve(aPoles);
      aMW.Add(BRepBuilderAPI_MakeEdge(aB, aVerts[i], aVerts[j]).Edge());
    }
    else
    {
      aMW.Add(BRepBuilderAPI_MakeEdge(aVerts[i], aVerts[j]).Edge());
    }
  }
  return aMW.Wire();
}

static TopoDS_Face ReadFace(Reader& theR)
{
  int         aNbWires = theR.I();
  TopoDS_Face aFace;
  for (int i = 0; i < aNbWires; i++)
  {
    bool        isClosed = false;
    TopoDS_Wire aW       = ReadWire(theR, isClosed);
    if (i == 0)
    {
      if (isClosed)
      {
        aFace = BRepBuilderAPI_MakeFace(gp_Pln(gp::XOY()), aW, true).Face();
      }
      else
      {
        aFace = BRepBuilderAPI_MakeFace(BRepBuilderAPI_MakeFace(gp_Pln(gp::XOY())).Face(), aW).Face();
      }
    }
    else
    {
      aFace = BRepBuilderAPI_MakeFace(aFace, aW).Face();
    }
  }
  return aFace;
}

//=================================================================================================
// The results.

static std::string ShapeName(const TopoDS_Shape&                                                  theS,
                             const NCollection_IndexedMap<TopoDS_Shape, TopTools_ShapeMapHasher>& theEdges,
                             const NCollection_IndexedMap<TopoDS_Shape, TopTools_ShapeMapHasher>& theVerts)
{
  static const char* anOri = "FRIE";
  if (theS.IsNull())
  {
    return "null";
  }
  std::string aName;
  if (theS.ShapeType() == TopAbs_EDGE)
  {
    aName = "E" + std::to_string(theEdges.FindIndex(theS));
  }
  else if (theS.ShapeType() == TopAbs_VERTEX)
  {
    aName = "V" + std::to_string(theVerts.FindIndex(theS));
  }
  else
  {
    aName = "S" + std::to_string((int)theS.ShapeType());
  }
  return aName + anOri[(int)theS.Orientation()];
}

static std::string GeomName(const occ::handle<Geom2d_Geometry>& theG)
{
  if (theG.IsNull())
  {
    return "null";
  }
  std::string aName = TypeName(typeid(*theG));
  occ::handle<Geom2d_TrimmedCurve> aT = occ::down_cast<Geom2d_TrimmedCurve>(theG);
  if (!aT.IsNull())
  {
    aName += "/" + TypeName(typeid(*aT->BasisCurve())) + "," + Hex(aT->FirstParameter()) + ","
             + Hex(aT->LastParameter());
  }
  occ::handle<Geom2d_CartesianPoint> aP = occ::down_cast<Geom2d_CartesianPoint>(theG);
  if (!aP.IsNull())
  {
    aName += "," + Hex(aP->Pnt2d());
  }
  return aName;
}

static void Bisec(const std::string& thePre, const Bisector_Bisec& theB)
{
  occ::handle<Geom2d_TrimmedCurve> aC = theB.Value();
  if (aC.IsNull())
  {
    Put(thePre + "null");
    return;
  }
  occ::handle<Geom2d_Curve> aBasis = aC->BasisCurve();
  std::string               aKind  = TypeName(typeid(*aBasis));
  occ::handle<Bisector_BisecAna> anAna = occ::down_cast<Bisector_BisecAna>(aBasis);
  if (!anAna.IsNull())
  {
    Tok(thePre + "G", [&]() { return TypeName(typeid(*anAna->Geom2dCurve())); });
  }
  Put(thePre + "T:" + aKind);
  double f = aC->FirstParameter(), l = aC->LastParameter();
  Put(thePre + "R:" + Hex(f) + "," + Hex(l));
  static const double aFrac[5] = {0.0, 0.25, 0.5, 0.75, 1.0};
  static const double aStep[5] = {0.0, 0.5, 1.0, 2.0, 5.0};
  for (int i = 0; i < 5; i++)
  {
    double u = (l >= 1.e50 || f <= -1.e50) ? (f <= -1.e50 ? l - aStep[i] : f + aStep[i]) : f + (l - f) * aFrac[i];
    Tok(thePre + "P" + std::to_string(i), [&]() { return Hex(aC->Value(u)); });
  }
}

static void RunCase(const std::string& theInput)
{
  theOut = theInput + " ;";
  Reader      aR(theInput);
  std::string aM    = aR.Word();
  std::string aSide = aR.Word();
  int         aJoin = aR.I();
  int         anOpen = aR.I();
  TopoDS_Face aFace;
  std::string aFail = Fail([&]() { aFace = ReadFace(aR); });
  if (!aFail.empty())
  {
    Put("face:" + aFail);
    std::printf("%s\n", theOut.c_str());
    return;
  }
  NCollection_IndexedMap<TopoDS_Shape, TopTools_ShapeMapHasher> anEdges, aVerts;
  TopExp::MapShapes(aFace, TopAbs_EDGE, anEdges);
  TopExp::MapShapes(aFace, TopAbs_VERTEX, aVerts);
  Put("nE:" + std::to_string(anEdges.Extent()));

  BRepMAT2d_Explorer anExplo;
  aFail = Fail([&]() { anExplo.Perform(aFace); });
  if (!aFail.empty())
  {
    Put("explo:" + aFail);
    std::printf("%s\n", theOut.c_str());
    return;
  }
  Put("nC:" + std::to_string(anExplo.NumberOfContours()));
  for (int ic = 1; ic <= anExplo.NumberOfContours(); ic++)
  {
    Put("c" + std::to_string(ic) + ":" + std::to_string(anExplo.NumberOfCurves(ic)));
  }
  Put(std::string("mod:") + (anExplo.IsModified(aFace) ? "1" : "0"));

  BRepMAT2d_BisectingLocus aLocus;
  aFail = Fail([&]() {
    aLocus.Compute(anExplo, 1, aSide == "L" ? MAT_Left : MAT_Right, (GeomAbs_JoinType)aJoin, anOpen != 0);
  });
  if (!aFail.empty())
  {
    Put("compute:" + aFail);
    std::printf("%s\n", theOut.c_str());
    return;
  }
  Put(std::string("done:") + (aLocus.IsDone() ? "1" : "0"));
  if (!aLocus.IsDone())
  {
    std::printf("%s\n", theOut.c_str());
    return;
  }
  for (int ic = 1; ic <= aLocus.NumberOfContours(); ic++)
  {
    Tok("ne" + std::to_string(ic), [&]() { return std::to_string(aLocus.NumberOfElts(ic)); });
    for (int j = 1; j <= anExplo.NumberOfCurves(ic); j++)
    {
      Tok("ns" + std::to_string(ic) + "." + std::to_string(j),
          [&]() { return std::to_string(aLocus.NumberOfSections(ic, j)); });
    }
    Tok("be" + std::to_string(ic), [&]() {
      std::string aS;
      for (int ie = 1; ie <= aLocus.NumberOfElts(ic); ie++)
      {
        aS += (ie > 1 ? "," : "") + std::to_string(aLocus.BasicElt(ic, ie)->Index());
      }
      return aS;
    });
  }
  occ::handle<MAT_Graph> aG = aLocus.Graph();
  Put("g:" + std::to_string(aG->NumberOfArcs()) + "," + std::to_string(aG->NumberOfNodes()) + ","
      + std::to_string(aG->NumberOfBasicElts()) + "," + std::to_string(aG->NumberOfInfiniteNodes()));
  for (int i = 1; i <= aG->NumberOfArcs(); i++)
  {
    std::string aPre = "a" + std::to_string(i);
    Tok(aPre, [&]() {
      occ::handle<MAT_Arc> anArc = aG->Arc(i);
      return std::to_string(anArc->Index()) + "," + std::to_string(anArc->GeomIndex()) + ","
             + std::to_string(anArc->FirstElement()->Index()) + ","
             + std::to_string(anArc->SecondElement()->Index()) + ","
             + std::to_string(anArc->FirstNode()->Index()) + ","
             + std::to_string(anArc->SecondNode()->Index());
    });
    aFail = Fail([&]() {
      bool           aRev = false;
      Bisector_Bisec aB   = aLocus.GeomBis(aG->Arc(i), aRev);
      Put(aPre + "rev:" + (aRev ? "1" : "0"));
      Bisec(aPre, aB);
    });
    if (!aFail.empty())
    {
      Put(aPre + "bis:" + aFail);
    }
  }
  for (int i = 1; i <= aG->NumberOfNodes(); i++)
  {
    std::string aPre = "n" + std::to_string(i);
    Tok(aPre, [&]() {
      occ::handle<MAT_Node> aN = aG->Node(i);
      return std::to_string(aN->Index()) + "," + std::to_string(aN->GeomIndex()) + "," + Hex(aN->Distance())
             + "," + (aN->Infinite() ? "1" : "0") + (aN->OnBasicElt() ? "1" : "0")
             + (aN->PendingNode() ? "1" : "0");
    });
    Tok(aPre + "p", [&]() { return Hex(aLocus.GeomElt(aG->Node(i))); });
    Tok(aPre + "l", [&]() {
      NCollection_Sequence<occ::handle<MAT_Arc>> aS;
      aG->Node(i)->LinkedArcs(aS);
      std::string aRes;
      for (int k = 1; k <= aS.Length(); k++)
      {
        aRes += (k > 1 ? "," : "") + std::to_string(aS(k)->Index());
      }
      return aRes;
    });
  }
  for (int i = 1; i <= aG->NumberOfBasicElts(); i++)
  {
    std::string aPre = "b" + std::to_string(i);
    Tok(aPre, [&]() {
      occ::handle<MAT_BasicElt> aBE = aG->BasicElt(i);
      return std::to_string(aBE->Index()) + "," + std::to_string(aBE->GeomIndex()) + ","
             + (aBE->StartArc().IsNull() ? "0" : std::to_string(aBE->StartArc()->Index())) + ","
             + (aBE->EndArc().IsNull() ? "0" : std::to_string(aBE->EndArc()->Index()));
    });
    Tok(aPre + "g", [&]() { return GeomName(aLocus.GeomElt(aG->BasicElt(i))); });
  }

  BRepMAT2d_LinkTopoBilo aLink;
  aFail = Fail([&]() { aLink.Perform(anExplo, aLocus); });
  if (!aFail.empty())
  {
    Put("link:" + aFail);
    std::printf("%s\n", theOut.c_str());
    return;
  }
  for (int k = 1; k <= anEdges.Extent() + aVerts.Extent(); k++)
  {
    const TopoDS_Shape& aS  = k <= anEdges.Extent() ? anEdges(k) : aVerts(k - anEdges.Extent());
    std::string         aNm = k <= anEdges.Extent() ? "lE" + std::to_string(k)
                                                    : "lV" + std::to_string(k - anEdges.Extent());
    Tok(aNm, [&]() {
      std::string aRes;
      for (aLink.Init(aS); aLink.More(); aLink.Next())
      {
        aRes += (aRes.empty() ? "" : ",") + std::to_string(aLink.Value()->Index());
      }
      return aRes;
    });
  }
  for (int i = 1; i <= aG->NumberOfBasicElts(); i++)
  {
    Tok("gs" + std::to_string(i),
        [&]() { return ShapeName(aLink.GeneratingShape(aG->BasicElt(i)), anEdges, aVerts); });
  }
  std::printf("%s\n", theOut.c_str());
}

//=================================================================================================
// The cases.

static uint64_t theSeed = 0x5eed5eed12345678ull;

static double Rand()
{
  theSeed = theSeed * 6364136223846793005ull + 1442695040888963407ull;
  return (double)(theSeed >> 11) * (1.0 / 9007199254740992.0);
}

// Rounded to a multiple of 1/64, so that the decimals are exact.
static double Grid(double theV) { return std::floor(theV * 64.0 + 0.5) / 64.0; }

struct Vtx
{
  double x, y;
  bool   arc;
  double mx, my;
  bool   bez = false;
  double nx = 0.0, ny = 0.0;
};

static std::string WireText(const std::vector<Vtx>& theV, bool theClosed)
{
  std::string aS = std::string("W ") + (theClosed ? "1 " : "0 ") + std::to_string(theV.size());
  for (size_t i = 0; i < theV.size(); i++)
  {
    aS += " " + Dec(theV[i].x) + " " + Dec(theV[i].y);
    if (!theClosed && i + 1 == theV.size())
    {
      aS += " -";
    }
    else if (theV[i].bez)
    {
      aS += " S " + Dec(theV[i].mx) + " " + Dec(theV[i].my) + " " + Dec(theV[i].nx) + " " + Dec(theV[i].ny);
    }
    else if (theV[i].arc)
    {
      aS += " A " + Dec(theV[i].mx) + " " + Dec(theV[i].my);
    }
    else
    {
      aS += " L";
    }
  }
  return aS;
}

// Bulges some segments into arcs through a point off their middle.
static void Bulge(std::vector<Vtx>& theV, bool theClosed, double theProba, double theMax)
{
  size_t aNb = theClosed ? theV.size() : theV.size() - 1;
  for (size_t i = 0; i < aNb; i++)
  {
    if (Rand() >= theProba)
    {
      continue;
    }
    const Vtx& b  = theV[(i + 1) % theV.size()];
    double     dx = b.x - theV[i].x, dy = b.y - theV[i].y;
    double     h  = (Rand() * 2.0 - 1.0) * theMax;
    if (std::abs(h) < 0.1)
    {
      continue;
    }
    theV[i].arc = true;
    theV[i].mx  = Grid((theV[i].x + b.x) * 0.5 - dy * h);
    theV[i].my  = Grid((theV[i].y + b.y) * 0.5 + dx * h);
  }
}

// Bends some straight segments into cubic Bezier curves (S shapes or bows).
static void Bend(std::vector<Vtx>& theV, bool theClosed, double theProba, double theMax)
{
  size_t aNb = theClosed ? theV.size() : theV.size() - 1;
  for (size_t i = 0; i < aNb; i++)
  {
    if (theV[i].arc || Rand() >= theProba)
    {
      continue;
    }
    const Vtx& b  = theV[(i + 1) % theV.size()];
    double     dx = b.x - theV[i].x, dy = b.y - theV[i].y;
    double     h1 = (Rand() * 2.0 - 1.0) * theMax, h2 = (Rand() * 2.0 - 1.0) * theMax;
    theV[i].bez = true;
    theV[i].mx  = Grid(theV[i].x + dx / 3.0 - dy * h1);
    theV[i].my  = Grid(theV[i].y + dy / 3.0 + dx * h1);
    theV[i].nx  = Grid(theV[i].x + 2.0 * dx / 3.0 - dy * h2);
    theV[i].ny  = Grid(theV[i].y + 2.0 * dy / 3.0 + dx * h2);
  }
}

// A star-shaped polygon around (cx, cy), counterclockwise unless theCW.
static std::vector<Vtx> Star(int theN, double theCx, double theCy, double theRmin, double theRmax, bool theCW)
{
  std::vector<Vtx> aV;
  for (int k = 0; k < theN; k++)
  {
    double a = 2.0 * M_PI * (k + (Rand() - 0.5) * 0.6) / theN;
    double r = theRmin + (theRmax - theRmin) * Rand();
    aV.push_back({Grid(theCx + r * std::cos(a)), Grid(theCy + r * std::sin(a)), false, 0.0, 0.0});
  }
  if (theCW)
  {
    std::reverse(aV.begin(), aV.end());
  }
  return aV;
}

static void RunVariants(const std::string& theFace, bool theOpen)
{
  if (theOpen)
  {
    RunCase("M L 0 0 " + theFace);
    RunCase("M L 0 1 " + theFace);
    RunCase("M R 2 1 " + theFace);
    RunCase("M R 0 0 " + theFace);
  }
  else
  {
    RunCase("M L 0 0 " + theFace);
    RunCase("M R 0 0 " + theFace);
    RunCase("M L 2 0 " + theFace);
    RunCase("M R 2 0 " + theFace);
  }
}

static const char* thePresets[] = {
  // square, rectangle, triangle, L-shape, concave arrow
  "1 W 1 4 0 0 L 10 0 L 10 10 L 0 10 L",
  "1 W 1 4 0 0 L 20 0 L 20 5 L 0 5 L",
  "1 W 1 3 0 0 L 12 0 L 3 9 L",
  "1 W 1 6 0 0 L 10 0 L 10 4 L 4 4 L 4 10 L 0 10 L",
  "1 W 1 7 0 0 L 6 3 L 12 0 L 9 6 L 12 12 L 6 9 L 0 12 L",
  // circles: a full circle, two half circles, a slot, a rounded rectangle, a D shape
  "1 O 0 0 5",
  "1 W 1 2 5 0 A 0 5 -5 0 A 0 -5",
  "1 W 1 4 0 0 L 10 0 A 13 3 10 6 L 0 6 A -3 3",
  "1 W 1 8 1 0 L 9 0 A 9.70703125 0.29296875 10 1 L 10 5 A 9.70703125 5.70703125 9 6 L 1 6 A 0.29296875 5.70703125 0 5 L 0 1 A 0.29296875 0.29296875",
  "1 W 1 2 0 -5 A 5 0 0 5 L",
  // holes: square in square, circle in square, two holes, off-centre hole
  "2 W 1 4 0 0 L 10 0 L 10 10 L 0 10 L W 1 4 3 3 L 3 7 L 7 7 L 7 3 L",
  "2 W 1 4 0 0 L 10 0 L 10 10 L 0 10 L W 1 2 7 5 A 5 3 3 5 A 5 7",
  "3 W 1 4 0 0 L 20 0 L 20 10 L 0 10 L W 1 4 3 3 L 3 7 L 7 7 L 7 3 L W 1 4 13 3 L 13 7 L 17 7 L 17 3 L",
  "2 W 1 4 0 0 L 10 0 L 10 10 L 0 10 L W 1 3 6 2 L 7 5 L 8 2 L",
  "2 O 0 0 10 W 1 4 -2 -2 L -2 2 L 2 2 L 2 -2 L",
  // Bezier curves: a bow, an S-shaped side, a drop
  "1 W 1 4 0 0 S 3 -2 7 -2 10 0 L 10 10 L 0 10 L",
  "1 W 1 4 0 0 S 3 -3 7 3 10 0 L 10 10 L 0 10 L",
  "1 W 1 2 0 0 S 4 -6 8 0 6 6 S 3 9 -2 4",
  "2 W 1 4 0 0 S 4 -2 8 2 12 0 L 12 12 L 0 12 L W 1 4 4 4 L 4 8 L 8 8 S 7 6 9 5 8 4 L",
};

static const char* theOpenPresets[] = {
  "1 W 0 2 0 0 L 10 0 -",
  "1 W 0 3 0 0 L 10 0 L 10 10 -",
  "1 W 0 4 0 0 L 5 5 L 10 0 L 15 5 -",
  "1 W 0 3 0 0 A 5 3 10 0 L 15 4 -",
  "1 W 0 2 0 0 A 5 5 10 0 -",
  "1 W 0 2 0 0 S 3 4 7 -4 10 0 -",
};

int main(int argc, char** argv)
{
  int aNbCases = argc > 1 ? std::atoi(argv[1]) : 60;
  for (const char* aPreset : thePresets)
  {
    RunVariants(aPreset, false);
  }
  for (const char* aPreset : theOpenPresets)
  {
    RunVariants(aPreset, true);
  }
  for (int k = 0; k < aNbCases; k++)
  {
    // a star polygon, sometimes with arcs, sometimes with a hole
    int              aN = 3 + (int)(Rand() * 10.0);
    std::vector<Vtx> aOuter = Star(aN, 0.0, 0.0, 4.0, 10.0, false);
    Bulge(aOuter, true, k % 3 == 0 ? 0.0 : 0.4, 0.3);
    if (k % 5 == 1 || k % 5 == 2)
    {
      Bend(aOuter, true, 0.4, 0.25);
    }
    std::string aFace = WireText(aOuter, true);
    int         aNbW  = 1;
    if (k % 4 == 3)
    {
      std::vector<Vtx> aHole = Star(3 + (int)(Rand() * 4.0), 0.0, 0.0, 0.75, 1.5, true);
      Bulge(aHole, true, 0.3, 0.2);
      aFace += " " + WireText(aHole, true);
      aNbW++;
    }
    RunVariants(std::to_string(aNbW) + " " + aFace, false);
  }
  for (int k = 0; k < aNbCases / 3; k++)
  {
    // an open polyline, sometimes with arcs
    int              aN = 2 + (int)(Rand() * 5.0);
    std::vector<Vtx> aV;
    double           x = 0.0, y = 0.0;
    for (int i = 0; i < aN; i++)
    {
      aV.push_back({Grid(x), Grid(y), false, 0.0, 0.0});
      x += 2.0 + 4.0 * Rand();
      y += (Rand() * 2.0 - 1.0) * 4.0;
    }
    Bulge(aV, false, 0.3, 0.25);
    if (k % 2 == 1)
    {
      Bend(aV, false, 0.4, 0.25);
    }
    RunVariants("1 " + WireText(aV, false), true);
  }
  return 0;
}
