// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
//
// chfi2drec: records native OCCT's ChFi2d_FilletAPI and ChFi2d_ChamferAPI (unit 5R) on random pairs of edges in a
// plane: lines, arcs of circles and ellipses, and cubic B-splines, sharing a vertex, meeting at a point, or apart by a
// small gap; given as a wire or as two edges. ChFi2d's two APIs aren't bound by build123d.NET, so there is no direct
// oracle. Each case's inputs are the exact doubles the edges are built from (64-bit hex); the port rebuilds the same
// edges and writes its results in the same format (tests/OcctNet.ModelingAlgorithms.Tests/ChFi2d).
// Built against build123d.NET's native OCCT V8_0_1 (tools/chfi2drec/build.ps1).
//
//   chfi2drec <cases per kind>   writes the recording to stdout
//
// One line per case:  <kind> <inputs...> ; <tokens...>
// kind: fw (fillet of a wire), fe (fillet of two edges), cw (chamfer of a wire), ce (chamfer of two edges).
// Inputs, in order: the plane (location, normal, X direction, 1 if direct), shared (1 if the edges share their
// vertex), reversed1, reversed2, the common point, edge 1, edge 2, then the radius (fillets) or the two lengths
// (chamfers). An edge is its type (0 line, 1 circle, 2 ellipse, 3 B-spline) and its data: a line its two ends; a
// circle its centre, normal, X direction, radius and two ends; an ellipse the same with two radii; a B-spline its four
// poles (cubic, one span). The tokens are named (name:value).
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <functional>
#include <string>
#include <typeinfo>
#include <vector>

#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRep_Tool.hxx>
#include <ChFi2d_ChamferAPI.hxx>
#include <ChFi2d_FilletAPI.hxx>
#include <ElCLib.hxx>
#include <ElSLib.hxx>
#include <Geom_BSplineCurve.hxx>
#include <Geom_Curve.hxx>
#include <NCollection_Array1.hxx>
#include <Standard_Failure.hxx>
#include <TopExp.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Vertex.hxx>
#include <TopoDS_Wire.hxx>
#include <gp_Ax2.hxx>
#include <gp_Ax3.hxx>
#include <gp_Circ.hxx>
#include <gp_Elips.hxx>
#include <gp_Pln.hxx>
#include <gp_Pnt.hxx>

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

static std::string Hex(const gp_Pnt& theP) { return Hex(theP.X()) + "," + Hex(theP.Y()) + "," + Hex(theP.Z()); }

static std::string ExcName(const Standard_Failure& theF)
{
  std::string aName = typeid(theF).name(); // "class Standard_OutOfRange"
  if (aName.rfind("class ", 0) == 0)
  {
    aName = aName.substr(6);
  }
  return "!" + aName;
}

// Runs an accessor, writing name:value or name:!Exception
static void Tok(const char* theName, const std::function<std::string()>& theF)
{
  std::string aVal;
  try
  {
    aVal = theF();
  }
  catch (const Standard_Failure& theF)
  {
    aVal = ExcName(theF);
  }
  Put(std::string(theName) + ":" + aVal);
}

// An edge: its curve's type, range and points at the ends and middle of the range, its orientation and its vertices
static std::string Desc(const TopoDS_Edge& theE)
{
  if (theE.IsNull())
  {
    return "null";
  }
  double                  f = 0., l = 0.;
  occ::handle<Geom_Curve> aC = BRep_Tool::Curve(theE, f, l);
  if (aC.IsNull())
  {
    return "nocurve";
  }
  std::string s = aC->DynamicType()->Name();
  s += "," + Hex(f) + "," + Hex(l) + "," + Hex(aC->Value(f)) + "," + Hex(aC->Value(0.5 * (f + l))) + ","
       + Hex(aC->Value(l));
  s += theE.Orientation() == TopAbs_FORWARD ? ",F" : (theE.Orientation() == TopAbs_REVERSED ? ",R" : ",O");
  TopoDS_Vertex aV1, aV2;
  TopExp::Vertices(theE, aV1, aV2);
  s += "," + (aV1.IsNull() ? std::string("nv") : Hex(BRep_Tool::Pnt(aV1)));
  s += "," + (aV2.IsNull() ? std::string("nv") : Hex(BRep_Tool::Pnt(aV2)));
  return s;
}

//=================================================================================================
// Random inputs

static uint64_t theState = 0xC4F12Du;

static uint64_t Next()
{
  uint64_t z = (theState += 0x9E3779B97F4A7C15ull);
  z          = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
  z          = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
  return z ^ (z >> 31);
}

static double U() { return (double)(Next() >> 11) * (1.0 / 9007199254740992.0); }

static int I(int theN) { return (int)(Next() % (uint64_t)theN); }

static double Grid(double theV) { return std::floor(theV * 1024.0 + 0.5) / 1024.0; }

static double R(double theA, double theB) { return Grid(theA + (theB - theA) * U()); }

// The inputs of the case, in reading order
static std::vector<double> theIn;

static double In(double theV)
{
  theIn.push_back(theV);
  return theV;
}

static gp_Pnt InP(const gp_Pnt& theP)
{
  In(theP.X());
  In(theP.Y());
  In(theP.Z());
  return theP;
}

// A direction as the port reads it: gp_Dir(x, y, z) of the recorded components, normalised again (which can change
// the last bit of an already unit direction)
static gp_Dir InD(const gp_Dir& theD)
{
  In(theD.X());
  In(theD.Y());
  In(theD.Z());
  return gp_Dir(theD.X(), theD.Y(), theD.Z());
}

//=================================================================================================
// The edges, built from the inputs as the port builds them

struct EdgeData
{
  int                 Type = 0;
  gp_Pnt              P1, P2;
  gp_Pnt              Center;
  gp_Dir              N, X;
  double              R1 = 0., R2 = 0.;
  std::vector<gp_Pnt> Poles;
};

static TopoDS_Edge MakeEdge(const EdgeData& theD, const TopoDS_Vertex& theV1, const TopoDS_Vertex& theV2)
{
  BRepBuilderAPI_MakeEdge aMaker;
  switch (theD.Type)
  {
    case 0:
      aMaker = BRepBuilderAPI_MakeEdge(theV1, theV2);
      break;
    case 1:
      aMaker = BRepBuilderAPI_MakeEdge(gp_Circ(gp_Ax2(theD.Center, theD.N, theD.X), theD.R1), theV1, theV2);
      break;
    case 2:
      aMaker =
        BRepBuilderAPI_MakeEdge(gp_Elips(gp_Ax2(theD.Center, theD.N, theD.X), theD.R1, theD.R2), theV1, theV2);
      break;
    default: {
      NCollection_Array1<gp_Pnt> aPoles(1, 4);
      for (int i = 1; i <= 4; i++)
      {
        aPoles(i) = theD.Poles[i - 1];
      }
      NCollection_Array1<double> aKnots(1, 2);
      aKnots(1) = 0.;
      aKnots(2) = 1.;
      NCollection_Array1<int> aMults(1, 2);
      aMults(1) = 4;
      aMults(2) = 4;
      occ::handle<Geom_BSplineCurve> aC = new Geom_BSplineCurve(aPoles, aKnots, aMults, 3);
      aMaker                              = BRepBuilderAPI_MakeEdge(aC, theV1, theV2);
      break;
    }
  }
  return aMaker.IsDone() ? aMaker.Edge() : TopoDS_Edge();
}

// A random edge of the plane with one end at theC: ending there (theIsFirst) or starting there.
static EdgeData RandomEdge(const gp_Pln& thePln, const gp_Pnt& theC, bool theIsFirst, const EdgeData* thePrev)
{
  EdgeData d;
  int      k = I(20);
  d.Type     = k < 8 ? 0 : (k < 15 ? 1 : (k < 17 ? 2 : 3));
  In(d.Type);
  double cu, cv;
  ElSLib::Parameters(thePln, theC, cu, cv);
  if (d.Type == 0)
  {
    gp_Pnt aOther;
    if (!theIsFirst && thePrev != nullptr && thePrev->Type == 0 && I(8) == 0)
    {
      // a tangent continuation of the first line, or a cusp
      gp_Vec aV(thePrev->P1, thePrev->P2);
      aOther = theC.Translated(aV.Multiplied(I(2) == 0 ? R(0.3, 1.5) : -R(0.3, 0.9)));
    }
    else
    {
      double a = 2.0 * M_PI * U(), l = R(0.5, 5.0);
      aOther   = ElSLib::Value(cu + l * std::cos(a), cv + l * std::sin(a), thePln);
    }
    d.P1 = InP(theIsFirst ? aOther : theC);
    d.P2 = InP(theIsFirst ? theC : aOther);
    return d;
  }
  if (d.Type == 1 || d.Type == 2)
  {
    double r  = R(0.5, 4.0);
    double r2 = d.Type == 2 ? R(0.3, r) : r;
    double a  = 2.0 * M_PI * U();
    gp_Dir aN = thePln.Axis().Direction();
    if (I(2) == 0)
    {
      aN.Reverse();
    }
    // the centre at the distance r from theC along X, so that theC is at the parameter 0
    gp_Dir aX(ElSLib::Value(cu + std::cos(a), cv + std::sin(a), thePln).XYZ() - theC.XYZ());
    gp_Pnt aCenter = theC.Translated(gp_Vec(aX).Multiplied(-r));
    d.Center       = InP(aCenter);
    d.N            = InD(aN);
    d.X            = InD(aX);
    d.R1           = In(r);
    if (d.Type == 2)
    {
      d.R2 = In(r2);
    }
    double t = R(0.3, 2.0 * M_PI - 0.3);
    gp_Pnt aOther, aC0;
    if (d.Type == 1)
    {
      gp_Circ aCirc(gp_Ax2(d.Center, d.N, d.X), d.R1);
      aOther = ElCLib::Value(t, aCirc);
      aC0    = ElCLib::Value(0., aCirc);
    }
    else
    {
      gp_Elips anEl(gp_Ax2(d.Center, d.N, d.X), d.R1, d.R2);
      aOther = ElCLib::Value(t, anEl);
      aC0    = ElCLib::Value(0., anEl);
    }
    // theC itself (within rounding) at the parameter 0
    d.P1 = InP(theIsFirst ? aOther : theC);
    d.P2 = InP(theIsFirst ? theC : aOther);
    return d;
  }
  // a cubic B-spline from or to theC
  double a  = 2.0 * M_PI * U();
  double l  = R(1.0, 4.0);
  gp_Pnt aE = ElSLib::Value(cu + l * std::cos(a), cv + l * std::sin(a), thePln);
  gp_Pnt aM1 = ElSLib::Value(cu + R(-2.0, 2.0), cv + R(-2.0, 2.0), thePln);
  gp_Pnt aM2 = ElSLib::Value(cu + R(-2.0, 2.0), cv + R(-2.0, 2.0), thePln);
  d.Poles    = theIsFirst ? std::vector<gp_Pnt>{aE, aM1, aM2, theC} : std::vector<gp_Pnt>{theC, aM1, aM2, aE};
  for (gp_Pnt& p : d.Poles)
  {
    InP(p);
  }
  d.P1 = d.Poles[0];
  d.P2 = d.Poles[3];
  return d;
}

//=================================================================================================

static void RunCase(const char* theKind)
{
  theOut.clear();
  theIn.clear();
  const bool isFillet = theKind[0] == 'f';
  const bool isWire   = theKind[1] == 'w';

  // the plane: XY, or a random one, sometimes left-handed
  gp_Pnt aLoc(0., 0., 0.);
  gp_Dir aN(0., 0., 1.), aX(1., 0., 0.);
  bool   isDirect = true;
  if (I(2) == 0)
  {
    aLoc = gp_Pnt(R(-5., 5.), R(-5., 5.), R(-5., 5.));
    gp_Vec aV(R(-1., 1.), R(-1., 1.), R(-1., 1.));
    if (aV.Magnitude() < 0.1)
    {
      aV = gp_Vec(0., 0., 1.);
    }
    gp_Ax3 anAx(aLoc, gp_Dir(aV));
    aN       = anAx.Direction();
    aX       = anAx.XDirection();
    isDirect = I(5) != 0;
  }
  InP(aLoc);
  aN = InD(aN);
  aX = InD(aX);
  In(isDirect ? 1. : 0.);
  gp_Ax3 anAx3(aLoc, aN, aX);
  if (!isDirect)
  {
    anAx3.YReverse();
  }
  gp_Pln aPln(anAx3);

  // shared vertex, gap, reversed edges
  int  k        = I(10);
  bool isShared = k < 7;
  In(isShared ? 1. : 0.);
  bool isRev1 = I(4) == 0, isRev2 = I(4) == 0;
  In(isRev1 ? 1. : 0.);
  In(isRev2 ? 1. : 0.);

  gp_Pnt aC = InP(ElSLib::Value(R(-3., 3.), R(-3., 3.), aPln));
  // the start of the second edge: the common point, or a point a gap away (in the plane)
  gp_Pnt aC2 = aC;
  if (k >= 8)
  {
    double cu, cv, g = std::pow(10.0, -(double)(3 + I(6)));
    ElSLib::Parameters(aPln, aC, cu, cv);
    aC2 = ElSLib::Value(cu + g, cv - 0.5 * g, aPln);
  }
  EdgeData d1 = RandomEdge(aPln, aC, true, nullptr);
  EdgeData d2 = RandomEdge(aPln, aC2, false, &d1);
  double   aRadius = 0., aL1 = 0., aL2 = 0.;
  if (isFillet)
  {
    int r   = I(10);
    aRadius = In(r == 0 ? 0.0 : (r < 6 ? 0.25 * (1 + I(8)) : R(0.01, 3.0)));
  }
  else
  {
    aL1 = In(R(0.05, 2.5));
    aL2 = In(I(3) == 0 ? aL1 : R(0.05, 2.5));
  }

  // the edges, from the inputs
  TopoDS_Vertex aV11 = BRepBuilderAPI_MakeVertex(d1.Type == 3 ? d1.Poles[0] : d1.P1).Vertex();
  TopoDS_Vertex aV12 = BRepBuilderAPI_MakeVertex(d1.Type == 3 ? d1.Poles[3] : d1.P2).Vertex();
  TopoDS_Vertex aV21 = isShared ? aV12 : BRepBuilderAPI_MakeVertex(d2.Type == 3 ? d2.Poles[0] : d2.P1).Vertex();
  TopoDS_Vertex aV22 = BRepBuilderAPI_MakeVertex(d2.Type == 3 ? d2.Poles[3] : d2.P2).Vertex();

  for (double v : theIn)
  {
    Put(Hex(v));
  }
  Put(";");

  TopoDS_Edge anE1, anE2;
  try
  {
    anE1 = MakeEdge(d1, aV11, aV12);
    anE2 = MakeEdge(d2, aV21, aV22);
  }
  catch (const Standard_Failure& theF)
  {
    Put("edges:" + ExcName(theF));
  }
  if (anE1.IsNull() || anE2.IsNull())
  {
    Put("edges:0");
    std::printf("%s%s\n", theKind, theOut.c_str());
    return;
  }
  if (isRev1)
  {
    anE1.Reverse();
  }
  if (isRev2)
  {
    anE2.Reverse();
  }
  TopoDS_Wire aWire;
  if (isWire)
  {
    BRepBuilderAPI_MakeWire aMW(anE1, anE2);
    if (!aMW.IsDone())
    {
      Put("wire:0");
      std::printf("%s%s\n", theKind, theOut.c_str());
      return;
    }
    aWire = aMW.Wire();
  }

  if (isFillet)
  {
    ChFi2d_FilletAPI anAlgo;
    bool             isInit = false;
    Tok("init", [&] {
      if (isWire)
        anAlgo.Init(aWire, aPln);
      else
        anAlgo.Init(anE1, anE2, aPln);
      isInit = true;
      return std::string("1");
    });
    if (!isInit)
    {
      std::printf("%s%s\n", theKind, theOut.c_str());
      return;
    }
    bool isDone = false;
    Tok("perf", [&] {
      isDone = anAlgo.Perform(aRadius);
      return std::string(isDone ? "1" : "0");
    });
    if (isDone)
    {
      Tok("nb", [&] { return std::to_string(anAlgo.NbResults(aC)); });
      for (int iSol : {-1, 1, 2})
      {
        TopoDS_Edge aM1, aM2, aF;
        bool        isRes = false;
        Tok(iSol == -1 ? "res" : (iSol == 1 ? "res1" : "res2"), [&] {
          aF    = anAlgo.Result(aC, aM1, aM2, iSol);
          isRes = true;
          return Desc(aF);
        });
        if (isRes)
        {
          Put("m1:" + Desc(aM1));
          Put("m2:" + Desc(aM2));
        }
      }
    }
  }
  else
  {
    ChFi2d_ChamferAPI anAlgo;
    if (isWire)
      anAlgo.Init(aWire);
    else
      anAlgo.Init(anE1, anE2);
    bool isDone = false;
    Tok("perf", [&] {
      isDone = anAlgo.Perform();
      return std::string(isDone ? "1" : "0");
    });
    if (isDone)
    {
      TopoDS_Edge aM1, aM2;
      bool        isRes = false;
      Tok("res", [&] {
        TopoDS_Edge aCh = anAlgo.Result(aM1, aM2, aL1, aL2);
        isRes           = true;
        return Desc(aCh);
      });
      if (isRes)
      {
        Put("m1:" + Desc(aM1));
        Put("m2:" + Desc(aM2));
      }
    }
  }
  std::printf("%s%s\n", theKind, theOut.c_str());
}

int main(int argc, char** argv)
{
  int aNbCases = argc > 1 ? std::atoi(argv[1]) : 100;
  for (const char* aKind : {"fw", "fe", "cw", "ce"})
  {
    for (int k = 0; k < aNbCases; k++)
    {
      RunCase(aKind);
    }
  }
  return 0;
}
