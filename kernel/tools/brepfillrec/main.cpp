// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
//
// brepfillrec: records native OCCT's sweep laws and sections of BRepFill (unit 5S1): ShapeUpgrade_RemoveLocations,
// the location laws (Edge3DLaw with several trihedrons, ACRLaw and Edge3DLaw with guides, EdgeOnSurfLaw), the section
// laws (ShapeLaw on vertices and wires, constant and evolved; NSections), BRepFill_Section and
// BRepFill_SectionPlacement. Every double is written as its 64 bits in hex; shapes as their BRep text with the Free
// flag masked. Built against build123d.NET's native OCCT V8_0_1 (tools/brepfillrec/build.ps1).
//
// The inputs are built from the deterministic generator of tools/sweeprec, which
// tests/OcctNet.ModelingAlgorithms.Tests/BRepFill/BRepFillRecordingCases.cs repeats line for line. Each case starts
// with "CASE <kind> <seed>".
//
//   brepfillrec > brepfillrec.txt
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cmath>
#include <sstream>
#include <string>
#include <typeinfo>
#include <vector>

#include <BRepAdaptor_CompCurve.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepFill_ACRLaw.hxx>
#include <BRepFill_Edge3DLaw.hxx>
#include <BRepFill_EdgeOnSurfLaw.hxx>
#include <BRepFill_LocationLaw.hxx>
#include <BRepFill_NSections.hxx>
#include <BRepFill_Section.hxx>
#include <BRepFill_SectionLaw.hxx>
#include <BRepFill_SectionPlacement.hxx>
#include <BRepFill_ShapeLaw.hxx>
#include <BRepLib.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepTools.hxx>
#include <BRep_Builder.hxx>
#include <BRep_Tool.hxx>
#include <GC_MakeArcOfCircle.hxx>
#include <Geom2d_Line.hxx>
#include <Geom2d_TrimmedCurve.hxx>
#include <GeomFill_ConstantBiNormal.hxx>
#include <GeomFill_CorrectedFrenet.hxx>
#include <GeomFill_CurveAndTrihedron.hxx>
#include <GeomFill_Fixed.hxx>
#include <GeomFill_Frenet.hxx>
#include <GeomFill_GuideTrihedronAC.hxx>
#include <GeomFill_GuideTrihedronPlan.hxx>
#include <GeomFill_LocationGuide.hxx>
#include <GeomFill_LocationLaw.hxx>
#include <GeomFill_SectionLaw.hxx>
#include <Geom_BSplineCurve.hxx>
#include <Geom_BSplineSurface.hxx>
#include <Geom_CylindricalSurface.hxx>
#include <Geom_Plane.hxx>
#include <Geom_TrimmedCurve.hxx>
#include <Law_Linear.hxx>
#include <NCollection_Array1.hxx>
#include <NCollection_Sequence.hxx>
#include <Precision.hxx>
#include <ShapeUpgrade_RemoveLocations.hxx>
#include <Standard_Failure.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopLoc_Location.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <gp_Ax1.hxx>
#include <gp_Ax3.hxx>
#include <gp_Mat.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>

static std::string Hex(double theValue)
{
  uint64_t aBits;
  std::memcpy(&aBits, &theValue, 8);
  char aBuf[20];
  std::snprintf(aBuf, sizeof(aBuf), "%016llx", (unsigned long long)aBits);
  return aBuf;
}

static std::string Hex(const gp_Pnt& theP)
{
  return Hex(theP.X()) + "," + Hex(theP.Y()) + "," + Hex(theP.Z());
}

static std::string Hex(const gp_Vec& theV)
{
  return Hex(theV.X()) + "," + Hex(theV.Y()) + "," + Hex(theV.Z());
}

static std::string Hex(const gp_Mat& theM)
{
  std::string aS;
  for (int i = 1; i <= 3; i++)
    for (int j = 1; j <= 3; j++)
      aS += (aS.empty() ? "" : ",") + Hex(theM(i, j));
  return aS;
}

static std::string Hex(const gp_Trsf& theT)
{
  std::string aS;
  for (int i = 1; i <= 3; i++)
    for (int j = 1; j <= 4; j++)
      aS += (aS.empty() ? "" : ",") + Hex(theT.Value(i, j));
  return aS;
}

static std::string Int(int theValue)
{
  return std::to_string(theValue);
}

//! The generator of tools/sweeprec: a 64-bit LCG; U() has 32 significant bits.
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

//! The BRep text of a shape with the Free flag (the first of the seven flag digits on their own line) masked.
static void Text(const std::string& theTag, const TopoDS_Shape& theShape)
{
  std::ostringstream aStream;
  BRepTools::Write(theShape, aStream);
  std::string aText = aStream.str();
  Line(theTag);
  std::istringstream aLines(aText);
  std::string        aLine;
  while (std::getline(aLines, aLine))
  {
    if (!aLine.empty() && aLine.back() == '\r')
      aLine.pop_back();
    bool aFlags = aLine.size() == 7;
    for (char c : aLine)
      aFlags = aFlags && (c == '0' || c == '1');
    if (aFlags)
      aLine[0] = 'x';
    Line(" " + aLine);
  }
}

template <typename F>
static void Guard(F theBody)
{
  try
  {
    theBody();
  }
  catch (const Standard_Failure& anExc)
  {
    std::string aName = typeid(anExc).name();
    if (aName.rfind("class ", 0) == 0)
    {
      aName = aName.substr(6);
    }
    Line("EXC " + aName);
  }
}

//======================================================================================================
// Inputs
//======================================================================================================

static gp_Pnt Pnt(Rng& r, double theScale)
{
  double x = r.R(-theScale, theScale);
  double y = r.R(-theScale, theScale);
  double z = r.R(-theScale, theScale);
  return gp_Pnt(x, y, z);
}

static gp_Dir Dir(Rng& r)
{
  for (;;)
  {
    double x = r.R(-1, 1);
    double y = r.R(-1, 1);
    double z = r.R(-1, 1);
    if (x * x + y * y + z * z > 0.01)
      return gp_Dir(x, y, z);
  }
}

static gp_Trsf Trsf(Rng& r, double theScale)
{
  gp_Trsf aRot;
  double  anAngle = r.R(-3, 3);
  aRot.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), Dir(r)), anAngle);
  gp_Trsf aTr;
  gp_Pnt  aP = Pnt(r, theScale);
  aTr.SetTranslation(gp_Vec(aP.XYZ()));
  return aTr * aRot;
}

//! A cubic B-spline from P1 to P2 through two random interior poles.
static occ::handle<Geom_BSplineCurve> Cubic(Rng& r, const gp_Pnt& P1, const gp_Pnt& P2, double theWiggle)
{
  NCollection_Array1<gp_Pnt> aPoles(1, 4);
  aPoles(1) = P1;
  aPoles(4) = P2;
  for (int i = 2; i <= 3; i++)
  {
    double t  = (i - 1) / 3.0;
    gp_Pnt aB = P1.XYZ() * (1 - t) + P2.XYZ() * t;
    double dx = r.R(-theWiggle, theWiggle);
    double dy = r.R(-theWiggle, theWiggle);
    double dz = r.R(-theWiggle, theWiggle);
    aPoles(i) = gp_Pnt(aB.X() + dx, aB.Y() + dy, aB.Z() + dz);
  }
  NCollection_Array1<double> aK(1, 2);
  aK(1) = 0;
  aK(2) = 1;
  NCollection_Array1<int> aM(1, 2);
  aM(1) = 4;
  aM(2) = 4;
  return new Geom_BSplineCurve(aPoles, aK, aM, 3);
}

//! A wire: theKind 0 polyline/arcs/cubics, open; 1 the same, closed; 2 one cubic B-spline cut into edges (G1);
//! edges are reversed at random.
static TopoDS_Wire MakeWire(Rng& r, int theKind, double theScale)
{
  BRepBuilderAPI_MakeWire aMW;
  if (theKind == 2)
  {
    int                        aNb = r.I(5, 8);
    NCollection_Array1<gp_Pnt> aPoles(1, aNb);
    for (int i = 1; i <= aNb; i++)
    {
      double x  = theScale * (i - 1) + r.R(-1, 1);
      double y  = r.R(-theScale, theScale) * 0.5;
      double z  = r.R(-theScale, theScale) * 0.5;
      aPoles(i) = gp_Pnt(x, y, z);
    }
    int                        aNbK = aNb - 2;
    NCollection_Array1<double> aK(1, aNbK);
    NCollection_Array1<int>    aM(1, aNbK);
    for (int i = 1; i <= aNbK; i++)
    {
      aK(i) = i - 1;
      aM(i) = 1;
    }
    aM(1)    = 4;
    aM(aNbK) = 4;
    occ::handle<Geom_BSplineCurve> aC  = new Geom_BSplineCurve(aPoles, aK, aM, 3);
    int                            aNE = r.I(1, 4);
    double                         aL  = aK(aNbK);
    std::vector<double>            aT;
    aT.push_back(0);
    for (int i = 1; i < aNE; i++)
      aT.push_back(aL * i / aNE + r.R(-0.1, 0.1));
    aT.push_back(aL);
    std::vector<TopoDS_Vertex> aV;
    for (double t : aT)
      aV.push_back(BRepBuilderAPI_MakeVertex(aC->Value(t)).Vertex());
    for (int i = 0; i < aNE; i++)
    {
      TopoDS_Edge anE = BRepBuilderAPI_MakeEdge(aC, aV[i], aV[i + 1], aT[i], aT[i + 1]).Edge();
      aMW.Add(anE);
    }
    return aMW.Wire();
  }
  bool                       aClosed = theKind == 1;
  int                        aNE     = aClosed ? r.I(3, 5) : r.I(1, 4);
  std::vector<gp_Pnt>        aP;
  std::vector<TopoDS_Vertex> aV;
  for (int i = 0; i < aNE; i++)
  {
    gp_Pnt aQ = Pnt(r, theScale);
    aP.push_back(gp_Pnt(aQ.X() + 2 * theScale * i, aQ.Y(), aQ.Z()));
  }
  if (!aClosed)
  {
    gp_Pnt aQ = Pnt(r, theScale);
    aP.push_back(gp_Pnt(aQ.X() + 2 * theScale * aNE, aQ.Y(), aQ.Z()));
  }
  for (const gp_Pnt& aQ : aP)
    aV.push_back(BRepBuilderAPI_MakeVertex(aQ).Vertex());
  for (int i = 0; i < aNE; i++)
  {
    int  j    = (i + 1) % (int)aP.size();
    int  aTyp = r.I(0, 2);
    bool aRev = r.I(0, 3) == 0;
    int  a = aRev ? j : i, b = aRev ? i : j;
    occ::handle<Geom_Curve> aC;
    if (aTyp == 1)
    {
      gp_Pnt aM = aP[a].XYZ() * 0.5 + aP[b].XYZ() * 0.5;
      gp_Pnt aQ = Pnt(r, theScale * 0.5);
      aC        = GC_MakeArcOfCircle(aP[a], gp_Pnt(aM.X() + aQ.X(), aM.Y() + aQ.Y(), aM.Z() + aQ.Z()), aP[b]).Value();
    }
    else if (aTyp == 2)
    {
      aC = Cubic(r, aP[a], aP[b], theScale * 0.5);
    }
    TopoDS_Edge anE;
    if (aC.IsNull())
      anE = BRepBuilderAPI_MakeEdge(aV[a], aV[b]).Edge();
    else
      anE = BRepBuilderAPI_MakeEdge(aC, aV[a], aV[b]).Edge();
    if (aRev)
      anE.Reverse();
    aMW.Add(anE);
  }
  return aMW.Wire();
}

//! A wire of a random kind in [0, theMaxKind] and a random scale in [theS0, theS1] (drawn in that order).
static TopoDS_Wire RandomWire(Rng& r, int theMaxKind, double theS0, double theS1)
{
  int    aKind  = r.I(0, theMaxKind);
  double aScale = r.R(theS0, theS1);
  return MakeWire(r, aKind, aScale);
}

static TopoDS_Shape Box(Rng& r)
{
  double dx = r.R(1, 3);
  double dy = r.R(1, 3);
  double dz = r.R(1, 3);
  return BRepPrimAPI_MakeBox(dx, dy, dz).Shape();
}

static TopoDS_Shape Cylinder(Rng& r)
{
  double aR = r.R(1, 3);
  double aH = r.R(1, 3);
  return BRepPrimAPI_MakeCylinder(aR, aH).Shape();
}

static occ::handle<GeomFill_LocationLaw> MakeTrihedron(Rng& r, int theKind)
{
  occ::handle<GeomFill_TrihedronLaw> aT;
  switch (theKind)
  {
    case 0:
      aT = new GeomFill_Frenet();
      break;
    case 1:
      aT = new GeomFill_CorrectedFrenet();
      break;
    case 2: {
      gp_Dir aD = Dir(r);
      gp_Dir aN = gp_Ax3(gp_Pnt(0, 0, 0), aD).XDirection();
      aT        = new GeomFill_Fixed(gp_Vec(aD), gp_Vec(aN));
      break;
    }
    default:
      aT = new GeomFill_ConstantBiNormal(Dir(r));
      break;
  }
  return new GeomFill_CurveAndTrihedron(aT);
}

//======================================================================================================
// Outputs
//======================================================================================================

static void DumpLocationLaw(Rng& r, const occ::handle<BRepFill_LocationLaw>& theLaw)
{
  int N = theLaw->NbLaw();
  Line("LAW " + Int(N) + " " + Int(theLaw->IsClosed()) + " " + Int((int)theLaw->GetStatus()));
  // the abscissa before the lengths are known (OCCT's bug)
  Guard([&]() {
    double f, l;
    theLaw->Law(N)->GetDomain(f, l);
    Line("ABS0 " + Hex(theLaw->Abscissa(N, 0.5 * (f + l))));
  });
  for (int i = 0; i <= N; i++)
  {
    Guard([&]() {
      int aG1  = theLaw->IsG1(i);
      int aG1b = theLaw->IsG1(i, 1.e-4, 1.e-2);
      Line("G1 " + Int(i) + " " + Int(aG1) + " " + Int(aG1b));
    });
  }
  int NH = theLaw->NbHoles();
  Line("HOLES " + Int(NH));
  if (NH > 0)
  {
    NCollection_Array1<int> aH(1, NH);
    theLaw->Holes(aH);
    std::string aS = "H";
    for (int i = 1; i <= NH; i++)
      aS += " " + Int(aH(i));
    Line(aS);
  }
  double aTotal = 0;
  for (int i = 1; i <= N; i++)
  {
    double f, l;
    theLaw->CurvilinearBounds(i, f, l);
    aTotal = l;
    Line("CB " + Hex(f) + " " + Hex(l));
    Line("V " + Hex(BRep_Tool::Pnt(theLaw->Vertex(i))));
  }
  Line("V " + Hex(BRep_Tool::Pnt(theLaw->Vertex(N + 1))));
  for (int k = 0; k <= 6; k++)
  {
    double s     = k == 6 ? aTotal * 1.1 : (k == 5 ? r.R(0, aTotal) : aTotal * k / 4.0);
    int    anInd = 0;
    double U     = 0;
    theLaw->Parameter(s, anInd, U);
    Line("PAR " + Int(anInd) + " " + Hex(U));
    if (anInd > 0)
      Line("ABS " + Hex(theLaw->Abscissa(anInd, U)));
    TopoDS_Shape aV = BRepBuilderAPI_MakeVertex(Pnt(r, 1)).Vertex();
    Guard([&]() {
      theLaw->D0(s, aV);
      Line(aV.IsNull() ? std::string("D0 null") : "D0 " + Hex(BRep_Tool::Pnt(TopoDS::Vertex(aV))));
    });
  }
  for (int i = 1; i <= N; i++)
  {
    double f, l;
    theLaw->Law(i)->GetDomain(f, l);
    gp_Mat M;
    gp_Vec V;
    theLaw->Law(i)->D0(0.5 * (f + l), M, V);
    Line("LD0 " + Hex(M) + " " + Hex(V));
  }
  TopoDS_Vertex anIn = BRepBuilderAPI_MakeVertex(Pnt(r, 1)).Vertex();
  auto          aPerform = [&](const char* theTag) {
    for (int i = 0; i <= N; i++)
    {
      for (int iloc = -1; iloc <= 1; iloc++)
      {
        Guard([&]() {
          TopoDS_Vertex anOut;
          theLaw->PerformVertex(i, anIn, 1.e-7, anOut, iloc);
          Line(std::string(theTag) + " " + Hex(BRep_Tool::Pnt(anOut)) + " " + Hex(BRep_Tool::Tolerance(anOut)));
        });
      }
    }
  };
  aPerform("PV");
  Guard([&]() {
    theLaw->TransformInG0Law();
    aPerform("PVG0");
  });
  Guard([&]() {
    theLaw->DeleteTransform();
    theLaw->TransformInCompatibleLaw(0.01);
    aPerform("PVC");
  });
  theLaw->DeleteTransform();
  Line("HOLES2 " + Int(theLaw->NbHoles(1.e-4)));
}

static void DumpSectionLaw(const occ::handle<BRepFill_SectionLaw>& theLaw, const TopoDS_Wire& theWire)
{
  Line("SEC " + Int(theLaw->NbLaw()) + " " + Int(theLaw->IsUClosed()) + " " + Int(theLaw->IsVClosed()) + " "
       + Int(theLaw->IsConstant()) + " " + Int(theLaw->IsVertex()) + " " + Int(theLaw->IsDone()));
  if (!theLaw->IsDone())
    return;
  int    N = theLaw->NbLaw();
  double VF = 0, VL = 1;
  for (int i = 1; i <= N; i++)
  {
    const occ::handle<GeomFill_SectionLaw>& aL = theLaw->Law(i);
    if (aL.IsNull())
    {
      Line("NULLLAW");
      continue;
    }
    aL->GetDomain(VF, VL);
    int NbP, NbK, Deg;
    aL->SectionShape(NbP, NbK, Deg);
    Line("SS " + Int(NbP) + " " + Int(NbK) + " " + Int(Deg) + " " + Int(aL->IsRational()) + " "
         + Int(aL->IsUPeriodic()) + " " + Hex(VF) + " " + Hex(VL));
    NCollection_Array1<double> aK(1, NbK);
    aL->Knots(aK);
    NCollection_Array1<int> aM(1, NbK);
    aL->Mults(aM);
    std::string aS = "K";
    for (int k = 1; k <= NbK; k++)
      aS += " " + Hex(aK(k)) + "/" + Int(aM(k));
    Line(aS);
    for (int k = 0; k <= 2; k++)
    {
      NCollection_Array1<gp_Pnt> aP(1, NbP);
      NCollection_Array1<double> aW(1, NbP);
      bool                       aOk = aL->D0(VF + (VL - VF) * k / 2.0, aP, aW);
      std::string                aT  = "D0 " + Int(aOk);
      for (int j = 1; j <= NbP; j++)
        aT += " " + Hex(aP(j)) + "," + Hex(aW(j));
      Line(aT);
    }
  }
  double aMid = 0.5 * (VF + VL);
  if (!theLaw->IsVertex())
  {
    for (int i = 0; i <= N; i++)
    {
      Guard([&]() { Line("VT " + Hex(theLaw->VertexTol(i, aMid))); });
      Guard([&]() { Line("CONT " + Int((int)theLaw->Continuity(i, 1.e-4))); });
    }
    for (int i = 1; i <= N + 1; i++)
    {
      Guard([&]() { Line("VX " + Hex(BRep_Tool::Pnt(theLaw->Vertex(i, aMid)))); });
    }
    for (TopExp_Explorer anExp(theWire, TopAbs_EDGE); anExp.More(); anExp.Next())
    {
      Guard([&]() { Line("IDX " + Int(theLaw->IndexOfEdge(anExp.Current()))); });
    }
  }
  Guard([&]() {
    occ::handle<GeomFill_SectionLaw> aC = theLaw->ConcatenedLaw();
    if (aC.IsNull())
    {
      Line("CONCAT null");
      return;
    }
    int NbP, NbK, Deg;
    aC->SectionShape(NbP, NbK, Deg);
    double f, l;
    aC->GetDomain(f, l);
    NCollection_Array1<gp_Pnt> aP(1, NbP);
    NCollection_Array1<double> aW(1, NbP);
    aC->D0(0.5 * (f + l), aP, aW);
    std::string aT = "CONCAT " + Int(NbP) + " " + Int(NbK) + " " + Int(Deg);
    for (int j = 1; j <= NbP; j++)
      aT += " " + Hex(aP(j)) + "," + Hex(aW(j));
    Line(aT);
  });
  Guard([&]() {
    TopoDS_Shape aS;
    theLaw->D0(aMid, aS);
    Text("SD0", aS);
  });
}

//======================================================================================================
// Cases
//======================================================================================================

static void Edge3D(Rng& r)
{
  TopoDS_Wire aW    = RandomWire(r, 2, 1, 5);
  int         aKind = r.I(0, 3);
  Line("TRI " + Int(aKind));
  occ::handle<GeomFill_LocationLaw> aL = MakeTrihedron(r, aKind);
  Guard([&]() {
    occ::handle<BRepFill_LocationLaw> aLaw = new BRepFill_Edge3DLaw(aW, aL);
    DumpLocationLaw(r, aLaw);
  });
}

static void Guided(Rng& r, bool theACR)
{
  TopoDS_Wire aW = RandomWire(r, 2, 1, 5);
  gp_Trsf     aT;
  double tx = r.R(-1, 1);
  double ty = r.R(2, 4);
  double tz = r.R(-1, 1);
  aT.SetTranslation(gp_Vec(tx, ty, tz));
  TopoDS_Wire aG = TopoDS::Wire(BRepBuilderAPI_Transform(aW, aT, true).Shape());
  Guard([&]() {
    occ::handle<BRepAdaptor_CompCurve> aGuide = new BRepAdaptor_CompCurve(aG);
    occ::handle<BRepFill_LocationLaw>  aLaw;
    if (theACR)
    {
      occ::handle<GeomFill_GuideTrihedronAC> aTLaw = new GeomFill_GuideTrihedronAC(aGuide);
      occ::handle<GeomFill_LocationGuide>    aLoc  = new GeomFill_LocationGuide(aTLaw);
      aLaw                                         = new BRepFill_ACRLaw(aW, aLoc);
    }
    else
    {
      occ::handle<GeomFill_GuideTrihedronPlan> aTLaw = new GeomFill_GuideTrihedronPlan(aGuide);
      occ::handle<GeomFill_LocationGuide>      aLoc  = new GeomFill_LocationGuide(aTLaw);
      aLaw                                           = new BRepFill_Edge3DLaw(aW, aLoc);
    }
    DumpLocationLaw(r, aLaw);
  });
}

static void EdgeOnSurf(Rng& r)
{
  int                       aKind = r.I(0, 2);
  occ::handle<Geom_Surface> aS;
  double                    u0 = 0, u1 = 1, v0 = 0, v1 = 1;
  if (aKind == 0)
  {
    gp_Pnt aO = Pnt(r, 2);
    gp_Dir aD = Dir(r);
    aS        = new Geom_Plane(gp_Ax3(aO, aD));
    u1 = v1 = 10;
  }
  else if (aKind == 1)
  {
    gp_Pnt aO = Pnt(r, 2);
    gp_Dir aD = Dir(r);
    double aR = r.R(2, 5);
    aS        = new Geom_CylindricalSurface(gp_Ax3(aO, aD), aR);
    u1 = 6;
    v1 = 10;
  }
  else
  {
    NCollection_Array2<gp_Pnt> aP(1, 4, 1, 4);
    for (int i = 1; i <= 4; i++)
      for (int j = 1; j <= 4; j++)
      {
        double z = r.R(-2, 2);
        aP(i, j) = gp_Pnt(3.0 * i, 3.0 * j, z);
      }
    NCollection_Array1<double> aK(1, 2);
    aK(1) = 0;
    aK(2) = 1;
    NCollection_Array1<int> aM(1, 2);
    aM(1) = 4;
    aM(2) = 4;
    aS    = new Geom_BSplineSurface(aP, aK, aK, aM, aM, 3, 3);
  }
  TopoDS_Face aF = BRepBuilderAPI_MakeFace(aS, u0, u1, v0, v1, 1.e-7).Face();
  int         aNE = r.I(1, 4);
  std::vector<gp_Pnt2d> aUV;
  for (int i = 0; i <= aNE; i++)
  {
    double u = u0 + (u1 - u0) * (0.1 + 0.8 * i / aNE);
    double v = v0 + (v1 - v0) * r.R(0.1, 0.9);
    aUV.push_back(gp_Pnt2d(u, v));
  }
  std::vector<TopoDS_Vertex> aV;
  for (const gp_Pnt2d& aQ : aUV)
    aV.push_back(BRepBuilderAPI_MakeVertex(aS->Value(aQ.X(), aQ.Y())).Vertex());
  BRepBuilderAPI_MakeWire aMW;
  bool                    aOff = r.I(0, 4) == 0;
  for (int i = 0; i < aNE; i++)
  {
    gp_Vec2d                 aD(aUV[i], aUV[i + 1]);
    occ::handle<Geom2d_Line> aL = new Geom2d_Line(aUV[i], gp_Dir2d(aD));
    TopoDS_Edge              anE;
    if (aOff && i == aNE - 1)
      anE = BRepBuilderAPI_MakeEdge(aV[i], aV[i + 1]).Edge();
    else
    {
      anE = BRepBuilderAPI_MakeEdge(aL, aS, aV[i], aV[i + 1], 0, aD.Magnitude()).Edge();
      BRepLib::BuildCurves3d(anE);
    }
    aMW.Add(anE);
  }
  TopoDS_Wire aW = aMW.Wire();
  Guard([&]() {
    occ::handle<BRepFill_EdgeOnSurfLaw> aLaw = new BRepFill_EdgeOnSurfLaw(aW, aF);
    Line("HAS " + Int(aLaw->HasResult()));
    if (aLaw->HasResult())
      DumpLocationLaw(r, aLaw);
  });
}

static void ShapeLaw(Rng& r)
{
  int aKind = r.I(0, 3);
  Line("KIND " + Int(aKind));
  if (aKind == 0)
  {
    TopoDS_Vertex aV = BRepBuilderAPI_MakeVertex(Pnt(r, 5)).Vertex();
    Guard([&]() { DumpSectionLaw(new BRepFill_ShapeLaw(aV), TopoDS_Wire()); });
    return;
  }
  TopoDS_Wire aW = RandomWire(r, 2, 0.5, 2);
  if (aKind == 1)
  {
    Guard([&]() { DumpSectionLaw(new BRepFill_ShapeLaw(aW), aW); });
  }
  else
  {
    occ::handle<Law_Linear> aL = new Law_Linear();
    double                  a  = r.R(0.5, 2);
    double                  b  = r.R(0.5, 2);
    aL->Set(0, a, 1, b);
    Guard([&]() { DumpSectionLaw(new BRepFill_ShapeLaw(aW, aL), aW); });
  }
}

static TopoDS_Wire PunctualWire(const gp_Pnt& theP)
{
  TopoDS_Vertex aV = BRepBuilderAPI_MakeVertex(theP).Vertex();
  BRep_Builder  BB;
  TopoDS_Edge   DegEdge;
  BB.MakeEdge(DegEdge);
  BB.Add(DegEdge, aV.Oriented(TopAbs_FORWARD));
  BB.Add(DegEdge, aV.Oriented(TopAbs_REVERSED));
  BB.Degenerated(DegEdge, true);
  TopoDS_Wire aW;
  BB.MakeWire(aW);
  BB.Add(aW, DegEdge);
  aW.Closed(true);
  return aW;
}

static void NSections(Rng& r)
{
  TopoDS_Wire                        aW0 = RandomWire(r, 1, 0.5, 2);
  int                                aNb = r.I(2, 4);
  NCollection_Sequence<TopoDS_Shape> aS;
  NCollection_Sequence<gp_Trsf>      aT;
  NCollection_Sequence<double>       aP;
  bool                               aFirstPoint = r.I(0, 3) == 0;
  bool                               aLastPoint  = r.I(0, 3) == 0;
  bool                               aLoop       = !aFirstPoint && !aLastPoint && r.I(0, 4) == 0;
  double                             aPar        = 0;
  for (int i = 1; i <= aNb; i++)
  {
    gp_Trsf aTr;
    double tx = r.R(-1, 1);
    double ty = r.R(-1, 1);
    aTr.SetTranslation(gp_Vec(tx, ty, 5.0 * i));
    gp_Trsf aRot;
    aRot.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)), r.R(-0.5, 0.5));
    aTr = aTr * aRot;
    if ((i == 1 && aFirstPoint) || (i == aNb && aLastPoint))
    {
      double px = r.R(-1, 1);
      double py = r.R(-1, 1);
      aS.Append(PunctualWire(gp_Pnt(px, py, 5.0 * i)));
    }
    else if (i == aNb && aLoop)
      aS.Append(aS(1));
    else
      aS.Append(BRepBuilderAPI_Transform(aW0, aTr, true).Shape());
    aT.Append(aTr);
    aP.Append(aPar);
    aPar += r.R(0.5, 2);
  }
  TopoDS_Wire aRef = TopoDS::Wire(aS(aFirstPoint ? 2 : 1));
  if (r.I(0, 1) == 0)
  {
    Line("NS1");
    Guard([&]() { DumpSectionLaw(new BRepFill_NSections(aS), aRef); });
  }
  else
  {
    Line("NS2");
    Guard([&]() { DumpSectionLaw(new BRepFill_NSections(aS, aT, aP, aP(1), aP(aNb)), aRef); });
  }
}

static TopoDS_Shape Located(Rng& r, const TopoDS_Shape& theShape)
{
  return theShape.Moved(TopLoc_Location(Trsf(r, 3)));
}

static void RemoveLoc(Rng& r)
{
  int          aKind = r.I(0, 3);
  TopoDS_Shape aShape;
  if (aKind == 0)
    aShape = Located(r, Box(r));
  else if (aKind == 1)
    aShape = Located(r, Cylinder(r));
  else if (aKind == 2)
    aShape = Located(r, RandomWire(r, 2, 2, 2));
  else
  {
    BRep_Builder    B;
    TopoDS_Compound C;
    B.MakeCompound(C);
    B.Add(C, Located(r, Box(r)));
    B.Add(C, Located(r, Cylinder(r)));
    B.Add(C, MakeWire(r, 0, 2));
    aShape = Located(r, C);
  }
  int              aLevel = r.I(0, 4);
  TopAbs_ShapeEnum aL     = aLevel == 0 ? TopAbs_SHAPE : (TopAbs_ShapeEnum)(aLevel - 1);
  Line("LEVEL " + Int(aLevel));
  Guard([&]() {
    ShapeUpgrade_RemoveLocations aRL;
    aRL.SetRemoveLevel(aL);
    // one call per statement: MSVC evaluates the operands of a chain of + in no fixed order
    bool aDone  = aRL.Remove(aShape);
    int  aLevel2 = (int)aRL.RemoveLevel();
    int  aNbMod = aRL.GetModifiedShapesMap().Extent();
    Line("REMOVE " + Int(aDone) + " " + Int(aLevel2) + " " + Int(aNbMod));
    Text("RESULT", aRL.GetResult());
  });
}

static void Section(Rng& r)
{
  int          aKind = r.I(0, 1);
  TopoDS_Shape aProfile;
  if (aKind == 0)
    aProfile = Located(r, BRepBuilderAPI_MakeVertex(Pnt(r, 2)).Vertex());
  else
    aProfile = Located(r, RandomWire(r, 2, 1, 1));
  TopoDS_Vertex aV = BRepBuilderAPI_MakeVertex(Pnt(r, 2)).Vertex();
  Guard([&]() {
    bool aContact    = r.I(0, 1) == 1;
    bool aCorrection = r.I(0, 1) == 1;
    BRepFill_Section aSec(aProfile, aV, aContact, aCorrection);
    Line("SECT " + Int(aSec.IsPunctual()) + " " + Int(aSec.WithContact()) + " " + Int(aSec.WithCorrection()));
    Text("WIRE", aSec.Wire());
    TopTools_IndexedMapOfShape aMap;
    TopExp::MapShapes(aSec.Wire(), aMap);
    for (TopExp_Explorer anExp(aProfile, TopAbs_EDGE); anExp.More(); anExp.Next())
      Line("MOD " + Int(aMap.FindIndex(aSec.ModifiedShape(anExp.Current()))));
    for (TopExp_Explorer anExp(aProfile, TopAbs_VERTEX); anExp.More(); anExp.Next())
      Line("MOD " + Int(aMap.FindIndex(aSec.ModifiedShape(anExp.Current()))));
    Line("MOD " + Int(aMap.FindIndex(aSec.ModifiedShape(aProfile))));
  });
}

static void Placement(Rng& r)
{
  TopoDS_Wire aW    = RandomWire(r, 2, 1, 5);
  int         aKind = r.I(0, 1);
  occ::handle<GeomFill_LocationLaw> aL = MakeTrihedron(r, aKind);
  int                               aSecKind = r.I(0, 1);
  // a section near a point of the spine
  BRepAdaptor_CompCurve aC(aW);
  double                t  = r.R(aC.FirstParameter(), aC.LastParameter());
  gp_Pnt                aQ = aC.Value(t);
  double                ox = r.R(-0.5, 0.5);
  double                oy = r.R(-0.5, 0.5);
  double                oz = r.R(-0.5, 0.5);
  gp_Pnt                aO(aQ.X() + ox, aQ.Y() + oy, aQ.Z() + oz);
  TopoDS_Shape          aSec;
  if (aSecKind == 0)
    aSec = BRepBuilderAPI_MakeVertex(aO).Vertex();
  else
  {
    gp_Trsf aT;
    aT.SetTranslation(gp_Vec(aO.XYZ()));
    aSec = BRepBuilderAPI_Transform(RandomWire(r, 1, 0.3, 0.3), aT, true).Shape();
  }
  bool aContact    = r.I(0, 1) == 1;
  bool aCorrection = r.I(0, 1) == 1;
  int  aOnVertex   = r.I(0, 2);
  Line("PL " + Int(aKind) + " " + Int(aSecKind) + " " + Int(aContact) + " " + Int(aCorrection) + " " + Int(aOnVertex));
  Guard([&]() {
    occ::handle<BRepFill_LocationLaw> aLaw = new BRepFill_Edge3DLaw(aW, aL);
    if (aOnVertex == 0)
    {
      BRepFill_SectionPlacement aP(aLaw, aSec, aContact, aCorrection);
      Line("TRSF " + Hex(aP.Transformation()));
      Line("ABSC " + Hex(aP.AbscissaOnPath()));
    }
    else
    {
      int           anIdx = r.I(1, aLaw->NbLaw() + 1);
      TopoDS_Vertex aV    = aLaw->Vertex(anIdx);
      BRepFill_SectionPlacement aP(aLaw, aSec, aV, aContact, aCorrection);
      Line("TRSF " + Hex(aP.Transformation()));
      Line("ABSC " + Hex(aP.AbscissaOnPath()));
    }
  });
}

int main()
{
  struct Kind
  {
    const char* Name;
    int         Count;
  };
  const Kind theKinds[] = {{"edge3d", 80},
                           {"acr", 20},
                           {"guideplan", 20},
                           {"edgeonsurf", 40},
                           {"shapelaw", 80},
                           {"nsections", 40},
                           {"removeloc", 60},
                           {"section", 40},
                           {"placement", 80}};
  int aKindIndex = 0;
  for (const Kind& aKind : theKinds)
  {
    for (int aSeed = 0; aSeed < aKind.Count; aSeed++)
    {
      Rng r((uint64_t)(aKindIndex * 1000 + aSeed));
      Line(std::string("CASE ") + aKind.Name + " " + Int(aSeed));
      std::string aName = aKind.Name;
      Guard([&]() {
        if (aName == "edge3d")
          Edge3D(r);
        else if (aName == "acr")
          Guided(r, true);
        else if (aName == "guideplan")
          Guided(r, false);
        else if (aName == "edgeonsurf")
          EdgeOnSurf(r);
        else if (aName == "shapelaw")
          ShapeLaw(r);
        else if (aName == "nsections")
          NSections(r);
        else if (aName == "removeloc")
          RemoveLoc(r);
        else if (aName == "section")
          Section(r);
        else if (aName == "placement")
          Placement(r);
      });
      Flush();
    }
    aKindIndex++;
  }
  return 0;
}
