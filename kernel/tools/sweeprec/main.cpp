// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
//
// sweeprec: records native OCCT's sweep surfaces (unit 5L2): GeomFill_Pipe in all its constructions (constant radius,
// constant section with each trihedron, fixed binormal, Darboux, two and N sections, guides, pipes on edges by
// AppSweep and by CircularBlendFunc, the particular cases), GeomFill_Sweep with EvolvedSection, the NSections law,
// CircularBlendFunc, SweepSectionGenerator, the GeomFill package functions (GetCircle and its convertors, GetShape,
// GetTolerance, GetMinimalWeights, Knots, Mults, Surface). Every double is written as its 64 bits in hex.
// Built against build123d.NET's native OCCT V8_0_1 (tools/sweeprec/build.ps1).
//
// The inputs are built from a small deterministic generator (an LCG giving doubles with 32 significant bits), which
// tests/OcctNet.ModelingAlgorithms.Tests/GeomFill/SweepRecordingCases.cs repeats line for line, so the recording holds
// only the results. Each case starts with "CASE <name>" and a line "IN ..." fingerprinting its inputs.
//
//   sweeprec > sweeprec.txt
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cmath>
#include <string>
#include <typeinfo>
#include <vector>

#include <Adaptor3d_Curve.hxx>
#include <Geom2dAdaptor_Curve.hxx>
#include <Geom2d_BSplineCurve.hxx>
#include <Geom2d_Curve.hxx>
#include <Geom2d_Line.hxx>
#include <Geom2d_TrimmedCurve.hxx>
#include <GeomAdaptor_Curve.hxx>
#include <GeomFill.hxx>
#include <GeomFill_CircularBlendFunc.hxx>
#include <GeomFill_CorrectedFrenet.hxx>
#include <GeomFill_CurveAndTrihedron.hxx>
#include <GeomFill_EvolvedSection.hxx>
#include <GeomFill_NSections.hxx>
#include <GeomFill_Pipe.hxx>
#include <GeomFill_Sweep.hxx>
#include <GeomFill_SweepSectionGenerator.hxx>
#include <GeomFill_UniformSection.hxx>
#include <Geom_BSplineCurve.hxx>
#include <Geom_BSplineSurface.hxx>
#include <Geom_Circle.hxx>
#include <Geom_CylindricalSurface.hxx>
#include <Geom_Ellipse.hxx>
#include <Geom_Line.hxx>
#include <Geom_Plane.hxx>
#include <Geom_RectangularTrimmedSurface.hxx>
#include <Geom_Surface.hxx>
#include <Geom_TrimmedCurve.hxx>
#include <Law_Linear.hxx>
#include <Law_S.hxx>
#include <NCollection_Array1.hxx>
#include <NCollection_Sequence.hxx>
#include <Precision.hxx>
#include <Standard_Failure.hxx>
#include <gp_Ax2.hxx>
#include <gp_Circ.hxx>
#include <gp_Dir.hxx>
#include <gp_Elips.hxx>
#include <gp_Pnt.hxx>
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

static std::string Hex(const gp_Pnt2d& theP)
{
  return Hex(theP.X()) + "," + Hex(theP.Y());
}

//! The FNV-1a hash of a text, in hex.
static std::string Fnv(const std::string& theText)
{
  uint64_t aHash = 14695981039346656037ULL;
  for (unsigned char aChar : theText)
  {
    aHash ^= aChar;
    aHash *= 1099511628211ULL;
  }
  char aBuf[20];
  std::snprintf(aBuf, sizeof(aBuf), "%016llx", (unsigned long long)aHash);
  return aBuf;
}

static std::string Int(int theValue)
{
  return std::to_string(theValue);
}

//! The generator: a 64-bit LCG; U() has 32 significant bits, so every value is exact in both languages.
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

//======================================================================================================
// Inputs
//======================================================================================================

//! A path: a cubic (or quartic) B-spline running along X with bounded wiggles, optionally rational.
static occ::handle<Geom_BSplineCurve> MakePath(Rng& r, bool theRational, double theWiggle)
{
  int                        aDeg = r.I(2, 4);
  int                        aNb  = r.I(aDeg + 1, aDeg + 4);
  NCollection_Array1<gp_Pnt> aPoles(1, aNb);
  NCollection_Array1<double> aW(1, aNb);
  for (int i = 1; i <= aNb; i++)
  {
    double x = 10.0 * (i - 1) + r.R(-2, 2);
    double y = r.R(-theWiggle, theWiggle);
    double z = r.R(-theWiggle, theWiggle);
    aPoles(i) = gp_Pnt(x, y, z);
    aW(i)     = theRational ? r.R(0.7, 1.4) : 1.0;
  }
  int                        aNbK = aNb - aDeg + 1;
  NCollection_Array1<double> aK(1, aNbK);
  NCollection_Array1<int>    aM(1, aNbK);
  for (int i = 1; i <= aNbK; i++)
  {
    aK(i) = (double)(i - 1);
    aM(i) = 1;
  }
  aM(1)    = aDeg + 1;
  aM(aNbK) = aDeg + 1;
  if (theRational)
  {
    return new Geom_BSplineCurve(aPoles, aW, aK, aM, aDeg);
  }
  return new Geom_BSplineCurve(aPoles, aK, aM, aDeg);
}

//! A section near P in the plane normal to T: 0 circle, 1 ellipse, 2 segment, 3 open B-spline, 4 periodic B-spline.
static occ::handle<Geom_Curve> MakeSection(Rng& r, int theKind, const gp_Pnt& P, const gp_Vec& T, double theScale)
{
  gp_Ax2 anAx(P, gp_Dir(T));
  switch (theKind)
  {
    case 0:
      return new Geom_Circle(anAx, theScale * r.R(0.5, 1.5));
    case 1: {
      double aMaj = theScale * r.R(1.0, 2.0);
      return new Geom_Ellipse(anAx, aMaj, aMaj * r.R(0.3, 0.9));
    }
    case 2: {
      gp_Vec aX(anAx.XDirection());
      gp_Pnt aP1 = P.Translated(aX * (-theScale * r.R(0.5, 1.5)));
      gp_Vec aD  = aX * theScale * r.R(1.0, 2.5);
      return new Geom_TrimmedCurve(new Geom_Line(aP1, gp_Dir(aD)), 0.0, aD.Magnitude());
    }
    default: {
      bool                       aPer = theKind == 4;
      int                        aNb  = r.I(5, 8);
      NCollection_Array1<gp_Pnt> aPoles(1, aNb);
      gp_Vec                     aX(anAx.XDirection()), aY(anAx.YDirection());
      for (int i = 1; i <= aNb; i++)
      {
        double anA = 2.0 * M_PI * (i - 1) / (aPer ? aNb : aNb - 1) * (aPer ? 1.0 : 0.6);
        double aR  = theScale * r.R(0.7, 1.3);
        aPoles(i)  = P.Translated(aX * (aR * std::cos(anA)) + aY * (aR * std::sin(anA)));
      }
      int aDeg = 3;
      if (aPer)
      {
        NCollection_Array1<double> aK(1, aNb + 1);
        NCollection_Array1<int>    aM(1, aNb + 1);
        for (int i = 1; i <= aNb + 1; i++)
        {
          aK(i) = (double)(i - 1);
          aM(i) = 1;
        }
        return new Geom_BSplineCurve(aPoles, aK, aM, aDeg, true);
      }
      int                        aNbK = aNb - aDeg + 1;
      NCollection_Array1<double> aK(1, aNbK);
      NCollection_Array1<int>    aM(1, aNbK);
      for (int i = 1; i <= aNbK; i++)
      {
        aK(i) = (double)(i - 1);
        aM(i) = 1;
      }
      aM(1)    = aDeg + 1;
      aM(aNbK) = aDeg + 1;
      return new Geom_BSplineCurve(aPoles, aK, aM, aDeg);
    }
  }
}

//======================================================================================================
// Outputs
//======================================================================================================

static double Clamp(double theV)
{
  return theV < -10.0 ? -10.0 : (theV > 10.0 ? 10.0 : theV);
}

static void DumpSurface(const occ::handle<Geom_Surface>& theS)
{
  if (theS.IsNull())
  {
    Line("SURF null");
    return;
  }
  double u1, u2, v1, v2;
  theS->Bounds(u1, u2, v1, v2);
  Line(std::string("SURF ") + theS->DynamicType()->Name() + " " + Hex(u1) + " " + Hex(u2) + " " + Hex(v1) + " "
       + Hex(v2));
  occ::handle<Geom_Surface> aBasis = theS;
  if (theS->IsKind(STANDARD_TYPE(Geom_RectangularTrimmedSurface)))
  {
    aBasis = occ::down_cast<Geom_RectangularTrimmedSurface>(theS)->BasisSurface();
    Line(std::string("BASIS ") + aBasis->DynamicType()->Name());
  }
  occ::handle<Geom_BSplineSurface> aBS = occ::down_cast<Geom_BSplineSurface>(aBasis);
  if (!aBS.IsNull())
  {
    Line("BS " + Int(aBS->UDegree()) + " " + Int(aBS->VDegree()) + " " + Int(aBS->NbUPoles()) + " "
         + Int(aBS->NbVPoles()) + " " + Int(aBS->NbUKnots()) + " " + Int(aBS->NbVKnots()) + " "
         + Int(aBS->IsUPeriodic()) + " " + Int(aBS->IsVPeriodic()) + " " + Int(aBS->IsURational()) + " "
         + Int(aBS->IsVRational()));
    std::string aK = "UK";
    for (int i = 1; i <= aBS->NbUKnots(); i++)
    {
      aK += " " + Hex(aBS->UKnot(i)) + "x" + Int(aBS->UMultiplicity(i));
    }
    Line(aK);
    aK = "VK";
    for (int i = 1; i <= aBS->NbVKnots(); i++)
    {
      aK += " " + Hex(aBS->VKnot(i)) + "x" + Int(aBS->VMultiplicity(i));
    }
    Line(aK);
    // the poles and weights as the FNV-1a hash of their hex text, with the first and last pole
    std::string aPoles;
    for (int i = 1; i <= aBS->NbUPoles(); i++)
    {
      for (int j = 1; j <= aBS->NbVPoles(); j++)
      {
        aPoles += Hex(aBS->Pole(i, j)) + "," + Hex(aBS->Weight(i, j)) + ";";
      }
    }
    Line("PR " + Fnv(aPoles) + " " + Hex(aBS->Pole(1, 1)) + " " + Hex(aBS->Pole(aBS->NbUPoles(), aBS->NbVPoles())));
  }
  std::string aGrid = "GRID";
  double      a1 = Clamp(u1), a2 = Clamp(u2), b1 = Clamp(v1), b2 = Clamp(v2);
  for (int i = 0; i <= 4; i++)
  {
    for (int j = 0; j <= 4; j++)
    {
      double u = a1 + (a2 - a1) * i / 4.0, v = b1 + (b2 - b1) * j / 4.0;
      aGrid += " " + Hex(theS->Value(u, v));
    }
  }
  Line(aGrid);
}

static void DumpCurve2d(const char* theTag, const occ::handle<Geom2d_Curve>& theC)
{
  if (theC.IsNull())
  {
    Line(std::string(theTag) + " null");
    return;
  }
  double      f = Clamp(theC->FirstParameter()), l = Clamp(theC->LastParameter());
  std::string aLine = std::string(theTag) + " " + theC->DynamicType()->Name() + " "
                      + Hex(theC->FirstParameter()) + " " + Hex(theC->LastParameter());
  for (int i = 0; i <= 4; i++)
  {
    aLine += " " + Hex(theC->Value(f + (l - f) * i / 4.0));
  }
  Line(aLine);
}

static void DumpPipe(GeomFill_Pipe& thePipe)
{
  Line("PIPE " + Int(thePipe.IsDone()) + " " + Int((int)thePipe.GetStatus()) + " " + Hex(thePipe.ErrorOnSurf()) + " "
       + Int(thePipe.ExchangeUV()));
  DumpSurface(thePipe.Surface());
}

static void DumpArrays(const char*                       theTag,
                       const NCollection_Array1<gp_Pnt>& theP,
                       const NCollection_Array1<double>& theW)
{
  std::string aLine = theTag;
  for (int i = theP.Lower(); i <= theP.Upper(); i++)
  {
    aLine += " " + Hex(theP(i)) + "," + Hex(theW(i));
  }
  Line(aLine);
}

static void DumpArrays(const char*                       theTag,
                       const NCollection_Array1<gp_Vec>& theP,
                       const NCollection_Array1<double>& theW)
{
  std::string aLine = theTag;
  for (int i = theP.Lower(); i <= theP.Upper(); i++)
  {
    aLine += " " + Hex(theP(i)) + "," + Hex(theW(i));
  }
  Line(aLine);
}

//! Runs a case body, writing "EXC <type>" if it throws.
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
// Cases
//======================================================================================================

static void PipeRadius(Rng& r)
{
  bool                           aRat  = r.I(0, 3) == 0;
  double                         aWig  = r.R(0.5, 5.0);
  occ::handle<Geom_BSplineCurve> aPath = MakePath(r, aRat, aWig);
  double                         aR    = r.R(0.3, 2.0);
  Line("IN " + Hex(aPath->Pole(1)) + " " + Hex(aR));
  Guard([&] {
    GeomFill_Pipe aPipe(aPath, aR);
    aPipe.GenerateParticularCase(true);
    aPipe.Perform(1.e-4, false, GeomAbs_C2);
    DumpPipe(aPipe);
  });
}

static void PipeSection(Rng& r, GeomFill_Trihedron theOption)
{
  bool                           aRat  = r.I(0, 3) == 0;
  double                         aWig  = r.R(0.5, 5.0);
  occ::handle<Geom_BSplineCurve> aPath = MakePath(r, aRat, aWig);
  int                            aKind = r.I(0, 4);
  gp_Pnt                         aP;
  gp_Vec                         aT;
  aPath->D1(aPath->FirstParameter(), aP, aT);
  double                  sx = r.R(-0.5, 0.5);
  double                  sy = r.R(-0.5, 0.5);
  double                  sz = r.R(-0.5, 0.5);
  gp_Vec                  aShift(sx, sy, sz);
  occ::handle<Geom_Curve> aSect = MakeSection(r, aKind, aP.Translated(aShift), aT, 1.0);
  int                     aDeg  = r.I(0, 2) == 0 ? 8 : 11;
  Line("IN " + Hex(aPath->Pole(1)) + " " + Int(aKind) + " " + Hex(aSect->Value(aSect->FirstParameter())));
  Guard([&] {
    GeomFill_Pipe aPipe;
    aPipe.GenerateParticularCase(true);
    aPipe.Init(aPath, aSect, theOption);
    aPipe.Perform(1.e-4, false, GeomAbs_C2, aDeg, 30);
    DumpPipe(aPipe);
  });
}

static void PipeDir(Rng& r)
{
  occ::handle<Geom_BSplineCurve> aPath = MakePath(r, false, r.R(0.5, 3.0));
  int                            aKind = r.I(0, 4);
  gp_Pnt                         aP;
  gp_Vec                         aT;
  aPath->D1(aPath->FirstParameter(), aP, aT);
  occ::handle<Geom_Curve> aSect = MakeSection(r, aKind, aP, aT, 1.0);
  double                  dx = r.R(-0.3, 0.3);
  double                  dy = r.R(0.5, 1.0);
  double                  dz = r.R(-0.3, 0.3);
  gp_Dir                  aDir(dx, dy, dz);
  Line("IN " + Hex(aPath->Pole(1)) + " " + Int(aKind));
  Guard([&] {
    GeomFill_Pipe aPipe;
    aPipe.GenerateParticularCase(true);
    aPipe.Init(aPath, aSect, aDir);
    aPipe.Perform(1.e-4, false, GeomAbs_C2, 11, 30);
    DumpPipe(aPipe);
  });
}

static void PipeDarboux(Rng& r)
{
  // a 2d path on a plane, a cylinder or a B-spline surface
  int                       aSupKind = r.I(0, 2);
  occ::handle<Geom_Surface> aSup;
  if (aSupKind == 0)
  {
    double dx = r.R(-0.2, 0.2);
    double dy = r.R(-0.2, 0.2);
    aSup      = new Geom_Plane(gp_Ax3(gp_Pnt(0, 0, 0), gp_Dir(dx, dy, 1.0)));
  }
  else if (aSupKind == 1)
  {
    aSup = new Geom_CylindricalSurface(gp_Ax3(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)), r.R(5.0, 10.0));
  }
  else
  {
    NCollection_Array2<gp_Pnt> aPoles(1, 4, 1, 4);
    for (int i = 1; i <= 4; i++)
    {
      for (int j = 1; j <= 4; j++)
      {
        aPoles(i, j) = gp_Pnt(10.0 * (i - 1), 10.0 * (j - 1), r.R(-2.0, 2.0));
      }
    }
    NCollection_Array1<double> aK(1, 2);
    NCollection_Array1<int>    aM(1, 2);
    aK(1) = 0.0;
    aK(2) = 1.0;
    aM(1) = aM(2) = 4;
    aSup          = new Geom_BSplineSurface(aPoles, aK, aK, aM, aM, 3, 3);
  }
  double                       aScaleU = aSupKind == 1 ? 1.0 : (aSupKind == 2 ? 0.1 : 10.0);
  NCollection_Array1<gp_Pnt2d> aP2(1, 4);
  for (int i = 1; i <= 4; i++)
  {
    aP2(i) = gp_Pnt2d((0.1 + 0.25 * (i - 1)) * aScaleU * (aSupKind == 1 ? 2.0 : 1.0),
                      (0.2 + r.R(0.0, 0.6)) * (aSupKind == 1 ? 10.0 : aScaleU));
  }
  NCollection_Array1<double> aK(1, 2);
  NCollection_Array1<int>    aM(1, 2);
  aK(1) = 0.0;
  aK(2) = 1.0;
  aM(1) = aM(2)                         = 4;
  occ::handle<Geom2d_BSplineCurve> aPath = new Geom2d_BSplineCurve(aP2, aK, aM, 3);
  Geom2dAdaptor_Curve              aC2(aPath);
  gp_Pnt2d                         aQ = aC2.Value(0.0);
  gp_Pnt                           aP;
  gp_Vec                           aDU, aDV;
  aSup->D1(aQ.X(), aQ.Y(), aP, aDU, aDV);
  gp_Vec2d aT2 = aC2.DN(0.0, 1);
  gp_Vec   aT  = aDU * aT2.X() + aDV * aT2.Y();
  int      aKind = r.I(0, 3);
  occ::handle<Geom_Curve> aSect = MakeSection(r, aKind, aP, aT, 0.5);
  Line("IN " + Int(aSupKind) + " " + Hex(aP) + " " + Int(aKind));
  Guard([&] {
    GeomFill_Pipe aPipe;
    aPipe.GenerateParticularCase(true);
    aPipe.Init(aPath, aSup, aSect);
    aPipe.Perform(1.e-4, false, GeomAbs_C2, 11, 30);
    DumpPipe(aPipe);
  });
}

//! The path's poles moved onto a convex planar arc: GeomFill_Pipe's evolving sections fail where the corrected
//! Frenet trihedron needs correcting (occt-issues.md).
static occ::handle<Geom_BSplineCurve> Flatten(const occ::handle<Geom_BSplineCurve>& thePath)
{
  NCollection_Array1<gp_Pnt> aPoles(1, thePath->NbPoles());
  for (int i = 1; i <= thePath->NbPoles(); i++)
  {
    double anA = 0.3 * (i - 1);
    aPoles(i)  = gp_Pnt(20.0 * std::sin(anA), 20.0 - 20.0 * std::cos(anA), 0.0);
  }
  return new Geom_BSplineCurve(aPoles, thePath->Knots(), thePath->Multiplicities(), thePath->Degree());
}

static void PipeTwoSections(Rng& r, bool thePlanar)
{
  occ::handle<Geom_BSplineCurve> aPath = MakePath(r, false, r.R(0.5, 4.0));
  if (thePlanar)
  {
    aPath = Flatten(aPath);
  }
  int                            aKind = r.I(0, 1) == 0 ? 0 : 3;
  gp_Pnt                         aP;
  gp_Vec                         aT;
  aPath->D1(aPath->FirstParameter(), aP, aT);
  Rng                     aR2(r.Next());
  occ::handle<Geom_Curve> aS1 = MakeSection(r, aKind, aP, aT, 1.0);
  occ::handle<Geom_Curve> aS2 = MakeSection(aR2, aKind, aP, aT, r.R(0.5, 2.0));
  Line("IN " + Hex(aPath->Pole(1)) + " " + Int(aKind));
  Guard([&] {
    GeomFill_Pipe aPipe;
    aPipe.GenerateParticularCase(true);
    aPipe.Init(aPath, aS1, aS2);
    aPipe.Perform(1.e-4, false, GeomAbs_C2);
    DumpPipe(aPipe);
  });
}

static void PipeNSections(Rng& r)
{
  occ::handle<Geom_BSplineCurve> aPath = MakePath(r, false, r.R(0.5, 3.0));
  int                            aNb   = r.I(2, 4);
  int                            aKind = r.I(0, 1) == 0 ? 0 : 3;
  NCollection_Sequence<occ::handle<Geom_Curve>> aSeq;
  double                                        f = aPath->FirstParameter(), l = aPath->LastParameter();
  for (int i = 0; i < aNb; i++)
  {
    double t = f + (l - f) * i / (aNb - 1.0);
    gp_Pnt aP;
    gp_Vec aT;
    aPath->D1(t, aP, aT);
    Rng aRi(r.Next());
    aSeq.Append(MakeSection(aRi, aKind, aP, aT, r.R(0.6, 1.6)));
  }
  Line("IN " + Hex(aPath->Pole(1)) + " " + Int(aNb) + " " + Int(aKind));
  Guard([&] {
    GeomFill_Pipe aPipe;
    aPipe.GenerateParticularCase(true);
    aPipe.Init(aPath, aSeq);
    aPipe.Perform(1.e-4, false, GeomAbs_C2);
    DumpPipe(aPipe);
  });
}

static void PipeGuide(Rng& r)
{
  occ::handle<Geom_BSplineCurve> aPath = MakePath(r, false, r.R(0.2, 1.5));
  // the guide: the path's poles shifted sideways and wiggled
  NCollection_Array1<gp_Pnt> aPoles(1, aPath->NbPoles());
  for (int i = 1; i <= aPath->NbPoles(); i++)
  {
    double x  = r.R(-0.5, 0.5);
    double y  = 3.0 + r.R(-0.5, 0.5);
    double z  = r.R(-0.5, 0.5);
    aPoles(i) = aPath->Pole(i).Translated(gp_Vec(x, y, z));
  }
  occ::handle<Geom_BSplineCurve> aGuide = new Geom_BSplineCurve(aPoles,
                                                                aPath->Knots(),
                                                                aPath->Multiplicities(),
                                                                aPath->Degree());
  gp_Pnt aP;
  gp_Vec aT;
  aPath->D1(aPath->FirstParameter(), aP, aT);
  int                     aKind = r.I(0, 3);
  occ::handle<Geom_Curve> aSect = MakeSection(r, aKind, aP, aT, 1.0);
  bool                    byACR = r.I(0, 1) == 1, rotat = r.I(0, 1) == 1;
  Line("IN " + Hex(aPath->Pole(1)) + " " + Int(aKind) + " " + Int(byACR) + " " + Int(rotat));
  Guard([&] {
    occ::handle<GeomAdaptor_Curve> aG = new GeomAdaptor_Curve(aGuide);
    GeomFill_Pipe                  aPipe(aPath, aG, aSect, byACR, rotat);
    aPipe.GenerateParticularCase(true);
    aPipe.Perform(1.e-4, false, GeomAbs_C2, 11, 30);
    DumpPipe(aPipe);
  });
}

//! The path and two guides of a pipe on an edge: the guides run at distance about R on either side.
static void EdgeCurves(Rng&                            r,
                       occ::handle<Geom_BSplineCurve>& thePath,
                       occ::handle<Geom_BSplineCurve>& theC1,
                       occ::handle<Geom_BSplineCurve>& theC2,
                       double&                         theR)
{
  thePath = MakePath(r, false, r.R(0.2, 2.0));
  theR    = r.R(1.0, 3.0);
  double anAng = r.R(0.6, 2.6);
  NCollection_Array1<gp_Pnt> aP1(1, thePath->NbPoles()), aP2(1, thePath->NbPoles());
  for (int i = 1; i <= thePath->NbPoles(); i++)
  {
    gp_Pnt aP = thePath->Pole(i);
    double x1 = r.R(-0.1, 0.1);
    double z1 = r.R(-0.1, 0.1);
    aP1(i)    = aP.Translated(gp_Vec(x1, theR, z1));
    double x2 = r.R(-0.1, 0.1);
    aP2(i)    = aP.Translated(gp_Vec(x2, theR * std::cos(anAng), theR * std::sin(anAng)));
  }
  theC1 = new Geom_BSplineCurve(aP1, thePath->Knots(), thePath->Multiplicities(), thePath->Degree());
  theC2 = new Geom_BSplineCurve(aP2, thePath->Knots(), thePath->Multiplicities(), thePath->Degree());
}

static void PipeEdge(Rng& r, bool theOld)
{
  occ::handle<Geom_BSplineCurve> aPath, aC1, aC2;
  double                         aR;
  EdgeCurves(r, aPath, aC1, aC2, aR);
  bool aPoly = r.I(0, 2) == 0;
  Line("IN " + Hex(aPath->Pole(1)) + " " + Hex(aR) + " " + Int(aPoly));
  Guard([&] {
    GeomFill_Pipe aPipe(aPath, aC1, aC2, aR);
    aPipe.GenerateParticularCase(true);
    if (theOld)
    {
      aPipe.Perform(true, aPoly);
    }
    else
    {
      aPipe.Perform(1.e-4, aPoly, GeomAbs_C2);
    }
    DumpPipe(aPipe);
  });
}

//! The particular cases: straight and circular paths with circles, lines and other sections, conical laws.
static void KPart(Rng& r)
{
  int                     aCase = r.I(0, 7);
  occ::handle<Geom_Curve> aPath;
  if (aCase < 4)
  {
    double dx = r.R(-0.2, 0.2);
    double dy = r.R(-0.2, 0.2);
    gp_Dir aD(dx, dy, 1.0);
    double px = r.R(-1, 1);
    double py = r.R(-1, 1);
    double aL = r.R(5.0, 20.0);
    aPath     = new Geom_TrimmedCurve(new Geom_Line(gp_Pnt(px, py, 0), aD), 0.0, aL);
  }
  else
  {
    double dx = r.R(-0.1, 0.1);
    double dy = r.R(-0.1, 0.1);
    gp_Ax2 anAx(gp_Pnt(0, 0, 0), gp_Dir(dx, dy, 1.0));
    double anEnd = aCase == 7 ? 2 * M_PI : r.R(1.0, 5.0);
    double aRad  = r.R(5.0, 10.0);
    aPath        = new Geom_TrimmedCurve(new Geom_Circle(anAx, aRad), 0.0, anEnd);
  }
  gp_Pnt aP;
  gp_Vec aT;
  aPath->D1(aPath->FirstParameter(), aP, aT);
  occ::handle<Geom_Curve> aSect;
  int                     aSectKind = 0;
  switch (aCase)
  {
    case 0:
    case 4:
    case 7:
      aSectKind = 0; // circle: cylinder, torus, sphere when centred on the axis
      if (aCase == 7 && r.I(0, 1) == 1)
      {
        // a half circle in the plane of the axis: a sphere
        gp_Ax2 anAx(gp_Pnt(0, 0, 0), gp_Dir(aT));
        aSect = new Geom_TrimmedCurve(new Geom_Circle(anAx, r.R(1.0, 3.0)), -M_PI / 2, M_PI / 2);
      }
      break;
    case 1:
    case 5:
      aSectKind = 2; // segment: plane, cone, cylinder
      break;
    case 2:
      aSectKind = 1; // ellipse: extrusion
      break;
    case 3:
      aSectKind = 3; // B-spline: extrusion
      break;
    default:
      aSectKind = 4; // revolution
  }
  if (aSect.IsNull())
  {
    aSect = MakeSection(r, aSectKind, aP, aT, 1.0);
  }
  bool aTwo = aCase == 0 && r.I(0, 1) == 1; // two coaxial circles: a conical law
  Line("IN " + Int(aCase) + " " + Int(aSectKind) + " " + Int(aTwo));
  Guard([&] {
    GeomFill_Pipe aPipe;
    aPipe.GenerateParticularCase(true);
    if (aTwo)
    {
      occ::handle<Geom_Circle> aC2 = occ::down_cast<Geom_Circle>(aSect->Copy());
      aC2->SetRadius(aC2->Radius() * r.R(0.5, 2.0));
      aPipe.Init(aPath, aSect, aC2);
    }
    else
    {
      aPipe.Init(aPath, aSect, GeomFill_IsCorrectedFrenet);
    }
    aPipe.Perform(1.e-4, false, GeomAbs_C2);
    DumpPipe(aPipe);
  });
}

static void SweepEvolved(Rng& r)
{
  occ::handle<Geom_BSplineCurve> aPath = MakePath(r, false, r.R(0.5, 4.0));
  gp_Pnt                         aP;
  gp_Vec                         aT;
  aPath->D1(aPath->FirstParameter(), aP, aT);
  int                     aKind = r.I(0, 4);
  occ::handle<Geom_Curve> aSect = MakeSection(r, aKind, gp_Pnt(0, 0, 0), gp_Vec(0, 0, 1), 1.0);
  occ::handle<Law_Function> aLaw;
  double                    f = aPath->FirstParameter(), l = aPath->LastParameter();
  if (r.I(0, 1) == 0)
  {
    occ::handle<Law_Linear> aLin = new Law_Linear();
    double v1 = r.R(0.5, 1.5);
    double v2 = r.R(0.5, 1.5);
    aLin->Set(f, v1, l, v2);
    aLaw = aLin;
  }
  else
  {
    occ::handle<Law_S> anS = new Law_S();
    double v1 = r.R(0.5, 1.5);
    double v2 = r.R(0.5, 1.5);
    anS->Set(f, v1, l, v2);
    aLaw = anS;
  }
  bool          aKPart = r.I(0, 1) == 1;
  GeomAbs_Shape aCont  = r.I(0, 1) == 0 ? GeomAbs_C1 : GeomAbs_C2;
  bool          aForce = r.I(0, 3) == 0;
  Line("IN " + Hex(aPath->Pole(1)) + " " + Int(aKind) + " " + Int(aKPart) + " " + Int((int)aCont));
  Guard([&] {
    occ::handle<GeomFill_CurveAndTrihedron> aLoc =
      new GeomFill_CurveAndTrihedron(new GeomFill_CorrectedFrenet());
    aLoc->SetCurve(new GeomAdaptor_Curve(aPath));
    occ::handle<GeomFill_EvolvedSection> aSec = new GeomFill_EvolvedSection(aSect, aLaw);
    GeomFill_Sweep                       aSweep(aLoc, aKPart);
    aSweep.SetTolerance(1.e-4, 1.e-4, 1.e-5, 0.01);
    aSweep.SetForceApproxC1(aForce);
    aSweep.Build(aSec, GeomFill_Location, aCont, 10, 30);
    Line("SWEEP " + Int(aSweep.IsDone()) + " " + Hex(aSweep.ErrorOnSurface()) + " " + Int(aSweep.ExchangeUV())
         + " " + Int(aSweep.UReversed()) + " " + Int(aSweep.VReversed()) + " " + Int(aSweep.NumberOfTrace()));
    if (aSweep.IsDone())
    {
      DumpSurface(aSweep.Surface());
      DumpCurve2d("R1", aSweep.Restriction(true));
      DumpCurve2d("R2", aSweep.Restriction(false));
      double aU, aV;
      aSweep.ErrorOnRestriction(true, aU, aV);
      Line("ER " + Hex(aU) + " " + Hex(aV));
    }
  });
}

static void NSectionsLaw(Rng& r)
{
  int                                           aNb   = r.I(2, 4);
  int                                           aKind = r.I(0, 4);
  NCollection_Sequence<occ::handle<Geom_Curve>> aSeq;
  NCollection_Sequence<double>                  aPar;
  for (int i = 0; i < aNb; i++)
  {
    Rng aRi(r.Next());
    double x = r.R(-1, 1);
    double y = r.R(-1, 1);
    double s = r.R(0.5, 2.0);
    aSeq.Append(MakeSection(aRi, aKind, gp_Pnt(x, y, 5.0 * i), gp_Vec(0, 0, 1), s));
    aPar.Append(i == 0 ? 0.0 : aPar.Last() + r.R(0.5, 2.0));
  }
  bool aWithPar = r.I(0, 2) != 0;
  Line("IN " + Int(aNb) + " " + Int(aKind) + " " + Int(aWithPar));
  Guard([&] {
    occ::handle<GeomFill_NSections> aLaw =
      aWithPar ? new GeomFill_NSections(aSeq, aPar, 0.0, 1.0, aPar.First(), aPar.Last())
               : new GeomFill_NSections(aSeq);
    occ::handle<Geom_BSplineSurface> aS = aLaw->BSplineSurface();
    DumpSurface(aS);
    if (aS.IsNull())
    {
      return;
    }
    int aNbP, aNbK, aDeg;
    aLaw->SectionShape(aNbP, aNbK, aDeg);
    double anErr = -1.0;
    bool   aConst = aLaw->IsConstant(anErr);
    bool   aConic = aLaw->IsConicalLaw(anErr);
    Line("SHAPE " + Int(aNbP) + " " + Int(aNbK) + " " + Int(aDeg) + " " + Int(aLaw->IsRational()) + " "
         + Int(aLaw->IsUPeriodic()) + " " + Int(aLaw->IsVPeriodic()) + " " + Int(aConst) + " " + Int(aConic) + " "
         + Hex(aLaw->MaximalSection()) + " " + Hex(aLaw->BarycentreOfSurf()));
    int                        aNbI = aLaw->NbIntervals(GeomAbs_C2);
    NCollection_Array1<double> anI(1, aNbI + 1);
    aLaw->Intervals(anI, GeomAbs_C2);
    std::string aLine = "INT " + Int(aNbI);
    for (int i = 1; i <= aNbI + 1; i++)
    {
      aLine += " " + Hex(anI(i));
    }
    Line(aLine);
    NCollection_Array1<gp_Pnt> aP(1, aNbP);
    NCollection_Array1<gp_Vec> aD1(1, aNbP), aD2(1, aNbP);
    NCollection_Array1<double> aW(1, aNbP), aDW(1, aNbP), aD2W(1, aNbP);
    aLaw->GetMinimalWeight(aW);
    DumpArrays("MINW", aP, aW);
    double f, l;
    aLaw->GetDomain(f, l);
    for (int k = 0; k < 4; k++)
    {
      double v = f + (l - f) * r.U();
      Line("V " + Hex(v) + " " + Int(aLaw->D0(v, aP, aW)));
      DumpArrays("D0", aP, aW);
      bool ok1 = aLaw->D1(v, aP, aD1, aW, aDW);
      Line("D1 " + Int(ok1));
      DumpArrays("D1P", aP, aW);
      DumpArrays("D1V", aD1, aDW);
      bool ok2 = aLaw->D2(v, aP, aD1, aD2, aW, aDW, aD2W);
      Line("D2 " + Int(ok2));
      DumpArrays("D2V", aD2, aD2W);
    }
  });
}

static void BlendFunc(Rng& r)
{
  occ::handle<Geom_BSplineCurve> aPath, aC1, aC2;
  double                         aR;
  EdgeCurves(r, aPath, aC1, aC2, aR);
  bool aPoly = r.I(0, 2) == 0;
  Line("IN " + Hex(aPath->Pole(1)) + " " + Hex(aR) + " " + Int(aPoly));
  Guard([&] {
    occ::handle<GeomFill_CircularBlendFunc> aF =
      new GeomFill_CircularBlendFunc(new GeomAdaptor_Curve(aPath),
                                     new GeomAdaptor_Curve(aC1),
                                     new GeomAdaptor_Curve(aC2),
                                     aR,
                                     aPoly);
    int aNbP, aNbK, aDeg;
    aF->SectionShape(aNbP, aNbK, aDeg);
    Line("SHAPE " + Int(aNbP) + " " + Int(aNbK) + " " + Int(aDeg) + " " + Int(aF->IsRational()) + " "
         + Hex(aF->MaximalSection()) + " " + Hex(aF->BarycentreOfSurf()));
    NCollection_Array1<double> aK(1, aNbK);
    NCollection_Array1<int>    aM(1, aNbK);
    aF->Knots(aK);
    aF->Mults(aM);
    std::string aLine = "KM";
    for (int i = 1; i <= aNbK; i++)
    {
      aLine += " " + Hex(aK(i)) + "x" + Int(aM(i));
    }
    Line(aLine);
    int aNbI = aF->NbIntervals(GeomAbs_C1);
    Line("NBI " + Int(aNbI));
    NCollection_Array1<double> aTol(1, aNbP);
    aF->GetTolerance(1.e-4, 1.e-3, 0.01, aTol);
    aLine = "TOL";
    for (int i = 1; i <= aNbP; i++)
    {
      aLine += " " + Hex(aTol(i));
    }
    Line(aLine);
    NCollection_Array1<gp_Pnt>   aP(1, aNbP);
    NCollection_Array1<gp_Vec>   aD1(1, aNbP), aD2(1, aNbP);
    NCollection_Array1<double>   aW(1, aNbP), aDW(1, aNbP), aD2W(1, aNbP);
    NCollection_Array1<gp_Pnt2d> aP2(1, 1);
    NCollection_Array1<gp_Vec2d> aV2(1, 1), aV22(1, 1);
    aF->GetMinimalWeight(aW);
    DumpArrays("MINW", aP, aW);
    double f = aPath->FirstParameter(), l = aPath->LastParameter();
    for (int k = 0; k < 3; k++)
    {
      double v = f + (l - f) * r.U();
      Line("V " + Hex(v));
      aF->D0(v, f, l, aP, aP2, aW);
      DumpArrays("D0", aP, aW);
      aF->D1(v, f, l, aP, aD1, aP2, aV2, aW, aDW);
      DumpArrays("D1P", aP, aW);
      DumpArrays("D1V", aD1, aDW);
      aF->D2(v, f, l, aP, aD1, aD2, aP2, aV2, aV22, aW, aDW, aD2W);
      DumpArrays("D2V", aD2, aD2W);
    }
  });
}

static void SectionGenerator(Rng& r)
{
  int                            aType = r.I(1, 6);
  occ::handle<Geom_Curve>        aPath;
  occ::handle<Geom_BSplineCurve> aBPath = MakePath(r, false, r.R(0.5, 3.0));
  if (aType >= 4)
  {
    double aRad = r.R(5, 10);
    double anEnd = r.R(1.0, 4.0);
    aPath = new Geom_TrimmedCurve(new Geom_Circle(gp_Ax2(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)), aRad), 0.0, anEnd);
  }
  else
  {
    aPath = aBPath;
  }
  gp_Pnt aP;
  gp_Vec aT;
  aPath->D1(aPath->FirstParameter(), aP, aT);
  int  aKind = r.I(0, 1) == 0 ? 0 : 3;
  Rng  aR2(r.Next());
  occ::handle<Geom_Curve> aS1 = MakeSection(r, aKind, aP, aT, 1.0);
  occ::handle<Geom_Curve> aS2 = MakeSection(aR2, aKind, aP, aT, 1.5);
  Line("IN " + Int(aType) + " " + Int(aKind));
  Guard([&] {
    GeomFill_SweepSectionGenerator aGen;
    if (aType == 1 || aType == 4)
    {
      aGen.Init(aPath, r.R(0.5, 2.0));
    }
    else if (aType == 2 || aType == 5)
    {
      aGen.Init(aPath, aS1);
    }
    else
    {
      aGen.Init(aPath, aS1, aS2);
    }
    aGen.Perform(false);
    int aNbP, aNbK, aDeg, aNb2d;
    aGen.GetShape(aNbP, aNbK, aDeg, aNb2d);
    int aNbS = aGen.NbSections();
    Line("SHAPE " + Int(aNbP) + " " + Int(aNbK) + " " + Int(aDeg) + " " + Int(aNb2d) + " " + Int(aNbS));
    NCollection_Array1<gp_Pnt>   aPo(1, aNbP);
    NCollection_Array1<gp_Vec>   aD(1, aNbP);
    NCollection_Array1<double>   aW(1, aNbP), aDW(1, aNbP);
    NCollection_Array1<gp_Pnt2d> aP2(1, 1);
    NCollection_Array1<gp_Vec2d> aV2(1, 1);
    int                          aIdx[4] = {1, 2, aNbS / 2, aNbS};
    for (int k = 0; k < 4; k++)
    {
      aGen.Section(aIdx[k], aPo, aP2, aW);
      Line("S " + Int(aIdx[k]) + " " + Hex(aGen.Parameter(aIdx[k])));
      DumpArrays("SP", aPo, aW);
    }
    for (int k = 0; k < 4; k += 3)
    {
      bool ok = aGen.Section(aIdx[k], aPo, aD, aP2, aV2, aW, aDW);
      Line("SD " + Int(ok));
      if (ok)
      {
        DumpArrays("SDV", aD, aDW);
      }
    }
  });
}

static void PackageFunctions(Rng& r)
{
  Line("IN");
  Guard([&] {
    // GetShape, Knots, Mults, GetTolerance, GetMinimalWeights
    double                       anAng = r.R(0.05, 2.0 * M_PI);
    Convert_ParameterisationType aConvs[3] = {Convert_QuasiAngular, Convert_Polynomial, Convert_TgtThetaOver2};
    for (Convert_ParameterisationType aConv : aConvs)
    {
      int aNbP, aNbK, aDeg;
      GeomFill::GetShape(anAng, aNbP, aNbK, aDeg, aConv);
      NCollection_Array1<double> aK(1, aNbK);
      NCollection_Array1<int>    aM(1, aNbK);
      GeomFill::Knots(aConv, aK);
      GeomFill::Mults(aConv, aM);
      std::string aLine = "GS " + Int((int)aConv) + " " + Int(aNbP) + " " + Int(aNbK) + " " + Int(aDeg);
      for (int i = 1; i <= aNbK; i++)
      {
        aLine += " " + Hex(aK(i)) + "x" + Int(aM(i));
      }
      Line(aLine);
      double aMin = r.R(0.01, anAng);
      Line("TOL " + Hex(GeomFill::GetTolerance(aConv, aMin, r.R(0.5, 3.0), 0.01, 1.e-4)));
      if (aConv != Convert_TgtThetaOver2 || aNbP <= 7)
      {
        NCollection_Array1<double> aW(1, aNbP);
        NCollection_Array1<gp_Pnt> aDummy(1, aNbP);
        GeomFill::GetMinimalWeights(aConv, aMin, anAng, aW);
        DumpArrays("MINW", aDummy, aW);
      }

      // GetCircle and its derivatives on a random arc and random derivatives
      double aRad = r.R(0.5, 3.0);
      double cx = r.R(-1, 1);
      double cy = r.R(-1, 1);
      double cz = r.R(-1, 1);
      gp_Pnt aC(cx, cy, cz);
      double nx = r.R(-0.3, 0.3);
      double ny = r.R(-0.3, 0.3);
      gp_Dir aN(nx, ny, 1.0);
      gp_Ax2 anAx(aC, aN);
      double a1 = r.R(0.0, 1.0), a2 = a1 + anAng;
      gp_Vec aX(anAx.XDirection()), aY(anAx.YDirection());
      gp_Vec aU1 = aX * std::cos(a1) + aY * std::sin(a1), aU2 = aX * std::cos(a2) + aY * std::sin(a2);
      gp_Pnt aP1 = aC.Translated(aU1 * aRad), aP2 = aC.Translated(aU2 * aRad);
      gp_Vec ns1 = -aU1, ns2 = -aU2, nplan(aN);
      auto   aRV = [&r]() {
        double x = r.R(-0.2, 0.2);
        double y = r.R(-0.2, 0.2);
        double z = r.R(-0.2, 0.2);
        return gp_Vec(x, y, z);
      };
      gp_Vec dn1 = aRV(), dn2 = aRV(), dnp = aRV(), d2n1 = aRV(), d2n2 = aRV(), d2np = aRV();
      gp_Vec t1 = aRV(), t2 = aRV(), dt1 = aRV(), dt2 = aRV(), dc = aRV(), d2c = aRV();
      NCollection_Array1<gp_Pnt> aP(1, aNbP);
      NCollection_Array1<gp_Vec> aD1(1, aNbP), aD2(1, aNbP);
      NCollection_Array1<double> aW(1, aNbP), aDW(1, aNbP), aD2W(1, aNbP);
      GeomFill::GetCircle(aConv, ns1, ns2, nplan, aP1, aP2, aRad, aC, aP, aW);
      DumpArrays("C0", aP, aW);
      bool ok1 = GeomFill::GetCircle(aConv,
                                     ns1,
                                     ns2,
                                     dn1,
                                     dn2,
                                     nplan,
                                     dnp,
                                     aP1,
                                     aP2,
                                     t1,
                                     t2,
                                     aRad,
                                     0.1,
                                     aC,
                                     dc,
                                     aP,
                                     aD1,
                                     aW,
                                     aDW);
      Line("C1 " + Int(ok1));
      DumpArrays("C1P", aP, aW);
      DumpArrays("C1V", aD1, aDW);
      bool ok2 = GeomFill::GetCircle(aConv,
                                     ns1,
                                     ns2,
                                     dn1,
                                     dn2,
                                     d2n1,
                                     d2n2,
                                     nplan,
                                     dnp,
                                     d2np,
                                     aP1,
                                     aP2,
                                     t1,
                                     t2,
                                     dt1,
                                     dt2,
                                     aRad,
                                     0.1,
                                     0.05,
                                     aC,
                                     dc,
                                     d2c,
                                     aP,
                                     aD1,
                                     aD2,
                                     aW,
                                     aDW,
                                     aD2W);
      Line("C2 " + Int(ok2));
      DumpArrays("C2P", aP, aW);
      DumpArrays("C2V", aD1, aDW);
      DumpArrays("C2W", aD2, aD2W);
    }
  });
}

static void Ruled(Rng& r)
{
  int                     aCase = r.I(0, 4);
  occ::handle<Geom_Curve> aC1, aC2;
  switch (aCase)
  {
    case 0: { // two parallel segments (a plane), the second reversed or not
      double dx = r.R(-1, 1);
      double dy = r.R(-1, 1);
      gp_Dir aD(dx, dy, 1.0);
      double px = r.R(-1, 1);
      double py = r.R(-1, 1);
      gp_Pnt aP(px, py, 0);
      double ox = r.R(1, 3);
      double oy = r.R(1, 3);
      gp_Vec aOff(ox, oy, 0);
      double a = r.R(-2, 0), b = r.R(1, 3);
      aC1 = new Geom_TrimmedCurve(new Geom_Line(aP, aD), a, b);
      gp_Vec aDV(aD);
      double proj = aOff.Dot(aDV);
      if (r.I(0, 1) == 0)
      {
        aC2 = new Geom_TrimmedCurve(new Geom_Line(aP.Translated(aOff), aD), a - proj, b - proj);
      }
      else
      {
        aC2 = new Geom_TrimmedCurve(new Geom_Line(aP.Translated(aOff), aD.Reversed()), proj - b, proj - a);
      }
      break;
    }
    case 1:
    case 2: { // coaxial circles: a cylinder or a cone
      double px = r.R(-1, 1);
      double dx = r.R(-0.2, 0.2);
      gp_Ax2 anAx(gp_Pnt(px, 0, 0), gp_Dir(dx, 0, 1));
      double aR = r.R(1, 3);
      aC1       = new Geom_Circle(anAx, aR);
      gp_Ax2 anAx2 = anAx.Translated(gp_Vec(anAx.Direction()) * r.R(-5, 5));
      aC2          = new Geom_Circle(anAx2, aCase == 1 ? aR : aR * r.R(0.3, 2.0));
      break;
    }
    default: { // B-splines and others: GeomFill_Generator
      aC1 = MakePath(r, false, 2.0);
      occ::handle<Geom_BSplineCurve> aB = occ::down_cast<Geom_BSplineCurve>(MakePath(r, false, 2.0));
      aC2 = aCase == 3 ? occ::handle<Geom_Curve>(aB)
                       : MakeSection(r, 1, gp_Pnt(0, 5, 0), gp_Vec(0, 1, 0), 2.0);
    }
  }
  Line("IN " + Int(aCase));
  Guard([&] { DumpSurface(GeomFill::Surface(aC1, aC2)); });
}

int main()
{
  struct Kind
  {
    const char* Name;
    int         Count;
  };
  const Kind theKinds[] = {{"pipe-radius", 40},
                           {"pipe-cfrenet", 40},
                           {"pipe-frenet", 30},
                           {"pipe-fixed", 30},
                           {"pipe-cnormal", 30},
                           {"pipe-dir", 30},
                           {"pipe-darboux", 30},
                           {"pipe-2sect", 30},
                           {"pipe-nsect", 30},
                           {"pipe-guide", 30},
                           {"pipe-edge-old", 30},
                           {"pipe-edge", 30},
                           {"kpart", 80},
                           {"sweep-evolved", 40},
                           {"nsections", 40},
                           {"blendfunc", 30},
                           {"secgen", 40},
                           {"package", 30},
                           {"ruled", 30},
                           {"pipe-2sect-plane", 40}};
  int aKindIndex = 0;
  for (const Kind& aKind : theKinds)
  {
    for (int aSeed = 0; aSeed < aKind.Count; aSeed++)
    {
      Rng r((uint64_t)(aKindIndex * 1000 + aSeed));
      Line(std::string("CASE ") + aKind.Name + " " + Int(aSeed));
      std::string aName = aKind.Name;
      if (aName == "pipe-radius")
        PipeRadius(r);
      else if (aName == "pipe-cfrenet")
        PipeSection(r, GeomFill_IsCorrectedFrenet);
      else if (aName == "pipe-frenet")
        PipeSection(r, GeomFill_IsFrenet);
      else if (aName == "pipe-fixed")
        PipeSection(r, GeomFill_IsFixed);
      else if (aName == "pipe-cnormal")
        PipeSection(r, GeomFill_IsConstantNormal);
      else if (aName == "pipe-dir")
        PipeDir(r);
      else if (aName == "pipe-darboux")
        PipeDarboux(r);
      else if (aName == "pipe-2sect")
        PipeTwoSections(r, false);
      else if (aName == "pipe-nsect")
        PipeNSections(r);
      else if (aName == "pipe-guide")
        PipeGuide(r);
      else if (aName == "pipe-edge-old")
        PipeEdge(r, true);
      else if (aName == "pipe-edge")
        PipeEdge(r, false);
      else if (aName == "kpart")
        KPart(r);
      else if (aName == "sweep-evolved")
        SweepEvolved(r);
      else if (aName == "nsections")
        NSectionsLaw(r);
      else if (aName == "blendfunc")
        BlendFunc(r);
      else if (aName == "secgen")
        SectionGenerator(r);
      else if (aName == "package")
        PackageFunctions(r);
      else if (aName == "ruled")
        Ruled(r);
      else if (aName == "pipe-2sect-plane")
        PipeTwoSections(r, true);
      Flush();
    }
    aKindIndex++;
  }
  return 0;
}
