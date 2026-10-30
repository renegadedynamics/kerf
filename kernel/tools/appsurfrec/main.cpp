// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
//
// appsurfrec: records native OCCT's surfaces through sections (unit 5L3): GeomFill_Profiler through
// GeomFill_Generator (B-spline, Bezier, circle-arc and segment sections, periodic or not), and GeomFill_AppSurf over a
// GeomFill_SectionGenerator and a GeomFill_Line (AppBlend_AppSurf: Bezier pieces joined, the B-spline least squares
// with each continuity and parametrisation, the variational smoothing, pieces of at most NbMaxP sections, known
// parameters or not). Random but reproducible inputs (a fixed splitmix64 stream) are written as round-trip
// decimals, the results as named tokens with every double as its 64 bits in hex (pole grids as an FNV-1a digest of
// their bits). The port replays each case from the same inputs and writes its results in the same format
// (tests/OcctNet.ModelingAlgorithms.Tests/GeomFill/AppSurfRecordings.cs).
// Built against build123d.NET's native OCCT V8_0_1 (tools/appsurfrec/build.ps1).
//
//   appsurfrec <cases per kind>   writes the recording to stdout
//
// One line per case:  <kind> <inputs...> ; <tokens...>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <functional>
#include <string>
#include <typeinfo>
#include <vector>

#include <GeomFill_AppSurf.hxx>
#include <GeomFill_Generator.hxx>
#include <GeomFill_Line.hxx>
#include <GeomFill_SectionGenerator.hxx>
#include <Geom_BSplineCurve.hxx>
#include <Geom_BSplineSurface.hxx>
#include <Geom_BezierCurve.hxx>
#include <Geom_Circle.hxx>
#include <Geom_Line.hxx>
#include <Geom_TrimmedCurve.hxx>
#include <NCollection_Array1.hxx>
#include <NCollection_Array2.hxx>
#include <NCollection_HArray1.hxx>
#include <Precision.hxx>
#include <Standard_Failure.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Pnt2d.hxx>
#include <gp_Vec.hxx>

//=================================================================================================
// Output

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

static std::string Hex(const gp_Pnt& theP)
{
  return Hex(theP.X()) + "," + Hex(theP.Y()) + "," + Hex(theP.Z());
}

// FNV-1a over the 64 bits of each value, for the pole grids
struct Digest
{
  uint64_t myH = 14695981039346656037ULL;
  int      myN = 0;

  void Add(double theValue)
  {
    uint64_t aBits;
    std::memcpy(&aBits, &theValue, 8);
    for (int i = 0; i < 8; i++)
    {
      myH ^= (aBits >> (8 * i)) & 0xff;
      myH *= 1099511628211ULL;
    }
    myN++;
  }

  void Add(const gp_Pnt& theP)
  {
    Add(theP.X());
    Add(theP.Y());
    Add(theP.Z());
  }

  std::string Str() const
  {
    char aBuf[40];
    std::snprintf(aBuf, sizeof(aBuf), "%d,%llx", myN, (unsigned long long)myH);
    return aBuf;
  }
};

static std::string ExcName(const Standard_Failure& theF)
{
  std::string aName = typeid(theF).name(); // "class Standard_OutOfRange"
  if (aName.rfind("class ", 0) == 0)
  {
    aName = aName.substr(6);
  }
  return "!" + aName;
}

//=================================================================================================
// Inputs

static std::vector<double> theIn;

static double In(double theValue)
{
  theIn.push_back(theValue);
  return theValue;
}

static uint64_t theState = 0xA995F5ECULL;

static uint64_t Next()
{
  uint64_t z = (theState += 0x9e3779b97f4a7c15ULL);
  z          = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ULL;
  z          = (z ^ (z >> 27)) * 0x94d049bb133111ebULL;
  return z ^ (z >> 31);
}

// A uniform double in [a, b)
static double Rnd(double a, double b)
{
  return a + (b - a) * (double)(Next() >> 11) * (1.0 / 9007199254740992.0);
}

// A uniform integer in [a, b]
static int Irnd(int a, int b)
{
  return a + (int)(Next() % (uint64_t)(b - a + 1));
}

//=================================================================================================
// Sections: generated, written to the inputs and built. Each starts with its type:
//   0 B-spline:  deg rat per np (x y z [w])* nk (k m)*
//   1 Bezier:    rat np (x y z [w])*
//   2 arc:       cx h cz r a b        (a circle of axis Y in the plane y = h, trimmed to [a, b])
//   3 segment:   x y z dx dy dz l     (a line trimmed to [0, l])

// A section in the plane y = theH (roughly), spanning x in [0, theW]
static occ::handle<Geom_Curve> MakeSection(int theType, double theH, double theW, bool thePeriodic)
{
  In(theType);
  if (theType == 0)
  {
    int  aDeg = Irnd(1, 4);
    bool aRat = Irnd(0, 9) < 3;
    In(aDeg);
    In(aRat ? 1 : 0);
    In(thePeriodic ? 1 : 0);
    if (thePeriodic)
    {
      // a closed loop around (theW/2, theH, 0): np poles, np + 1 knots of multiplicity 1
      int aNbP = Irnd(aDeg + 1, 8);
      In(aNbP);
      NCollection_Array1<gp_Pnt> aPoles(1, aNbP);
      NCollection_Array1<double> aW(1, aNbP);
      double                     aR = Rnd(1.0, 3.0);
      for (int i = 1; i <= aNbP; i++)
      {
        double t = 2.0 * M_PI * (i - 1) / aNbP;
        double x = In(theW / 2 + aR * std::cos(t) + Rnd(-0.2, 0.2));
        double y = In(theH + Rnd(-0.2, 0.2));
        double z = In(aR * std::sin(t) + Rnd(-0.2, 0.2));
        aPoles(i) = gp_Pnt(x, y, z);
        if (aRat)
          aW(i) = In(Rnd(0.5, 2.0));
      }
      In(aNbP + 1);
      NCollection_Array1<double> aKnots(1, aNbP + 1);
      NCollection_Array1<int>    aMults(1, aNbP + 1);
      for (int i = 1; i <= aNbP + 1; i++)
      {
        aKnots(i) = In((double)(i - 1));
        aMults(i) = (int)In(1);
      }
      if (aRat)
        return new Geom_BSplineCurve(aPoles, aW, aKnots, aMults, aDeg, true);
      return new Geom_BSplineCurve(aPoles, aKnots, aMults, aDeg, true);
    }
    int              aNbI = Irnd(0, 3);
    std::vector<int> aSlots;
    while ((int)aSlots.size() < aNbI)
    {
      int  s    = Irnd(1, 15);
      bool aNew = true;
      for (int t : aSlots)
        aNew = aNew && t != s;
      if (aNew)
        aSlots.push_back(s);
    }
    std::sort(aSlots.begin(), aSlots.end());
    static const double THE_STARTS[]  = {0.0, 1.0, -2.5, 10.0};
    static const double THE_LENGTHS[] = {1.0, 2.0, 0.5, 7.25};
    double              t0            = THE_STARTS[Irnd(0, 3)];
    double              aLen          = THE_LENGTHS[Irnd(0, 3)];
    int                 aNbK          = aNbI + 2;
    std::vector<double> aK(aNbK);
    std::vector<int>    aM(aNbK);
    aK[0]        = t0;
    aK[aNbK - 1] = t0 + aLen;
    aM[0] = aM[aNbK - 1] = aDeg + 1;
    for (int i = 0; i < aNbI; i++)
    {
      aK[i + 1] = t0 + aLen * aSlots[i] / 16.0;
      aM[i + 1] = (aDeg >= 2 && Irnd(0, 3) == 0) ? 2 : 1;
    }
    int aNbP = -aDeg - 1;
    for (int m : aM)
      aNbP += m;
    In(aNbP);
    NCollection_Array1<gp_Pnt> aPoles(1, aNbP);
    NCollection_Array1<double> aW(1, aNbP);
    for (int i = 1; i <= aNbP; i++)
    {
      double f = (double)(i - 1) / (aNbP - 1);
      double x = In(theW * f + Rnd(-0.3, 0.3));
      double y = In(theH + Rnd(-0.3, 0.3));
      double z = In(Rnd(-1.5, 1.5));
      aPoles(i) = gp_Pnt(x, y, z);
      if (aRat)
        aW(i) = In(Rnd(0.5, 2.0));
    }
    In(aNbK);
    NCollection_Array1<double> aKnots(1, aNbK);
    NCollection_Array1<int>    aMults(1, aNbK);
    for (int i = 1; i <= aNbK; i++)
    {
      aKnots(i) = In(aK[i - 1]);
      aMults(i) = (int)In(aM[i - 1]);
    }
    if (aRat)
      return new Geom_BSplineCurve(aPoles, aW, aKnots, aMults, aDeg);
    return new Geom_BSplineCurve(aPoles, aKnots, aMults, aDeg);
  }
  if (theType == 1)
  {
    int  aNbP = Irnd(2, 6);
    bool aRat = Irnd(0, 9) < 3;
    In(aRat ? 1 : 0);
    In(aNbP);
    NCollection_Array1<gp_Pnt> aPoles(1, aNbP);
    NCollection_Array1<double> aW(1, aNbP);
    for (int i = 1; i <= aNbP; i++)
    {
      double f = (double)(i - 1) / (aNbP - 1);
      double x = In(theW * f + Rnd(-0.3, 0.3));
      double y = In(theH + Rnd(-0.3, 0.3));
      double z = In(Rnd(-1.5, 1.5));
      aPoles(i) = gp_Pnt(x, y, z);
      if (aRat)
        aW(i) = In(Rnd(0.5, 2.0));
    }
    if (aRat)
      return new Geom_BezierCurve(aPoles, aW);
    return new Geom_BezierCurve(aPoles);
  }
  if (theType == 2)
  {
    double cx = In(Rnd(-0.5, 0.5));
    double h  = In(theH);
    double cz = In(Rnd(-0.5, 0.5));
    double r  = In(Rnd(1.0, 4.0));
    double a  = In(Rnd(0.0, 1.0));
    double b  = In(a + Rnd(1.0, 5.0));
    occ::handle<Geom_Circle> aC = new Geom_Circle(gp_Ax2(gp_Pnt(cx, h, cz), gp_Dir(0, 1, 0)), r);
    return new Geom_TrimmedCurve(aC, a, b);
  }
  double x  = In(Rnd(-0.5, 0.5));
  double y  = In(theH + Rnd(-0.3, 0.3));
  double z  = In(Rnd(-0.5, 0.5));
  double dx = In(Rnd(0.5, 1.0));
  double dy = In(Rnd(-0.2, 0.2));
  double dz = In(Rnd(-0.5, 0.5));
  double l  = In(Rnd(0.5, 1.0) * theW);
  occ::handle<Geom_Line> aL = new Geom_Line(gp_Pnt(x, y, z), gp_Dir(dx, dy, dz));
  return new Geom_TrimmedCurve(aL, 0.0, l);
}

// The section types of a case: all one type, or mixed, or B-splines of which some are periodic. Never all periodic:
// the profiler then inserts knots in periodic curves with Add = false, where BSplCLib::PrepareInsertKnots
// mispredicts the pole count and BSplCLib::InsertKnots writes past its arrays (heap corruption natively,
// occt-issues.md; the port throws Standard_RangeError).
static void Sections(int theNb, std::vector<occ::handle<Geom_Curve>>& theCurves)
{
  int  aMode     = Irnd(0, 9);
  bool aPeriodic = false;
  int  aOne      = Irnd(0, 3);
  double aW      = Rnd(3.0, 8.0);
  double aH      = 0.0;
  for (int i = 0; i < theNb; i++)
  {
    int aType = aPeriodic ? 0 : (aMode < 5 ? aOne : Irnd(0, 3));
    if (aMode >= 8)
    {
      aType = 0; // B-splines, some periodic: the profiler makes them all non-periodic
    }
    theCurves.push_back(MakeSection(aType, aH, aW, aPeriodic || (aMode >= 8 && Irnd(0, 1) == 0)));
    aH += Rnd(1.0, 4.0);
  }
}

//=================================================================================================
// Results

// Records the common profile of the curves; false when the curves have not been brought to one number of poles
// (UnifyBySettingMiddleKnots only moves knots; then OCCT's Poles, Generator and AppSurf read and write past the
// arrays sized for the first curve, occt-issues.md), in which case nothing else is run.
static bool PutProfile(const std::vector<occ::handle<Geom_Curve>>& theCurves)
{
  GeomFill_Profiler aP;
  for (const occ::handle<Geom_Curve>& aC : theCurves)
    aP.AddCurve(aC);
  aP.Perform(Precision::PConfusion());
  int aDeg = aP.Degree(), aNbP = aP.NbPoles(), aNbK = aP.NbKnots();
  Put("prof:" + std::to_string(aDeg) + "," + std::to_string(aNbP) + "," + std::to_string(aNbK) + ","
      + std::to_string(aP.IsPeriodic() ? 1 : 0));
  NCollection_Array1<double> aK(1, aNbK);
  NCollection_Array1<int>    aM(1, aNbK);
  aP.KnotsAndMults(aK, aM);
  std::string aS;
  for (int i = 1; i <= aNbK; i++)
    aS += (aS.empty() ? "" : ",") + Hex(aK(i)) + "," + std::to_string(aM(i));
  Put("pk:" + aS);
  std::string aCounts;
  bool        anEqual = true;
  for (int c = 1; c <= (int)theCurves.size(); c++)
  {
    occ::handle<Geom_BSplineCurve> aC = occ::down_cast<Geom_BSplineCurve>(aP.Curve(c));
    aCounts += (aCounts.empty() ? "" : ",") + std::to_string(aC->NbPoles()) + "," + std::to_string(aC->NbKnots());
    anEqual = anEqual && aC->NbPoles() == aNbP && aC->NbKnots() == aNbK;
  }
  Put("pn:" + aCounts);
  if (!anEqual)
    return false;
  for (int c = 1; c <= (int)theCurves.size(); c++)
  {
    NCollection_Array1<gp_Pnt> aPoles(1, aNbP);
    NCollection_Array1<double> aW(1, aNbP);
    aP.Poles(c, aPoles);
    aP.Weights(c, aW);
    Digest aDP, aDW;
    for (int i = 1; i <= aNbP; i++)
    {
      aDP.Add(aPoles(i));
      aDW.Add(aW(i));
    }
    Put("pp" + std::to_string(c) + ":" + aDP.Str() + "," + aDW.Str() + "," + Hex(aPoles(1)));
  }
  return true;
}

static void PutBSplineSurface(const occ::handle<Geom_BSplineSurface>& theS)
{
  Put("sdeg:" + std::to_string(theS->UDegree()) + "," + std::to_string(theS->VDegree()) + ","
      + std::to_string(theS->NbUPoles()) + "," + std::to_string(theS->NbVPoles()) + ","
      + std::to_string(theS->IsURational() ? 1 : 0) + std::to_string(theS->IsVRational() ? 1 : 0)
      + std::to_string(theS->IsUPeriodic() ? 1 : 0) + std::to_string(theS->IsVPeriodic() ? 1 : 0));
  Digest aP, aW;
  for (int i = 1; i <= theS->NbUPoles(); i++)
    for (int j = 1; j <= theS->NbVPoles(); j++)
    {
      aP.Add(theS->Pole(i, j));
      aW.Add(theS->Weight(i, j));
    }
  Put("spol:" + aP.Str());
  Put("swt:" + aW.Str());
  std::string aU, aV;
  for (int i = 1; i <= theS->NbUKnots(); i++)
    aU += (aU.empty() ? "" : ",") + Hex(theS->UKnot(i)) + "," + std::to_string(theS->UMultiplicity(i));
  for (int i = 1; i <= theS->NbVKnots(); i++)
    aV += (aV.empty() ? "" : ",") + Hex(theS->VKnot(i)) + "," + std::to_string(theS->VMultiplicity(i));
  Put("suk:" + aU);
  Put("svk:" + aV);
  double u1, u2, v1, v2;
  theS->Bounds(u1, u2, v1, v2);
  std::string aD;
  for (int i = 0; i <= 2; i++)
    for (int j = 0; j <= 2; j++)
      aD += (aD.empty() ? "" : ",") + Hex(theS->Value(u1 + (u2 - u1) * i / 2.0, v1 + (v2 - v1) * j / 2.0));
  Put("sd0:" + aD);
}

static void PutApprox(GeomFill_AppSurf& theA)
{
  Put(std::string("done:") + (theA.IsDone() ? "1" : "0"));
  if (!theA.IsDone())
    return;
  int ud, vd, nup, nvp, nuk, nvk;
  theA.SurfShape(ud, vd, nup, nvp, nuk, nvk);
  Put("shape:" + std::to_string(ud) + "," + std::to_string(vd) + "," + std::to_string(nup) + ","
      + std::to_string(nvp) + "," + std::to_string(nuk) + "," + std::to_string(nvk));
  Digest aP, aW;
  const NCollection_Array2<gp_Pnt>& aPoles = theA.SurfPoles();
  const NCollection_Array2<double>& aWts   = theA.SurfWeights();
  for (int i = aPoles.LowerRow(); i <= aPoles.UpperRow(); i++)
    for (int j = aPoles.LowerCol(); j <= aPoles.UpperCol(); j++)
    {
      aP.Add(aPoles(i, j));
      aW.Add(aWts(i, j));
    }
  Put("apol:" + aP.Str() + "," + Hex(aPoles(aPoles.LowerRow(), aPoles.LowerCol())));
  Put("awt:" + aW.Str());
  std::string aU, aV;
  for (int i = theA.SurfUKnots().Lower(); i <= theA.SurfUKnots().Upper(); i++)
    aU += (aU.empty() ? "" : ",") + Hex(theA.SurfUKnots()(i)) + "," + std::to_string(theA.SurfUMults()(i));
  for (int i = theA.SurfVKnots().Lower(); i <= theA.SurfVKnots().Upper(); i++)
    aV += (aV.empty() ? "" : ",") + Hex(theA.SurfVKnots()(i)) + "," + std::to_string(theA.SurfVMults()(i));
  Put("auk:" + aU);
  Put("avk:" + aV);
  double t3, t2;
  theA.TolReached(t3, t2);
  Put("tol:" + Hex(t3) + "," + Hex(t2) + "," + Hex(theA.TolCurveOnSurf(1)) + "," + std::to_string(theA.NbCurves2d()));
  // the surface Draw's appsurf builds
  try
  {
    occ::handle<Geom_BSplineSurface> aS = new Geom_BSplineSurface(theA.SurfPoles(),
                                                                  theA.SurfWeights(),
                                                                  theA.SurfUKnots(),
                                                                  theA.SurfVKnots(),
                                                                  theA.SurfUMults(),
                                                                  theA.SurfVMults(),
                                                                  theA.UDegree(),
                                                                  theA.VDegree());
    PutBSplineSurface(aS);
  }
  catch (const Standard_Failure& theF)
  {
    Put("srf:" + ExcName(theF));
  }
}

static void Line(const char* theKind)
{
  std::string aLine = theKind;
  char        aBuf[40];
  for (double v : theIn)
  {
    std::snprintf(aBuf, sizeof(aBuf), " %.17g", v);
    aLine += aBuf;
  }
  aLine += " ;" + theOut;
  std::puts(aLine.c_str());
  theIn.clear();
  theOut.clear();
}

//=================================================================================================
// The cases

// GeomFill_Generator on 2 to 6 sections: the profile, then the ruled surface
static void GeneratorCase()
{
  int                                  aNb = (int)In(Irnd(2, 6));
  std::vector<occ::handle<Geom_Curve>> aCurves;
  Sections(aNb, aCurves);
  try
  {
    if (!PutProfile(aCurves))
      return;
    GeomFill_Generator aGen;
    for (const occ::handle<Geom_Curve>& aC : aCurves)
      aGen.AddCurve(aC);
    aGen.Perform(Precision::PConfusion());
    PutBSplineSurface(occ::down_cast<Geom_BSplineSurface>(aGen.Surface()));
  }
  catch (const Standard_Failure& theF)
  {
    Put("exc:" + ExcName(theF));
  }
}

// GeomFill_AppSurf on 2 to 8 sections, in one of its four ways
static void AppSurfCase(int theMode)
{
  int                                  aNb = (int)In(Irnd(theMode == 3 ? 4 : 2, 8));
  std::vector<occ::handle<Geom_Curve>> aCurves;
  Sections(aNb, aCurves);
  static const double THE_TOLS[] = {1.0e-7, 1.0e-5, 1.0e-3};
  int    aDMin   = (int)In(Irnd(1, 4));
  int    aDMax   = (int)In(Irnd(aDMin, 8));
  double aTol3d  = In(THE_TOLS[Irnd(0, 2)]);
  double aTol2d  = In(THE_TOLS[Irnd(0, 2)]);
  int    aNbIt   = (int)In(Irnd(0, 3));
  bool   aKnown  = In(Irnd(0, 2) == 0 ? 1 : 0) != 0;
  int    aCont   = (int)In(theMode == 2 ? Irnd(1, 2) : Irnd(0, 3)); // no C0 smoothing (decision 206)
  int    aParT   = (int)In(Irnd(0, 2));
  double aW1     = In(Rnd(0.1, 1.0));
  double aW2     = In(Rnd(0.1, 1.0));
  double aW3     = In(Rnd(0.1, 1.0));
  int    aNbMaxP = (int)In(Irnd(2, aNb));
  bool   aDraw   = theMode == 0 && In(Irnd(0, 3) == 0 ? 1 : 0) != 0;
  std::vector<double> aParams;
  double              aU = In(Rnd(-2.0, 2.0));
  for (int i = 0; i < aNb; i++)
  {
    aParams.push_back(aU);
    aU = In(aU + Rnd(0.25, 3.0));
  }
  try
  {
    if (!PutProfile(aCurves))
      return;
    GeomFill_SectionGenerator aSec;
    for (const occ::handle<Geom_Curve>& aC : aCurves)
      aSec.AddCurve(aC);
    aSec.Perform(Precision::PConfusion());
    occ::handle<NCollection_HArray1<double>> aPar = new NCollection_HArray1<double>(1, aNb);
    for (int i = 1; i <= aNb; i++)
      aPar->SetValue(i, aParams[i - 1]);
    aSec.SetParam(aPar);
    occ::handle<GeomFill_Line> aLine = new GeomFill_Line(aNb);
    GeomFill_AppSurf           anApp;
    if (aDraw)
    {
      anApp.Init(3, 8, Precision::Confusion(), Precision::PConfusion(), 0);
    }
    else
    {
      anApp.Init(aDMin, aDMax, aTol3d, aTol2d, aNbIt, aKnown);
      static const GeomAbs_Shape              THE_CONT[] = {GeomAbs_C0, GeomAbs_C1, GeomAbs_C2, GeomAbs_C3};
      static const Approx_ParametrizationType THE_PAR[]  = {Approx_ChordLength,
                                                            Approx_Centripetal,
                                                            Approx_IsoParametric};
      anApp.SetContinuity(THE_CONT[aCont]);
      anApp.SetParType(THE_PAR[aParT]);
      anApp.SetCriteriumWeight(aW1, aW2, aW3);
    }
    if (theMode == 0)
      anApp.Perform(aLine, aSec);
    else if (theMode == 1)
      anApp.Perform(aLine, aSec, true);
    else if (theMode == 2)
      anApp.PerformSmoothing(aLine, aSec);
    else
      anApp.Perform(aLine, aSec, aNbMaxP);
    PutApprox(anApp);
  }
  catch (const Standard_Failure& theF)
  {
    Put("exc:" + ExcName(theF));
  }
}

int main(int argc, char** argv)
{
  int aNb = argc > 1 ? std::atoi(argv[1]) : 100;
  for (int i = 0; i < aNb; i++)
  {
    GeneratorCase();
    Line("GEN");
  }
  static const char* THE_KINDS[] = {"APP", "APPSP", "APPSM", "APPMAXP"};
  for (int m = 0; m < 4; m++)
    for (int i = 0; i < aNb; i++)
    {
      AppSurfCase(m);
      Line(THE_KINDS[m]);
    }
  return 0;
}
