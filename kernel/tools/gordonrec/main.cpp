// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
//
// gordonrec: records native OCCT's Gordon surfaces (GeomFill_Gordon, and GeomFill_NetworkSurface directly) on
// reproducible random curve networks (unit 5L3). Each case writes its input curves as B-spline data in round-trip
// decimals, then the results: status, report and, when done, the surface (degrees, periodicity, an FNV-1a hash of every
// pole and weight with the first twelve written out, knots, multiplicities) and a 5 x 5 grid of points on it, every double
// as its 64 bits in hex. The port rebuilds the same
// curves from the inputs and writes its results in the same format
// (tests/OcctNet.ModelingAlgorithms.Tests/GeomFill/GeomFill_GordonRecordingsTests.cs).
// Built against build123d.NET's native OCCT V8_0_1 (tools/gordonrec/build.ps1).
//
//   gordonrec <seed>   writes the recording to stdout
//
// Lines:
//   G <name> <parallel 0|1> <approx 0|1> <tol> P <n> <curve>... Q <m> <curve>... ; <tokens>
//   N <name> <tol> <uclosed> <vclosed> P <n> <curve>... Q <m> <curve>... V <n> <v>... U <m> <u>...
//     X <m*n points> W <m*n weights> ; <tokens>
// A curve is: <degree> <periodic> <rational> <nbpoles> (x y z [w])... <nbknots> (knot mult)...
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include <GeomAPI_Interpolate.hxx>
#include <GeomFill_Gordon.hxx>
#include <GeomFill_NetworkSurface.hxx>
#include <Geom_BSplineCurve.hxx>
#include <Geom_BSplineSurface.hxx>
#include <NCollection_Array1.hxx>
#include <NCollection_Array2.hxx>
#include <NCollection_HArray1.hxx>
#include <Precision.hxx>
#include <Standard_Failure.hxx>
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
  std::snprintf(aBuf, sizeof(aBuf), "%016llx", (unsigned long long)aBits);
  return aBuf;
}

static std::string Dec(double theValue)
{
  char aBuf[40];
  std::snprintf(aBuf, sizeof(aBuf), "%.17g", theValue);
  return aBuf;
}

static std::string Int(int theValue)
{
  return std::to_string(theValue);
}

// xorshift64*
static uint64_t theState = 88172645463325252ull;

static double Rand()
{
  theState ^= theState >> 12;
  theState ^= theState << 25;
  theState ^= theState >> 27;
  uint64_t aValue = theState * 2685821657736338717ull;
  return (double)(aValue >> 11) * (1.0 / 9007199254740992.0);
}

static double Rand(double theMin, double theMax)
{
  return theMin + (theMax - theMin) * Rand();
}

static int RandInt(int theMin, int theMax) // inclusive
{
  int aValue = theMin + (int)(Rand() * (theMax - theMin + 1));
  return aValue > theMax ? theMax : aValue;
}

// Rounds to 1/1024 so that the inputs are short and exact in binary.
static double Snap(double theValue)
{
  return std::floor(theValue * 1024.0 + 0.5) / 1024.0;
}

static void PutCurve(const occ::handle<Geom_BSplineCurve>& theCurve)
{
  Put(Int(theCurve->Degree()));
  Put(theCurve->IsPeriodic() ? "1" : "0");
  Put(theCurve->IsRational() ? "1" : "0");
  Put(Int(theCurve->NbPoles()));
  for (int i = 1; i <= theCurve->NbPoles(); ++i)
  {
    const gp_Pnt aP = theCurve->Pole(i);
    Put(Dec(aP.X()));
    Put(Dec(aP.Y()));
    Put(Dec(aP.Z()));
    if (theCurve->IsRational())
    {
      Put(Dec(theCurve->Weight(i)));
    }
  }
  Put(Int(theCurve->NbKnots()));
  for (int i = 1; i <= theCurve->NbKnots(); ++i)
  {
    Put(Dec(theCurve->Knot(i)));
    Put(Int(theCurve->Multiplicity(i)));
  }
}

static void PutSurface(const occ::handle<Geom_BSplineSurface>& theSurface)
{
  Put("ud:" + Int(theSurface->UDegree()));
  Put("vd:" + Int(theSurface->VDegree()));
  Put(std::string("up:") + (theSurface->IsUPeriodic() ? "1" : "0"));
  Put(std::string("vp:") + (theSurface->IsVPeriodic() ? "1" : "0"));
  Put(std::string("ur:") + (theSurface->IsURational() ? "1" : "0"));
  Put(std::string("vr:") + (theSurface->IsVRational() ? "1" : "0"));
  Put("nu:" + Int(theSurface->NbUPoles()));
  Put("nv:" + Int(theSurface->NbVPoles()));
  // Every pole and weight goes into an FNV-1a hash of their bits (ph); the first 12 are also written out (p, w).
  const bool isRational = theSurface->IsURational() || theSurface->IsVRational();
  uint64_t   aHash      = 14695981039346656037ull;
  int        aCount     = 0;
  for (int i = 1; i <= theSurface->NbUPoles(); ++i)
  {
    for (int j = 1; j <= theSurface->NbVPoles(); ++j)
    {
      const gp_Pnt aP       = theSurface->Pole(i, j);
      const double aW       = isRational ? theSurface->Weight(i, j) : 1.0;
      const double aVals[4] = {aP.X(), aP.Y(), aP.Z(), aW};
      for (int k = 0; k < (isRational ? 4 : 3); ++k)
      {
        uint64_t aBits;
        std::memcpy(&aBits, &aVals[k], 8);
        for (int b = 0; b < 8; ++b)
        {
          aHash ^= (aBits >> (8 * b)) & 0xff;
          aHash *= 1099511628211ull;
        }
      }
      if (aCount++ < 12)
      {
        Put("p:" + Hex(aP.X()) + "," + Hex(aP.Y()) + "," + Hex(aP.Z()));
        if (isRational)
        {
          Put("w:" + Hex(aW));
        }
      }
    }
  }
  char aHashBuf[20];
  std::snprintf(aHashBuf, sizeof(aHashBuf), "%016llx", (unsigned long long)aHash);
  Put(std::string("ph:") + aHashBuf);
  for (int i = 1; i <= theSurface->NbUKnots(); ++i)
  {
    Put("uk:" + Hex(theSurface->UKnot(i)) + "," + Int(theSurface->UMultiplicity(i)));
  }
  for (int i = 1; i <= theSurface->NbVKnots(); ++i)
  {
    Put("vk:" + Hex(theSurface->VKnot(i)) + "," + Int(theSurface->VMultiplicity(i)));
  }
  const double aU0 = theSurface->UKnot(1), aU1 = theSurface->UKnot(theSurface->NbUKnots());
  const double aV0 = theSurface->VKnot(1), aV1 = theSurface->VKnot(theSurface->NbVKnots());
  for (int i = 0; i <= 4; ++i)
  {
    for (int j = 0; j <= 4; ++j)
    {
      const gp_Pnt aP = theSurface->Value(aU0 + (aU1 - aU0) * i / 4.0, aV0 + (aV1 - aV0) * j / 4.0);
      Put("g:" + Hex(aP.X()) + "," + Hex(aP.Y()) + "," + Hex(aP.Z()));
    }
  }
}

static occ::handle<Geom_BSplineCurve> Interpolate(const std::vector<gp_Pnt>&  thePoints,
                                                  const std::vector<double>&  theParams)
{
  const int                                n       = (int)thePoints.size();
  occ::handle<NCollection_HArray1<gp_Pnt>> aPoints = new NCollection_HArray1<gp_Pnt>(1, n);
  occ::handle<NCollection_HArray1<double>> aParams = new NCollection_HArray1<double>(1, n);
  for (int i = 0; i < n; ++i)
  {
    aPoints->SetValue(i + 1, thePoints[i]);
    aParams->SetValue(i + 1, theParams[i]);
  }
  GeomAPI_Interpolate anInterpolator(aPoints, aParams, false, Precision::Confusion());
  anInterpolator.Perform();
  return anInterpolator.IsDone() ? anInterpolator.Curve() : occ::handle<Geom_BSplineCurve>();
}

// A random smooth height field.
struct Height
{
  double A, Fu, Pu, Fv, Pv, B, C, D;

  void Init()
  {
    A  = Snap(Rand(-0.4, 0.4));
    Fu = Snap(Rand(0.3, 2.0));
    Pu = Snap(Rand(0.0, 1.0));
    Fv = Snap(Rand(0.3, 2.0));
    Pv = Snap(Rand(0.0, 1.0));
    B  = Snap(Rand(-0.5, 0.5));
    C  = Snap(Rand(-0.3, 0.3));
    D  = Snap(Rand(-0.3, 0.3));
  }

  double operator()(double u, double v) const
  {
    return A * std::sin(M_PI * (Fu * u + Pu)) * std::cos(M_PI * (Fv * v + Pv)) + B * u * v + C * v * v
           + D * u * u;
  }
};

static std::vector<double> SortedParams(int theNb, bool theUniform)
{
  std::vector<double> aParams(theNb);
  for (int i = 0; i < theNb; ++i)
  {
    aParams[i] = (double)i / (theNb - 1);
  }
  if (!theUniform)
  {
    for (int i = 1; i + 1 < theNb; ++i)
    {
      const double aLow  = aParams[i - 1] + 0.25 / (theNb - 1);
      const double aHigh = (double)(i + 1) / (theNb - 1) - 0.25 / (theNb - 1);
      aParams[i]         = Snap(Rand(aLow, aHigh));
    }
  }
  return aParams;
}

struct Network
{
  std::vector<occ::handle<Geom_BSplineCurve>> Profiles;
  std::vector<occ::handle<Geom_BSplineCurve>> Guides;
  std::vector<double>                         V; // profile positions
  std::vector<double>                         U; // guide positions
};

// Profiles through the grid points at the guides' U, guides through those at the profiles' V.
static bool AnalyticNetwork(int theNbProfiles, int theNbGuides, bool theUniform, Network& theNet)
{
  Height aHeight;
  aHeight.Init();
  const double aScale = Snap(Rand(0.5, 3.0));
  const gp_XYZ anOrigin(Snap(Rand(-5, 5)), Snap(Rand(-5, 5)), Snap(Rand(-5, 5)));
  theNet.V = SortedParams(theNbProfiles, theUniform);
  theNet.U = SortedParams(theNbGuides, theUniform);
  for (int i = 0; i < theNbProfiles; ++i)
  {
    std::vector<gp_Pnt> aPoints;
    for (int j = 0; j < theNbGuides; ++j)
    {
      const double u = theNet.U[j], v = theNet.V[i];
      aPoints.push_back(gp_Pnt(anOrigin + gp_XYZ(aScale * u, aScale * v, aScale * aHeight(u, v))));
    }
    occ::handle<Geom_BSplineCurve> aCurve = Interpolate(aPoints, theNet.U);
    if (aCurve.IsNull())
      return false;
    theNet.Profiles.push_back(aCurve);
  }
  for (int j = 0; j < theNbGuides; ++j)
  {
    std::vector<gp_Pnt> aPoints;
    for (int i = 0; i < theNbProfiles; ++i)
    {
      const double u = theNet.U[j], v = theNet.V[i];
      aPoints.push_back(gp_Pnt(anOrigin + gp_XYZ(aScale * u, aScale * v, aScale * aHeight(u, v))));
    }
    occ::handle<Geom_BSplineCurve> aCurve = Interpolate(aPoints, theNet.V);
    if (aCurve.IsNull())
      return false;
    theNet.Guides.push_back(aCurve);
  }
  return true;
}

static occ::handle<Geom_BSplineCurve> RationalQuadraticLine(const gp_Pnt& theP1,
                                                            const gp_Pnt& theP2,
                                                            double        theW1,
                                                            double        theW2,
                                                            double        theW3)
{
  NCollection_Array1<gp_Pnt> aPoles(1, 3);
  aPoles(1) = theP1;
  aPoles(2) = gp_Pnt((theP1.XYZ() + theP2.XYZ()) * 0.5);
  aPoles(3) = theP2;
  NCollection_Array1<double> aWeights(1, 3);
  aWeights(1) = theW1;
  aWeights(2) = theW2;
  aWeights(3) = theW3;
  NCollection_Array1<double> aKnots(1, 2);
  aKnots(1) = 0.0;
  aKnots(2) = 1.0;
  NCollection_Array1<int> aMults(1, 2);
  aMults(1) = 3;
  aMults(2) = 3;
  return new Geom_BSplineCurve(aPoles, aWeights, aKnots, aMults, 2);
}

static occ::handle<Geom_BSplineCurve> Line(const gp_Pnt& theP1, const gp_Pnt& theP2)
{
  NCollection_Array1<gp_Pnt> aPoles(1, 2);
  aPoles(1) = theP1;
  aPoles(2) = theP2;
  NCollection_Array1<double> aKnots(1, 2);
  aKnots(1) = 0.0;
  aKnots(2) = 1.0;
  NCollection_Array1<int> aMults(1, 2);
  aMults(1) = 2;
  aMults(2) = 2;
  return new Geom_BSplineCurve(aPoles, aKnots, aMults, 1);
}

static occ::handle<Geom_BSplineCurve> ClosedProfile(double theY, double theRx, double theRz, double theZ0)
{
  NCollection_Array1<gp_Pnt> aPoles(1, 9);
  aPoles(1) = gp_Pnt(theRx, theY, theZ0);
  aPoles(2) = gp_Pnt(theRx, theY, theZ0 + 0.7 * theRz);
  aPoles(3) = gp_Pnt(0.3 * theRx, theY, theZ0 + theRz);
  aPoles(4) = gp_Pnt(-0.7 * theRx, theY, theZ0 + 0.8 * theRz);
  aPoles(5) = gp_Pnt(-theRx, theY, theZ0);
  aPoles(6) = gp_Pnt(-0.7 * theRx, theY, theZ0 - 0.8 * theRz);
  aPoles(7) = gp_Pnt(0.3 * theRx, theY, theZ0 - theRz);
  aPoles(8) = gp_Pnt(theRx, theY, theZ0 - 0.7 * theRz);
  aPoles(9) = gp_Pnt(theRx, theY, theZ0);
  NCollection_Array1<double> aKnots(1, 7);
  NCollection_Array1<int>    aMults(1, 7);
  for (int k = 1; k <= 7; ++k)
  {
    aKnots(k) = (double)(k - 1) / 6.0;
    aMults(k) = 1;
  }
  aMults(1) = 4;
  aMults(7) = 4;
  return new Geom_BSplineCurve(aPoles, aKnots, aMults, 3, false);
}

static occ::handle<Geom_BSplineCurve> PeriodicProfile(double theY, double theR)
{
  NCollection_Array1<gp_Pnt> aPoles(1, 5);
  aPoles(1) = gp_Pnt(theR, theY, 0.0);
  aPoles(2) = gp_Pnt(0.309 * theR, theY, 0.951 * theR);
  aPoles(3) = gp_Pnt(-0.809 * theR, theY, 0.588 * theR);
  aPoles(4) = gp_Pnt(-0.809 * theR, theY, -0.588 * theR);
  aPoles(5) = gp_Pnt(0.309 * theR, theY, -0.951 * theR);
  NCollection_Array1<double> aKnots(1, 6);
  for (int k = 1; k <= 6; ++k)
    aKnots(k) = 0.2 * (double)(k - 1);
  NCollection_Array1<int> aMults(1, 6);
  aMults.Init(1);
  return new Geom_BSplineCurve(aPoles, aKnots, aMults, 3, true);
}

static void RunGordon(const std::string&                                 theName,
                      const std::vector<occ::handle<Geom_BSplineCurve>>& theProfiles,
                      const std::vector<occ::handle<Geom_BSplineCurve>>& theGuides,
                      double                                             theTol,
                      bool                                               theParallel,
                      bool                                               theApprox)
{
  theOut = "G " + theName;
  Put(theParallel ? "1" : "0");
  Put(theApprox ? "1" : "0");
  Put(Dec(theTol));
  Put("P");
  Put(Int((int)theProfiles.size()));
  for (const auto& aC : theProfiles)
    PutCurve(aC);
  Put("Q");
  Put(Int((int)theGuides.size()));
  for (const auto& aC : theGuides)
    PutCurve(aC);
  Put(";");

  NCollection_Array1<occ::handle<Geom_Curve>> aProfiles(1, (int)theProfiles.size());
  NCollection_Array1<occ::handle<Geom_Curve>> aGuides(1, (int)theGuides.size());
  for (int i = 0; i < (int)theProfiles.size(); ++i)
    aProfiles(i + 1) = theProfiles[i];
  for (int i = 0; i < (int)theGuides.size(); ++i)
    aGuides(i + 1) = theGuides[i];
  try
  {
    GeomFill_Gordon aGordon;
    aGordon.SetParallelMode(theParallel);
    if (theApprox)
      aGordon.SetApproximationMode(GeomFill_Gordon::ApproximationMode::AllowApproximateFallback);
    aGordon.Init(aProfiles, aGuides, theTol);
    aGordon.Perform();
    const GeomFill_Gordon::BuildReport& aReport = aGordon.Report();
    Put("st:" + Int((int)aReport.Status));
    Put("fs:" + Int((int)aReport.FailedStage));
    Put(std::string("ap:") + (aReport.IsApproximate ? "1" : "0"));
    Put("gap:" + Hex(aReport.MaxContactGap));
    Put("rep:" + Hex(aReport.MaxReparametrizationDeviation));
    Put("pd:" + Hex(aReport.MaxProfileDeviation));
    Put("gd:" + Hex(aReport.MaxGuideDeviation));
    Put("ad:" + Hex(aReport.MaxApproximationDeviation));
    if (aGordon.IsDone())
      PutSurface(aGordon.Surface());
  }
  catch (Standard_Failure const&)
  {
    Put("exc");
  }
  std::printf("%s\n", theOut.c_str());
}

static void RunNetwork(const std::string&                                 theName,
                       const std::vector<occ::handle<Geom_BSplineCurve>>& theProfiles,
                       const std::vector<occ::handle<Geom_BSplineCurve>>& theGuides,
                       const std::vector<double>&                         theV,
                       const std::vector<double>&                         theU,
                       double                                             theTol,
                       bool                                               theUClosed,
                       bool                                               theVClosed)
{
  const int n = (int)theProfiles.size(), m = (int)theGuides.size();
  theOut     = "N " + theName;
  Put(Dec(theTol));
  Put(theUClosed ? "1" : "0");
  Put(theVClosed ? "1" : "0");
  Put("P");
  Put(Int(n));
  for (const auto& aC : theProfiles)
    PutCurve(aC);
  Put("Q");
  Put(Int(m));
  for (const auto& aC : theGuides)
    PutCurve(aC);
  Put("V");
  Put(Int(n));
  for (double v : theV)
    Put(Dec(v));
  Put("U");
  Put(Int(m));
  for (double u : theU)
    Put(Dec(u));
  NCollection_Array2<gp_Pnt> aGrid(1, m, 1, n);
  NCollection_Array2<double> aWeights(1, m, 1, n);
  Put("X");
  for (int j = 1; j <= m; ++j)
  {
    for (int i = 1; i <= n; ++i)
    {
      aGrid(j, i) = theProfiles[i - 1]->Value(theU[j - 1]);
      Put(Dec(aGrid(j, i).X()));
      Put(Dec(aGrid(j, i).Y()));
      Put(Dec(aGrid(j, i).Z()));
    }
  }
  Put("W");
  for (int j = 1; j <= m; ++j)
  {
    for (int i = 1; i <= n; ++i)
    {
      aWeights(j, i) = 1.0;
      Put(Dec(1.0));
    }
  }
  Put(";");

  NCollection_Array1<occ::handle<Geom_BSplineCurve>> aProfiles(1, n), aGuides(1, m);
  NCollection_Array1<double>                         aV(1, n), aU(1, m);
  for (int i = 0; i < n; ++i)
  {
    aProfiles(i + 1) = theProfiles[i];
    aV(i + 1)        = theV[i];
  }
  for (int j = 0; j < m; ++j)
  {
    aGuides(j + 1) = theGuides[j];
    aU(j + 1)      = theU[j];
  }
  try
  {
    GeomFill_NetworkSurface aNetwork;
    aNetwork.Init(aProfiles, aGuides, aV, aU, aGrid, aWeights, theTol, theUClosed, theVClosed);
    aNetwork.Perform();
    Put("st:" + Int((int)aNetwork.Status()));
    if (aNetwork.IsDone())
      PutSurface(aNetwork.Surface());
  }
  catch (Standard_Failure const&)
  {
    Put("exc");
  }
  std::printf("%s\n", theOut.c_str());
}

template <typename T>
static void Shuffle(std::vector<T>& theValues)
{
  for (int i = (int)theValues.size() - 1; i > 0; --i)
  {
    int j = RandInt(0, i);
    std::swap(theValues[i], theValues[j]);
  }
}

static occ::handle<Geom_BSplineCurve> Reversed(const occ::handle<Geom_BSplineCurve>& theCurve)
{
  return occ::down_cast<Geom_BSplineCurve>(theCurve->Reversed());
}

int main(int argc, char** argv)
{
  if (argc > 1)
  {
    theState = std::strtoull(argv[1], nullptr, 10) * 2654435761ull + 1;
  }
  const double aTols[3] = {1.0e-7, 1.0e-6, 1.0e-5};
  char         aName[64];

  // 1. Analytic networks, uniform and non-uniform positions, 2 to 5 curves each way.
  for (int k = 0; k < 120; ++k)
  {
    Network aNet;
    const int n = RandInt(2, 5), m = RandInt(2, 5);
    if (!AnalyticNetwork(n, m, k % 2 == 0, aNet))
      continue;
    std::snprintf(aName, sizeof(aName), "grid-%d-%dx%d", k, n, m);
    RunGordon(aName, aNet.Profiles, aNet.Guides, aTols[k % 3], k % 4 == 1, false);
  }

  // 2. The same with curves reversed and the input order shuffled.
  for (int k = 0; k < 80; ++k)
  {
    Network aNet;
    const int n = RandInt(2, 5), m = RandInt(2, 5);
    if (!AnalyticNetwork(n, m, k % 3 != 0, aNet))
      continue;
    for (auto& aC : aNet.Profiles)
      if (Rand() < 0.4)
        aC = Reversed(aC);
    for (auto& aC : aNet.Guides)
      if (Rand() < 0.4)
        aC = Reversed(aC);
    Shuffle(aNet.Profiles);
    Shuffle(aNet.Guides);
    std::snprintf(aName, sizeof(aName), "shuffled-%d-%dx%d", k, n, m);
    RunGordon(aName, aNet.Profiles, aNet.Guides, aTols[k % 3], k % 2 == 0, false);
  }

  // 3. Rational quadratic line profiles with random weights, linear guides.
  for (int k = 0; k < 40; ++k)
  {
    const int                                   n = RandInt(2, 5), m = RandInt(2, 5);
    std::vector<double>                         aV = SortedParams(n, false), aU = SortedParams(m, false);
    std::vector<occ::handle<Geom_BSplineCurve>> aProfiles, aGuides;
    for (int i = 0; i < n; ++i)
      aProfiles.push_back(RationalQuadraticLine(gp_Pnt(0, aV[i], 0),
                                                gp_Pnt(1, aV[i], 0),
                                                Snap(Rand(0.6, 1.5)),
                                                Snap(Rand(0.4, 2.0)),
                                                Snap(Rand(0.6, 1.5))));
    for (int j = 0; j < m; ++j)
      aGuides.push_back(Line(gp_Pnt(aU[j], 0, 0), gp_Pnt(aU[j], 1, 0)));
    std::snprintf(aName, sizeof(aName), "rational-%d-%dx%d", k, n, m);
    RunGordon(aName, aProfiles, aGuides, 1.0e-7, false, k % 2 == 1);
  }

  // 4. Closed (non-periodic) B-spline profiles with interior linear guides.
  for (int k = 0; k < 40; ++k)
  {
    const int                                   n = RandInt(2, 4), m = RandInt(2, 5);
    std::vector<occ::handle<Geom_BSplineCurve>> aProfiles, aGuides;
    for (int i = 0; i < n; ++i)
      aProfiles.push_back(
        ClosedProfile((double)i, Snap(Rand(0.8, 1.3)), Snap(Rand(0.8, 1.3)), Snap(Rand(-0.2, 0.2))));
    std::vector<double> aU = SortedParams(m + 2, false);
    for (int j = 1; j <= m; ++j)
    {
      const double u = aU[j];
      aGuides.push_back(Interpolate(
        [&] {
          std::vector<gp_Pnt> aPts;
          for (int i = 0; i < n; ++i)
            aPts.push_back(aProfiles[i]->Value(u));
          return aPts;
        }(),
        SortedParams(n, true)));
    }
    if (n == 2)
    {
      for (auto& aG : aGuides)
        aG = Line(aG->StartPoint(), aG->EndPoint());
    }
    std::snprintf(aName, sizeof(aName), "closed-%d-%dx%d", k, n, m);
    RunGordon(aName, aProfiles, aGuides, 1.0e-7, false, false);
  }

  // 5. Periodic profiles with linear guides at random parameters.
  for (int k = 0; k < 30; ++k)
  {
    const int                                   n = RandInt(2, 4), m = RandInt(2, 4);
    std::vector<occ::handle<Geom_BSplineCurve>> aProfiles, aGuides;
    std::vector<double>                         aV = SortedParams(n, false);
    for (int i = 0; i < n; ++i)
      aProfiles.push_back(PeriodicProfile(2.0 * aV[i], Snap(Rand(0.8, 1.2))));
    std::vector<double> aU = SortedParams(m, false);
    for (int j = 0; j < m; ++j)
    {
      std::vector<gp_Pnt> aPts;
      for (int i = 0; i < n; ++i)
        aPts.push_back(aProfiles[i]->Value(aU[j]));
      aGuides.push_back(n == 2 ? Line(aPts[0], aPts[1]) : Interpolate(aPts, aV));
    }
    std::snprintf(aName, sizeof(aName), "periodic-%d-%dx%d", k, n, m);
    RunGordon(aName, aProfiles, aGuides, 1.0e-7, false, false);
  }

  // 6. NetworkSurface directly on prepared analytic networks.
  for (int k = 0; k < 50; ++k)
  {
    Network   aNet;
    const int n = RandInt(2, 5), m = RandInt(2, 5);
    if (!AnalyticNetwork(n, m, k % 2 == 0, aNet))
      continue;
    std::snprintf(aName, sizeof(aName), "network-%d-%dx%d", k, n, m);
    RunNetwork(aName, aNet.Profiles, aNet.Guides, aNet.V, aNet.U, aTols[k % 3], false, false);
  }

  // 7. NetworkSurface on U-closed quadratic loops.
  for (int k = 0; k < 20; ++k)
  {
    const int                                   n = RandInt(2, 4);
    std::vector<double>                         aV = SortedParams(n, true);
    std::vector<double>                         aU = {0.0, 0.5, 1.0};
    std::vector<occ::handle<Geom_BSplineCurve>> aProfiles, aGuides;
    for (int i = 0; i < n; ++i)
    {
      NCollection_Array1<gp_Pnt> aPoles(1, 3);
      aPoles(1) = gp_Pnt(0, 2.0 * aV[i], 0);
      aPoles(2) = gp_Pnt(Snap(Rand(0.5, 1.5)), 2.0 * aV[i], Snap(Rand(-1.0, 1.0)));
      aPoles(3) = gp_Pnt(0, 2.0 * aV[i], 0);
      NCollection_Array1<double> aKnots(1, 2);
      aKnots(1) = 0.0;
      aKnots(2) = 1.0;
      NCollection_Array1<int> aMults(1, 2);
      aMults(1) = 3;
      aMults(2) = 3;
      aProfiles.push_back(new Geom_BSplineCurve(aPoles, aKnots, aMults, 2));
    }
    for (int j = 0; j < 3; ++j)
    {
      std::vector<gp_Pnt> aPts;
      for (int i = 0; i < n; ++i)
        aPts.push_back(aProfiles[i]->Value(aU[j]));
      aGuides.push_back(n == 2 ? Line(aPts[0], aPts[1]) : Interpolate(aPts, aV));
    }
    std::snprintf(aName, sizeof(aName), "uclosed-%d-%dx3", k, n);
    RunNetwork(aName, aProfiles, aGuides, aV, aU, 1.0e-7, true, false);
  }
  return 0;
}
