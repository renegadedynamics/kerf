// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
//
// fillrec: records native OCCT's GeomFill fillings (unit 5L3): GeomFill_BSplineCurves and GeomFill_BezierCurves
// (each style, two, three and four curves, rational or not, arranged in any order and direction), GeomFill_Coons,
// Curved and Stretch directly, and GeomFill_ConstrainedFilling with GeomFill_SimpleBound and GeomFill_BoundWithSurf
// boundaries (three and four sides, Init with and without NoCheck, SetDomain and ReBuild, Eval), with the
// boundaries' own evaluation and reparametrisation, GeomFill_CoonsAlgPatch and GeomFill_TgtOnCoons.
// Random but reproducible inputs (a fixed splitmix64 stream) are written as round-trip decimals, the results as
// named tokens with every double as its 64 bits in hex. The port replays each case from the same inputs and writes
// its results in the same format (tests/OcctNet.ModelingAlgorithms.Tests/GeomFill/FillRecordings.cs).
// Built against build123d.NET's native OCCT V8_0_1 (tools/fillrec/build.ps1).
//
//   fillrec <cases per kind>   writes the recording to stdout
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

#include <Adaptor3d_CurveOnSurface.hxx>
#include <Geom2dAdaptor_Curve.hxx>
#include <Geom2d_Line.hxx>
#include <GeomAdaptor_Curve.hxx>
#include <GeomAdaptor_Surface.hxx>
#include <GeomFill_BSplineCurves.hxx>
#include <GeomFill_BezierCurves.hxx>
#include <GeomFill_BoundWithSurf.hxx>
#include <GeomFill_ConstrainedFilling.hxx>
#include <GeomFill_Coons.hxx>
#include <GeomFill_CoonsAlgPatch.hxx>
#include <GeomFill_Curved.hxx>
#include <GeomFill_SimpleBound.hxx>
#include <GeomFill_Stretch.hxx>
#include <GeomFill_TgtOnCoons.hxx>
#include <Geom_BSplineCurve.hxx>
#include <Geom_BSplineSurface.hxx>
#include <Geom_BezierCurve.hxx>
#include <Geom_BezierSurface.hxx>
#include <NCollection_Array1.hxx>
#include <NCollection_Array2.hxx>
#include <Standard_Failure.hxx>
#include <gp_Dir2d.hxx>
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

static std::string Hex(const gp_XYZ& theP) { return Hex(theP.X()) + "," + Hex(theP.Y()) + "," + Hex(theP.Z()); }

static std::string Hex(const gp_Pnt& theP) { return Hex(theP.XYZ()); }

static std::string Hex(const gp_Vec& theV) { return Hex(theV.XYZ()); }

// FNV-1a over the 64 bits of each value, for the large pole grids
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

// Runs an accessor, writing name:value or name:!Exception
static void Tok(const std::string& theName, const std::function<std::string()>& theF)
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
  Put(theName + ":" + aVal);
}

//=================================================================================================
// Inputs

static std::vector<double> theIn;

static double In(double theValue)
{
  theIn.push_back(theValue);
  return theValue;
}

// Writes a point's coordinates in order (a constructor's arguments are evaluated in an unspecified order)
static gp_Pnt InPnt(const gp_Pnt& theP)
{
  double x = In(theP.X());
  double y = In(theP.Y());
  double z = In(theP.Z());
  return gp_Pnt(x, y, z);
}

static uint64_t theState = 0x5135F111ULL;

static uint64_t Next()
{
  uint64_t z = (theState += 0x9e3779b97f4a7c15ULL);
  z          = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ULL;
  z          = (z ^ (z >> 27)) * 0x94d049bb133111ebULL;
  return z ^ (z >> 31);
}

// A uniform double in [a, b)
static double Rnd(double a, double b) { return a + (b - a) * (double)(Next() >> 11) * (1.0 / 9007199254740992.0); }

// A uniform integer in [a, b]
static int Irnd(int a, int b) { return a + (int)(Next() % (uint64_t)(b - a + 1)); }

static gp_Pnt RndPnt(double theR) { return gp_Pnt(Rnd(-theR, theR), Rnd(-theR, theR), Rnd(-theR, theR)); }

//=================================================================================================
// Curves: generated, written to the inputs and built

// A B-spline curve from theA to theB, written as: deg rat np (x y z [w])* nk (k m)*
static occ::handle<Geom_BSplineCurve> MakeBSpline(const gp_Pnt& theA, const gp_Pnt& theB, bool theDegenerate = false)
{
  int  aDeg = Irnd(1, 4);
  int  aNbI = Irnd(0, 3);
  bool aRat = Irnd(0, 9) < 4;
  // interior knots: distinct multiples of 1/16 of the range
  std::vector<int> aSlots;
  while ((int)aSlots.size() < aNbI)
  {
    int s     = Irnd(1, 15);
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

  In(aDeg);
  In(aRat ? 1 : 0);
  In(aNbP);
  NCollection_Array1<gp_Pnt> aPoles(1, aNbP);
  NCollection_Array1<double> aW(1, aNbP);
  for (int i = 1; i <= aNbP; i++)
  {
    gp_Pnt aP;
    if (theDegenerate)
      aP = theA;
    else if (i == 1)
      aP = theA;
    else if (i == aNbP)
      aP = theB;
    else
    {
      double f = (double)(i - 1) / (aNbP - 1);
      aP       = gp_Pnt(theA.XYZ() * (1 - f) + theB.XYZ() * f + RndPnt(2.0).XYZ());
    }
    aPoles(i) = InPnt(aP);
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

// A Bezier curve from theA to theB, written as: rat np (x y z [w])*
static occ::handle<Geom_BezierCurve> MakeBezier(const gp_Pnt& theA, const gp_Pnt& theB, bool theDegenerate = false)
{
  int  aNbP = Irnd(2, 6);
  bool aRat = Irnd(0, 9) < 4;
  In(aRat ? 1 : 0);
  In(aNbP);
  NCollection_Array1<gp_Pnt> aPoles(1, aNbP);
  NCollection_Array1<double> aW(1, aNbP);
  for (int i = 1; i <= aNbP; i++)
  {
    gp_Pnt aP;
    if (theDegenerate || i == 1)
      aP = theA;
    else if (i == aNbP)
      aP = theB;
    else
    {
      double f = (double)(i - 1) / (aNbP - 1);
      aP       = gp_Pnt(theA.XYZ() * (1 - f) + theB.XYZ() * f + RndPnt(2.0).XYZ());
    }
    aPoles(i) = InPnt(aP);
    if (aRat)
      aW(i) = In(Rnd(0.5, 2.0));
  }
  if (aRat)
    return new Geom_BezierCurve(aPoles, aW);
  return new Geom_BezierCurve(aPoles);
}

// A contour A -> B -> C -> D -> A; theCorners[3] may equal theCorners[2] (a degenerated side)
static void Corners(gp_Pnt theCorners[4], bool& theDegenerated)
{
  double L        = Rnd(3, 12);
  double M        = Rnd(3, 12);
  theCorners[0]   = gp_Pnt(Rnd(-1, 1), Rnd(-1, 1), Rnd(-1, 1));
  theCorners[1]   = gp_Pnt(L + Rnd(-1, 1), Rnd(-1, 1), Rnd(-2, 2));
  theCorners[2]   = gp_Pnt(L + Rnd(-1, 1), M + Rnd(-1, 1), Rnd(-2, 2));
  theCorners[3]   = gp_Pnt(Rnd(-1, 1), M + Rnd(-1, 1), Rnd(-2, 2));
  theDegenerated  = Irnd(0, 9) == 0;
  if (theDegenerated)
    theCorners[3] = theCorners[2];
}

//=================================================================================================
// Results

static void PutBSplineSurface(const occ::handle<Geom_BSplineSurface>& theS)
{
  if (theS.IsNull())
  {
    Put("srf:null");
    return;
  }
  Put("sdeg:" + std::to_string(theS->UDegree()) + "," + std::to_string(theS->VDegree()) + ","
      + std::to_string(theS->NbUPoles()) + "," + std::to_string(theS->NbVPoles()) + ","
      + std::to_string(theS->IsURational() ? 1 : 0) + std::to_string(theS->IsVRational() ? 1 : 0));
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
    {
      gp_Pnt P;
      gp_Vec DU, DV;
      theS->D1(u1 + (u2 - u1) * i / 2.0, v1 + (v2 - v1) * j / 2.0, P, DU, DV);
      aD += (aD.empty() ? "" : ",") + Hex(P) + "," + Hex(DU) + "," + Hex(DV);
    }
  Put("sd1:" + aD);
}

static void PutBezierSurface(const occ::handle<Geom_BezierSurface>& theS)
{
  if (theS.IsNull())
  {
    Put("srf:null");
    return;
  }
  Put("sdeg:" + std::to_string(theS->UDegree()) + "," + std::to_string(theS->VDegree()) + ","
      + std::to_string(theS->IsURational() ? 1 : 0) + std::to_string(theS->IsVRational() ? 1 : 0));
  Digest aP, aW;
  for (int i = 1; i <= theS->NbUPoles(); i++)
    for (int j = 1; j <= theS->NbVPoles(); j++)
    {
      aP.Add(theS->Pole(i, j));
      aW.Add(theS->Weight(i, j));
    }
  Put("spol:" + aP.Str());
  Put("swt:" + aW.Str());
  std::string aD;
  for (int i = 0; i <= 2; i++)
    for (int j = 0; j <= 2; j++)
    {
      gp_Pnt P;
      gp_Vec DU, DV;
      theS->D1(i / 2.0, j / 2.0, P, DU, DV);
      aD += (aD.empty() ? "" : ",") + Hex(P) + "," + Hex(DU) + "," + Hex(DV);
    }
  Put("sd1:" + aD);
}

static void PutFilling(const GeomFill_Filling& theF)
{
  Put("nb:" + std::to_string(theF.NbUPoles()) + "," + std::to_string(theF.NbVPoles()) + ","
      + std::to_string(theF.isRational() ? 1 : 0));
  NCollection_Array2<gp_Pnt> aP(1, theF.NbUPoles(), 1, theF.NbVPoles());
  theF.Poles(aP);
  std::string aS;
  for (int i = aP.LowerRow(); i <= aP.UpperRow(); i++)
    for (int j = aP.LowerCol(); j <= aP.UpperCol(); j++)
      aS += (aS.empty() ? "" : ",") + Hex(aP(i, j));
  Put("pol:" + aS);
  if (theF.isRational())
  {
    NCollection_Array2<double> aW(1, theF.NbUPoles(), 1, theF.NbVPoles());
    theF.Weights(aW);
    std::string aT;
    for (int i = aW.LowerRow(); i <= aW.UpperRow(); i++)
      for (int j = aW.LowerCol(); j <= aW.UpperCol(); j++)
        aT += (aT.empty() ? "" : ",") + Hex(aW(i, j));
    Put("wt:" + aT);
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

static const GeomFill_FillingStyle THE_STYLES[] = {GeomFill_StretchStyle,
                                                   GeomFill_CoonsStyle,
                                                   GeomFill_CurvedStyle};

// GeomFill_BSplineCurves / GeomFill_BezierCurves on 2, 3 or 4 curves
template <class TheCurve, class TheFill, class TheMaker>
static void CurvesCase(int theNbCurves, TheMaker theMake, const std::function<void(TheFill&)>& thePut)
{
  int aStyle = (int)In(Irnd(0, 2));
  gp_Pnt aC[4];
  bool   aDegen;
  Corners(aC, aDegen);
  std::vector<occ::handle<TheCurve>> aCurves;
  if (theNbCurves >= 3)
  {
    // side i from corner i to corner i+1, written in a random order and direction (side 0 first)
    int aPerm[3] = {1, 2, 3};
    for (int i = 2; i > 0; i--)
      std::swap(aPerm[i], aPerm[Irnd(0, i)]);
    int aSides[4] = {0, aPerm[0], aPerm[1], aPerm[2]};
    for (int k = 0; k < theNbCurves; k++)
    {
      int s = aSides[k];
      if (theNbCurves == 3)
        s = k; // three consecutive sides
      bool aRev = k > 0 && Irnd(0, 1) == 1;
      In(aRev ? 1 : 0);
      gp_Pnt a = aC[s], b = aC[(s + 1) % 4];
      if (aRev)
        std::swap(a, b);
      aCurves.push_back(theMake(a, b, aDegen && s == 2));
    }
  }
  else
  {
    // two curves; for the Curved style they share an end (one of four ways)
    int    aWay = (int)In(Irnd(0, 3));
    gp_Pnt a = aC[0], b = aC[1], e = aC[3];
    aCurves.push_back(theMake(a, b, false));
    if (THE_STYLES[aStyle] != GeomFill_CurvedStyle)
      aCurves.push_back(theMake(aC[3], aC[2], false));
    else if (aWay == 0)
      aCurves.push_back(theMake(a, e, false));
    else if (aWay == 1)
      aCurves.push_back(theMake(e, a, false));
    else if (aWay == 2)
      aCurves.push_back(theMake(b, e, false));
    else
      aCurves.push_back(theMake(e, b, false));
  }
  try
  {
    TheFill aFill;
    if (theNbCurves == 4)
      aFill.Init(aCurves[0], aCurves[1], aCurves[2], aCurves[3], THE_STYLES[aStyle]);
    else if (theNbCurves == 3)
      aFill.Init(aCurves[0], aCurves[1], aCurves[2], THE_STYLES[aStyle]);
    else
      aFill.Init(aCurves[0], aCurves[1], THE_STYLES[aStyle]);
    thePut(aFill);
  }
  catch (const Standard_Failure& theF)
  {
    Put("init:" + ExcName(theF));
  }
}

// GeomFill_Coons, Curved (both forms) and Stretch on random rows of poles
static void FillingCase(const char* theKind)
{
  std::string aKind = theKind;
  bool        aRat  = In(Irnd(0, 1)) != 0;
  int         NU    = (int)In(Irnd(2, 7));
  int         NV    = (int)In(Irnd(2, 7));
  bool        aTwo  = aKind == "CURVED2";
  int         aNb   = aTwo ? 2 : 4;
  NCollection_Array1<gp_Pnt> P[4] = {NCollection_Array1<gp_Pnt>(1, NU),
                                     NCollection_Array1<gp_Pnt>(1, NV),
                                     NCollection_Array1<gp_Pnt>(1, NU),
                                     NCollection_Array1<gp_Pnt>(1, NV)};
  NCollection_Array1<double> W[4] = {NCollection_Array1<double>(1, NU),
                                     NCollection_Array1<double>(1, NV),
                                     NCollection_Array1<double>(1, NU),
                                     NCollection_Array1<double>(1, NV)};
  for (int k = 0; k < aNb; k++)
    for (int i = P[k].Lower(); i <= P[k].Upper(); i++)
    {
      gp_Pnt aP = RndPnt(10.0);
      P[k](i)   = InPnt(aP);
    }
  if (aRat)
    for (int k = 0; k < aNb; k++)
      for (int i = W[k].Lower(); i <= W[k].Upper(); i++)
        W[k](i) = In(Rnd(0.5, 2.0));
  try
  {
    if (aKind == "COONS")
    {
      GeomFill_Coons F;
      if (aRat)
        F.Init(P[0], P[1], P[2], P[3], W[0], W[1], W[2], W[3]);
      else
        F.Init(P[0], P[1], P[2], P[3]);
      PutFilling(F);
    }
    else if (aKind == "STRETCH")
    {
      GeomFill_Stretch F;
      if (aRat)
        F.Init(P[0], P[1], P[2], P[3], W[0], W[1], W[2], W[3]);
      else
        F.Init(P[0], P[1], P[2], P[3]);
      PutFilling(F);
    }
    else if (aKind == "CURVED")
    {
      GeomFill_Curved F;
      if (aRat)
        F.Init(P[0], P[1], P[2], P[3], W[0], W[1], W[2], W[3]);
      else
        F.Init(P[0], P[1], P[2], P[3]);
      PutFilling(F);
    }
    else
    {
      GeomFill_Curved F;
      if (aRat)
        F.Init(P[0], P[1], W[0], W[1]);
      else
        F.Init(P[0], P[1]);
      PutFilling(F);
    }
  }
  catch (const Standard_Failure& theF)
  {
    Put("init:" + ExcName(theF));
  }
}

//=================================================================================================
// ConstrainedFilling: the four sides of a random bicubic patch S on [0,1]^2 as SimpleBounds (S's iso curves) or
// BoundWithSurf (2d lines on S)

static occ::handle<Geom_BSplineSurface> MakeSurface()
{
  int NU = (int)In(Irnd(4, 5));
  int NV = (int)In(Irnd(4, 5));
  NCollection_Array2<gp_Pnt> aP(1, NU, 1, NV);
  double                     L = Rnd(4, 10), M = Rnd(4, 10);
  for (int i = 1; i <= NU; i++)
    for (int j = 1; j <= NV; j++)
    {
      gp_Pnt aQ(L * (i - 1) / (NU - 1) + Rnd(-0.5, 0.5), M * (j - 1) / (NV - 1) + Rnd(-0.5, 0.5), Rnd(-2, 2));
      aP(i, j) = InPnt(aQ);
    }
  auto aKnots = [](int theNb, NCollection_Array1<double>& theK, NCollection_Array1<int>& theM) {
    // cubic: NU = 4 -> one span, NU = 5 -> two spans
    int aNbK = theNb - 2;
    theK.Resize(1, aNbK, false);
    theM.Resize(1, aNbK, false);
    for (int i = 1; i <= aNbK; i++)
    {
      theK(i) = (double)(i - 1) / (aNbK - 1);
      theM(i) = (i == 1 || i == aNbK) ? 4 : 1;
    }
  };
  NCollection_Array1<double> UK(1, 1), VK(1, 1);
  NCollection_Array1<int>    UM(1, 1), VM(1, 1);
  aKnots(NU, UK, UM);
  aKnots(NV, VK, VM);
  return new Geom_BSplineSurface(aP, UK, VK, UM, VM, 3, 3);
}

// Side k of S: 0 v=0 (u 0->1), 1 u=1 (v 0->1), 2 v=1 (u 0->1), 3 u=0 (v 0->1)
static occ::handle<GeomFill_Boundary> MakeBound(const occ::handle<Geom_BSplineSurface>& theS,
                                                int                                     theSide,
                                                int                                     theType,
                                                double                                  theTol3d,
                                                double                                  theTolAng)
{
  if (theType == 0)
  {
    occ::handle<Geom_Curve> anIso = (theSide == 0)   ? theS->VIso(0.0)
                                    : (theSide == 1) ? theS->UIso(1.0)
                                    : (theSide == 2) ? theS->VIso(1.0)
                                                     : theS->UIso(0.0);
    occ::handle<GeomAdaptor_Curve> aC = new GeomAdaptor_Curve(anIso);
    return new GeomFill_SimpleBound(aC, theTol3d, theTolAng);
  }
  gp_Pnt2d aP = (theSide == 0) ? gp_Pnt2d(0, 0) : (theSide == 1) ? gp_Pnt2d(1, 0) : (theSide == 2) ? gp_Pnt2d(0, 1) : gp_Pnt2d(0, 0);
  gp_Dir2d aD = (theSide == 0 || theSide == 2) ? gp_Dir2d(1, 0) : gp_Dir2d(0, 1);
  occ::handle<Geom2d_Line>         aL  = new Geom2d_Line(aP, aD);
  occ::handle<Geom2dAdaptor_Curve> aC2 = new Geom2dAdaptor_Curve(aL, 0.0, 1.0);
  occ::handle<GeomAdaptor_Surface> aSA = new GeomAdaptor_Surface(theS);
  Adaptor3d_CurveOnSurface         aCoS(aC2, aSA);
  return new GeomFill_BoundWithSurf(aCoS, theTol3d, theTolAng);
}

static void ConstrainedCase()
{
  int    aNbSides = (int)In(Irnd(3, 4));
  int    aMaxDeg  = (int)In(Irnd(8, 11));
  int    aMaxSeg  = (int)In(Irnd(6, 20));
  bool   aNoCheck = In(Irnd(0, 1)) != 0;
  double aTol3d   = In(Irnd(0, 1) ? 1.0e-4 : 1.0e-3);
  double aTolAng  = In(Irnd(0, 1) ? 1.0e-2 : 0.1);
  occ::handle<Geom_BSplineSurface> S = MakeSurface();
  int                              aType[4];
  for (int k = 0; k < 4; k++)
    aType[k] = (int)In(Irnd(0, 2) == 0 ? 0 : 1);
  int aOrder[4] = {0, 1, 2, 3};
  for (int i = 3; i > 0; i--)
    std::swap(aOrder[i], aOrder[Irnd(0, i)]);
  for (int k = 0; k < 4; k++)
    In(aOrder[k]);
  bool   aHasDF = In(Irnd(0, 1)) != 0, aHasDL = In(Irnd(0, 1)) != 0;
  double aDF = In(Rnd(0.2, 2.0)), aDL = In(Rnd(0.2, 2.0));
  bool   aRev    = In(Irnd(0, 1)) != 0;
  bool   aDomain = In(Irnd(0, 1)) != 0;
  int    aDomSide = (int)In(Irnd(0, aNbSides - 1));
  double aDomL    = In(Rnd(0.2, 0.9));

  // the boundaries alone
  for (int k = 0; k < 4; k++)
  {
    occ::handle<GeomFill_Boundary> B  = MakeBound(S, k, aType[k], aTol3d, aTolAng);
    std::string                    aK = std::to_string(k);
    Tok("b" + aK, [&] {
      std::string s;
      for (double w : {0.0, 0.37, 1.0})
      {
        gp_Pnt P;
        gp_Vec V;
        B->D1(w, P, V);
        s += (s.empty() ? "" : ",") + Hex(B->Value(w)) + "," + Hex(P) + "," + Hex(V);
        if (B->HasNormals())
        {
          gp_Vec N, DN;
          B->D1Norm(w, N, DN);
          s += "," + Hex(B->Norm(w)) + "," + Hex(N) + "," + Hex(DN);
        }
      }
      return s;
    });
    Tok("r" + aK, [&] {
      B->Reparametrize(0.0, 1.0, aHasDF, aHasDL, aDF, aDL, aRev);
      double f, l;
      B->Bounds(f, l);
      gp_Pnt P, P1, P2;
      gp_Vec V;
      B->D1(0.21, P, V);
      B->Points(P1, P2);
      return Hex(f) + "," + Hex(l) + "," + Hex(B->Value(0.21)) + "," + Hex(P) + "," + Hex(V) + "," + Hex(P1) + ","
             + Hex(P2);
    });
  }

  // the algorithmic patch and its tangent fields
  {
    occ::handle<GeomFill_Boundary> B[4];
    for (int k = 0; k < 4; k++)
      B[k] = MakeBound(S, k, aType[k], aTol3d, aTolAng);
    occ::handle<GeomFill_CoonsAlgPatch> K = new GeomFill_CoonsAlgPatch(B[0], B[1], B[2], B[3]);
    Tok("pc", [&] { return Hex(K->Corner(0)) + "," + Hex(K->Corner(1)) + "," + Hex(K->Corner(2)) + "," + Hex(K->Corner(3)); });
    Tok("pv", [&] {
      std::string s;
      for (double u : {0.25, 0.5, 0.9})
        for (double v : {0.1, 0.5, 0.75})
          s += (s.empty() ? "" : ",") + Hex(K->Value(u, v)) + "," + Hex(K->D1U(u, v)) + "," + Hex(K->D1V(u, v)) + ","
               + Hex(K->DUV(u, v));
      return s;
    });
    for (int k = 0; k < 4; k++)
    {
      if (!B[k]->HasNormals())
        continue;
      occ::handle<GeomFill_TgtOnCoons> T = new GeomFill_TgtOnCoons(K, k);
      Tok("t" + std::to_string(k), [&] {
        std::string s;
        for (double w : {0.3, 0.7})
        {
          gp_Vec V, DV;
          T->D1(w, V, DV);
          s += (s.empty() ? "" : ",") + Hex(T->Value(w)) + "," + Hex(T->D1(w)) + "," + Hex(V) + "," + Hex(DV);
        }
        return s;
      });
    }
  }

  // the filling
  occ::handle<GeomFill_Boundary> B[4];
  for (int k = 0; k < 4; k++)
    B[k] = MakeBound(S, k, aType[k], aTol3d, aTolAng);
  // the sides used, in the written order (three sides: the first three of the order that are consecutive is not
  // required; sortbounds chains them)
  std::vector<occ::handle<GeomFill_Boundary>> aUsed;
  for (int k = 0; k < 4; k++)
    if (aNbSides == 4 || aOrder[k] != 3)
      aUsed.push_back(B[aOrder[k]]);
  GeomFill_ConstrainedFilling CF(aMaxDeg, aMaxSeg);
  bool                        anOK = true;
  try
  {
    if (aNbSides == 4)
      CF.Init(aUsed[0], aUsed[1], aUsed[2], aUsed[3], aNoCheck);
    else
      CF.Init(aUsed[0], aUsed[1], aUsed[2], aNoCheck);
  }
  catch (const Standard_Failure& theF)
  {
    Put("init:" + ExcName(theF));
    anOK = false;
  }
  if (!anOK)
    return;
  PutBSplineSurface(CF.Surface());
  Tok("bnd", [&] {
    std::string s;
    for (int i = 0; i < 4; i++)
    {
      occ::handle<GeomFill_Boundary> b = CF.Boundary(i);
      double                         f, l;
      b->Bounds(f, l);
      s += (s.empty() ? "" : ",") + Hex(f) + "," + Hex(l) + "," + Hex(b->Value(0.5)) + "," + std::to_string(b->IsDegenerated() ? 1 : 0);
    }
    return s;
  });
  Tok("eval", [&] {
    std::string s;
    for (double w : {0.0, 0.3, 1.0})
      for (int ord = 0; ord <= 1; ord++)
      {
        double r[12] = {0};
        CF.Eval(w, ord, r[0]);
        for (double x : r)
          s += (s.empty() ? "" : ",") + Hex(x);
      }
    return s;
  });
  if (aDomain)
  {
    occ::handle<GeomFill_BoundWithSurf> aBWS = occ::down_cast<GeomFill_BoundWithSurf>(CF.Boundary(aDomSide));
    if (!aBWS.IsNull())
    {
      Tok("rebuild", [&] {
        CF.SetDomain(aDomL, aBWS);
        CF.ReBuild();
        return std::string("ok");
      });
      PutBSplineSurface(CF.Surface());
    }
  }
}

int main(int argc, char** argv)
{
  int aNb = argc > 1 ? std::atoi(argv[1]) : 100;
  auto aBS = [](const gp_Pnt& a, const gp_Pnt& b, bool d) { return MakeBSpline(a, b, d); };
  auto aBZ = [](const gp_Pnt& a, const gp_Pnt& b, bool d) { return MakeBezier(a, b, d); };
  std::function<void(GeomFill_BSplineCurves&)> aPutBS = [](GeomFill_BSplineCurves& f) { PutBSplineSurface(f.Surface()); };
  std::function<void(GeomFill_BezierCurves&)>  aPutBZ = [](GeomFill_BezierCurves& f) {
    occ::handle<Geom_BezierSurface> aS = occ::down_cast<Geom_BezierSurface>(f.Surface());
    PutBezierSurface(aS);
  };
  for (int n = 4; n >= 2; n--)
  {
    std::string aBSKind = "BSC" + std::to_string(n), aBZKind = "BZC" + std::to_string(n);
    for (int i = 0; i < aNb; i++)
    {
      CurvesCase<Geom_BSplineCurve, GeomFill_BSplineCurves>(n, aBS, aPutBS);
      Line(aBSKind.c_str());
    }
    for (int i = 0; i < aNb; i++)
    {
      CurvesCase<Geom_BezierCurve, GeomFill_BezierCurves>(n, aBZ, aPutBZ);
      Line(aBZKind.c_str());
    }
  }
  for (const char* aKind : {"COONS", "CURVED", "STRETCH", "CURVED2"})
    for (int i = 0; i < aNb; i++)
    {
      FillingCase(aKind);
      Line(aKind);
    }
  for (int i = 0; i < aNb; i++)
  {
    ConstrainedCase();
    Line("CF");
  }
  return 0;
}
