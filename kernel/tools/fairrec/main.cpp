// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
//
// fairrec: records native OCCT's fair curves (FairCurve, unit 5Q) on random configurations: FairCurve_Batten and
// FairCurve_MinimalVariation through their setters and successive Computes (the homotopy from the previous state),
// Draw's battencurve/minvarcurve sequences (GeometryTest_FairCurveCommands, DrawFairCurve_*), and the energies
// FairCurve_EnergyOfBatten and FairCurve_EnergyOfMVC (value, gradient, hessian) on random poles. Every double is
// written as its 64 bits in hex; the inputs as round-trip decimals. The port replays each case from the same inputs
// and writes its results in the same format (tests/OcctNet.ModelingAlgorithms.Tests/FairCurve).
// Built against build123d.NET's native OCCT V8_0_1 (tools/fairrec/build.ps1).
//
//   fairrec <cases per kind>   writes the recording to stdout
//
// One line per case:  <kind> <inputs...> ; <tokens...>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <functional>
#include <string>
#include <typeinfo>
#include <vector>

#include <BSplCLib.hxx>
#include <FairCurve_Batten.hxx>
#include <FairCurve_BattenLaw.hxx>
#include <FairCurve_EnergyOfBatten.hxx>
#include <FairCurve_EnergyOfMVC.hxx>
#include <FairCurve_MinimalVariation.hxx>
#include <Geom2d_BSplineCurve.hxx>
#include <NCollection_Array1.hxx>
#include <NCollection_HArray1.hxx>
#include <Standard_Failure.hxx>
#include <gp_Pnt2d.hxx>
#include <math_Matrix.hxx>
#include <math_Vector.hxx>

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

static std::string ExcName(const Standard_Failure& theF)
{
  std::string aName = typeid(theF).name(); // "class Standard_OutOfRange"
  if (aName.rfind("class ", 0) == 0)
  {
    aName = aName.substr(6);
  }
  return "!" + aName;
}

// Runs an action or accessor, writing name:value or name:!Exception
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

static std::string Curve(const occ::handle<Geom2d_BSplineCurve>& theC)
{
  std::string aS = std::to_string(theC->Degree()) + "," + std::to_string(theC->NbPoles());
  for (int i = 1; i <= theC->NbPoles(); i++)
  {
    aS += "," + Hex(theC->Pole(i));
  }
  aS += "," + std::to_string(theC->NbKnots());
  for (int i = 1; i <= theC->NbKnots(); i++)
  {
    aS += "," + Hex(theC->Knot(i)) + "," + std::to_string(theC->Multiplicity(i));
  }
  return aS;
}

static std::string Vec(const math_Vector& theV)
{
  std::string aS = std::to_string(theV.Lower()) + "," + std::to_string(theV.Upper());
  for (int i = theV.Lower(); i <= theV.Upper(); i++)
  {
    aS += "," + Hex(theV(i));
  }
  return aS;
}

static std::string Mat(const math_Matrix& theM)
{
  std::string aS = std::to_string(theM.RowNumber());
  for (int i = theM.LowerRow(); i <= theM.UpperRow(); i++)
  {
    for (int j = theM.LowerCol(); j <= theM.UpperCol(); j++)
    {
      aS += "," + Hex(theM(i, j));
    }
  }
  return aS;
}

//=================================================================================================
// Random inputs, rounded to multiples of 2^-10 so that they print short and exactly.

static uint64_t theState = 0xFA1Bu;

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

static std::vector<double> theIn;

static double In(double theV)
{
  theIn.push_back(theV);
  return theV;
}

static int InI(int theV) { return (int)In(theV); }

static gp_Pnt2d Pnt()
{
  double x = In(I(3) == 0 ? (double)(I(9) - 4) : Grid(-8.0 + 16.0 * U()));
  double y = In(I(3) == 0 ? (double)(I(9) - 4) : Grid(-8.0 + 16.0 * U()));
  return gp_Pnt2d(x, y);
}

// an angle in radians from degrees: often a multiple of 15, sometimes beyond 90
static double Angle()
{
  int    k = I(10);
  double d = k < 4 ? 15.0 * (I(13) - 6) : (k < 9 ? Grid(-100.0 + 200.0 * U()) : Grid(-170.0 + 340.0 * U()));
  return In(d) * M_PI / 180;
}

static double Height()
{
  int k = I(10);
  return In(k < 3 ? 0.1 * (1 + I(5)) : (k < 9 ? Grid(0.02 + 1.5 * U()) : -0.5 * I(2)));
}

static double Slope()
{
  int k = I(10);
  return In(k < 6 ? 0.0 : Grid(-1.0 + 2.0 * U()));
}

static int Order(int theMax) { return InI(I(5) == 0 ? I(theMax + 1) : 1); }

static double Factor()
{
  int k = I(4);
  return In(k == 0 ? 1.0 : (k == 1 ? Grid(0.7 + 0.8 * U()) : 0.5 * (1 + I(4))));
}

static int Iterations()
{
  int k = I(6);
  return InI(k < 4 ? 50 : (k == 4 ? 10 : 100));
}

static double Tolerance()
{
  int k = I(6);
  return In(k < 4 ? 1.0e-3 : (k == 4 ? 1.0e-2 : 1.0e-4));
}

static double Ratio()
{
  int k = I(12);
  return In(k < 4 ? 0.0 : (k < 7 ? 1.0 : (k < 11 ? Grid(U()) : 1.5)));
}

static double Curvature()
{
  int k = I(4);
  return In(k == 0 ? 0.0 : Grid(-1.0 + 2.0 * U()));
}

//=================================================================================================

static void State(FairCurve_Batten& theB, const char* thePrefix, const FairCurve_MinimalVariation* theMV)
{
  std::string p = thePrefix;
  Tok((p + "a1").c_str(), [&] { return Hex(theB.GetAngle1()); });
  Tok((p + "a2").c_str(), [&] { return Hex(theB.GetAngle2()); });
  Tok((p + "sf").c_str(), [&] { return Hex(theB.GetSlidingFactor()); });
  Tok((p + "p1").c_str(), [&] { return Hex(theB.GetP1()); });
  Tok((p + "p2").c_str(), [&] { return Hex(theB.GetP2()); });
  Tok((p + "h").c_str(), [&] { return Hex(theB.GetHeight()); });
  Tok((p + "sl").c_str(), [&] { return Hex(theB.GetSlope()); });
  Tok((p + "o").c_str(), [&] {
    return std::to_string(theB.GetConstraintOrder1()) + "," + std::to_string(theB.GetConstraintOrder2()) + ","
           + std::to_string(theB.GetFreeSliding() ? 1 : 0);
  });
  if (theMV != nullptr)
  {
    Tok((p + "c").c_str(), [&] {
      return Hex(theMV->GetCurvature1()) + "," + Hex(theMV->GetCurvature2()) + "," + Hex(theMV->GetPhysicalRatio());
    });
  }
  Tok((p + "ref").c_str(), [&] { return Hex(theB.SlidingOfReference()); });
  Tok((p + "curve").c_str(), [&] { return Curve(theB.Curve()); });
}

static void Compute(FairCurve_Batten& theB, const char* thePrefix, int theNbIter, double theTol)
{
  Tok(thePrefix, [&] {
    FairCurve_AnalysisCode aCode = FairCurve_OK;
    bool                   anOk  = theB.Compute(aCode, theNbIter, theTol);
    return std::to_string(anOk ? 1 : 0) + "," + std::to_string((int)aCode);
  });
}

// Batten or MinimalVariation through the setters, computed, then changed and computed again
static void RunFair(bool theMV)
{
  gp_Pnt2d P1 = Pnt(), P2 = Pnt();
  double   h = Height(), s = Slope();
  double   r = theMV ? Ratio() : 0.0;

  FairCurve_Batten*           aB  = nullptr;
  FairCurve_MinimalVariation* aMV = nullptr;
  try
  {
    if (theMV)
    {
      aMV = new FairCurve_MinimalVariation(P1, P2, h, s, r);
      aB  = aMV;
    }
    else
    {
      aB = new FairCurve_Batten(P1, P2, h, s);
    }
  }
  catch (const Standard_Failure& theF)
  {
    Put("ctor:" + ExcName(theF));
    return;
  }

  double a1 = Angle(), a2 = Angle();
  int    o1 = Order(theMV ? 2 : 1), o2 = Order(theMV ? 2 : 1);
  bool   fs = In(I(2)) != 0.0;
  double f  = Factor();
  aB->SetAngle1(a1);
  aB->SetAngle2(a2);
  aB->SetConstraintOrder1(o1);
  aB->SetConstraintOrder2(o2);
  aB->SetFreeSliding(fs);
  aB->SetSlidingFactor(f);
  if (theMV)
  {
    double c1 = Curvature(), c2 = Curvature();
    aMV->SetCurvature1(c1);
    aMV->SetCurvature2(c2);
  }
  Tok("ref0", [&] { return Hex(aB->SlidingOfReference()); });
  int    n = Iterations();
  double t = Tolerance();
  Compute(*aB, "c1", n, t);
  State(*aB, "", aMV);

  if (In(I(2)) != 0.0)
  {
    gp_Pnt2d Q2 = Pnt();
    double   b1 = Angle();
    double   h2 = Height();
    double   s2 = Slope();
    Tok("setp2", [&] {
      aB->SetP2(Q2);
      return std::string("1");
    });
    aB->SetAngle1(b1);
    Tok("seth", [&] {
      aB->SetHeight(h2);
      return std::string("1");
    });
    aB->SetSlope(s2);
    if (theMV)
    {
      double r2 = Ratio();
      Tok("setr", [&] {
        aMV->SetPhysicalRatio(r2);
        return std::string("1");
      });
    }
    Compute(*aB, "c2", 50, 1.0e-3);
    State(*aB, "2", aMV);
  }
  delete aB;
}

// Draw's battencurve/minvarcurve, then a few of Draw's editing commands (each recomputes with 50, 1.0e-2)
static void RunDraw(bool theMV)
{
  gp_Pnt2d P1 = Pnt(), P2 = Pnt();
  double   a1 = In(Grid(-120.0 + 240.0 * U())), a2 = In(Grid(-120.0 + 240.0 * U()));
  double   h  = Height();

  FairCurve_Batten*           aB  = nullptr;
  FairCurve_MinimalVariation* aMV = nullptr;
  try
  {
    if (theMV)
    {
      aMV = new FairCurve_MinimalVariation(P1, P2, h);
      aB  = aMV;
    }
    else
    {
      aB = new FairCurve_Batten(P1, P2, h);
    }
  }
  catch (const Standard_Failure& theF)
  {
    Put("ctor:" + ExcName(theF));
    return;
  }
  aB->SetAngle1(a1 * M_PI / 180);
  aB->SetAngle2(a2 * M_PI / 180);
  Compute(*aB, "c0", 50, 1.0e-3);
  Compute(*aB, "c1", 50, 1.0e-2);
  State(*aB, "", aMV);

  int aNbOps = InI(I(4));
  for (int k = 0; k < aNbOps; k++)
  {
    int         anOp   = InI(I(theMV ? 9 : 6));
    int         aSide  = InI(1 + I(2));
    double      aValue = In(Grid(-60.0 + 120.0 * U()));
    std::string aName  = "op" + std::to_string(k);
    Tok(aName.c_str(), [&] {
      switch (anOp)
      {
        case 0: // setangle side angle
          if (aSide == 1)
          {
            aB->SetAngle1(aValue * M_PI / 180);
            if (aB->GetConstraintOrder1() == 0)
              aB->SetConstraintOrder1(1);
          }
          else
          {
            aB->SetAngle2(aValue * M_PI / 180);
            if (aB->GetConstraintOrder2() == 0)
              aB->SetConstraintOrder2(1);
          }
          break;
        case 1: // setslide
          aB->SetFreeSliding(false);
          aB->SetSlidingFactor(std::abs(aValue) / 30.0);
          break;
        case 2: // freeslide
          aB->SetFreeSliding(true);
          break;
        case 3: // freeangle
          if (aSide == 1)
            aB->SetConstraintOrder1(0);
          else
            aB->SetConstraintOrder2(0);
          break;
        case 4: // setheight
          aB->SetHeight(aValue / 60.0);
          break;
        case 5: // setslope
          aB->SetSlope(aValue / 60.0);
          break;
        case 6: // setcurvature
          if (aSide == 1)
          {
            aMV->SetCurvature1(aValue / 60.0);
            aMV->SetConstraintOrder1(2);
          }
          else
          {
            aMV->SetCurvature2(aValue / 60.0);
            aMV->SetConstraintOrder2(2);
          }
          break;
        case 7: // freecurvature
          if (aSide == 1)
          {
            if (aMV->GetConstraintOrder1() > 1)
              aMV->SetConstraintOrder1(1);
          }
          else
          {
            if (aMV->GetConstraintOrder2() > 1)
              aMV->SetConstraintOrder2(1);
          }
          break;
        case 8: // setphysicalratio
          aMV->SetPhysicalRatio(std::abs(aValue) / 40.0);
          break;
      }
      FairCurve_AnalysisCode aCode = FairCurve_OK;
      bool                   anOk  = aB->Compute(aCode, 50, 1.0e-2);
      return std::to_string(anOk ? 1 : 0) + "," + std::to_string((int)aCode);
    });
    State(*aB, (aName + ".").c_str(), aMV);
  }
  delete aB;
}

// The energies on random poles of a degree 9 B-spline (one span, or two with an interior knot of multiplicity 6)
static void RunEnergy(bool theMVC)
{
  const int Degree   = 9;
  int       aNbKnots = InI(2 + I(2));
  NCollection_Array1<double> aKnots(1, aNbKnots);
  NCollection_Array1<int>    aMults(1, aNbKnots);
  for (int i = 1; i <= aNbKnots; i++)
  {
    aKnots(i) = (double)(i - 1) / (aNbKnots - 1);
    aMults(i) = (i == 1 || i == aNbKnots) ? Degree + 1 : Degree - 3;
  }
  occ::handle<NCollection_HArray1<double>> aFlat =
    new NCollection_HArray1<double>(1, BSplCLib::KnotSequenceLength(aMults, Degree, false));
  BSplCLib::KnotSequence(aKnots, aMults, Degree, false, aFlat->ChangeArray1());
  int aNbPoles = aFlat->Length() - Degree - 1;
  occ::handle<NCollection_HArray1<gp_Pnt2d>> aPoles = new NCollection_HArray1<gp_Pnt2d>(1, aNbPoles);
  double aLength = In(Grid(1.0 + 9.0 * U()));
  double aBend   = In(Grid(-0.5 + U()));
  for (int i = 1; i <= aNbPoles; i++)
  {
    double t = (double)(i - 1) / (aNbPoles - 1);
    double x = In(Grid(aLength * t + 0.05 * aLength * (U() - 0.5)));
    double y = In(Grid(aBend * aLength * t * (1 - t) + 0.05 * aLength * (U() - 0.5)));
    aPoles->SetValue(i, gp_Pnt2d(x, y));
  }
  int    o1 = Order(theMVC ? 2 : 1), o2 = Order(theMVC ? 2 : 1);
  bool   fs = In(I(2)) != 0.0;
  double a1 = Angle(), a2 = Angle();
  double h  = In(Grid(0.05 + U())), s = Slope();
  double aSliding = In(Grid(aLength * (1.0 + 0.3 * U())));
  FairCurve_BattenLaw aLaw(h, s, aSliding);

  FairCurve_Energy* anE = nullptr;
  try
  {
    if (theMVC)
    {
      double r  = Ratio();
      double c1 = Curvature(), c2 = Curvature();
      anE = new FairCurve_EnergyOfMVC(Degree + 1, aFlat, aPoles, o1, o2, aLaw, r, aSliding, fs, a1, a2, c1, c2);
    }
    else
    {
      anE = new FairCurve_EnergyOfBatten(Degree + 1, aFlat, aPoles, o1, o2, aLaw, aSliding, fs, a1, a2);
    }
  }
  catch (const Standard_Failure& theF)
  {
    Put("ctor:" + ExcName(theF));
    return;
  }
  int         n = anE->NbVariables();
  // zeros, as the port's: on failure Values leaves G and H as they were
  math_Vector X(1, n, 0.0), G(1, n, 0.0);
  math_Matrix H(1, n, 1, n, 0.0);
  Tok("var", [&] {
    bool anOk = anE->Variable(X);
    return std::to_string(anOk ? 1 : 0) + "," + Vec(X);
  });
  for (int k = 0; k < 2; k++)
  {
    std::string p = std::to_string(k);
    Tok(("v" + p).c_str(), [&] {
      double E    = 0;
      bool   anOk = anE->Value(X, E);
      return std::to_string(anOk ? 1 : 0) + "," + Hex(E);
    });
    Tok(("g" + p).c_str(), [&] {
      double E    = 0;
      bool   anOk = anE->Values(X, E, G);
      return std::to_string(anOk ? 1 : 0) + "," + Hex(E) + "," + Vec(G);
    });
    Tok(("h" + p).c_str(), [&] {
      double E    = 0;
      bool   anOk = anE->Values(X, E, G, H);
      return std::to_string(anOk ? 1 : 0) + "," + Hex(E) + "," + Vec(G) + "," + Mat(H);
    });
    Tok(("p" + p).c_str(), [&] {
      std::string aS;
      for (int i = 1; i <= aNbPoles; i++)
      {
        aS += (i > 1 ? "," : "") + Hex(anE->Poles()->Value(i));
      }
      return aS;
    });
    // move every variable a little for the second evaluation
    for (int i = 1; i <= n; i++)
    {
      X(i) += In(Grid(0.02 * aLength * (U() - 0.5)));
    }
  }
  delete anE;
}

struct Kind
{
  const char*           Name;
  std::function<void()> Run;
};

int main(int argc, char** argv)
{
  int                     aNbCases = argc > 1 ? std::atoi(argv[1]) : 100;
  const std::vector<Kind> aKinds   = {
    {"Batten", [] { RunFair(false); }},
    {"MinVar", [] { RunFair(true); }},
    {"DrawBatten", [] { RunDraw(false); }},
    {"DrawMinVar", [] { RunDraw(true); }},
    {"EBatten", [] { RunEnergy(false); }},
    {"EMVC", [] { RunEnergy(true); }},
  };
  for (const Kind& aKind : aKinds)
  {
    int aNb = (aKind.Name[0] == 'E') ? aNbCases / 2 : aNbCases;
    for (int k = 0; k < aNb; k++)
    {
      theIn.clear();
      theOut.clear();
      try
      {
        aKind.Run();
      }
      catch (const Standard_Failure& theF)
      {
        Put("fail:" + ExcName(theF));
      }
      std::printf("%s", aKind.Name);
      for (double x : theIn)
      {
        std::printf(" %.17g", x);
      }
      std::printf(" ;%s\n", theOut.c_str());
      std::fflush(stdout);
    }
  }
  return 0;
}
