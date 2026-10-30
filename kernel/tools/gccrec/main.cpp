// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
//
// gccrec: records native OCCT's analytic constraint solvers (GccAna, unit 5K1) on random configurations: every
// constructor of every GccAna class, each solution with its qualifiers, tangency, centre and intersection data, every
// double as its 64 bits in hex. The inputs are written as round-trip decimals. The port replays each case from the
// same inputs and writes its results in the same format (tests/OcctNet.ModelingAlgorithms.Tests/GccAna).
// Built against build123d.NET's native OCCT V8_0_1 (tools/gccrec/build.ps1).
//
//   gccrec <cases per constructor>   writes the recording to stdout
//
// One line per case:  <constructor> <inputs...> ; <tokens...>
// The tokens are named (name:value), so that a difference can be attributed to an accessor.
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <functional>
#include <typeinfo>
#include <string>
#include <vector>

#include <GccAna_Circ2d2TanOn.hxx>
#include <GccAna_Circ2d2TanRad.hxx>
#include <GccAna_Circ2d3Tan.hxx>
#include <GccAna_Circ2dBisec.hxx>
#include <GccAna_Circ2dTanCen.hxx>
#include <GccAna_Circ2dTanOnRad.hxx>
#include <GccAna_CircLin2dBisec.hxx>
#include <GccAna_CircPnt2dBisec.hxx>
#include <GccAna_Lin2d2Tan.hxx>
#include <GccAna_Lin2dBisec.hxx>
#include <GccAna_Lin2dTanObl.hxx>
#include <GccAna_Lin2dTanPar.hxx>
#include <GccAna_Lin2dTanPer.hxx>
#include <GccAna_LinPnt2dBisec.hxx>
#include <GccAna_Pnt2dBisec.hxx>
#include <GccEnt_QualifiedCirc.hxx>
#include <GccEnt_QualifiedLin.hxx>
#include <GccInt_Bisec.hxx>
#include <Standard_Failure.hxx>
#include <gp_Circ2d.hxx>
#include <gp_Elips2d.hxx>
#include <gp_Hypr2d.hxx>
#include <gp_Lin2d.hxx>
#include <gp_Parab2d.hxx>
#include <gp_Pnt2d.hxx>

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

static std::string Hex(const gp_Dir2d& theD) { return Hex(theD.X()) + "," + Hex(theD.Y()); }

static std::string Hex(const gp_Ax22d& theA)
{
  return Hex(theA.Location()) + "," + Hex(theA.XDirection()) + "," + Hex(theA.YDirection());
}

static std::string Hex(const gp_Circ2d& theC) { return Hex(theC.Position()) + "," + Hex(theC.Radius()); }

static std::string Hex(const gp_Lin2d& theL) { return Hex(theL.Location()) + "," + Hex(theL.Direction()); }

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

static std::string Tan(const std::function<void(double&, double&, gp_Pnt2d&)>& theF)
{
  double   aP1 = 0., aP2 = 0.;
  gp_Pnt2d aP;
  theF(aP1, aP2, aP);
  return Hex(aP1) + "," + Hex(aP2) + "," + Hex(aP);
}

static std::string Bisec(const occ::handle<GccInt_Bisec>& theB)
{
  if (theB.IsNull())
  {
    return "null";
  }
  switch (theB->ArcType())
  {
    case GccInt_Lin:
      return "Lin," + Hex(theB->Line());
    case GccInt_Cir:
      return "Cir," + Hex(theB->Circle());
    case GccInt_Ell: {
      gp_Elips2d anE = theB->Ellipse();
      return "Ell," + Hex(anE.Axis()) + "," + Hex(anE.MajorRadius()) + "," + Hex(anE.MinorRadius());
    }
    case GccInt_Hpr: {
      gp_Hypr2d aH = theB->Hyperbola();
      return "Hpr," + Hex(aH.Axis()) + "," + Hex(aH.MajorRadius()) + "," + Hex(aH.MinorRadius());
    }
    case GccInt_Par: {
      gp_Parab2d aP = theB->Parabola();
      return "Par," + Hex(aP.Axis()) + "," + Hex(aP.Focal());
    }
    case GccInt_Pnt:
      return "Pnt," + Hex(theB->Point());
  }
  return "?";
}

//=================================================================================================
// Random inputs: half on a coarse grid (to reach tangent, concentric, coincident and parallel cases), half
// uniform; all rounded to multiples of 2^-10 so that they print short and exactly.

static uint64_t theState = 0x5EEDu;

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

// Hand-picked cases: In() then takes its values from thePreset instead of the random ones
static std::vector<double> thePreset;
static size_t              thePresetPos = 0;

static double In(double theV)
{
  if (thePresetPos < thePreset.size())
  {
    theV = thePreset[thePresetPos++];
  }
  theIn.push_back(theV);
  return theV;
}

static double Coord() { return In(I(2) == 0 ? 0.5 * (I(13) - 6) : Grid(-4.0 + 8.0 * U())); }

static double Radius()
{
  int k = I(20);
  return In(k == 0 ? 0.0 : (k < 11 ? 0.5 * (1 + I(6)) : Grid(0.05 + 3.0 * U())));
}

static gp_Pnt2d Pnt()
{
  double x = Coord();
  double y = Coord();
  return gp_Pnt2d(x, y);
}

static gp_Dir2d Dir()
{
  static const int aPairs[][2] = {{1, 0}, {0, 1}, {-1, 0}, {0, -1}, {1, 1}, {-1, 1}, {3, 4}, {4, -3}, {1, 2}};
  double           x, y;
  if (I(2) == 0)
  {
    const int* p = aPairs[I(9)];
    x            = In(p[0]);
    y            = In(p[1]);
  }
  else
  {
    double a = 2.0 * M_PI * U();
    x        = In(Grid(std::cos(a) * 4.0));
    y        = In(Grid(std::sin(a) * 4.0));
    if (x == 0.0 && y == 0.0 && thePreset.empty())
    {
      theIn.back() = y = 1.0;
    }
  }
  return gp_Dir2d(x, y);
}

static gp_Lin2d Lin()
{
  gp_Pnt2d p = Pnt();
  gp_Dir2d d = Dir();
  return gp_Lin2d(p, d);
}

static gp_Circ2d Circ()
{
  gp_Pnt2d p = Pnt();
  gp_Dir2d d = Dir();
  double   r = Radius();
  bool     s = In(I(4) != 0) != 0.0;
  return gp_Circ2d(gp_Ax2d(p, d), r, s);
}

static GccEnt_Position QualC()
{
  int k = I(50);
  return (GccEnt_Position)(int)In(k == 0 ? 4 : k % 4);
}

static GccEnt_Position QualL()
{
  int k = I(60);
  return (GccEnt_Position)(int)In(k == 0 ? 4 : (k == 1 ? 1 : (k % 3 == 0 ? 0 : (k % 3 == 1 ? 2 : 3))));
}

static GccEnt_QualifiedCirc QC()
{
  gp_Circ2d c = Circ();
  return GccEnt_QualifiedCirc(c, QualC());
}

static GccEnt_QualifiedLin QL()
{
  gp_Lin2d l = Lin();
  return GccEnt_QualifiedLin(l, QualL());
}

static double Tol() { return In(I(10) == 0 ? 1.0e-4 : 1.0e-7); }

static double Rad()
{
  int k = I(20);
  return In(k == 0 ? -1.0 : (k < 11 ? 0.5 * (1 + I(6)) : Grid(0.05 + 3.0 * U())));
}

static double Angle()
{
  int k = I(3);
  return In(k == 0 ? (I(8) - 3) * (M_PI / 4.0) : Grid(-3.0 + 6.0 * U()));
}

//=================================================================================================
// The dumps of the solver kinds.

// Circles: Circ2d3Tan (3 tangencies), Circ2d2TanOn (2 + centre), Circ2d2TanRad (2), Circ2dTanOnRad (1 + centre),
// Circ2dTanCen (1)
template <int theNbTan, bool theOn, class T>
static void DumpCirc(const T& S)
{
  Put(std::string("done:") + (S.IsDone() ? "1" : "0"));
  if (!S.IsDone())
  {
    return;
  }
  int aNb = 0;
  try
  {
    aNb = S.NbSolutions();
  }
  catch (const Standard_Failure& theF)
  {
    Put("nb:" + ExcName(theF));
    return;
  }
  Put("nb:" + std::to_string(aNb));
  for (int i = 1; i <= aNb; i++)
  {
    Tok("sol", [&] { return Hex(S.ThisSolution(i)); });
    Tok("q", [&] { return Qualifiers(S, i); });
    Tok("t1", [&] { return Tan([&](double& a, double& b, gp_Pnt2d& p) { S.Tangency1(i, a, b, p); }); });
    Tok("s1", [&] { return std::string(S.IsTheSame1(i) ? "1" : "0"); });
    if constexpr (theNbTan >= 2)
    {
      Tok("t2", [&] { return Tan2(S, i); });
      Tok("s2", [&] { return Same2(S, i); });
    }
    if constexpr (theNbTan >= 3)
    {
      Tok("t3", [&] { return Tan3(S, i); });
      Tok("s3", [&] { return Same3(S, i); });
    }
    if constexpr (theOn)
    {
      Tok("c3", [&] { return On3(S, i); });
    }
  }
}

// Lines: Lin2d2Tan (2 tangencies), Lin2dTanObl (1 + intersection), Lin2dTanPar (1), Lin2dTanPer (1 + intersection)
template <int theNbTan, bool theInter, class T>
static void DumpLin(const T& S)
{
  Put(std::string("done:") + (S.IsDone() ? "1" : "0"));
  if (!S.IsDone())
  {
    return;
  }
  int aNb = 0;
  try
  {
    aNb = S.NbSolutions();
  }
  catch (const Standard_Failure& theF)
  {
    Put("nb:" + ExcName(theF));
    return;
  }
  Put("nb:" + std::to_string(aNb));
  for (int i = 1; i <= aNb; i++)
  {
    Tok("sol", [&] { return Hex(S.ThisSolution(i)); });
    Tok("q", [&] { return Qualifiers(S, i); });
    Tok("t1", [&] { return Tan([&](double& a, double& b, gp_Pnt2d& p) { S.Tangency1(i, a, b, p); }); });
    if constexpr (theNbTan >= 2)
    {
      Tok("t2", [&] { return Tan2(S, i); });
    }
    if constexpr (theInter)
    {
      Tok("i2", [&] { return Inter2(S, i); });
    }
  }
}

// The accessors that only some classes have, through overloads.
static std::string Q(GccEnt_Position p) { return std::to_string((int)p); }

static std::string Qualifiers(const GccAna_Circ2d3Tan& S, int i)
{
  GccEnt_Position a = GccEnt_noqualifier, b = GccEnt_noqualifier, c = GccEnt_noqualifier;
  S.WhichQualifier(i, a, b, c);
  return Q(a) + "," + Q(b) + "," + Q(c);
}

template <class T>
static std::string Qualifiers2(const T& S, int i)
{
  GccEnt_Position a = GccEnt_noqualifier, b = GccEnt_noqualifier;
  S.WhichQualifier(i, a, b);
  return Q(a) + "," + Q(b);
}

template <class T>
static std::string Qualifiers1(const T& S, int i)
{
  GccEnt_Position a = GccEnt_noqualifier;
  S.WhichQualifier(i, a);
  return Q(a);
}

static std::string Qualifiers(const GccAna_Circ2d2TanOn& S, int i) { return Qualifiers2(S, i); }
static std::string Qualifiers(const GccAna_Circ2d2TanRad& S, int i) { return Qualifiers2(S, i); }
static std::string Qualifiers(const GccAna_Lin2d2Tan& S, int i) { return Qualifiers2(S, i); }
static std::string Qualifiers(const GccAna_Circ2dTanOnRad& S, int i) { return Qualifiers1(S, i); }
static std::string Qualifiers(const GccAna_Circ2dTanCen& S, int i) { return Qualifiers1(S, i); }
static std::string Qualifiers(const GccAna_Lin2dTanObl& S, int i) { return Qualifiers1(S, i); }
static std::string Qualifiers(const GccAna_Lin2dTanPar& S, int i) { return Qualifiers1(S, i); }
static std::string Qualifiers(const GccAna_Lin2dTanPer& S, int i) { return Qualifiers1(S, i); }

template <class T>
static std::string Tan2(const T& S, int i)
{
  return Tan([&](double& a, double& b, gp_Pnt2d& p) { S.Tangency2(i, a, b, p); });
}

template <class T>
static std::string Tan3(const T& S, int i)
{
  return Tan([&](double& a, double& b, gp_Pnt2d& p) { S.Tangency3(i, a, b, p); });
}

template <class T>
static std::string Inter2(const T& S, int i)
{
  return Tan([&](double& a, double& b, gp_Pnt2d& p) { S.Intersection2(i, a, b, p); });
}

template <class T>
static std::string On3(const T& S, int i)
{
  double   a = 0.;
  gp_Pnt2d p;
  S.CenterOn3(i, a, p);
  return Hex(a) + "," + Hex(p);
}

template <class T>
static std::string Same2(const T& S, int i)
{
  return S.IsTheSame2(i) ? "1" : "0";
}

template <class T>
static std::string Same3(const T& S, int i)
{
  return S.IsTheSame3(i) ? "1" : "0";
}

// The bisectors
template <class T>
static void DumpBisecN(const T& S)
{
  Put(std::string("done:") + (S.IsDone() ? "1" : "0"));
  if (!S.IsDone())
  {
    return;
  }
  int aNb = 0;
  try
  {
    aNb = S.NbSolutions();
  }
  catch (const Standard_Failure& theF)
  {
    Put("nb:" + ExcName(theF));
    return;
  }
  Put("nb:" + std::to_string(aNb));
  for (int i = 1; i <= aNb; i++)
  {
    Tok("bis", [&] { return Bisec(S.ThisSolution(i)); });
  }
}

//=================================================================================================

struct Ctor
{
  const char*           Name;
  std::function<void()> Run;
};

static std::vector<Ctor> Ctors()
{
  std::vector<Ctor> v;
  // GccAna_Circ2d3Tan
  v.push_back({"3Tan.CCC", [] { auto a = QC(); auto b = QC(); auto c = QC(); double t = Tol(); DumpCirc<3, false>(GccAna_Circ2d3Tan(a, b, c, t)); }});
  v.push_back({"3Tan.CCL", [] { auto a = QC(); auto b = QC(); auto c = QL(); double t = Tol(); DumpCirc<3, false>(GccAna_Circ2d3Tan(a, b, c, t)); }});
  v.push_back({"3Tan.CLL", [] { auto a = QC(); auto b = QL(); auto c = QL(); double t = Tol(); DumpCirc<3, false>(GccAna_Circ2d3Tan(a, b, c, t)); }});
  v.push_back({"3Tan.LLL", [] { auto a = QL(); auto b = QL(); auto c = QL(); double t = Tol(); DumpCirc<3, false>(GccAna_Circ2d3Tan(a, b, c, t)); }});
  v.push_back({"3Tan.CCP", [] { auto a = QC(); auto b = QC(); auto c = Pnt(); double t = Tol(); DumpCirc<3, false>(GccAna_Circ2d3Tan(a, b, c, t)); }});
  v.push_back({"3Tan.CLP", [] { auto a = QC(); auto b = QL(); auto c = Pnt(); double t = Tol(); DumpCirc<3, false>(GccAna_Circ2d3Tan(a, b, c, t)); }});
  v.push_back({"3Tan.LLP", [] { auto a = QL(); auto b = QL(); auto c = Pnt(); double t = Tol(); DumpCirc<3, false>(GccAna_Circ2d3Tan(a, b, c, t)); }});
  v.push_back({"3Tan.CPP", [] { auto a = QC(); auto b = Pnt(); auto c = Pnt(); double t = Tol(); DumpCirc<3, false>(GccAna_Circ2d3Tan(a, b, c, t)); }});
  v.push_back({"3Tan.LPP", [] { auto a = QL(); auto b = Pnt(); auto c = Pnt(); double t = Tol(); DumpCirc<3, false>(GccAna_Circ2d3Tan(a, b, c, t)); }});
  v.push_back({"3Tan.PPP", [] { auto a = Pnt(); auto b = Pnt(); auto c = Pnt(); double t = Tol(); DumpCirc<3, false>(GccAna_Circ2d3Tan(a, b, c, t)); }});
  // GccAna_Circ2d2TanOn
  v.push_back({"2TanOn.CCl", [] { auto a = QC(); auto b = QC(); auto c = Lin(); double t = Tol(); DumpCirc<2, true>(GccAna_Circ2d2TanOn(a, b, c, t)); }});
  v.push_back({"2TanOn.CLl", [] { auto a = QC(); auto b = QL(); auto c = Lin(); double t = Tol(); DumpCirc<2, true>(GccAna_Circ2d2TanOn(a, b, c, t)); }});
  v.push_back({"2TanOn.LLl", [] { auto a = QL(); auto b = QL(); auto c = Lin(); double t = Tol(); DumpCirc<2, true>(GccAna_Circ2d2TanOn(a, b, c, t)); }});
  v.push_back({"2TanOn.CPl", [] { auto a = QC(); auto b = Pnt(); auto c = Lin(); double t = Tol(); DumpCirc<2, true>(GccAna_Circ2d2TanOn(a, b, c, t)); }});
  v.push_back({"2TanOn.LPl", [] { auto a = QL(); auto b = Pnt(); auto c = Lin(); double t = Tol(); DumpCirc<2, true>(GccAna_Circ2d2TanOn(a, b, c, t)); }});
  v.push_back({"2TanOn.PPl", [] { auto a = Pnt(); auto b = Pnt(); auto c = Lin(); double t = Tol(); DumpCirc<2, true>(GccAna_Circ2d2TanOn(a, b, c, t)); }});
  v.push_back({"2TanOn.CCc", [] { auto a = QC(); auto b = QC(); auto c = Circ(); double t = Tol(); DumpCirc<2, true>(GccAna_Circ2d2TanOn(a, b, c, t)); }});
  v.push_back({"2TanOn.CLc", [] { auto a = QC(); auto b = QL(); auto c = Circ(); double t = Tol(); DumpCirc<2, true>(GccAna_Circ2d2TanOn(a, b, c, t)); }});
  v.push_back({"2TanOn.CPc", [] { auto a = QC(); auto b = Pnt(); auto c = Circ(); double t = Tol(); DumpCirc<2, true>(GccAna_Circ2d2TanOn(a, b, c, t)); }});
  v.push_back({"2TanOn.LLc", [] { auto a = QL(); auto b = QL(); auto c = Circ(); double t = Tol(); DumpCirc<2, true>(GccAna_Circ2d2TanOn(a, b, c, t)); }});
  v.push_back({"2TanOn.LPc", [] { auto a = QL(); auto b = Pnt(); auto c = Circ(); double t = Tol(); DumpCirc<2, true>(GccAna_Circ2d2TanOn(a, b, c, t)); }});
  v.push_back({"2TanOn.PPc", [] { auto a = Pnt(); auto b = Pnt(); auto c = Circ(); double t = Tol(); DumpCirc<2, true>(GccAna_Circ2d2TanOn(a, b, c, t)); }});
  // GccAna_Circ2d2TanRad
  v.push_back({"2TanRad.CC", [] { auto a = QC(); auto b = QC(); double r = Rad(); double t = Tol(); DumpCirc<2, false>(GccAna_Circ2d2TanRad(a, b, r, t)); }});
  v.push_back({"2TanRad.CL", [] { auto a = QC(); auto b = QL(); double r = Rad(); double t = Tol(); DumpCirc<2, false>(GccAna_Circ2d2TanRad(a, b, r, t)); }});
  v.push_back({"2TanRad.CP", [] { auto a = QC(); auto b = Pnt(); double r = Rad(); double t = Tol(); DumpCirc<2, false>(GccAna_Circ2d2TanRad(a, b, r, t)); }});
  v.push_back({"2TanRad.LP", [] { auto a = QL(); auto b = Pnt(); double r = Rad(); double t = Tol(); DumpCirc<2, false>(GccAna_Circ2d2TanRad(a, b, r, t)); }});
  v.push_back({"2TanRad.LL", [] { auto a = QL(); auto b = QL(); double r = Rad(); double t = Tol(); DumpCirc<2, false>(GccAna_Circ2d2TanRad(a, b, r, t)); }});
  v.push_back({"2TanRad.PP", [] { auto a = Pnt(); auto b = Pnt(); double r = Rad(); double t = Tol(); DumpCirc<2, false>(GccAna_Circ2d2TanRad(a, b, r, t)); }});
  // GccAna_Circ2dTanOnRad
  v.push_back({"TanOnRad.Cl", [] { auto a = QC(); auto c = Lin(); double r = Rad(); double t = Tol(); DumpCirc<1, true>(GccAna_Circ2dTanOnRad(a, c, r, t)); }});
  v.push_back({"TanOnRad.Ll", [] { auto a = QL(); auto c = Lin(); double r = Rad(); double t = Tol(); DumpCirc<1, true>(GccAna_Circ2dTanOnRad(a, c, r, t)); }});
  v.push_back({"TanOnRad.Pl", [] { auto a = Pnt(); auto c = Lin(); double r = Rad(); double t = Tol(); DumpCirc<1, true>(GccAna_Circ2dTanOnRad(a, c, r, t)); }});
  v.push_back({"TanOnRad.Cc", [] { auto a = QC(); auto c = Circ(); double r = Rad(); double t = Tol(); DumpCirc<1, true>(GccAna_Circ2dTanOnRad(a, c, r, t)); }});
  v.push_back({"TanOnRad.Lc", [] { auto a = QL(); auto c = Circ(); double r = Rad(); double t = Tol(); DumpCirc<1, true>(GccAna_Circ2dTanOnRad(a, c, r, t)); }});
  v.push_back({"TanOnRad.Pc", [] { auto a = Pnt(); auto c = Circ(); double r = Rad(); double t = Tol(); DumpCirc<1, true>(GccAna_Circ2dTanOnRad(a, c, r, t)); }});
  // GccAna_Circ2dTanCen
  v.push_back({"TanCen.C", [] { auto a = QC(); auto p = Pnt(); double t = Tol(); DumpCirc<1, false>(GccAna_Circ2dTanCen(a, p, t)); }});
  v.push_back({"TanCen.L", [] { auto a = Lin(); auto p = Pnt(); DumpCirc<1, false>(GccAna_Circ2dTanCen(a, p)); }});
  v.push_back({"TanCen.P", [] { auto a = Pnt(); auto p = Pnt(); DumpCirc<1, false>(GccAna_Circ2dTanCen(a, p)); }});
  // GccAna_Lin2d2Tan
  v.push_back({"Lin2d2Tan.PP", [] { auto a = Pnt(); auto b = Pnt(); double t = Tol(); DumpLin<2, false>(GccAna_Lin2d2Tan(a, b, t)); }});
  v.push_back({"Lin2d2Tan.CP", [] { auto a = QC(); auto b = Pnt(); double t = Tol(); DumpLin<2, false>(GccAna_Lin2d2Tan(a, b, t)); }});
  v.push_back({"Lin2d2Tan.CC", [] { auto a = QC(); auto b = QC(); double t = Tol(); DumpLin<2, false>(GccAna_Lin2d2Tan(a, b, t)); }});
  // GccAna_Lin2dTanObl
  v.push_back({"TanObl.P", [] { auto a = Pnt(); auto l = Lin(); double g = Angle(); DumpLin<1, true>(GccAna_Lin2dTanObl(a, l, g)); }});
  v.push_back({"TanObl.C", [] { auto a = QC(); auto l = Lin(); double g = Angle(); DumpLin<1, true>(GccAna_Lin2dTanObl(a, l, g)); }});
  // GccAna_Lin2dTanPar
  v.push_back({"TanPar.P", [] { auto a = Pnt(); auto l = Lin(); DumpLin<1, false>(GccAna_Lin2dTanPar(a, l)); }});
  v.push_back({"TanPar.C", [] { auto a = QC(); auto l = Lin(); DumpLin<1, false>(GccAna_Lin2dTanPar(a, l)); }});
  // GccAna_Lin2dTanPer
  v.push_back({"TanPer.Pl", [] { auto a = Pnt(); auto l = Lin(); DumpLin<1, true>(GccAna_Lin2dTanPer(a, l)); }});
  v.push_back({"TanPer.Pc", [] { auto a = Pnt(); auto c = Circ(); DumpLin<1, true>(GccAna_Lin2dTanPer(a, c)); }});
  v.push_back({"TanPer.Cl", [] { auto a = QC(); auto l = Lin(); DumpLin<1, true>(GccAna_Lin2dTanPer(a, l)); }});
  v.push_back({"TanPer.Cc", [] { auto a = QC(); auto c = Circ(); DumpLin<1, true>(GccAna_Lin2dTanPer(a, c)); }});
  // The bisectors
  v.push_back({"Bisec.PP", [] {
                 auto a = Pnt(); auto b = Pnt();
                 GccAna_Pnt2dBisec S(a, b);
                 Put(std::string("done:") + (S.IsDone() ? "1" : "0"));
                 Put(std::string("has:") + (S.HasSolution() ? "1" : "0"));
                 Tok("lin", [&] { return Hex(S.ThisSolution()); });
               }});
  v.push_back({"Bisec.LP", [] {
                 auto a = Lin(); auto b = Pnt();
                 GccAna_LinPnt2dBisec S(a, b);
                 Put(std::string("done:") + (S.IsDone() ? "1" : "0"));
                 Tok("bis", [&] { return Bisec(S.ThisSolution()); });
               }});
  v.push_back({"Bisec.LL", [] {
                 auto a = Lin(); auto b = Lin();
                 GccAna_Lin2dBisec S(a, b);
                 Put(std::string("done:") + (S.IsDone() ? "1" : "0"));
                 if (!S.IsDone()) return;
                 int aNb = S.NbSolutions();
                 Put("nb:" + std::to_string(aNb));
                 for (int i = 1; i <= aNb; i++)
                 {
                   Tok("sol", [&] { return Hex(S.ThisSolution(i)); });
                   Tok("i1", [&] { return Tan([&](double& x, double& y, gp_Pnt2d& p) { S.Intersection1(i, x, y, p); }); });
                   Tok("i2", [&] { return Tan([&](double& x, double& y, gp_Pnt2d& p) { S.Intersection2(i, x, y, p); }); });
                 }
               }});
  v.push_back({"Bisec.CL", [] { auto a = Circ(); auto b = Lin(); DumpBisecN(GccAna_CircLin2dBisec(a, b)); }});
  v.push_back({"Bisec.CP", [] { auto a = Circ(); auto b = Pnt(); DumpBisecN(GccAna_CircPnt2dBisec(a, b)); }});
  v.push_back({"Bisec.CPt", [] { auto a = Circ(); auto b = Pnt(); double t = Tol(); DumpBisecN(GccAna_CircPnt2dBisec(a, b, t)); }});
  v.push_back({"Bisec.CC", [] { auto a = Circ(); auto b = Circ(); DumpBisecN(GccAna_Circ2dBisec(a, b)); }});
  return v;
}

// The configurations the unit tests single out (degenerate and boundary cases), in the order the constructors read
// their inputs: a point is x y, a line x y dx dy, a circle x y dx dy r sense, a qualified one adds the qualifier
// (GccEnt_Position as an int), then the radius, tolerance or angle.
static const std::vector<std::pair<std::string, std::vector<double>>> thePresets = {
  // two unit circles, the radius just over 3 within the tolerance: OCCT's "sign" branch writes qualifier1(0)
  {"TanOnRad.Cc", {0, 0, 1, 0, 1, 1, 0, 1, 0, 1, 0, 1, 1, 3.0000000005, 1.0e-9}},
  // tangency points at the centre of a circle: ElCLib's parameter of a circle's centre
  {"TanOnRad.Cl", {0, 0, 1, 0, 1, 1, 2, 0, 0, 1, 0, 0.5, 1.0e-9}},
  {"TanOnRad.Lc", {0, 0, 1, 0, 0, 0, 0, 1, 0, 2, 1, 1, 1.0e-9}},
  {"TanCen.C", {0, 0, 1, 0, 2, 1, 0, 2, 0, 1.0e-9}},
  {"TanCen.L", {1, 1, 1, 0, 1, 1}},
  {"2TanRad.CP", {0, 0, 1, 0, 1, 1, 0, 1, 0, 1, 1.0e-9}},
  {"2TanRad.CP", {1, 2, 1, 0, 1, 1, 0, 2, 2, 1, 1.0e-9}},
  // concentric circles, coincident points, points on the curves
  {"Bisec.CC", {0, 0, 1, 0, 3, 1, 0, 0, 1, 0, 1, 1}},
  {"Bisec.CC", {1, 1, 1, 0, 2, 1, 1, 1, 1, 0, 2, 1}},
  {"Bisec.CP", {0, 0, 1, 0, 2, 1, 0, 0}},
  {"Bisec.CP", {0, 0, 1, 0, 2, 1, 2, 0}},
  {"Bisec.PP", {1, 1, 1, 1}},
  {"Bisec.LP", {0, 0, 1, 0, 2, 0}},
  {"3Tan.PPP", {0, 0, 1, 1, 2, 2, 1.0e-9}},
  {"3Tan.PPP", {1, 1, 1, 1, 1, 1, 1.0e-9}},
  {"Lin2d2Tan.CC", {0, 0, 1, 0, 1, 1, 0, 0, 0, 1, 0, 2, 1, 0, 1.0e-9}},
  {"TanPer.Pc", {0, 0, 0, 0, 1, 0, 1, 1}},
  {"TanPer.Cc", {0, 0, 1, 0, 1, 1, 0, 0, 0, 1, 0, 2, 1}},
};

static void RunCase(const Ctor& theCtor)
{
  theIn.clear();
  theOut.clear();
  try
  {
    theCtor.Run();
  }
  catch (const Standard_Failure& theF)
  {
    Put("ctor:" + ExcName(theF));
  }
  std::printf("%s", theCtor.Name);
  for (double x : theIn)
  {
    std::printf(" %.17g", x);
  }
  std::printf(" ;%s\n", theOut.c_str());
}

int main(int argc, char** argv)
{
  int                     aNbCases = argc > 1 ? std::atoi(argv[1]) : 100;
  const std::vector<Ctor> aCtors   = Ctors();
  for (const Ctor& aCtor : aCtors)
  {
    for (int k = 0; k < aNbCases; k++)
    {
      RunCase(aCtor);
    }
  }
  for (const auto& aPreset : thePresets)
  {
    for (const Ctor& aCtor : aCtors)
    {
      if (aPreset.first == aCtor.Name)
      {
        thePreset    = aPreset.second;
        thePresetPos = 0;
        RunCase(aCtor);
        if (thePresetPos != thePreset.size())
        {
          std::fprintf(stderr, "preset %s: %zu values read of %zu\n", aCtor.Name, thePresetPos, thePreset.size());
          return 1;
        }
      }
    }
  }
  return 0;
}
