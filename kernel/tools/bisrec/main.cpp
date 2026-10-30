// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
//
// bisrec: records native OCCT's bisectors (Bisector, unit 5M) on random and hand-picked configurations:
// Bisector_Bisec::Perform between lines, circular arcs, elliptic arcs, B-splines and points, called as MAT2d_Tool2d
// calls it (the joined case: the end of the first element, its tangent, the reversed tangent of the second; and the
// general case), and Bisector_Inter between the bisectors of two neighbouring corners of a chain, on the domains
// MAT2d_Tool2d gives them. Every double of the results is written as its 64 bits in hex; the inputs are written as
// round-trip decimals and both sides build their geometry from that text (tests/OcctNet.ModelingAlgorithms.Tests/
// Bisector). Built against build123d.NET's native OCCT V8_0_1 (tools/bisrec/build.ps1).
//
//   bisrec <random cases per kind>   writes the recording to stdout
//
// One line per case:  <inputs...> ; <tokens...>
//   B <elt1> <elt2> px py v1x v1y v2x v2y sense join tol oncurve        Bisector_Bisec::Perform
//   I <eltA> <eltB> <eltC> sense join tol                               Bisector_Inter of bisec(A,B) and bisec(B,C)
// Elements: L x y dx dy u1 u2 (trimmed line), C cx cy r s u1 u2 (trimmed circle, x axis, s = 1 direct),
//           E cx cy dx dy a b u1 u2 (trimmed ellipse), S x1 y1 .. x4 y4 (cubic B-spline on [0, 1]), P x y (point).
// join is GeomAbs_JoinType's value (0 arc, 2 intersection).
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

#include <Bisector_Bisec.hxx>
#include <Bisector_BisecAna.hxx>
#include <Bisector_BisecCC.hxx>
#include <Bisector_BisecPC.hxx>
#include <Bisector_Curve.hxx>
#include <Bisector_Inter.hxx>
#include <ElCLib.hxx>
#include <Geom2d_BSplineCurve.hxx>
#include <Geom2d_CartesianPoint.hxx>
#include <Geom2d_Circle.hxx>
#include <Geom2d_Ellipse.hxx>
#include <Geom2d_Hyperbola.hxx>
#include <Geom2d_Line.hxx>
#include <Geom2d_Parabola.hxx>
#include <Geom2d_TrimmedCurve.hxx>
#include <IntRes2d_Domain.hxx>
#include <IntRes2d_IntersectionPoint.hxx>
#include <IntRes2d_IntersectionSegment.hxx>
#include <NCollection_Array1.hxx>
#include <Precision.hxx>
#include <Standard_Failure.hxx>
#include <gp_Circ2d.hxx>
#include <gp_Elips2d.hxx>

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

static std::string Hex(const gp_Vec2d& theV) { return Hex(theV.X()) + "," + Hex(theV.Y()); }

static std::string Dec(double theV)
{
  char aBuf[40];
  std::snprintf(aBuf, sizeof(aBuf), "%.17g", theV);
  return aBuf;
}

static std::string TypeName(const std::type_info& theT)
{
  std::string aName = theT.name(); // "class Bisector_BisecAna"
  if (aName.rfind("class ", 0) == 0)
  {
    aName = aName.substr(6);
  }
  return aName;
}

// Runs an accessor, writing name:value or name:!Exception
static void Tok(const std::string& theName, const std::function<std::string()>& theF)
{
  std::string aVal;
  try
  {
    aVal = theF();
  }
  catch (const Standard_Failure& theFail)
  {
    aVal = "!" + TypeName(typeid(theFail));
  }
  catch (...)
  {
    aVal = "!SEH";
  }
  Put(theName + ":" + aVal);
}

//=================================================================================================
// Parsing the inputs (the C# side parses the same text the same way).

struct Reader
{
  std::istringstream myIn;

  explicit Reader(const std::string& theText) : myIn(theText) {}

  std::string Word()
  {
    std::string aW;
    myIn >> aW;
    return aW;
  }

  double D() { return std::strtod(Word().c_str(), nullptr); }

  occ::handle<Geom2d_Geometry> Elt()
  {
    std::string aKind = Word();
    if (aKind == "L")
    {
      double x = D(), y = D(), dx = D(), dy = D(), u1 = D(), u2 = D();
      occ::handle<Geom2d_Line> aL = new Geom2d_Line(gp_Pnt2d(x, y), gp_Dir2d(dx, dy));
      return new Geom2d_TrimmedCurve(aL, u1, u2);
    }
    if (aKind == "C")
    {
      double cx = D(), cy = D(), r = D(), s = D(), u1 = D(), u2 = D();
      occ::handle<Geom2d_Circle> aC =
        new Geom2d_Circle(gp_Circ2d(gp_Ax2d(gp_Pnt2d(cx, cy), gp_Dir2d(1.0, 0.0)), r, s != 0.0));
      return new Geom2d_TrimmedCurve(aC, u1, u2);
    }
    if (aKind == "E")
    {
      double cx = D(), cy = D(), dx = D(), dy = D(), a = D(), b = D(), u1 = D(), u2 = D();
      occ::handle<Geom2d_Ellipse> aE =
        new Geom2d_Ellipse(gp_Elips2d(gp_Ax2d(gp_Pnt2d(cx, cy), gp_Dir2d(dx, dy)), a, b));
      return new Geom2d_TrimmedCurve(aE, u1, u2);
    }
    if (aKind == "S")
    {
      NCollection_Array1<gp_Pnt2d> aPoles(1, 4);
      for (int i = 1; i <= 4; i++)
      {
        double x = D(), y = D();
        aPoles(i) = gp_Pnt2d(x, y);
      }
      NCollection_Array1<double> aKnots(1, 2);
      aKnots(1) = 0.0;
      aKnots(2) = 1.0;
      NCollection_Array1<int> aMults(1, 2);
      aMults(1) = 4;
      aMults(2) = 4;
      return new Geom2d_BSplineCurve(aPoles, aKnots, aMults, 3);
    }
    double x = D(), y = D(); // "P"
    return new Geom2d_CartesianPoint(x, y);
  }
};

//=================================================================================================
// Results

// The sampling range of a trimmed curve: its bounds, with infinite ends cut to 100 from the other end.
static void Range(double theF, double theL, double& theA, double& theB)
{
  theA = theF;
  theB = theL;
  if (theA < -1.e50 && theB > 1.e50)
  {
    theA = -50.0;
    theB = 50.0;
  }
  else if (theA < -1.e50)
  {
    theA = theB - 100.0;
  }
  else if (theB > 1.e50)
  {
    theB = theA + 100.0;
  }
}

static void PutBisec(const std::string& thePre, const occ::handle<Geom2d_TrimmedCurve>& theT)
{
  if (theT.IsNull())
  {
    Put(thePre + "V:null");
    return;
  }
  occ::handle<Geom2d_Curve>   aBasis = theT->BasisCurve();
  occ::handle<Bisector_Curve> aBis   = occ::down_cast<Bisector_Curve>(aBasis);
  Put(thePre + "T:" + TypeName(typeid(*aBasis)));
  occ::handle<Bisector_BisecAna> anAna = occ::down_cast<Bisector_BisecAna>(aBasis);
  if (!anAna.IsNull())
  {
    Tok(thePre + "G", [&] { return TypeName(typeid(*anAna->Geom2dCurve())); });
  }
  Tok(thePre + "F", [&] { return Hex(theT->FirstParameter()); });
  Tok(thePre + "L", [&] { return Hex(theT->LastParameter()); });
  if (aBis.IsNull())
  {
    return;
  }
  Tok(thePre + "BF", [&] { return Hex(aBis->FirstParameter()); });
  Tok(thePre + "BL", [&] { return Hex(aBis->LastParameter()); });
  Tok(thePre + "X", [&] {
    return std::string(aBis->IsExtendAtStart() ? "1" : "0") + (aBis->IsExtendAtEnd() ? "1" : "0");
  });
  Tok(thePre + "N", [&] {
    std::string aS = std::to_string(aBis->NbIntervals());
    for (int i = 1; i <= aBis->NbIntervals(); i++)
    {
      aS += "," + Hex(aBis->IntervalFirst(i)) + "," + Hex(aBis->IntervalLast(i));
    }
    return aS;
  });
  double aA, aB;
  Range(theT->FirstParameter(), theT->LastParameter(), aA, aB);
  for (int k = 0; k <= 6; k++)
  {
    double u = aA + k * (aB - aA) / 6.0;
    Tok(thePre + "P" + std::to_string(k), [&] { return Hex(theT->Value(u)); });
    Tok(thePre + "D" + std::to_string(k), [&] {
      gp_Pnt2d aP;
      gp_Vec2d aV;
      theT->D1(u, aP, aV);
      return Hex(aV);
    });
  }
  Tok(thePre + "Pm", [&] { return Hex(aBis->Parameter(theT->Value(0.5 * (aA + aB)))); });
}

//=================================================================================================
// The cases

// Bisector_Bisec::Perform as MAT2d_Tool2d::CreateBisector calls it.
static void Perform(Bisector_Bisec&                     theB,
                    const occ::handle<Geom2d_Geometry>& theE1,
                    const occ::handle<Geom2d_Geometry>& theE2,
                    const gp_Pnt2d&                     theP,
                    const gp_Vec2d&                     theV1,
                    const gp_Vec2d&                     theV2,
                    double                              theSense,
                    GeomAbs_JoinType                    theJoin,
                    double                              theTol,
                    bool                                theOnCurve)
{
  occ::handle<Geom2d_Point> aP1 = occ::down_cast<Geom2d_Point>(theE1);
  occ::handle<Geom2d_Point> aP2 = occ::down_cast<Geom2d_Point>(theE2);
  occ::handle<Geom2d_Curve> aC1 = occ::down_cast<Geom2d_Curve>(theE1);
  occ::handle<Geom2d_Curve> aC2 = occ::down_cast<Geom2d_Curve>(theE2);
  if (!aC1.IsNull() && !aC2.IsNull())
  {
    theB.Perform(aC1, aC2, theP, theV1, theV2, theSense, theJoin, theTol, theOnCurve);
  }
  else if (!aP1.IsNull() && !aP2.IsNull())
  {
    theB.Perform(aP1, aP2, theP, theV1, theV2, theSense, theTol, theOnCurve);
  }
  else if (!aP1.IsNull())
  {
    theB.Perform(aP1, aC2, theP, theV1, theV2, theSense, theTol, theOnCurve);
  }
  else
  {
    theB.Perform(aC1, aP2, theP, theV1, theV2, theSense, theTol, theOnCurve);
  }
}

// MAT2d_Tool2d's static Domain
static IntRes2d_Domain Domain(const occ::handle<Geom2d_TrimmedCurve>& Bisector1, const double Tolerance)
{
  double Param1 = Bisector1->FirstParameter();
  double Param2 = Bisector1->LastParameter();
  if (Param2 > 10000.)
  {
    Param2                                = 10000.;
    occ::handle<Geom2d_Curve>  BasisCurve = Bisector1->BasisCurve();
    occ::handle<Standard_Type> Type1      = BasisCurve->DynamicType();
    if (Type1 == STANDARD_TYPE(Bisector_BisecAna))
    {
      BasisCurve = occ::down_cast<Bisector_BisecAna>(Bisector1->BasisCurve())->Geom2dCurve();
      Type1      = BasisCurve->DynamicType();
    }
    double Limit = 50000.;
    if (Type1 == STANDARD_TYPE(Geom2d_Parabola))
    {
      gp_Parab2d gpParabola = occ::down_cast<Geom2d_Parabola>(BasisCurve)->Parab2d();
      double     Focus      = gpParabola.Focal();
      double     Val1       = std::sqrt(Limit * Focus);
      double     Val2       = std::sqrt(Limit * Limit);
      Param2                = (Val1 <= Val2 ? Val1 : Val2);
    }
    else if (Type1 == STANDARD_TYPE(Geom2d_Hyperbola))
    {
      gp_Hypr2d gpHyperbola = occ::down_cast<Geom2d_Hyperbola>(BasisCurve)->Hypr2d();
      double    Majr        = gpHyperbola.MajorRadius();
      double    Minr        = gpHyperbola.MinorRadius();
      double    Valu1       = Limit / Majr;
      double    Valu2       = Limit / Minr;
      double    Val1        = std::log(Valu1 + std::sqrt(Valu1 * Valu1 - 1));
      double    Val2        = std::log(Valu2 + std::sqrt(Valu2 * Valu2 + 1));
      Param2                = (Val1 <= Val2 ? Val1 : Val2);
    }
  }
  IntRes2d_Domain Domain1(Bisector1->Value(Param1),
                          Param1,
                          Tolerance,
                          Bisector1->Value(Param2),
                          Param2,
                          Tolerance);
  if (Bisector1->BasisCurve()->IsPeriodic())
  {
    Domain1.SetEquivalentParameters(0., 2. * M_PI);
  }
  return Domain1;
}

static void PutInter(const std::string& thePre, const Bisector_Inter& theI)
{
  Tok(thePre + "D", [&] { return std::string(theI.IsDone() ? "1" : "0"); });
  if (!theI.IsDone())
  {
    return;
  }
  Tok(thePre + "NP", [&] { return std::to_string(theI.NbPoints()); });
  for (int i = 1; i <= theI.NbPoints(); i++)
  {
    Tok(thePre + "p" + std::to_string(i), [&] {
      const IntRes2d_IntersectionPoint& aP = theI.Point(i);
      return Hex(aP.Value()) + "," + Hex(aP.ParamOnFirst()) + "," + Hex(aP.ParamOnSecond());
    });
  }
  Tok(thePre + "NS", [&] { return std::to_string(theI.NbSegments()); });
}

static void RunCase(const std::string& theIn)
{
  theOut = theIn + " ;";
  Reader aR(theIn);
  std::string anOp = aR.Word();
  try
  {
    if (anOp == "B")
    {
      occ::handle<Geom2d_Geometry> aE1 = aR.Elt();
      occ::handle<Geom2d_Geometry> aE2 = aR.Elt();
      double px = aR.D(), py = aR.D(), v1x = aR.D(), v1y = aR.D(), v2x = aR.D(), v2y = aR.D();
      double aSense = aR.D();
      int    aJoin  = (int)aR.D();
      double aTol   = aR.D();
      bool   anOn   = aR.D() != 0.0;
      Bisector_Bisec aB;
      std::string    aRes;
      try
      {
        Perform(aB, aE1, aE2, gp_Pnt2d(px, py), gp_Vec2d(v1x, v1y), gp_Vec2d(v2x, v2y), aSense,
                (GeomAbs_JoinType)aJoin, aTol, anOn);
      }
      catch (const Standard_Failure& theFail)
      {
        Put("E:!" + TypeName(typeid(theFail)));
        std::puts(theOut.c_str());
        return;
      }
      PutBisec("", aB.Value());
    }
    else
    {
      occ::handle<Geom2d_Curve> aA     = occ::down_cast<Geom2d_Curve>(aR.Elt());
      occ::handle<Geom2d_Curve> aBc    = occ::down_cast<Geom2d_Curve>(aR.Elt());
      occ::handle<Geom2d_Curve> aC     = occ::down_cast<Geom2d_Curve>(aR.Elt());
      double                    aSense = aR.D();
      int                       aJoin  = (int)aR.D();
      double                    aTol   = aR.D();
      Bisector_Bisec            aB1, aB2;
      try
      {
        gp_Pnt2d aP1 = aA->Value(aA->LastParameter());
        aB1.Perform(aA, aBc, aP1, aA->DN(aA->LastParameter(), 1), aBc->DN(aBc->FirstParameter(), 1).Reversed(),
                    aSense, (GeomAbs_JoinType)aJoin, aTol, true);
        gp_Pnt2d aP2 = aBc->Value(aBc->LastParameter());
        aB2.Perform(aBc, aC, aP2, aBc->DN(aBc->LastParameter(), 1), aC->DN(aC->FirstParameter(), 1).Reversed(),
                    aSense, (GeomAbs_JoinType)aJoin, aTol, true);
      }
      catch (const Standard_Failure& theFail)
      {
        Put("E:!" + TypeName(typeid(theFail)));
        std::puts(theOut.c_str());
        return;
      }
      PutBisec("a", aB1.Value());
      PutBisec("b", aB2.Value());
      for (int aComun = 1; aComun >= 0; aComun--)
      {
        std::string aPre = aComun ? "i" : "j";
        try
        {
          IntRes2d_Domain aD1 = Domain(aB1.Value(), aTol);
          IntRes2d_Domain aD2 = Domain(aB2.Value(), aTol);
          Bisector_Inter  anInter;
          anInter.Perform(aB1, aD1, aB2, aD2, aTol, aTol, aComun != 0);
          PutInter(aPre, anInter);
        }
        catch (const Standard_Failure& theFail)
        {
          Put(aPre + "E:!" + TypeName(typeid(theFail)));
        }
        catch (...)
        {
          Put(aPre + "E:!SEH");
        }
      }
    }
  }
  catch (const Standard_Failure& theFail)
  {
    Put("X:!" + TypeName(typeid(theFail)));
  }
  catch (...)
  {
    Put("X:!SEH");
  }
  std::puts(theOut.c_str());
}

//=================================================================================================
// Random inputs: coordinates half on a coarse grid (to reach parallel, tangent, concentric and symmetric cases),
// half rounded to multiples of 2^-10.

static uint64_t theState = 0xB15EC;

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

static double Coord() { return I(2) == 0 ? 0.5 * (I(13) - 6) : Grid(-4.0 + 8.0 * U()); }

static double Len() { return I(2) == 0 ? 0.5 * (1 + I(8)) : Grid(0.2 + 4.0 * U()); }

static void Dir(double& theX, double& theY)
{
  static const int aPairs[][2] = {{1, 0}, {0, 1}, {-1, 0}, {0, -1}, {1, 1}, {-1, 1}, {3, 4}, {4, -3}, {1, 2}, {-2, -1}};
  if (I(2) == 0)
  {
    const int* p = aPairs[I(10)];
    theX         = p[0];
    theY         = p[1];
  }
  else
  {
    double a = 2.0 * M_PI * U();
    theX     = Grid(std::cos(a));
    theY     = Grid(std::sin(a));
    if (theX == 0.0 && theY == 0.0)
    {
      theX = 1.0;
    }
  }
}

// An element of kind theKind starting at theP (or anywhere if theFree).
static std::string NewElt(char theKind, const gp_Pnt2d& theP)
{
  std::string aS;
  double      dx, dy;
  switch (theKind)
  {
    case 'L': {
      Dir(dx, dy);
      gp_Dir2d aD(dx, dy);
      aS = "L " + Dec(theP.X()) + " " + Dec(theP.Y()) + " " + Dec(dx) + " " + Dec(dy) + " 0 " + Dec(Len());
      break;
    }
    case 'C': {
      double r    = Len();
      int    s    = I(2);
      double t    = 2.0 * M_PI * U();
      double sp   = 0.3 + 2.5 * U();
      gp_Circ2d aC(gp_Ax2d(gp_Pnt2d(0.0, 0.0), gp_Dir2d(1.0, 0.0)), r, s != 0);
      gp_Pnt2d  aO = ElCLib::Value(t, aC); // the point at t of the circle centred at the origin
      double    cx = theP.X() - aO.X(), cy = theP.Y() - aO.Y();
      aS = "C " + Dec(cx) + " " + Dec(cy) + " " + Dec(r) + " " + std::to_string(s) + " " + Dec(t) + " " + Dec(t + sp);
      break;
    }
    case 'E': {
      Dir(dx, dy);
      double a  = Len() + 0.5;
      double b  = a * (0.2 + 0.7 * U());
      double t  = 2.0 * M_PI * U();
      double sp = 0.3 + 2.0 * U();
      gp_Elips2d aE(gp_Ax2d(gp_Pnt2d(0.0, 0.0), gp_Dir2d(dx, dy)), a, b);
      gp_Pnt2d   aO = ElCLib::Value(t, aE);
      double     cx = theP.X() - aO.X(), cy = theP.Y() - aO.Y();
      aS = "E " + Dec(cx) + " " + Dec(cy) + " " + Dec(dx) + " " + Dec(dy) + " " + Dec(a) + " " + Dec(b) + " " + Dec(t)
           + " " + Dec(t + sp);
      break;
    }
    case 'S': {
      aS = "S " + Dec(theP.X()) + " " + Dec(theP.Y());
      gp_Pnt2d aQ = theP;
      for (int i = 0; i < 3; i++)
      {
        Dir(dx, dy);
        double l = Len();
        aQ.SetCoord(aQ.X() + l * dx / std::sqrt(dx * dx + dy * dy), aQ.Y() + l * dy / std::sqrt(dx * dx + dy * dy));
        aS += " " + Dec(aQ.X()) + " " + Dec(aQ.Y());
      }
      break;
    }
    default:
      aS = "P " + Dec(theP.X()) + " " + Dec(theP.Y());
  }
  return aS;
}

static char CurveKind()
{
  static const char aK[] = {'L', 'L', 'C', 'C', 'E', 'S'};
  return aK[I(6)];
}

static gp_Pnt2d RandomPnt()
{
  double x = Coord();
  double y = Coord();
  return gp_Pnt2d(x, y);
}

static std::string Tail(double theSense, int theJoin, double theTol, bool theOn)
{
  return " " + Dec(theSense) + " " + std::to_string(theJoin) + " " + Dec(theTol) + " " + (theOn ? "1" : "0");
}

static std::string V(const gp_Vec2d& theV) { return " " + Dec(theV.X()) + " " + Dec(theV.Y()); }

static std::string P(const gp_Pnt2d& theP) { return " " + Dec(theP.X()) + " " + Dec(theP.Y()); }

static occ::handle<Geom2d_Curve> Curve(const std::string& theElt)
{
  Reader aR(theElt);
  return occ::down_cast<Geom2d_Curve>(aR.Elt());
}

// A bisector case of the given form: 0 joined curves, 1 curve then point at its end, 2 point at its start then
// curve, 3 two points, 4 free curves, 5 free curve and point, 6 free point and curve.
static std::string NewBisecCase(int theForm)
{
  double aSense = I(2) == 0 ? 1.0 : -1.0;
  int    aJoin  = I(3) == 0 ? 2 : 0;
  double aTol   = I(5) == 0 ? 1.e-3 : 1.e-7;
  double dx, dy;
  switch (theForm)
  {
    case 0: {
      std::string               a  = NewElt(CurveKind(), RandomPnt());
      occ::handle<Geom2d_Curve> ca = Curve(a);
      gp_Pnt2d                  pe = ca->Value(ca->LastParameter());
      std::string               b  = NewElt(CurveKind(), pe);
      occ::handle<Geom2d_Curve> cb = Curve(b);
      return "B " + a + " " + b + P(pe) + V(ca->DN(ca->LastParameter(), 1))
             + V(cb->DN(cb->FirstParameter(), 1).Reversed()) + Tail(aSense, aJoin, aTol, true);
    }
    case 1: {
      std::string               a  = NewElt(CurveKind(), RandomPnt());
      occ::handle<Geom2d_Curve> ca = Curve(a);
      gp_Pnt2d                  pe = ca->Value(ca->LastParameter());
      gp_Vec2d                  t  = ca->DN(ca->LastParameter(), 1);
      Dir(dx, dy);
      gp_Vec2d v2 = I(2) == 0 ? t.Reversed() : gp_Vec2d(dx, dy);
      return "B " + a + " P" + P(pe) + P(pe) + V(t) + V(v2) + Tail(aSense, aJoin, aTol, true);
    }
    case 2: {
      std::string               b  = NewElt(CurveKind(), RandomPnt());
      occ::handle<Geom2d_Curve> cb = Curve(b);
      gp_Pnt2d                  ps = cb->Value(cb->FirstParameter());
      gp_Vec2d                  t  = cb->DN(cb->FirstParameter(), 1);
      Dir(dx, dy);
      gp_Vec2d v1 = I(2) == 0 ? t : gp_Vec2d(dx, dy);
      return "B P" + P(ps) + " " + b + P(ps) + V(v1) + V(t.Reversed()) + Tail(aSense, aJoin, aTol, true);
    }
    case 3: {
      gp_Pnt2d p1 = RandomPnt(), p2 = RandomPnt();
      gp_Pnt2d pm(0.5 * (p1.X() + p2.X()), 0.5 * (p1.Y() + p2.Y()));
      Dir(dx, dy);
      gp_Vec2d v1(dx, dy);
      Dir(dx, dy);
      gp_Vec2d v2(dx, dy);
      return "B P" + P(p1) + " P" + P(p2) + P(I(2) == 0 ? pm : RandomPnt()) + V(v1) + V(v2)
             + Tail(aSense, aJoin, aTol, I(2) == 0);
    }
    default: {
      std::string a = theForm == 6 ? NewElt('P', RandomPnt()) : NewElt(CurveKind(), RandomPnt());
      std::string b = theForm == 5 ? NewElt('P', RandomPnt()) : NewElt(CurveKind(), RandomPnt());
      Dir(dx, dy);
      gp_Vec2d v1(dx, dy);
      Dir(dx, dy);
      gp_Vec2d v2(dx, dy);
      return "B " + a + " " + b + P(RandomPnt()) + V(v1) + V(v2) + Tail(aSense, aJoin, aTol, false);
    }
  }
}

static std::string NewInterCase()
{
  double                    aSense = I(2) == 0 ? 1.0 : -1.0;
  int                       aJoin  = I(3) == 0 ? 2 : 0;
  std::string               a      = NewElt(CurveKind(), RandomPnt());
  occ::handle<Geom2d_Curve> ca     = Curve(a);
  std::string               b      = NewElt(CurveKind(), ca->Value(ca->LastParameter()));
  occ::handle<Geom2d_Curve> cb     = Curve(b);
  std::string               c      = NewElt(CurveKind(), cb->Value(cb->LastParameter()));
  return "I " + a + " " + b + " " + c + " " + Dec(aSense) + " " + std::to_string(aJoin) + " 1e-07";
}

// Hand-picked cases: a square's corners, concentric and tangent circles, a line tangent to a circle, parallel and
// collinear lines, coincident points, a point on a line, a point at a circle's centre.
static const char* thePresets[] = {
  "B L 0 0 1 0 0 2 L 2 0 0 1 0 2 2 0 1 0 0 -1 1 0 1e-07 1",
  "B L 0 0 1 0 0 2 L 2 0 0 1 0 2 2 0 1 0 0 -1 -1 0 1e-07 1",
  "B L 0 0 1 0 0 2 L 2 0 1 0 0 2 2 0 1 0 -1 0 1 0 1e-07 1",
  "B L 0 0 1 0 0 2 L 2 0 -1 0 0 2 2 0 1 0 1 0 1 0 1e-07 1",
  "B L 0 0 1 0 0 2 L 0 1 1 0 0 2 0.5 0.5 1 0 -1 0 1 0 1e-07 0",
  "B C 0 0 1 1 0 1.5 C 0 0 2 1 1.5 3 1 0 0 1 0 -1 1 0 1e-07 0",
  "B C 0 0 1 1 0 1.5 C 0 0 1 1 1.5 3 0.0707372016677029 0.997494986604054 -0.997494986604054 0.0707372016677029 0.997494986604054 -0.0707372016677029 1 0 1e-07 1",
  "B C 0 0 1 1 0 1.5707963267948966 C 0 2 1 0 4.71238898038469 6 0 1 -1 0 1 0 1 0 1e-07 1",
  "B L -2 1 1 0 0 2 C 0 0 1 0 1.5707963267948966 3 0 1 1 0 1 0 1 0 1e-07 1",
  "B L -2 1 1 0 0 2 C 0 0 1 0 1.5707963267948966 3 0 1 1 0 1 0 1 2 1e-07 1",
  "B P 0 0 P 0 0 0 0 1 0 0 1 1 0 1e-07 1",
  "B P 0 0 P 2 0 1 0 0 1 0 -1 1 0 1e-07 0",
  "B L 0 0 1 0 -1 1 P 0 1 0 0.5 1 0 -1 0 1 0 1e-07 0",
  "B C 0 0 1 1 0 3 P 0 0 1 0 0 1 1 0 1 0 1e-07 0",
  "B E 0 0 1 0 2 1 0 1.5 E 0 0 1 0 2 1 1.5 3 0 1 -1 0 1 0 1 0 1e-07 1",
  "B S 0 0 1 1 2 1 3 0 L 3 0 1 -1 0 1 3 0 1 -1 -1 1 1 0 1e-07 1",
  "B S 0 0 1 1 2 1 3 0 P 3 0 3 0 1 -1 -1 1 1 0 1e-07 1",
  "B P 0 0 S 0 0 1 1 2 1 3 0 0 0 1 1 -1 -1 -1 0 1e-07 1",
  "I L 0 0 1 0 0 2 L 2 0 0 1 0 2 L 2 2 -1 0 0 2 1 0 1e-07",
  "I L 0 0 1 0 0 2 L 2 0 0 1 0 2 L 2 2 -1 0 0 2 -1 0 1e-07",
  "I L 0 0 1 0 0 2 C 2 1 1 1 4.71238898038469 7.853981633974483 L 2 2 -1 0 0 2 1 0 1e-07",
  "I E 0 0 1 0 2 1 0 1 E 0 0 1 0 2 1 1 2 E 0 0 1 0 2 1 2 3 1 0 1e-07",
  "I S 0 0 1 1 2 1 3 0 S 3 0 4 -1 5 -1 6 0 S 6 0 7 1 8 1 9 0 1 0 1e-07",
};

int main(int argc, char** argv)
{
  int aNbCases = argc > 1 ? std::atoi(argv[1]) : 100;
  for (int aForm = 0; aForm <= 6; aForm++)
  {
    int aNb = aForm == 0 ? 3 * aNbCases : aNbCases;
    for (int k = 0; k < aNb; k++)
    {
      RunCase(NewBisecCase(aForm));
    }
  }
  for (int k = 0; k < 2 * aNbCases; k++)
  {
    RunCase(NewInterCase());
  }
  for (const char* aPreset : thePresets)
  {
    RunCase(aPreset);
  }
  return 0;
}
