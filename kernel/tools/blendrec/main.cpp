// blendrec: records OCCT's blend functions (BlendFunc) and fillet spines (ChFiDS) for the tests of unit 7J, every double
// as its 64 bits in hex. BlendFunc and ChFiDS are not bound in build123d.NET, so this is built with MSVC against its
// native OCCT V8_0_1. tests/OcctNet.ModelingAlgorithms.Tests/BlendFunc/BlendRecorder.cs is the same program in C#,
// line for line: the tests run it on the port and compare the two texts.
#include <cstdio>
#include <cstring>
#include <cstdint>
#include <cmath>
#include <string>
#include <typeinfo>

#include <Blend_Point.hxx>
#include <BlendFunc_ChAsym.hxx>
#include <BlendFunc_ChAsymInv.hxx>
#include <BlendFunc_ChamfInv.hxx>
#include <BlendFunc_Chamfer.hxx>
#include <BlendFunc_ConstRad.hxx>
#include <BlendFunc_ConstRadInv.hxx>
#include <BlendFunc_ConstThroat.hxx>
#include <BlendFunc_ConstThroatInv.hxx>
#include <BlendFunc_ConstThroatWithPenetration.hxx>
#include <BlendFunc_ConstThroatWithPenetrationInv.hxx>
#include <BlendFunc_CSCircular.hxx>
#include <BlendFunc_CSConstRad.hxx>
#include <BlendFunc_EvolRad.hxx>
#include <BlendFunc_EvolRadInv.hxx>
#include <BlendFunc_Ruled.hxx>
#include <BlendFunc_RuledInv.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRep_Tool.hxx>
#include <ChFiDS_ElSpine.hxx>
#include <ChFiDS_FilSpine.hxx>
#include <ChFiDS_Spine.hxx>
#include <GeomAdaptor_Curve.hxx>
#include <GeomAdaptor_Surface.hxx>
#include <GeomConvert.hxx>
#include <Geom2dAdaptor_Curve.hxx>
#include <Geom2d_Line.hxx>
#include <Geom_BSplineCurve.hxx>
#include <Geom_BSplineSurface.hxx>
#include <Geom_Circle.hxx>
#include <Geom_ConicalSurface.hxx>
#include <Geom_CylindricalSurface.hxx>
#include <Geom_Line.hxx>
#include <Geom_Plane.hxx>
#include <Geom_TrimmedCurve.hxx>
#include <Law_Composite.hxx>
#include <Law_Linear.hxx>
#include <NCollection_Array1.hxx>
#include <NCollection_Array2.hxx>
#include <OSD.hxx>
#include <Standard_Failure.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <gp_Ax3.hxx>
#include <gp_Circ.hxx>
#include <gp_Lin.hxx>
#include <math_FunctionSetRoot.hxx>
#include <math_Matrix.hxx>
#include <math_Vector.hxx>

static FILE* gOut = stdout;

static void Hex(double theValue)
{
  uint64_t aBits;
  std::memcpy(&aBits, &theValue, 8);
  std::fprintf(gOut, " %016llx", (unsigned long long)aBits);
}

static void Tag(const char* theTag) { std::fprintf(gOut, "%s", theTag); }
static void Int(int theValue) { std::fprintf(gOut, " %d", theValue); }
static void End()
{
  std::fprintf(gOut, "\n");
  std::fflush(gOut);
}
static void Pnt(const gp_Pnt& P) { Hex(P.X()); Hex(P.Y()); Hex(P.Z()); }
static void Vec(const gp_Vec& V) { Hex(V.X()); Hex(V.Y()); Hex(V.Z()); }
static void Pnt2d(const gp_Pnt2d& P) { Hex(P.X()); Hex(P.Y()); }
static void Vec2d(const gp_Vec2d& V) { Hex(V.X()); Hex(V.Y()); }

static void Vector(const char* theTag, const math_Vector& V)
{
  Tag(theTag);
  for (int i = V.Lower(); i <= V.Upper(); i++)
  {
    Hex(V(i));
  }
  End();
}

static void Matrix(const char* theTag, const math_Matrix& M)
{
  Tag(theTag);
  for (int i = M.LowerRow(); i <= M.UpperRow(); i++)
  {
    for (int j = M.LowerCol(); j <= M.UpperCol(); j++)
    {
      Hex(M(i, j));
    }
  }
  End();
}

static void Fail(const char* theTag)
{
  Tag(theTag);
  Tag(" X");
  End();
}

// ---------------------------------------------------------------------------------------------------------------------
// Geometry (the same in BlendRecorder.cs)

static occ::handle<Geom_BSplineSurface> Patch()
{
  NCollection_Array2<gp_Pnt> aPoles(1, 4, 1, 4);
  const double               aZ[4][4] = {{0.10, -0.20, 0.30, 0.00},
                                         {-0.30, 0.40, -0.10, 0.20},
                                         {0.20, -0.10, 0.50, -0.30},
                                         {0.00, 0.30, -0.20, 0.10}};
  for (int i = 1; i <= 4; i++)
  {
    for (int j = 1; j <= 4; j++)
    {
      aPoles(i, j) = gp_Pnt(3.0 * (i - 1) / 3.0, 3.0 * (j - 1) / 3.0, aZ[i - 1][j - 1]);
    }
  }
  NCollection_Array1<double> aKnots(1, 2);
  aKnots(1) = 0.0;
  aKnots(2) = 1.0;
  NCollection_Array1<int> aMults(1, 2);
  aMults(1) = 4;
  aMults(2) = 4;
  return new Geom_BSplineSurface(aPoles, aKnots, aKnots, aMults, aMults, 3, 3);
}

static occ::handle<Geom_BSplineSurface> Wall()
{
  NCollection_Array2<gp_Pnt> aPoles(1, 2, 1, 2);
  for (int i = 1; i <= 2; i++)
  {
    for (int j = 1; j <= 2; j++)
    {
      aPoles(i, j) = gp_Pnt(1.5, -1.0 + 5.0 * (i - 1), -2.0 + 4.0 * (j - 1));
    }
  }
  NCollection_Array1<double> aKnots(1, 2);
  aKnots(1) = 0.0;
  aKnots(2) = 1.0;
  NCollection_Array1<int> aMults(1, 2);
  aMults(1) = 2;
  aMults(2) = 2;
  return new Geom_BSplineSurface(aPoles, aKnots, aKnots, aMults, aMults, 1, 1);
}

static occ::handle<Adaptor3d_Surface> Surf(int theId)
{
  occ::handle<Geom_Surface> aS;
  switch (theId)
  {
    case 0: aS = new Geom_Plane(gp_Ax3(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1), gp_Dir(1, 0, 0))); break;
    case 1: aS = new Geom_Plane(gp_Ax3(gp_Pnt(0, 0, 0), gp_Dir(0, 1, 0), gp_Dir(1, 0, 0))); break;
    case 2: aS = new Geom_CylindricalSurface(gp_Ax3(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)), 5.0); break;
    case 3: aS = new Geom_CylindricalSurface(gp_Ax3(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)), 3.0); break;
    case 4: aS = new Geom_CylindricalSurface(gp_Ax3(gp_Pnt(5, 0, 0), gp_Dir(0, 0, 1)), 3.0); break;
    case 5: aS = new Geom_ConicalSurface(gp_Ax3(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)), M_PI / 6.0, 5.0); break;
    case 6: aS = Patch(); break;
    default: aS = Wall(); break;
  }
  return new GeomAdaptor_Surface(aS);
}

static occ::handle<Adaptor3d_Curve> Guide(int theId)
{
  switch (theId)
  {
    case 0: return new GeomAdaptor_Curve(new Geom_Line(gp_Pnt(0, 0, 0), gp_Dir(1, 0, 0)), 0.0, 10.0);
    case 1:
      return new GeomAdaptor_Curve(new Geom_Circle(gp_Ax2(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)), 5.0), 0.0, 2.0 * M_PI);
    case 2:
      return new GeomAdaptor_Curve(new Geom_Line(gp_Pnt(2.5, std::sqrt(2.75), 0), gp_Dir(0, 0, 1)), -5.0, 5.0);
    case 3: return new GeomAdaptor_Curve(new Geom_Line(gp_Pnt(1.5, 0, 0), gp_Dir(0, 1, 0)), 0.0, 3.0);
    default: return new GeomAdaptor_Curve(new Geom_Line(gp_Pnt(0, -1, 1.5), gp_Dir(1, 0, 0)), -2.0, 12.0);
  }
}

// The pairs of surfaces: S1, S2, guide, guide parameter, start point.
struct PairCase
{
  int    S1, S2, G;
  double W;
  double X0[4];
};

static const PairCase THE_PAIRS[] = {
  {0, 1, 0, 4.0, {4.0, 1.0, 4.0, -1.0}},                        // plane / plane
  {0, 2, 1, 1.0, {4.0 * 0.5403023058681398, 4.0 * 0.8414709848078965, 1.0, 1.0}}, // plane / cylinder
  {3, 4, 2, 0.5, {0.2857, 0.5, 2.8559, 0.5}},                   // cylinder / cylinder
  {5, 0, 1, 1.0, {1.0, 1.0, 4.0 * 0.5403023058681398, 4.0 * 0.8414709848078965}}, // cone / plane
  {6, 7, 3, 1.5, {0.83, 0.5, 0.5, 0.75}},                       // B-spline / B-spline
};

// ---------------------------------------------------------------------------------------------------------------------
// Functions

static bool Solve(math_FunctionSetWithDerivatives& F,
                  const math_Vector&               X0,
                  const math_Vector&               Tol,
                  const math_Vector&               Inf,
                  const math_Vector&               Sup,
                  math_Vector&                     Sol)
{
  math_FunctionSetRoot aRoot(F, Tol, 30);
  aRoot.Perform(F, X0, Inf, Sup);
  if (!aRoot.IsDone())
  {
    Tag("root 0");
    End();
    return false;
  }
  aRoot.Root(Sol);
  Vector("root 1", Sol);
  return true;
}

static void RecValues(math_FunctionSetWithDerivatives& F, const math_Vector& X)
{
  const int   n = F.NbEquations(), m = F.NbVariables();
  math_Vector aF(1, n);
  math_Matrix aD(1, n, 1, m);
  Tag("dims");
  Int(n);
  Int(m);
  End();
  try
  {
    const bool isOk = F.Value(X, aF);
    Tag("ok");
    Int(isOk ? 1 : 0);
    End();
    Vector("value", aF);
  }
  catch (Standard_Failure&)
  {
    Fail("value");
  }
  try
  {
    F.Derivatives(X, aD);
    Matrix("derivatives", aD);
  }
  catch (Standard_Failure&)
  {
    Fail("derivatives");
  }
  try
  {
    F.Values(X, aF, aD);
    Vector("values.f", aF);
    Matrix("values.d", aD);
  }
  catch (Standard_Failure&)
  {
    Fail("values");
  }
}

// What the approximation reads of any Blend_AppFunction.
static void RecApp(Blend_AppFunction& F)
{
  int aNbPoles = 0, aNbKnots = 0, aDegree = 0, aNbPoles2d = 0;
  F.GetShape(aNbPoles, aNbKnots, aDegree, aNbPoles2d);
  Tag("shape");
  Int(aNbPoles);
  Int(aNbKnots);
  Int(aDegree);
  Int(aNbPoles2d);
  Int(F.IsRational() ? 1 : 0);
  End();
  NCollection_Array1<double> aKnots(1, aNbKnots);
  NCollection_Array1<int>    aMults(1, aNbKnots);
  F.Knots(aKnots);
  F.Mults(aMults);
  Tag("knots");
  for (int i = 1; i <= aNbKnots; i++)
  {
    Hex(aKnots(i));
    Int(aMults(i));
  }
  End();
  NCollection_Array1<double> aW(1, aNbPoles);
  F.GetMinimalWeight(aW);
  Tag("minweight");
  for (int i = 1; i <= aNbPoles; i++)
  {
    Hex(aW(i));
  }
  End();
  try
  {
    const double aSize = F.GetSectionSize();
    Tag("sectionsize");
    Hex(aSize);
    End();
  }
  catch (Standard_Failure&)
  {
    Fail("sectionsize");
  }
  try
  {
    const double aDist = F.GetMinimalDistance();
    Tag("mindist");
    Hex(aDist);
    End();
  }
  catch (Standard_Failure&)
  {
    Fail("mindist");
  }
  math_Vector aTol3d(1, aNbPoles, 0.0), aTol1d(1, aNbPoles, 0.0); // GetTolerance may leave Tol1d unset
  try
  {
    F.GetTolerance(1.e-4, 1.e-5, 1.e-3, aTol3d, aTol1d);
    Vector("tol3d", aTol3d);
    Vector("tol1d", aTol1d);
  }
  catch (Standard_Failure&)
  {
    Fail("tol3d");
  }

  const int aNbInt = F.NbIntervals(GeomAbs_C1);
  NCollection_Array1<double> aT(1, aNbInt + 1);
  F.Intervals(aT, GeomAbs_C1);
  Tag("intervals");
  for (int i = 1; i <= aNbInt + 1; i++)
  {
    Hex(aT(i));
  }
  End();
  double aTolU = 0., aTolV = 0.;
  F.Resolution(1, 1.e-3, aTolU, aTolV);
  Tag("resolution");
  Hex(aTolU);
  Hex(aTolV);
  End();
}

static void RecSections(Blend_AppFunction& F, const Blend_Point& P, int theNbPoles2d)
{
  int aNbPoles = 0, aNbKnots = 0, aDegree = 0, aNbPoles2d = 0;
  F.GetShape(aNbPoles, aNbKnots, aDegree, aNbPoles2d);
  NCollection_Array1<gp_Pnt>   aPoles(1, aNbPoles);
  NCollection_Array1<gp_Vec>   aDPoles(1, aNbPoles), aD2Poles(1, aNbPoles);
  NCollection_Array1<gp_Pnt2d> aPoles2d(1, theNbPoles2d);
  NCollection_Array1<gp_Vec2d> aDPoles2d(1, theNbPoles2d), aD2Poles2d(1, theNbPoles2d);
  NCollection_Array1<double>   aW(1, aNbPoles), aDW(1, aNbPoles), aD2W(1, aNbPoles);
  try
  {
    F.Section(P, aPoles, aPoles2d, aW);
    Tag("section0");
    for (int i = 1; i <= aNbPoles; i++)
    {
      Pnt(aPoles(i));
      Hex(aW(i));
    }
    for (int i = 1; i <= theNbPoles2d; i++)
    {
      Pnt2d(aPoles2d(i));
    }
    End();
  }
  catch (Standard_Failure&)
  {
    Fail("section0");
  }
  try
  {
    const bool isD = F.Section(P, aPoles, aDPoles, aPoles2d, aDPoles2d, aW, aDW);
    Tag("section1");
    Int(isD ? 1 : 0);
    for (int i = 1; i <= aNbPoles; i++)
    {
      Pnt(aPoles(i));
      Hex(aW(i));
      if (isD)
      {
        Vec(aDPoles(i));
        Hex(aDW(i));
      }
    }
    for (int i = 1; i <= theNbPoles2d; i++)
    {
      Pnt2d(aPoles2d(i));
      if (isD)
      {
        Vec2d(aDPoles2d(i));
      }
    }
    End();
  }
  catch (Standard_Failure&)
  {
    Fail("section1");
  }
  try
  {
    const bool isD =
      F.Section(P, aPoles, aDPoles, aD2Poles, aPoles2d, aDPoles2d, aD2Poles2d, aW, aDW, aD2W);
    Tag("section2");
    Int(isD ? 1 : 0);
    if (isD)
    {
      for (int i = 1; i <= aNbPoles; i++)
      {
        Pnt(aPoles(i));
        Vec(aDPoles(i));
        Vec(aD2Poles(i));
        Hex(aW(i));
        Hex(aDW(i));
        Hex(aD2W(i));
      }
      for (int i = 1; i <= theNbPoles2d; i++)
      {
        Pnt2d(aPoles2d(i));
        Vec2d(aDPoles2d(i));
        Vec2d(aD2Poles2d(i));
      }
    }
    End();
  }
  catch (Standard_Failure&)
  {
    Fail("section2");
  }
}

// A function between two surfaces.
static void RecFunction(Blend_Function& F, const PairCase& C)
{
  F.Set(C.W);
  math_Vector X0(1, 4), Tol(1, 4), Inf(1, 4), Sup(1, 4), Sol(1, 4);
  for (int i = 1; i <= 4; i++)
  {
    X0(i) = C.X0[i - 1];
  }
  RecValues(F, X0);
  F.GetTolerance(Tol, 1.e-7);
  Vector("tolerance", Tol);
  F.GetBounds(Inf, Sup);
  Vector("inf", Inf);
  Vector("sup", Sup);
  if (!Solve(F, X0, Tol, Inf, Sup, Sol))
  {
    return;
  }
  bool isSol = false;
  try
  {
    isSol = F.IsSolution(Sol, 1.e-6);
  }
  catch (Standard_Failure&)
  {
    Fail("issolution");
    return;
  }
  Tag("issolution");
  Int(isSol ? 1 : 0);
  Int(F.IsTangencyPoint() ? 1 : 0);
  Pnt(F.PointOnS1());
  Pnt(F.PointOnS2());
  End();
  if (!F.IsTangencyPoint())
  {
    Tag("tangents");
    Vec(F.TangentOnS1());
    Vec(F.TangentOnS2());
    Vec2d(F.Tangent2dOnS1());
    Vec2d(F.Tangent2dOnS2());
    try
    {
      Int(F.TwistOnS1() ? 1 : 0);
      Int(F.TwistOnS2() ? 1 : 0);
    }
    catch (Standard_Failure&)
    {
      Tag(" X");
    }
    End();
  }
  gp_Vec aTgF, aTgL, aNmF, aNmL;
  F.Tangent(Sol(1), Sol(2), Sol(3), Sol(4), aTgF, aTgL, aNmF, aNmL);
  Tag("tangent");
  Vec(aTgF);
  Vec(aTgL);
  Vec(aNmF);
  Vec(aNmL);
  End();
  RecApp(F);
  Blend_Point aP;
  if (F.IsTangencyPoint())
  {
    aP = Blend_Point(F.PointOnS1(), F.PointOnS2(), C.W, Sol(1), Sol(2), Sol(3), Sol(4));
  }
  else
  {
    aP = Blend_Point(F.PointOnS1(),
                     F.PointOnS2(),
                     C.W,
                     Sol(1),
                     Sol(2),
                     Sol(3),
                     Sol(4),
                     F.TangentOnS1(),
                     F.TangentOnS2(),
                     F.Tangent2dOnS1(),
                     F.Tangent2dOnS2());
  }
  RecSections(F, aP, 2);
}

static void RecInv(Blend_FuncInv& F, const math_Vector& X0)
{
  math_Vector Tol(1, 4), Inf(1, 4), Sup(1, 4), Sol(1, 4);
  RecValues(F, X0);
  F.GetTolerance(Tol, 1.e-7);
  Vector("tolerance", Tol);
  F.GetBounds(Inf, Sup);
  Vector("inf", Inf);
  Vector("sup", Sup);
  if (!Solve(F, X0, Tol, Inf, Sup, Sol))
  {
    return;
  }
  bool isSol = false;
  try
  {
    isSol = F.IsSolution(Sol, 1.e-6);
  }
  catch (Standard_Failure&)
  {
    Fail("issolution");
    return;
  }
  Tag("issolution");
  Int(isSol ? 1 : 0);
  End();
}

static void RecCS(Blend_CSFunction& F, double theW, const math_Vector& X0)
{
  const int   m = F.NbVariables();
  math_Vector Tol(1, m), Inf(1, m), Sup(1, m), Sol(1, m);
  F.Set(theW);
  RecValues(F, X0);
  F.GetTolerance(Tol, 1.e-7);
  Vector("tolerance", Tol);
  F.GetBounds(Inf, Sup);
  Vector("inf", Inf);
  Vector("sup", Sup);
  if (!Solve(F, X0, Tol, Inf, Sup, Sol))
  {
    return;
  }
  const bool isSol = F.IsSolution(Sol, 1.e-6);
  Tag("issolution");
  Int(isSol ? 1 : 0);
  Int(F.IsTangencyPoint() ? 1 : 0);
  Pnt(F.PointOnS());
  Pnt(F.PointOnC());
  Pnt2d(F.Pnt2d());
  Hex(F.ParameterOnC());
  End();
  if (!F.IsTangencyPoint())
  {
    Tag("tangents");
    Vec(F.TangentOnS());
    Vec(F.TangentOnC());
    Vec2d(F.Tangent2d());
    End();
  }
  gp_Vec aTgS, aNmS;
  F.Tangent(Sol(1), Sol(2), aTgS, aNmS);
  Tag("tangent");
  Vec(aTgS);
  Vec(aNmS);
  End();
  RecApp(F);
  const double aW = (m == 3) ? Sol(3) : F.ParameterOnC();
  Blend_Point  aP;
  if (F.IsTangencyPoint())
  {
    aP = Blend_Point(F.PointOnS(), F.PointOnC(), theW, Sol(1), Sol(2), aW);
  }
  else
  {
    aP = Blend_Point(F.PointOnS(),
                     F.PointOnC(),
                     theW,
                     Sol(1),
                     Sol(2),
                     aW,
                     F.TangentOnS(),
                     F.TangentOnC(),
                     F.Tangent2d());
  }
  RecSections(F, aP, 1);
}

static void RecordFunctions()
{
  const int aNbPairs = (int)(sizeof(THE_PAIRS) / sizeof(THE_PAIRS[0]));
  for (int ip = 0; ip < aNbPairs; ip++)
  {
    const PairCase&                C  = THE_PAIRS[ip];
    occ::handle<Adaptor3d_Surface> S1 = Surf(C.S1), S2 = Surf(C.S2);
    occ::handle<Adaptor3d_Curve>   G  = Guide(C.G);
    for (int choix = 1; choix <= 8; choix++)
    {
      std::fprintf(gOut, "case constrad %d %d\n", ip, choix);
      BlendFunc_ConstRad aConstRad(S1, S2, G);
      aConstRad.Set(1.0, choix);
      aConstRad.Set(G->FirstParameter(), G->LastParameter());
      RecFunction(aConstRad, C);
      gp_Circ aCirc;
      double  aPdeb = 0., aPfin = 0.;
      aConstRad.Section(C.W, C.X0[0], C.X0[1], C.X0[2], C.X0[3], aPdeb, aPfin, aCirc);
      Tag("circle");
      Pnt(aCirc.Location());
      Vec(gp_Vec(aCirc.Axis().Direction()));
      Vec(gp_Vec(aCirc.XAxis().Direction()));
      Hex(aCirc.Radius());
      Hex(aPdeb);
      Hex(aPfin);
      End();
      const gp_Ax1 anAx = aConstRad.AxeRot(C.W);
      Tag("axerot");
      Pnt(anAx.Location());
      Vec(gp_Vec(anAx.Direction()));
      End();

      std::fprintf(gOut, "case evolrad %d %d\n", ip, choix);
      occ::handle<Law_Linear> aLaw = new Law_Linear();
      aLaw->Set(G->FirstParameter(), 0.8, G->LastParameter(), 1.4);
      occ::handle<Law_Composite> aComp =
        new Law_Composite(G->FirstParameter(), G->LastParameter(), 1.e-9);
      aComp->ChangeLaws().Append(aLaw);
      BlendFunc_EvolRad anEvolRad(S1, S2, G, aComp);
      anEvolRad.Set(choix);
      anEvolRad.Set(G->FirstParameter(), G->LastParameter());
      RecFunction(anEvolRad, C);
    }
    for (int choix = 1; choix <= 2; choix++)
    {
      std::fprintf(gOut, "case chamfer %d %d\n", ip, choix);
      BlendFunc_Chamfer aChamfer(S1, S2, G);
      aChamfer.Set(1.0, 1.5, choix);
      RecFunction(aChamfer, C);

      std::fprintf(gOut, "case constthroat %d %d\n", ip, choix);
      BlendFunc_ConstThroat aThroat(S1, S2, G);
      aThroat.Set(0.7, 0., choix);
      RecFunction(aThroat, C);

      std::fprintf(gOut, "case constthroatpen %d %d\n", ip, choix);
      BlendFunc_ConstThroatWithPenetration aPen(S1, S2, G);
      aPen.Set(1.0, 0., choix);
      RecFunction(aPen, C);

      std::fprintf(gOut, "case chasym %d %d\n", ip, choix);
      BlendFunc_ChAsym aChAsym(S1, S2, G);
      aChAsym.Set(1.0, M_PI / 3.0, choix);
      aChAsym.Set(G->FirstParameter(), G->LastParameter());
      RecFunction(aChAsym, C);

      std::fprintf(gOut, "case ruled %d %d\n", ip, choix);
      BlendFunc_Ruled aRuled(S1, S2, G);
      RecFunction(aRuled, C);
      NCollection_Array1<gp_Pnt> aTabP(1, 5);
      NCollection_Array1<gp_Vec> aTabV(1, 5);
      bool isOk = false;
      try
      {
        isOk = aRuled.GetSection(C.W, C.X0[0], C.X0[1], C.X0[2], C.X0[3], aTabP, aTabV);
        Tag("getsection");
        Int(isOk ? 1 : 0);
        if (isOk)
        {
          for (int i = 1; i <= 5; i++)
          {
            Pnt(aTabP(i));
            Vec(aTabV(i));
          }
        }
        End();
      }
      catch (Standard_Failure&)
      {
        Fail("getsection");
      }
    }
  }

  // The inverse functions: a restriction (a 2d line) on the first surface, or on the second.
  struct InvCase
  {
    int    Pair;
    bool   OnFirst;
    double P0[2], D0[2];
    double X0[4];
  };

  static const InvCase THE_INVS[] = {
    {0, true, {5.0, 0.0}, {0.0, 1.0}, {1.0, 5.0, 5.0, -1.0}},
    {0, false, {5.0, 0.0}, {0.0, 1.0}, {-1.0, 5.0, 5.0, 1.0}},
    {1, true, {0.0, 0.0}, {0.3623577544766736, 0.9320390859672263}, {4.0, 1.2, 1.2, 1.0}},
    {4, true, {0.0, 0.5}, {1.0, 0.0}, {0.83, 1.5, 0.5, 0.75}},
  };
  const int aNbInv = (int)(sizeof(THE_INVS) / sizeof(THE_INVS[0]));
  for (int ii = 0; ii < aNbInv; ii++)
  {
    const InvCase&                 I  = THE_INVS[ii];
    const PairCase&                C  = THE_PAIRS[I.Pair];
    occ::handle<Adaptor3d_Surface> S1 = Surf(C.S1), S2 = Surf(C.S2);
    occ::handle<Adaptor3d_Curve>   G  = Guide(C.G);
    occ::handle<Adaptor2d_Curve2d> R  = new Geom2dAdaptor_Curve(
      new Geom2d_Line(gp_Pnt2d(I.P0[0], I.P0[1]), gp_Dir2d(I.D0[0], I.D0[1])), -10.0, 10.0);
    math_Vector X0(1, 4);
    for (int i = 1; i <= 4; i++)
    {
      X0(i) = I.X0[i - 1];
    }
    for (int choix = 1; choix <= 8; choix += 3)
    {
      std::fprintf(gOut, "case constradinv %d %d\n", ii, choix);
      BlendFunc_ConstRadInv aConstRad(S1, S2, G);
      aConstRad.Set(1.0, choix);
      aConstRad.Set(I.OnFirst, R);
      RecInv(aConstRad, X0);

      std::fprintf(gOut, "case evolradinv %d %d\n", ii, choix);
      occ::handle<Law_Linear> aLaw = new Law_Linear();
      aLaw->Set(G->FirstParameter(), 0.8, G->LastParameter(), 1.4);
      BlendFunc_EvolRadInv anEvolRad(S1, S2, G, aLaw);
      anEvolRad.Set(choix);
      anEvolRad.Set(I.OnFirst, R);
      RecInv(anEvolRad, X0);

      std::fprintf(gOut, "case chamfinv %d %d\n", ii, choix);
      BlendFunc_ChamfInv aChamf(S1, S2, G);
      aChamf.Set(1.0, 1.5, choix);
      aChamf.Set(I.OnFirst, R);
      RecInv(aChamf, X0);

      std::fprintf(gOut, "case constthroatinv %d %d\n", ii, choix);
      BlendFunc_ConstThroatInv aThroat(S1, S2, G);
      aThroat.Set(0.7, 0., choix);
      aThroat.Set(I.OnFirst, R);
      RecInv(aThroat, X0);
      RecInv(aThroat, X0); // again: Derivatives reads the previous call's points

      std::fprintf(gOut, "case constthroatpeninv %d %d\n", ii, choix);
      BlendFunc_ConstThroatWithPenetrationInv aPen(S1, S2, G);
      aPen.Set(1.0, 0., choix);
      aPen.Set(I.OnFirst, R);
      RecInv(aPen, X0);

      std::fprintf(gOut, "case chasyminv %d %d\n", ii, choix);
      BlendFunc_ChAsymInv aChAsym(S1, S2, G);
      aChAsym.Set(1.0, M_PI / 3.0, choix);
      aChAsym.Set(I.OnFirst, R);
      RecInv(aChAsym, X0);

      std::fprintf(gOut, "case ruledinv %d %d\n", ii, choix);
      BlendFunc_RuledInv aRuled(S1, S2, G);
      aRuled.Set(I.OnFirst, R);
      RecInv(aRuled, X0);
    }
  }

  // A surface and a curve.
  {
    occ::handle<Adaptor3d_Surface> S  = Surf(0);
    occ::handle<Adaptor3d_Curve>   C  = Guide(4);
    occ::handle<Adaptor3d_Curve>   G  = Guide(0);
    math_Vector                    X3(1, 3), X2(1, 2);
    X3(1)                             = 3.0;
    X3(2)                             = -0.5;
    X3(3)                             = 3.0;
    X2(1)                             = 3.0;
    X2(2)                             = -0.5;
    for (int choix = 1; choix <= 8; choix++)
    {
      std::fprintf(gOut, "case csconstrad %d\n", choix);
      BlendFunc_CSConstRad aCS(S, C, G);
      aCS.Set(1.0, choix);
      RecCS(aCS, 3.0, X3);
      NCollection_Array1<gp_Pnt> aTabP(1, 5);
      NCollection_Array1<gp_Vec> aTabV(1, 5);
      Tag("getsection");
      const bool isOk = aCS.GetSection(3.0, X3(1), X3(2), X3(3), aTabP, aTabV);
      Int(isOk ? 1 : 0);
      if (isOk)
      {
        for (int i = 1; i <= 5; i++)
        {
          Pnt(aTabP(i));
          Vec(aTabV(i));
        }
      }
      End();
      gp_Circ aCirc;
      double  aPdeb = 0., aPfin = 0.;
      aCS.Section(3.0, X3(1), X3(2), X3(3), aPdeb, aPfin, aCirc);
      Tag("circle");
      Pnt(aCirc.Location());
      Vec(gp_Vec(aCirc.Axis().Direction()));
      Hex(aCirc.Radius());
      Hex(aPdeb);
      Hex(aPfin);
      End();

      std::fprintf(gOut, "case cscircular %d\n", choix);
      occ::handle<Law_Linear> aLaw = new Law_Linear();
      aLaw->Set(0.0, 0.0, 10.0, 10.0);
      BlendFunc_CSCircular aCirc2(S, C, G, aLaw);
      aCirc2.Set(1.0, choix);
      RecCS(aCirc2, 3.0, X2);
    }
  }
}

// ---------------------------------------------------------------------------------------------------------------------
// Spines

static void RecSpineEval(occ::handle<ChFiDS_Spine>& Sp, double theFrom, double theTo)
{
  for (int k = 0; k <= 12; k++)
  {
    const double W = theFrom + (theTo - theFrom) * k / 12.0;
    try
    {
      gp_Pnt P;
      gp_Vec V1, V2;
      Sp->D2(W, P, V1, V2);
      Tag("d2");
      Hex(W);
      Pnt(P);
      Vec(V1);
      Vec(V2);
      Int(Sp->Index(W, true));
      Int(Sp->Index(W, false));
      Int(Sp->CurrentIndexOfElementarySpine());
      End();
      gp_Pnt Q;
      gp_Vec T;
      Sp->D1(W, Q, T);
      Tag("d1");
      Pnt(Q);
      Vec(T);
      Pnt(Sp->Value(W));
      End();
    }
    catch (Standard_Failure&)
    {
      Fail("d2");
    }
  }
}

static void RecSpine(occ::handle<ChFiDS_Spine>& Sp)
{
  Sp->Load();
  const int n = Sp->NbEdges();
  Tag("spine");
  Int(n);
  Hex(Sp->FirstParameter());
  Hex(Sp->LastParameter());
  Int(Sp->IsClosed() ? 1 : 0);
  Int(Sp->IsPeriodic() ? 1 : 0);
  Hex(Sp->GetTolesp());
  Pnt(BRep_Tool::Pnt(Sp->FirstVertex()));
  Pnt(BRep_Tool::Pnt(Sp->LastVertex()));
  End();
  for (int i = 1; i <= n; i++)
  {
    const TopoDS_Edge& E = Sp->Edges(i);
    Tag("edge");
    Int(i);
    Int((int)E.Orientation());
    Hex(Sp->FirstParameter(i));
    Hex(Sp->LastParameter(i));
    Hex(Sp->Length(i));
    Int(Sp->Index(E));
    TopoDS_Vertex V1, V2;
    TopExp::Vertices(E, V1, V2);
    Hex(Sp->Absc(V1));
    Hex(Sp->Absc(V2));
    End();
    const BRepAdaptor_Curve& aC = Sp->CurrentElementarySpine(i);
    const double             U  = 0.3 * aC.FirstParameter() + 0.7 * aC.LastParameter();
    const double             A  = Sp->Absc(U);
    double                   U1 = 0., U2 = 0.;
    Sp->Parameter(A, U1, true);
    Sp->Parameter(i, A, U2, false);
    Tag("absc");
    Hex(U);
    Hex(A);
    Hex(U1);
    Hex(U2);
    Int((int)Sp->GetType());
    End();
    if (Sp->GetType() == GeomAbs_Line)
    {
      const gp_Lin L = Sp->Line();
      Tag("line");
      Pnt(L.Location());
      Vec(gp_Vec(L.Direction()));
      End();
    }
    else if (Sp->GetType() == GeomAbs_Circle)
    {
      const gp_Circ C = Sp->Circle();
      Tag("circ");
      Pnt(C.Location());
      Vec(gp_Vec(C.Axis().Direction()));
      Vec(gp_Vec(C.XAxis().Direction()));
      Hex(C.Radius());
      End();
    }
  }
  RecSpineEval(Sp, Sp->FirstParameter(), Sp->LastParameter());
}

static void RecSpines()
{
  // A box: the edges of its first face, then the first two of them with tangent extensions and references.
  TopoDS_Shape aBox = BRepPrimAPI_MakeBox(10., 20., 30.).Shape();
  TopExp_Explorer anExpF(aBox, TopAbs_FACE);
  const TopoDS_Shape aFace = anExpF.Current();
  {
    std::fprintf(gOut, "case spine-box\n");
    occ::handle<ChFiDS_Spine> Sp = new ChFiDS_Spine(1.e-7);
    for (TopExp_Explorer anExp(aFace, TopAbs_EDGE); anExp.More(); anExp.Next())
    {
      Sp->SetEdges(TopoDS::Edge(anExp.Current()));
    }
    RecSpine(Sp);
    Sp->SetFirstStatus(ChFiDS_Closed);
    Sp->SetLastStatus(ChFiDS_Closed);
    Tag("periodic");
    Int(Sp->IsPeriodic() ? 1 : 0);
    Hex(Sp->Period());
    End();
    RecSpineEval(Sp, -15.0, 75.0);
  }
  {
    std::fprintf(gOut, "case spine-box-ext\n");
    occ::handle<ChFiDS_Spine> Sp = new ChFiDS_Spine();
    TopExp_Explorer           anExp(aFace, TopAbs_EDGE);
    TopoDS_Edge               E1 = TopoDS::Edge(anExp.Current());
    anExp.Next();
    TopoDS_Edge E2 = TopoDS::Edge(anExp.Current());
    Sp->SetEdges(E2);
    Sp->PutInFirst(E1);
    RecSpine(Sp);
    const double L = Sp->LastParameter();
    Sp->SetFirstParameter(-3.0);
    Sp->SetLastParameter(L + 4.0);
    Sp->SetFirstTgt(0.0);
    Sp->SetLastTgt(L);
    Tag("ext");
    Hex(Sp->FirstParameter());
    Hex(Sp->LastParameter());
    Int(Sp->HasFirstTgt() ? 1 : 0);
    Int(Sp->HasLastTgt() ? 1 : 0);
    End();
    RecSpineEval(Sp, -3.0, L + 4.0);
    Sp->SetReference(1);
    RecSpineEval(Sp, 9.0, 11.0);
    Sp->SetReference(15.0);
    RecSpineEval(Sp, 9.0, 11.0);
    Sp->UnsetReference();
    Sp->Reset(true);
    Tag("reset");
    Hex(Sp->FirstParameter());
    Hex(Sp->LastParameter());
    End();
  }
  // A cylinder: the circle of its top face, a closed chain.
  {
    std::fprintf(gOut, "case spine-cylinder\n");
    TopoDS_Shape    aCyl = BRepPrimAPI_MakeCylinder(5., 10.).Shape();
    TopExp_Explorer anExpF2(aCyl, TopAbs_FACE);
    anExpF2.Next();
    occ::handle<ChFiDS_Spine> Sp = new ChFiDS_Spine(1.e-7);
    for (TopExp_Explorer anExp(anExpF2.Current(), TopAbs_EDGE); anExp.More(); anExp.Next())
    {
      Sp->SetEdges(TopoDS::Edge(anExp.Current()));
    }
    RecSpine(Sp);
    Sp->SetFirstStatus(ChFiDS_Closed);
    Tag("periodic");
    Int(Sp->IsPeriodic() ? 1 : 0);
    Hex(Sp->Period());
    End();
    RecSpineEval(Sp, -10.0, 40.0);
  }
  // A B-spline solid: a prism on a face bounded by a periodic B-spline curve split into two edges.
  {
    std::fprintf(gOut, "case spine-bspline\n");
    occ::handle<Geom_BSplineCurve> aBS = GeomConvert::CurveToBSplineCurve(
      new Geom_Circle(gp_Ax2(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)), 4.0),
      Convert_QuasiAngular);
    aBS->SetPole(2, gp_Pnt(aBS->Pole(2).X() * 1.3, aBS->Pole(2).Y() * 0.8, 0.0));
    const double    f = aBS->FirstParameter(), l = aBS->LastParameter(), m = 0.4 * f + 0.6 * l;
    BRepBuilderAPI_MakeWire aW;
    aW.Add(BRepBuilderAPI_MakeEdge(aBS, f, m).Edge());
    aW.Add(BRepBuilderAPI_MakeEdge(aBS, m, l).Edge());
    TopoDS_Face  aF     = BRepBuilderAPI_MakeFace(aW.Wire(), true).Face();
    TopoDS_Shape aPrism = BRepPrimAPI_MakePrism(aF, gp_Vec(0, 0, 5)).Shape();
    TopExp_Explorer anExpF3(aPrism, TopAbs_FACE);
    occ::handle<ChFiDS_FilSpine> Sp = new ChFiDS_FilSpine(1.e-7);
    occ::handle<ChFiDS_Spine>    aSp = Sp;
    for (; anExpF3.More(); anExpF3.Next())
    {
      if (BRep_Tool::Surface(TopoDS::Face(anExpF3.Current()))->IsKind(STANDARD_TYPE(Geom_Plane)))
      {
        break;
      }
    }
    for (TopExp_Explorer anExp(anExpF3.Current(), TopAbs_EDGE); anExp.More(); anExp.Next())
    {
      Sp->SetEdges(TopoDS::Edge(anExp.Current()));
    }
    RecSpine(aSp);
    Sp->SetFirstStatus(ChFiDS_Closed);
    Tag("periodic");
    Int(Sp->IsPeriodic() ? 1 : 0);
    Hex(Sp->Period());
    End();
    RecSpineEval(aSp, -5.0, 35.0);

    // The radii and laws.
    Sp->SetFirstStatus(ChFiDS_OnSame);
    Sp->SetRadius(2.0);
    Tag("radius");
    Int(Sp->IsConstant() ? 1 : 0);
    Hex(Sp->Radius());
    End();
    Sp->SetRadius(3.0, Sp->Edges(2));
    Tag("radius2");
    Int(Sp->IsConstant() ? 1 : 0);
    Int(Sp->IsConstant(1) ? 1 : 0);
    Int(Sp->IsConstant(2) ? 1 : 0);
    Hex(Sp->Radius(2));
    Hex(Sp->MaxRadFromSeqAndLaws());
    End();
    occ::handle<ChFiDS_ElSpine> anEls = new ChFiDS_ElSpine();
    anEls->FirstParameter(Sp->FirstParameter());
    anEls->LastParameter(Sp->LastParameter());
    anEls->SetCurve(aBS);
    Sp->AppendElSpine(anEls);
    occ::handle<Law_Composite> aLaw = Sp->Law(anEls);
    Tag("law");
    for (int k = 0; k <= 10; k++)
    {
      const double W = Sp->FirstParameter() + (Sp->LastParameter() - Sp->FirstParameter()) * k / 10.0;
      Hex(aLaw->Value(W));
    }
    Hex(Sp->MaxRadFromSeqAndLaws());
    End();
  }
  // An elementary spine on a periodic B-spline curve.
  {
    std::fprintf(gOut, "case elspine\n");
    occ::handle<Geom_BSplineCurve> aBS = GeomConvert::CurveToBSplineCurve(
      new Geom_Circle(gp_Ax2(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)), 4.0),
      Convert_QuasiAngular);
    aBS->SetPeriodic();
    occ::handle<ChFiDS_ElSpine> anEls = new ChFiDS_ElSpine();
    anEls->SetCurve(aBS);
    anEls->FirstParameter(aBS->FirstParameter());
    anEls->LastParameter(aBS->LastParameter());
    anEls->SetPeriodic(true);
    anEls->SetFirstPointAndTgt(gp_Pnt(1, 2, 3), gp_Vec(0, 1, 0));
    anEls->AddVertexWithTangent(gp_Ax1(gp_Pnt(4, 0, 0), gp_Dir(0, 1, 0)));
    Tag("els");
    Hex(anEls->FirstParameter());
    Hex(anEls->LastParameter());
    Hex(anEls->Period());
    Int((int)anEls->Continuity());
    Int(anEls->NbIntervals(GeomAbs_C2));
    Int((int)anEls->GetType());
    Int(anEls->NbVertices());
    Hex(anEls->Resolution(1.e-3));
    End();
    occ::handle<Adaptor3d_Curve> aCopy = anEls->ShallowCopy();
    for (int k = 0; k <= 8; k++)
    {
      const double U = 6.0 * k / 8.0;
      gp_Pnt       Q;
      gp_Vec       W1;
      aCopy->D1(U, Q, W1);
      Tag("elscopy");
      Pnt(Q);
      Vec(W1);
      End();
    }
    anEls->SetOrigin(1.0);
    for (int k = 0; k <= 8; k++)
    {
      const double U = 6.0 * k / 8.0;
      gp_Pnt       P;
      gp_Vec       V1, V2, V3;
      anEls->D3(U, P, V1, V2, V3);
      Tag("elsd3");
      Pnt(P);
      Vec(V1);
      Vec(V2);
      Vec(V3);
      End();
    }

  }
}

int main(int argc, char** argv)
{
  OSD::SetSignal(false);
  if (argc > 1)
  {
    gOut = std::fopen(argv[1], "wb");
  }
  try
  {
    RecordFunctions();
    RecSpines();
  }
  catch (Standard_Failure& anExc)
  {
    std::fprintf(stderr, "%s: %s\n", typeid(anExc).name(), anExc.GetMessageString());
    return 1;
  }

  if (gOut != stdout)
  {
    std::fclose(gOut);
  }
  return 0;
}
