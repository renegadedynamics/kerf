// Records the steps of OCCT's walking (IntWalk_IWalking through IntPatch_ImpPrmIntersection, and IntWalk_PWalking):
// every return of TestDeflection (status, point) and the results, every double as its 64 bits in hex.
// Built against build123d.NET's native OCCT V8_0_1 with instrumented copies of IntWalk_IWalking.gxx,
// IntPatch_TheIWalking_0.cxx, IntPatch_ImpPrmIntersection.cxx and IntWalk_PWalking.cxx (unit 5C, decision 687).
#include <cstdio>
#include <cstring>
#include <cstdint>
#include <string>

#include <Adaptor3d_TopolTool.hxx>
#include <GeomAdaptor_Surface.hxx>
#include <Geom_BSplineSurface.hxx>
#include <Geom_CylindricalSurface.hxx>
#include <Geom_Plane.hxx>
#include <Geom_SphericalSurface.hxx>
#include <Geom_ConicalSurface.hxx>
#include <Geom_RectangularTrimmedSurface.hxx>
#include <Geom_ToroidalSurface.hxx>
#include <IntPatch_ImpPrmIntersection.hxx>
#include <IntPatch_Line.hxx>
#include <IntPatch_Point.hxx>
#include <IntPatch_RLine.hxx>
#include <IntPatch_WLine.hxx>
#include <IntSurf_LineOn2S.hxx>
#include <IntWalk_PWalking.hxx>
#include <NCollection_Array1.hxx>
#include <NCollection_Array2.hxx>
#include <gp_Ax3.hxx>

FILE* gRec = stdout;

static void Hex(double theValue)
{
  uint64_t aBits;
  std::memcpy(&aBits, &theValue, 8);
  std::fprintf(gRec, " %016llx", (unsigned long long)aBits);
}

void RecPnt(const char* tag, int status, const IntSurf_PntOn2S& p)
{
  std::fprintf(gRec, "%s %d", tag, status);
  Hex(p.Value().X());
  Hex(p.Value().Y());
  Hex(p.Value().Z());
  double u1, v1, u2, v2;
  p.Parameters(u1, v1, u2, v2);
  Hex(u1);
  Hex(v1);
  Hex(u2);
  Hex(v2);
  std::fprintf(gRec, "\n");
}

// A bicubic patch over [-2, 2] x [-2, 2] (x, y) with a wavy z.
static occ::handle<Geom_BSplineSurface> Patch(double theX0, double theX1, double theY0, double theY1, int theTable = 0)
{
  NCollection_Array2<gp_Pnt> aPoles(1, 4, 1, 4);
  const double aZ[4][4] = {{0.10, -0.20, 0.30, 0.00},
                           {-0.30, 0.40, -0.10, 0.20},
                           {0.20, -0.10, 0.50, -0.30},
                           {0.00, 0.30, -0.20, 0.10}};
  const double aZB[4][4] = {{0.40, 0.10, -0.20, 0.30},
                            {-0.10, -0.30, 0.20, 0.00},
                            {0.30, 0.20, -0.40, 0.10},
                            {-0.20, 0.00, 0.30, -0.10}};
  for (int i = 1; i <= 4; i++)
    for (int j = 1; j <= 4; j++)
      aPoles(i, j) = gp_Pnt(theX0 + (theX1 - theX0) * (i - 1) / 3.0,
                            theY0 + (theY1 - theY0) * (j - 1) / 3.0,
                            theTable == 0 ? aZ[i - 1][j - 1] : aZB[i - 1][j - 1]);
  NCollection_Array1<double> aKnots(1, 2);
  aKnots(1) = 0.0;
  aKnots(2) = 1.0;
  NCollection_Array1<int> aMults(1, 2);
  aMults(1) = 4;
  aMults(2) = 4;
  return new Geom_BSplineSurface(aPoles, aKnots, aKnots, aMults, aMults, 3, 3);
}

static void DumpResult(const IntPatch_ImpPrmIntersection& theInt)
{
  std::fprintf(gRec, "DONE %d\n", theInt.IsDone() ? 1 : 0);
  if (!theInt.IsDone())
    return;
  std::fprintf(gRec, "EMPTY %d NBPNTS %d NBLINES %d\n", theInt.IsEmpty() ? 1 : 0, theInt.NbPnts(), theInt.NbLines());
  for (int i = 1; i <= theInt.NbLines(); i++)
  {
    const occ::handle<IntPatch_Line>& aLine = theInt.Line(i);
    std::fprintf(gRec, "LINE %d TYPE %d TRANS %d %d\n", i, (int)aLine->ArcType(), (int)aLine->TransitionOnS1(),
                 (int)aLine->TransitionOnS2());
    occ::handle<IntPatch_WLine> aWL = occ::down_cast<IntPatch_WLine>(aLine);
    occ::handle<IntPatch_RLine> aRL = occ::down_cast<IntPatch_RLine>(aLine);
    if (!aWL.IsNull())
    {
      std::fprintf(gRec, "NBP %d NBV %d\n", aWL->NbPnts(), aWL->NbVertex());
      for (int k = 1; k <= aWL->NbPnts(); k++)
        RecPnt("P", 0, aWL->Point(k));
      for (int k = 1; k <= aWL->NbVertex(); k++)
      {
        const IntPatch_Point& aV = aWL->Vertex(k);
        std::fprintf(gRec, "V %d %d", aV.IsOnDomS1() ? 1 : 0, aV.IsOnDomS2() ? 1 : 0);
        Hex(aV.ParameterOnLine());
        RecPnt("", 0, aV.PntOn2S());
      }
    }
    else if (!aRL.IsNull())
    {
      std::fprintf(gRec, "RLINE NBP %d NBV %d\n", aRL->HasPolygon() ? aRL->NbPnts() : 0, aRL->NbVertex());
      if (aRL->HasPolygon())
        for (int k = 1; k <= aRL->NbPnts(); k++)
          RecPnt("P", 0, aRL->Point(k));
      for (int k = 1; k <= aRL->NbVertex(); k++)
      {
        const IntPatch_Point& aV = aRL->Vertex(k);
        std::fprintf(gRec, "V %d %d", aV.IsOnDomS1() ? 1 : 0, aV.IsOnDomS2() ? 1 : 0);
        Hex(aV.ParameterOnLine());
        RecPnt("", 0, aV.PntOn2S());
      }
    }
  }
}

static void ImpPrm(const occ::handle<Geom_Surface>& theS1, const occ::handle<Geom_Surface>& theS2, double theFleche,
                   double thePas)
{
  occ::handle<GeomAdaptor_Surface> aS1 = new GeomAdaptor_Surface(theS1);
  occ::handle<GeomAdaptor_Surface> aS2 = new GeomAdaptor_Surface(theS2);
  occ::handle<Adaptor3d_TopolTool> aD1 = new Adaptor3d_TopolTool(aS1);
  occ::handle<Adaptor3d_TopolTool> aD2 = new Adaptor3d_TopolTool(aS2);
  IntPatch_ImpPrmIntersection anInt(aS1, aD1, aS2, aD2, 1.0e-7, 1.0e-7, theFleche, thePas);
  DumpResult(anInt);
}

static void PWalk(const occ::handle<Geom_Surface>& theS1, const occ::handle<Geom_Surface>& theS2, double theU1,
                  double theV1, double theU2, double theV2)
{
  occ::handle<GeomAdaptor_Surface> aS1 = new GeomAdaptor_Surface(theS1);
  occ::handle<GeomAdaptor_Surface> aS2 = new GeomAdaptor_Surface(theS2);
  IntWalk_PWalking aPW(aS1, aS2, 1.0e-8, 1.0e-8, 0.01, 0.05);
  NCollection_Array1<double> aParDep(1, 4);
  aParDep(1) = theU1;
  aParDep(2) = theV1;
  aParDep(3) = theU2;
  aParDep(4) = theV2;
  aPW.Perform(aParDep);
  std::fprintf(gRec, "DONE %d\n", aPW.IsDone() ? 1 : 0);
  if (!aPW.IsDone())
    return;
  std::fprintf(gRec, "NBP %d CLOSED %d\n", aPW.NbPoints(), aPW.IsClosed() ? 1 : 0);
  for (int k = 1; k <= aPW.NbPoints(); k++)
    RecPnt("P", 0, aPW.Value(k));
}

int main(int argc, char** argv)
{
  const std::string aCase = argc > 1 ? argv[1] : "";
  occ::handle<Geom_CylindricalSurface> aCyl = new Geom_CylindricalSurface(gp_Ax3(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)), 1.0);
  if (aCase == "cyl-bspline-closed")
    ImpPrm(aCyl, Patch(-2.0, 2.0, -2.0, 2.0), 0.01, 0.05);
  else if (aCase == "cyl-bspline-open")
    ImpPrm(aCyl, Patch(0.0, 3.0, -2.0, 2.0), 0.01, 0.05);
  else if (aCase == "plane-torus")
  {
    occ::handle<Geom_Plane> aPln = new Geom_Plane(gp_Pnt(0, 0, 0.2), gp_Dir(0.1, 0.05, 1.0));
    occ::handle<Geom_ToroidalSurface> aTor =
      new Geom_ToroidalSurface(gp_Ax3(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)), 2.0, 0.5);
    ImpPrm(aPln, aTor, 0.01, 0.05);
  }
  else if (aCase == "pwalk-cyl-bspline")
  {
    occ::handle<Geom_RectangularTrimmedSurface> aTrim =
      new Geom_RectangularTrimmedSurface(aCyl, 0.0, 2.0 * M_PI, -2.0, 2.0);
    PWalk(aTrim, Patch(0.0, 3.0, -2.0, 2.0), 0.3, 0.1, 0.4, 0.55);
  }
  else if (aCase == "sphere-bspline")
    ImpPrm(new Geom_SphericalSurface(gp_Ax3(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)), 1.2),
           Patch(-2.0, 2.0, -2.0, 2.0), 0.01, 0.05);
  else if (aCase == "cone-bspline")
    ImpPrm(new Geom_ConicalSurface(gp_Ax3(gp_Pnt(0.1, -0.2, -1.0), gp_Dir(0, 0, 1)), M_PI / 6, 0.5),
           Patch(-2.0, 2.0, -2.0, 2.0), 0.01, 0.05);
  else if (aCase == "bspline-plane")
    ImpPrm(Patch(-2.0, 2.0, -2.0, 2.0, 1), new Geom_Plane(gp_Pnt(0, 0, 0.05), gp_Dir(0.2, -0.1, 1.0)), 0.01, 0.05);
  else if (aCase == "pwalk-torus-bspline")
    PWalk(new Geom_ToroidalSurface(gp_Ax3(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)), 2.0, 0.5),
          Patch(-2.0, 2.0, -2.0, 2.0), 0.0, M_PI, 0.875, 0.5);
  else if (aCase == "pwalk-bspline-bspline")
    PWalk(Patch(-2.0, 2.0, -2.0, 2.0), Patch(-2.0, 2.0, -2.0, 2.0, 1), 0.5, 0.5, 0.5, 0.5);
  else
  {
    std::fprintf(stderr, "unknown case\n");
    return 1;
  }
  return 0;
}
