// Records the steps of OCCT's walking (IntWalk_IWalking through IntPatch_ImpPrmIntersection, and IntWalk_PWalking):
// every return of TestDeflection (status, point) and the results, every double as its 64 bits in hex.
// Built against build123d.NET's native OCCT V8_0_1 with instrumented copies of IntWalk_IWalking.gxx,
// IntPatch_TheIWalking_0.cxx, IntPatch_ImpPrmIntersection.cxx and IntWalk_PWalking.cxx (unit 5C, decision 687), and
// OCCT's own IntPatch_PrmPrmIntersection.cxx (unit 5D), whose walkings are the instrumented IntWalk_PWalking.
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
#include <Geom_OffsetSurface.hxx>
#include <IntPatch_ImpPrmIntersection.hxx>
#include <IntPatch_InterferencePolyhedron.hxx>
#include <IntPatch_Polyhedron.hxx>
#include <IntPatch_PrmPrmIntersection.hxx>
#include <Intf_SectionLine.hxx>
#include <Intf_SectionPoint.hxx>
#include <Intf_TangentZone.hxx>
#include <NCollection_List.hxx>
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

// A bicubic B-spline of 30 x 30 poles over [-2, 2] x [-2, 2] with bumps from an integer recurrence (no
// transcendental, so that the port builds the same poles): enough samples for PrmPrm's voxel search.
static occ::handle<Geom_BSplineSurface> Bumpy(int theSeed, double theZ0)
{
  const int                  aN = 30;
  NCollection_Array2<gp_Pnt> aPoles(1, aN, 1, aN);
  for (int i = 1; i <= aN; i++)
    for (int j = 1; j <= aN; j++)
    {
      const int aR = (i * 7919 + j * 104729 + theSeed * 31) % 97;
      aPoles(i, j) = gp_Pnt(-2.0 + 4.0 * (i - 1) / (aN - 1), -2.0 + 4.0 * (j - 1) / (aN - 1),
                            theZ0 + 0.4 * aR / 97.0 - 0.2);
    }
  NCollection_Array1<double> aKnots(1, aN - 2);
  NCollection_Array1<int>    aMults(1, aN - 2);
  for (int k = 1; k <= aN - 2; k++)
  {
    aKnots(k) = (k - 1) / double(aN - 3);
    aMults(k) = (k == 1 || k == aN - 2) ? 4 : 1;
  }
  return new Geom_BSplineSurface(aPoles, aKnots, aKnots, aMults, aMults, 3, 3);
}

// A bicubic surface folded over itself along U (a looped profile swept along Y), for the self-intersection.
static occ::handle<Geom_BSplineSurface> Folded()
{
  const double               aX[6] = {-1.0, 2.0, 2.0, 0.0, 0.0, 1.5};
  const double               aZ[6] = {0.0, 0.0, 1.2, 1.2, -0.6, -0.6};
  NCollection_Array2<gp_Pnt> aPoles(1, 6, 1, 4);
  for (int i = 1; i <= 6; i++)
    for (int j = 1; j <= 4; j++)
      aPoles(i, j) = gp_Pnt(aX[i - 1], -1.0 + 2.0 * (j - 1) / 3.0, aZ[i - 1] + 0.05 * (j - 1));
  NCollection_Array1<double> aUKnots(1, 3), aVKnots(1, 2);
  NCollection_Array1<int>    aUMults(1, 3), aVMults(1, 2);
  aUKnots(1) = 0.0;
  aUKnots(2) = 0.5;
  aUKnots(3) = 1.0;
  aUMults(1) = 4;
  aUMults(2) = 2;
  aUMults(3) = 4;
  aVKnots(1) = 0.0;
  aVKnots(2) = 1.0;
  aVMults(1) = 4;
  aVMults(2) = 4;
  return new Geom_BSplineSurface(aPoles, aUKnots, aVKnots, aUMults, aVMults, 3, 3);
}

static void DumpPrmPrm(const IntPatch_PrmPrmIntersection& theInt)
{
  std::fprintf(gRec, "DONE %d\n", theInt.IsDone() ? 1 : 0);
  if (!theInt.IsDone())
    return;
  std::fprintf(gRec, "EMPTY %d NBLINES %d\n", theInt.IsEmpty() ? 1 : 0, theInt.NbLines());
  for (int i = 1; i <= theInt.NbLines(); i++)
  {
    const occ::handle<IntPatch_Line>& aLine = theInt.Line(i);
    std::fprintf(gRec, "LINE %d TYPE %d TRANS %d %d\n", i, (int)aLine->ArcType(), (int)aLine->TransitionOnS1(),
                 (int)aLine->TransitionOnS2());
    occ::handle<IntPatch_WLine> aWL = occ::down_cast<IntPatch_WLine>(aLine);
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
}

static void DumpSectionPoint(const char* theTag, const Intf_SectionPoint& theP)
{
  Intf_PIType aT1, aT2;
  int         a1, a2, a3, a4;
  double      p1, p2;
  theP.InfoFirst(aT1, a1, a2, p1);
  theP.InfoSecond(aT2, a3, a4, p2);
  std::fprintf(gRec, "%s %d %d %d %d %d %d", theTag, (int)aT1, a1, a2, (int)aT2, a3, a4);
  Hex(p1);
  Hex(p2);
  Hex(theP.Incidence());
  Hex(theP.Pnt().X());
  Hex(theP.Pnt().Y());
  Hex(theP.Pnt().Z());
  std::fprintf(gRec, "\n");
}

static void DumpPolyhedron(const IntPatch_Polyhedron& thePoly)
{
  int aNbU = 0, aNbV = 0;
  thePoly.Size(aNbU, aNbV);
  std::fprintf(gRec, "POLY %d %d %d", aNbU, aNbV, thePoly.NbTriangles());
  Hex(thePoly.DeflectionOverEstimation());
  std::fprintf(gRec, "\n");
}

static void DumpInterference(const IntPatch_InterferencePolyhedron& theIntf)
{
  std::fprintf(gRec, "INTF %d %d %d", theIntf.NbSectionPoints(), theIntf.NbSectionLines(), theIntf.NbTangentZones());
  Hex(theIntf.GetTolerance());
  std::fprintf(gRec, "\n");
  for (int i = 1; i <= theIntf.NbSectionLines(); i++)
  {
    const Intf_SectionLine& aL = theIntf.LineValue(i);
    std::fprintf(gRec, "SL %d %d\n", i, aL.NumberOfPoints());
    for (int k = 1; k <= aL.NumberOfPoints(); k++)
      DumpSectionPoint("SP", aL.GetPoint(k));
  }
  for (int i = 1; i <= theIntf.NbTangentZones(); i++)
  {
    const Intf_TangentZone& aZ = theIntf.ZoneValue(i);
    std::fprintf(gRec, "TZ %d %d\n", i, aZ.NumberOfPoints());
    for (int k = 1; k <= aZ.NumberOfPoints(); k++)
      DumpSectionPoint("TP", aZ.GetPoint(k));
  }
}

static const double THE_TOL = 1.0e-7, THE_FLECHE = 0.01, THE_PAS = 0.05;

static void PrmPrm(const occ::handle<Geom_Surface>& theS1, const occ::handle<Geom_Surface>& theS2)
{
  occ::handle<GeomAdaptor_Surface> aS1 = new GeomAdaptor_Surface(theS1);
  occ::handle<GeomAdaptor_Surface> aS2 = new GeomAdaptor_Surface(theS2);
  occ::handle<Adaptor3d_TopolTool> aD1 = new Adaptor3d_TopolTool(aS1);
  occ::handle<Adaptor3d_TopolTool> aD2 = new Adaptor3d_TopolTool(aS2);
  IntPatch_PrmPrmIntersection      anInt;
  anInt.Perform(aS1, aD1, aS2, aD2, THE_TOL, THE_TOL, THE_FLECHE, THE_PAS);
  DumpPrmPrm(anInt);
}

static void PrmPrmPoly(const occ::handle<Geom_Surface>& theS1, const occ::handle<Geom_Surface>& theS2)
{
  occ::handle<GeomAdaptor_Surface> aS1 = new GeomAdaptor_Surface(theS1);
  occ::handle<GeomAdaptor_Surface> aS2 = new GeomAdaptor_Surface(theS2);
  occ::handle<Adaptor3d_TopolTool> aD1 = new Adaptor3d_TopolTool(aS1);
  occ::handle<Adaptor3d_TopolTool> aD2 = new Adaptor3d_TopolTool(aS2);
  IntPatch_Polyhedron              aPoly1(aS1);
  IntPatch_Polyhedron              aPoly2(aS2);
  DumpPolyhedron(aPoly1);
  DumpPolyhedron(aPoly2);
  DumpInterference(IntPatch_InterferencePolyhedron(aPoly1, aPoly2));
  IntPatch_PrmPrmIntersection anInt;
  anInt.Perform(aS1, aPoly1, aD1, aS2, aD2, THE_TOL, THE_TOL, THE_FLECHE, THE_PAS);
  DumpPrmPrm(anInt);
}

static void PrmPrmSelf(const occ::handle<Geom_Surface>& theS)
{
  occ::handle<GeomAdaptor_Surface> aS = new GeomAdaptor_Surface(theS);
  occ::handle<Adaptor3d_TopolTool> aD = new Adaptor3d_TopolTool(aS);
  IntPatch_Polyhedron              aPoly(aS, aD->NbSamplesU(), aD->NbSamplesV());
  DumpPolyhedron(aPoly);
  DumpInterference(IntPatch_InterferencePolyhedron(aPoly));
  IntPatch_PrmPrmIntersection anInt;
  anInt.Perform(aS, aD, THE_TOL, THE_TOL, THE_FLECHE, THE_PAS);
  DumpPrmPrm(anInt);
}

static void PrmPrmVoxels(const occ::handle<Geom_Surface>& theS1, const occ::handle<Geom_Surface>& theS2)
{
  occ::handle<GeomAdaptor_Surface> aS1 = new GeomAdaptor_Surface(theS1);
  occ::handle<GeomAdaptor_Surface> aS2 = new GeomAdaptor_Surface(theS2);
  occ::handle<Adaptor3d_TopolTool> aD1 = new Adaptor3d_TopolTool(aS1);
  occ::handle<Adaptor3d_TopolTool> aD2 = new Adaptor3d_TopolTool(aS2);
  aD1->SamplePnts(THE_FLECHE, 10, 10);
  aD2->SamplePnts(THE_FLECHE, 10, 10);
  std::fprintf(gRec, "SAMPLES %d %d %d %d\n", aD1->NbSamplesU(), aD1->NbSamplesV(), aD2->NbSamplesU(),
               aD2->NbSamplesV());
  IntPatch_PrmPrmIntersection   anInt;
  occ::handle<IntSurf_LineOn2S> aLOn2S = new IntSurf_LineOn2S();
  anInt.PointDepart(aLOn2S, aS1, aD1->NbSamplesU(), aD1->NbSamplesV(), aS2, aD2->NbSamplesU(), aD2->NbSamplesV());
  std::fprintf(gRec, "DEPART %d\n", aLOn2S->NbPoints());
  for (int k = 1; k <= aLOn2S->NbPoints(); k++)
    RecPnt("D", 0, aLOn2S->Value(k));
  anInt.Perform(aS1, aD1, aS2, aD2, THE_TOL, THE_TOL, THE_FLECHE, THE_PAS);
  DumpPrmPrm(anInt);
}

static void PrmPrmStart(const occ::handle<Geom_Surface>& theS1, const occ::handle<Geom_Surface>& theS2, double theU1,
                        double theV1, double theU2, double theV2)
{
  occ::handle<GeomAdaptor_Surface> aS1 = new GeomAdaptor_Surface(theS1);
  occ::handle<GeomAdaptor_Surface> aS2 = new GeomAdaptor_Surface(theS2);
  occ::handle<Adaptor3d_TopolTool> aD1 = new Adaptor3d_TopolTool(aS1);
  occ::handle<Adaptor3d_TopolTool> aD2 = new Adaptor3d_TopolTool(aS2);
  IntPatch_PrmPrmIntersection      anInt;
  anInt.Perform(aS1, aD1, aS2, aD2, theU1, theV1, theU2, theV2, THE_TOL, THE_TOL, THE_FLECHE, THE_PAS);
  DumpPrmPrm(anInt);
}

static void PrmPrmList(const occ::handle<Geom_Surface>& theS1, const occ::handle<Geom_Surface>& theS2)
{
  occ::handle<GeomAdaptor_Surface>  aS1 = new GeomAdaptor_Surface(theS1);
  occ::handle<GeomAdaptor_Surface>  aS2 = new GeomAdaptor_Surface(theS2);
  occ::handle<Adaptor3d_TopolTool>  aD1 = new Adaptor3d_TopolTool(aS1);
  occ::handle<Adaptor3d_TopolTool>  aD2 = new Adaptor3d_TopolTool(aS2);
  NCollection_List<IntSurf_PntOn2S> aList;
  IntSurf_PntOn2S                   aP;
  aP.SetValue(0.0, 0.0, 1.0 / 3.0, 0.5);
  aList.Append(aP);
  aP.SetValue(0.3, 0.1, 0.4, 0.55);
  aList.Append(aP);
  IntPatch_PrmPrmIntersection anInt;
  anInt.Perform(aS1, aD1, aS2, aD2, THE_TOL, THE_TOL, THE_FLECHE, THE_PAS, aList);
  std::fprintf(gRec, "LIST %d\n", (int)aList.Size());
  for (NCollection_List<IntSurf_PntOn2S>::Iterator anIt(aList); anIt.More(); anIt.Next())
    RecPnt("L", 0, anIt.Value());
  DumpPrmPrm(anInt);
  // Then the sampled Perform keeping these lines, as IntPatch_Intersection does with a list of points.
  anInt.Perform(aS1, aD1, aS2, aD2, THE_TOL, THE_TOL, THE_FLECHE, THE_PAS, false);
  DumpPrmPrm(anInt);
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
  else if (aCase == "prmprm-bspline-bspline")
    PrmPrm(Patch(-2.0, 2.0, -2.0, 2.0), Patch(-2.0, 2.0, -2.0, 2.0, 1));
  else if (aCase == "prmprm-offset-bspline")
    PrmPrm(new Geom_OffsetSurface(Patch(-2.0, 2.0, -2.0, 2.0), 0.15), Patch(-2.0, 2.0, -2.0, 2.0, 1));
  else if (aCase == "prmprm-poly")
    PrmPrmPoly(Patch(-2.0, 2.0, -2.0, 2.0), Patch(-2.0, 2.0, -2.0, 2.0, 1));
  else if (aCase == "prmprm-start")
    PrmPrmStart(Patch(-2.0, 2.0, -2.0, 2.0), Patch(-2.0, 2.0, -2.0, 2.0, 1), 0.5, 0.5, 0.5, 0.5);
  else if (aCase == "prmprm-voxels")
    PrmPrmVoxels(Bumpy(1, 0.0), Bumpy(2, 0.12));
  else if (aCase == "prmprm-listpnts")
    PrmPrmList(new Geom_RectangularTrimmedSurface(aCyl, 0.0, 2.0 * M_PI, -2.0, 2.0), Patch(0.0, 3.0, -2.0, 2.0));
  else if (aCase == "prmprm-self")
    PrmPrmSelf(Folded());
  else
  {
    std::fprintf(stderr, "unknown case\n");
    return 1;
  }
  return 0;
}
