// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
//
// edgerec: records native OCCT's edge intersectors (IntTools_EdgeEdge, IntTools_EdgeFace and through it
// IntTools_BeanFaceIntersector) on data/occ shapes, every double as its 64 bits in hex (unit 6C, decision 6602), and
// measures how IntTools_Context's cached surface boxes change IntTools_EdgeFace's results (decision 6601).
// Built against build123d.NET's native OCCT V8_0_1 (tools/edgerec/build.ps1).
//
//   edgerec record <data/occ dir>   writes the recording to stdout
//   edgerec study <data/occ dir>    prints the context-history study
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <typeinfo>
#include <cmath>
#include <string>
#include <vector>

#include <BRep_Builder.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <GC_MakeArcOfCircle.hxx>
#include <GeomAPI_Interpolate.hxx>
#include <Geom_TrimmedCurve.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Iterator.hxx>
#include <TopoDS_Vertex.hxx>
#include <TColgp_HArray1OfPnt.hxx>
#include <sstream>
#include <BRep_Tool.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepBndLib.hxx>
#include <BRepTools.hxx>
#include <Bnd_Box.hxx>
#include <IntTools_CommonPrt.hxx>
#include <IntTools_Context.hxx>
#include <IntTools_EdgeEdge.hxx>
#include <IntTools_EdgeFace.hxx>
#include <IntTools_Range.hxx>
#include <NCollection_IndexedMap.hxx>
#include <Precision.hxx>
#include <TopExp.hxx>
#include <TopLoc_Location.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <TopTools_ShapeMapHasher.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>

typedef NCollection_IndexedMap<TopoDS_Shape, TopTools_ShapeMapHasher> ShapeMap;

static std::string Hex(double theValue)
{
  uint64_t aBits;
  std::memcpy(&aBits, &theValue, 8);
  char aBuf[20];
  std::snprintf(aBuf, sizeof(aBuf), "%016llx", (unsigned long long)aBits);
  return aBuf;
}

// The cases: a shape against a moved copy of itself, against itself, or against another shape. The shift is a
// fraction of the first shape's box, rounded to 1/64 so that it is exact in binary and printed exactly.
struct Case
{
  const char* Name;
  const char* FileA;
  const char* FileB; // nullptr: A itself
  double      Fx, Fy, Fz; // shift of B as fractions of A's box size; all 0 for no shift
  int         MaxEE;
  int         MaxEF;
};

static const Case THE_CASES[] = {
  {"bottle-shift", "bottle.brep", nullptr, 0.23, 0.17, 0.11, 400, 300},
  {"bottle-self", "bottle.brep", nullptr, 0, 0, 0, 250, 250},
  {"wing-shift", "wing.brep", nullptr, 0.07, 0.31, 0.05, 200, 200},
  {"wing-self", "wing.brep", nullptr, 0, 0, 0, 100, 150},
  {"ecrou-vis", "mal_ecrou.brep", "mal_vis.brep", 0, 0, 0, 300, 250},
  {"pumpnut-shift", "Pump_Nut.brep", nullptr, 0.13, 0.29, 0.19, 300, 250},
  {"ball-shift", "Ball.brep", nullptr, 0.21, 0.09, 0.27, 250, 200},
  {"crankarm-shift", "CrankArm.brep", nullptr, 0.11, 0.23, 0.07, 250, 200},
  {"hammer-shift", "hammer.brep", nullptr, 0.05, 0.13, 0.17, 250, 250},
  {"topcover-self", "Pump_TopCover.brep", nullptr, 0, 0, 0, 200, 200},
  {"tige-shift", "mal_tige.brep", nullptr, 0.19, 0.03, 0.31, 200, 200},
  {"shell1-shift", "shell1.brep", nullptr, 0.13, 0.07, 0.03, 300, 500},
  {"shell1-self", "shell1.brep", nullptr, 0, 0, 0, 250, 400},
  {"mat-shift", "MAT.brep", nullptr, 0.03, 0.05, 0.02, 100, 100},
  {"shell1-probes", "#probes", "shell1.brep", 0, 0, 0, 300, 600},
  {"bottle-probes", "#probes", "bottle.brep", 0, 0, 0, 200, 300},
  {"crankarm-probes", "#probes", "CrankArm.brep", 0, 0, 0, 200, 300},
  {"ball-probes", "#probes", "Ball.brep", 0, 0, 0, 150, 200},
  {"mat-probes", "#probes", "MAT.brep", 0, 0, 0, 50, 60},
};

static TopoDS_Shape Read(const std::string& theDir, const char* theFile)
{
  TopoDS_Shape aS;
  BRep_Builder aB;
  std::string  aPath = theDir + "/" + theFile;
  if (!BRepTools::Read(aS, aPath.c_str(), aB))
  {
    std::fprintf(stderr, "cannot read %s\n", aPath.c_str());
    std::exit(1);
  }
  return aS;
}

static Bnd_Box Box(const TopoDS_Shape& theS)
{
  Bnd_Box aB;
  BRepBndLib::Add(theS, aB);
  return aB;
}


// Probe edges for a shape: lines, arcs and interpolated B-splines between pseudo-random points of its box, with
// tolerances 1e-7, 1e-5 and 1e-3 in turn. Written into the recording as BRep text, so the port reads the same edges.
static TopoDS_Shape Probes(const Bnd_Box& theBox, int theNb)
{
  double x0, y0, z0, x1, y1, z1;
  theBox.Get(x0, y0, z0, x1, y1, z1);
  uint64_t aSeed = 0x9E3779B97F4A7C15ull;
  auto     aRand = [&aSeed]() {
    aSeed = aSeed * 6364136223846793005ull + 1442695040888963407ull;
    return (double)(aSeed >> 11) / 9007199254740992.0;
  };
  auto aPnt = [&]() { return gp_Pnt(x0 + (x1 - x0) * aRand(), y0 + (y1 - y0) * aRand(), z0 + (z1 - z0) * aRand()); };
  BRep_Builder    aB;
  TopoDS_Compound aC;
  aB.MakeCompound(aC);
  const double aTols[3] = {1.e-7, 1.e-5, 1.e-3};
  for (int k = 0; k < theNb; k++)
  {
    gp_Pnt      aP1 = aPnt(), aP2 = aPnt(), aP3 = aPnt();
    TopoDS_Edge aE;
    try
    {
      if (k % 3 == 0)
        aE = BRepBuilderAPI_MakeEdge(aP1, aP2);
      else if (k % 3 == 1)
      {
        GC_MakeArcOfCircle anArc(aP1, aP3, aP2);
        if (!anArc.IsDone())
          continue;
        aE = BRepBuilderAPI_MakeEdge(anArc.Value());
      }
      else
      {
        occ::handle<TColgp_HArray1OfPnt> aPts = new TColgp_HArray1OfPnt(1, 5);
        aPts->SetValue(1, aP1);
        aPts->SetValue(2, aP3);
        aPts->SetValue(3, aPnt());
        aPts->SetValue(4, aPnt());
        aPts->SetValue(5, aP2);
        GeomAPI_Interpolate anInt(aPts, false, 1.e-7);
        anInt.Perform();
        if (!anInt.IsDone())
          continue;
        aE = BRepBuilderAPI_MakeEdge(anInt.Curve());
      }
    }
    catch (const Standard_Failure&)
    {
      continue;
    }
    if (aE.IsNull())
      continue;
    double aTol = aTols[k % 3 == 0 ? (k / 3) % 3 : k % 3];
    aB.UpdateEdge(aE, aTol);
    for (TopoDS_Iterator it(aE); it.More(); it.Next())
      aB.UpdateVertex(TopoDS::Vertex(it.Value()), aTol);
    aB.Add(aC, aE);
  }
  return aC;
}

// Builds the case's shapes and pairs; the pairs are those whose boxes (with tolerances) meet, sampled evenly down to
// the case's maximum.
struct Prepared
{
  TopoDS_Shape                  A, B;
  gp_Vec                        Shift;
  ShapeMap                      EA, EB, FB;
  std::vector<std::pair<int, int>> EE, EF;
};

static void Sample(std::vector<std::pair<int, int>>& thePairs, int theMax)
{
  if ((int)thePairs.size() <= theMax)
    return;
  std::vector<std::pair<int, int>> aKept;
  double                           aStep = (double)thePairs.size() / theMax;
  for (int i = 0; i < theMax; i++)
    aKept.push_back(thePairs[(size_t)(i * aStep)]);
  thePairs = aKept;
}

static Prepared Prepare(const std::string& theDir, const Case& theCase, bool theToSample = true)
{
  Prepared aP;
  bool bProbes = std::strcmp(theCase.FileA, "#probes") == 0;
  aP.B = theCase.FileB ? Read(theDir, theCase.FileB) : TopoDS_Shape();
  aP.A = bProbes ? Probes(Box(aP.B), 60) : Read(theDir, theCase.FileA);
  if (bProbes)
  {
    // Run on the shape as the recording's text gives it, which the port reads.
    std::ostringstream aOut;
    BRepTools::Write(aP.A, aOut);
    std::istringstream anIn(aOut.str());
    BRep_Builder       aBld;
    BRepTools::Read(aP.A, anIn, aBld);
  }
  if (!theCase.FileB)
    aP.B = aP.A;
  if (theCase.Fx != 0 || theCase.Fy != 0 || theCase.Fz != 0)
  {
    double x0, y0, z0, x1, y1, z1;
    Box(aP.A).Get(x0, y0, z0, x1, y1, z1);
    aP.Shift = gp_Vec(std::round(theCase.Fx * (x1 - x0) * 64) / 64,
                      std::round(theCase.Fy * (y1 - y0) * 64) / 64,
                      std::round(theCase.Fz * (z1 - z0) * 64) / 64);
    gp_Trsf aT;
    aT.SetTranslation(aP.Shift);
    aP.B = aP.B.Moved(TopLoc_Location(aT));
  }
  TopExp::MapShapes(aP.A, TopAbs_EDGE, aP.EA);
  TopExp::MapShapes(aP.B, TopAbs_EDGE, aP.EB);
  TopExp::MapShapes(aP.B, TopAbs_FACE, aP.FB);
  bool bSelf = !theCase.FileB && aP.Shift.SquareMagnitude() == 0;

  std::vector<Bnd_Box> aBA, aBB, aBF;
  for (int i = 1; i <= aP.EA.Extent(); i++)
    aBA.push_back(Box(aP.EA(i)));
  for (int i = 1; i <= aP.EB.Extent(); i++)
    aBB.push_back(Box(aP.EB(i)));
  for (int i = 1; i <= aP.FB.Extent(); i++)
    aBF.push_back(Box(aP.FB(i)));
  for (int i = 1; i <= aP.EA.Extent(); i++)
  {
    const TopoDS_Edge& aE = TopoDS::Edge(aP.EA(i));
    if (BRep_Tool::Degenerated(aE))
      continue;
    for (int j = bSelf ? i + 1 : 1; j <= aP.EB.Extent(); j++)
    {
      if (BRep_Tool::Degenerated(TopoDS::Edge(aP.EB(j))))
        continue;
      if (!aBA[i - 1].IsOut(aBB[j - 1]))
        aP.EE.push_back({i, j});
    }
    for (int j = 1; j <= aP.FB.Extent(); j++)
      if (!aBA[i - 1].IsOut(aBF[j - 1]))
        aP.EF.push_back({i, j});
  }
  if (theToSample)
  {
    Sample(aP.EE, theCase.MaxEE);
    Sample(aP.EF, theCase.MaxEF);
  }
  return aP;
}

static std::string ExName(const Standard_Failure& theEx)
{
  std::string aName = typeid(theEx).name();
  return aName.rfind("class ", 0) == 0 ? aName.substr(6) : aName;
}

static std::string CommonPart(const IntTools_CommonPrt& theCP, const TopoDS_Edge& theE1, bool theWithPoints)
{
  std::string aS = " | " + std::to_string((int)theCP.Type()) + (theCP.Edge1().IsSame(theE1) ? " 1" : " 2");
  double      f, l;
  theCP.Range1(f, l);
  aS += " " + Hex(f) + " " + Hex(l);
  const NCollection_Sequence<IntTools_Range>& aR2 = theCP.Ranges2();
  aS += " " + std::to_string(aR2.Length());
  for (int k = 1; k <= aR2.Length(); k++)
    aS += " " + Hex(aR2(k).First()) + " " + Hex(aR2(k).Last());
  aS += " " + Hex(theCP.VertexParameter1()) + " " + Hex(theCP.VertexParameter2());
  aS += theCP.AllNullFlag() ? " 1" : " 0";
  if (theWithPoints)
  {
    gp_Pnt aP1, aP2;
    theCP.BoundingPoints(aP1, aP2);
    aS += " " + Hex(aP1.X()) + " " + Hex(aP1.Y()) + " " + Hex(aP1.Z()) + " " + Hex(aP2.X()) + " " + Hex(aP2.Y())
          + " " + Hex(aP2.Z());
  }
  return aS;
}

// The variants of an edge/edge run: 0 whole ranges; 1 sub-ranges; 2 whole ranges with the quick coincidence check;
// 3 whole ranges with a fuzzy value of 1e-4.
static std::string RunEE(const TopoDS_Edge& theE1, const TopoDS_Edge& theE2, int theVariant)
{
  IntTools_EdgeEdge anEE;
  anEE.SetEdge1(theE1);
  anEE.SetEdge2(theE2);
  if (theVariant == 1)
  {
    double f1, l1, f2, l2;
    BRep_Tool::Range(theE1, f1, l1);
    BRep_Tool::Range(theE2, f2, l2);
    anEE.SetRange1(f1 + 0.125 * (l1 - f1), l1 - 0.0625 * (l1 - f1));
    anEE.SetRange2(f2 + 0.0625 * (l2 - f2), l2 - 0.125 * (l2 - f2));
  }
  if (theVariant == 2)
    anEE.UseQuickCoincidenceCheck(true);
  if (theVariant == 3)
    anEE.SetFuzzyValue(1.e-4);
  std::string aS;
  try
  {
    anEE.Perform();
    aS = std::string(anEE.IsDone() ? "1" : "0") + " " + std::to_string(anEE.CommonParts().Length());
    for (int k = 1; k <= anEE.CommonParts().Length(); k++)
      aS += CommonPart(anEE.CommonParts()(k), theE1, false);
  }
  catch (const Standard_Failure& anEx)
  {
    aS = std::string("X ") + ExName(anEx);
  }
  return aS;
}

// An edge/face run on the edge's range, with the given context (null: a fresh one) and fuzzy value (0: default).
static std::string RunEF(const TopoDS_Edge& theE, const TopoDS_Face& theF, const occ::handle<IntTools_Context>& theCtx,
                         double theFuzz)
{
  IntTools_EdgeFace anEF;
  anEF.SetEdge(theE);
  anEF.SetFace(theF);
  double f, l;
  BRep_Tool::Range(theE, f, l);
  anEF.SetRange(f, l);
  anEF.SetContext(theCtx.IsNull() ? new IntTools_Context : theCtx);
  if (theFuzz > 0)
    anEF.SetFuzzyValue(theFuzz);
  std::string aS;
  try
  {
    anEF.Perform();
    aS = std::string(anEF.IsDone() ? "1" : "0") + " " + std::to_string(anEF.ErrorStatus()) + " "
         + Hex(anEF.MinimalDistance()) + " " + std::to_string(anEF.CommonParts().Length());
    for (int k = 1; k <= anEF.CommonParts().Length(); k++)
      aS += CommonPart(anEF.CommonParts()(k), theE, true);
  }
  catch (const Standard_Failure& anEx)
  {
    aS = std::string("X ") + ExName(anEx);
  }
  return aS;
}

// True if IntTools_BeanFaceIntersector localises on this face (its boxes are cached in the context).
static bool IsLocalized(const TopoDS_Face& theF)
{
  BRepAdaptor_Surface aS(theF);
  GeomAbs_SurfaceType aT = aS.GetType();
  if (Precision::IsInfinite(aS.FirstUParameter()) || Precision::IsInfinite(aS.LastUParameter())
      || Precision::IsInfinite(aS.FirstVParameter()) || Precision::IsInfinite(aS.LastVParameter()))
    return false;
  return aT == GeomAbs_BezierSurface || aT == GeomAbs_OtherSurface
         || (aT == GeomAbs_BSplineSurface && (aS.UDegree() > 2 || aS.VDegree() > 2) && aS.NbUKnots() > 2
             && aS.NbVKnots() > 2);
}

static void Record(const std::string& theDir)
{
  for (const Case& aCase : THE_CASES)
  {
    Prepared aP = Prepare(theDir, aCase);
    std::printf("CASE %s %s %s %s %s %s %d %d\n", aCase.Name, aCase.FileA, aCase.FileB ? aCase.FileB : "-",
                Hex(aP.Shift.X()).c_str(), Hex(aP.Shift.Y()).c_str(), Hex(aP.Shift.Z()).c_str(), (int)aP.EE.size(),
                (int)aP.EF.size());
    if (std::strcmp(aCase.FileA, "#probes") == 0)
    {
      std::ostringstream aText;
      BRepTools::Write(aP.A, aText);
      std::string aStr = aText.str();
      if (!aStr.empty() && aStr.back() != '\n')
        aStr += '\n';
      int aNbLines = 0;
      for (char c : aStr)
        aNbLines += c == '\n' ? 1 : 0;
      std::printf("BREP %d\n%s", aNbLines, aStr.c_str());
    }
    int n = 0;
    for (const auto& aPair : aP.EE)
    {
      const TopoDS_Edge& aE1 = TopoDS::Edge(aP.EA(aPair.first));
      const TopoDS_Edge& aE2 = TopoDS::Edge(aP.EB(aPair.second));
      int                aVariant = n % 11 == 3 ? 1 : n % 11 == 7 ? 2 : n % 13 == 5 ? 3 : 0;
      std::printf("EE %d %d %d %s\n", aPair.first, aPair.second, aVariant, RunEE(aE1, aE2, aVariant).c_str());
      n++;
    }
    // Edge/face: a fresh context per pair (F), then one context for the case's pairs in order (S); S is written only
    // where it differs from F.
    occ::handle<IntTools_Context> aCtx = new IntTools_Context;
    for (const auto& aPair : aP.EF)
    {
      const TopoDS_Edge& aE = TopoDS::Edge(aP.EA(aPair.first));
      const TopoDS_Face& aF = TopoDS::Face(aP.FB(aPair.second));
      std::string        aFresh = RunEF(aE, aF, nullptr, 0);
      std::string        aShared = RunEF(aE, aF, aCtx, 0);
      std::printf("EF %d %d %d %s\n", aPair.first, aPair.second, IsLocalized(aF) ? 1 : 0, aFresh.c_str());
      if (aShared != aFresh)
        std::printf("EFS %s\n", aShared.c_str());
    }
  }
}

// The study of decision 6601: on the faces where the intersector caches boxes, edge/face results with one context
// for all pairs (in order, in reverse, with alternating fuzzy values so that the criteria differ) against a fresh
// context per pair.
static void Study(const std::string& theDir)
{
  int aTot = 0, aDiffFwd = 0, aDiffRev = 0, aTotZ = 0, aDiffZ = 0, aDiffZSame = 0, aDiffZOther = 0;
  for (const Case& aCase : THE_CASES)
  {
    Prepared aP = Prepare(theDir, aCase, false);
    std::vector<std::pair<int, int>> aLoc;
    for (const auto& aPair : aP.EF)
      if (IsLocalized(TopoDS::Face(aP.FB(aPair.second))))
        aLoc.push_back(aPair);
    if (aLoc.empty())
      continue;
    std::vector<std::string> aFresh, aFreshZ;
    for (size_t k = 0; k < aLoc.size(); k++)
    {
      const TopoDS_Edge& aE = TopoDS::Edge(aP.EA(aLoc[k].first));
      const TopoDS_Face& aF = TopoDS::Face(aP.FB(aLoc[k].second));
      aFresh.push_back(RunEF(aE, aF, nullptr, 0));
      aFreshZ.push_back(RunEF(aE, aF, nullptr, k % 2 ? 1.e-4 : 0));
    }
    int                           aCaseDiff = 0;
    occ::handle<IntTools_Context> aCtx = new IntTools_Context;
    for (size_t k = 0; k < aLoc.size(); k++)
    {
      std::string aS = RunEF(TopoDS::Edge(aP.EA(aLoc[k].first)), TopoDS::Face(aP.FB(aLoc[k].second)), aCtx, 0);
      if (aS != aFresh[k])
      {
        aDiffFwd++;
        aCaseDiff++;
      }
    }
    aCtx = new IntTools_Context;
    for (size_t k = aLoc.size(); k-- > 0;)
    {
      std::string aS = RunEF(TopoDS::Edge(aP.EA(aLoc[k].first)), TopoDS::Face(aP.FB(aLoc[k].second)), aCtx, 0);
      if (aS != aFresh[k])
      {
        aDiffRev++;
        const TopoDS_Edge& aE = TopoDS::Edge(aP.EA(aLoc[k].first));
        std::printf("  reverse differs: %s EF %d %d (edge tolerance %g)\n    fresh  %s\n    shared %s\n", aCase.Name,
                    aLoc[k].first, aLoc[k].second, BRep_Tool::Tolerance(aE), aFresh[k].c_str(), aS.c_str());
      }
    }
    // Alternating fuzzy values: a pair's face may have been visited first with the other criteria.
    aCtx = new IntTools_Context;
    ShapeMap aSeenSame[2];
    for (size_t k = 0; k < aLoc.size(); k++)
    {
      const TopoDS_Face& aF = TopoDS::Face(aP.FB(aLoc[k].second));
      int                aParity = (int)(k % 2);
      bool               bOtherFirst = aSeenSame[1 - aParity].Contains(aF) && !aSeenSame[aParity].Contains(aF);
      std::string aS = RunEF(TopoDS::Edge(aP.EA(aLoc[k].first)), aF, aCtx, aParity ? 1.e-4 : 0);
      aSeenSame[aParity].Add(aF);
      aTotZ++;
      if (aS != aFreshZ[k])
      {
        aDiffZ++;
        if (bOtherFirst)
          aDiffZOther++;
        else
          aDiffZSame++;
      }
    }
    aTot += (int)aLoc.size();
    std::printf("%-16s localized pairs %4d, shared != fresh %d\n", aCase.Name, (int)aLoc.size(), aCaseDiff);
  }
  std::printf("TOTAL equal criteria: %d pairs; shared context != fresh: forward %d, reverse %d\n", aTot, aDiffFwd,
              aDiffRev);
  std::printf("TOTAL alternating criteria: %d pairs; shared != fresh %d (face first seen with the other criteria %d, "
              "otherwise %d)\n",
              aTotZ, aDiffZ, aDiffZOther, aDiffZSame);
}

int main(int argc, char** argv)
{
  if (argc < 3)
  {
    std::fprintf(stderr, "edgerec record|study <data/occ dir>\n");
    return 2;
  }
  std::string aMode = argv[1];
  if (aMode == "scan")
  {
    for (int i = 3; i < argc; i++)
    {
      TopoDS_Shape aS = Read(argv[2], argv[i]);
      ShapeMap aFM;
      TopExp::MapShapes(aS, TopAbs_FACE, aFM);
      int n = 0;
      for (int k = 1; k <= aFM.Extent(); k++)
        n += IsLocalized(TopoDS::Face(aFM(k))) ? 1 : 0;
      std::printf("%s %d %d\n", argv[i], aFM.Extent(), n);
    }
    return 0;
  }
  if (aMode == "record")
    Record(argv[2]);
  else
    Study(argv[2]);
  return 0;
}
