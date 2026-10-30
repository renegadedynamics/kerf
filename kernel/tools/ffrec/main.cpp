// Records native IntTools_FaceFace results (unit 6D): for each face pair, the outcome, every line's curves' presence
// and tolerances, the token count and FNV-1a hash of the BRep text records of its 3D curve and its 2D curves (an
// edge with the 2D curves on two fixed planes), nine points of each curve, and every intersection point, every double
// as its 64 bits in hex. Built against build123d.NET's native OCCT V8_0_1 (TKBO's own IntTools_FaceFace, not a
// copy). The faces come from primitive solids and free-form sheets at pseudo-random placements and from special
// placements (written into the recording as BRep text), and from OCCT's data/occ shapes (named by file and face index).
//   ffrec primitives > a.txt;  ffrec <data/occ directory> > b.txt
#include <cstdio>
#include <cstring>
#include <cctype>
#include <cstdint>
#include <sstream>
#include <string>
#include <vector>

#include <BRepBndLib.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCone.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRepPrimAPI_MakeRevol.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <BRepPrimAPI_MakeTorus.hxx>
#include <BRepTools.hxx>
#include <BRep_Builder.hxx>
#include <BRep_Tool.hxx>
#include <Bnd_Box.hxx>
#include <Geom2d_Curve.hxx>
#include <Precision.hxx>
#include <gp_Pnt2d.hxx>
#include <Geom_BSplineCurve.hxx>
#include <Geom_BSplineSurface.hxx>
#include <Geom_BezierSurface.hxx>
#include <Geom_Curve.hxx>
#include <Geom_OffsetSurface.hxx>
#include <Geom_Plane.hxx>
#include <GeomConvert.hxx>
#include <Geom_ConicalSurface.hxx>
#include <Geom_CylindricalSurface.hxx>
#include <Geom_SphericalSurface.hxx>
#include <gp_Ax3.hxx>
#include <IntTools_Curve.hxx>
#include <IntTools_FaceFace.hxx>
#include <IntTools_PntOn2Faces.hxx>
#include <IntTools_PntOnFace.hxx>
#include <NCollection_Array1.hxx>
#include <NCollection_Array2.hxx>
#include <NCollection_IndexedDataMap.hxx>
#include <NCollection_IndexedMap.hxx>
#include <Standard_Failure.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopLoc_Location.hxx>
#include <TopTools_FormatVersion.hxx>
#include <TopTools_ShapeMapHasher.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <gp_Ax1.hxx>
#include <gp_Ax2.hxx>
#include <gp_Pln.hxx>
#include <gp_Trsf.hxx>

static void Hex(double theValue)
{
  uint64_t aBits;
  std::memcpy(&aBits, &theValue, 8);
  std::printf(" %016llx", (unsigned long long)aBits);
}

static std::string Text(const TopoDS_Shape& theShape)
{
  std::ostringstream aStream;
  BRepTools::Write(theShape, aStream, true, true, TopTools_FormatVersion_VERSION_3);
  return aStream.str();
}

static void Block(const char* theTag, const std::string& theText)
{
  std::printf("%s %d\n", theTag, (int)theText.size());
  std::fwrite(theText.data(), 1, theText.size(), stdout);
  std::printf("\n");
}

// A deterministic generator (the port reads the faces from the recording, so only the recorder uses it).
static uint64_t gSeed = 88172645463325252ull;

static double Rand()
{
  gSeed ^= gSeed << 13;
  gSeed ^= gSeed >> 7;
  gSeed ^= gSeed << 17;
  return (double)(gSeed % 1000003ull) / 1000003.0;
}

static double Range(double theA, double theB) { return theA + (theB - theA) * Rand(); }

static gp_Trsf RandomPlacement(double theShift)
{
  gp_Trsf aRot;
  gp_Dir  anAxis(Range(-1, 1), Range(-1, 1), Range(-1, 1) + 0.01);
  aRot.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), anAxis), Range(0.0, 6.0));
  gp_Trsf aMove;
  aMove.SetTranslation(gp_Vec(Range(-theShift, theShift), Range(-theShift, theShift), Range(-theShift, theShift)));
  return aMove * aRot;
}

// The tokens of a section of a BRep text ("Curves", "Curve2ds"): the lines after its header, up to the next section.
static std::vector<std::string> SectionTokens(const std::string& theText, const char* theName)
{
  std::vector<std::string> aTokens;
  std::istringstream       aStream(theText);
  std::string              aLine;
  const std::string        aHeader = std::string(theName) + " ";
  bool                     anIn    = false;
  while (std::getline(aStream, aLine))
  {
    if (!aLine.empty() && aLine.back() == '\r')
      aLine.pop_back();
    if (!anIn)
    {
      anIn = aLine.compare(0, aHeader.size(), aHeader) == 0;
      continue;
    }
    if (!aLine.empty() && std::isalpha((unsigned char)aLine[0]))
      break;
    std::istringstream aWords(aLine);
    std::string        aWord;
    while (aWords >> aWord)
      aTokens.push_back(aWord);
  }
  return aTokens;
}

// A record's token count and the 64-bit FNV-1a hash of its tokens joined by single spaces.
static void Record(const char* theTag, const std::vector<std::string>& theTokens)
{
  uint64_t aHash = 14695981039346656037ull;
  for (size_t i = 0; i < theTokens.size(); i++)
  {
    const std::string aWord = (i ? " " : "") + theTokens[i];
    for (unsigned char aC : aWord)
    {
      aHash ^= aC;
      aHash *= 1099511628211ull;
    }
  }
  std::printf("%s %d %016llx\n", theTag, (int)theTokens.size(), (unsigned long long)aHash);
}

// Nine points of a curve at equal steps of its range (an infinite range as [-10, 10]).
static void Range9(double theF, double theL, double* theT)
{
  const double aF = Precision::IsInfinite(theF) ? -10.0 : theF;
  const double aL = Precision::IsInfinite(theL) ? 10.0 : theL;
  for (int k = 0; k <= 8; k++)
    theT[k] = aF + (aL - aF) * k / 8.0;
}

static void Samples3d(const char* theTag, const occ::handle<Geom_Curve>& theC)
{
  double aT[9];
  Range9(theC->FirstParameter(), theC->LastParameter(), aT);
  std::printf("%s", theTag);
  for (int k = 0; k < 9; k++)
  {
    const gp_Pnt aP = theC->Value(aT[k]);
    Hex(aT[k]);
    Hex(aP.X());
    Hex(aP.Y());
    Hex(aP.Z());
  }
  std::printf("\n");
}

static void Samples2d(const char* theTag, const occ::handle<Geom2d_Curve>& theC)
{
  double aT[9];
  Range9(theC->FirstParameter(), theC->LastParameter(), aT);
  std::printf("%s", theTag);
  for (int k = 0; k < 9; k++)
  {
    const gp_Pnt2d aP = theC->Value(aT[k]);
    Hex(aT[k]);
    Hex(aP.X());
    Hex(aP.Y());
  }
  std::printf("\n");
}

// The variants of the parameters, as the Boolean operations and other callers set them.
struct Variant
{
  bool   Approx, Approx1, Approx2;
  double TolApprox, Fuzz;
  bool   Reverse;
  int    Prepare; // 0: none, 1: PrepareLines3D(false), 2: PrepareLines3D(true)
};

static const Variant gVariants[4] = {{true, true, true, 1.0e-7, 1.0e-7, false, 1},
                                     {false, true, true, 1.0e-7, 1.0e-7, false, 2},
                                     {true, false, false, 1.0e-7, 1.0e-5, false, 0},
                                     {true, true, false, 1.0e-6, 1.0e-7, true, 1}};

static int gCase = 0;

static void Run(const TopoDS_Face& theF1, const TopoDS_Face& theF2, int theVariant)
{
  const Variant& aV = gVariants[theVariant];
  std::printf("variant %d %d %d %d", theVariant, aV.Approx ? 1 : 0, aV.Approx1 ? 1 : 0, aV.Approx2 ? 1 : 0);
  Hex(aV.TolApprox);
  Hex(aV.Fuzz);
  std::printf(" %d %d\n", aV.Reverse ? 1 : 0, aV.Prepare);
  IntTools_FaceFace aFF;
  try
  {
    aFF.SetParameters(aV.Approx, aV.Approx1, aV.Approx2, aV.TolApprox);
    aFF.SetFuzzyValue(aV.Fuzz);
    if (aV.Reverse)
      aFF.Perform(theF2, theF1);
    else
      aFF.Perform(theF1, theF2);
    if (aFF.IsDone() && aV.Prepare != 0)
      aFF.PrepareLines3D(aV.Prepare == 2);
  }
  catch (Standard_Failure&)
  {
    std::printf("out throws\nend\n");
    return;
  }
  if (!aFF.IsDone())
  {
    std::printf("out notdone %d\nend\n", aFF.TangentFaces() ? 1 : 0);
    return;
  }
  const NCollection_Sequence<IntTools_Curve>&       aLines  = aFF.Lines();
  const NCollection_Sequence<IntTools_PntOn2Faces>& aPoints = aFF.Points();
  std::printf("out done %d %d %d\n", aFF.TangentFaces() ? 1 : 0, aLines.Length(), aPoints.Length());
  occ::handle<Geom_Plane> aP1 = new Geom_Plane(gp_Pln());
  occ::handle<Geom_Plane> aP2 = new Geom_Plane(gp_Pln(gp_Pnt(0, 0, 1), gp_Dir(0, 0, 1)));
  BRep_Builder            aBB;
  for (int i = 1; i <= aLines.Length(); i++)
  {
    const IntTools_Curve& aIC = aLines(i);
    std::printf("line %d %d %d %d", i, aIC.Curve().IsNull() ? 0 : 1, aIC.FirstCurve2d().IsNull() ? 0 : 1,
                aIC.SecondCurve2d().IsNull() ? 0 : 1);
    Hex(aIC.Tolerance());
    Hex(aIC.TangentialTolerance());
    std::printf("\n");
    TopoDS_Edge anE;
    aBB.MakeEdge(anE);
    if (!aIC.Curve().IsNull())
      aBB.UpdateEdge(anE, aIC.Curve(), 1.0e-7);
    if (!aIC.FirstCurve2d().IsNull())
      aBB.UpdateEdge(anE, aIC.FirstCurve2d(), aP1, TopLoc_Location(), 1.0e-7);
    if (!aIC.SecondCurve2d().IsNull())
      aBB.UpdateEdge(anE, aIC.SecondCurve2d(), aP2, TopLoc_Location(), 1.0e-7);
    const std::string aText = Text(anE);
    Record("c3", SectionTokens(aText, "Curves"));
    Record("c2", SectionTokens(aText, "Curve2ds"));
    if (!aIC.Curve().IsNull())
      Samples3d("s3", aIC.Curve());
    if (!aIC.FirstCurve2d().IsNull())
      Samples2d("s1", aIC.FirstCurve2d());
    if (!aIC.SecondCurve2d().IsNull())
      Samples2d("s2", aIC.SecondCurve2d());
  }
  for (int i = 1; i <= aPoints.Length(); i++)
  {
    const IntTools_PntOn2Faces& aP = aPoints(i);
    double                      u1, v1, u2, v2;
    aP.P1().Parameters(u1, v1);
    aP.P2().Parameters(u2, v2);
    std::printf("pnt %d", i);
    Hex(aP.P1().Pnt().X());
    Hex(aP.P1().Pnt().Y());
    Hex(aP.P1().Pnt().Z());
    Hex(u1);
    Hex(v1);
    Hex(aP.P2().Pnt().X());
    Hex(aP.P2().Pnt().Y());
    Hex(aP.P2().Pnt().Z());
    Hex(u2);
    Hex(v2);
    std::printf("\n");
  }
  std::printf("end\n");
}

static bool Overlap(const TopoDS_Shape& theA, const TopoDS_Shape& theB)
{
  Bnd_Box aBA, aBB;
  BRepBndLib::Add(theA, aBA);
  BRepBndLib::Add(theB, aBB);
  aBA.Enlarge(1.0e-6);
  aBB.Enlarge(1.0e-6);
  return !aBA.IsOut(aBB);
}

// A pair of faces written into the recording.
static void InlinePair(const char* theKind, const TopoDS_Face& theF1, const TopoDS_Face& theF2, int theVariant)
{
  BRep_Builder    aBB;
  TopoDS_Compound aC;
  aBB.MakeCompound(aC);
  aBB.Add(aC, theF1);
  aBB.Add(aC, theF2);
  std::printf("case %d %s\n", ++gCase, theKind);
  Block("faces", Text(aC));
  // the faces as read back, so that both sides intersect the bits of the text
  TopoDS_Shape       aRead;
  std::istringstream aStream(Text(aC));
  BRepTools::Read(aRead, aStream, aBB);
  NCollection_IndexedMap<TopoDS_Shape, TopTools_ShapeMapHasher> aFaces;
  TopExp::MapShapes(aRead, TopAbs_FACE, aFaces);
  Run(TopoDS::Face(aFaces(1)), TopoDS::Face(aFaces(aFaces.Extent())), theVariant);
}

static TopoDS_Shape Solid(int theKind)
{
  switch (theKind)
  {
    case 0:
      return BRepPrimAPI_MakeBox(gp_Pnt(-1, -0.8, -0.6), Range(1.5, 2.5), Range(1.2, 2.0), Range(1.0, 2.0)).Shape();
    case 1:
      return BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(0, 0, -1), gp_Dir(0, 0, 1)), Range(0.6, 1.2), Range(1.5, 2.5))
        .Shape();
    case 2:
      return BRepPrimAPI_MakeCone(gp_Ax2(gp_Pnt(0, 0, -1), gp_Dir(0, 0, 1)), Range(0.8, 1.3), Range(0.1, 0.5),
                                  Range(1.5, 2.5))
        .Shape();
    case 3:
      return BRepPrimAPI_MakeSphere(Range(0.8, 1.4)).Shape();
    default:
      return BRepPrimAPI_MakeTorus(Range(1.0, 1.4), Range(0.3, 0.5)).Shape();
  }
}

static const char* gSolidNames[5] = {"box", "cylinder", "cone", "sphere", "torus"};

static occ::handle<Geom_BSplineSurface> Patch(double theZ)
{
  NCollection_Array2<gp_Pnt> aPoles(1, 5, 1, 5);
  for (int i = 1; i <= 5; i++)
    for (int j = 1; j <= 5; j++)
      aPoles(i, j) = gp_Pnt(-1.5 + 0.75 * (i - 1), -1.5 + 0.75 * (j - 1), theZ + Range(-0.4, 0.4));
  NCollection_Array1<double> aKnots(1, 3);
  aKnots(1) = 0.0;
  aKnots(2) = 0.5;
  aKnots(3) = 1.0;
  NCollection_Array1<int> aMults(1, 3);
  aMults(1) = 4;
  aMults(2) = 1;
  aMults(3) = 4;
  return new Geom_BSplineSurface(aPoles, aKnots, aKnots, aMults, aMults, 3, 3);
}

static occ::handle<Geom_BSplineCurve> Profile()
{
  NCollection_Array1<gp_Pnt> aPoles(1, 5);
  for (int i = 1; i <= 5; i++)
    aPoles(i) = gp_Pnt(Range(0.5, 1.2), 0.0, -1.0 + 0.5 * (i - 1));
  NCollection_Array1<double> aKnots(1, 2);
  aKnots(1) = 0.0;
  aKnots(2) = 1.0;
  NCollection_Array1<int> aMults(1, 2);
  aMults(1) = 5;
  aMults(2) = 5;
  return new Geom_BSplineCurve(aPoles, aKnots, aMults, 4);
}

// A free-form face: 0 B-spline, 1 Bezier, 2 revolution, 3 extrusion, 4 offset.
static TopoDS_Face Sheet(int theKind)
{
  switch (theKind)
  {
    case 0:
      return BRepBuilderAPI_MakeFace(Patch(0.0), 1.0e-7).Face();
    case 1: {
      NCollection_Array2<gp_Pnt> aPoles(1, 4, 1, 4);
      for (int i = 1; i <= 4; i++)
        for (int j = 1; j <= 4; j++)
          aPoles(i, j) = gp_Pnt(-1.5 + 1.0 * (i - 1), -1.5 + 1.0 * (j - 1), Range(-0.5, 0.5));
      return BRepBuilderAPI_MakeFace(new Geom_BezierSurface(aPoles), 1.0e-7).Face();
    }
    case 2: {
      TopoDS_Edge  anE = BRepBuilderAPI_MakeEdge(Profile()).Edge();
      TopoDS_Shape aR  = BRepPrimAPI_MakeRevol(anE, gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)), Range(2.0, 6.28)).Shape();
      return TopoDS::Face(TopExp_Explorer(aR, TopAbs_FACE).Current());
    }
    case 3: {
      TopoDS_Edge  anE = BRepBuilderAPI_MakeEdge(Profile()).Edge();
      TopoDS_Shape aP  = BRepPrimAPI_MakePrism(anE, gp_Vec(Range(-0.3, 0.3), 2.0, Range(-0.3, 0.3))).Shape();
      gp_Trsf      aT;
      aT.SetTranslation(gp_Vec(0, -1, 0));
      return TopoDS::Face(TopExp_Explorer(BRepBuilderAPI_Transform(aP, aT, true).Shape(), TopAbs_FACE).Current());
    }
    default:
      return BRepBuilderAPI_MakeFace(new Geom_OffsetSurface(Patch(0.0), Range(-0.3, 0.3)), 1.0e-7).Face();
  }
}

static const char* gSheetNames[5] = {"bspline", "bezier", "revolution", "extrusion", "offset"};

static TopoDS_Shape Placed(const TopoDS_Shape& theShape, double theShift)
{
  return BRepBuilderAPI_Transform(theShape, RandomPlacement(theShift), true).Shape();
}

static void Primitives()
{
  int aVariant = 0;
  // solids against solids: every pair of kinds, twice, every pair of faces whose boxes overlap
  for (int aRep = 0; aRep < 2; aRep++)
    for (int k1 = 0; k1 < 5; k1++)
      for (int k2 = k1; k2 < 5; k2++)
      {
        TopoDS_Shape aS1 = Placed(Solid(k1), 0.3);
        TopoDS_Shape aS2 = Placed(Solid(k2), 0.6);
        NCollection_IndexedMap<TopoDS_Shape, TopTools_ShapeMapHasher> aF1, aF2;
        TopExp::MapShapes(aS1, TopAbs_FACE, aF1);
        TopExp::MapShapes(aS2, TopAbs_FACE, aF2);
        std::string aKind = std::string(gSolidNames[k1]) + "-" + gSolidNames[k2];
        for (int i = 1; i <= aF1.Extent(); i++)
          for (int j = 1; j <= aF2.Extent(); j++)
            if (Overlap(aF1(i), aF2(j)))
              InlinePair(aKind.c_str(), TopoDS::Face(aF1(i)), TopoDS::Face(aF2(j)), aVariant++ % 4);
      }
  // free-form sheets against sheets and against the solids' faces
  for (int aRep = 0; aRep < 2; aRep++)
    for (int k1 = 0; k1 < 5; k1++)
    {
      for (int k2 = k1; k2 < 5; k2++)
      {
        TopoDS_Shape aF1 = Placed(Sheet(k1), 0.2);
        TopoDS_Shape aF2 = Placed(Sheet(k2), 0.2);
        std::string  aKind = std::string(gSheetNames[k1]) + "-" + gSheetNames[k2];
        InlinePair(aKind.c_str(), TopoDS::Face(aF1), TopoDS::Face(aF2), aVariant++ % 4);
      }
      for (int k2 = 0; k2 < 5; k2++)
      {
        TopoDS_Shape aF1 = Placed(Sheet(k1), 0.2);
        TopoDS_Shape aS2 = Placed(Solid(k2), 0.3);
        NCollection_IndexedMap<TopoDS_Shape, TopTools_ShapeMapHasher> aF2;
        TopExp::MapShapes(aS2, TopAbs_FACE, aF2);
        std::string aKind = std::string(gSheetNames[k1]) + "-" + gSolidNames[k2];
        for (int j = 1; j <= aF2.Extent(); j++)
          if (Overlap(aF1, aF2(j)))
            InlinePair(aKind.c_str(), TopoDS::Face(aF1), TopoDS::Face(aF2(j)), aVariant++ % 4);
      }
    }
}

static TopoDS_Face FirstFace(const TopoDS_Shape& theShape, int theIndex = 1)
{
  NCollection_IndexedMap<TopoDS_Shape, TopTools_ShapeMapHasher> aFaces;
  TopExp::MapShapes(theShape, TopAbs_FACE, aFaces);
  return TopoDS::Face(aFaces(theIndex));
}

static TopoDS_Face NaturalFace(const occ::handle<Geom_Surface>& theS)
{
  BRep_Builder aBB;
  TopoDS_Face  aF;
  aBB.MakeFace(aF, theS, Precision::Confusion());
  return aF;
}

// Special placements: tangencies (points, tangent faces), unbounded faces, the analytic shortcuts' conditions, the
// same face twice, degenerated zones of B-splines.
static void Specials()
{
  int          aVariant = 0;
  TopoDS_Face  aPlaneZ0 = BRepBuilderAPI_MakeFace(gp_Pln(), -3, 3, -3, 3).Face();
  // a sphere touching a plane, two spheres touching
  InlinePair("special-sphere-plane-point", FirstFace(BRepPrimAPI_MakeSphere(gp_Pnt(0.3, 0.2, 1.0), 1.0).Shape()),
             aPlaneZ0, aVariant++ % 4);
  InlinePair("special-sphere-sphere-point", FirstFace(BRepPrimAPI_MakeSphere(gp_Pnt(0, 0, 0), 1.0).Shape()),
             FirstFace(BRepPrimAPI_MakeSphere(gp_Pnt(1.2, 1.6, 0), 1.0).Shape()), aVariant++ % 4);
  // coplanar planes, coaxial cylinders of the same radius
  InlinePair("special-coplanar", aPlaneZ0, BRepBuilderAPI_MakeFace(gp_Pln(), -1, 4, -2, 2).Face(), aVariant++ % 4);
  InlinePair("special-coaxial-cylinders",
             FirstFace(BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)), 1.0, 2.0).Shape()),
             FirstFace(BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(0, 0, 1), gp_Dir(0, 0, 1)), 1.0, 2.0).Shape()),
             aVariant++ % 4);
  // unbounded faces
  for (int k = 0; k < 4; k++)
  {
    occ::handle<Geom_Surface> aPln =
      new Geom_Plane(gp_Ax3(gp_Pnt(0.1 * k, -0.2, 0.3), gp_Dir(0.2 * k, 1.0 - 0.1 * k, 0.3)));
    occ::handle<Geom_Surface> aPln2 = new Geom_Plane(gp_Ax3(gp_Pnt(0, 0, 0.1 * k), gp_Dir(1.0, 0.1 * k, 0.5)));
    occ::handle<Geom_Surface> aCyl =
      new Geom_CylindricalSurface(gp_Ax3(gp_Pnt(0, 0, 0), gp_Dir(1, 0.1 * k, 0.05)), 1.0 + 0.2 * k);
    occ::handle<Geom_Surface> aCone =
      new Geom_ConicalSurface(gp_Ax3(gp_Pnt(0, 0.2, 0), gp_Dir(0.1, 0.2 * k, 1)), 0.4 + 0.05 * k, 0.5);
    InlinePair("special-infinite-plane-plane", NaturalFace(aPln), NaturalFace(aPln2), k);
    InlinePair("special-infinite-plane-cylinder", NaturalFace(aPln), NaturalFace(aCyl), k);
    InlinePair("special-infinite-plane-cone", NaturalFace(aPln2), NaturalFace(aCone), k);
    InlinePair("special-infinite-cylinder-cone", NaturalFace(aCyl), NaturalFace(aCone), k);
  }
  // OCC24005's plane and cylinder
  InlinePair("special-occ24005",
             NaturalFace(new Geom_Plane(gp_Ax3(gp_Pnt(-72.948737453424499, 754.30437716359393, 259.52151854671678),
                                               gp_Dir(6.2471473085930200e-007, -0.99999999999980493, 0.0),
                                               gp_Dir(0.99999999999980493, 6.2471473085930200e-007, 0.0)))),
             NaturalFace(new Geom_CylindricalSurface(
               gp_Ax3(gp_Pnt(-6.4812490053250649, 753.39408794522092, 279.16400974257465), gp_Dir(1.0, 0.0, 0.0),
                      gp_Dir(0.0, 1.0, 0.0)),
               19.712534607908712)),
             1);
  // a sphere near a cylinder twice its radius (ApproxWithPCurves false), a cylinder and a torus of its minor radius
  for (int k = 0; k < 4; k++)
  {
    InlinePair("special-cylinder-sphere-nopc",
               FirstFace(BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(0, 0, -2), gp_Dir(0, 0, 1)), 2.0, 4.0).Shape()),
               FirstFace(BRepPrimAPI_MakeSphere(gp_Pnt(2.0 + 0.05 * (k - 2), 0.1 * k, 0.2), 0.5).Shape()), k);
    InlinePair("special-cylinder-torus-minor",
               FirstFace(BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(1.5, -2, 0), gp_Dir(0, 1, 0)), 0.4, 4.0).Shape()),
               FirstFace(BRepPrimAPI_MakeTorus(gp_Ax2(gp_Pnt(0, 0, 0.1 * k), gp_Dir(0, 0, 1)), 1.5, 0.4).Shape()),
               k);
    // a plane through a cone's axis (four lines before PrepareLines3D's rejection)
    InlinePair("special-plane-cone-axis",
               BRepBuilderAPI_MakeFace(gp_Pln(gp_Pnt(0, 0, 0), gp_Dir(1, 0.1 * k, 0)), -3, 3, -3, 3).Face(),
               FirstFace(BRepPrimAPI_MakeCone(gp_Ax2(gp_Pnt(0, 0, -1), gp_Dir(0, 0, 1)), 1.0, 0.0, 2.0).Shape()),
               1);
  }
  // the same face twice (self-intersection)
  InlinePair("special-same-cylinder", FirstFace(BRepPrimAPI_MakeCylinder(1.0, 2.0).Shape()),
             FirstFace(BRepPrimAPI_MakeCylinder(1.0, 2.0).Shape()), 0);
  {
    TopoDS_Face aF = Sheet(0);
    InlinePair("special-same-bspline", aF, aF, 0);
  }
  // B-splines with degenerated zones (spheres converted), against each other and a plane
  for (int k = 0; k < 4; k++)
  {
    occ::handle<Geom_Surface> aS1 = GeomConvert::SurfaceToBSplineSurface(
      new Geom_SphericalSurface(gp_Ax3(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)), 1.0));
    occ::handle<Geom_Surface> aS2 = GeomConvert::SurfaceToBSplineSurface(
      new Geom_SphericalSurface(gp_Ax3(gp_Pnt(0.6 + 0.1 * k, 0.1 * k, 0.3), gp_Dir(0.1 * k, 0, 1)), 0.8));
    InlinePair("special-bspline-sphere-poles", BRepBuilderAPI_MakeFace(aS1, 1.0e-7).Face(),
               BRepBuilderAPI_MakeFace(aS2, 1.0e-7).Face(), k);
    InlinePair("special-bspline-sphere-plane", BRepBuilderAPI_MakeFace(aS1, 1.0e-7).Face(),
               BRepBuilderAPI_MakeFace(gp_Pln(gp_Pnt(0, 0, 0.2 * k), gp_Dir(0.1, 0.2 * k, 1)), -2, 2, -2, 2).Face(), k);
  }
}

static TopoDS_Shape ReadOcc(const std::string& theDir, const char* theName)
{
  TopoDS_Shape aS;
  BRep_Builder aBB;
  BRepTools::Read(aS, (theDir + "/" + theName).c_str(), aBB);
  return aS;
}

// data/occ: faces of two shapes (or of one shape: the adjacent pairs) whose boxes overlap, at most theMax pairs
// spread over all of them.
static void OccPairs(const std::string& theDir, const char* theFile1, const char* theFile2, int theMax)
{
  TopoDS_Shape                                                  aS1 = ReadOcc(theDir, theFile1);
  TopoDS_Shape                                                  aS2 = ReadOcc(theDir, theFile2);
  NCollection_IndexedMap<TopoDS_Shape, TopTools_ShapeMapHasher> aF1, aF2;
  TopExp::MapShapes(aS1, TopAbs_FACE, aF1);
  TopExp::MapShapes(aS2, TopAbs_FACE, aF2);
  const bool               aSame = std::strcmp(theFile1, theFile2) == 0;
  std::vector<std::pair<int, int>> aPairs;
  if (aSame)
  {
    NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>, TopTools_ShapeMapHasher> anEF;
    TopExp::MapShapesAndAncestors(aS1, TopAbs_EDGE, TopAbs_FACE, anEF);
    for (int i = 1; i <= aF1.Extent(); i++)
      for (int j = i + 1; j <= aF1.Extent(); j++)
      {
        bool anAdjacent = false;
        for (int e = 1; e <= anEF.Extent() && !anAdjacent; e++)
        {
          bool aHasI = false, aHasJ = false;
          for (NCollection_List<TopoDS_Shape>::Iterator anIt(anEF(e)); anIt.More(); anIt.Next())
          {
            aHasI = aHasI || anIt.Value().IsSame(aF1(i));
            aHasJ = aHasJ || anIt.Value().IsSame(aF1(j));
          }
          anAdjacent = aHasI && aHasJ;
        }
        if (anAdjacent)
          aPairs.push_back({i, j});
      }
  }
  else
  {
    for (int i = 1; i <= aF1.Extent(); i++)
      for (int j = 1; j <= aF2.Extent(); j++)
        if (Overlap(aF1(i), aF2(j)))
          aPairs.push_back({i, j});
  }
  const int aN    = (int)aPairs.size();
  const int aStep = aN > theMax ? aN / theMax : 1;
  int       aVariantShift = 0;
  for (int k = 0, aDone = 0; k < aN && aDone < theMax; k += aStep, aDone++)
  {
    const int i = aPairs[k].first, j = aPairs[k].second;
    std::printf("case %d occ\nfile %s %d %s %d\n", ++gCase, theFile1, i, theFile2, j);
    Run(TopoDS::Face(aF1(i)), TopoDS::Face(aF2(j)), (aVariantShift++ % 3 == 2) ? 3 : 0);
  }
}

int main(int argc, char** argv)
{
  if (argc > 1 && std::strcmp(argv[1], "primitives") == 0)
  {
    Primitives();
    Specials();
    return 0;
  }
  if (argc < 2)
  {
    std::fprintf(stderr, "ffrec primitives | ffrec <data/occ directory>\n");
    return 1;
  }
  const std::string aDir = argv[1];
  gCase                  = 10000;
  const char* aCross[][2] = {{"mal_ecrou.brep", "mal_tige.brep"},
                             {"mal_ecrou.brep", "mal_vis.brep"},
                             {"mal_tige.brep", "mal_vis.brep"},
                             {"Pump_Nut.brep", "Pump_TopCover.brep"},
                             {"face1.brep", "face2.brep"},
                             {"MODERN_Sink_1.brep", "MODERN_Sink_1_opened.brep"},
                             {"MODERN_Cooker_1.brep", "MODERN_Cooker_1_opened.brep"}};
  for (const auto& aPair : aCross)
    OccPairs(aDir, aPair[0], aPair[1], 12);
  const char* aSelf[] = {"bottle.brep", "hammer.brep", "Axis_of_bearing.brep", "Ball.brep", "CrankArm.brep",
                         "wedge_ok.brep", "Bottom.brep", "fuse.brep", "wing.brep", "Motor-c.brep", "asahi.brep",
                         "Pump_Nut.brep", "mal_vis.brep", "solid.brep"};
  for (const char* aFile : aSelf)
    OccPairs(aDir, aFile, aFile, 10);
  return 0;
}
