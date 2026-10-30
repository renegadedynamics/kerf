// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
//
// cwrec: records native OCCT's BRepFill_CompatibleWires (unit 5S1) on sections built from a small deterministic
// generator: closed polygons, circles, ellipses and periodic B-splines split into random numbers of edges, single-edge
// circles and ellipses, open polylines, arcs and B-splines, point sections (a wire of one degenerated edge) at either
// end, looping sections (the first wire repeated last), sections shifted, rotated, tilted, reversed and placed by
// locations, and open and closed sections mixed. Built against build123d.NET's native OCCT V8_0_1
// (tools/cwrec/build.ps1).
//
// Each case writes its input sections as the BRep text of a compound and reads that text back, so native OCCT runs on
// exactly the shapes the port reads from the recording (tests/OcctNet.ModelingAlgorithms.Tests/BRepFill/
// BRepFill_CompatibleWiresNativeTests.cs). Per case:
//   CASE <kind> <seed>
//   OPT <WithRotation 0|1> <SetPercent's argument in hex, d for SetPercent() or - for no call>
//   IN <n>          followed by n lines: the input compound
//   THROW <type>    if Perform threw, else
//   STATUS <s> <IsDone> <degen1> <degen2>
//   OUT <n>         followed by n lines: a compound of the result wires, in order
//   GEN <edges>     for each edge of the input (TopExp::MapShapes order), the generated edges as <index><F|R|I|E> in
//                   the result's map of edges (0 if absent), ',' between them, ';' after each input edge
//
//   cwrec > cwrec.txt
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cmath>
#include <sstream>
#include <string>
#include <typeinfo>
#include <vector>

#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepFill_CompatibleWires.hxx>
#include <BRepTools.hxx>
#include <BRep_Builder.hxx>
#include <GeomAPI_Interpolate.hxx>
#include <Geom_BSplineCurve.hxx>
#include <Geom_Circle.hxx>
#include <Geom_Ellipse.hxx>
#include <NCollection_HArray1.hxx>
#include <Standard_Failure.hxx>
#include <TopExp.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Iterator.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <gp_Ax2.hxx>
#include <gp_Circ.hxx>
#include <gp_Elips.hxx>
#include <gp_Trsf.hxx>

static std::string Hex(double theValue)
{
  uint64_t aBits;
  std::memcpy(&aBits, &theValue, 8);
  char aBuf[20];
  std::snprintf(aBuf, sizeof(aBuf), "%016llx", (unsigned long long)aBits);
  return aBuf;
}

//! The generator: a 64-bit LCG; U() has 32 significant bits.
struct Rng
{
  uint64_t s;

  explicit Rng(uint64_t theSeed)
      : s(theSeed * 2862933555777941757ULL + 3037000493ULL)
  {
  }

  uint64_t Next()
  {
    s = s * 6364136223846793005ULL + 1442695040888963407ULL;
    return s;
  }

  double U() { return (double)(Next() >> 32) / 4294967296.0; }

  double R(double a, double b) { return a + (b - a) * U(); }

  int I(int a, int b) { return a + (int)((Next() >> 33) % (uint64_t)(b - a + 1)); }
};

static std::string theOut;

static void Line(const std::string& theLine)
{
  theOut += theLine;
  theOut += "\n";
}

static void Flush()
{
  std::fputs(theOut.c_str(), stdout);
  theOut.clear();
}

static std::string Text(const TopoDS_Shape& theShape)
{
  std::ostringstream aStream;
  BRepTools::Write(theShape, aStream);
  return aStream.str();
}

static void Block(const char* theTag, const std::string& theText)
{
  int aCount = 0;
  for (char aChar : theText)
  {
    if (aChar == '\n')
    {
      aCount++;
    }
  }
  if (!theText.empty() && theText.back() != '\n')
  {
    aCount++;
  }
  Line(std::string(theTag) + " " + std::to_string(aCount));
  theOut += theText;
  if (!theText.empty() && theText.back() != '\n')
  {
    theOut += "\n";
  }
}

// ------------------------------------------------------------------------------------------------ sections

//! A frame for section number theIndex: height theIndex * gap, with some tilt and twist.
static gp_Ax2 Frame(Rng& r, int theIndex, double theTilt, double theTwist)
{
  gp_Pnt aO(r.R(-0.5, 0.5), r.R(-0.5, 0.5), theIndex * r.R(2.0, 4.0));
  gp_Dir aN(r.R(-theTilt, theTilt), r.R(-theTilt, theTilt), 1.0);
  double aA = theIndex * theTwist + r.R(-0.2, 0.2);
  gp_Ax2 anAx(aO, aN);
  gp_Ax2 aRes = anAx.Rotated(gp_Ax1(aO, aN), aA);
  return aRes;
}

static TopoDS_Wire Polygon(Rng& r, const gp_Ax2& theAx, int theNb, bool theClosed)
{
  BRepBuilderAPI_MakePolygon aPoly;
  double aR = r.R(1.0, 3.0);
  for (int i = 0; i < theNb; i++)
  {
    double aT = (2 * M_PI * i + r.R(-0.6, 0.6)) / theNb;
    if (!theClosed)
    {
      aT = M_PI * i / (theNb - 1);
    }
    double aRi = aR * r.R(0.7, 1.3);
    gp_Pnt aP = theAx.Location().XYZ() + aRi * cos(aT) * theAx.XDirection().XYZ()
                + aRi * sin(aT) * theAx.YDirection().XYZ();
    aPoly.Add(aP);
  }
  if (theClosed)
  {
    aPoly.Close();
  }
  return aPoly.Wire();
}

//! A curve between its parameters theT0 and theT1 cut into theNb edges sharing their vertices.
static TopoDS_Wire Split(Rng& r, const occ::handle<Geom_Curve>& theC, double theT0, double theT1, int theNb, bool theClosed)
{
  std::vector<double>        aT(theNb + 1);
  std::vector<TopoDS_Vertex> aV(theNb + 1);
  for (int i = 0; i <= theNb; i++)
  {
    aT[i] = theT0 + (theT1 - theT0) * (i + (i > 0 && i < theNb ? r.R(-0.3, 0.3) : 0.0)) / theNb;
  }
  BRep_Builder B;
  for (int i = 0; i <= theNb; i++)
  {
    if (theClosed && i == theNb)
    {
      aV[i] = aV[0];
    }
    else
    {
      B.MakeVertex(aV[i], theC->Value(aT[i]), 1.e-7);
    }
  }
  BRepBuilderAPI_MakeWire aMW;
  for (int i = 0; i < theNb; i++)
  {
    aMW.Add(BRepBuilderAPI_MakeEdge(theC, aV[i], aV[i + 1], aT[i], aT[i + 1]).Edge());
  }
  return aMW.Wire();
}

static TopoDS_Wire ClosedSmooth(Rng& r, const gp_Ax2& theAx, int theNb, int theKind)
{
  occ::handle<Geom_Curve> aC;
  double                  aT0 = 0.0, aT1 = 2 * M_PI;
  if (theKind == 0)
  {
    aC = new Geom_Circle(gp_Circ(theAx, r.R(1.0, 3.0)));
  }
  else if (theKind == 1)
  {
    double aMaj = r.R(1.5, 3.0);
    aC          = new Geom_Ellipse(gp_Elips(theAx, aMaj, aMaj * r.R(0.4, 0.9)));
  }
  else
  {
    int                                      aNbP = r.I(5, 8);
    occ::handle<NCollection_HArray1<gp_Pnt>> aPts = new NCollection_HArray1<gp_Pnt>(1, aNbP);
    for (int i = 1; i <= aNbP; i++)
    {
      double aT  = 2 * M_PI * (i - 1) / aNbP;
      double aRi = r.R(1.2, 2.5);
      aPts->SetValue(i,
                     theAx.Location().XYZ() + aRi * cos(aT) * theAx.XDirection().XYZ()
                       + aRi * sin(aT) * theAx.YDirection().XYZ());
    }
    GeomAPI_Interpolate anInt(aPts, true, 1.e-7);
    anInt.Perform();
    occ::handle<Geom_BSplineCurve> aBS = anInt.Curve();
    aC                                 = aBS;
    aT0                                = aBS->FirstParameter();
    aT1                                = aBS->LastParameter();
  }
  double aShift = r.R(0.0, aT1 - aT0);
  if (theKind == 2)
  {
    aShift = 0.0;
  }
  return Split(r, aC, aT0 + aShift, aT1 + aShift, theNb, true);
}

static TopoDS_Wire OpenSmooth(Rng& r, const gp_Ax2& theAx, int theNb, int theKind)
{
  occ::handle<Geom_Curve> aC;
  double                  aT0, aT1;
  if (theKind == 0)
  {
    aC  = new Geom_Circle(gp_Circ(theAx, r.R(1.0, 3.0)));
    aT0 = r.R(0.0, 1.0);
    aT1 = aT0 + r.R(1.5, 4.0);
  }
  else
  {
    int                                      aNbP = r.I(3, 6);
    occ::handle<NCollection_HArray1<gp_Pnt>> aPts = new NCollection_HArray1<gp_Pnt>(1, aNbP);
    for (int i = 1; i <= aNbP; i++)
    {
      double aX = -2.0 + 4.0 * (i - 1) / (aNbP - 1);
      aPts->SetValue(i,
                     theAx.Location().XYZ() + aX * theAx.XDirection().XYZ()
                       + r.R(-1.0, 1.0) * theAx.YDirection().XYZ());
    }
    GeomAPI_Interpolate anInt(aPts, false, 1.e-7);
    anInt.Perform();
    occ::handle<Geom_BSplineCurve> aBS = anInt.Curve();
    aC                                 = aBS;
    aT0                                = aBS->FirstParameter();
    aT1                                = aBS->LastParameter();
  }
  return Split(r, aC, aT0, aT1, theNb, false);
}

static TopoDS_Wire PointSection(Rng& r, const gp_Ax2& theAx)
{
  BRep_Builder  B;
  TopoDS_Vertex aV;
  B.MakeVertex(aV, theAx.Location().Translated(gp_Vec(r.R(-0.3, 0.3), r.R(-0.3, 0.3), 0.0)), 1.e-7);
  TopoDS_Edge aDegEdge;
  B.MakeEdge(aDegEdge);
  B.Add(aDegEdge, aV.Oriented(TopAbs_FORWARD));
  B.Add(aDegEdge, aV.Oriented(TopAbs_REVERSED));
  B.Degenerated(aDegEdge, true);
  TopoDS_Wire aW;
  B.MakeWire(aW);
  B.Add(aW, aDegEdge);
  aW.Closed(true);
  return aW;
}

//! Random variations: reversed wire, a location or a geometric copy moving it slightly.
static TopoDS_Wire Vary(Rng& r, const TopoDS_Wire& theW)
{
  TopoDS_Wire aW = theW;
  int         aK = r.I(0, 5);
  if (aK == 1)
  {
    aW.Reverse();
  }
  else if (aK == 2 || aK == 3)
  {
    gp_Trsf aT;
    aT.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)), r.R(-0.5, 0.5));
    BRepBuilderAPI_Transform aTr(aW, aT, aK == 3);
    aW = TopoDS::Wire(aTr.Shape());
  }
  return aW;
}

static const char* Kinds[] = {"closed-diff",
                              "closed-same",
                              "closed-smooth",
                              "closed-point",
                              "closed-loop",
                              "closed-single",
                              "closed-tilt",
                              "open-diff",
                              "open-same",
                              "open-smooth",
                              "open-point",
                              "mixed"};
static const int   Counts[] = {50, 40, 40, 30, 20, 30, 40, 40, 30, 30, 30, 10};

static std::vector<TopoDS_Shape> Sections(Rng& r, int theKind)
{
  std::vector<TopoDS_Shape> aS;
  int                       aNb   = r.I(2, 4);
  double                    aTilt = (theKind == 6) ? 0.6 : 0.05;
  double                    aTw   = r.R(-0.6, 0.6);
  int                       aSame = r.I(3, 6);
  int                       aSm   = r.I(0, 2);
  for (int i = 0; i < aNb; i++)
  {
    gp_Ax2      anAx = Frame(r, i, aTilt, aTw);
    TopoDS_Wire aW;
    switch (theKind)
    {
      case 0:
      case 6:
        aW = r.I(0, 2) == 0 ? ClosedSmooth(r, anAx, r.I(1, 5), r.I(0, 2)) : Polygon(r, anAx, r.I(3, 7), true);
        break;
      case 1:
        aW = Polygon(r, anAx, aSame, true);
        break;
      case 2:
        aW = ClosedSmooth(r, anAx, r.I(2, 5), aSm);
        break;
      case 3:
      case 4:
        aW = r.I(0, 1) == 0 ? ClosedSmooth(r, anAx, r.I(1, 4), r.I(0, 2)) : Polygon(r, anAx, r.I(3, 6), true);
        break;
      case 5:
        aW = ClosedSmooth(r, anAx, 1, r.I(0, 1));
        break;
      case 7:
        aW = Polygon(r, anAx, r.I(2, 6), false);
        break;
      case 8:
        aW = Polygon(r, anAx, aSame, false);
        break;
      case 9:
      case 10:
        aW = r.I(0, 1) == 0 ? OpenSmooth(r, anAx, r.I(1, 4), r.I(0, 1)) : Polygon(r, anAx, r.I(2, 5), false);
        break;
      default:
        aW = r.I(0, 1) == 0 ? Polygon(r, anAx, r.I(3, 5), true) : Polygon(r, anAx, r.I(2, 5), false);
        break;
    }
    aS.push_back(Vary(r, aW));
  }
  if (theKind == 3 || theKind == 10)
  {
    int aWhich = r.I(1, 3);
    if (aWhich & 1)
    {
      aS.insert(aS.begin(), PointSection(r, Frame(r, -1, 0.0, 0.0)));
    }
    if (aWhich & 2)
    {
      aS.push_back(PointSection(r, Frame(r, aNb, 0.0, 0.0)));
    }
  }
  if (theKind == 4)
  {
    aS.push_back(aS.front());
  }
  return aS;
}

static std::string Orient(TopAbs_Orientation theO)
{
  switch (theO)
  {
    case TopAbs_FORWARD:
      return "F";
    case TopAbs_REVERSED:
      return "R";
    case TopAbs_INTERNAL:
      return "I";
    default:
      return "E";
  }
}

static void Case(int theKind, int theSeed)
{
  Rng r((uint64_t)(theKind * 1000 + theSeed));
  Line(std::string("CASE ") + Kinds[theKind] + " " + std::to_string(theSeed));
  std::vector<TopoDS_Shape> aSecs = Sections(r, theKind);
  bool                      aRot  = r.I(0, 2) != 0;
  int                       aPctK = r.I(0, 3);
  double                    aPct  = aPctK == 1 ? r.R(0.01, 0.3) : (aPctK == 2 ? 1.5 : 0.0);
  Line(std::string("OPT ") + (aRot ? "1" : "0") + " " + (aPctK == 1 || aPctK == 2 ? Hex(aPct) : (aPctK == 3 ? "d" : "-")));

  BRep_Builder    B;
  TopoDS_Compound anIn;
  B.MakeCompound(anIn);
  for (const TopoDS_Shape& aS : aSecs)
  {
    B.Add(anIn, aS);
  }
  std::string anInText = Text(anIn);
  Block("IN", anInText);

  // run on the shapes read back, as the port does
  TopoDS_Shape       aRead;
  std::istringstream anInStream(anInText);
  BRepTools::Read(aRead, anInStream, B);
  NCollection_Sequence<TopoDS_Shape> aSeq;
  for (TopoDS_Iterator anIt(aRead); anIt.More(); anIt.Next())
  {
    aSeq.Append(anIt.Value());
  }
  try
  {
    BRepFill_CompatibleWires aCW(aSeq);
    if (aPctK != 0 && aPctK != 3)
    {
      aCW.SetPercent(aPct);
    }
    else if (aPctK == 3)
    {
      aCW.SetPercent();
    }
    aCW.Perform(aRot);
    Line("STATUS " + std::to_string((int)aCW.GetStatus()) + " " + (aCW.IsDone() ? "1" : "0") + " "
         + (aCW.IsDegeneratedFirstSection() ? "1" : "0") + " " + (aCW.IsDegeneratedLastSection() ? "1" : "0"));
    TopoDS_Compound anOut;
    B.MakeCompound(anOut);
    const NCollection_Sequence<TopoDS_Shape>& aRes = aCW.Shape();
    for (int i = 1; i <= aRes.Length(); i++)
    {
      B.Add(anOut, aRes(i));
    }
    Block("OUT", Text(anOut));
    TopTools_IndexedMapOfShape anInEdges, anOutEdges;
    TopExp::MapShapes(aRead, TopAbs_EDGE, anInEdges);
    TopExp::MapShapes(anOut, TopAbs_EDGE, anOutEdges);
    std::string aGen = "GEN ";
    for (int i = 1; i <= anInEdges.Extent(); i++)
    {
      const NCollection_List<TopoDS_Shape>& aL = aCW.GeneratedShapes(TopoDS::Edge(anInEdges(i)));
      bool                                  aFirst = true;
      for (NCollection_List<TopoDS_Shape>::Iterator anIt(aL); anIt.More(); anIt.Next())
      {
        if (!aFirst)
        {
          aGen += ",";
        }
        aFirst = false;
        aGen += std::to_string(anOutEdges.FindIndex(anIt.Value())) + Orient(anIt.Value().Orientation());
      }
      aGen += ";";
    }
    Line(aGen);
  }
  catch (const Standard_Failure& anE)
  {
    std::string aName = typeid(anE).name();
    if (aName.rfind("class ", 0) == 0)
    {
      aName = aName.substr(6);
    }
    Line("THROW " + aName);
  }
}

int main()
{
  for (int k = 0; k < (int)(sizeof(Counts) / sizeof(Counts[0])); k++)
  {
    for (int s = 1; s <= Counts[k]; s++)
    {
      Case(k, s);
      Flush();
    }
  }
  return 0;
}
