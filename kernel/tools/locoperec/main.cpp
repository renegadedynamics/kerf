// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
//
// locoperec: records native OCCT's LocOpe local operations, which build123d.NET doesn't bind (unit 7H), on primitives,
// data/occ's shapes and the feat group's data files (a file holding only wires stands for the planar face of its
// first wire, as the feat scripts' mkplane). Built against build123d.NET's native OCCT V8_0_1
// (tools/locoperec/build.ps1).
//
// Inputs: the BRep files listed one per line in the file given as the first argument; the recording goes to the file
// given as the second argument. Per input (the primitives first, named PRIM_*):
//   FILE <name>
//   <tool> <index> <fields...>   or   <tool> <index> THROW <type>
// Every tool runs on the input made again (native writes into it). F(i) is the i-th face of the input's face map,
// E(i) its i-th edge; the first 8 faces and 12 edges are used.
//   SPLITE   0  LocOpe_SplitShape: a vertex at the middle of each of the first 12 edges (skipping degenerated ones)
//   SPLITW   i  LocOpe_SplitShape on a planar F(i): vertices at the middles of the first and the middle edge of its
//               outer wire, and the segment between them added as a wire
//   SPLITC   i  LocOpe_SplitShape on a planar F(i): a circle of radius 0.1 * the smaller UV extent around the UV middle
//               (when it is inside) added as a wire
//   SPLITTER i  LocOpe_WiresOnShape (Bind(E, F)) and LocOpe_Spliter, with the U iso of F(i) at its middle U, between
//               its V bounds
//   SPLITADD 0  LocOpe_WiresOnShape::Add with the iso edges of the first 4 faces, and LocOpe_Spliter
//   PRISM    i  LocOpe_Prism of F(i) along (3, 4, 12); PRISMT with a translation (1, 1, 1) first
//   REVOL    i  LocOpe_Revol of F(i) by 30 degrees around the Z axis through (xmin - 5, ymin - 5, zmin - 5) of its
//               vertices; REVOLD rotated by 15 degrees first
//   LFORM    i  LocOpe_LinearForm of F(i) along (3, 4, 12)
//   DPRISM   i  LocOpe_DPrism of a planar F(i) with at most 16 edges, height 10, angle 3 degrees; DPRISM2 heights 10
//               and 4
//   FIND     i  LocOpe_FindEdges(F(i), input): the pairs (index of the edge from, index of the edge to)
//   FIF      i  LocOpe_FindEdgesInFace(input, F(i)): the indices of the edges
// A shape is written <hash> <faces> <edges> <vertices>: hash, FNV-1a 64 of its BRep text (format 3, triangles, no
// normals), the Free and Checked columns of each line of flags replaced by 'x' (NULL for a null shape). A list of faces
// whose order natively follows addresses is written as a multiset hash (each face's hash, sorted). Doubles are their
// IEEE bits in hexadecimal. DPrism's shapes are written canonically (see Canonical): native varies run to run.
//
//   locoperec inputs.txt locoperec.txt
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <functional>

#include <BRepAlgoAPI_Cut.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCone.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <BRepPrimAPI_MakeTorus.hxx>
#include <BRepTools.hxx>
#include <BRepTopAdaptor_FClass2d.hxx>
#include <BRep_Builder.hxx>
#include <BRep_Tool.hxx>
#include <Geom_Curve.hxx>
#include <Geom_Line.hxx>
#include <Geom_Plane.hxx>
#include <Geom_RectangularTrimmedSurface.hxx>
#include <Geom_Surface.hxx>
#include <gp_Ax2.hxx>
#include <gp_Circ.hxx>
#include <gp_Pln.hxx>
#include <memory>
#include <LocOpe_DPrism.hxx>
#include <LocOpe_FindEdges.hxx>
#include <LocOpe_FindEdgesInFace.hxx>
#include <LocOpe_LinearForm.hxx>
#include <LocOpe_Prism.hxx>
#include <LocOpe_Revol.hxx>
#include <LocOpe_SplitShape.hxx>
#include <LocOpe_Spliter.hxx>
#include <LocOpe_WiresOnShape.hxx>
#include <OSD.hxx>
#include <Standard_ErrorHandler.hxx>
#include <Standard_Failure.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopTools_IndexedMapOfShape.hxx>

static std::ofstream gOut;
static std::string   gName;
static std::string   gDump; // LOCOPEREC_DUMP: a directory the shapes' texts are written to
static std::string   gTag;  // <input>.<tool>.<index>
static int           gNbDumped = 0;

static std::string Masked(const std::string& theText)
{
  std::string        aRes;
  std::istringstream aS(theText);
  std::string        aLine;
  while (std::getline(aS, aLine))
  {
    bool isFlags = aLine.size() == 7;
    for (size_t i = 0; isFlags && i < 7; i++)
    {
      isFlags = aLine[i] == '0' || aLine[i] == '1';
    }
    if (isFlags)
    {
      aLine[0] = 'x';
      aLine[2] = 'x';
    }
    aRes += aLine;
    aRes += '\n';
  }
  return aRes;
}

static uint64_t Fnv(const std::string& theText)
{
  uint64_t aHash = 14695981039346656037ull;
  for (unsigned char c : theText)
  {
    aHash ^= c;
    aHash *= 1099511628211ull;
  }
  return aHash;
}

static std::string Hex(uint64_t theValue)
{
  char aBuf[32];
  std::snprintf(aBuf, sizeof(aBuf), "%016llx", (unsigned long long)theValue);
  return aBuf;
}

static std::string D(double theValue)
{
  uint64_t aBits;
  std::memcpy(&aBits, &theValue, sizeof(aBits));
  return Hex(aBits);
}

static uint64_t ShapeHash(const TopoDS_Shape& theShape)
{
  std::ostringstream aS;
  BRepTools::Write(theShape, aS, true, false, TopTools_FormatVersion_VERSION_3);
  if (!gDump.empty())
  {
    std::ofstream aF(gDump + "/" + gTag + "." + std::to_string(gNbDumped++) + ".brep", std::ios::binary);
    aF << aS.str();
  }
  return Fnv(Masked(aS.str()));
}

static int Count(const TopoDS_Shape& theShape, TopAbs_ShapeEnum theType)
{
  if (theShape.IsNull())
  {
    return 0;
  }
  TopTools_IndexedMapOfShape aMap;
  TopExp::MapShapes(theShape, theType, aMap);
  return aMap.Extent();
}

static std::string Rec(const TopoDS_Shape& theShape)
{
  if (theShape.IsNull())
  {
    return "NULL";
  }
  return Hex(ShapeHash(theShape)) + " " + std::to_string(Count(theShape, TopAbs_FACE)) + " "
         + std::to_string(Count(theShape, TopAbs_EDGE)) + " "
         + std::to_string(Count(theShape, TopAbs_VERTEX));
}

// The multiset hash of a list of shapes: each one's hash, sorted, joined
static std::string SetHash(const NCollection_List<TopoDS_Shape>& theList)
{
  std::vector<std::string> aHashes;
  for (NCollection_List<TopoDS_Shape>::Iterator anIt(theList); anIt.More(); anIt.Next())
  {
    aHashes.push_back(Hex(ShapeHash(anIt.Value())));
  }
  std::sort(aHashes.begin(), aHashes.end());
  std::string aJoined;
  for (const std::string& aH : aHashes)
  {
    aJoined += aH + ",";
  }
  return Hex(Fnv(aJoined));
}

static void Run(const char* theTool, int theIndex, const std::function<std::string()>& theRun)
{
  std::string aLine;
  gTag      = gName + "." + theTool + "." + std::to_string(theIndex);
  gNbDumped = 0;
  try
  {
    OCC_CATCH_SIGNALS
    aLine = theRun();
  }
  catch (const Standard_Failure& e)
  {
    aLine = std::string("THROW ") + e.ExceptionType();
  }
  if (aLine == "SKIP")
  {
    return;
  }
  gOut << theTool << " " << theIndex << " " << aLine << "\n";
  gOut.flush();
}

// The input: a primitive or a file; a file holding only wires stands for the planar face of its first wire
static TopoDS_Shape Make(const std::string& theName)
{
  if (theName == "PRIM_BOX")
  {
    return BRepPrimAPI_MakeBox(10, 20, 30).Shape();
  }
  if (theName == "PRIM_CYLINDER")
  {
    return BRepPrimAPI_MakeCylinder(5, 10).Shape();
  }
  if (theName == "PRIM_CONE")
  {
    return BRepPrimAPI_MakeCone(5, 2, 8).Shape();
  }
  if (theName == "PRIM_SPHERE")
  {
    return BRepPrimAPI_MakeSphere(6).Shape();
  }
  if (theName == "PRIM_TORUS")
  {
    return BRepPrimAPI_MakeTorus(10, 3).Shape();
  }
  if (theName == "PRIM_BOXHOLE")
  {
    TopoDS_Shape aBox = BRepPrimAPI_MakeBox(10, 20, 30).Shape();
    TopoDS_Shape aCyl = BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(5, 10, -1), gp::DZ()), 3, 32).Shape();
    return BRepAlgoAPI_Cut(aBox, aCyl).Shape();
  }
  if (theName == "PRIM_HEXAGON")
  {
    BRepBuilderAPI_MakePolygon aPoly;
    for (int i = 0; i < 6; i++)
    {
      aPoly.Add(gp_Pnt(10 * std::cos(i * M_PI / 3), 10 * std::sin(i * M_PI / 3), 0));
    }
    aPoly.Close();
    return BRepBuilderAPI_MakeFace(aPoly.Wire(), true).Shape();
  }
  TopoDS_Shape aShape;
  BRep_Builder aB;
  BRepTools::Read(aShape, theName.c_str(), aB);
  if (!aShape.IsNull() && Count(aShape, TopAbs_FACE) == 0)
  {
    TopExp_Explorer anExp(aShape, TopAbs_WIRE);
    if (anExp.More())
    {
      BRepBuilderAPI_MakeFace aMF(TopoDS::Wire(anExp.Current()), true);
      aShape = aMF.IsDone() ? aMF.Shape() : TopoDS_Shape();
    }
    else
    {
      aShape.Nullify();
    }
  }
  return aShape;
}

static TopoDS_Face Face(const TopoDS_Shape& theShape, int theIndex)
{
  TopTools_IndexedMapOfShape aMap;
  TopExp::MapShapes(theShape, TopAbs_FACE, aMap);
  return TopoDS::Face(aMap(theIndex));
}

static TopoDS_Edge Edge(const TopoDS_Shape& theShape, int theIndex)
{
  TopTools_IndexedMapOfShape aMap;
  TopExp::MapShapes(theShape, TopAbs_EDGE, aMap);
  return TopoDS::Edge(aMap(theIndex));
}

static bool IsPlanar(const TopoDS_Face& theFace)
{
  occ::handle<Geom_Surface> aS = BRep_Tool::Surface(theFace);
  if (aS.IsNull())
  {
    return false;
  }
  if (aS->IsKind(STANDARD_TYPE(Geom_RectangularTrimmedSurface)))
  {
    aS = occ::down_cast<Geom_RectangularTrimmedSurface>(aS)->BasisSurface();
  }
  return aS->IsKind(STANDARD_TYPE(Geom_Plane));
}

// The middle of an edge with a 3D curve: its parameter and point
static bool Middle(const TopoDS_Edge& theEdge, double& thePrm, gp_Pnt& thePnt)
{
  if (BRep_Tool::Degenerated(theEdge))
  {
    return false;
  }
  double                  f, l;
  occ::handle<Geom_Curve> aC = BRep_Tool::Curve(theEdge, f, l);
  if (aC.IsNull())
  {
    return false;
  }
  thePrm = (f + l) / 2;
  thePnt = aC->Value(thePrm);
  return true;
}

// The U iso of the face at its middle U, between its V bounds
static TopoDS_Edge IsoEdge(const TopoDS_Face& theFace)
{
  occ::handle<Geom_Surface> aS = BRep_Tool::Surface(theFace);
  double                    u1, u2, v1, v2;
  BRepTools::UVBounds(theFace, u1, u2, v1, v2);
  occ::handle<Geom_Curve> anIso = aS->UIso((u1 + u2) / 2);
  BRepBuilderAPI_MakeEdge aME(anIso, v1, v2);
  return aME.IsDone() ? aME.Edge() : TopoDS_Edge();
}

static std::string DescCounts(LocOpe_Spliter& theSpliter, const TopoDS_Shape& theShape)
{
  TopTools_IndexedMapOfShape aMap;
  TopExp::MapShapes(theShape, TopAbs_FACE, aMap);
  std::string aRes;
  for (int i = 1; i <= std::min(30, aMap.Extent()); i++)
  {
    aRes += (i > 1 ? "," : "") + std::to_string(theSpliter.DescendantShapes(aMap(i)).Extent());
  }
  return aRes.empty() ? "-" : aRes;
}

static std::string Spliter(LocOpe_WiresOnShape* theWOnS, const TopoDS_Shape& theShape)
{
  occ::handle<LocOpe_WiresOnShape> aWOnS = theWOnS;
  LocOpe_Spliter                   aSpliter(theShape);
  aSpliter.Perform(aWOnS);
  if (!aSpliter.IsDone())
  {
    return "NOTDONE";
  }
  return Rec(aSpliter.ResultingShape()) + " " + std::to_string(aSpliter.DirectLeft().Extent()) + " "
         + std::to_string(aSpliter.Left().Extent()) + " " + SetHash(aSpliter.Left()) + " "
         + DescCounts(aSpliter, theShape);
}

// A canonical record of a shape whose BRep text natively varies run to run (DPrism's: the order of an edge's pcurves
// follows addresses): C<hash> <faces> <edges> <vertices>, the hash of the sorted lines F <area>, E <tolerance> <ends>
// and V <point> <tolerance>, each end sorted, the numbers rounded to 1e-6
// A double rounded to 1e-6, for canonical records (native's last bits vary run to run)
static std::string Q(double theValue)
{
  return std::to_string((long long)std::llround(theValue * 1e6)) + ";";
}

static std::string Canonical(const TopoDS_Shape& theShape)
{
  if (theShape.IsNull())
  {
    return "NULL";
  }
  std::vector<std::string>   aLines;
  TopTools_IndexedMapOfShape aMap;
  TopExp::MapShapes(theShape, TopAbs_FACE, aMap);
  for (int i = 1; i <= aMap.Extent(); i++)
  {
    GProp_GProps aProps;
    BRepGProp::SurfaceProperties(aMap(i), aProps);
    aLines.push_back("F " + Q(aProps.Mass()) + " " + std::to_string(Count(aMap(i), TopAbs_EDGE)));
  }
  aMap.Clear();
  TopExp::MapShapes(theShape, TopAbs_EDGE, aMap);
  for (int i = 1; i <= aMap.Extent(); i++)
  {
    const TopoDS_Edge& anE = TopoDS::Edge(aMap(i));
    TopoDS_Vertex      aV1, aV2;
    TopExp::Vertices(anE, aV1, aV2);
    std::string anEnds[2];
    for (int k = 0; k < 2; k++)
    {
      const TopoDS_Vertex& aV = k == 0 ? aV1 : aV2;
      if (!aV.IsNull())
      {
        gp_Pnt aP = BRep_Tool::Pnt(aV);
        anEnds[k] = Q(aP.X()) + Q(aP.Y()) + Q(aP.Z());
      }
    }
    if (anEnds[1] < anEnds[0])
    {
      std::swap(anEnds[0], anEnds[1]);
    }
    aLines.push_back("E " + Q(BRep_Tool::Tolerance(anE)) + " " + anEnds[0] + " " + anEnds[1]);
  }
  aMap.Clear();
  TopExp::MapShapes(theShape, TopAbs_VERTEX, aMap);
  for (int i = 1; i <= aMap.Extent(); i++)
  {
    const TopoDS_Vertex& aV = TopoDS::Vertex(aMap(i));
    gp_Pnt               aP = BRep_Tool::Pnt(aV);
    aLines.push_back("V " + Q(aP.X()) + Q(aP.Y()) + Q(aP.Z()) + " " + Q(BRep_Tool::Tolerance(aV)));
  }
  std::sort(aLines.begin(), aLines.end());
  std::string aJoined;
  for (const std::string& aL : aLines)
  {
    aJoined += aL + "\n";
  }
  if (!gDump.empty())
  {
    std::ofstream aF(gDump + "/" + gTag + ".canonical.txt", std::ios::binary);
    aF << aJoined;
  }
  return "C" + Hex(Fnv(aJoined)) + " " + std::to_string(Count(theShape, TopAbs_FACE)) + " "
         + std::to_string(Count(theShape, TopAbs_EDGE)) + " "
         + std::to_string(Count(theShape, TopAbs_VERTEX));
}

template <class T>
static std::string Swept(T& theSwept, const TopoDS_Face& theFace, bool theCanonical = false)
{
  std::function<std::string(const TopoDS_Shape&)> aRec = theCanonical ? Canonical : Rec;
  std::string aRes = aRec(theSwept.Shape()) + " " + aRec(theSwept.FirstShape()) + " " + aRec(theSwept.LastShape()) + " ";
  int         i    = 0;
  for (TopExp_Explorer anExp(theFace, TopAbs_EDGE); anExp.More() && i < 16; anExp.Next(), i++)
  {
    aRes += (i > 0 ? "," : "") + std::to_string(theSwept.Shapes(anExp.Current()).Extent());
  }
  return aRes;
}

template <class T>
static std::string Curves(const T& theSwept)
{
  NCollection_Sequence<occ::handle<Geom_Curve>> aCurves;
  theSwept.Curves(aCurves);
  std::string               aRes  = " " + std::to_string(aCurves.Length());
  occ::handle<Geom_Curve> aBary = theSwept.BarycCurve();
  occ::handle<Geom_Line>  aLine = occ::down_cast<Geom_Line>(aBary);
  if (!aLine.IsNull())
  {
    const gp_Pnt& aP = aLine->Lin().Location();
    aRes += " " + D(aP.X()) + " " + D(aP.Y()) + " " + D(aP.Z());
  }
  return aRes;
}

static void Tools(const std::string& theName)
{
  TopoDS_Shape S = Make(theName);
  if (S.IsNull())
  {
    return;
  }
  TopTools_IndexedMapOfShape aFMap, anEMap;
  TopExp::MapShapes(S, TopAbs_FACE, aFMap);
  TopExp::MapShapes(S, TopAbs_EDGE, anEMap);
  int aNbF = std::min(8, aFMap.Extent());
  int aNbE = std::min(12, anEMap.Extent());

  Run("SPLITE", 0, [&]() -> std::string {
    if (aNbE == 0)
    {
      return "SKIP";
    }
    TopoDS_Shape      aS = Make(theName);
    LocOpe_SplitShape aSplit(aS);
    for (int i = 1; i <= aNbE; i++)
    {
      TopoDS_Edge anE = Edge(aS, i);
      double      aPrm;
      gp_Pnt      aP;
      if (Middle(anE, aPrm, aP) && aSplit.CanSplit(anE))
      {
        aSplit.Add(BRepBuilderAPI_MakeVertex(aP).Vertex(), aPrm, anE);
      }
    }
    const NCollection_List<TopoDS_Shape>& aRes = aSplit.DescendantShapes(aS);
    std::string                           aLine =
      aRes.Extent() == 1 ? Rec(aRes.First()) : "N" + std::to_string(aRes.Extent());
    aLine += " ";
    for (int i = 1; i <= aNbE; i++)
    {
      aLine += (i > 1 ? "," : "") + std::to_string(aSplit.DescendantShapes(Edge(aS, i)).Extent());
    }
    return aLine;
  });

  for (int f = 1; f <= aNbF; f++)
  {
    if (!IsPlanar(TopoDS::Face(aFMap(f))))
    {
      continue;
    }
    Run("SPLITW", f, [&]() -> std::string {
      TopoDS_Shape                  aS = Make(theName);
      TopoDS_Face                   aF = Face(aS, f);
      std::vector<TopoDS_Edge>      anEdges;
      for (TopExp_Explorer anExp(BRepTools::OuterWire(aF), TopAbs_EDGE); anExp.More(); anExp.Next())
      {
        const TopoDS_Edge& anE = TopoDS::Edge(anExp.Current());
        double             aPrm;
        gp_Pnt             aP;
        bool               isNew = true;
        for (const TopoDS_Edge& anOther : anEdges)
        {
          isNew = isNew && !anOther.IsSame(anE);
        }
        if (isNew && Middle(anE, aPrm, aP))
        {
          anEdges.push_back(anE);
        }
      }
      if (anEdges.size() < 3)
      {
        return "SKIP";
      }
      const TopoDS_Edge& anE1 = anEdges[0];
      const TopoDS_Edge& anE2 = anEdges[anEdges.size() / 2];
      double             aPrm1, aPrm2;
      gp_Pnt             aP1, aP2;
      Middle(anE1, aPrm1, aP1);
      Middle(anE2, aPrm2, aP2);
      if (aP1.Distance(aP2) < 1e-3)
      {
        return "SKIP";
      }
      TopoDS_Vertex     aV1 = BRepBuilderAPI_MakeVertex(aP1).Vertex();
      TopoDS_Vertex     aV2 = BRepBuilderAPI_MakeVertex(aP2).Vertex();
      LocOpe_SplitShape aSplit(aS);
      aSplit.Add(aV1, aPrm1, anE1);
      aSplit.Add(aV2, aPrm2, anE2);
      TopoDS_Wire aW  = BRepBuilderAPI_MakeWire(BRepBuilderAPI_MakeEdge(aV1, aV2).Edge()).Wire();
      bool        isOk = aSplit.Add(aW, aF);
      const NCollection_List<TopoDS_Shape>& aRes = aSplit.DescendantShapes(aS);
      std::string aLine = std::string(isOk ? "1 " : "0 ")
                          + (aRes.Extent() == 1 ? Rec(aRes.First()) : "N" + std::to_string(aRes.Extent()));
      aLine += " " + std::to_string(aSplit.DescendantShapes(aF).Extent()) + " "
               + std::to_string(aSplit.LeftOf(aW, aF).Extent());
      return aLine;
    });

    Run("SPLITC", f, [&]() -> std::string {
      TopoDS_Shape aS = Make(theName);
      TopoDS_Face  aF = Face(aS, f);
      double       u1, u2, v1, v2;
      BRepTools::UVBounds(aF, u1, u2, v1, v2);
      gp_Pnt2d                aUV((u1 + u2) / 2, (v1 + v2) / 2);
      BRepTopAdaptor_FClass2d aClass(aF, 1e-9);
      if (aClass.Perform(aUV) != TopAbs_IN)
      {
        return "SKIP";
      }
      occ::handle<Geom_Surface> aSurf = BRep_Tool::Surface(aF);
      if (aSurf->IsKind(STANDARD_TYPE(Geom_RectangularTrimmedSurface)))
      {
        aSurf = occ::down_cast<Geom_RectangularTrimmedSurface>(aSurf)->BasisSurface();
      }
      gp_Pln            aPln = occ::down_cast<Geom_Plane>(aSurf)->Pln();
      gp_Pnt            aC   = BRep_Tool::Surface(aF)->Value(aUV.X(), aUV.Y());
      double            aR   = 0.1 * std::min(u2 - u1, v2 - v1);
      gp_Circ           aCirc(gp_Ax2(aC, aPln.Axis().Direction()), aR);
      TopoDS_Wire       aW = BRepBuilderAPI_MakeWire(BRepBuilderAPI_MakeEdge(aCirc).Edge()).Wire();
      LocOpe_SplitShape aSplit(aS);
      bool              isOk = aSplit.Add(aW, aF);
      const NCollection_List<TopoDS_Shape>& aRes = aSplit.DescendantShapes(aS);
      return std::string(isOk ? "1 " : "0 ")
             + (aRes.Extent() == 1 ? Rec(aRes.First()) : "N" + std::to_string(aRes.Extent())) + " "
             + std::to_string(aSplit.DescendantShapes(aF).Extent());
    });
  }

  for (int f = 1; f <= aNbF; f++)
  {
    Run("SPLITTER", f, [&]() -> std::string {
      TopoDS_Shape aS  = Make(theName);
      TopoDS_Face  aF  = Face(aS, f);
      TopoDS_Edge  anE = IsoEdge(aF);
      if (anE.IsNull())
      {
        return "SKIP";
      }
      LocOpe_WiresOnShape* aWOnS = new LocOpe_WiresOnShape(aS);
      aWOnS->Bind(anE, aF);
      return Spliter(aWOnS, aS);
    });
  }

  Run("SPLITADD", 0, [&]() -> std::string {
    TopoDS_Shape                       aS = Make(theName);
    NCollection_Sequence<TopoDS_Shape> anEdges;
    for (int f = 1; f <= std::min(4, aNbF); f++)
    {
      TopoDS_Edge anE = IsoEdge(Face(aS, f));
      if (!anE.IsNull())
      {
        anEdges.Append(anE);
      }
    }
    if (anEdges.IsEmpty())
    {
      return "SKIP";
    }
    LocOpe_WiresOnShape* aWOnS = new LocOpe_WiresOnShape(aS);
    bool                 isAdded = aWOnS->Add(anEdges);
    return std::string(isAdded ? "1 " : "0 ") + Spliter(aWOnS, aS);
  });

  for (int f = 1; f <= std::min(6, aNbF); f++)
  {
    Run("PRISM", f, [&]() -> std::string {
      TopoDS_Shape aS = Make(theName);
      TopoDS_Face  aF = Face(aS, f);
      LocOpe_Prism aPrism(aF, gp_Vec(3, 4, 12));
      return Swept(aPrism, aF) + Curves(aPrism);
    });
    Run("PRISMT", f, [&]() -> std::string {
      TopoDS_Shape aS = Make(theName);
      TopoDS_Face  aF = Face(aS, f);
      LocOpe_Prism aPrism(aF, gp_Vec(3, 4, 12), gp_Vec(1, 1, 1));
      return Swept(aPrism, aF);
    });
    for (int aDec = 0; aDec <= 1; aDec++)
    {
      Run(aDec ? "REVOLD" : "REVOL", f, [&]() -> std::string {
        TopoDS_Shape aS = Make(theName);
        TopoDS_Face  aF = Face(aS, f);
        double       aMin[3] = {1e100, 1e100, 1e100};
        for (TopExp_Explorer anExp(aF, TopAbs_VERTEX); anExp.More(); anExp.Next())
        {
          gp_Pnt aP = BRep_Tool::Pnt(TopoDS::Vertex(anExp.Current()));
          aMin[0]   = std::min(aMin[0], aP.X());
          aMin[1]   = std::min(aMin[1], aP.Y());
          aMin[2]   = std::min(aMin[2], aP.Z());
        }
        if (aMin[0] > 1e99)
        {
          return "SKIP";
        }
        gp_Ax1       anAxis(gp_Pnt(aMin[0] - 5, aMin[1] - 5, aMin[2] - 5), gp::DZ());
        LocOpe_Revol aRevol;
        if (aDec)
        {
          aRevol.Perform(aF, anAxis, M_PI / 6, M_PI / 12);
        }
        else
        {
          aRevol.Perform(aF, anAxis, M_PI / 6);
        }
        return Swept(aRevol, aF);
      });
    }
    Run("LFORM", f, [&]() -> std::string {
      TopoDS_Shape      aS = Make(theName);
      TopoDS_Face       aF = Face(aS, f);
      LocOpe_LinearForm aForm(aF, gp_Vec(3, 4, 12), gp_Pnt(0, 0, 0), gp_Pnt(1, 0, 0));
      return Swept(aForm, aF);
    });
  }

  for (int f = 1; f <= aNbF; f++)
  {
    TopoDS_Face aF0 = TopoDS::Face(aFMap(f));
    if (!IsPlanar(aF0) || Count(aF0, TopAbs_EDGE) > 16)
    {
      continue;
    }
    for (int aTwo = 0; aTwo <= 1; aTwo++)
    {
      Run(aTwo ? "DPRISM2" : "DPRISM", f, [&]() -> std::string {
        TopoDS_Shape                   aS = Make(theName);
        TopoDS_Face                    aF = Face(aS, f);
        std::unique_ptr<LocOpe_DPrism> aDPrism(aTwo ? new LocOpe_DPrism(aF, 10, 4, 3 * M_PI / 180)
                                                    : new LocOpe_DPrism(aF, 10, 3 * M_PI / 180));
        if (!aDPrism->IsDone())
        {
          return "NOTDONE";
        }
        return Swept(*aDPrism, aF, true) + Curves(*aDPrism);
      });
    }
  }

  for (int f = 1; f <= aNbF; f++)
  {
    Run("FIND", f, [&]() -> std::string {
      TopoDS_Shape     aS = Make(theName);
      TopoDS_Face      aF = Face(aS, f);
      LocOpe_FindEdges aFind(aF, aS);
      TopTools_IndexedMapOfShape anEdges;
      TopExp::MapShapes(aS, TopAbs_EDGE, anEdges);
      std::string aRes;
      int         aNb = 0;
      for (aFind.InitIterator(); aFind.More(); aFind.Next(), aNb++)
      {
        aRes += " " + std::to_string(anEdges.FindIndex(aFind.EdgeFrom())) + ":"
                + std::to_string(anEdges.FindIndex(aFind.EdgeTo()));
      }
      return std::to_string(aNb) + aRes;
    });
    Run("FIF", f, [&]() -> std::string {
      TopoDS_Shape           aS = Make(theName);
      TopoDS_Face            aF = Face(aS, f);
      LocOpe_FindEdgesInFace aFind(aS, aF);
      TopTools_IndexedMapOfShape anEdges;
      TopExp::MapShapes(aS, TopAbs_EDGE, anEdges);
      std::string aRes;
      int         aNb = 0;
      for (aFind.Init(); aFind.More(); aFind.Next(), aNb++)
      {
        aRes += " " + std::to_string(anEdges.FindIndex(aFind.Edge()));
      }
      return std::to_string(aNb) + aRes;
    });
  }
}

int main(int argc, char** argv)
{
  if (argc < 3)
  {
    std::fprintf(stderr, "usage: locoperec inputs.txt out.txt\n");
    return 2;
  }
  OSD::SetSignal(false);
  if (const char* aDump = std::getenv("LOCOPEREC_DUMP"))
  {
    gDump = aDump;
  }
  std::ifstream aList(argv[1]);
  gOut.open(argv[2], std::ios::binary);
  std::vector<std::string> aPaths = {"PRIM_BOX", "PRIM_CYLINDER", "PRIM_CONE", "PRIM_SPHERE",
                                     "PRIM_TORUS", "PRIM_BOXHOLE", "PRIM_HEXAGON"};
  std::string              aPath;
  while (std::getline(aList, aPath))
  {
    if (!aPath.empty() && aPath.back() == '\r')
    {
      aPath.pop_back();
    }
    if (!aPath.empty())
    {
      aPaths.push_back(aPath);
    }
  }
  for (const std::string& aP : aPaths)
  {
    TopoDS_Shape aShape;
    try
    {
      OCC_CATCH_SIGNALS
      aShape = Make(aP);
    }
    catch (const Standard_Failure&)
    {
      continue;
    }
    if (aShape.IsNull())
    {
      continue;
    }
    std::printf("%s\n", aP.c_str());
    std::fflush(stdout);
    gName = aP.substr(aP.find_last_of("/\\") + 1);
    gOut << "FILE " << gName << "\n";
    Tools(aP);
    gOut.flush();
  }
  return 0;
}
