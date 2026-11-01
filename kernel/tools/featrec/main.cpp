// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
//
// featrec: records native OCCT's BRepFeat makers that build123d.NET doesn't bind (unit 7I): MakePrism, MakeRevol,
// MakePipe, MakeCylindricalHole and Gluer, on primitives with profiles on their planar faces. Built against
// build123d.NET's native OCCT V8_0_1 (tools/featrec/build.ps1).
//
// The recording holds each input's BRep text (the test reads it, as a caller's frozen shape, and runs the same
// operations): a compound of the basis shape and, for the face operations, a profile face (a rectangle or a circle of a
// sixth of the face's size around its centre, in its plane).
//   INPUT <k> <name>          then the BRep text (format 3), then END
//   PARAMS <k> <centre> <normal> <x direction> <size>   the profile's frame, which places the axes and spines
//   <op> <k> <params> => <done> <status> <hash> <coarse> <history>     or  => THROW <type>
// hash: FNV-1a 64 of the result's BRep text, the Free and Checked columns masked; coarse: FNV-1a 64 of the canonical
// record (decision 1017: the sorted faces' areas and edge counts, edges' tolerances and ends, vertices' points and
// tolerances, rounded to 1e-6, and the volume); history: for every face of the basis shape 'd' (deleted) or the number
// of Modified faces, then '|' and the number of Generated shapes of every edge of the profile.
//
//   featrec out.txt
#include <cstdio>
#include <cstdint>
#include <cmath>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <functional>

#include <BRepAdaptor_Surface.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepFeat_Gluer.hxx>
#include <BRepFeat_MakeCylindricalHole.hxx>
#include <BRepFeat_MakePipe.hxx>
#include <BRepFeat_MakePrism.hxx>
#include <BRepFeat_MakeRevol.hxx>
#include <BRepGProp.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRepTools.hxx>
#include <BRep_Builder.hxx>
#include <BRep_Tool.hxx>
#include <GProp_GProps.hxx>
#include <LocOpe_FindEdges.hxx>
#include <OSD.hxx>
#include <Standard_ErrorHandler.hxx>
#include <Standard_Failure.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Iterator.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <gp_Circ.hxx>
#include <gp_Pln.hxx>

static std::ofstream gOut;

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

static std::string Text(const TopoDS_Shape& theShape)
{
  std::ostringstream aS;
  BRepTools::Write(theShape, aS, true, false, TopTools_FormatVersion_VERSION_3);
  return aS.str();
}

static std::string Masked(const std::string& theText)
{
  std::string        aRes;
  std::istringstream aS(theText);
  std::string        aLine;
  while (std::getline(aS, aLine))
  {
    if (!aLine.empty() && aLine.back() == '\r')
    {
      aLine.pop_back();
    }
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

static std::string Q(double theValue)
{
  return std::to_string((long long)std::llround(theValue * 1e6)) + ";";
}

static std::string P(const gp_Pnt& p)
{
  return Q(p.X()) + Q(p.Y()) + Q(p.Z());
}

static std::string Coarse(const TopoDS_Shape& theShape)
{
  std::vector<std::string>   aLines;
  TopTools_IndexedMapOfShape aMap;
  TopExp::MapShapes(theShape, TopAbs_FACE, aMap);
  for (int i = 1; i <= aMap.Extent(); i++)
  {
    GProp_GProps aProps;
    BRepGProp::SurfaceProperties(aMap(i), aProps);
    TopTools_IndexedMapOfShape anEdges;
    TopExp::MapShapes(aMap(i), TopAbs_EDGE, anEdges);
    aLines.push_back("F " + Q(aProps.Mass()) + " " + std::to_string(anEdges.Extent()));
  }
  aMap.Clear();
  TopExp::MapShapes(theShape, TopAbs_EDGE, aMap);
  for (int i = 1; i <= aMap.Extent(); i++)
  {
    TopoDS_Vertex aV1, aV2;
    TopExp::Vertices(TopoDS::Edge(aMap(i)), aV1, aV2);
    std::string anEnds[2] = {aV1.IsNull() ? "" : P(BRep_Tool::Pnt(aV1)), aV2.IsNull() ? "" : P(BRep_Tool::Pnt(aV2))};
    if (anEnds[1] < anEnds[0])
    {
      std::swap(anEnds[0], anEnds[1]);
    }
    aLines.push_back("E " + Q(BRep_Tool::Tolerance(TopoDS::Edge(aMap(i)))) + " " + anEnds[0] + " " + anEnds[1]);
  }
  aMap.Clear();
  TopExp::MapShapes(theShape, TopAbs_VERTEX, aMap);
  for (int i = 1; i <= aMap.Extent(); i++)
  {
    const TopoDS_Vertex& aV = TopoDS::Vertex(aMap(i));
    aLines.push_back("V " + P(BRep_Tool::Pnt(aV)) + " " + Q(BRep_Tool::Tolerance(aV)));
  }
  std::sort(aLines.begin(), aLines.end());
  GProp_GProps aVolume;
  BRepGProp::VolumeProperties(theShape, aVolume);
  std::string aJoined;
  for (const std::string& aL : aLines)
  {
    aJoined += aL + "\n";
  }
  aJoined += "volume " + Q(aVolume.Mass());
  return Hex(Fnv(aJoined));
}

static std::vector<TopoDS_Shape> Map(const TopoDS_Shape& theShape, TopAbs_ShapeEnum theType)
{
  TopTools_IndexedMapOfShape aMap;
  TopExp::MapShapes(theShape, theType, aMap);
  std::vector<TopoDS_Shape> aList;
  for (int i = 1; i <= aMap.Extent(); i++)
  {
    aList.push_back(aMap(i));
  }
  return aList;
}

// The result, the status and the history of a maker
static std::string Result(BRepBuilderAPI_MakeShape& theMaker, const std::string& theStatus,
                          const std::vector<TopoDS_Shape>& theFaces, const std::vector<TopoDS_Shape>& theEdges)
{
  if (!theMaker.IsDone())
  {
    return "0 " + theStatus;
  }
  TopoDS_Shape aRes  = theMaker.Shape();
  std::string  aLine = "1 " + theStatus + " " + (aRes.IsNull() ? std::string("NULL NULL") : Hex(Fnv(Masked(Text(aRes)))) + " " + Coarse(aRes)) + " ";
  for (const TopoDS_Shape& aF : theFaces)
  {
    aLine += theMaker.IsDeleted(aF) ? std::string("d ") : std::to_string(theMaker.Modified(aF).Extent()) + " ";
  }
  aLine += "|";
  for (const TopoDS_Shape& anE : theEdges)
  {
    aLine += " " + std::to_string(theMaker.Generated(anE).Extent());
  }
  return aLine;
}

static void Run(const std::string& theHead, const std::function<std::string()>& theRun)
{
  std::string aLine;
  try
  {
    OCC_CATCH_SIGNALS
    aLine = theRun();
  }
  catch (const Standard_Failure& e)
  {
    aLine = std::string("THROW ") + e.ExceptionType();
  }
  gOut << theHead << " => " << aLine << "\n";
  gOut.flush();
}

static std::vector<std::pair<std::string, TopoDS_Shape>> Bases()
{
  std::vector<std::pair<std::string, TopoDS_Shape>> aList;
  aList.emplace_back("box", BRepPrimAPI_MakeBox(10, 20, 30).Shape());
  aList.emplace_back("cylinder", BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)), 8, 20).Shape());
  TopoDS_Shape aBox  = BRepPrimAPI_MakeBox(gp_Pnt(-10, -10, 0), 20, 20, 12).Shape();
  TopoDS_Shape aHole = BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(6, 6, -1), gp_Dir(0, 0, 1)), 2, 14).Shape();
  aList.emplace_back("boxhole", BRepAlgoAPI_Cut(aBox, aHole).Shape());
  BRepBuilderAPI_MakePolygon aHexagon;
  for (int i = 0; i < 6; i++)
  {
    aHexagon.Add(gp_Pnt(12 * std::cos(i * M_PI / 3), 12 * std::sin(i * M_PI / 3), 0));
  }
  aHexagon.Close();
  TopoDS_Face aHexFace = BRepBuilderAPI_MakeFace(aHexagon.Wire(), true).Face();
  aList.emplace_back("hexagon", BRepPrimAPI_MakePrism(aHexFace, gp_Vec(0, 0, 15)).Shape());
  return aList;
}

// A rectangle or a circle around the centre of a planar face, in its plane, of a sixth of its size
static bool Profile(const TopoDS_Face& theFace, bool theCircle, TopoDS_Face& theProfile, gp_Dir& theNormal, gp_Pnt& theCentre,
                    double& theSize, gp_Dir& theX)
{
  BRepAdaptor_Surface anAdaptor(theFace);
  if (anAdaptor.GetType() != GeomAbs_Plane)
  {
    return false;
  }
  gp_Pln       aPln = anAdaptor.Plane();
  GProp_GProps aProps;
  BRepGProp::SurfaceProperties(theFace, aProps);
  gp_Pnt aC    = aProps.CentreOfMass();
  double aSize = std::sqrt(aProps.Mass()) / 6;
  gp_Dir aN = aPln.Axis().Direction(), aX = aPln.XAxis().Direction();
  if (theFace.Orientation() == TopAbs_REVERSED)
  {
    aN.Reverse();
  }
  gp_Pln aPlane(gp_Ax3(aC, aN, aX));
  if (theCircle)
  {
    TopoDS_Edge anEdge = BRepBuilderAPI_MakeEdge(gp_Circ(gp_Ax2(aC, aN, aX), aSize)).Edge();
    theProfile         = BRepBuilderAPI_MakeFace(aPlane, BRepBuilderAPI_MakeWire(anEdge).Wire(), true).Face();
  }
  else
  {
    gp_Dir aY  = aN.Crossed(aX);
    auto   Pt = [&](double u, double v) { return aC.Translated(gp_Vec(aX) * u + gp_Vec(aY) * v); };
    BRepBuilderAPI_MakePolygon aPoly(Pt(-aSize, -aSize / 2), Pt(aSize, -aSize / 2), Pt(aSize, aSize / 2), Pt(-aSize, aSize / 2), true);
    theProfile = BRepBuilderAPI_MakeFace(aPlane, aPoly.Wire(), true).Face();
  }
  theNormal = aN;
  theCentre = aC;
  theSize   = aSize;
  theX      = aX;
  return true;
}

static int gInput = 0;

// Writes the input and gives back the shape read from its text (both sides run on the text's shape)
static TopoDS_Shape Input(const std::string& theName, const TopoDS_Shape& theShape)
{
  std::string aText = Text(theShape);
  gOut << "INPUT " << gInput << " " << theName << "\n" << aText;
  if (!aText.empty() && aText.back() != '\n')
  {
    gOut << "\n";
  }
  gOut << "END\n";
  std::istringstream aS(aText);
  TopoDS_Shape       aShape;
  BRep_Builder       aB;
  BRepTools::Read(aShape, aS, aB);
  return aShape;
}

int main(int argc, char** argv)
{
  if (argc < 2)
  {
    std::fprintf(stderr, "usage: featrec out.txt\n");
    return 2;
  }
  OSD::SetSignal(false);
  gOut.open(argv[1], std::ios::binary);
  for (auto& [aName, aBase] : Bases())
  {
    std::vector<TopoDS_Shape> aBaseFaces = Map(aBase, TopAbs_FACE);
    for (int iF = 0; iF < (int)aBaseFaces.size() && iF < 8; iF++)
    {
      for (int isCircle = 0; isCircle < 2; isCircle++)
      {
        TopoDS_Face aProf;
        gp_Dir      aN, aX;
        gp_Pnt      aC;
        double      aSize;
        if (!Profile(TopoDS::Face(aBaseFaces[iF]), isCircle != 0, aProf, aN, aC, aSize, aX))
        {
          continue;
        }
        TopoDS_Compound aComp;
        BRep_Builder    aB;
        aB.MakeCompound(aComp);
        aB.Add(aComp, aBase);
        aB.Add(aComp, aProf);
        TopoDS_Shape aIn = Input(aName + "." + std::to_string(iF) + (isCircle ? ".circle" : ".rect"), aComp);
        const int    k   = gInput++;
        {
          char aBuf[512];
          std::snprintf(aBuf, sizeof(aBuf), "PARAMS %d %.17g %.17g %.17g %.17g %.17g %.17g %.17g %.17g %.17g %.17g\n", k, aC.X(), aC.Y(),
                        aC.Z(), aN.X(), aN.Y(), aN.Z(), aX.X(), aX.Y(), aX.Z(), aSize);
          gOut << aBuf;
        }
        // the reading's shapes
        TopoDS_Iterator anIt(aIn);
        TopoDS_Shape    aS = anIt.Value();
        anIt.Next();
        TopoDS_Face               aP     = TopoDS::Face(anIt.Value());
        std::vector<TopoDS_Shape> aFaces = Map(aS, TopAbs_FACE);
        std::vector<TopoDS_Shape> aEdges = Map(aP, TopAbs_EDGE);
        TopoDS_Face               aSk    = TopoDS::Face(aFaces[iF]);
        int                       iUntil = -1;
        for (int j = (int)aFaces.size() - 1; j >= 0; j--)
        {
          TopoDS_Face aDummy;
          gp_Dir      d1, d2;
          gp_Pnt      c;
          double      s;
          if (j != iF && Profile(TopoDS::Face(aFaces[j]), false, aDummy, d1, c, s, d2))
          {
            iUntil = j;
            break;
          }
        }
        for (int aFuse = 0; aFuse < 2; aFuse++)
        {
          for (int aMod = 0; aMod < 2; aMod++)
          {
            const std::string aP2 = std::to_string(aFuse) + " " + std::to_string(aMod);
            auto Prism = [&](int theMode, const std::string& theOp) {
              Run("PRISM " + std::to_string(k) + " " + theOp + " " + aP2, [&]() {
                BRepFeat_MakePrism aM(aS, aP, aSk, aFuse ? aN : aN.Reversed(), aFuse, aMod != 0);
                switch (theMode)
                {
                  case 0: aM.Perform(5.0); break;
                  case 1: aM.PerformThruAll(); break;
                  case 2: aM.Perform(aFaces[iUntil]); break;
                  case 3: aM.PerformUntilEnd(); break;
                  default: aM.PerformFromEnd(aFaces[iUntil]); break;
                }
                return Result(aM, std::to_string((int)aM.CurrentStatusError()), aFaces, aEdges);
              });
            };
            Prism(0, "length");
            Prism(1, "thruall");
            Prism(3, "untilend");
            if (iUntil >= 0)
            {
              Prism(2, "until");
              Prism(4, "fromend");
            }
            // a revolution around a line of the face's plane, a size away from the profile
            gp_Ax1 anAxis(aC.Translated(gp_Vec(aN.Crossed(aX)) * (2.5 * aSize)), aX);
            auto Revol = [&](int theMode, const std::string& theOp) {
              Run("REVOL " + std::to_string(k) + " " + theOp + " " + aP2, [&]() {
                BRepFeat_MakeRevol aM(aS, aP, aSk, anAxis, aFuse, aMod != 0);
                switch (theMode)
                {
                  case 0: aM.Perform(30 * M_PI / 180); break;
                  case 1: aM.PerformThruAll(); break;
                  default: aM.Perform(aFaces[iUntil]); break;
                }
                return Result(aM, std::to_string((int)aM.CurrentStatusError()), aFaces, aEdges);
              });
            };
            Revol(0, "angle");
            Revol(1, "thruall");
            // a pipe along a straight spine on the normal
            Run("PIPE " + std::to_string(k) + " spine " + aP2, [&]() {
              gp_Dir      aDir   = aFuse ? aN : aN.Reversed();
              TopoDS_Edge aSpine = BRepBuilderAPI_MakeEdge(aC, aC.Translated(gp_Vec(aDir) * 5.0)).Edge();
              TopoDS_Wire aW     = BRepBuilderAPI_MakeWire(aSpine).Wire();
              BRepFeat_MakePipe aM(aS, aP, aSk, aW, aFuse, aMod != 0);
              aM.Perform();
              return Result(aM, std::to_string((int)aM.CurrentStatusError()), aFaces, aEdges);
            });
          }
        }
        // a hole along the face's inward normal, through the face's centre
        for (int aMode = 0; aMode < 5; aMode++)
        {
          Run("HOLE " + std::to_string(k) + " " + std::to_string(aMode), [&]() {
            BRepFeat_MakeCylindricalHole aM;
            aM.Init(aS, gp_Ax1(aC.Translated(gp_Vec(aN) * 1.0), aN.Reversed()));
            switch (aMode)
            {
              case 0: aM.Perform(aSize / 2); break;
              case 1: aM.PerformThruNext(aSize / 2); break;
              case 2: aM.PerformUntilEnd(aSize / 2); break;
              case 3: aM.PerformBlind(aSize / 2, 4.0); break;
              default: aM.Perform(aSize / 2, 0.5, 4.0); break;
            }
            aM.Build();
            std::string aLine = std::to_string((int)aM.Status()) + " " + std::to_string(aM.HasErrors() ? 1 : 0);
            if (aM.Status() == BRepFeat_NoError && !aM.HasErrors())
            {
              const TopoDS_Shape& aRes = aM.Shape();
              aLine += " " + (aRes.IsNull() ? std::string("NULL NULL") : Hex(Fnv(Masked(Text(aRes)))) + " " + Coarse(aRes));
            }
            return aLine;
          });
        }
      }
    }
    // a box glued on the first planar face, made from its profile's prism of length 3 outwards
    TopoDS_Face aProf;
    gp_Dir      aN, aX;
    gp_Pnt      aC;
    double      aSize;
    if (Profile(TopoDS::Face(aBaseFaces[0]), false, aProf, aN, aC, aSize, aX))
    {
      TopoDS_Shape    aNew = BRepPrimAPI_MakePrism(aProf, gp_Vec(aN) * 3.0).Shape();
      TopoDS_Compound aComp;
      BRep_Builder    aB;
      aB.MakeCompound(aComp);
      aB.Add(aComp, aBase);
      aB.Add(aComp, aNew);
      TopoDS_Shape aIn = Input(aName + ".glue", aComp);
      const int    k   = gInput++;
      {
        char aBuf[512];
        std::snprintf(aBuf, sizeof(aBuf), "PARAMS %d %.17g %.17g %.17g %.17g %.17g %.17g %.17g %.17g %.17g %.17g\n", k, aC.X(), aC.Y(),
                      aC.Z(), aN.X(), aN.Y(), aN.Z(), aX.X(), aX.Y(), aX.Z(), aSize);
        gOut << aBuf;
      }
      TopoDS_Iterator anIt(aIn);
      TopoDS_Shape    aS = anIt.Value();
      anIt.Next();
      TopoDS_Shape              aNw    = anIt.Value();
      std::vector<TopoDS_Shape> aFaces = Map(aS, TopAbs_FACE);
      std::vector<TopoDS_Shape> aNew2  = Map(aNw, TopAbs_FACE);
      Run("GLUE " + std::to_string(k) + " 0", [&]() {
        BRepFeat_Gluer aG(aNw, aS);
        // the new shape's face on the basis face: the prism's base, the one centred on the profile's centre
        TopoDS_Face aFnew;
        for (const TopoDS_Shape& aF : aNew2)
        {
          GProp_GProps aProps;
          BRepGProp::SurfaceProperties(aF, aProps);
          if (aProps.CentreOfMass().Distance(aC) < 1e-7)
          {
            aFnew = TopoDS::Face(aF);
          }
        }
        aG.Bind(aFnew, TopoDS::Face(aFaces[0]));
        LocOpe_FindEdges aFE(aFnew, aFaces[0]);
        for (aFE.InitIterator(); aFE.More(); aFE.Next())
        {
          aG.Bind(aFE.EdgeFrom(), aFE.EdgeTo());
        }
        aG.Build();
        std::vector<TopoDS_Shape> aNoEdges;
        return std::to_string((int)aG.OpeType()) + " " + Result(aG, "-", aFaces, aNoEdges);
      });
    }
  }
  return 0;
}
