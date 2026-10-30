// The shared part of borec, the native recorder of unit 6G's algorithms (BOPAlgo's checks and other builders, and
// BRepAlgoAPI_Check/Defeaturing): number and shape formatting, the capture files Draw-lite writes from the corpus
// (tests/OcctNet.DrawLite.Tests/DrawCapture.cs), the history text the port's DeterminismHarness.HistoryText writes,
// and the random primitives. Each area (check.cpp, volume.cpp, connected.cpp, defeature.cpp) writes its own
// recording, as the port's test of that area reads and writes it.
#pragma once
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCone.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <BRepPrimAPI_MakeTorus.hxx>
#include <BRepTools.hxx>
#include <BRepTools_History.hxx>
#include <BRep_Builder.hxx>
#include <Message_Alert.hxx>
#include <Message_Report.hxx>
#include <NCollection_IndexedMap.hxx>
#include <NCollection_List.hxx>
#include <Standard_Failure.hxx>
#include <TopExp.hxx>
#include <TopLoc_Location.hxx>
#include <TopTools_FormatVersion.hxx>
#include <TopTools_ShapeMapHasher.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Iterator.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Ax1.hxx>
#include <gp_Ax2.hxx>
#include <gp_Trsf.hxx>

namespace borec
{
//! A double as its 64 bits in hex.
inline std::string H(double theValue)
{
  uint64_t aBits;
  std::memcpy(&aBits, &theValue, 8);
  char aBuf[24];
  std::snprintf(aBuf, sizeof(aBuf), "%016llx", (unsigned long long)aBits);
  return aBuf;
}

//! BRep text as the port's tests compare it (BRepTextComparison.Write): version 3 with triangles and normals.
inline std::string BRepText(const TopoDS_Shape& theShape)
{
  std::ostringstream aStream;
  BRepTools::Write(theShape, aStream, true, true, TopTools_FormatVersion_VERSION_3);
  return aStream.str();
}

inline TopoDS_Shape ReadBRep(const std::string& theText)
{
  TopoDS_Shape       aShape;
  BRep_Builder       aB;
  std::istringstream aStream(theText);
  BRepTools::Read(aShape, aStream, aB);
  return aShape;
}

//! "<tag> <length>\n<text>\n": a block of text the port reads back by its length.
inline void WriteBlock(std::ostream& theOut, const std::string& theTag, const std::string& theText)
{
  theOut << theTag << " " << theText.size() << "\n" << theText << "\n";
}

//! A shape written as a block of BRep text (an empty block for a null shape).
inline void WriteShape(std::ostream& theOut, const std::string& theTag, const TopoDS_Shape& theShape)
{
  WriteBlock(theOut, theTag, theShape.IsNull() ? std::string() : BRepText(theShape));
}

//! Writes a shape as text and reads it back, so both sides start from the same bits.
inline TopoDS_Shape RoundTrip(const TopoDS_Shape& theShape)
{
  return ReadBRep(BRepText(theShape));
}

//! The alerts of a gravity, as " key1 key2 ..." (the port's alerts' GetMessageKey are the class names).
inline std::string Alerts(const occ::handle<Message_Report>& theReport, Message_Gravity theGravity)
{
  std::string aS;
  for (NCollection_List<occ::handle<Message_Alert>>::Iterator it(theReport->GetAlerts(theGravity)); it.More(); it.Next())
    aS += std::string(" ") + it.Value()->GetMessageKey();
  return aS;
}

inline const char* TypeName(TopAbs_ShapeEnum theType)
{
  static const char* aNames[] = {"TopAbs_COMPOUND", "TopAbs_COMPSOLID", "TopAbs_SOLID",  "TopAbs_SHELL", "TopAbs_FACE",
                                 "TopAbs_WIRE",     "TopAbs_EDGE",      "TopAbs_VERTEX", "TopAbs_SHAPE"};
  return aNames[(int)theType];
}

//! The history as the port's DeterminismHarness.HistoryText writes it: for every argument sub-shape of a supported
//! type, in TopExp::MapShapes order over the arguments in turn, the indices of its modified and generated shapes in
//! the result's MapShapes numbering (0 for a shape not in the result), and whether it is removed.
inline std::string HistoryText(const std::vector<TopoDS_Shape>&      theArguments,
                               const TopoDS_Shape&                   theResult,
                               const occ::handle<BRepTools_History>& theHistory)
{
  if (theHistory.IsNull())
    return "no history";
  NCollection_IndexedMap<TopoDS_Shape, TopTools_ShapeMapHasher> aResultMap, anArgumentMap;
  if (!theResult.IsNull())
    TopExp::MapShapes(theResult, aResultMap);
  for (const TopoDS_Shape& anArg : theArguments)
    TopExp::MapShapes(anArg, anArgumentMap);
  std::string aText;
  for (int i = 1; i <= anArgumentMap.Extent(); ++i)
  {
    const TopoDS_Shape& aShape = anArgumentMap.FindKey(i);
    if (!BRepTools_History::IsSupportedType(aShape))
      continue;
    aText += std::to_string(i) + " " + TypeName(aShape.ShapeType()) + " M[";
    bool aFirst = true;
    for (NCollection_List<TopoDS_Shape>::Iterator it(theHistory->Modified(aShape)); it.More(); it.Next(), aFirst = false)
      aText += (aFirst ? "" : " ") + std::to_string(aResultMap.FindIndex(it.Value()));
    aText += "] G[";
    aFirst = true;
    for (NCollection_List<TopoDS_Shape>::Iterator it(theHistory->Generated(aShape)); it.More(); it.Next(), aFirst = false)
      aText += (aFirst ? "" : " ") + std::to_string(aResultMap.FindIndex(it.Value()));
    aText += theHistory->IsRemoved(aShape) ? "] removed\n" : "]\n";
  }
  return aText;
}

// ------------------------------------------------------------------------------------------------ captures

//! A command a corpus case ran, captured by Draw-lite: its words, and the BRep text of every word naming a shape.
struct CapturedCall
{
  std::vector<std::string>            Words;
  std::map<std::string, TopoDS_Shape> Shapes;
  std::map<std::string, std::string>  ShapeTexts;
};

struct CapturedCase
{
  std::string               Name;
  std::vector<CapturedCall> Calls;
};

//! Reads a capture file (DrawCapture.Write): "case <name>", then per call "call <n>", n lines "w <word>",
//! "shape <name> <length>" blocks; words hold no newline.
inline std::vector<CapturedCase> ReadCaptures(const std::string& thePath)
{
  std::vector<CapturedCase> aCases;
  std::ifstream             aF(thePath, std::ios::binary);
  std::stringstream         aSS;
  aSS << aF.rdbuf();
  std::string aText = aSS.str();
  size_t      aPos  = 0;
  auto        aLine = [&]() {
    size_t      anEnd = aText.find('\n', aPos);
    std::string aL    = aText.substr(aPos, anEnd - aPos);
    aPos              = anEnd + 1;
    return aL;
  };
  while (aPos < aText.size())
  {
    std::string aL = aLine();
    if (aL.rfind("case ", 0) == 0)
      aCases.push_back(CapturedCase{aL.substr(5), {}});
    else if (aL.rfind("call ", 0) == 0)
      aCases.back().Calls.emplace_back();
    else if (aL.rfind("w ", 0) == 0)
      aCases.back().Calls.back().Words.push_back(aL.substr(2));
    else if (aL.rfind("shape ", 0) == 0)
    {
      size_t      aSp   = aL.rfind(' ');
      std::string aKey  = aL.substr(6, aSp - 6);
      size_t      aLen  = std::stoul(aL.substr(aSp + 1));
      std::string aBody = aText.substr(aPos, aLen);
      aPos += aLen + 1;
      aCases.back().Calls.back().ShapeTexts[aKey] = aBody;
      aCases.back().Calls.back().Shapes[aKey]     = ReadBRep(aBody);
    }
  }
  return aCases;
}

// ------------------------------------------------------------------------------------------------ random primitives

//! xorshift, as tools/pfrec (each area seeds its own sequence).
struct Random
{
  uint64_t Seed;

  explicit Random(uint64_t theSeed)
      : Seed(theSeed)
  {
  }

  double Next()
  {
    Seed ^= Seed << 13;
    Seed ^= Seed >> 7;
    Seed ^= Seed << 17;
    return (double)(Seed % 1000003ull) / 1000003.0;
  }

  double Range(double theA, double theB) { return theA + (theB - theA) * Next(); }

  //! A box, cylinder, cone, sphere or torus about the origin (kind 0 to 4).
  TopoDS_Shape Primitive(int theKind)
  {
    switch (theKind)
    {
      case 0:
        return BRepPrimAPI_MakeBox(gp_Pnt(-1, -1, -1), Range(1.5, 2.5), Range(1.5, 2.5), Range(1.5, 2.5)).Shape();
      case 1:
        return BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(0, 0, -1), gp_Dir(0, 0, 1)), Range(0.6, 1.2), Range(1.5, 2.5)).Shape();
      case 2:
        return BRepPrimAPI_MakeCone(gp_Ax2(gp_Pnt(0, 0, -1), gp_Dir(0, 0, 1)), Range(0.8, 1.4), Range(0.0, 0.5), Range(1.5, 2.5))
          .Shape();
      case 3:
        return BRepPrimAPI_MakeSphere(Range(0.8, 1.3)).Shape();
      default:
        return BRepPrimAPI_MakeTorus(Range(0.9, 1.3), Range(0.2, 0.45)).Shape();
    }
  }

  //! The shape turned about a random axis through the origin and moved by up to theShift on each axis.
  TopoDS_Shape Placed(const TopoDS_Shape& theS, double theShift)
  {
    gp_Trsf aRot;
    gp_Dir  anAxis(Range(-1, 1), Range(-1, 1), Range(-1, 1) + 0.01);
    aRot.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), anAxis), Range(0.0, 6.0));
    gp_Trsf aMove;
    aMove.SetTranslation(gp_Vec(Range(-theShift, theShift), Range(-theShift, theShift), Range(-theShift, theShift)));
    return theS.Moved(TopLoc_Location(aMove * aRot));
  }
};
} // namespace borec

// The areas, each in its own file: write the recording of the captured corpus calls and of their own random cases.
int RecordCheck(const std::vector<borec::CapturedCase>& theCases, std::ostream& theOut);
int RecordVolume(const std::vector<borec::CapturedCase>& theCases, std::ostream& theOut);
int RecordConnected(const std::vector<borec::CapturedCase>& theCases, std::ostream& theOut);
int RecordDefeature(const std::vector<borec::CapturedCase>& theCases, std::ostream& theOut);
