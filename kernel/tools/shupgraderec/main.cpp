// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
//
// shupgraderec: records native OCCT's ShapeUpgrade divide and convert tools, which build123d.NET doesn't bind (unit
// 7B), with the parameters of the heal group's Draw scripts, on data/occ's shapes and the heal group's public inputs.
// Built against build123d.NET's native OCCT V8_0_1 (tools/shupgraderec/build.ps1).
//
// Inputs: the BRep files listed one per line in the file given as the first argument; the recording goes to the file
// given as the second argument. Per input:
//   FILE <name>
//   <tool> <status> <hash> <faces> <edges> <vertices>   or   <tool> THROW <type>
// for each tool, in this order:
//   ANGLE    ShapeUpgrade_ShapeDivideAngle, 95 degrees (DT_SplitAngle)
//   CLOSED   ShapeUpgrade_ShapeDivideClosed (DT_ClosedSplit)
//   CLEDGES  ShapeUpgrade_ShapeDivideClosedEdges
//   CONT     ShapeUpgrade_ShapeDivideContinuity, tolerance 0.01 (DT_ShapeDivide a 0.01)
//   CONTD    ShapeUpgrade_ShapeDivideContinuity, default parameters (DT_ShapeDivide a)
//   CONT2    ShapeUpgrade_ShapeDivideContinuity, C2 boundary, pcurve and surface criteria, tolerances 1e-3 and 1e-5
//   NUM4     ShapeUpgrade_ShapeDivideArea, splitting by number into 4 parts (DT_SplitByNumber a 4)
//   NUM23    ShapeUpgrade_ShapeDivideArea, 2 by 3 splits (DT_SplitByNumber a 2 3)
//   BEZ      ShapeUpgrade_ShapeConvertToBezier, surfaces, 3D curves and pcurves (DT_ShapeConvert a 1 1)
//   BEZC     ShapeUpgrade_ShapeConvertToBezier, 3D curves and pcurves only, lines and circles kept
//   RIW      ShapeUpgrade_RemoveInternalWires, minimal area 10 (RemoveIntWires 10 a)
//   SEW      ShapeUpgrade_ShellSewing, the shape's mean tolerance
// status: the hexadecimal flags of Status (bits 0-7 DONE1-8, 8-15 FAIL1-8) and bit 16 Perform's result (SEW: 0);
// hash: FNV-1a 64 of the result's BRep text (format 3, triangles, no normals), the Free and Checked columns of each
// line of flags replaced by 'x'; the counts are the result's distinct faces, edges and vertices.
// With SHUPGRADEREC_DUMP set to a directory, the results' texts are also written there (<name>.<tool>.brep).
//
//   shupgraderec inputs.txt shupgraderec.txt
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <functional>

#include <BRepTools.hxx>
#include <BRep_Builder.hxx>
#include <OSD.hxx>
#include <ShapeExtend.hxx>
#include <ShapeUpgrade_RemoveInternalWires.hxx>
#include <ShapeUpgrade_ShapeConvertToBezier.hxx>
#include <ShapeUpgrade_ShapeDivideAngle.hxx>
#include <ShapeUpgrade_ShapeDivideArea.hxx>
#include <ShapeUpgrade_ShapeDivideClosed.hxx>
#include <ShapeUpgrade_ShapeDivideClosedEdges.hxx>
#include <ShapeUpgrade_ShapeDivideContinuity.hxx>
#include <ShapeUpgrade_ShellSewing.hxx>
#include <Standard_ErrorHandler.hxx>
#include <Standard_Failure.hxx>
#include <TopExp.hxx>
#include <TopTools_IndexedMapOfShape.hxx>

static std::ofstream gOut;
static std::string   gDump;
static std::string   gName;

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

static void Record(const char* theTool, const TopoDS_Shape& theRes, int theStatus)
{
  std::ostringstream aS;
  BRepTools::Write(theRes, aS, true, false, TopTools_FormatVersion_VERSION_3);
  std::string aText = Masked(aS.str());
  if (!gDump.empty())
  {
    std::ofstream aF(gDump + "/" + gName + "." + theTool + ".brep", std::ios::binary);
    aF << aS.str();
  }
  uint64_t aHash = 14695981039346656037ull;
  for (unsigned char c : aText)
  {
    aHash ^= c;
    aHash *= 1099511628211ull;
  }
  int aNb[3] = {0, 0, 0};
  if (!theRes.IsNull())
  {
    const TopAbs_ShapeEnum aTypes[3] = {TopAbs_FACE, TopAbs_EDGE, TopAbs_VERTEX};
    for (int i = 0; i < 3; i++)
    {
      TopTools_IndexedMapOfShape aMap;
      TopExp::MapShapes(theRes, aTypes[i], aMap);
      aNb[i] = aMap.Extent();
    }
  }
  char aBuf[160];
  std::snprintf(aBuf, sizeof(aBuf), "%s %05x %016llx %d %d %d\n", theTool, theStatus, (unsigned long long)aHash,
                aNb[0], aNb[1], aNb[2]);
  gOut << aBuf;
}

template <class T>
static int Status(const T& theTool, bool theResult)
{
  int aStatus = 0;
  const ShapeExtend_Status aFlags[16] = {ShapeExtend_DONE1, ShapeExtend_DONE2, ShapeExtend_DONE3, ShapeExtend_DONE4,
                                         ShapeExtend_DONE5, ShapeExtend_DONE6, ShapeExtend_DONE7, ShapeExtend_DONE8,
                                         ShapeExtend_FAIL1, ShapeExtend_FAIL2, ShapeExtend_FAIL3, ShapeExtend_FAIL4,
                                         ShapeExtend_FAIL5, ShapeExtend_FAIL6, ShapeExtend_FAIL7, ShapeExtend_FAIL8};
  for (int i = 0; i < 16; i++)
  {
    if (theTool.Status(aFlags[i]))
    {
      aStatus |= 1 << i;
    }
  }
  return aStatus | (theResult ? 1 << 16 : 0);
}

static void Run(const char* theTool, const std::function<void()>& theRun)
{
  try
  {
    OCC_CATCH_SIGNALS
    theRun();
  }
  catch (const Standard_Failure& e)
  {
    gOut << theTool << " THROW " << e.ExceptionType() << "\n";
  }
}

template <class T>
static void Divide(const char* theTool, T& theDivider)
{
  Run(theTool, [&]() {
    bool isDone = theDivider.Perform();
    Record(theTool, theDivider.Result(), Status(theDivider, isDone));
  });
}

// Native OCCT changes some inputs in place (ShapeConvertToBezier's edges, a surface of linear extrusion's basis curve):
// each tool reads the file again.
static TopoDS_Shape Fresh(const std::string& thePath)
{
  TopoDS_Shape aShape;
  BRep_Builder aB;
  BRepTools::Read(aShape, thePath.c_str(), aB);
  return aShape;
}

static void Tools(const std::string& thePath)
{
  TopoDS_Shape theShape;
  {
    theShape = Fresh(thePath);
    ShapeUpgrade_ShapeDivideAngle aTool(95 * M_PI / 180, theShape);
    Divide("ANGLE", aTool);
  }
  {
    theShape = Fresh(thePath);
    ShapeUpgrade_ShapeDivideClosed aTool(theShape);
    Divide("CLOSED", aTool);
  }
  {
    theShape = Fresh(thePath);
    ShapeUpgrade_ShapeDivideClosedEdges aTool(theShape);
    Divide("CLEDGES", aTool);
  }
  {
    theShape = Fresh(thePath);
    ShapeUpgrade_ShapeDivideContinuity aTool(theShape);
    aTool.SetTolerance(0.01);
    Divide("CONT", aTool);
  }
  {
    theShape = Fresh(thePath);
    ShapeUpgrade_ShapeDivideContinuity aTool(theShape);
    Divide("CONTD", aTool);
  }
  {
    theShape = Fresh(thePath);
    ShapeUpgrade_ShapeDivideContinuity aTool(theShape);
    aTool.SetBoundaryCriterion(GeomAbs_C2);
    aTool.SetPCurveCriterion(GeomAbs_C2);
    aTool.SetSurfaceCriterion(GeomAbs_C2);
    aTool.SetTolerance(1e-3);
    aTool.SetTolerance2d(1e-5);
    Divide("CONT2", aTool);
  }
  {
    theShape = Fresh(thePath);
    ShapeUpgrade_ShapeDivideArea aTool(theShape);
    aTool.SetSplittingByNumber(true);
    aTool.NbParts() = 4;
    Divide("NUM4", aTool);
  }
  {
    theShape = Fresh(thePath);
    ShapeUpgrade_ShapeDivideArea aTool(theShape);
    aTool.SetSplittingByNumber(true);
    aTool.SetNumbersUVSplits(2, 3);
    Divide("NUM23", aTool);
  }
  {
    theShape = Fresh(thePath);
    ShapeUpgrade_ShapeConvertToBezier aTool(theShape);
    aTool.SetSurfaceConversion(true);
    aTool.Set2dConversion(true);
    aTool.Set3dConversion(true);
    Divide("BEZ", aTool);
  }
  {
    theShape = Fresh(thePath);
    ShapeUpgrade_ShapeConvertToBezier aTool(theShape);
    aTool.Set2dConversion(true);
    aTool.Set3dConversion(true);
    aTool.Set3dLineConversion(false);
    aTool.Set3dCircleConversion(false);
    Divide("BEZC", aTool);
  }
  theShape = Fresh(thePath);
  Run("RIW", [&]() {
    occ::handle<ShapeUpgrade_RemoveInternalWires> aTool = new ShapeUpgrade_RemoveInternalWires(theShape);
    aTool->MinArea() = 10;
    bool isDone = aTool->Perform();
    Record("RIW", aTool->GetResult(), Status(*aTool, isDone));
  });
  theShape = Fresh(thePath);
  Run("SEW", [&]() {
    ShapeUpgrade_ShellSewing aTool;
    Record("SEW", aTool.ApplySewing(theShape), 0);
  });
}

int main(int argc, char** argv)
{
  if (argc < 3)
  {
    std::fprintf(stderr, "usage: shupgraderec inputs.txt out.txt\n");
    return 2;
  }
  OSD::SetSignal(false);
  if (const char* aDump = std::getenv("SHUPGRADEREC_DUMP"))
  {
    gDump = aDump;
  }
  std::ifstream aList(argv[1]);
  gOut.open(argv[2], std::ios::binary);
  std::string aPath;
  while (std::getline(aList, aPath))
  {
    if (!aPath.empty() && aPath.back() == '\r')
    {
      aPath.pop_back();
    }
    if (aPath.empty())
    {
      continue;
    }
    TopoDS_Shape aShape;
    BRep_Builder aB;
    try
    {
      OCC_CATCH_SIGNALS
      if (!BRepTools::Read(aShape, aPath.c_str(), aB) || aShape.IsNull())
      {
        continue;
      }
    }
    catch (const Standard_Failure&)
    {
      continue;
    }
    gName = aPath.substr(aPath.find_last_of("/\\") + 1);
    gOut << "FILE " << gName << "\n";
    Tools(aPath);
    gOut.flush();
  }
  return 0;
}
