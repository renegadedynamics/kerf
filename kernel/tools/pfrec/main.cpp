// Records native BOPAlgo_PaveFiller results (unit 6E): for each case, the arguments as BRep text (read back before
// filling, so that native and the port fill the same bits), the filler's errors and warnings, and a dump of the data
// structure after filling, written exactly as the port's tests/OcctNet.ModelingAlgorithms.Tests/BOPAlgo/
// PaveFillerDump.cs writes it (every double as its 64 bits in hex). Built against build123d.NET's native OCCT V8_0_1
// (TKBO's own pave filler). The cases are the boolean group's *_simple scripts made of primitives and
// transformations (one case per distinct script), and pairs of primitive solids at pseudo-random placements; each is
// filled twice, in the default (destructive) mode and non-destructively.
//   pfrec <tests/boolean directory> <case list: group/name lines> > recording.txt
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cctype>
#include <algorithm>
#include <typeinfo>
#include <fstream>
#include <functional>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include <BOPAlgo_PaveFiller.hxx>
#include <BOPDS_CommonBlock.hxx>
#include <BOPDS_Curve.hxx>
#include <BOPDS_DS.hxx>
#include <BOPDS_FaceInfo.hxx>
#include <BOPDS_Interf.hxx>
#include <BOPDS_PaveBlock.hxx>
#include <BOPDS_Point.hxx>
#include <BOPDS_ShapeInfo.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCone.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <BRepPrimAPI_MakeTorus.hxx>
#include <BRepTools.hxx>
#include <BRep_Builder.hxx>
#include <BRep_Tool.hxx>
#include <Geom2d_Curve.hxx>
#include <Geom_Curve.hxx>
#include <IntTools_CommonPrt.hxx>
#include <IntTools_Curve.hxx>
#include <IntTools_Range.hxx>
#include <Message_Report.hxx>
#include <Message_Alert.hxx>
#include <Standard_Failure.hxx>
#include <TopLoc_Location.hxx>
#include <TopTools_FormatVersion.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Iterator.hxx>
#include <gp_Ax1.hxx>
#include <gp_Ax2.hxx>
#include <gp_Trsf.hxx>

static std::string H(double theValue)
{
  uint64_t aBits;
  std::memcpy(&aBits, &theValue, 8);
  char aBuf[24];
  std::snprintf(aBuf, sizeof(aBuf), "%016llx", (unsigned long long)aBits);
  return aBuf;
}

static std::string P(const gp_Pnt& theP) { return H(theP.X()) + " " + H(theP.Y()) + " " + H(theP.Z()); }

static std::string P2(const gp_Pnt2d& theP) { return H(theP.X()) + " " + H(theP.Y()); }

static std::string Join(const std::vector<int>& theValues, bool theSort)
{
  std::vector<int> aV = theValues;
  if (theSort)
    std::sort(aV.begin(), aV.end());
  std::string aS;
  for (size_t i = 0; i < aV.size(); i++)
    aS += (i ? "," : "") + std::to_string(aV[i]);
  return aS;
}

static std::vector<int> ToVec(const NCollection_List<int>& theL)
{
  std::vector<int> aV;
  for (NCollection_List<int>::Iterator it(theL); it.More(); it.Next())
    aV.push_back(it.Value());
  return aV;
}

static std::vector<int> ToVec(const NCollection_Map<int>& theM)
{
  std::vector<int> aV;
  for (NCollection_Map<int>::Iterator it(theM); it.More(); it.Next())
    aV.push_back(it.Value());
  return aV;
}

static std::string PB(const occ::handle<BOPDS_PaveBlock>& thePB)
{
  int nV1, nV2;
  thePB->Indices(nV1, nV2);
  return "[" + std::to_string(thePB->OriginalEdge()) + " " + std::to_string(thePB->Edge()) + " " + std::to_string(nV1)
         + " " + H(thePB->Pave1().Parameter()) + " " + std::to_string(nV2) + " " + H(thePB->Pave2().Parameter()) + "]";
}

static std::string CP(const IntTools_CommonPrt& theCP)
{
  std::string aS = std::to_string((int)theCP.Type());
  aS += " " + H(theCP.Range1().First()) + " " + H(theCP.Range1().Last());
  aS += " r2:";
  const NCollection_Sequence<IntTools_Range>& aR2 = theCP.Ranges2();
  for (int i = 1; i <= aR2.Length(); i++)
    aS += " " + H(aR2(i).First()) + " " + H(aR2(i).Last());
  aS += " vp " + H(theCP.VertexParameter1()) + " " + H(theCP.VertexParameter2());
  return aS;
}

// The dump, as PaveFillerDump.cs writes it.
static std::string Dump(const BOPDS_DS& theDS)
{
  std::string aB;
  const int   aNb = theDS.NbShapes(), aNbSrc = theDS.NbSourceShapes();
  aB += "ds " + std::to_string(aNb) + " " + std::to_string(aNbSrc) + "\n";
  std::vector<int> aFaces;
  for (int i = 0; i < aNb; i++)
  {
    const BOPDS_ShapeInfo& aSI   = theDS.ShapeInfo(i);
    const TopoDS_Shape&    aS    = aSI.Shape();
    TopAbs_ShapeEnum       aType = aSI.ShapeType();
    aB += "s " + std::to_string(i) + " " + std::to_string((int)aType) + " " + std::to_string((int)aS.Orientation());
    aB += " sub " + Join(ToVec(aSI.SubShapes()), i < aNbSrc);
    if (aType == TopAbs_VERTEX)
    {
      const TopoDS_Vertex& aV = TopoDS::Vertex(aS);
      aB += " tol " + H(BRep_Tool::Tolerance(aV)) + " p " + P(BRep_Tool::Pnt(aV));
    }
    else if (aType == TopAbs_EDGE)
    {
      const TopoDS_Edge& aE = TopoDS::Edge(aS);
      aB += " tol " + H(BRep_Tool::Tolerance(aE)) + " deg " + (BRep_Tool::Degenerated(aE) ? "1" : "0");
      if (i >= aNbSrc)
      {
        double                  aT1, aT2;
        occ::handle<Geom_Curve> aC = BRep_Tool::Curve(aE, aT1, aT2);
        aB += " r " + H(aT1) + " " + H(aT2);
        if (!aC.IsNull())
          aB += " c " + P(aC->Value(aT1)) + " " + P(aC->Value((aT1 + aT2) * 0.5)) + " " + P(aC->Value(aT2));
      }
    }
    else if (aType == TopAbs_FACE)
    {
      aB += " tol " + H(BRep_Tool::Tolerance(TopoDS::Face(aS)));
      aFaces.push_back(i);
    }
    aB += "\n";
  }
  for (int i = aNbSrc; i < aNb; i++)
  {
    if (theDS.ShapeInfo(i).ShapeType() != TopAbs_EDGE)
      continue;
    const TopoDS_Edge& aE = TopoDS::Edge(theDS.Shape(i));
    if (BRep_Tool::Degenerated(aE))
      continue;
    for (int nF : aFaces)
    {
      const TopoDS_Face&        aF = TopoDS::Face(theDS.Shape(nF));
      double                    aT1, aT2;
      bool                      anIsStored = false;
      occ::handle<Geom2d_Curve> aC2        = BRep_Tool::CurveOnSurface(aE, aF, aT1, aT2, &anIsStored);
      if (!aC2.IsNull() && anIsStored)
        aB += "pc " + std::to_string(i) + " " + std::to_string(nF) + " " + H(aT1) + " " + H(aT2) + " "
              + P2(aC2->Value((aT1 + aT2) * 0.5)) + "\n";
    }
  }
  for (int i = 0; i < aNb; i++)
  {
    int nSD = -1;
    if (theDS.HasShapeSD(i, nSD))
      aB += "sd " + std::to_string(i) + " " + std::to_string(nSD) + "\n";
  }
  std::map<const BOPDS_CommonBlock*, int>     aCBs;
  std::vector<occ::handle<BOPDS_CommonBlock>> aCBList;
  for (int i = 0; i < aNb; i++)
  {
    const BOPDS_ShapeInfo& aSI = theDS.ShapeInfo(i);
    if (aSI.ShapeType() != TopAbs_EDGE || !theDS.HasPaveBlocks(i))
      continue;
    const NCollection_List<occ::handle<BOPDS_PaveBlock>>& aLPB = theDS.PaveBlocks(i);
    for (NCollection_List<occ::handle<BOPDS_PaveBlock>>::Iterator it(aLPB); it.More(); it.Next())
    {
      const occ::handle<BOPDS_PaveBlock>& aPB = it.Value();
      aB += "pb " + std::to_string(i) + " " + PB(aPB);
      if (aPB->HasShrunkData())
      {
        double  aTS1, aTS2;
        Bnd_Box aBox;
        bool    aSplittable;
        aPB->ShrunkData(aTS1, aTS2, aBox, aSplittable);
        aB += " sh " + H(aTS1) + " " + H(aTS2) + " " + (aSplittable ? "1" : "0");
      }
      const occ::handle<BOPDS_CommonBlock>& aCB = theDS.CommonBlock(aPB);
      if (!aCB.IsNull())
      {
        auto anIt = aCBs.find(aCB.get());
        int  anId;
        if (anIt == aCBs.end())
        {
          anId = (int)aCBs.size();
          aCBs[aCB.get()] = anId;
          aCBList.push_back(aCB);
        }
        else
          anId = anIt->second;
        aB += " cb " + std::to_string(anId);
      }
      aB += "\n";
    }
  }
  for (size_t k = 0; k < aCBList.size(); k++)
  {
    const occ::handle<BOPDS_CommonBlock>& aCB = aCBList[k];
    aB += "cb " + std::to_string(k) + " " + H(aCB->Tolerance()) + " pbs";
    for (NCollection_List<occ::handle<BOPDS_PaveBlock>>::Iterator it(aCB->PaveBlocks()); it.More(); it.Next())
      aB += " " + PB(it.Value());
    aB += " faces " + Join(ToVec(aCB->Faces()), true) + "\n";
  }
  for (int nF : aFaces)
  {
    if (!theDS.HasFaceInfo(nF))
      continue;
    const BOPDS_FaceInfo& aFI = theDS.FaceInfo(nF);
    aB += "fi " + std::to_string(nF);
    const char* aTags[3] = {"on", "in", "sc"};
    const NCollection_IndexedMap<occ::handle<BOPDS_PaveBlock>>* aMaps[3] = {&aFI.PaveBlocksOn(),
                                                                             &aFI.PaveBlocksIn(),
                                                                             &aFI.PaveBlocksSc()};
    for (int t = 0; t < 3; t++)
    {
      aB += std::string(" ") + aTags[t];
      for (int j = 1; j <= aMaps[t]->Extent(); j++)
        aB += " " + PB(aMaps[t]->FindKey(j));
    }
    aB += " von " + Join(ToVec(aFI.VerticesOn()), true) + " vin " + Join(ToVec(aFI.VerticesIn()), true) + " vsc "
          + Join(ToVec(aFI.VerticesSc()), true) + "\n";
  }
  BOPDS_DS& aDS = const_cast<BOPDS_DS&>(theDS);
  for (int i = 0; i < aDS.InterfVV().Length(); i++)
  {
    const BOPDS_InterfVV& aI = aDS.InterfVV()(i);
    aB += "vv " + std::to_string(aI.Index1()) + " " + std::to_string(aI.Index2()) + " " + std::to_string(aI.IndexNew())
          + "\n";
  }
  for (int i = 0; i < aDS.InterfVE().Length(); i++)
  {
    const BOPDS_InterfVE& aI = aDS.InterfVE()(i);
    aB += "ve " + std::to_string(aI.Index1()) + " " + std::to_string(aI.Index2()) + " " + std::to_string(aI.IndexNew())
          + " " + H(aI.Parameter()) + "\n";
  }
  for (int i = 0; i < aDS.InterfVF().Length(); i++)
  {
    const BOPDS_InterfVF& aI = aDS.InterfVF()(i);
    double                aU, aV;
    aI.UV(aU, aV);
    aB += "vf " + std::to_string(aI.Index1()) + " " + std::to_string(aI.Index2()) + " " + std::to_string(aI.IndexNew())
          + " " + H(aU) + " " + H(aV) + "\n";
  }
  for (int i = 0; i < aDS.InterfEE().Length(); i++)
  {
    const BOPDS_InterfEE& aI = aDS.InterfEE()(i);
    aB += "ee " + std::to_string(aI.Index1()) + " " + std::to_string(aI.Index2()) + " " + std::to_string(aI.IndexNew())
          + " " + CP(aI.CommonPart()) + "\n";
  }
  for (int i = 0; i < aDS.InterfEF().Length(); i++)
  {
    const BOPDS_InterfEF& aI = aDS.InterfEF()(i);
    aB += "ef " + std::to_string(aI.Index1()) + " " + std::to_string(aI.Index2()) + " " + std::to_string(aI.IndexNew())
          + " " + CP(aI.CommonPart()) + "\n";
  }
  for (int i = 0; i < aDS.InterfFF().Length(); i++)
  {
    const BOPDS_InterfFF&                        aI  = aDS.InterfFF()(i);
    const NCollection_DynamicArray<BOPDS_Curve>& aVC = aI.Curves();
    const NCollection_DynamicArray<BOPDS_Point>& aVP = aI.Points();
    aB += "ff " + std::to_string(aI.Index1()) + " " + std::to_string(aI.Index2()) + " "
          + (aI.TangentFaces() ? "1" : "0") + " " + std::to_string(aVC.Length()) + " " + std::to_string(aVP.Length())
          + "\n";
    for (int j = 0; j < aVC.Length(); j++)
    {
      const BOPDS_Curve& aNC = aVC(j);
      aB += " c " + H(aNC.Tolerance()) + " " + H(aNC.TangentialTolerance()) + " tv "
            + Join(ToVec(aNC.TechnoVertices()), false);
      double aT1 = 0, aT2 = 0;
      gp_Pnt aP1, aP2;
      if (aNC.Curve().Bounds(aT1, aT2, aP1, aP2))
        aB += " b " + H(aT1) + " " + H(aT2) + " " + P(aP1) + " " + P(aP2);
      aB += " pbs";
      for (NCollection_List<occ::handle<BOPDS_PaveBlock>>::Iterator it(aNC.PaveBlocks()); it.More(); it.Next())
        aB += " " + PB(it.Value());
      aB += "\n";
    }
    for (int j = 0; j < aVP.Length(); j++)
      aB += " p " + std::to_string(aVP(j).Index()) + " " + P(aVP(j).Pnt()) + "\n";
  }
  return aB;
}

static std::string Text(const TopoDS_Shape& theShape)
{
  std::ostringstream aStream;
  BRepTools::Write(theShape, aStream, true, true, TopTools_FormatVersion_VERSION_3);
  return aStream.str();
}

static std::string Alerts(const occ::handle<Message_Report>& theReport, Message_Gravity theGravity)
{
  std::vector<std::string> aKeys;
  for (NCollection_List<occ::handle<Message_Alert>>::Iterator it(theReport->GetAlerts(theGravity)); it.More(); it.Next())
    aKeys.push_back(it.Value()->GetMessageKey());
  std::string aS;
  for (const std::string& aK : aKeys)
    aS += " " + aK;
  return aS;
}

static int gCase = 0;

// Writes the case: the arguments' compound as BRep text, then for each mode the filler's outcome and dump.
static void Record(const std::string& theName, const std::vector<TopoDS_Shape>& theArgs)
{
  TopoDS_Compound aC;
  BRep_Builder    aBB;
  aBB.MakeCompound(aC);
  for (const TopoDS_Shape& aS : theArgs)
    aBB.Add(aC, aS);
  const std::string aText = Text(aC);
  std::printf("case %d %s\nshapes %d\n", ++gCase, theName.c_str(), (int)aText.size());
  std::fwrite(aText.data(), 1, aText.size(), stdout);
  std::printf("\n");
  for (int aMode = 0; aMode < 2; aMode++)
  {
    // the same bits on both sides: fill shapes read back from the text
    TopoDS_Shape       aRead;
    BRep_Builder       aB2;
    std::istringstream anIn(aText);
    BRepTools::Read(aRead, anIn, aB2);
    NCollection_List<TopoDS_Shape> aLS;
    for (TopoDS_Iterator it(aRead); it.More(); it.Next())
      aLS.Append(it.Value());
    BOPAlgo_PaveFiller aPF;
    aPF.SetArguments(aLS);
    aPF.SetNonDestructive(aMode == 1);
    std::string aDump, anOutcome;
    try
    {
      aPF.Perform();
      anOutcome = "errors" + Alerts(aPF.GetReport(), Message_Fail) + "\nwarnings"
                  + Alerts(aPF.GetReport(), Message_Warning) + "\n";
      if (aPF.PDS() != nullptr)
        aDump = Dump(aPF.DS());
    }
    catch (Standard_Failure const&)
    {
      anOutcome = "throws\n";
    }
    std::printf("mode %d\n%s%send\n", aMode, anOutcome.c_str(), aDump.c_str());
  }
  std::fflush(stdout);
}

// ------------------------------------------------------------------------------------------------ Draw scripts

// Tcl-like numeric expressions of the boolean scripts: numbers, variables, pi, + - * / and functions.
struct Expr
{
  const std::map<std::string, double>& Vars;
  const char*                          p;
  bool                                 Ok = true;

  void   Skip() { while (*p == ' ') p++; }
  double Primary()
  {
    Skip();
    if (*p == '(')
    {
      p++;
      double v = Sum();
      Skip();
      if (*p == ')') p++; else Ok = false;
      return v;
    }
    if (std::isdigit((unsigned char)*p) || *p == '.')
    {
      char*  e;
      double v = std::strtod(p, &e);
      p        = e;
      return v;
    }
    if (std::isalpha((unsigned char)*p) || *p == '_')
    {
      std::string n;
      while (std::isalnum((unsigned char)*p) || *p == '_')
        n += *p++;
      Skip();
      if (*p == '(')
      {
        p++;
        std::vector<double> a;
        Skip();
        if (*p != ')')
        {
          for (;;)
          {
            a.push_back(Sum());
            Skip();
            if (*p == ',') { p++; continue; }
            break;
          }
        }
        if (*p == ')') p++; else Ok = false;
        if (n == "sqrt" && a.size() == 1) return std::sqrt(a[0]);
        if (n == "atan2" && a.size() == 2) return std::atan2(a[0], a[1]);
        if (n == "sin" && a.size() == 1) return std::sin(a[0]);
        if (n == "cos" && a.size() == 1) return std::cos(a[0]);
        if (n == "tan" && a.size() == 1) return std::tan(a[0]);
        if (n == "atan" && a.size() == 1) return std::atan(a[0]);
        if (n == "abs" && a.size() == 1) return std::fabs(a[0]);
        Ok = false;
        return 0;
      }
      if (n == "pi") return M_PI;
      auto it = Vars.find(n);
      if (it == Vars.end()) { Ok = false; return 0; }
      return it->second;
    }
    Ok = false;
    return 0;
  }
  double Unary()
  {
    Skip();
    if (*p == '-') { p++; return -Unary(); }
    if (*p == '+') { p++; return Unary(); }
    return Primary();
  }
  double Product()
  {
    double v = Unary();
    for (;;)
    {
      Skip();
      if (*p == '*') { p++; v *= Unary(); }
      else if (*p == '/') { p++; v /= Unary(); }
      else return v;
    }
  }
  double Sum()
  {
    double v = Product();
    for (;;)
    {
      Skip();
      if (*p == '+') { p++; v += Product(); }
      else if (*p == '-') { p++; v -= Product(); }
      else return v;
    }
  }
};

static bool Eval(const std::string& theS, const std::map<std::string, double>& theVars, double& theV)
{
  Expr e{theVars, theS.c_str()};
  theV = e.Sum();
  e.Skip();
  return e.Ok && *e.p == 0;
}

static std::vector<std::string> Words(const std::string& theLine)
{
  std::vector<std::string> aW;
  std::istringstream       aS(theLine);
  std::string              w;
  while (aS >> w)
    aW.push_back(w);
  return aW;
}

// Runs a script of box/pcylinder/pcone/psphere/ptorus/trotate/ttranslate/dset/bop lines; false if it has anything
// else. The shapes are made as Draw makes them; the transformations move them (Draw's basic transformations).
static bool RunScript(const std::string& theText, std::vector<TopoDS_Shape>& theArgs, std::string& theKey)
{
  std::map<std::string, double>       aVars;
  std::map<std::string, TopoDS_Shape> aShapes;
  std::istringstream                  aIn(theText);
  std::string                         aLine;
  bool                                aBop = false;
  while (std::getline(aIn, aLine))
  {
    if (!aLine.empty() && aLine.back() == '\r')
      aLine.pop_back();
    std::vector<std::string> w = Words(aLine);
    if (w.empty() || w[0][0] == '#')
      continue;
    const std::string& c = w[0];
    if (c == "checkprops" || c == "checkview" || c == "puts" || c.rfind("bop", 0) == 0 && c != "bop")
      continue;
    theKey += aLine + "\n";
    auto num = [&](size_t i, double& v) { return i < w.size() && Eval(w[i], aVars, v); };
    if (c == "dset")
    {
      for (size_t i = 1; i + 1 < w.size(); i += 2)
      {
        double v;
        if (!num(i + 1, v)) return false;
        aVars[w[i]] = v;
      }
    }
    else if (c == "box")
    {
      double v[6];
      if (w.size() == 8)
      {
        for (int k = 0; k < 6; k++) if (!num(2 + k, v[k])) return false;
        aShapes[w[1]] = BRepPrimAPI_MakeBox(gp_Pnt(v[0], v[1], v[2]), v[3], v[4], v[5]).Shape();
      }
      else if (w.size() == 5)
      {
        for (int k = 0; k < 3; k++) if (!num(2 + k, v[k])) return false;
        aShapes[w[1]] = BRepPrimAPI_MakeBox(v[0], v[1], v[2]).Shape();
      }
      else return false;
    }
    else if (c == "pcylinder")
    {
      double r, h;
      if (w.size() != 4 || !num(2, r) || !num(3, h)) return false;
      aShapes[w[1]] = BRepPrimAPI_MakeCylinder(r, h).Shape();
    }
    else if (c == "pcone")
    {
      double r1, r2, h;
      if (w.size() != 5 || !num(2, r1) || !num(3, r2) || !num(4, h)) return false;
      aShapes[w[1]] = BRepPrimAPI_MakeCone(r1, r2, h).Shape();
    }
    else if (c == "psphere")
    {
      double r;
      if (w.size() != 3 || !num(2, r)) return false;
      aShapes[w[1]] = BRepPrimAPI_MakeSphere(r).Shape();
    }
    else if (c == "ptorus")
    {
      double r1, r2;
      if (w.size() != 4 || !num(2, r1) || !num(3, r2)) return false;
      aShapes[w[1]] = BRepPrimAPI_MakeTorus(r1, r2).Shape();
    }
    else if (c == "trotate" || c == "ttranslate")
    {
      const size_t aNbNum = c == "trotate" ? 7 : 3;
      if (w.size() < aNbNum + 2) return false;
      const size_t aFirst = w.size() - aNbNum;
      double       v[7];
      for (size_t k = 0; k < aNbNum; k++) if (!num(aFirst + k, v[k])) return false;
      gp_Trsf aT;
      if (c == "trotate")
        aT.SetRotation(gp_Ax1(gp_Pnt(v[0], v[1], v[2]), gp_Dir(v[3], v[4], v[5])), v[6] * M_PI / 180.0);
      else
        aT.SetTranslation(gp_Vec(v[0], v[1], v[2]));
      for (size_t k = 1; k < aFirst; k++)
      {
        auto it = aShapes.find(w[k]);
        if (it == aShapes.end()) return false;
        it->second = it->second.Moved(TopLoc_Location(aT));
      }
    }
    else if (c == "bop")
    {
      if (w.size() != 3) return false;
      for (size_t k = 1; k < 3; k++)
      {
        auto it = aShapes.find(w[k]);
        if (it == aShapes.end()) return false;
        theArgs.push_back(it->second);
      }
      aBop = true;
    }
    else
      return false;
  }
  return aBop;
}

static void Scripts(const std::string& theDir, const std::string& theList)
{
  // theList: the cases as "group/name" lines (made by build.ps1 from the grids' directories)
  std::set<std::string> aSeen;
  std::ifstream         aList(theList);
  std::string           aName;
  while (std::getline(aList, aName))
  {
    if (!aName.empty() && aName.back() == '\r')
      aName.pop_back();
    std::ifstream     aF(theDir + "/" + aName, std::ios::binary);
    std::stringstream aSS;
    aSS << aF.rdbuf();
    std::vector<TopoDS_Shape> anArgs;
    std::string               aKey;
    if (!RunScript(aSS.str(), anArgs, aKey) || !aSeen.insert(aKey).second)
      continue;
    Record(aName, anArgs);
  }
}

// ------------------------------------------------------------------------------------------------ random primitives

static uint64_t gSeed = 88172645463325252ull;

static double Rand()
{
  gSeed ^= gSeed << 13;
  gSeed ^= gSeed >> 7;
  gSeed ^= gSeed << 17;
  return (double)(gSeed % 1000003ull) / 1000003.0;
}

static double Range(double theA, double theB) { return theA + (theB - theA) * Rand(); }

static TopoDS_Shape Primitive(int theKind)
{
  switch (theKind)
  {
    case 0: return BRepPrimAPI_MakeBox(gp_Pnt(-1, -1, -1), Range(1.5, 2.5), Range(1.5, 2.5), Range(1.5, 2.5)).Shape();
    case 1: return BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(0, 0, -1), gp_Dir(0, 0, 1)), Range(0.6, 1.2), Range(1.5, 2.5)).Shape();
    case 2: return BRepPrimAPI_MakeCone(gp_Ax2(gp_Pnt(0, 0, -1), gp_Dir(0, 0, 1)), Range(0.8, 1.4), Range(0.0, 0.5), Range(1.5, 2.5)).Shape();
    case 3: return BRepPrimAPI_MakeSphere(Range(0.8, 1.3)).Shape();
    default: return BRepPrimAPI_MakeTorus(Range(0.9, 1.3), Range(0.2, 0.45)).Shape();
  }
}

static TopoDS_Shape Placed(const TopoDS_Shape& theS, double theShift)
{
  gp_Trsf aRot;
  gp_Dir  anAxis(Range(-1, 1), Range(-1, 1), Range(-1, 1) + 0.01);
  aRot.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), anAxis), Range(0.0, 6.0));
  gp_Trsf aMove;
  aMove.SetTranslation(gp_Vec(Range(-theShift, theShift), Range(-theShift, theShift), Range(-theShift, theShift)));
  return theS.Moved(TopLoc_Location(aMove * aRot));
}

static void Primitives()
{
  const char* aNames[] = {"box", "cylinder", "cone", "sphere", "torus"};
  for (int aRound = 0; aRound < 2; aRound++)
    for (int i = 0; i < 5; i++)
      for (int j = i; j < 5; j++)
      {
        TopoDS_Shape a = Placed(Primitive(i), 0.3), b = Placed(Primitive(j), 0.6);
        Record(std::string("random/") + aNames[i] + "-" + aNames[j] + "-" + std::to_string(aRound), {a, b});
      }
}

int main(int argc, char** argv)
{
  if (argc < 3)
  {
    std::fprintf(stderr, "pfrec <tests/boolean directory> <case list>\n");
    return 1;
  }
  Scripts(argv[1], argv[2]);
  Primitives();
  return 0;
}
