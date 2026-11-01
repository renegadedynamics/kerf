// dsrec: records the legacy data structure (TopOpeBRepDS) that native OCCT's fillets and chamfers fill, as it is when
// ChFi3d_Builder::Compute hands it to TopOpeBRepBuild_HBuilder::Perform (unit 7L). The recording is the ground truth
// that units 7N1 and 7N2 compare their data structure with before Perform, and that 7R rebuilds the topology from.
//
// The hook: ChFi3d_Builder::Compute (TKFillet.dll) calls TopOpeBRepBuild_HBuilder::Perform(HDS) (TKBool.dll) through
// TKFillet's import table. dsrec patches that one import to a function that dumps the data structure and then calls
// the real Perform; nothing else in OCCT changes. Perform changes the data structure (its filters and reducers), so
// the dump is taken before; dsrec also dumps it after Build and records whether the two differ.
//
// The dump's text format is TopOpeBRepDS_TextDump's (src/OcctNet.ModelingAlgorithms/TopOpeBRepDS), which writes the
// port's data structure the same way, so the two texts can be compared byte for byte. Reals are written as printf's
// "%.17g", geometry with GeomTools::Write at precision 17. Built with MSVC against build123d.NET's native OCCT
// V8_0_1 (TKFillet and TopOpeBRepDS are not bound there).
#define WIN32_LEAN_AND_MEAN
#include <windows.h>

#include <cstdio>
#include <cstring>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <typeinfo>
#include <vector>

#include <BRepAlgoAPI_Cut.hxx>
#include <BRepBndLib.hxx>
#include <TopExp_Explorer.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <BRepOffsetAPI_ThruSections.hxx>
#include <BRep_Builder.hxx>
#include <Bnd_Box.hxx>
#include <Geom_Line.hxx>
#include <Geom_TrimmedCurve.hxx>
#include <TopoDS_Shell.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepFilletAPI_MakeChamfer.hxx>
#include <BRepFilletAPI_MakeFillet.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCone.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <BRepPrimAPI_MakeTorus.hxx>
#include <BRepTools.hxx>
#include <BRep_Tool.hxx>
#include <GeomTools.hxx>
#include <Geom_Curve.hxx>
#include <Geom_Surface.hxx>
#include <Geom2d_Curve.hxx>
#include <NCollection_IndexedDataMap.hxx>
#include <NCollection_IndexedMap.hxx>
#include <OSD.hxx>
#include <Standard_ErrorHandler.hxx>
#include <Standard_Failure.hxx>
#include <TopExp.hxx>
#include <TopTools_ShapeMapHasher.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Shape.hxx>
#include <TopOpeBRepBuild_HBuilder.hxx>
#include <TopOpeBRepDS_CurveExplorer.hxx>
#include <TopOpeBRepDS_CurvePointInterference.hxx>
#include <TopOpeBRepDS_DataStructure.hxx>
#include <TopOpeBRepDS_EdgeVertexInterference.hxx>
#include <TopOpeBRepDS_FaceEdgeInterference.hxx>
#include <TopOpeBRepDS_HDataStructure.hxx>
#include <TopOpeBRepDS_Interference.hxx>
#include <TopOpeBRepDS_PointExplorer.hxx>
#include <TopOpeBRepDS_ShapeShapeInterference.hxx>
#include <TopOpeBRepDS_SolidSurfaceInterference.hxx>
#include <TopOpeBRepDS_SurfaceCurveInterference.hxx>
#include <TopOpeBRepDS_SurfaceExplorer.hxx>
#include <gp_Ax1.hxx>
#include <gp_Trsf.hxx>

// ---------------------------------------------------------------------------------------------------------------------
// The text dump (TopOpeBRepDS_TextDump.Write in the port: keep the two in step)
// ---------------------------------------------------------------------------------------------------------------------

static std::string R(double v)
{
  char b[64];
  std::snprintf(b, sizeof(b), "%.17g", v);
  return b;
}

static const char* TypeName(TopAbs_ShapeEnum t)
{
  static const char* n[] = {"COMPOUND", "COMPSOLID", "SOLID", "SHELL", "FACE", "WIRE", "EDGE", "VERTEX", "SHAPE"};
  return n[(int)t];
}

static const char* OriName(TopAbs_Orientation o)
{
  static const char* n[] = {"FORWARD", "REVERSED", "INTERNAL", "EXTERNAL"};
  return n[(int)o];
}

static const char* StateName(TopAbs_State s)
{
  static const char* n[] = {"IN", "OUT", "ON", "UNKNOWN"};
  return n[(int)s];
}

static const char* KindName(TopOpeBRepDS_Kind k)
{
  static const char* n[] = {"POINT", "CURVE", "SURFACE", "VERTEX", "EDGE", "WIRE",
                            "FACE", "SHELL", "SOLID", "COMPSOLID", "COMPOUND", "UNKNOWN"};
  return n[(int)k];
}

static const char* ConfigName(TopOpeBRepDS_Config c)
{
  static const char* n[] = {"UNSH", "SAME", "DIFF"};
  return n[(int)c];
}

struct Dumper
{
  const TopOpeBRepDS_DataStructure& DS;
  const TopoDS_Shape&               Input;
  std::ostringstream                OS;
  std::map<const Standard_Transient*, int> Ids;
  std::vector<occ::handle<TopOpeBRepDS_Interference>> Order;
  std::map<int, bool> Written;
  NCollection_IndexedMap<TopoDS_Shape, TopTools_ShapeMapHasher> InMaps[9];

  Dumper(const TopOpeBRepDS_DataStructure& theDS, const TopoDS_Shape& theInput)
      : DS(theDS),
        Input(theInput)
  {
    OS.precision(17);
    for (int t = 0; t < 8; t++)
    {
      if (!Input.IsNull())
      {
        TopExp::MapShapes(Input, (TopAbs_ShapeEnum)t, InMaps[t]);
      }
    }
  }

  // "<TYPE> <ORI> in <k>" or "<TYPE> <ORI> new [x y z]"; "null" for a null shape
  std::string ShapeRef(const TopoDS_Shape& S)
  {
    if (S.IsNull())
    {
      return "null";
    }
    std::string s = std::string(TypeName(S.ShapeType())) + " " + OriName(S.Orientation());
    int k = (int)S.ShapeType() < 8 ? InMaps[(int)S.ShapeType()].FindIndex(S) : 0;
    if (k > 0)
    {
      return s + " in " + std::to_string(k);
    }
    s += " new";
    if (S.ShapeType() == TopAbs_VERTEX)
    {
      gp_Pnt P = BRep_Tool::Pnt(TopoDS::Vertex(S));
      s += " " + R(P.X()) + " " + R(P.Y()) + " " + R(P.Z());
    }
    return s;
  }

  void Collect(const NCollection_List<occ::handle<TopOpeBRepDS_Interference>>& L)
  {
    for (NCollection_List<occ::handle<TopOpeBRepDS_Interference>>::Iterator it(L); it.More(); it.Next())
    {
      Collect(it.Value());
    }
  }

  void Collect(const occ::handle<TopOpeBRepDS_Interference>& I)
  {
    if (I.IsNull() || Ids.count(I.get()) != 0)
    {
      return;
    }
    int id = (int)Order.size() + 1;
    Ids[I.get()] = id;
    Order.push_back(I);
  }

  std::string IdRef(const occ::handle<TopOpeBRepDS_Interference>& I)
  {
    return I.IsNull() ? "-" : "#" + std::to_string(Ids[I.get()]);
  }

  void Geom3d(const char* theTag, const occ::handle<Geom_Curve>& C)
  {
    if (C.IsNull())
    {
      OS << theTag << " null\n";
      return;
    }
    OS << theTag << "\n";
    GeomTools::Write(C, OS);
  }

  void Geom2d(const char* theTag, const occ::handle<Geom2d_Curve>& C)
  {
    if (C.IsNull())
    {
      OS << theTag << " null\n";
      return;
    }
    OS << theTag << "\n";
    GeomTools::Write(C, OS);
  }

  void GeomS(const char* theTag, const occ::handle<Geom_Surface>& S)
  {
    if (S.IsNull())
    {
      OS << theTag << " null\n";
      return;
    }
    OS << theTag << "\n";
    GeomTools::Write(S, OS);
  }

  // an interference: in full the first time, "R #id" afterwards
  void Interf(const occ::handle<TopOpeBRepDS_Interference>& I)
  {
    int id = Ids[I.get()];
    if (Written[id])
    {
      OS << "R #" << id << "\n";
      return;
    }
    Written[id] = true;
    const TopOpeBRepDS_Transition& T = I->Transition();
    occ::handle<Standard_Type> aType = I->DynamicType();
    const char* cls = "INTF";
    if (aType == STANDARD_TYPE(TopOpeBRepDS_CurvePointInterference))
      cls = "CPI";
    else if (aType == STANDARD_TYPE(TopOpeBRepDS_SurfaceCurveInterference))
      cls = "SCI";
    else if (aType == STANDARD_TYPE(TopOpeBRepDS_SolidSurfaceInterference))
      cls = "SSI";
    else if (aType == STANDARD_TYPE(TopOpeBRepDS_ShapeShapeInterference))
      cls = "SSHI";
    else if (aType == STANDARD_TYPE(TopOpeBRepDS_EdgeVertexInterference))
      cls = "EVI";
    else if (aType == STANDARD_TYPE(TopOpeBRepDS_FaceEdgeInterference))
      cls = "FEI";
    else if (aType != STANDARD_TYPE(TopOpeBRepDS_Interference))
      cls = "OTHER";
    OS << "I #" << id << " " << cls << " " << StateName(T.Before()) << "/" << StateName(T.After()) << " "
       << TypeName(T.ShapeBefore()) << "/" << TypeName(T.ShapeAfter()) << " " << T.IndexBefore() << "/"
       << T.IndexAfter() << " " << KindName(I->SupportType()) << " " << I->Support() << " "
       << KindName(I->GeometryType()) << " " << I->Geometry();
    if (aType == STANDARD_TYPE(TopOpeBRepDS_CurvePointInterference))
    {
      OS << " " << R(occ::down_cast<TopOpeBRepDS_CurvePointInterference>(I)->Parameter()) << "\n";
    }
    else if (aType == STANDARD_TYPE(TopOpeBRepDS_SurfaceCurveInterference))
    {
      OS << "\n";
      Geom2d("pcurve", occ::down_cast<TopOpeBRepDS_SurfaceCurveInterference>(I)->PCurve());
    }
    else if (aType == STANDARD_TYPE(TopOpeBRepDS_ShapeShapeInterference)
             || aType == STANDARD_TYPE(TopOpeBRepDS_FaceEdgeInterference))
    {
      occ::handle<TopOpeBRepDS_ShapeShapeInterference> S =
        occ::down_cast<TopOpeBRepDS_ShapeShapeInterference>(I);
      OS << " " << (S->GBound() ? 1 : 0) << " " << ConfigName(S->Config()) << "\n";
    }
    else if (aType == STANDARD_TYPE(TopOpeBRepDS_EdgeVertexInterference))
    {
      occ::handle<TopOpeBRepDS_EdgeVertexInterference> S =
        occ::down_cast<TopOpeBRepDS_EdgeVertexInterference>(I);
      OS << " " << (S->GBound() ? 1 : 0) << " " << ConfigName(S->Config()) << " " << R(S->Parameter()) << "\n";
    }
    else
    {
      OS << "\n";
    }
  }

  void List(const NCollection_List<occ::handle<TopOpeBRepDS_Interference>>& L)
  {
    OS << "interferences " << L.Extent() << "\n";
    for (NCollection_List<occ::handle<TopOpeBRepDS_Interference>>::Iterator it(L); it.More(); it.Next())
    {
      Interf(it.Value());
    }
  }

  std::string Run()
  {
    TopOpeBRepDS_DataStructure& ds = const_cast<TopOpeBRepDS_DataStructure&>(DS);
    TopOpeBRepDS_PointExplorer   pex(DS, false);
    TopOpeBRepDS_CurveExplorer   cex(DS, false);
    TopOpeBRepDS_SurfaceExplorer sex(DS, false);
    int nS = DS.NbShapes(), nP = DS.NbPoints(), nC = DS.NbCurves(), nF = DS.NbSurfaces();

    // the interferences' identifiers, in the order the dump meets them
    for (int i = 1; i <= nS; i++)
      Collect(DS.ShapeInterferences(i, false));
    for (int i = 1; i <= nP; i++)
      Collect(DS.PointInterferences(i));
    for (int i = 1; i <= nC; i++)
      Collect(DS.CurveInterferences(i));
    for (int i = 1; i <= nF; i++)
      Collect(DS.SurfaceInterferences(i));
    for (int i = 1; i <= nC; i++)
    {
      if (cex.IsCurve(i))
      {
        Collect(DS.Curve(i).GetSCI1());
        Collect(DS.Curve(i).GetSCI2());
      }
    }

    OS << "dsdump 1\n";
    OS << "shapes " << nS << "\n";
    for (int i = 1; i <= nS; i++)
    {
      const TopoDS_Shape& S = DS.Shape(i, false);
      const NCollection_List<TopoDS_Shape>& SD = DS.ShapeSameDomain(i);
      OS << "shape " << i << " " << ShapeRef(S) << " keep " << (DS.KeepShape(i) ? 1 : 0) << " sdref "
         << DS.SameDomainRef(i) << " sdori " << ConfigName(DS.SameDomainOri(i)) << " sdind " << DS.SameDomainInd(i)
         << " rank " << DS.AncestorRank(i) << " sd " << SD.Extent();
      for (NCollection_List<TopoDS_Shape>::Iterator it(SD); it.More(); it.Next())
      {
        OS << " " << DS.Shape(it.Value(), false);
      }
      OS << "\n";
      List(DS.ShapeInterferences(i, false));
    }

    OS << "points " << nP << "\n";
    for (int i = 1; i <= nP; i++)
    {
      if (!pex.IsPoint(i))
      {
        OS << "point " << i << " unbound\n";
        continue;
      }
      const TopOpeBRepDS_Point& P = DS.Point(i);
      OS << "point " << i << " " << R(P.Point().X()) << " " << R(P.Point().Y()) << " " << R(P.Point().Z()) << " "
         << R(P.Tolerance()) << " keep " << (P.Keep() ? 1 : 0) << "\n";
      List(DS.PointInterferences(i));
    }

    OS << "curves " << nC << "\n";
    for (int i = 1; i <= nC; i++)
    {
      if (!cex.IsCurve(i))
      {
        OS << "curve " << i << " unbound\n";
        continue;
      }
      const TopOpeBRepDS_Curve& C = DS.Curve(i);
      double f = 0., l = 0.;
      bool   r = C.Range(f, l);
      OS << "curve " << i << " " << R(C.Tolerance()) << " range " << (r ? 1 : 0) << " " << R(r ? f : 0.) << " "
         << R(r ? l : 0.) << " keep " << (C.Keep() ? 1 : 0) << " mother " << C.Mother() << " index "
         << C.DSIndex() << " walk " << (C.IsWalk() ? 1 : 0) << " sci " << IdRef(C.GetSCI1()) << " "
         << IdRef(C.GetSCI2()) << "\n";
      OS << "s1 " << ShapeRef(C.Shape1()) << "\n";
      OS << "s2 " << ShapeRef(C.Shape2()) << "\n";
      Geom3d("c3d", C.Curve());
      List(DS.CurveInterferences(i));
    }

    OS << "surfaces " << nF << "\n";
    for (int i = 1; i <= nF; i++)
    {
      if (!sex.IsSurface(i))
      {
        OS << "surface " << i << " unbound\n";
        continue;
      }
      const TopOpeBRepDS_Surface& F = DS.Surface(i);
      OS << "surface " << i << " " << R(F.Tolerance()) << " keep " << (F.Keep() ? 1 : 0) << "\n";
      GeomS("surf", F.Surface());
      List(DS.SurfaceInterferences(i));
    }

    // the interferences only the curves hold
    int nExtra = 0;
    for (size_t k = 0; k < Order.size(); k++)
      if (!Written[(int)k + 1])
        nExtra++;
    OS << "extra " << nExtra << "\n";
    for (size_t k = 0; k < Order.size(); k++)
      if (!Written[(int)k + 1])
        Interf(Order[k]);

    // the new surfaces of faces: the input's faces in order, then the data structure's other faces
    std::vector<TopoDS_Shape> aFaces;
    for (int k = 1; k <= InMaps[TopAbs_FACE].Extent(); k++)
      aFaces.push_back(InMaps[TopAbs_FACE](k));
    for (int i = 1; i <= nS; i++)
    {
      const TopoDS_Shape& S = DS.Shape(i, false);
      if (S.ShapeType() == TopAbs_FACE && InMaps[TopAbs_FACE].FindIndex(S) == 0)
        aFaces.push_back(S);
    }
    int nNew = 0;
    for (size_t k = 0; k < aFaces.size(); k++)
      if (DS.HasNewSurface(aFaces[k]))
        nNew++;
    OS << "newsurfaces " << nNew << "\n";
    for (size_t k = 0; k < aFaces.size(); k++)
    {
      if (DS.HasNewSurface(aFaces[k]))
      {
        OS << "newsurface " << ShapeRef(aFaces[k]) << "\n";
        GeomS("surf", DS.NewSurface(aFaces[k]));
      }
    }

    OS << "sectionedges " << DS.NbSectionEdges() << "\n";
    for (int i = 1; i <= DS.NbSectionEdges(); i++)
    {
      OS << "sectionedge " << ShapeRef(DS.SectionEdge(i, false)) << "\n";
    }
    OS << "isfafa " << (ds.Isfafa() ? 1 : 0) << "\n";
    OS << "end\n";
    return OS.str();
  }
};

std::string DumpDS(const TopOpeBRepDS_DataStructure& DS, const TopoDS_Shape& theInput) // kpart.cpp uses it too
{
  Dumper d(DS, theInput);
  return d.Run();
}

// ---------------------------------------------------------------------------------------------------------------------
// The hook on TopOpeBRepBuild_HBuilder::Perform
// ---------------------------------------------------------------------------------------------------------------------

static TopoDS_Shape gInput;
static int          gCalls = 0;
static std::string  gBefore;

static void HookPerform(TopOpeBRepBuild_HBuilder* theSelf, const occ::handle<TopOpeBRepDS_HDataStructure>& HDS)
{
  gCalls++;
  if (gCalls == 1)
  {
    gBefore = DumpDS(HDS->DS(), gInput);
  }
  theSelf->Perform(HDS); // through dsrec's own import: the real Perform
}

static bool PatchImport()
{
  HMODULE aFillet = GetModuleHandleA("TKFillet.dll");
  HMODULE aBool   = GetModuleHandleA("TKBool.dll");
  if (aFillet == nullptr || aBool == nullptr)
  {
    return false;
  }
  FARPROC aTarget = GetProcAddress(
    aBool,
    "?Perform@TopOpeBRepBuild_HBuilder@@QEAAXAEBV?$handle@VTopOpeBRepDS_HDataStructure@@@opencascade@@@Z");
  if (aTarget == nullptr)
  {
    return false;
  }
  BYTE*              aBase = (BYTE*)aFillet;
  IMAGE_DOS_HEADER*  aDos  = (IMAGE_DOS_HEADER*)aBase;
  IMAGE_NT_HEADERS*  aNt   = (IMAGE_NT_HEADERS*)(aBase + aDos->e_lfanew);
  IMAGE_DATA_DIRECTORY aDir = aNt->OptionalHeader.DataDirectory[IMAGE_DIRECTORY_ENTRY_IMPORT];
  int nPatched = 0;
  for (IMAGE_IMPORT_DESCRIPTOR* aDesc = (IMAGE_IMPORT_DESCRIPTOR*)(aBase + aDir.VirtualAddress); aDesc->Name != 0;
       aDesc++)
  {
    if (_stricmp((const char*)(aBase + aDesc->Name), "TKBool.dll") != 0)
    {
      continue;
    }
    for (IMAGE_THUNK_DATA* aThunk = (IMAGE_THUNK_DATA*)(aBase + aDesc->FirstThunk); aThunk->u1.Function != 0;
         aThunk++)
    {
      if ((FARPROC)aThunk->u1.Function == aTarget)
      {
        DWORD anOld = 0;
        VirtualProtect(&aThunk->u1.Function, sizeof(void*), PAGE_READWRITE, &anOld);
        aThunk->u1.Function = (ULONG_PTR)&HookPerform;
        VirtualProtect(&aThunk->u1.Function, sizeof(void*), anOld, &anOld);
        nPatched++;
      }
    }
  }
  return nPatched == 1;
}

// ---------------------------------------------------------------------------------------------------------------------
// The cases
// ---------------------------------------------------------------------------------------------------------------------

static std::ofstream gOut;

static TopoDS_Edge EdgeOf(const TopoDS_Shape& S, int k)
{
  NCollection_IndexedMap<TopoDS_Shape, TopTools_ShapeMapHasher> M;
  TopExp::MapShapes(S, TopAbs_EDGE, M);
  return TopoDS::Edge(M(k));
}

static TopoDS_Face FaceOf(const TopoDS_Shape& S, const TopoDS_Edge& E)
{
  NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>, TopTools_ShapeMapHasher> M;
  TopExp::MapShapesAndAncestors(S, TopAbs_EDGE, TopAbs_FACE, M);
  return TopoDS::Face(M.FindFromKey(E).First());
}

// the edges at the first vertex of S
static std::vector<int> EdgesAtVertex1(const TopoDS_Shape& S)
{
  NCollection_IndexedMap<TopoDS_Shape, TopTools_ShapeMapHasher> ME;
  TopExp::MapShapes(S, TopAbs_EDGE, ME);
  NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>, TopTools_ShapeMapHasher> M;
  TopExp::MapShapesAndAncestors(S, TopAbs_VERTEX, TopAbs_EDGE, M);
  std::vector<int> r;
  for (NCollection_List<TopoDS_Shape>::Iterator it(M(1)); it.More(); it.Next())
  {
    int k = ME.FindIndex(it.Value());
    bool seen = false;
    for (int j : r)
      seen = seen || j == k;
    if (!seen)
      r.push_back(k);
  }
  return r;
}

static TopoDS_Shape Rotated(const TopoDS_Shape& S, const gp_Ax1& A, double theDeg)
{
  gp_Trsf T;
  T.SetRotation(A, theDeg * M_PI / 180.);
  return S.Moved(TopLoc_Location(T));
}

static TopoDS_Shape Scaled(const TopoDS_Shape& S, double theFactor)
{
  gp_Trsf T;
  T.SetScale(gp_Pnt(0, 0, 0), theFactor);
  return BRepBuilderAPI_Transform(S, T, true).Shape();
}

struct Edge
{
  int    k;
  double r1;
  double r2;       // evolving fillet: r1 to r2; chamfer: distances (r2 < 0: one distance)
  int    mode = 0; // chamfer: 0 distances, 1 distance and angle (r2 in degrees) (unit 7N2)
};

// Unit 7N2: the input goes through BRep text first, as the port reads it, so both sides compute on the same shape
// (a rotated or scaled shape's location read back can differ from the in-memory one by an ulp).
static TopoDS_Shape ThroughText(const TopoDS_Shape& S)
{
  std::ostringstream anOut;
  BRepTools::Write(S, anOut);
  std::istringstream anIn(anOut.str());
  TopoDS_Shape       aRes;
  BRep_Builder       aB;
  BRepTools::Read(aRes, anIn, aB);
  return aRes;
}

static void Record(const char* theName, const TopoDS_Shape& S0, bool theChamfer, const std::vector<Edge>& theEdges)
{
  const TopoDS_Shape S = ThroughText(S0);
  gInput               = S;
  gCalls               = 0;
  gBefore.clear();
  std::string               anAfter, aStatus, aFaults;
  bool                      aDone = false;
  BRepFilletAPI_MakeChamfer aMkC(S);
  BRepFilletAPI_MakeFillet  aMkF(S);
  try
  {
    OCC_CATCH_SIGNALS
    occ::handle<TopOpeBRepBuild_HBuilder> aHB;
    if (theChamfer)
    {
      for (const Edge& e : theEdges)
      {
        TopoDS_Edge E = EdgeOf(S, e.k);
        if (e.mode == 1)
          aMkC.AddDA(e.r1, e.r2 * M_PI / 180., E, FaceOf(S, E));
        else if (e.r2 < 0)
          aMkC.Add(e.r1, E);
        else
          aMkC.Add(e.r1, e.r2, E, FaceOf(S, E));
      }
      aMkC.Build();
      aDone = aMkC.IsDone();
      aHB   = aMkC.Builder();
    }
    else
    {
      for (const Edge& e : theEdges)
      {
        TopoDS_Edge E = EdgeOf(S, e.k);
        if (e.r2 < 0)
          aMkF.Add(e.r1, E);
        else
          aMkF.Add(e.r1, e.r2, E);
      }
      aMkF.Build();
      aDone = aMkF.IsDone();
      aHB   = aMkF.Builder();
    }
    if (gCalls > 0 && !aHB.IsNull() && !aHB->DataStructure().IsNull())
    {
      anAfter = DumpDS(aHB->DataStructure()->DS(), gInput);
    }
  }
  catch (const Standard_Failure& e)
  {
    aStatus = std::string(" failure ") + typeid(e).name();
  }
  if (!theChamfer)
  {
    // the diagnostics build123d.NET reads (plan 2.6): faulty contours, stripe statuses, faulty vertices
    std::ostringstream aF;
    try
    {
      OCC_CATCH_SIGNALS
      aF << " contours " << aMkF.NbContours() << " faulty " << aMkF.NbFaultyContours();
      for (int i = 1; i <= aMkF.NbFaultyContours(); i++)
        aF << " " << aMkF.FaultyContour(i);
      aF << " statuses";
      for (int ic = 1; ic <= aMkF.NbContours(); ic++)
        aF << " " << (int)aMkF.StripeStatus(ic);
      aF << " faultyvertices " << aMkF.NbFaultyVertices() << " hasresult " << (aMkF.HasResult() ? 1 : 0);
    }
    catch (const Standard_Failure&)
    {
      aF << " diagnostics-failed";
    }
    aFaults = aF.str();
  }
  std::ostringstream aBRep;
  BRepTools::Write(S, aBRep);
  std::string aB = aBRep.str();
  if (!aB.empty() && aB.back() != '\n')
  {
    aB += '\n';
  }
  int nB = 0;
  for (char c : aB)
    nB += c == '\n';
  int nD = 0;
  for (char c : gBefore)
    nD += c == '\n';
  gOut << "case " << theName << "\n";
  // the edges and their parameters (k:r1:r2:mode), so that the port's tests need no edge selection of their own
  std::string anEdges = theChamfer ? " chamfer" : " fillet";
  for (const Edge& e : theEdges)
    anEdges += " " + std::to_string(e.k) + ":" + R(e.r1) + ":" + R(e.r2) + ":" + std::to_string(e.mode);
  gOut << "status done " << (aDone ? 1 : 0) << " perform " << gCalls << " unchanged "
       << (gCalls > 0 && anAfter == gBefore ? 1 : 0) << aFaults << (aStatus.empty() ? "" : " exception") << anEdges
       << "\n";
  gOut << "brep " << nB << "\n" << aB;
  gOut << "ds " << nD << "\n" << gBefore;
  gOut << "end-case\n";
  std::fprintf(stdout,
               "%s: done %d, %d lines, perform %d%s%s\n",
               theName,
               aDone ? 1 : 0,
               nD,
               gCalls,
               aFaults.c_str(),
               aStatus.c_str());
}

static TopoDS_Shape Translated(const TopoDS_Shape& S, double x, double y, double z)
{
  gp_Trsf T;
  T.SetTranslation(gp_Vec(x, y, z));
  return S.Moved(TopLoc_Location(T));
}

// build123d.NET's Box(x, y, z) and Cylinder(r, h): centred at the origin
static TopoDS_Shape CBox(double x, double y, double z)
{
  return Translated(BRepPrimAPI_MakeBox(x, y, z).Shape(), -x / 2, -y / 2, -z / 2);
}

static TopoDS_Shape CCyl(double r, double h)
{
  return Translated(BRepPrimAPI_MakeCylinder(r, h).Shape(), 0, 0, -h / 2);
}

static bool IsLine(const occ::handle<Geom_Curve>& C)
{
  occ::handle<Geom_Curve> B = C;
  occ::handle<Geom_TrimmedCurve> T = occ::down_cast<Geom_TrimmedCurve>(B);
  if (!T.IsNull())
    B = T->BasisCurve();
  return B->DynamicType() == STANDARD_TYPE(Geom_Line);
}

// the edges whose ends and middle are at Z = z (and, with theLinesOnly, that are lines)
static std::vector<int> EdgesAtZ(const TopoDS_Shape& S, double z, bool theLinesOnly = false)
{
  NCollection_IndexedMap<TopoDS_Shape, TopTools_ShapeMapHasher> M;
  TopExp::MapShapes(S, TopAbs_EDGE, M);
  std::vector<int> r;
  for (int k = 1; k <= M.Extent(); k++)
  {
    TopoDS_Edge E = TopoDS::Edge(M(k));
    if (BRep_Tool::Degenerated(E))
      continue;
    double                  f, l;
    occ::handle<Geom_Curve> C = BRep_Tool::Curve(E, f, l);
    if (C.IsNull())
      continue;
    if (std::abs(C->Value(f).Z() - z) < 1e-7 && std::abs(C->Value(l).Z() - z) < 1e-7
        && std::abs(C->Value(0.5 * (f + l)).Z() - z) < 1e-7)
    {
      if (theLinesOnly && !IsLine(C))
        continue;
      r.push_back(k);
    }
  }
  return r;
}

// the edges parallel to Z
static std::vector<int> VerticalEdges(const TopoDS_Shape& S)
{
  NCollection_IndexedMap<TopoDS_Shape, TopTools_ShapeMapHasher> M;
  TopExp::MapShapes(S, TopAbs_EDGE, M);
  std::vector<int> r;
  for (int k = 1; k <= M.Extent(); k++)
  {
    TopoDS_Edge E = TopoDS::Edge(M(k));
    if (BRep_Tool::Degenerated(E))
      continue;
    double                  f, l;
    occ::handle<Geom_Curve> C = BRep_Tool::Curve(E, f, l);
    gp_Pnt                  a = C->Value(f), b = C->Value(l);
    if (std::abs(a.X() - b.X()) < 1e-7 && std::abs(a.Y() - b.Y()) < 1e-7 && std::abs(a.Z() - b.Z()) > 1e-7)
      r.push_back(k);
  }
  return r;
}

static std::vector<Edge> All(const std::vector<int>& ks, double r1, double r2 = -1, int mode = 0)
{
  std::vector<Edge> r;
  for (int k : ks)
    r.push_back({k, r1, r2, mode});
  return r;
}

static std::vector<int> AllEdges(const TopoDS_Shape& S)
{
  NCollection_IndexedMap<TopoDS_Shape, TopTools_ShapeMapHasher> M;
  TopExp::MapShapes(S, TopAbs_EDGE, M);
  std::vector<int> r;
  for (int k = 1; k <= M.Extent(); k++)
    if (!BRep_Tool::Degenerated(TopoDS::Edge(M(k))))
      r.push_back(k);
  return r;
}

// the edges at the vertex at P
static std::vector<int> EdgesAtPoint(const TopoDS_Shape& S, const gp_Pnt& P)
{
  NCollection_IndexedMap<TopoDS_Shape, TopTools_ShapeMapHasher> ME;
  TopExp::MapShapes(S, TopAbs_EDGE, ME);
  NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>, TopTools_ShapeMapHasher> M;
  TopExp::MapShapesAndAncestors(S, TopAbs_VERTEX, TopAbs_EDGE, M);
  std::vector<int> r;
  for (int i = 1; i <= M.Extent(); i++)
  {
    if (BRep_Tool::Pnt(TopoDS::Vertex(M.FindKey(i))).Distance(P) > 1e-6)
      continue;
    for (NCollection_List<TopoDS_Shape>::Iterator it(M(i)); it.More(); it.Next())
    {
      int  k    = ME.FindIndex(it.Value());
      bool seen = false;
      for (int j : r)
        seen = seen || j == k;
      if (!seen && !BRep_Tool::Degenerated(TopoDS::Edge(it.Value())))
        r.push_back(k);
    }
  }
  return r;
}

// the edges whose ends and middle have X = x and Z = z
static std::vector<int> EdgesAtXZ(const TopoDS_Shape& S, double x, double z)
{
  NCollection_IndexedMap<TopoDS_Shape, TopTools_ShapeMapHasher> M;
  TopExp::MapShapes(S, TopAbs_EDGE, M);
  std::vector<int> r;
  for (int k = 1; k <= M.Extent(); k++)
  {
    double                  f, l;
    occ::handle<Geom_Curve> C = BRep_Tool::Curve(TopoDS::Edge(M(k)), f, l);
    if (C.IsNull())
      continue;
    gp_Pnt a = C->Value(f), b = C->Value(l);
    if (std::abs(a.X() - x) < 1e-7 && std::abs(b.X() - x) < 1e-7 && std::abs(a.Z() - z) < 1e-7
        && std::abs(b.Z() - z) < 1e-7)
      r.push_back(k);
  }
  return r;
}

int RunKPart(const char* thePath); // kpart.cpp

int main(int argc, char** argv)
{
  if (argc < 2)
  {
    std::fprintf(stderr, "usage: dsrec out.txt | dsrec --kpart out.txt\n");
    return 2;
  }
  OSD::SetSignal(false);
  if (argc >= 3 && std::strcmp(argv[1], "--kpart") == 0)
  {
    return RunKPart(argv[2]); // the analytic fillets and ChFi3d_Builder_0's functions (unit 7N1)
  }
  if (!PatchImport())
  {
    std::fprintf(stderr, "dsrec: could not hook TopOpeBRepBuild_HBuilder::Perform in TKFillet.dll\n");
    return 3;
  }
  gOut.open(argv[1], std::ios::binary);
  const gp_Ax1 aZ(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1));
  const gp_Ax1 aX(gp_Pnt(0, 0, 0), gp_Dir(1, 0, 0));

  TopoDS_Shape aBox = BRepPrimAPI_MakeBox(100, 100, 100).Shape();
  std::vector<int> aCorner = EdgesAtVertex1(aBox);

  // fillets on a box: one edge (blend/simple A1), two edges meeting at a corner, three, all twelve
  Record("box_edge", aBox, false, {{5, 10, -1}});
  Record("box_two_edges", aBox, false, {{aCorner[0], 10, -1}, {aCorner[1], 10, -1}});
  Record("box_corner", aBox, false, {{aCorner[0], 10, -1}, {aCorner[1], 10, -1}, {aCorner[2], 10, -1}});
  {
    std::vector<Edge> all;
    for (int k = 1; k <= 12; k++)
      all.push_back({k, 10, -1});
    Record("box_all_edges", aBox, false, all);
  }
  Record("box_edge_evolving", aBox, false, {{5, 5, 15}});

  // a cylinder's top edge
  {
    TopoDS_Shape aCyl = BRepPrimAPI_MakeCylinder(50, 100).Shape();
    NCollection_IndexedMap<TopoDS_Shape, TopTools_ShapeMapHasher> M;
    TopExp::MapShapes(aCyl, TopAbs_EDGE, M);
    int kTop = 0;
    for (int k = 1; k <= M.Extent() && kTop == 0; k++)
    {
      TopoDS_Edge E = TopoDS::Edge(M(k));
      if (!BRep_Tool::Degenerated(E) && !BRep_Tool::IsClosed(E, FaceOf(aCyl, E)))
      {
        double f, l;
        if (BRep_Tool::Curve(E, f, l)->Value(f).Z() > 50)
          kTop = k;
      }
    }
    Record("cylinder_top_edge", aCyl, false, {{kTop, 10, -1}});
  }

  // blend/simple B1, C1, C3 (D1), E1, F1 and K1
  Record("B1_partial_cylinder", Rotated(BRepPrimAPI_MakeCylinder(50, 100, 90 * M_PI / 180).Shape(), aZ, 90), false,
         {{9, 10, -1}, {6, 10, -1}, {5, 10, -1}});
  Record("C1_partial_cone", Rotated(BRepPrimAPI_MakeCone(90, 40, 150, 90 * M_PI / 180).Shape(), aZ, 270), false,
         {{3, 10, -1}});
  Record("D1_partial_cone_apex", Rotated(BRepPrimAPI_MakeCone(50, 0, 120, 90 * M_PI / 180).Shape(), aZ, 270),
         false, {{3, 10, -1}, {2, 10, -1}, {5, 10, -1}});
  Record("E1_partial_sphere",
         Scaled(Rotated(BRepPrimAPI_MakeSphere(15, -90 * M_PI / 180, 60 * M_PI / 180, 90 * M_PI / 180).Shape(),
                        aZ, 90),
                10),
         false, {{7, 10, -1}});
  Record("F1_partial_torus",
         Scaled(BRepPrimAPI_MakeTorus(20, 5, 0, 160 * M_PI / 180, 90 * M_PI / 180).Shape(), 10), false,
         {{4, 10, -1}, {1, 10, -1}, {5, 10, -1}});
  {
    TopoDS_Shape aB = BRepPrimAPI_MakeBox(10, 10, 10).Shape();
    TopoDS_Shape aC = BRepPrimAPI_MakeCylinder(3, 20).Shape();
    aC              = Rotated(aC, aX, 90);
    gp_Trsf aT;
    aT.SetTranslation(gp_Vec(5, 15, 5));
    aC = aC.Moved(TopLoc_Location(aT));
    TopoDS_Shape aCut = BRepAlgoAPI_Cut(aB, aC).Shape();
    Record("K1_box_cut_cylinder", Scaled(aCut, 10), false, {{5, 5, -1}});
  }

  // chamfers: one distance, two distances, a corner
  Record("chamfer_box_edge", aBox, true, {{5, 10, -1}});
  Record("chamfer_box_edge_two_distances", aBox, true, {{5, 10, 20}});
  Record("chamfer_box_corner", aBox, true, {{aCorner[0], 10, -1}, {aCorner[1], 10, -1}, {aCorner[2], 10, -1}});

  // unit 7N2: more of the blend and chamfer grids' features, and build123d.NET's fillet and chamfer tests
  {
    // a tangent chain: a box whose vertical edge is filleted, then the top edges around it (line-arc-line)
    BRepFilletAPI_MakeFillet aMk(aBox);
    aMk.Add(20, EdgeOf(aBox, VerticalEdges(aBox)[0]));
    aMk.Build();
    TopoDS_Shape aRounded = aMk.Shape();
    Record("tangent_chain_top", aRounded, false, All(EdgesAtZ(aRounded, 100), 5));
    Record("tangent_chain_chamfer", aRounded, true, All(EdgesAtZ(aRounded, 100), 5));
  }
  {
    // a four-edge corner: the apex of a square pyramid
    BRepBuilderAPI_MakePolygon aSq(gp_Pnt(0, 0, 0), gp_Pnt(100, 0, 0), gp_Pnt(100, 100, 0), gp_Pnt(0, 100, 0), true);
    BRepOffsetAPI_ThruSections aTS(true, true);
    aTS.AddWire(aSq.Wire());
    aTS.AddVertex(BRepBuilderAPI_MakeVertex(gp_Pnt(50, 50, 80)).Vertex());
    aTS.Build();
    TopoDS_Shape aPyr = aTS.Shape();
    Record("pyramid_apex_four_edges", aPyr, false, All(EdgesAtPoint(aPyr, gp_Pnt(50, 50, 80)), 5));
    Record("pyramid_base_edges", aPyr, false, All(EdgesAtZ(aPyr, 0), 5));
  }
  {
    // concave and convex: an L-shaped block (a box cut by a box), its inner edge, and with the top edges
    TopoDS_Shape aL = BRepAlgoAPI_Cut(BRepPrimAPI_MakeBox(100, 100, 100).Shape(),
                                      BRepPrimAPI_MakeBox(gp_Pnt(50, -10, 50), gp_Pnt(110, 110, 110)).Shape())
                        .Shape();
    std::vector<int> aIn = EdgesAtXZ(aL, 50, 50);
    Record("L_concave_edge", aL, false, All(aIn, 10));
    std::vector<int> aMix = aIn;
    for (int k : EdgesAtZ(aL, 100))
      aMix.push_back(k);
    Record("L_concave_and_top", aL, false, All(aMix, 10));
    Record("L_concave_chamfer", aL, true, All(aIn, 10));
  }
  Record("box_two_edges_evolving", aBox, false, {{aCorner[0], 5, 15}, {aCorner[1], 5, 15}});
  Record("chamfer_box_edge_distance_angle", aBox, true, {{5, 10, 30, 1}});
  Record("chamfer_box_two_edges", aBox, true, {{aCorner[0], 10, -1}, {aCorner[1], 10, -1}});
  Record("chamfer_box_two_distances_corner",
         aBox,
         true,
         {{aCorner[0], 10, 20}, {aCorner[1], 10, 20}, {aCorner[2], 10, 20}});
  {
    // a fillet on a shell: the box's faces but the top
    TopoDS_Shell aSh;
    BRep_Builder aB;
    aB.MakeShell(aSh);
    for (TopExp_Explorer ex(aBox, TopAbs_FACE); ex.More(); ex.Next())
    {
      Bnd_Box aBB;
      BRepBndLib::Add(ex.Current(), aBB);
      double x0, y0, z0, x1, y1, z1;
      aBB.Get(x0, y0, z0, x1, y1, z1);
      if (z0 > 99)
        continue;
      aB.Add(aSh, ex.Current());
    }
    Record("open_box_shell_vertical_edges", aSh, false, All(VerticalEdges(aSh), 10));
  }
  // build123d.NET's tests (tests/Build123d.Tests): its Box and Cylinder are centred
  {
    TopoDS_Shape b = CBox(10, 10, 10);
    Record("b3d_box_all_edges_r1", b, false, All(AllEdges(b), 1));
    Record("b3d_box_top_edges_r2", b, false, All(EdgesAtZ(b, 5), 2));
    Record("b3d_box_vertical_edges_r1", b, false, All(VerticalEdges(b), 1));
    Record("b3d_box_top_chamfer_d1", b, true, All(EdgesAtZ(b, 5), 1));
    Record("b3d_box_top_chamfer_two_distances", b, true, All(EdgesAtZ(b, 5), 0.1, 0.2));
    TopoDS_Shape plate = BRepAlgoAPI_Cut(CBox(20, 10, 4), CCyl(2, 10)).Shape();
    Record("b3d_plate_hole_top_lines_r1", plate, false, All(EdgesAtZ(plate, 2, true), 1));
    TopoDS_Shape cyl = CCyl(1, 2);
    Record("b3d_cylinder_bottom_r01", cyl, false, All(EdgesAtZ(cyl, -1), 0.1));
    // failures: build123d.NET turns them into ArgumentException from the stripe statuses (plan 2.6)
    TopoDS_Shape small = CBox(1, 1, 1);
    Record("b3d_fail_box_fillet_too_large", small, false, All(AllEdges(small), 5));
    Record("b3d_fail_box_chamfer_too_large", small, true, All(AllEdges(small), 5));
    TopoDS_Shape thin = CBox(20, 20, 2);
    Record("b3d_fail_thin_plate_fillet_r3", thin, false, All(AllEdges(thin), 3));
    Record("b3d_fail_box_top_edges_r6", b, false, All(EdgesAtZ(b, 5), 6));
  }
  return 0;
}
