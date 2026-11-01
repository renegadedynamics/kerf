// dsrec --kpart: records native OCCT's analytic fillets and chamfers (ChFiKPart_ComputeData::Compute and the three
// ComputeCorner, exported by TKFillet) on hand-made surface data and spines, and the free functions of
// ChFi3d_Builder_0 on simple inputs (unit 7N1).
//
// ChFi3d_Builder_0's functions are not exported by TKFillet.dll (but ChFi3d_FilCommonPoint and ChFi3d_PerformElSpine):
// build.ps1 compiles OCCT's own ChFi3d_Builder_0.cxx, unchanged, into dsrec, against the same native OCCT. What these
// cases call is therefore native OCCT's code.
//
// The port's side is tests/OcctNet.ModelingAlgorithms.Tests/ChFiKPart/KPartRecorder.cs, the same program in C#: keep
// the two in step. Reals are "%.17g", geometry GeomTools::Write at precision 17, the data structure dsrec's dump.
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <typeinfo>
#include <vector>

#include <Adaptor3d_CurveOnSurface.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCone.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepTools.hxx>
#include <BRep_Builder.hxx>
#include <BRep_Tool.hxx>
#include <ChFi3d.hxx>
#include <ChFi3d_Builder_0.hxx>
#include <ChFiDS_ChamfSpine.hxx>
#include <ChFiDS_ElSpine.hxx>
#include <ChFiDS_FilSpine.hxx>
#include <ChFiDS_Map.hxx>
#include <ChFiDS_SurfData.hxx>
#include <ChFiKPart_ComputeData.hxx>
#include <ElSLib.hxx>
#include <GeomAdaptor_Curve.hxx>
#include <GeomAdaptor_Surface.hxx>
#include <GeomFill_Boundary.hxx>
#include <GeomTools.hxx>
#include <Geom_BSplineCurve.hxx>
#include <Geom_BezierCurve.hxx>
#include <Geom_Circle.hxx>
#include <Geom_CylindricalSurface.hxx>
#include <Geom_Line.hxx>
#include <Geom_Plane.hxx>
#include <Geom_SphericalSurface.hxx>
#include <Geom_TrimmedCurve.hxx>
#include <Geom2d_Curve.hxx>
#include <Geom2d_Line.hxx>
#include <NCollection_Array1.hxx>
#include <NCollection_IndexedMap.hxx>
#include <Standard_ErrorHandler.hxx>
#include <Standard_Failure.hxx>
#include <TopExp.hxx>
#include <TopOpeBRepDS_DataStructure.hxx>
#include <TopTools_ShapeMapHasher.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Vertex.hxx>
#include <gp_Ax3.hxx>
#include <gp_Pln.hxx>

std::string DumpDS(const TopOpeBRepDS_DataStructure& DS, const TopoDS_Shape& theInput); // main.cpp

namespace
{
std::ostringstream gOS;

std::string R(double v)
{
  char b[64];
  std::snprintf(b, sizeof(b), "%.17g", v);
  return b;
}

const char* Ori(TopAbs_Orientation o)
{
  static const char* n[] = {"FORWARD", "REVERSED", "INTERNAL", "EXTERNAL"};
  return n[o];
}

void P3(const char* theTag, const gp_Pnt& P)
{
  gOS << theTag << " " << R(P.X()) << " " << R(P.Y()) << " " << R(P.Z()) << "\n";
}

void V3(const char* theTag, const gp_Vec& V)
{
  gOS << theTag << " " << R(V.X()) << " " << R(V.Y()) << " " << R(V.Z()) << "\n";
}

void P2(const char* theTag, const gp_Pnt2d& P)
{
  gOS << theTag << " " << R(P.X()) << " " << R(P.Y()) << "\n";
}

void Real(const char* theTag, double v)
{
  gOS << theTag << " " << R(v) << "\n";
}

void C2(const char* theTag, const occ::handle<Geom2d_Curve>& C)
{
  if (C.IsNull())
  {
    gOS << theTag << " null\n";
    return;
  }
  gOS << theTag << "\n";
  gOS.precision(17);
  GeomTools::Write(C, gOS);
}

void C3(const char* theTag, const occ::handle<Geom_Curve>& C)
{
  if (C.IsNull())
  {
    gOS << theTag << " null\n";
    return;
  }
  gOS << theTag << "\n";
  gOS.precision(17);
  GeomTools::Write(C, gOS);
}

void Cp(const char* theTag, const ChFiDS_CommonPoint& P)
{
  gOS << theTag << " vertex " << (P.IsVertex() ? 1 : 0) << " arc " << (P.IsOnArc() ? 1 : 0) << " tol "
      << R(P.Tolerance()) << " param " << R(P.Parameter()) << "\n";
  P3("  point", P.Point());
  if (P.HasVector())
  {
    V3("  vector", P.Vector());
  }
  if (P.IsOnArc())
  {
    gOS << "  onarc " << R(P.ParameterOnArc()) << " " << Ori(P.TransitionOnArc()) << "\n";
  }
}

void SurfData(const occ::handle<ChFiDS_SurfData>& D)
{
  gOS << "surf " << D->Surf() << " " << Ori(D->Orientation()) << "\n";
  for (int i = 1; i <= 2; i++)
  {
    const ChFiDS_FaceInterference& F = D->Interference(i);
    gOS << "fi" << i << " line " << F.LineIndex() << " " << Ori(F.Transition()) << " " << R(F.FirstParameter())
        << " " << R(F.LastParameter()) << "\n";
    C2("pcface", F.PCurveOnFace());
    C2("pcsurf", F.PCurveOnSurf());
  }
  Cp("vf1", D->VertexFirstOnS1());
  Cp("vl1", D->VertexLastOnS1());
  Cp("vf2", D->VertexFirstOnS2());
  Cp("vl2", D->VertexLastOnS2());
  gp_Pnt2d a, b, c, d;
  D->Get2dPoints(a, b, c, d);
  P2("p2d", a);
  P2("p2d", b);
  P2("p2d", c);
  P2("p2d", d);
}

TopoDS_Edge EdgeOf(const TopoDS_Shape& S, int k)
{
  NCollection_IndexedMap<TopoDS_Shape, TopTools_ShapeMapHasher> M;
  TopExp::MapShapes(S, TopAbs_EDGE, M);
  return TopoDS::Edge(M(k));
}

TopoDS_Face FaceOf(const TopoDS_Shape& S, int k)
{
  NCollection_IndexedMap<TopoDS_Shape, TopTools_ShapeMapHasher> M;
  TopExp::MapShapes(S, TopAbs_FACE, M);
  return TopoDS::Face(M(k));
}

// the two faces of an edge, in the order TopExp_Explorer meets them
void FacesOf(const TopoDS_Shape& S, const TopoDS_Edge& E, TopoDS_Face& F1, TopoDS_Face& F2)
{
  ChFiDS_Map M;
  M.Fill(S, TopAbs_EDGE, TopAbs_FACE);
  ChFi3d_conexfaces(E, F1, F2, M);
}

// the shape read back from its BRep text: the port's side reads the recorded text, so both compute on the same input
TopoDS_Shape RoundTrip(const TopoDS_Shape& S)
{
  std::ostringstream aB;
  BRepTools::Write(S, aB);
  std::istringstream aIn(aB.str());
  TopoDS_Shape       aRes;
  BRep_Builder       aBuilder;
  BRepTools::Read(aRes, aIn, aBuilder);
  return aRes;
}

void Begin(const char* theName, const TopoDS_Shape& S)
{
  gOS.str("");
  gOS.clear();
  gOS << "case " << theName << "\n";
  std::ostringstream aB;
  BRepTools::Write(S, aB);
  std::string s = aB.str();
  if (!s.empty() && s.back() != '\n')
  {
    s += '\n';
  }
  int n = 0;
  for (char c : s)
    n += c == '\n';
  gOS << "brep " << n << "\n" << s;
}

void End(std::ofstream& theOut)
{
  gOS << "end-case\n";
  theOut << gOS.str();
}

void DS(const TopOpeBRepDS_DataStructure& DStr)
{
  std::string s = DumpDS(DStr, TopoDS_Shape());
  int         n = 0;
  for (char c : s)
    n += c == '\n';
  gOS << "ds " << n << "\n" << s;
}

// --------------------------------------------------------------------------------------------------------------------
// ChFiKPart_ComputeData::Compute on the edge k of S, between its two faces (swapped when theSwap)
// kind: 0 fillet of radius a; 1 symmetric chamfer a; 2 two distances a, b; 3 distance a and angle b
// --------------------------------------------------------------------------------------------------------------------

void KPart(std::ofstream& theOut, const char* theName, const TopoDS_Shape& S, int k, int theKind, double a, double b,
           ChFiDS_ChamfMode theMode, bool theSwap)
{
  Begin(theName, S);
  try
  {
    OCC_CATCH_SIGNALS
    TopoDS_Edge E = EdgeOf(S, k);
    TopoDS_Face F1, F2;
    FacesOf(S, E, F1, F2);
    if (theSwap)
    {
      std::swap(F1, F2);
    }
    occ::handle<BRepAdaptor_Surface> HS1 = new BRepAdaptor_Surface(F1);
    occ::handle<BRepAdaptor_Surface> HS2 = new BRepAdaptor_Surface(F2);
    TopAbs_Orientation               Or1, Or2;
    int                              aChoix = ChFi3d::ConcaveSide(*HS1, *HS2, E, Or1, Or2);
    gOS << "concave " << aChoix << " " << Ori(Or1) << " " << Ori(Or2) << "\n";
    occ::handle<ChFiDS_Spine> Sp;
    if (theKind == 0)
    {
      occ::handle<ChFiDS_FilSpine> fs = new ChFiDS_FilSpine();
      fs->SetEdges(E);
      fs->Load();
      fs->SetRadius(a); // after Load: the radius is set at the spine's first and last abscissa
      Sp = fs;
    }
    else
    {
      occ::handle<ChFiDS_ChamfSpine> cs = new ChFiDS_ChamfSpine();
      cs->SetEdges(E);
      if (theKind == 1)
        cs->SetDist(a);
      else if (theKind == 2)
        cs->SetDists(a, b);
      else
        cs->SetDistAngle(a, b);
      cs->SetMode(theMode);
      Sp = cs;
    }
    Sp->Load();
    TopOpeBRepDS_DataStructure   DStr;
    occ::handle<ChFiDS_SurfData> Data = new ChFiDS_SurfData();
    bool ok = ChFiKPart_ComputeData::Compute(DStr, Data, HS1, HS2, Or1, Or2, Sp, 1);
    gOS << "ok " << (ok ? 1 : 0) << "\n";
    SurfData(Data);
    DS(DStr);
  }
  catch (const Standard_Failure& e)
  {
    gOS << "exception " << typeid(e).name() << "\n";
  }
  End(theOut);
}

// the toric corner: S1 a plane (z = 0), S2 a cylinder or a sphere, from P1S2 to P2S2
void CornerTorus(std::ofstream&                  theOut,
                 const char*                     theName,
                 const occ::handle<Geom_Surface>& S2,
                 const gp_Pnt2d&                 P1S1,
                 const gp_Pnt2d&                 P2S1,
                 const gp_Pnt2d&                 P1S2,
                 const gp_Pnt2d&                 P2S2,
                 TopAbs_Orientation              Or1,
                 TopAbs_Orientation              Or2)
{
  Begin(theName, TopoDS_Shape());
  try
  {
    OCC_CATCH_SIGNALS
    occ::handle<GeomAdaptor_Surface> HS1 = new GeomAdaptor_Surface(new Geom_Plane(gp_Pln()));
    occ::handle<GeomAdaptor_Surface> HS2 = new GeomAdaptor_Surface(S2);
    TopOpeBRepDS_DataStructure       DStr;
    occ::handle<ChFiDS_SurfData>     Data = new ChFiDS_SurfData();
    bool ok = ChFiKPart_ComputeData::ComputeCorner(DStr, Data, HS1, HS2, TopAbs_FORWARD, TopAbs_FORWARD, Or1, Or2,
                                                   5., 20., P1S1, P2S1, P1S2, P2S2);
    gOS << "ok " << (ok ? 1 : 0) << "\n";
    SurfData(Data);
    DS(DStr);
  }
  catch (const Standard_Failure& e)
  {
    gOS << "exception " << typeid(e).name() << "\n";
  }
  End(theOut);
}

// the spherical corner: S1 the plane z = 0 (its point PS1 the south pole), S2 a cylinder of radius 10
void CornerSphere(std::ofstream& theOut, const char* theName, double theRad, TopAbs_Orientation Or1)
{
  Begin(theName, TopoDS_Shape());
  try
  {
    OCC_CATCH_SIGNALS
    occ::handle<GeomAdaptor_Surface> HS1 = new GeomAdaptor_Surface(new Geom_Plane(gp_Pln()));
    occ::handle<GeomAdaptor_Surface> HS2 =
      new GeomAdaptor_Surface(new Geom_CylindricalSurface(gp_Ax3(), 10.));
    TopOpeBRepDS_DataStructure   DStr;
    occ::handle<ChFiDS_SurfData> Data = new ChFiDS_SurfData();
    bool ok = ChFiKPart_ComputeData::ComputeCorner(DStr, Data, HS1, HS2, TopAbs_FORWARD, TopAbs_FORWARD, Or1,
                                                   TopAbs_FORWARD, theRad, gp_Pnt2d(0., 0.), gp_Pnt2d(0., 10.),
                                                   gp_Pnt2d(M_PI / 2, 10.));
    gOS << "ok " << (ok ? 1 : 0) << "\n";
    SurfData(Data);
    DS(DStr);
  }
  catch (const Standard_Failure& e)
  {
    gOS << "exception " << typeid(e).name() << "\n";
  }
  End(theOut);
}

// the rotule on the corner of three faces of a box (S the face fs, S1 and S2 the faces f1 and f2)
void Rotule(std::ofstream&      theOut,
            const char*         theName,
            const TopoDS_Shape& S,
            int                 fs,
            int                 f1,
            int                 f2,
            TopAbs_Orientation  OS,
            TopAbs_Orientation  OS1,
            TopAbs_Orientation  OS2)
{
  Begin(theName, S);
  try
  {
    OCC_CATCH_SIGNALS
    TopoDS_Face                      F  = FaceOf(S, fs);
    occ::handle<BRepAdaptor_Surface> HS = new BRepAdaptor_Surface(F);
    occ::handle<BRepAdaptor_Surface> H1 = new BRepAdaptor_Surface(FaceOf(S, f1));
    occ::handle<BRepAdaptor_Surface> H2 = new BRepAdaptor_Surface(FaceOf(S, f2));
    TopOpeBRepDS_DataStructure       DStr;
    occ::handle<ChFiDS_SurfData>     Data = new ChFiDS_SurfData();
    bool ok = ChFiKPart_ComputeData::ComputeCorner(DStr, Data, HS, H1, H2, F.Orientation(), OS, OS1, OS2, 10.);
    gOS << "ok " << (ok ? 1 : 0) << "\n";
    SurfData(Data);
    DS(DStr);
  }
  catch (const Standard_Failure& e)
  {
    gOS << "exception " << typeid(e).name() << "\n";
  }
  End(theOut);
}

// --------------------------------------------------------------------------------------------------------------------
// ChFi3d_Builder_0's free functions
// --------------------------------------------------------------------------------------------------------------------

void Helpers(std::ofstream& theOut)
{
  const double aPi = M_PI;
  // ComputePCurv: the four isoparametric lines and the general segment, each reversed or not
  {
    Begin("ComputePCurv", TopoDS_Shape());
    const gp_Pnt2d aUV[5][2] = {{gp_Pnt2d(1, 2), gp_Pnt2d(1, 5)},
                                {gp_Pnt2d(1, 5), gp_Pnt2d(1, 2)},
                                {gp_Pnt2d(1, 2), gp_Pnt2d(4, 2)},
                                {gp_Pnt2d(4, 2), gp_Pnt2d(1, 2)},
                                {gp_Pnt2d(1, 2), gp_Pnt2d(3, 7)}};
    for (int i = 0; i < 5; i++)
    {
      for (int r = 0; r < 2; r++)
      {
        occ::handle<Geom2d_Curve> C;
        ChFi3d_ComputePCurv(aUV[i][0], aUV[i][1], C, 2., 5., r == 1);
        C2("pcurv", C);
      }
    }
    // with the same parameter on a cylinder along a helix-like line
    occ::handle<Geom_Surface> aCyl = new Geom_CylindricalSurface(gp_Ax3(), 3.);
    occ::handle<Geom_Curve>   aC3d = aCyl->VIso(1.);
    occ::handle<Geom2d_Curve> C;
    double                    tolr = 0.;
    ChFi3d_ComputePCurv(aC3d, gp_Pnt2d(0.5, 1.), gp_Pnt2d(2., 1.), C, aCyl, 0.5, 2., 1.e-6, tolr, false);
    C2("pcurv", C);
    Real("tolreached", tolr);
    End(theOut);
  }
  // BuildPCurve: the 2D cubic, on a cylinder with 2D and 3D tangents
  {
    Begin("BuildPCurve", TopoDS_Shape());
    gp_Dir2d d1(1, 1), d2(1, -1);
    C2("bezier", ChFi3d_BuildPCurve(gp_Pnt2d(0, 0), d1, gp_Pnt2d(3, 1), d2, true));
    gp_Dir2d e1(1, 1), e2(1, -1);
    C2("bezier", ChFi3d_BuildPCurve(gp_Pnt2d(0, 0), e1, gp_Pnt2d(3, 1), e2, false));
    occ::handle<GeomAdaptor_Surface> aCyl =
      new GeomAdaptor_Surface(new Geom_CylindricalSurface(gp_Ax3(), 3.));
    C2("bezier", ChFi3d_BuildPCurve(aCyl, gp_Pnt2d(0.2, 0), gp_Vec2d(1, 0.5), gp_Pnt2d(1.5, 2), gp_Vec2d(0.2, 1), true));
    C2("bezier", ChFi3d_BuildPCurve(aCyl, gp_Pnt2d(0.2, 0), gp_Vec(0, 1, 0.5), gp_Pnt2d(1.5, 2), gp_Vec(-1, 0, 1), true));
    occ::handle<Geom2d_Curve> aCurv;
    occ::handle<GeomFill_Boundary> aB = ChFi3d_mkbound(aCyl, aCurv, gp_Pnt2d(0.2, 0), gp_Pnt2d(1.5, 2), 1.e-4, 1.e-2);
    C2("mkbound", aCurv);
    for (int i = 0; i <= 4; i++)
    {
      P3("value", aB->Value(0.25 * i));
    }
    End(theOut);
  }
  // ComputeArete on a cylinder: an iso u, an iso v, and a general edge (Bézier and its 3D curve)
  {
    Begin("ComputeArete", TopoDS_Shape());
    occ::handle<Geom_Surface> aCyl = new Geom_CylindricalSurface(gp_Ax3(), 3.);
    ChFiDS_CommonPoint        P1, P2;
    const gp_Pnt2d            aUV[3][2] = {{gp_Pnt2d(1, 0.5), gp_Pnt2d(1, 4)},
                                           {gp_Pnt2d(2, 1), gp_Pnt2d(0.5, 1)},
                                           {gp_Pnt2d(0.5, 0.5), gp_Pnt2d(2, 3)}};
    for (int i = 0; i < 3; i++)
    {
      occ::handle<Geom_Curve>   C3d;
      occ::handle<Geom2d_Curve> Pc;
      double                    Pardeb = 0., Parfin = 0., tolr = 0.;
      ChFi3d_ComputeArete(P1, aUV[i][0], P2, aUV[i][1], aCyl, C3d, Pc, Pardeb, Parfin, 1.e-6, 1.e-7, tolr, 0);
      C3("c3d", C3d);
      C2("pcurv", Pc);
      gOS << "range " << R(Pardeb) << " " << R(Parfin) << " tol " << R(tolr) << "\n";
    }
    End(theOut);
  }
  // ComputeCurves: plane/plane, plane/cylinder (a line, a circle), cylinder/sphere (GeomInt)
  {
    Begin("ComputeCurves", TopoDS_Shape());
    occ::handle<GeomAdaptor_Surface> aPlnZ = new GeomAdaptor_Surface(new Geom_Plane(gp_Pln()));
    occ::handle<GeomAdaptor_Surface> aPlnX =
      new GeomAdaptor_Surface(new Geom_Plane(gp_Pln(gp_Pnt(1, 0, 0), gp_Dir(1, 0, 0))));
    occ::handle<GeomAdaptor_Surface> aCylZ =
      new GeomAdaptor_Surface(new Geom_CylindricalSurface(gp_Ax3(), 3.), 0., 2 * aPi, -10., 10.);
    occ::handle<GeomAdaptor_Surface> aPlnZ2 =
      new GeomAdaptor_Surface(new Geom_Plane(gp_Pln(gp_Pnt(0, 0, 2), gp_Dir(0, 0, 1))));
    occ::handle<GeomAdaptor_Surface> aPlnY3 =
      new GeomAdaptor_Surface(new Geom_Plane(gp_Pln(gp_Pnt(0, 3, 0), gp_Dir(0, 1, 0))));
    occ::handle<GeomAdaptor_Surface> aSph =
      new GeomAdaptor_Surface(new Geom_SphericalSurface(gp_Ax3(gp_Pnt(0, 0, 1), gp_Dir(0, 0, 1)), 4.));
    struct Pair
    {
      occ::handle<Adaptor3d_Surface> S1, S2;
      double                         d[4], f[4];
    };
    // the parameters are those of the curve's ends on each surface
    std::vector<Pair> aPairs;
    {
      // the planes z = 0 and x = 1 meet along x = 1, z = 0: from y = -2 to y = 3
      // (z = 0: u = x, v = y; x = 1: u, v in the plane's axes)
      Pair p;
      p.S1 = aPlnZ;
      p.S2 = aPlnX;
      double u, v;
      ElSLib::Parameters(aPlnX->Plane(), gp_Pnt(1, -2, 0), u, v);
      p.d[0] = 1;
      p.d[1] = -2;
      p.d[2] = u;
      p.d[3] = v;
      ElSLib::Parameters(aPlnX->Plane(), gp_Pnt(1, 3, 0), u, v);
      p.f[0] = 1;
      p.f[1] = 3;
      p.f[2] = u;
      p.f[3] = v;
      aPairs.push_back(p);
    }
    {
      // the plane z = 2 and the cylinder meet along a circle: from u = 0.5 to u = 2
      Pair p;
      p.S1 = aPlnZ2;
      p.S2 = aCylZ;
      double u, v;
      gp_Pnt a = ElSLib::Value(0.5, 2., aCylZ->Cylinder()), b = ElSLib::Value(2., 2., aCylZ->Cylinder());
      ElSLib::Parameters(aPlnZ2->Plane(), a, u, v);
      p.d[0] = u;
      p.d[1] = v;
      p.d[2] = 0.5;
      p.d[3] = 2.;
      ElSLib::Parameters(aPlnZ2->Plane(), b, u, v);
      p.f[0] = u;
      p.f[1] = v;
      p.f[2] = 2.;
      p.f[3] = 2.;
      aPairs.push_back(p);
    }
    {
      // the cylinder and the plane y = 3 touch along a line (u = pi/2): from v = -1 to v = 4
      Pair p;
      p.S1 = aCylZ;
      p.S2 = aPlnY3;
      double u, v;
      p.d[0] = aPi / 2;
      p.d[1] = -1.;
      ElSLib::Parameters(aPlnY3->Plane(), gp_Pnt(0, 3, -1), u, v);
      p.d[2] = u;
      p.d[3] = v;
      p.f[0] = aPi / 2;
      p.f[1] = 4.;
      ElSLib::Parameters(aPlnY3->Plane(), gp_Pnt(0, 3, 4), u, v);
      p.f[2] = u;
      p.f[3] = v;
      aPairs.push_back(p);
    }
    {
      // the cylinder (r 3) and the sphere (r 4, centre z = 1) meet on the circle z = 1 + sqrt(7)
      Pair   p;
      double z = 1. + std::sqrt(7.);
      p.S1     = aCylZ;
      p.S2     = aSph;
      double u, v;
      p.d[0] = 0.3;
      p.d[1] = z;
      ElSLib::Parameters(aSph->Sphere(), ElSLib::Value(0.3, z, aCylZ->Cylinder()), u, v);
      p.d[2] = u;
      p.d[3] = v;
      p.f[0] = 1.8;
      p.f[1] = z;
      ElSLib::Parameters(aSph->Sphere(), ElSLib::Value(1.8, z, aCylZ->Cylinder()), u, v);
      p.f[2] = u;
      p.f[3] = v;
      aPairs.push_back(p);
    }
    for (size_t i = 0; i < aPairs.size(); i++)
    {
      for (int whole = 0; whole < 2; whole++)
      {
        NCollection_Array1<double> d(1, 4), f(1, 4);
        for (int j = 0; j < 4; j++)
        {
          d(j + 1) = aPairs[i].d[j];
          f(j + 1) = aPairs[i].f[j];
        }
        occ::handle<Geom_Curve>   C3d;
        occ::handle<Geom2d_Curve> Pc1, Pc2;
        double                    tolr = 0.;
        try
        {
          OCC_CATCH_SIGNALS
          bool ok =
            ChFi3d_ComputeCurves(aPairs[i].S1, aPairs[i].S2, d, f, C3d, Pc1, Pc2, 1.e-6, 1.e-7, tolr, whole == 1);
          gOS << "ok " << (ok ? 1 : 0) << " tol " << R(tolr) << "\n";
          if (ok)
          {
            C3("c3d", C3d);
            C2("pc1", Pc1);
            C2("pc2", Pc2);
          }
        }
        catch (const Standard_Failure& e)
        {
          gOS << "exception " << typeid(e).name() << "\n";
        }
      }
    }
    End(theOut);
  }
  // IntCS, InterPlaneEdge, Parameters, TrimCurve, CircularSpine, Spine, tolerances, BoundSrf
  {
    Begin("Misc", TopoDS_Shape());
    occ::handle<GeomAdaptor_Surface> aCyl =
      new GeomAdaptor_Surface(new Geom_CylindricalSurface(gp_Ax3(), 3.), 0., 2 * aPi, -10., 10.);
    occ::handle<GeomAdaptor_Curve> aLin =
      new GeomAdaptor_Curve(new Geom_Line(gp_Pnt(-10, 1, 2), gp_Dir(1, 0, 0.1)), 0., 30.);
    for (double wc : {-2.e100, 2.e100, 15.})
    {
      gp_Pnt2d p2d;
      double   w = wc;
      bool     ok = ChFi3d_IntCS(aCyl, aLin, p2d, w);
      gOS << "intcs " << (ok ? 1 : 0) << " " << R(p2d.X()) << " " << R(p2d.Y()) << " " << R(w) << "\n";
    }
    occ::handle<GeomAdaptor_Surface> aPln =
      new GeomAdaptor_Surface(new Geom_Plane(gp_Pln(gp_Pnt(0, 0, 0), gp_Dir(1, 0, 0.2))));
    occ::handle<GeomAdaptor_Curve> aCir =
      new GeomAdaptor_Curve(new Geom_Circle(gp_Ax2(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)), 5.), 0., 2 * aPi);
    for (int s = 0; s < 2; s++)
    {
      double W  = -1.;
      bool   ok = ChFi3d_InterPlaneEdge(aPln, aCir, W, s == 1, 1.e-7);
      gOS << "interplane " << (ok ? 1 : 0) << " " << R(W) << "\n";
    }
    double u, v;
    ChFi3d_Parameters(new Geom_CylindricalSurface(gp_Ax3(), 3.), gp_Pnt(0, 3, 7), u, v);
    gOS << "parameters " << R(u) << " " << R(v) << "\n";
    ChFi3d_Parameters(new Geom_SphericalSurface(gp_Ax3(), 3.), gp_Pnt(0, 0, 3), u, v);
    gOS << "parameters " << R(u) << " " << R(v) << "\n";
    occ::handle<Geom_TrimmedCurve> aTr;
    ChFi3d_TrimCurve(new Geom_Circle(gp_Ax2(), 5.), gp_Pnt(0, 5, 0), gp_Pnt(-5, 0, 0), aTr);
    C3("trimmed", aTr);
    double                   wf = -1., wl = -1.;
    occ::handle<Geom_Circle> aCs =
      ChFi3d_CircularSpine(wf, wl, gp_Pnt(5, 0, 0), gp_Vec(0, 1, 0), gp_Pnt(0, 5, 0), gp_Vec(-1, 0, 0), 2.);
    C3("circularspine", aCs);
    gOS << "w " << R(wf) << " " << R(wl) << "\n";
    gp_Vec vd(0, 1, 0), vf(-1, 0, 0);
    C3("spine", ChFi3d_Spine(gp_Pnt(5, 0, 0), vd, gp_Pnt(0, 5, 0), vf, 2.));
    V3("vd", vd);
    V3("vf", vf);
    Real("convtol", ChFi3d_ConvTol2dToTol3d(aCyl, 1.e-5));
    occ::handle<Geom_Curve> aC3 = new Geom_TrimmedCurve(new Geom_Circle(gp_Ax2(gp_Pnt(0, 0, 1), gp_Dir(0, 0, 1)), 3.), 0., 2.);
    occ::handle<Geom2d_Curve> aPc1 = new Geom2d_Line(gp_Pnt2d(0., 1.), gp_Dir2d(1., 0.));
    occ::handle<GeomAdaptor_Surface> aSph =
      new GeomAdaptor_Surface(new Geom_SphericalSurface(gp_Ax3(), 3.2));
    occ::handle<Geom2d_Curve> aPc2 = new Geom2d_Line(gp_Pnt2d(0., 0.3), gp_Dir2d(1., 0.));
    Real("evaltol", ChFi3d_EvalTolReached(aCyl, aPc1, aSph, aPc2, aC3));
    GeomAdaptor_Surface aB(new Geom_CylindricalSurface(gp_Ax3(), 3.));
    ChFi3d_BoundSrf(aB, 0.5, 1., 0., 2., true);
    gOS << "boundsrf " << R(aB.FirstUParameter()) << " " << R(aB.LastUParameter()) << " "
        << R(aB.FirstVParameter()) << " " << R(aB.LastVParameter()) << "\n";
    GeomAdaptor_Surface aP(new Geom_Plane(gp_Pln()));
    ChFi3d_BoundSrf(aP, 0., 1., 0., 4., true);
    gOS << "boundsrf " << R(aP.FirstUParameter()) << " " << R(aP.LastUParameter()) << " "
        << R(aP.FirstVParameter()) << " " << R(aP.LastVParameter()) << "\n";
    occ::handle<Geom_Curve> aArc = new Geom_TrimmedCurve(new Geom_Circle(gp_Ax2(), 5.), 0., 2.);
    C3("approxbyc2", ChFi3d_ApproxByC2(aArc));
    gOS << "issmooth " << (ChFi3d_IsSmooth(aArc) ? 1 : 0) << " " << (ChFi3d_IsSmooth(ChFi3d_ApproxByC2(aArc)) ? 1 : 0)
        << "\n";
    End(theOut);
  }
}

// the topology helpers on a box: the edges at its first vertex, the faces of its edges
void Topology(std::ofstream& theOut, const TopoDS_Shape& S)
{
  Begin("Topology", S);
  ChFiDS_Map EF, VE, VF;
  EF.Fill(S, TopAbs_EDGE, TopAbs_FACE);
  VE.Fill(S, TopAbs_VERTEX, TopAbs_EDGE);
  VF.Fill(S, TopAbs_VERTEX, TopAbs_FACE);
  NCollection_IndexedMap<TopoDS_Shape, TopTools_ShapeMapHasher> MV;
  TopExp::MapShapes(S, TopAbs_VERTEX, MV);
  for (int i = 1; i <= MV.Extent(); i++)
  {
    TopoDS_Vertex V = TopoDS::Vertex(MV(i));
    gOS << "vertex " << i << " nbface " << ChFi3d_nbface(VF(V)) << " nbedges " << ChFi3d_NumberOfEdges(V, VE)
        << " nbsharp " << ChFi3d_NumberOfSharpEdges(V, VE, EF) << "\n";
    TopoDS_Edge E[3];
    int         n = 0;
    for (NCollection_List<TopoDS_Shape>::Iterator it(VE(V)); it.More() && n < 3; it.Next())
    {
      bool seen = false;
      for (int j = 0; j < n; j++)
        seen = seen || E[j].IsSame(it.Value());
      if (!seen)
        E[n++] = TopoDS::Edge(it.Value());
    }
    if (n == 3)
    {
      gOS << "edgestate " << (int)ChFi3d_EdgeState(E, EF) << " angle " << R(ChFi3d_AngleEdge(V, E[0], E[1]))
          << "\n";
    }
  }
  NCollection_IndexedMap<TopoDS_Shape, TopTools_ShapeMapHasher> ME;
  TopExp::MapShapes(S, TopAbs_EDGE, ME);
  for (int i = 1; i <= ME.Extent(); i++)
  {
    TopoDS_Edge E = TopoDS::Edge(ME(i));
    TopoDS_Face F1, F2;
    ChFi3d_conexfaces(E, F1, F2, EF);
    gOS << "edge " << i << " conti " << (int)ChFi3d_evalconti(E, F1, F2) << " pseudoseam "
        << (ChFi3d_IsPseudoSeam(E, F1) ? 1 : 0) << "\n";
  }
  End(theOut);
}

// PerformElSpine on the top circle of a cylinder (an arc of it) and on a box edge
void ElSpine(std::ofstream& theOut, const char* theName, const TopoDS_Shape& S, int k, double WF, double WL)
{
  Begin(theName, S);
  try
  {
    OCC_CATCH_SIGNALS
    occ::handle<ChFiDS_FilSpine> fs = new ChFiDS_FilSpine();
    fs->SetEdges(EdgeOf(S, k));
    fs->Load();
    fs->SetRadius(1.);
    occ::handle<ChFiDS_Spine>   Sp  = fs;
    occ::handle<ChFiDS_ElSpine> HES = new ChFiDS_ElSpine();
    HES->FirstParameter(WF);
    HES->LastParameter(WL);
    gp_Pnt P;
    gp_Vec V;
    Sp->D1(WF, P, V);
    HES->SetFirstPointAndTgt(P, V);
    Sp->D1(WL, P, V);
    HES->SetLastPointAndTgt(P, V);
    ChFi3d_PerformElSpine(HES, Sp, GeomAbs_C1, 1.e-4, false);
    gOS << "range " << R(HES->FirstParameter()) << " " << R(HES->LastParameter()) << " type " << (int)HES->GetType()
        << "\n";
    for (int i = 0; i <= 4; i++)
    {
      P3("value", HES->Value(HES->FirstParameter() + 0.25 * i * (HES->LastParameter() - HES->FirstParameter())));
    }
    if (HES->GetType() == GeomAbs_BSplineCurve)
    {
      C3("curve", HES->BSpline());
    }
  }
  catch (const Standard_Failure& e)
  {
    gOS << "exception " << typeid(e).name() << "\n";
  }
  End(theOut);
}
} // namespace

int RunKPart(const char* thePath)
{
  std::ofstream aOut(thePath, std::ios::binary);
  TopoDS_Shape  aBox = RoundTrip(BRepPrimAPI_MakeBox(100, 100, 100).Shape());
  // a quarter cylinder: its lateral face meets the planes along lines and circles
  TopoDS_Shape aCylQ = RoundTrip(BRepPrimAPI_MakeCylinder(50, 100, M_PI / 2).Shape());
  TopoDS_Shape aCyl  = RoundTrip(BRepPrimAPI_MakeCylinder(50, 100).Shape());
  TopoDS_Shape aCone = RoundTrip(BRepPrimAPI_MakeCone(90, 40, 150).Shape());
  // the edges: on the box, 1 to 12; on the quarter cylinder and the cone, as MapShapes numbers them
  NCollection_IndexedMap<TopoDS_Shape, TopTools_ShapeMapHasher> M;
  std::vector<int> aCylQLines, aCylQCircles, aCylCircles, aConeCircles, aCylSeam;
  TopExp::MapShapes(aCylQ, TopAbs_EDGE, M);
  for (int k = 1; k <= M.Extent(); k++)
  {
    BRepAdaptor_Curve C(TopoDS::Edge(M(k)));
    TopoDS_Face       F1, F2;
    FacesOf(aCylQ, TopoDS::Edge(M(k)), F1, F2);
    bool aCylFace = BRepAdaptor_Surface(F1).GetType() == GeomAbs_Cylinder
                    || (!F2.IsNull() && BRepAdaptor_Surface(F2).GetType() == GeomAbs_Cylinder);
    if (!aCylFace || F2.IsNull())
      continue;
    if (C.GetType() == GeomAbs_Line)
      aCylQLines.push_back(k);
    else if (C.GetType() == GeomAbs_Circle)
      aCylQCircles.push_back(k);
  }
  M.Clear();
  TopExp::MapShapes(aCyl, TopAbs_EDGE, M);
  for (int k = 1; k <= M.Extent(); k++)
  {
    if (BRepAdaptor_Curve(TopoDS::Edge(M(k))).GetType() == GeomAbs_Circle)
      aCylCircles.push_back(k);
    else if (BRepAdaptor_Curve(TopoDS::Edge(M(k))).GetType() == GeomAbs_Line)
      aCylSeam.push_back(k);
  }
  M.Clear();
  TopExp::MapShapes(aCone, TopAbs_EDGE, M);
  for (int k = 1; k <= M.Extent(); k++)
  {
    if (BRepAdaptor_Curve(TopoDS::Edge(M(k))).GetType() == GeomAbs_Circle)
      aConeCircles.push_back(k);
  }

  const ChFiDS_ChamfMode C0 = ChFiDS_ClassicChamfer, C1 = ChFiDS_ConstThroatChamfer,
                         C2m = ChFiDS_ConstThroatWithPenetrationChamfer;
  for (int sw = 0; sw < 2; sw++)
  {
    std::string s = sw ? "_swapped" : "";
    KPart(aOut, ("fil_pln_pln" + s).c_str(), aBox, 5, 0, 10., 0., C0, sw);
    KPart(aOut, ("fil_pln_cyl_line" + s).c_str(), aCylQ, aCylQLines[0], 0, 10., 0., C0, sw);
    KPart(aOut, ("fil_pln_cyl_circle" + s).c_str(), aCylQ, aCylQCircles[0], 0, 10., 0., C0, sw);
    KPart(aOut, ("fil_pln_cyl_full" + s).c_str(), aCyl, aCylCircles[0], 0, 10., 0., C0, sw);
    KPart(aOut, ("fil_pln_cyl_sphere" + s).c_str(), aCyl, aCylCircles[0], 0, 50., 0., C0, sw);
    KPart(aOut, ("fil_pln_con_bottom" + s).c_str(), aCone, aConeCircles[0], 0, 10., 0., C0, sw);
    KPart(aOut, ("fil_pln_con_top" + s).c_str(), aCone, aConeCircles[1], 0, 10., 0., C0, sw);
    KPart(aOut, ("ch_pln_pln" + s).c_str(), aBox, 5, 1, 10., 0., C0, sw);
    KPart(aOut, ("ch_pln_pln_two" + s).c_str(), aBox, 5, 2, 10., 20., C0, sw);
    KPart(aOut, ("ch_pln_pln_angle" + s).c_str(), aBox, 5, 3, 10., 0.6, C0, sw);
    KPart(aOut, ("ch_pln_pln_throat" + s).c_str(), aBox, 5, 1, 10., 0., C1, sw);
    KPart(aOut, ("ch_pln_pln_penetration" + s).c_str(), aBox, 5, 2, 10., 15., C2m, sw);
    KPart(aOut, ("ch_pln_cyl_line" + s).c_str(), aCylQ, aCylQLines[0], 1, 10., 0., C0, sw);
    KPart(aOut, ("ch_pln_cyl_line_two" + s).c_str(), aCylQ, aCylQLines[0], 2, 10., 20., C0, sw);
    KPart(aOut, ("ch_pln_cyl_line_angle" + s).c_str(), aCylQ, aCylQLines[0], 3, 10., 0.6, C0, sw);
    KPart(aOut, ("ch_pln_cyl_circle" + s).c_str(), aCyl, aCylCircles[0], 1, 10., 0., C0, sw);
    KPart(aOut, ("ch_pln_cyl_circle_two" + s).c_str(), aCyl, aCylCircles[0], 2, 10., 20., C0, sw);
    KPart(aOut, ("ch_pln_cyl_circle_angle" + s).c_str(), aCyl, aCylCircles[0], 3, 10., 0.6, C0, sw);
    KPart(aOut, ("ch_pln_cyl_circle_throat" + s).c_str(), aCyl, aCylCircles[0], 1, 10., 0., C1, sw);
    KPart(aOut, ("ch_pln_cyl_circle_penetration" + s).c_str(), aCyl, aCylCircles[0], 2, 10., 15., C2m, sw);
    KPart(aOut, ("ch_pln_con" + s).c_str(), aCone, aConeCircles[0], 1, 10., 0., C0, sw);
    KPart(aOut, ("ch_pln_con_two" + s).c_str(), aCone, aConeCircles[1], 2, 10., 20., C0, sw);
    KPart(aOut, ("ch_pln_con_angle" + s).c_str(), aCone, aConeCircles[0], 3, 10., 0.6, C0, sw);
    KPart(aOut, ("ch_pln_con_angle_top" + s).c_str(), aCone, aConeCircles[1], 3, 10., 0.6, C0, sw);
    KPart(aOut, ("ch_pln_con_throat" + s).c_str(), aCone, aConeCircles[0], 1, 10., 0., C1, sw);
  }
  // a case that is not particular (the seam, between the cylinder and itself): Compute throws
  KPart(aOut, "fil_cyl_cyl_not_implemented", aCyl, aCylSeam[0], 0, 10., 0., C0, false);

  CornerTorus(aOut,
              "corner_torus_cylinder",
              new Geom_CylindricalSurface(gp_Ax3(), 20.),
              gp_Pnt2d(15, 0),
              gp_Pnt2d(0, 15),
              gp_Pnt2d(0, 0),
              gp_Pnt2d(M_PI / 2, 0),
              TopAbs_FORWARD,
              TopAbs_REVERSED);
  CornerTorus(aOut,
              "corner_torus_sphere",
              new Geom_SphericalSurface(gp_Ax3(), 20.),
              gp_Pnt2d(15, 0),
              gp_Pnt2d(0, 15),
              gp_Pnt2d(0, 0),
              gp_Pnt2d(M_PI / 2, 0),
              TopAbs_FORWARD,
              TopAbs_REVERSED);
  CornerSphere(aOut, "corner_sphere", 10., TopAbs_FORWARD);
  CornerSphere(aOut, "corner_sphere_reversed", 10., TopAbs_REVERSED);
  Rotule(aOut, "rotule", aBox, 1, 3, 5, TopAbs_FORWARD, TopAbs_REVERSED, TopAbs_REVERSED);
  Rotule(aOut, "rotule_reversed", aBox, 1, 3, 5, TopAbs_REVERSED, TopAbs_FORWARD, TopAbs_REVERSED);

  Helpers(aOut);
  Topology(aOut, aBox);
  ElSpine(aOut, "elspine_circle", aCyl, aCylCircles[0], -5., 2 * M_PI * 50 + 5.);
  ElSpine(aOut, "elspine_arc", aCylQ, aCylQCircles[0], 10., 60.);
  ElSpine(aOut, "elspine_line_extended", aBox, 5, -10., 110.);
  return 0;
}
