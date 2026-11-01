// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_HDataStructure.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_HDataStructure.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;
using static Occt.TopAbs_State;
using static Occt.TopOpeBRepDS_Config;
using static Occt.TopOpeBRepDS_Kind;

namespace Occt;

/// <summary>
/// The legacy data structure held by handle, with the queries and storing functions its builders use: iterators on
/// a face's curves, a curve's points, an edge's points, a solid's surfaces; storing an interference on a shape and on
/// its geometry at once.
/// </summary>
/// <remarks>
/// A working object, internal to the fillet builders and unit 7R's reconstruction: <see cref="DS"/> is public for
/// reading, <see cref="ChangeDS"/> and the storing functions internal. <c>ChkIntg</c> (TopOpeBRepDS_Check, a builder
/// tool) is not ported.
/// </remarks>
public sealed class TopOpeBRepDS_HDataStructure : Standard_Transient
{
    private readonly TopOpeBRepDS_DataStructure myDS = new();

    // the geometry and support kinds and indices of I (left unchanged if I is null)
    private static void FUN_HDS_data(TopOpeBRepDS_Interference? I,
                                     ref TopOpeBRepDS_Kind GT1,
                                     ref int G1,
                                     ref TopOpeBRepDS_Kind ST1,
                                     ref int S1)
    {
        if (I is null)
        {
            return;
        }
        GT1 = I.GeometryType();
        G1 = I.Geometry();
        ST1 = I.SupportType();
        S1 = I.Support();
    }

    /// <summary>Whether the face <paramref name="F1"/> has a face/edge/face interference with the face <paramref name="F2"/>.</summary>
    internal static bool FUN_HDS_FACESINTERFER(in TopoDS_Shape F1, in TopoDS_Shape F2, TopOpeBRepDS_HDataStructure HDS)
    {
        bool yainterf = false;
        TopOpeBRepDS_DataStructure DS = HDS.DS();
        bool ya = DS.HasShape(F1);
        if (!ya)
        {
            return false;
        }

        //                         DS.Shape(F1);
        int iF2 = DS.Shape(F2);

        NCollection_List<TopOpeBRepDS_Interference> L1 = DS.ShapeInterferences(F1);
        NCollection_List<TopOpeBRepDS_Interference>.Iterator itL1 = new(L1);
        for (; itL1.More(); itL1.Next())
        {
            TopOpeBRepDS_Interference I = itL1.Value();
            TopOpeBRepDS_Kind GT = TopOpeBRepDS_UNKNOWN, ST = TopOpeBRepDS_UNKNOWN;
            int G = 0, S = 0; // OCCT leaves G unset (it is set from I, never null here)
            FUN_HDS_data(I, ref GT, ref G, ref ST, ref S);
            // interference face1/edge/face2
            bool fef = true;
            fef = fef && (GT == TopOpeBRepDS_EDGE);
            fef = fef && (ST == TopOpeBRepDS_FACE);
            fef = fef && (S == iF2);
            if (fef)
            {
                yainterf = true;
                break;
            }
        }
        return yainterf;
    }

    /// <summary>An empty data structure.</summary>
    public TopOpeBRepDS_HDataStructure()
    {
    }

    /// <summary>Adds to the data structure the wires, faces, shells and solids of <paramref name="S"/> that contain a shape of it.</summary>
    internal void AddAncestors(in TopoDS_Shape S)
    {
        AddAncestors(S, TopAbs_WIRE, TopAbs_EDGE);
        AddAncestors(S, TopAbs_FACE, TopAbs_WIRE);
        AddAncestors(S, TopAbs_SHELL, TopAbs_FACE);
        AddAncestors(S, TopAbs_SOLID, TopAbs_SHELL);
    }

    /// <summary>
    /// Adds the shapes of type <paramref name="T1"/> of <paramref name="S"/> containing a sub-shape of type
    /// <paramref name="T2"/> that is in the data structure, with <paramref name="S"/>'s ancestor rank.
    /// </summary>
    internal void AddAncestors(in TopoDS_Shape S, TopAbs_ShapeEnum T1, TopAbs_ShapeEnum T2)
    {
        TopOpeBRepDS_DataStructure BDS = ChangeDS();
        int rankS = myDS.AncestorRank(S);

        // find the shapes of type T1 containing HasShape() of type T2
        for (TopOpeBRepTool_ShapeExplorer eT1 = new(S, T1); eT1.More(); eT1.Next())
        {
            TopoDS_Shape ST1 = eT1.Current();
            for (TopOpeBRepTool_ShapeExplorer eT2 = new(ST1, T2); eT2.More(); eT2.Next())
            {
                TopoDS_Shape ST2 = eT2.Current();
                if (BDS.HasShape(ST2))
                {
                    BDS.AddShape(ST1, rankS);
                    break;
                }
            }
        }
    }

    // ChkIntg: not ported (TopOpeBRepDS_Check, a tool of the legacy builder; plan 2.1, decision 992)

    /// <summary>The data structure (its mutators are internal).</summary>
    public TopOpeBRepDS_DataStructure DS() => myDS;

    internal TopOpeBRepDS_DataStructure ChangeDS() => myDS;

    /// <summary>The number of surfaces.</summary>
    public int NbSurfaces() => myDS.NbSurfaces();

    /// <summary>The number of curves.</summary>
    public int NbCurves() => myDS.NbCurves();

    /// <summary>The number of points.</summary>
    public int NbPoints() => myDS.NbPoints();

    /// <summary>The surface <paramref name="I"/>.</summary>
    public TopOpeBRepDS_Surface Surface(int I) => myDS.Surface(I);

    /// <summary>An iterator on the curves on the surface <paramref name="I"/>.</summary>
    public TopOpeBRepDS_CurveIterator SurfaceCurves(int I)
    {
        NCollection_List<TopOpeBRepDS_Interference> L = myDS.SurfaceInterferences(I);
        return new TopOpeBRepDS_CurveIterator(L);
    }

    /// <summary>The curve <paramref name="I"/>.</summary>
    public TopOpeBRepDS_Curve Curve(int I) => myDS.Curve(I);

    internal ref TopOpeBRepDS_Curve ChangeCurve(int I) => ref myDS.ChangeCurve(I);

    /// <summary>An iterator on the points on the curve <paramref name="I"/>.</summary>
    public TopOpeBRepDS_PointIterator CurvePoints(int I)
    {
        NCollection_List<TopOpeBRepDS_Interference> L = myDS.CurveInterferences(I);
        return new TopOpeBRepDS_PointIterator(L);
    }

    /// <summary>The point <paramref name="I"/>.</summary>
    public TopOpeBRepDS_Point Point(int I) => myDS.Point(I);

    /// <summary>Whether the shape <paramref name="S"/> has new geometries.</summary>
    public bool HasGeometry(in TopoDS_Shape S)
    {
        bool b = myDS.HasGeometry(S);
        return b;
    }

    /// <summary>Whether the shape <paramref name="S"/> is in the data structure.</summary>
    public bool HasShape(in TopoDS_Shape S, bool FindKeep = true)
    {
        bool b = myDS.HasShape(S, FindKeep);
        return b;
    }

    /// <summary>Whether the shape <paramref name="S"/> shares its geometrical domain with other shapes.</summary>
    public bool HasSameDomain(in TopoDS_Shape S, bool FindKeep = true)
    {
        if (!HasShape(S, FindKeep))
        {
            return false;
        }
        NCollection_List<TopoDS_Shape> l = myDS.ShapeSameDomain(S);
        bool res = !l.IsEmpty();
        return res;
    }

    /// <summary>An iterator on the shapes sharing the geometrical domain of <paramref name="S"/>.</summary>
    public NCollection_List<TopoDS_Shape>.Iterator SameDomain(in TopoDS_Shape S)
    {
        NCollection_List<TopoDS_Shape> L = myDS.ShapeSameDomain(S);
        return new NCollection_List<TopoDS_Shape>.Iterator(L);
    }

    /// <summary>How <paramref name="S"/> lies with respect to its same-domain reference.</summary>
    public TopOpeBRepDS_Config SameDomainOrientation(in TopoDS_Shape S)
    {
        if (!HasShape(S))
        {
            return TopOpeBRepDS_UNSHGEOMETRY;
        }
        return myDS.SameDomainOri(S);
    }

    /// <summary>The index of the same-domain reference of <paramref name="S"/>.</summary>
    public int SameDomainReference(in TopoDS_Shape S)
    {
        if (!HasShape(S))
        {
            return 0;
        }
        return myDS.SameDomainRef(S);
    }

    /// <summary>The number of shapes.</summary>
    public int NbShapes() => myDS.NbShapes();

    /// <summary>The shape <paramref name="I"/>.</summary>
    public TopoDS_Shape Shape(int I, bool FindKeep = true) => myDS.Shape(I, FindKeep);

    /// <summary>The index of the shape <paramref name="S"/>, or 0.</summary>
    public int Shape(in TopoDS_Shape S, bool FindKeep = true) => myDS.Shape(S, FindKeep);

    /// <summary>
    /// An iterator on the surfaces attached to the solid <paramref name="S"/> (see <see cref="TopOpeBRepDS_SurfaceIterator"/>
    /// for OCCT's first value).
    /// </summary>
    public TopOpeBRepDS_SurfaceIterator SolidSurfaces(in TopoDS_Shape S)
    {
        NCollection_List<TopOpeBRepDS_Interference> L = myDS.ShapeInterferences(S);
        return new TopOpeBRepDS_SurfaceIterator(L);
    }

    /// <summary>An iterator on the surfaces attached to the solid <paramref name="I"/>.</summary>
    public TopOpeBRepDS_SurfaceIterator SolidSurfaces(int I)
    {
        NCollection_List<TopOpeBRepDS_Interference> L = myDS.ShapeInterferences(I);
        return new TopOpeBRepDS_SurfaceIterator(L);
    }

    /// <summary>An iterator on the curves attached to the face <paramref name="F"/>.</summary>
    public TopOpeBRepDS_CurveIterator FaceCurves(in TopoDS_Shape F)
    {
        NCollection_List<TopOpeBRepDS_Interference> L = myDS.ShapeInterferences(F);
        return new TopOpeBRepDS_CurveIterator(L);
    }

    /// <summary>An iterator on the curves attached to the face <paramref name="I"/>.</summary>
    public TopOpeBRepDS_CurveIterator FaceCurves(int I)
    {
        NCollection_List<TopOpeBRepDS_Interference> L = myDS.ShapeInterferences(I);
        return new TopOpeBRepDS_CurveIterator(L);
    }

    /// <summary>An iterator on the points attached to the edge <paramref name="E"/>.</summary>
    public TopOpeBRepDS_PointIterator EdgePoints(in TopoDS_Shape E)
    {
        NCollection_List<TopOpeBRepDS_Interference> L = myDS.ShapeInterferences(E);
        return new TopOpeBRepDS_PointIterator(L);
    }

    /// <summary>
    /// Adds <paramref name="newC"/> as a daughter of <paramref name="curC"/>: on the same shapes, with copies of its
    /// surface/curve interferences (added to the shapes' lists, their geometry the new curve); returns its index.
    /// </summary>
    internal int MakeCurve(in TopOpeBRepDS_Curve curC, ref TopOpeBRepDS_Curve newC)
    {
        // SCI1, SCI2 = new surface/curve interf. build from
        // the surface/curve interf. of the curve curC
        TopOpeBRepDS_Interference? I1 = curC.GetSCI1();
        TopOpeBRepDS_Interference? I2 = curC.GetSCI2();
        TopOpeBRepDS_SurfaceCurveInterference? SCI1 = null, SCI2 = null;
        if (I1 is not null)
        {
            SCI1 = new TopOpeBRepDS_SurfaceCurveInterference(I1);
        }
        if (I2 is not null)
        {
            SCI2 = new TopOpeBRepDS_SurfaceCurveInterference(I2);
        }

        TopoDS_Shape S1 = curC.Shape1();
        TopoDS_Shape S2 = curC.Shape2();

        // add the new SCIs in the lists of SCIs connected to the shapes
        if (SCI1 is not null)
        {
            myDS.AddShapeInterference(S1, SCI1);
        }
        if (SCI2 is not null)
        {
            myDS.AddShapeInterference(S2, SCI2);
        }

        // the shapes of the new curve are the shapes of curve curC
        newC.SetShapes(S1, S2);

        // set surface/curve interferences of the new curve
        newC.SetSCI(SCI1, SCI2);

        // newC is a daughter of curC
        newC.ChangeMother(curC.DSIndex());

        // add the new curve in the DS
        int inewC = myDS.AddCurve(ref newC);

        // the geometry of the new surface/curve interf. is new curve inewC
        if (SCI1 is not null)
        {
            SCI1.Geometry(inewC);
        }
        if (SCI2 is not null)
        {
            SCI2.Geometry(inewC);
        }

        return inewC;
    }

    internal void RemoveCurve(int icurC)
    {
        myDS.RemoveCurve(icurC);
    }

    /// <summary>The number of geometries of kind <paramref name="K"/> (0 if it isn't a geometry kind).</summary>
    public int NbGeometry(TopOpeBRepDS_Kind K)
    {
        if (!TopOpeBRepDS.IsGeometry(K))
        {
            return 0;
        }

        int n = 0;
        switch (K)
        {
            case TopOpeBRepDS_POINT:
                n = NbPoints();
                break;
            case TopOpeBRepDS_CURVE:
                n = NbCurves();
                break;
            case TopOpeBRepDS_SURFACE:
                n = NbSurfaces();
                break;
            default:
                n = 0;
                break;
        }
        return n;
    }

    /// <summary>The number of shapes.</summary>
    public int NbTopology()
    {
        int n = myDS.NbShapes();
        return n;
    }

    /// <summary>The number of shapes of kind <paramref name="K"/>, kept or not.</summary>
    public int NbTopology(TopOpeBRepDS_Kind K)
    {
        if (!TopOpeBRepDS.IsTopology(K))
        {
            return 0;
        }
        int res = 0;
        const bool FindKeep = false;

        TopAbs_ShapeEnum S = TopOpeBRepDS.KindToShape(K);
        for (int i = 1; i <= NbTopology(); i++)
        {
            if (myDS.KeepShape(i, FindKeep))
            {
                if (myDS.Shape(i, FindKeep).ShapeType() == S)
                {
                    res++;
                }
            }
        }
        return res;
    }

    /// <summary>Whether every edge of the data structure is SameParameter.</summary>
    public bool EdgesSameParameter()
    {
        int i, n = myDS.NbShapes();
        for (i = 1; i <= n; i++)
        {
            TopoDS_Shape s = myDS.Shape(i);
            if (s.ShapeType() == TopAbs_EDGE)
            {
                TopoDS_Edge e = TopoDS.Edge(s);
                if (!BRep_Tool.SameParameter(e))
                {
                    return false;
                }
            }
        }
        return true;
    }

    /// <summary>
    /// Appends to <paramref name="SList"/> the point interferences of <paramref name="List"/> by increasing parameter
    /// (a selection sort, as OCCT's; equal parameters keep the list's order).
    /// </summary>
    /// <exception cref="Standard_ProgramError">A point interference has no parameter (<see cref="TopOpeBRepDS_PointIterator.Parameter"/>).</exception>
    internal static void FUN_TopOpeBRepDS_SortOnParameter(NCollection_List<TopOpeBRepDS_Interference> List,
                                                          NCollection_List<TopOpeBRepDS_Interference> SList)
    {
        // NYI : sort a list of Items, giving a sorting FUNCTION is impossible
        // NYI : --> foobar method complexity n2.

        int iIntf = 0, nIntf = List.Extent();
        if (nIntf == 0)
        {
            return;
        }

        NCollection_HArray1<bool> HT = new(1, nIntf, false);
        NCollection_Array1<bool> T = HT.ChangeArray1();

        TopOpeBRepDS_Interference? Intf = null;
        for (int i = 1; i <= nIntf; i++)
        {
            double parmin = RealLast();
            TopOpeBRepDS_PointIterator it = new(List);
            for (int itest = 1; it.More(); it.Next(), itest++)
            {
                if (!T[itest])
                {
                    double par = it.Parameter();
                    if (par < parmin)
                    {
                        parmin = par;
                        Intf = it.Value();
                        iIntf = itest;
                    }
                }
            }
            // OCCT: with fewer point interferences than interferences, the last one found is appended again
            SList.Append(Intf!);
            T[iIntf] = true;
        }
    }

    /// <summary>
    /// <paramref name="SList"/> := the point interferences of <paramref name="List"/> sorted on their parameter, then
    /// rotated so that the first FORWARD one (for IN) comes first.
    /// </summary>
    internal void SortOnParameter(NCollection_List<TopOpeBRepDS_Interference> List,
                                  NCollection_List<TopOpeBRepDS_Interference> SList)
    {
        // NYI : sort a list of Items, giving a sorting FUNCTION is impossible
        // NYI : --> foobar method complexity n2.

        FUN_TopOpeBRepDS_SortOnParameter(List, SList);

        // tete = interf FORWARD :
        // modifier TopOpeBRepBuild_ParametrizedVertexSet ::SortParametrizedVertex()
        bool found = false;
        NCollection_List<TopOpeBRepDS_Interference>.Iterator it = new(SList);
        NCollection_List<TopOpeBRepDS_Interference> L1 = new(), L2 = new();
        for (; it.More(); it.Next())
        {
            TopOpeBRepDS_Interference I = it.Value();
            if (!found)
            {
                TopAbs_Orientation o = I.Transition().Orientation(TopAbs_IN);
                if (o == TopAbs_FORWARD)
                {
                    found = true;
                    L1.Append(I);
                }
                else
                {
                    L2.Append(I);
                }
            }
            else
            {
                L1.Append(I);
            }
        }

        SList.Clear();
        SList.Append(L1);
        SList.Append(L2);
    }

    /// <summary>Sorts <paramref name="List"/> on its point parameters (as the other overload), if it has a point interference.</summary>
    internal void SortOnParameter(NCollection_List<TopOpeBRepDS_Interference> List)
    {
        TopOpeBRepDS_PointIterator it = new(List);
        if (it.More())
        {
            NCollection_List<TopOpeBRepDS_Interference> SList = new();
            SortOnParameter(List, SList);
            List.Assign(SList);
        }
    }

    /// <summary>The smallest and largest parameters of the point interferences of <paramref name="List"/> (unchanged if it is empty).</summary>
    public void MinMaxOnParameter(NCollection_List<TopOpeBRepDS_Interference> List, ref double parmin, ref double parmax)
    {
        if (!List.IsEmpty())
        {
            double parline;
            parmin = RealLast();
            parmax = RealFirst();
            TopOpeBRepDS_PointIterator it = new(List);
            for (; it.More(); it.Next())
            {
                parline = it.Parameter();
                parmin = Min(parmin, parline);
                parmax = Max(parmax, parline);
            }
        }
    }

    /// <summary>
    /// Moves <paramref name="IT"/> to the first interference, from its current one, whose point (or vertex) equals
    /// <paramref name="PDS"/> within tolerance; false if there is none.
    /// </summary>
    public bool ScanInterfList(NCollection_List<TopOpeBRepDS_Interference>.Iterator IT, in TopOpeBRepDS_Point PDS)
    {
        for (; IT.More(); IT.Next())
        {
            TopOpeBRepDS_Kind GT = IT.Value().GeometryType();
            int G = IT.Value().Geometry();
            if (GT == TopOpeBRepDS_POINT)
            {
                TopOpeBRepDS_Point OOPDS = myDS.Point(G);
                bool iseq = PDS.IsEqual(OOPDS);
                if (iseq)
                {
                    return iseq;
                }
            }
            else if (GT == TopOpeBRepDS_VERTEX)
            {
                TopOpeBRepDS_Point OOPDS = new(myDS.Shape(G));
                bool iseq = PDS.IsEqual(OOPDS);
                if (iseq)
                {
                    return iseq;
                }
            }
        }
        return false;
    }

    /// <summary>As <see cref="ScanInterfList"/>; if found, the geometry index and kind of the interference found.</summary>
    public bool GetGeometry(NCollection_List<TopOpeBRepDS_Interference>.Iterator IT,
                            in TopOpeBRepDS_Point PDS,
                            ref int G,
                            ref TopOpeBRepDS_Kind K)
    {
        bool found = ScanInterfList(IT, PDS);
        if (found)
        {
            G = IT.Value().Geometry();
            K = IT.Value().GeometryType();
        }
        return found;
    }

    /// <summary>
    /// Appends the interference <paramref name="I"/> to the list <paramref name="LI"/>, and to the interferences of its
    /// geometry if that is a surface or a curve.
    /// </summary>
    internal void StoreInterference(TopOpeBRepDS_Interference I, NCollection_List<TopOpeBRepDS_Interference> LI, string str = "")
    {
        _ = str;
        // append I to list LI
        LI.Append(I);

        int G = I.Geometry();

        // append I to list of interference connected to G = I->Geometry()
        switch (I.GeometryType())
        {
            case TopOpeBRepDS_SOLID:
            case TopOpeBRepDS_FACE:
            case TopOpeBRepDS_EDGE:
            case TopOpeBRepDS_VERTEX:
                //    appendtoG = true;
                //    myDS.ChangeShapeInterferences(G).Append(I);
                break;

            case TopOpeBRepDS_SURFACE:
                myDS.ChangeSurfaceInterferences(G).Append(I);
                break;

            case TopOpeBRepDS_CURVE:
                myDS.ChangeCurveInterferences(G).Append(I);
                break;

            case TopOpeBRepDS_POINT:
                //    appendtoG = true;
                //    myDS.ChangePointInterferences(G).Append(I);
                break;
            default:
                break;
        }
    }

    /// <summary>Stores the interference <paramref name="I"/> on the shape <paramref name="S"/> (and its geometry).</summary>
    /// <exception cref="Standard_ProgramError"><paramref name="S"/> is not in the data structure.</exception>
    internal void StoreInterference(TopOpeBRepDS_Interference I, in TopoDS_Shape S, string str = "")
    {
        _ = str;
        bool h = myDS.HasShape(S);
        if (!h)
        {
            throw new Standard_ProgramError("StoreInterference on shape out of DS");
        }
        StoreInterference(I, myDS.ChangeShapeInterferences(S));
    }

    /// <summary>Stores the interference <paramref name="I"/> on the shape <paramref name="IS"/> (and its geometry).</summary>
    /// <exception cref="Standard_ProgramError"><paramref name="IS"/> is not a shape index.</exception>
    internal void StoreInterference(TopOpeBRepDS_Interference I, int IS, string str = "")
    {
        _ = str;
        int n = myDS.NbShapes();
        if (IS < 1 || IS > n)
        {
            throw new Standard_ProgramError("StoreInterference on index out of DS");
        }

        StoreInterference(I, myDS.ChangeShapeInterferences(IS));
    }

    /// <summary>Stores each interference of <paramref name="LI"/> on the shape <paramref name="IS"/>.</summary>
    internal void StoreInterferences(NCollection_List<TopOpeBRepDS_Interference> LI, int IS, string str = "")
    {
        _ = str;
        NCollection_List<TopOpeBRepDS_Interference> lids = myDS.ChangeShapeInterferences(IS);
        NCollection_List<TopOpeBRepDS_Interference>.Iterator it = new(LI);
        for (; it.More(); it.Next())
        {
            TopOpeBRepDS_Interference I = it.Value();
            StoreInterference(I, lids);
        }
    }

    /// <summary>Stores each interference of <paramref name="LI"/> on the shape <paramref name="S"/>.</summary>
    internal void StoreInterferences(NCollection_List<TopOpeBRepDS_Interference> LI, in TopoDS_Shape S, string str = "")
    {
        _ = str;
        NCollection_List<TopOpeBRepDS_Interference> lids = myDS.ChangeShapeInterferences(S);
        NCollection_List<TopOpeBRepDS_Interference>.Iterator it = new(LI);
        for (; it.More(); it.Next())
        {
            TopOpeBRepDS_Interference I = it.Value();
            StoreInterference(I, lids);
        }
    }

    /// <summary>Empties the interferences of the shape <paramref name="IS"/>, then stores those of <paramref name="LI"/>.</summary>
    internal void ClearStoreInterferences(NCollection_List<TopOpeBRepDS_Interference> LI, int IS, string str = "")
    {
        _ = str;
        NCollection_List<TopOpeBRepDS_Interference> lids = myDS.ChangeShapeInterferences(IS);
        lids.Clear();
        NCollection_List<TopOpeBRepDS_Interference>.Iterator it = new(LI);
        for (; it.More(); it.Next())
        {
            TopOpeBRepDS_Interference I = it.Value();
            StoreInterference(I, lids);
        }
    }

    /// <summary>Empties the interferences of the shape <paramref name="S"/>, then stores those of <paramref name="LI"/>.</summary>
    internal void ClearStoreInterferences(NCollection_List<TopOpeBRepDS_Interference> LI, in TopoDS_Shape S, string str = "")
    {
        _ = str;
        NCollection_List<TopOpeBRepDS_Interference> lids = myDS.ChangeShapeInterferences(S);
        lids.Clear();
        NCollection_List<TopOpeBRepDS_Interference>.Iterator it = new(LI);
        for (; it.More(); it.Next())
        {
            TopOpeBRepDS_Interference I = it.Value();
            StoreInterference(I, lids);
        }
    }
}
