// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_HDataStructure.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_HDataStructure.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Legacy.Tools;
using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Topology;
using Kerf.Engine;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;
using static Kerf.Topology.Containment;
using static Kerf.Engine.Booleans.Legacy.Data.DsConfig;
using static Kerf.Engine.Booleans.Legacy.Data.DsKind;

namespace Kerf.Engine.Booleans.Legacy.Data;

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
internal sealed class DsHDataStructure : Transient
{
    private readonly DsDataStructure _ds = new();

    // the geometry and support kinds and indices of I (left unchanged if I is null)
    private static void FUN_HDS_data(DsInterference? i,
                                     ref DsKind gt1,
                                     ref int g1,
                                     ref DsKind st1,
                                     ref int s1)
    {
        if (i is null)
        {
            return;
        }
        gt1 = i.GeometryType();
        g1 = i.Geometry();
        st1 = i.SupportType();
        s1 = i.Support();
    }

    /// <summary>Whether the face <paramref name="f1"/> has a face/edge/face interference with the face <paramref name="f2"/>.</summary>
    internal static bool FUN_HDS_FACESINTERFER(in Shape f1, in Shape f2, DsHDataStructure hds)
    {
        bool yainterf = false;
        DsDataStructure ds = hds.DS();
        bool ya = ds.HasShape(f1);
        if (!ya)
        {
            return false;
        }

        //                         DS.Shape(F1);
        int iF2 = ds.Shape(f2);

        NCollection_List<DsInterference> l1 = ds.ShapeInterferences(f1);
        NCollection_List<DsInterference>.Iterator itL1 = new(l1);
        for (; itL1.More(); itL1.Next())
        {
            DsInterference i = itL1.Value();
            DsKind gt = DsKind.Unknown, st = DsKind.Unknown;
            int g = 0, s = 0; // OCCT leaves G unset (it is set from I, never null here)
            FUN_HDS_data(i, ref gt, ref g, ref st, ref s);
            // interference face1/edge/face2
            bool fef = true;
            fef = fef && (gt == DsKind.Edge);
            fef = fef && (st == DsKind.Face);
            fef = fef && (s == iF2);
            if (fef)
            {
                yainterf = true;
                break;
            }
        }
        return yainterf;
    }

    /// <summary>An empty data structure.</summary>
    public DsHDataStructure()
    {
    }

    /// <summary>Adds to the data structure the wires, faces, shells and solids of <paramref name="s"/> that contain a shape of it.</summary>
    internal void AddAncestors(in Shape s)
    {
        AddAncestors(s, ShapeType.Wire, ShapeType.Edge);
        AddAncestors(s, ShapeType.Face, ShapeType.Wire);
        AddAncestors(s, ShapeType.Shell, ShapeType.Face);
        AddAncestors(s, ShapeType.Solid, ShapeType.Shell);
    }

    /// <summary>
    /// Adds the shapes of type <paramref name="t1"/> of <paramref name="s"/> containing a sub-shape of type
    /// <paramref name="t2"/> that is in the data structure, with <paramref name="s"/>'s ancestor rank.
    /// </summary>
    internal void AddAncestors(in Shape s, ShapeType t1, ShapeType t2)
    {
        DsDataStructure bds = ChangeDS();
        int rankS = _ds.AncestorRank(s);

        // find the shapes of type T1 containing HasShape() of type T2
        for (ToolShapeExplorer eT1 = new(s, t1); eT1.More(); eT1.Next())
        {
            Shape st1 = eT1.Current();
            for (ToolShapeExplorer eT2 = new(st1, t2); eT2.More(); eT2.Next())
            {
                Shape st2 = eT2.Current();
                if (bds.HasShape(st2))
                {
                    bds.AddShape(st1, rankS);
                    break;
                }
            }
        }
    }

    // ChkIntg: not ported (TopOpeBRepDS_Check, a tool of the legacy builder; plan 2.1, decision 992)

    /// <summary>The data structure (its mutators are internal).</summary>
    public DsDataStructure DS() => _ds;

    internal DsDataStructure ChangeDS() => _ds;

    /// <summary>The number of surfaces.</summary>
    public int SurfaceCount => _ds.NbSurfaces();

    /// <summary>The number of curves.</summary>
    public int CurveCount => _ds.NbCurves();

    /// <summary>The number of points.</summary>
    public int PointCount => _ds.NbPoints();

    /// <summary>The surface <paramref name="i"/>.</summary>
    public DsSurface Surface(int i) => _ds.Surface(i);

    /// <summary>An iterator on the curves on the surface <paramref name="i"/>.</summary>
    public DsCurveIterator SurfaceCurves(int i)
    {
        NCollection_List<DsInterference> l = _ds.SurfaceInterferences(i);
        return new DsCurveIterator(l);
    }

    /// <summary>The curve <paramref name="i"/>.</summary>
    public DsCurve Curve(int i) => _ds.Curve(i);

    internal ref DsCurve ChangeCurve(int i) => ref _ds.ChangeCurve(i);

    /// <summary>An iterator on the points on the curve <paramref name="i"/>.</summary>
    public DsPointIterator CurvePoints(int i)
    {
        NCollection_List<DsInterference> l = _ds.CurveInterferences(i);
        return new DsPointIterator(l);
    }

    /// <summary>The point <paramref name="i"/>.</summary>
    public DsPoint Point(int i) => _ds.Point(i);

    /// <summary>Whether the shape <paramref name="s"/> has new geometries.</summary>
    public bool HasGeometry(in Shape s)
    {
        bool b = _ds.HasGeometry(s);
        return b;
    }

    /// <summary>Whether the shape <paramref name="s"/> is in the data structure.</summary>
    public bool HasShape(in Shape s, bool findKeep = true)
    {
        bool b = _ds.HasShape(s, findKeep);
        return b;
    }

    /// <summary>Whether the shape <paramref name="s"/> shares its geometrical domain with other shapes.</summary>
    public bool HasSameDomain(in Shape s, bool findKeep = true)
    {
        if (!HasShape(s, findKeep))
        {
            return false;
        }
        NCollection_List<Shape> l = _ds.ShapeSameDomain(s);
        bool res = !l.IsEmpty();
        return res;
    }

    /// <summary>An iterator on the shapes sharing the geometrical domain of <paramref name="s"/>.</summary>
    public NCollection_List<Shape>.Iterator SameDomain(in Shape s)
    {
        NCollection_List<Shape> l = _ds.ShapeSameDomain(s);
        return new NCollection_List<Shape>.Iterator(l);
    }

    /// <summary>How <paramref name="s"/> lies with respect to its same-domain reference.</summary>
    public DsConfig SameDomainOrientation(in Shape s)
    {
        if (!HasShape(s))
        {
            return DsConfig.Unshgeometry;
        }
        return _ds.SameDomainOri(s);
    }

    /// <summary>The index of the same-domain reference of <paramref name="s"/>.</summary>
    public int SameDomainReference(in Shape s)
    {
        if (!HasShape(s))
        {
            return 0;
        }
        return _ds.SameDomainRef(s);
    }

    /// <summary>The number of shapes.</summary>
    public int ShapeCount => _ds.NbShapes();

    /// <summary>The shape <paramref name="i"/>.</summary>
    public Shape Shape(int i, bool findKeep = true) => _ds.Shape(i, findKeep);

    /// <summary>The index of the shape <paramref name="s"/>, or 0.</summary>
    public int Shape(in Shape s, bool findKeep = true) => _ds.Shape(s, findKeep);

    /// <summary>
    /// An iterator on the surfaces attached to the solid <paramref name="s"/> (see <see cref="DsSurfaceIterator"/>
    /// for OCCT's first value).
    /// </summary>
    public DsSurfaceIterator SolidSurfaces(in Shape s)
    {
        NCollection_List<DsInterference> l = _ds.ShapeInterferences(s);
        return new DsSurfaceIterator(l);
    }

    /// <summary>An iterator on the surfaces attached to the solid <paramref name="i"/>.</summary>
    public DsSurfaceIterator SolidSurfaces(int i)
    {
        NCollection_List<DsInterference> l = _ds.ShapeInterferences(i);
        return new DsSurfaceIterator(l);
    }

    /// <summary>An iterator on the curves attached to the face <paramref name="f"/>.</summary>
    public DsCurveIterator FaceCurves(in Shape f)
    {
        NCollection_List<DsInterference> l = _ds.ShapeInterferences(f);
        return new DsCurveIterator(l);
    }

    /// <summary>An iterator on the curves attached to the face <paramref name="i"/>.</summary>
    public DsCurveIterator FaceCurves(int i)
    {
        NCollection_List<DsInterference> l = _ds.ShapeInterferences(i);
        return new DsCurveIterator(l);
    }

    /// <summary>An iterator on the points attached to the edge <paramref name="e"/>.</summary>
    public DsPointIterator EdgePoints(in Shape e)
    {
        NCollection_List<DsInterference> l = _ds.ShapeInterferences(e);
        return new DsPointIterator(l);
    }

    /// <summary>
    /// Adds <paramref name="newC"/> as a daughter of <paramref name="curC"/>: on the same shapes, with copies of its
    /// surface/curve interferences (added to the shapes' lists, their geometry the new curve); returns its index.
    /// </summary>
    internal int MakeCurve(in DsCurve curC, ref DsCurve newC)
    {
        // SCI1, SCI2 = new surface/curve interf. build from
        // the surface/curve interf. of the curve curC
        DsInterference? i1 = curC.GetSCI1();
        DsInterference? i2 = curC.GetSCI2();
        DsSurfaceCurveInterference? sci1 = null, sci2 = null;
        if (i1 is not null)
        {
            sci1 = new DsSurfaceCurveInterference(i1);
        }
        if (i2 is not null)
        {
            sci2 = new DsSurfaceCurveInterference(i2);
        }

        Shape s1 = curC.Shape1();
        Shape s2 = curC.Shape2();

        // add the new SCIs in the lists of SCIs connected to the shapes
        if (sci1 is not null)
        {
            _ds.AddShapeInterference(s1, sci1);
        }
        if (sci2 is not null)
        {
            _ds.AddShapeInterference(s2, sci2);
        }

        // the shapes of the new curve are the shapes of curve curC
        newC.SetShapes(s1, s2);

        // set surface/curve interferences of the new curve
        newC.SetSCI(sci1, sci2);

        // newC is a daughter of curC
        newC.ChangeMother(curC.DSIndex());

        // add the new curve in the DS
        int inewC = _ds.AddCurve(ref newC);

        // the geometry of the new surface/curve interf. is new curve inewC
        if (sci1 is not null)
        {
            sci1.Geometry(inewC);
        }
        if (sci2 is not null)
        {
            sci2.Geometry(inewC);
        }

        return inewC;
    }

    internal void RemoveCurve(int icurC)
    {
        _ds.RemoveCurve(icurC);
    }

    /// <summary>The number of geometries of kind <paramref name="k"/> (0 if it isn't a geometry kind).</summary>
    public int NbGeometry(DsKind k)
    {
        if (!DsTopOpeBRepDS.IsGeometry(k))
        {
            return 0;
        }

        int n = 0;
        switch (k)
        {
            case DsKind.Point:
                n = PointCount;
                break;
            case DsKind.Curve:
                n = CurveCount;
                break;
            case DsKind.Surface:
                n = SurfaceCount;
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
        int n = _ds.NbShapes();
        return n;
    }

    /// <summary>The number of shapes of kind <paramref name="k"/>, kept or not.</summary>
    public int NbTopology(DsKind k)
    {
        if (!DsTopOpeBRepDS.IsTopology(k))
        {
            return 0;
        }
        int res = 0;
        const bool findKeep = false;

        ShapeType s = DsTopOpeBRepDS.KindToShape(k);
        for (int i = 1; i <= NbTopology(); i++)
        {
            if (_ds.KeepShape(i, findKeep))
            {
                if (_ds.Shape(i, findKeep).Type == s)
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
        int i, n = _ds.NbShapes();
        for (i = 1; i <= n; i++)
        {
            Shape s = _ds.Shape(i);
            if (s.Type == ShapeType.Edge)
            {
                Edge e = ShapeCasts.Edge(s);
                if (!Tool.SameParameter(e))
                {
                    return false;
                }
            }
        }
        return true;
    }

    /// <summary>
    /// Appends to <paramref name="sList"/> the point interferences of <paramref name="list"/> by increasing parameter
    /// (a selection sort, as OCCT's; equal parameters keep the list's order).
    /// </summary>
    /// <exception cref="ProgramErrorException">A point interference has no parameter (<see cref="DsPointIterator.Parameter"/>).</exception>
    internal static void FUN_TopOpeBRepDS_SortOnParameter(NCollection_List<DsInterference> list,
                                                          NCollection_List<DsInterference> sList)
    {
        // NYI : sort a list of Items, giving a sorting FUNCTION is impossible
        // NYI : --> foobar method complexity n2.

        int iIntf = 0, nIntf = list.Extent();
        if (nIntf == 0)
        {
            return;
        }

        NCollection_HArray1<bool> ht = new(1, nIntf, false);
        NCollection_Array1<bool> t = ht.ChangeArray1();

        DsInterference? intf = null;
        for (int i = 1; i <= nIntf; i++)
        {
            double parmin = RealLast();
            DsPointIterator it = new(list);
            for (int itest = 1; it.More(); it.Next(), itest++)
            {
                if (!t[itest])
                {
                    double par = it.Parameter();
                    if (par < parmin)
                    {
                        parmin = par;
                        intf = it.Value();
                        iIntf = itest;
                    }
                }
            }
            // OCCT: with fewer point interferences than interferences, the last one found is appended again
            sList.Append(intf!);
            t[iIntf] = true;
        }
    }

    /// <summary>
    /// <paramref name="sList"/> := the point interferences of <paramref name="list"/> sorted on their parameter, then
    /// rotated so that the first FORWARD one (for IN) comes first.
    /// </summary>
    internal void SortOnParameter(NCollection_List<DsInterference> list,
                                  NCollection_List<DsInterference> sList)
    {
        // NYI : sort a list of Items, giving a sorting FUNCTION is impossible
        // NYI : --> foobar method complexity n2.

        FUN_TopOpeBRepDS_SortOnParameter(list, sList);

        // tete = interf FORWARD :
        // modifier TopOpeBRepBuild_ParametrizedVertexSet ::SortParametrizedVertex()
        bool found = false;
        NCollection_List<DsInterference>.Iterator it = new(sList);
        NCollection_List<DsInterference> l1 = new(), l2 = new();
        for (; it.More(); it.Next())
        {
            DsInterference i = it.Value();
            if (!found)
            {
                Orientation o = i.Transition().Orientation(In);
                if (o == Forward)
                {
                    found = true;
                    l1.Append(i);
                }
                else
                {
                    l2.Append(i);
                }
            }
            else
            {
                l1.Append(i);
            }
        }

        sList.Clear();
        sList.Append(l1);
        sList.Append(l2);
    }

    /// <summary>Sorts <paramref name="list"/> on its point parameters (as the other overload), if it has a point interference.</summary>
    internal void SortOnParameter(NCollection_List<DsInterference> list)
    {
        DsPointIterator it = new(list);
        if (it.More())
        {
            NCollection_List<DsInterference> sList = new();
            SortOnParameter(list, sList);
            list.Assign(sList);
        }
    }

    /// <summary>The smallest and largest parameters of the point interferences of <paramref name="list"/> (unchanged if it is empty).</summary>
    public void MinMaxOnParameter(NCollection_List<DsInterference> list, ref double parmin, ref double parmax)
    {
        if (!list.IsEmpty())
        {
            double parline;
            parmin = RealLast();
            parmax = RealFirst();
            DsPointIterator it = new(list);
            for (; it.More(); it.Next())
            {
                parline = it.Parameter();
                parmin = Min(parmin, parline);
                parmax = Max(parmax, parline);
            }
        }
    }

    /// <summary>
    /// Moves <paramref name="it"/> to the first interference, from its current one, whose point (or vertex) equals
    /// <paramref name="pds"/> within tolerance; false if there is none.
    /// </summary>
    public bool ScanInterfList(NCollection_List<DsInterference>.Iterator it, in DsPoint pds)
    {
        for (; it.More(); it.Next())
        {
            DsKind gt = it.Value().GeometryType();
            int g = it.Value().Geometry();
            if (gt == DsKind.Point)
            {
                DsPoint oopds = _ds.Point(g);
                bool iseq = pds.IsEqual(oopds);
                if (iseq)
                {
                    return iseq;
                }
            }
            else if (gt == DsKind.Vertex)
            {
                DsPoint oopds = new(_ds.Shape(g));
                bool iseq = pds.IsEqual(oopds);
                if (iseq)
                {
                    return iseq;
                }
            }
        }
        return false;
    }

    /// <summary>As <see cref="ScanInterfList"/>; if found, the geometry index and kind of the interference found.</summary>
    public bool GetGeometry(NCollection_List<DsInterference>.Iterator it,
                            in DsPoint pds,
                            ref int g,
                            ref DsKind k)
    {
        bool found = ScanInterfList(it, pds);
        if (found)
        {
            g = it.Value().Geometry();
            k = it.Value().GeometryType();
        }
        return found;
    }

    /// <summary>
    /// Appends the interference <paramref name="i"/> to the list <paramref name="li"/>, and to the interferences of its
    /// geometry if that is a surface or a curve.
    /// </summary>
    internal void StoreInterference(DsInterference i, NCollection_List<DsInterference> li, string str = "")
    {
        _ = str;
        // append I to list LI
        li.Append(i);

        int g = i.Geometry();

        // append I to list of interference connected to G = I->Geometry()
        switch (i.GeometryType())
        {
            case DsKind.Solid:
            case DsKind.Face:
            case DsKind.Edge:
            case DsKind.Vertex:
                //    appendtoG = true;
                //    myDS.ChangeShapeInterferences(G).Append(I);
                break;

            case DsKind.Surface:
                _ds.ChangeSurfaceInterferences(g).Append(i);
                break;

            case DsKind.Curve:
                _ds.ChangeCurveInterferences(g).Append(i);
                break;

            case DsKind.Point:
                //    appendtoG = true;
                //    myDS.ChangePointInterferences(G).Append(I);
                break;
            default:
                break;
        }
    }

    /// <summary>Stores the interference <paramref name="i"/> on the shape <paramref name="s"/> (and its geometry).</summary>
    /// <exception cref="ProgramErrorException"><paramref name="s"/> is not in the data structure.</exception>
    internal void StoreInterference(DsInterference i, in Shape s, string str = "")
    {
        _ = str;
        bool h = _ds.HasShape(s);
        if (!h)
        {
            throw new ProgramErrorException("StoreInterference on shape out of DS");
        }
        StoreInterference(i, _ds.ChangeShapeInterferences(s));
    }

    /// <summary>Stores the interference <paramref name="i"/> on the shape <paramref name="IS"/> (and its geometry).</summary>
    /// <exception cref="ProgramErrorException"><paramref name="IS"/> is not a shape index.</exception>
    internal void StoreInterference(DsInterference i, int IS, string str = "")
    {
        _ = str;
        int n = _ds.NbShapes();
        if (IS < 1 || IS > n)
        {
            throw new ProgramErrorException("StoreInterference on index out of DS");
        }

        StoreInterference(i, _ds.ChangeShapeInterferences(IS));
    }

    /// <summary>Stores each interference of <paramref name="li"/> on the shape <paramref name="IS"/>.</summary>
    internal void StoreInterferences(NCollection_List<DsInterference> li, int IS, string str = "")
    {
        _ = str;
        NCollection_List<DsInterference> lids = _ds.ChangeShapeInterferences(IS);
        NCollection_List<DsInterference>.Iterator it = new(li);
        for (; it.More(); it.Next())
        {
            DsInterference i = it.Value();
            StoreInterference(i, lids);
        }
    }

    /// <summary>Stores each interference of <paramref name="li"/> on the shape <paramref name="s"/>.</summary>
    internal void StoreInterferences(NCollection_List<DsInterference> li, in Shape s, string str = "")
    {
        _ = str;
        NCollection_List<DsInterference> lids = _ds.ChangeShapeInterferences(s);
        NCollection_List<DsInterference>.Iterator it = new(li);
        for (; it.More(); it.Next())
        {
            DsInterference i = it.Value();
            StoreInterference(i, lids);
        }
    }

    /// <summary>Empties the interferences of the shape <paramref name="IS"/>, then stores those of <paramref name="li"/>.</summary>
    internal void ClearStoreInterferences(NCollection_List<DsInterference> li, int IS, string str = "")
    {
        _ = str;
        NCollection_List<DsInterference> lids = _ds.ChangeShapeInterferences(IS);
        lids.Clear();
        NCollection_List<DsInterference>.Iterator it = new(li);
        for (; it.More(); it.Next())
        {
            DsInterference i = it.Value();
            StoreInterference(i, lids);
        }
    }

    /// <summary>Empties the interferences of the shape <paramref name="s"/>, then stores those of <paramref name="li"/>.</summary>
    internal void ClearStoreInterferences(NCollection_List<DsInterference> li, in Shape s, string str = "")
    {
        _ = str;
        NCollection_List<DsInterference> lids = _ds.ChangeShapeInterferences(s);
        lids.Clear();
        NCollection_List<DsInterference>.Iterator it = new(li);
        for (; it.More(); it.Next())
        {
            DsInterference i = it.Value();
            StoreInterference(i, lids);
        }
    }
}
