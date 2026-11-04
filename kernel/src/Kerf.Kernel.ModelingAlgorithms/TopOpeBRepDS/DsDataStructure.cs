// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_DataStructure.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_DataStructure.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Legacy.Tools;
using Kerf.Engine.Collections;
using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Booleans.Legacy.Data.DsConfig;

namespace Kerf.Engine.Booleans.Legacy.Data;

/// <summary>
/// The legacy topological data structure: the new geometries (surfaces, curves, points) a topological operation
/// creates, the shapes they touch, and the interferences that attach each to the others.
/// </summary>
/// <remarks>
/// <para>
/// Surfaces, curves and points are numbered from 1 in the order added; a removed curve or surface keeps its number
/// (unbound, or not kept). Shapes are numbered from 1 in the order added (an indexed map keyed by
/// <see cref="ShapeHasher"/>: same TShape and location, any orientation).
/// </para>
/// <para>
/// A working object, internal to the fillet builders and to unit 7R's reconstruction (plan 2.1): the mutators are
/// internal; the queries are public, so a caller can read a builder's data structure. The <c>Change*</c> accessors
/// give the stored value for writing (by <c>ref</c> for the value types <see cref="DsCurve"/>,
/// <see cref="DsPoint"/> and <see cref="DsSurface"/>; the lists themselves for interferences).
/// </para>
/// <para>
/// As in OCCT, an accessor asked for a geometry or shape that isn't there returns the data structure's own empty
/// object, and the <c>Change*</c> accessors return it for writing: what is written there is seen by later calls that
/// miss (decision 1066). OCCT's two function-scope statics (the empty shape-with-state map and value) are members here.
/// </para>
/// </remarks>
internal sealed class DsDataStructure
{
    private int _nbSurfaces;
    internal readonly Dictionary<int, DsSurfaceData> _surfaces = new();
    private int _nbCurves;
    internal readonly Dictionary<int, DsCurveData> _curves = new();
    private int _nbPoints;
    internal readonly Dictionary<int, DsPointData> _points = new();
    private readonly IndexedMap<Shape, DsShapeData> _shapes = new(ShapeHasher.Instance);
    private readonly IndexedSet<Shape> _sectionEdges = new(ShapeHasher.Instance);
    private readonly NCollection_List<DsInterference> _emptyListOfInterference = new();
    private readonly NCollection_List<Shape> _emptyListOfShape = new();
    private Shape _emptyShape = new();
    private DsPoint _emptyPoint = new();
    private DsSurface _emptySurface = new();
    private DsCurve _emptyCurve = new();
    private readonly Dictionary<Shape, Surface> _newSurface = new(ShapeHasher.Instance);
    private bool _isfafa;
    private int _i;
    private readonly IndexedMap<Shape, DsShapeWithState> _mapOfShapeWithStateObj = new(ShapeHasher.Instance);
    private readonly IndexedMap<Shape, DsShapeWithState> _mapOfShapeWithStateTool = new(ShapeHasher.Instance);
    private readonly IndexedSet<Shape> _mapOfRejectedShapesObj = new(ShapeHasher.Instance);
    private readonly IndexedSet<Shape> _mapOfRejectedShapesTool = new(ShapeHasher.Instance);

    // OCCT.NET: OCCT's function-scope statics of ChangeMapOfShapeWithState and GetShapeWithState (decision 1063)
    private readonly IndexedMap<Shape, DsShapeWithState> _dummyMapOfShapeWithState = new(ShapeHasher.Instance);
    private readonly DsShapeWithState _dummyShapeWithState = new();

    /// <summary>An empty data structure.</summary>
    public DsDataStructure()
    {
        _nbSurfaces = 0;
        _nbCurves = 0;
        _nbPoints = 0;
        _isfafa = false;
        _i = 0;
        _ = _i;
    }

    /// <summary>Empties the data structure (the new surfaces of faces and the fafa flag are kept, as in OCCT).</summary>
    internal void Init()
    {
        _nbSurfaces = 0;
        _nbCurves = 0;
        _nbPoints = 0;
        _surfaces.Clear();
        _curves.Clear();
        _points.Clear();
        _shapes.Clear();
        // Begin modified by NIZHNY-MZV  Tue Apr 18 16:33:26 2000
        _mapOfShapeWithStateObj.Clear();
        _mapOfShapeWithStateTool.Clear();
        _mapOfRejectedShapesObj.Clear();
        _mapOfRejectedShapesTool.Clear();
        // End modified by NIZHNY-MZV  Tue Apr 18 16:33:32 2000
        InitSectionEdges();
    }

    /// <summary>Adds a copy of the surface <paramref name="s"/>; returns its index.</summary>
    internal int AddSurface(in DsSurface s)
    {
        _nbSurfaces++;
        // modified by NIZNHY-PKV Tue Oct 30 09:22:04 2001 f
        DsSurfaceData sd = new(s);
        _surfaces[_nbSurfaces] = sd;
        // mySurfaces.Bind(myNbSurfaces,S);
        // modified by NIZNHY-PKV Tue Oct 30 09:22:20 2001 t
        return _nbSurfaces;
    }

    /// <summary>Removes the surface <paramref name="i"/> (its index is not reused).</summary>
    internal void RemoveSurface(int i)
    {
        _surfaces.Remove(i);
    }

    /// <summary>Whether the surface <paramref name="i"/> is kept.</summary>
    public bool KeepSurface(int i)
    {
        DsSurface s = Surface(i);
        bool b = s.Keep();
        return b;
    }

    /// <summary>Whether the surface <paramref name="s"/> is kept.</summary>
    public bool KeepSurface(in DsSurface s)
    {
        bool b = s.Keep();
        return b;
    }

    internal void ChangeKeepSurface(int i, bool findKeep)
    {
        ref DsSurface s = ref ChangeSurface(i);
        s.ChangeKeep(findKeep);
    }

    internal static void ChangeKeepSurface(ref DsSurface s, bool findKeep)
    {
        s.ChangeKeep(findKeep);
    }

    /// <summary>
    /// Adds a copy of the curve <paramref name="c"/>; returns its index. As in OCCT, <paramref name="c"/>'s own
    /// <see cref="DsCurve.DSIndex"/> is set to it first.
    /// </summary>
    internal int AddCurve(ref DsCurve c)
    {
        _nbCurves++;

        // NYI : modifier const & sur Curve dans le CDL NYI
        c.ChangeDSIndex(_nbCurves);

        DsCurveData cd = new(c);
        _curves[_nbCurves] = cd;
        return _nbCurves;
    }

    /// <summary>
    /// Adds a copy of the curve <paramref name="c"/> (OCCT's calls with a temporary); returns its index. Use the
    /// <c>ref</c> overload where the caller reads the curve's <see cref="DsCurve.DSIndex"/> afterwards.
    /// </summary>
    internal int AddCurve(DsCurve c) => AddCurve(ref c);

    /// <summary>Removes the curve <paramref name="i"/>: its two surface/curve interferences leave their shapes' lists, and the curve is no longer kept.</summary>
    internal void RemoveCurve(int i)
    {
        ref DsCurve c = ref ChangeCurve(i);
        c.GetShapes(out Shape s1, out Shape s2);
        DsInterference? i1 = c.GetSCI1();
        DsInterference? i2 = c.GetSCI2();
        if (i1 is not null)
        {
            RemoveShapeInterference(s1, i1);
        }
        if (i2 is not null)
        {
            RemoveShapeInterference(s2, i2);
        }
        c.ChangeKeep(false);

        // NYI : nullify interferences I1,I2
    }

    /// <summary>Whether the curve <paramref name="i"/> is kept.</summary>
    public bool KeepCurve(int i)
    {
        DsCurve c = Curve(i);
        bool b = c.Keep();
        return b;
    }

    /// <summary>Whether the curve <paramref name="c"/> is kept.</summary>
    public bool KeepCurve(in DsCurve c)
    {
        bool b = c.Keep();
        return b;
    }

    internal void ChangeKeepCurve(int i, bool findKeep)
    {
        ref DsCurve c = ref ChangeCurve(i);
        c.ChangeKeep(findKeep);
    }

    internal static void ChangeKeepCurve(ref DsCurve c, bool findKeep)
    {
        c.ChangeKeep(findKeep);
    }

    /// <summary>Adds a copy of the point <paramref name="pds"/>; returns its index.</summary>
    internal int AddPoint(in DsPoint pds)
    {
        _nbPoints++;
        _points[_nbPoints] = new DsPointData(pds);
        return _nbPoints;
    }

    /// <summary>Adds a copy of the point <paramref name="pds"/> (the shapes are ignored, as in OCCT); returns its index.</summary>
    internal int AddPointSS(in DsPoint pds, in Shape s1, in Shape s2)
    {
        _ = s1;
        _ = s2;
        int i = AddPoint(pds);
        return i;
    }

    /// <summary>Marks the point <paramref name="i"/> as not kept.</summary>
    internal void RemovePoint(int i)
    {
        ref DsPoint p = ref ChangePoint(i);
        p.ChangeKeep(false);
    }

    /// <summary>Whether the point <paramref name="i"/> is kept.</summary>
    public bool KeepPoint(int i)
    {
        DsPoint p = Point(i);
        bool b = p.Keep();
        return b;
    }

    /// <summary>Whether the point <paramref name="p"/> is kept.</summary>
    public bool KeepPoint(in DsPoint p)
    {
        bool b = p.Keep();
        return b;
    }

    internal void ChangeKeepPoint(int i, bool findKeep)
    {
        ref DsPoint p = ref ChangePoint(i);
        p.ChangeKeep(findKeep);
    }

    internal static void ChangeKeepPoint(ref DsPoint p, bool findKeep)
    {
        p.ChangeKeep(findKeep);
    }

    /// <summary>Adds the shape <paramref name="s"/> if it is new (its own same-domain reference); returns its index.</summary>
    internal int AddShape(in Shape s)
    {
        int iS = _shapes.IndexOf(s) + 1;
        if (iS == 0)
        {
            DsShapeData sd = new();
            iS = _shapes.Add(s, sd) + 1;
            // a shape is its own reference, oriented as itself
            SameDomainRef(iS, iS);
            SameDomainOri(iS, DsConfig.Sameoriented);
        }
        return iS;
    }

    /// <summary>
    /// Adds the shape <paramref name="s"/> if it is new, with the ancestor rank <paramref name="ianc"/>; returns its
    /// index. The rank of a shape already there is not changed.
    /// </summary>
    internal int AddShape(in Shape s, int ianc)
    {
        int iS = _shapes.IndexOf(s) + 1;
        if (iS == 0)
        {
            DsShapeData sd = new();
            iS = _shapes.Add(s, sd) + 1;
            // a shape is its own reference, oriented as itself
            SameDomainRef(iS, iS);
            SameDomainOri(iS, DsConfig.Sameoriented);
            AncestorRank(iS, ianc);
        }
        return iS;
    }

    /// <summary>Whether the shape <paramref name="i"/> exists and (if <paramref name="findKeep"/>) is kept.</summary>
    public bool KeepShape(int i, bool findKeep = true)
    {
        bool b = false;
        if (i >= 1 && i <= _shapes.Count)
        {
            Shape s = _shapes.KeyAt(i - 1);
            if (findKeep)
            {
                b = KeepShape(s);
            }
            else
            {
                b = true;
            }
        }
        return b;
    }

    /// <summary>Whether the shape <paramref name="s"/> is in the data structure and (if <paramref name="findKeep"/>) is kept.</summary>
    public bool KeepShape(in Shape s, bool findKeep = true)
    {
        bool b = false;
        if ((_shapes.Count != 0) && (!s.IsNull) && _shapes.ContainsKey(s))
        {
            DsShapeData sd = _shapes.GetValue(s);
            if (findKeep)
            {
                b = sd.Keep();
            }
            else
            {
                b = true;
            }
        }
        return b;
    }

    internal void ChangeKeepShape(int i, bool findKeep)
    {
        if (i >= 1 && i <= _shapes.Count)
        {
            Shape s = _shapes.KeyAt(i - 1);
            ChangeKeepShape(s, findKeep);
        }
    }

    internal void ChangeKeepShape(in Shape s, bool findKeep)
    {
        if (!s.IsNull && _shapes.ContainsKey(s))
        {
            DsShapeData sd = _shapes.GetValue(s);
            sd.ChangeKeep(findKeep);
        }
    }

    internal void InitSectionEdges()
    {
        _sectionEdges.Clear();
    }

    /// <summary>Adds the section edge <paramref name="e"/> if it is new; returns its index among the section edges.</summary>
    internal int AddSectionEdge(in Edge e)
    {
        int iE = _sectionEdges.IndexOf(e) + 1;
        if (iE == 0)
        {
            iE = _sectionEdges.Add(e) + 1;
        }
        return iE;
    }

    /// <summary>The map of the shapes and their data.</summary>
    internal IndexedMap<Shape, DsShapeData> ChangeShapes() => _shapes;

    /// <summary>The interferences of the surface <paramref name="i"/> (the shared empty list if there is none).</summary>
    public NCollection_List<DsInterference> SurfaceInterferences(int i)
    {
        if (!_surfaces.ContainsKey(i))
        {
            return _emptyListOfInterference;
        }
        DsSurfaceData sd = _surfaces.GetValue(i);
        NCollection_List<DsInterference> li = sd.Interferences();
        return li;
    }

    internal NCollection_List<DsInterference> ChangeSurfaceInterferences(int i)
    {
        if (!_surfaces.ContainsKey(i))
        {
            return _emptyListOfInterference;
        }
        DsSurfaceData sd = _surfaces.GetValue(i);
        NCollection_List<DsInterference> li = sd.ChangeInterferences();
        return li;
    }

    /// <summary>The interferences of the curve <paramref name="i"/> (the shared empty list if there is none).</summary>
    public NCollection_List<DsInterference> CurveInterferences(int i)
    {
        if (!_curves.ContainsKey(i))
        {
            return _emptyListOfInterference;
        }
        DsCurveData cd = _curves.GetValue(i);
        NCollection_List<DsInterference> li = cd.Interferences();
        return li;
    }

    internal NCollection_List<DsInterference> ChangeCurveInterferences(int i)
    {
        if (!_curves.ContainsKey(i))
        {
            return _emptyListOfInterference;
        }
        DsCurveData cd = _curves.GetValue(i);
        NCollection_List<DsInterference> li = cd.ChangeInterferences();
        return li;
    }

    /// <summary>The interferences of the point <paramref name="i"/> (the shared empty list if there is none).</summary>
    public NCollection_List<DsInterference> PointInterferences(int i)
    {
        if (!_points.ContainsKey(i))
        {
            return _emptyListOfInterference;
        }
        DsPointData pd = _points.GetValue(i);
        NCollection_List<DsInterference> li = pd.Interferences();
        return li;
    }

    internal NCollection_List<DsInterference> ChangePointInterferences(int i)
    {
        if (!_points.ContainsKey(i))
        {
            return _emptyListOfInterference;
        }
        DsPointData pd = _points.GetValue(i);
        NCollection_List<DsInterference> li = pd.ChangeInterferences();
        return li;
    }

    /// <summary>The interferences of the shape <paramref name="s"/> (the shared empty list if it isn't there or kept).</summary>
    public NCollection_List<DsInterference> ShapeInterferences(in Shape s, bool findKeep = true)
    {
        if (HasShape(s, findKeep))
        {
            return _shapes.GetValue(s)._interferences;
        }
        return _emptyListOfInterference;
    }

    internal NCollection_List<DsInterference> ChangeShapeInterferences(in Shape s)
    {
        if (!HasShape(s))
        {
            // OCCT: the shared empty list, for writing (decision 1066)
            return _emptyListOfInterference;
        }
        DsShapeData sd = _shapes.GetValue(s);
        return sd._interferences;
    }

    /// <summary>The interferences of the shape <paramref name="i"/> (the shared empty list if it isn't kept).</summary>
    public NCollection_List<DsInterference> ShapeInterferences(int i, bool findKeep = true)
    {
        if (findKeep && !KeepShape(i))
        {
            return _emptyListOfInterference;
        }
        return _shapes.ValueAt(i - 1)._interferences;
    }

    internal NCollection_List<DsInterference> ChangeShapeInterferences(int i)
    {
        DsShapeData sd = _shapes.ValueAt(i - 1);
        return sd._interferences;
    }

    /// <summary>The shapes sharing the geometrical domain of <paramref name="s"/>.</summary>
    public NCollection_List<Shape> ShapeSameDomain(in Shape s)
    {
        if (!s.IsNull)
        {
            if (_shapes.ContainsKey(s))
            {
                DsShapeData sd = _shapes.GetValue(s);
                NCollection_List<Shape> l = sd._sameDomain;
                return l;
            }
        }
        return _emptyListOfShape;
    }

    internal NCollection_List<Shape> ChangeShapeSameDomain(in Shape s)
    {
        DsShapeData sd = _shapes.GetValue(s);
        return sd._sameDomain;
    }

    /// <summary>The shapes sharing the geometrical domain of the shape <paramref name="i"/>.</summary>
    public NCollection_List<Shape> ShapeSameDomain(int i)
    {
        if (i >= 1 && i <= _shapes.Count)
        {
            DsShapeData sd = _shapes.ValueAt(i - 1);
            NCollection_List<Shape> l = sd._sameDomain;
            return l;
        }
        else
        {
            return _emptyListOfShape;
        }
    }

    internal NCollection_List<Shape> ChangeShapeSameDomain(int i)
    {
        DsShapeData sd = _shapes.ValueAt(i - 1);
        return sd._sameDomain;
    }

    internal void AddShapeSameDomain(in Shape s, in Shape ssd)
    {
        bool append = true;
        {
            NCollection_List<Shape>.Iterator it = new(ShapeSameDomain(s));
            for (; it.More(); it.Next())
            {
                Shape itS = it.Value();
                if (itS.IsSame(ssd))
                {
                    append = false;
                    break;
                }
            }
        }
        if (append)
        {
            ChangeShapeSameDomain(s).Append(ssd);
        }
    }

    internal void RemoveShapeSameDomain(in Shape s, in Shape ssd)
    {
        NCollection_List<Shape> l = ChangeShapeSameDomain(s);
        NCollection_List<Shape>.Iterator it = new(l);
        while (it.More())
        {
            Shape itS = it.Value();
            bool remove = itS.IsSame(ssd);
            if (remove)
            {
                l.Remove(it);
            }
            else
            {
                it.Next();
            }
        }
    }

    /// <summary>The index of the same-domain reference of the shape <paramref name="i"/>, or 0.</summary>
    public int SameDomainRef(int i)
    {
        if (i >= 1 && i <= _shapes.Count)
        {
            return _shapes.ValueAt(i - 1)._sameDomainRef;
        }
        return 0;
    }

    /// <summary>The index of the same-domain reference of the shape <paramref name="s"/>, or 0.</summary>
    public int SameDomainRef(in Shape s)
    {
        if (s.IsNull)
        {
            return 0;
        }
        if (_shapes.ContainsKey(s))
        {
            return _shapes.GetValue(s)._sameDomainRef;
        }
        return 0;
    }

    internal void SameDomainRef(int i, int Ref)
    {
        if (i >= 1 && i <= _shapes.Count)
        {
            DsShapeData sd = _shapes.ValueAt(i - 1);
            sd._sameDomainRef = Ref;
        }
    }

    internal void SameDomainRef(in Shape s, int Ref)
    {
        if (s.IsNull)
        {
            return;
        }
        if (_shapes.ContainsKey(s))
        {
            DsShapeData sd = _shapes.GetValue(s);
            sd._sameDomainRef = Ref;
        }
    }

    /// <summary>How the shape <paramref name="i"/> lies with respect to its same-domain reference.</summary>
    public DsConfig SameDomainOri(int i)
    {
        if (i >= 1 && i <= _shapes.Count)
        {
            return _shapes.ValueAt(i - 1)._sameDomainOri;
        }
        return DsConfig.Unshgeometry;
    }

    /// <summary>How the shape <paramref name="s"/> lies with respect to its same-domain reference.</summary>
    public DsConfig SameDomainOri(in Shape s)
    {
        if (!s.IsNull)
        {
            if (_shapes.ContainsKey(s))
            {
                return _shapes.GetValue(s)._sameDomainOri;
            }
        }
        return DsConfig.Unshgeometry;
    }

    internal void SameDomainOri(int i, DsConfig ori)
    {
        if (i >= 1 && i <= _shapes.Count)
        {
            DsShapeData sd = _shapes.ValueAt(i - 1);
            sd._sameDomainOri = ori;
        }
    }

    internal void SameDomainOri(in Shape s, DsConfig ori)
    {
        if (!s.IsNull)
        {
            if (_shapes.ContainsKey(s))
            {
                DsShapeData sd = _shapes.GetValue(s);
                sd._sameDomainOri = ori;
            }
        }
    }

    /// <summary>The same-domain index (1 or 2: which argument) of the shape <paramref name="i"/>, or 0.</summary>
    public int SameDomainInd(int i)
    {
        if (i >= 1 && i <= _shapes.Count)
        {
            return _shapes.ValueAt(i - 1)._sameDomainInd;
        }
        return 0;
    }

    /// <summary>The same-domain index of the shape <paramref name="s"/>, or 0.</summary>
    public int SameDomainInd(in Shape s)
    {
        if (!s.IsNull)
        {
            if (_shapes.ContainsKey(s))
            {
                return _shapes.GetValue(s)._sameDomainInd;
            }
        }
        return 0;
    }

    internal void SameDomainInd(int i, int ind)
    {
        if (i >= 1 && i <= _shapes.Count)
        {
            DsShapeData sd = _shapes.ValueAt(i - 1);
            sd._sameDomainInd = ind;
        }
    }

    internal void SameDomainInd(in Shape s, int ind)
    {
        if (s.IsNull)
        {
            return;
        }
        if (_shapes.ContainsKey(s))
        {
            DsShapeData sd = _shapes.GetValue(s);
            sd._sameDomainInd = ind;
        }
    }

    /// <summary>The ancestor rank (the argument it comes from) of the shape <paramref name="i"/>, or 0.</summary>
    public int AncestorRank(int i)
    {
        if (i >= 1 && i <= _shapes.Count)
        {
            return _shapes.ValueAt(i - 1)._ancestorRank;
        }
        return 0;
    }

    /// <summary>The ancestor rank of the shape <paramref name="s"/>, or 0.</summary>
    public int AncestorRank(in Shape s)
    {
        if (s.IsNull)
        {
            return 0;
        }
        if (_shapes.ContainsKey(s))
        {
            return _shapes.GetValue(s)._ancestorRank;
        }
        return 0;
    }

    internal void AncestorRank(int i, int ianc)
    {
        if (i >= 1 && i <= _shapes.Count)
        {
            DsShapeData sd = _shapes.ValueAt(i - 1);
            sd._ancestorRank = ianc;
        }
    }

    internal void AncestorRank(in Shape s, int ianc)
    {
        if (s.IsNull)
        {
            return;
        }
        if (_shapes.ContainsKey(s))
        {
            DsShapeData sd = _shapes.GetValue(s);
            sd._ancestorRank = ianc;
        }
    }

    /// <summary>Appends the interference <paramref name="i"/> to the shape's list (to the shared empty list if the shape isn't there).</summary>
    internal void AddShapeInterference(in Shape s, DsInterference i)
    {
        ChangeShapeInterferences(s).Append(i);
    }

    internal void RemoveShapeInterference(in Shape s, DsInterference i)
    {
        NCollection_List<DsInterference> l = ChangeShapeInterferences(s);
        NCollection_List<DsInterference>.Iterator it = new(l);
        bool b = FindInterference(it, i);
        if (b)
        {
            l.Remove(it);
        }
    }

    /// <summary>
    /// Records that <paramref name="s1"/> (of the first argument) and <paramref name="s2"/> (of the second) share
    /// their geometrical domain, and gives both the same reference.
    /// </summary>
    internal void FillShapesSameDomain(in Shape s1, in Shape s2, bool refFirst = true)
    {
        Orientation o1 = s1.Orientation;
        Orientation o2 = s2.Orientation;

        int iS1 = AddShape(s1, 1);
        DsShapeData sd1 = _shapes.ValueAt(iS1 - 1);
        bool isdef1 = sd1._orientationDef;
        bool todef1 = (!isdef1);
        if (isdef1 && sd1._sameDomainOri == DsConfig.Unshgeometry)
        {
            todef1 = true;
        }
        if (todef1)
        {
            sd1._orientation = o1;
            sd1._orientationDef = true;
        }

        int iS2 = AddShape(s2, 2);
        DsShapeData sd2 = _shapes.ValueAt(iS2 - 1);
        bool isdef2 = sd2._orientationDef;
        bool todef2 = (!isdef2);
        if (isdef2 && sd2._sameDomainOri == DsConfig.Unshgeometry)
        {
            todef2 = true;
        }
        if (todef2)
        {
            sd2._orientation = o2;
            sd2._orientationDef = true;
        }

        int n1 = ShapeSameDomain(s1).Extent();
        int n2 = ShapeSameDomain(s2).Extent();

        AddShapeSameDomain(s1, s2);
        AddShapeSameDomain(s2, s1);

        int n11 = ShapeSameDomain(s1).Extent();
        int n22 = ShapeSameDomain(s2).Extent();

        bool cond = (n11 == n1) && (n22 == n2);
        cond = cond && (!todef1) && (!todef2);
        if (cond)
        {
            // nothing changed in SameDomain data of S1 and S2 : return
            return;
        }

        int r1 = SameDomainRef(s1);
        int r2 = SameDomainRef(s2);
        int r = 0;
        // r1 == i1  r2 == i2 : shapes have no SameDomain ref : take S1 as reference
        // r1 == i1  r2 != i2 : S2 has a SameDomain reference : give it to S1
        // r1 != i1  r2 == i2 : S1 has a SameDomain reference : give it to S2
        // r1 != i1  r2 != i2 : S1,S2 have SameDomain reference : check equality
        if (r1 == iS1 && r2 == iS2)
        {
            r = (refFirst ? iS1 : iS2);
        }
        else if (r1 == iS1 && r2 != iS2)
        {
            r = r2;
        }
        else if (r1 != iS1 && r2 == iS2)
        {
            r = r1;
        }
        else if (r1 != iS1 && r2 != iS2)
        {
            if (r1 != r2)
            {
                //      throw Standard_ProgramError("FacesFiller::Insert SD 1");
            }
            r = (refFirst ? r1 : r2);
        }

        if (r == 0)
        {
            throw new ProgramErrorException("FacesFiller::Insert SD 2");
        }

        Shape sr = Shape(r);
        DsShapeData sd = _shapes.GetValue(sr);
        Orientation oSr = sd._orientation;
        sr.Orient(oSr);

        if (r != r1 || todef1)
        { // S1 gets a new reference r
            DsConfig o = DsConfig.Sameoriented;
            if (r != iS1 || todef1)
            {
                bool sso = ToolShapeTool.ShapesSameOriented(s1, sr);
                if (!sso)
                {
                    o = DsConfig.Difforiented;
                }
            }
            SameDomainRef(iS1, r);
            SameDomainOri(iS1, o);
        }

        if (r != r2 || todef2)
        { // S2 gets a new reference r
            DsConfig o = DsConfig.Sameoriented;
            if (r != iS2 || todef2)
            {
                bool sso = ToolShapeTool.ShapesSameOriented(s2, sr);
                if (!sso)
                {
                    o = DsConfig.Difforiented;
                }
            }
            SameDomainRef(iS2, r);
            SameDomainOri(iS2, o);
        }

        // index
        SameDomainInd(s1, 1);
        SameDomainInd(s2, 2);
    }

    /// <summary>As the other overload, with the configurations <paramref name="c1"/> and <paramref name="c2"/> given.</summary>
    internal void FillShapesSameDomain(in Shape s1,
                                       in Shape s2,
                                       DsConfig c1,
                                       DsConfig c2,
                                       bool refFirst = true)
    {
        int iS1 = AddShape(s1, 1);
        DsShapeData sd1 = _shapes.ValueAt(iS1 - 1);
        bool isdef1 = sd1._orientationDef;
        bool todef1 = true;
        if (c1 == DsConfig.Unshgeometry && isdef1)
        {
            todef1 = false;
        }

        int iS2 = AddShape(s2, 2);
        DsShapeData sd2 = _shapes.ValueAt(iS2 - 1);
        bool isdef2 = sd2._orientationDef;
        bool todef2 = true;
        if (c2 == DsConfig.Unshgeometry && isdef2)
        {
            todef2 = false;
        }

        if (todef1 || todef2)
        {
            FillShapesSameDomain(s1, s2, refFirst);
        }

        if (todef1 && c1 == DsConfig.Unshgeometry)
        {
            SameDomainOri(s1, DsConfig.Unshgeometry);
        }

        if (todef2 && c2 == DsConfig.Unshgeometry)
        {
            SameDomainOri(s2, DsConfig.Unshgeometry);
        }
    }

    internal void UnfillShapesSameDomain(in Shape s1, in Shape s2)
    {
        RemoveShapeSameDomain(s1, s2);
        RemoveShapeSameDomain(s2, s1);
    }

    /// <summary>The number of surfaces added (removed ones included).</summary>
    public int NbSurfaces()
    {
        int n = _nbSurfaces;
        return n;
    }

    /// <summary>The number of curves added (removed ones included).</summary>
    public int NbCurves()
    {
        int n = _nbCurves;
        return n;
    }

    internal void ChangeNbCurves(int n)
    {
        _nbCurves = n;
    }

    /// <summary>The number of points added (removed ones included).</summary>
    public int NbPoints()
    {
        int n = _nbPoints;
        return n;
    }

    /// <summary>The number of shapes.</summary>
    public int NbShapes()
    {
        int n = _shapes.Count;
        return n;
    }

    /// <summary>The number of section edges.</summary>
    public int NbSectionEdges()
    {
        int n = _sectionEdges.Count;
        return n;
    }

    /// <summary>The surface <paramref name="i"/> (the data structure's empty surface if there is none).</summary>
    public DsSurface Surface(int i)
    {
        if (_surfaces.ContainsKey(i))
        {
            return _surfaces.GetValue(i)._surface;
        }
        else
        {
            return _emptySurface;
        }
    }

    /// <summary>The surface <paramref name="i"/> for writing (the data structure's empty surface if there is none).</summary>
    internal ref DsSurface ChangeSurface(int i)
    {
        if (_surfaces.ContainsKey(i))
        {
            return ref _surfaces.GetValue(i)._surface;
        }
        else
        {
            return ref _emptySurface;
        }
    }

    /// <summary>The curve <paramref name="i"/> (the data structure's empty curve if there is none).</summary>
    public DsCurve Curve(int i)
    {
        if (_curves.ContainsKey(i))
        {
            DsCurveData cd = _curves.GetValue(i);
            DsCurve c = cd._curve;
            return c;
        }
        else
        {
            return _emptyCurve;
        }
    }

    /// <summary>The curve <paramref name="i"/> for writing (the data structure's empty curve if there is none).</summary>
    internal ref DsCurve ChangeCurve(int i)
    {
        if (_curves.ContainsKey(i))
        {
            DsCurveData cd = _curves.GetValue(i);
            return ref cd._curve;
        }
        return ref _emptyCurve;
    }

    /// <summary>The point <paramref name="i"/>.</summary>
    /// <exception cref="ProgramErrorException"><paramref name="i"/> is not between 1 and <see cref="NbPoints"/>.</exception>
    public DsPoint Point(int i)
    {
        if (i < 1 || i > _nbPoints)
        {
            throw new ProgramErrorException("TopOpeBRepDS_DataStructure::Point");
        }

        if (_points.ContainsKey(i))
        {
            return _points.GetValue(i)._point;
        }
        else
        {
            return _emptyPoint;
        }
    }

    /// <summary>The point <paramref name="i"/> for writing.</summary>
    /// <exception cref="ProgramErrorException"><paramref name="i"/> is not between 1 and <see cref="NbPoints"/>.</exception>
    internal ref DsPoint ChangePoint(int i)
    {
        if (i < 1 || i > _nbPoints)
        {
            throw new ProgramErrorException("TopOpeBRepDS_DataStructure::Point");
        }

        if (_points.ContainsKey(i))
        {
            return ref _points.GetValue(i)._point;
        }
        else
        {
            return ref _emptyPoint;
        }
    }

    /// <summary>The shape <paramref name="i"/>, or a null shape if it isn't there or (if <paramref name="findKeep"/>) kept.</summary>
    public Shape Shape(int i, bool findKeep = true)
    {
        if (KeepShape(i, findKeep))
        {
            Shape s = _shapes.KeyAt(i - 1);
            return s;
        }
        return _emptyShape;
    }

    /// <summary>The index of the shape <paramref name="s"/>, or 0 if it isn't there or (if <paramref name="findKeep"/>) kept.</summary>
    public int Shape(in Shape s, bool findKeep = true)
    {
        int i = 0;
        bool hs = HasShape(s, findKeep);
        if (hs)
        {
            i = _shapes.IndexOf(s) + 1;
        }
        return i;
    }

    /// <summary>The section edge <paramref name="i"/>, or a null edge if it isn't a kept shape of the data structure.</summary>
    public Edge SectionEdge(int i, bool findKeep = true)
    {
        Shape s = _sectionEdges[i - 1];
        if (HasShape(s, findKeep))
        {
            return ShapeCasts.Edge(s);
        }
        return ShapeCasts.Edge(_emptyShape);
    }

    /// <summary>The index of the section edge <paramref name="s"/>, or 0.</summary>
    public int SectionEdge(in Edge s, bool findKeep = true)
    {
        int i = 0;
        if (KeepShape(s, findKeep))
        {
            i = _sectionEdges.IndexOf(s) + 1;
        }
        return i;
    }

    /// <summary>Whether <paramref name="s"/> is a section edge.</summary>
    public bool IsSectionEdge(in Edge s, bool findKeep = true)
    {
        bool b = KeepShape(s, findKeep);
        if (b)
        {
            b = _sectionEdges.Contains(s);
        }
        return b;
    }

    /// <summary>Whether the shape <paramref name="s"/> is in the data structure with at least one interference.</summary>
    public bool HasGeometry(in Shape s)
    {
        bool has = HasShape(s);
        if (has)
        {
            has = !ShapeInterferences(s).IsEmpty();
        }
        return has;
    }

    /// <summary>Whether the shape <paramref name="s"/> is in the data structure and (if <paramref name="findKeep"/>) kept.</summary>
    public bool HasShape(in Shape s, bool findKeep = true)
    {
        bool res = KeepShape(s, findKeep);
        return res;
    }

    /// <summary>Sets the new surface of the face <paramref name="f"/>.</summary>
    internal void SetNewSurface(in Shape f, Surface s)
    {
        _newSurface[f] = s;
    }

    /// <summary>Whether the face <paramref name="f"/> has a new surface.</summary>
    public bool HasNewSurface(in Shape f)
    {
        bool b = _newSurface.ContainsKey(f);
        return b;
    }

    /// <summary>The new surface of the face <paramref name="f"/>, or null.</summary>
    public Surface? NewSurface(in Shape f)
    {
        if (HasNewSurface(f))
        {
            return _newSurface.GetValue(f);
        }
        return null;
    }

    private static bool FindInterference(NCollection_List<DsInterference>.Iterator it,
                                         DsInterference i)
    {
        for (; it.More(); it.Next())
        {
            if (ReferenceEquals(it.Value(), i))
            {
                return true;
            }
        }
        return false;
    }

    internal void Isfafa(bool isfafa)
    {
        _isfafa = isfafa;
    }

    /// <summary>Whether the data structure comes from a face/face operation.</summary>
    public bool Isfafa() => _isfafa;

    internal IndexedMap<Shape, DsShapeWithState> ChangeMapOfShapeWithStateObj()
        => _mapOfShapeWithStateObj;

    internal IndexedMap<Shape, DsShapeWithState> ChangeMapOfShapeWithStateTool()
        => _mapOfShapeWithStateTool;

    /// <summary>
    /// The map of shapes with state (the object's or the tool's) that holds <paramref name="aShape"/>; if neither does,
    /// <paramref name="aFlag"/> is false and the result is an empty map.
    /// </summary>
    /// <remarks>OCCT returns a function-scope static map, shared by every data structure; the port's is this one's (decision 1063).</remarks>
    internal IndexedMap<Shape, DsShapeWithState> ChangeMapOfShapeWithState(in Shape aShape,
                                                                                                             out bool aFlag)
    {
        aFlag = true;

        if (_mapOfShapeWithStateObj.ContainsKey(aShape))
        {
            return _mapOfShapeWithStateObj;
        }

        if (_mapOfShapeWithStateTool.ContainsKey(aShape))
        {
            return _mapOfShapeWithStateTool;
        }

        aFlag = false;
        return _dummyMapOfShapeWithState;
    }

    /// <summary>The shape with state of <paramref name="aShape"/>, or an empty one.</summary>
    public DsShapeWithState GetShapeWithState(in Shape aShape)
    {
        if (_mapOfShapeWithStateObj.ContainsKey(aShape))
        {
            return _mapOfShapeWithStateObj.GetValue(aShape);
        }
        else if (_mapOfShapeWithStateTool.ContainsKey(aShape))
        {
            return _mapOfShapeWithStateTool.GetValue(aShape);
        }
        return _dummyShapeWithState;
    }

    internal IndexedSet<Shape> ChangeMapOfRejectedShapesObj() => _mapOfRejectedShapesObj;

    internal IndexedSet<Shape> ChangeMapOfRejectedShapesTool() => _mapOfRejectedShapesTool;
}
