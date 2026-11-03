// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepTool/TopOpeBRepTool_ShapeClassifier.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepTool/TopOpeBRepTool_ShapeClassifier.cxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepTool/TopOpeBRepTool_PShapeClassifier.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Classification;
using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;
using static Kerf.Topology.Containment;

namespace Kerf.Engine.Booleans.Legacy.Tools;

/// <summary>
/// Classifies a shape against a reference shape (a face, a solid or a shell): through a point of one of its edges (2D
/// classification in a reference face when the two are of the same domain, 3D otherwise) or a point of a face.
/// </summary>
/// <remarks>
/// <para>
/// A working object. OCCT's default constructor leaves <c>mySameDomain</c>, <c>mymren</c>, <c>mymredone</c>,
/// <c>myState</c> and <c>myPAvLS</c> uninitialised; the port's are zero, null and false (marked <c>// OCCT</c>).
/// Every caller sets them (SetReference, SameDomain, StateShapeReference) before they are read.
/// </para>
/// <para>The list of shapes to avoid (<c>myPAvLS</c>, a pointer in OCCT) is a reference to the caller's list.</para>
/// </remarks>
public sealed class ShapeClassifier
{
    private Shape _s = new();
    private Shape _ref = new();
    private Shape _avS = new();
    private NCollection_List<Shape>? _pAvLS;
    private readonly NCollection_IndexedMap<Shape> _mapAvS = new(ShapeHasher.Instance);
    private readonly NCollection_IndexedMap<Shape> _mymre = new(ShapeHasher.Instance);
    private int _mymren;
    private bool _mymredone;
    private Containment _state;
    private Edge _edge = new();
    private Face _face = new();
    private bool _p3Ddef;
    private Point3D _p3D = new();
    private bool _p2Ddef;
    private Point2D _p2D = new();
    private readonly ToolSolidClassifier _solidClassifier = new();
    private int _sameDomain;

    /// <summary>A classifier without a reference shape.</summary>
    public ShapeClassifier()
    {
        // OCCT: mySameDomain, mymren, mymredone, myState and myPAvLS are left uninitialised (zero, false and null here)
        _p3Ddef = false;
        _p2Ddef = false;
    }

    /// <summary>A classifier with the reference shape <paramref name="sRef"/>.</summary>
    public ShapeClassifier(in Shape sRef)
    {
        _p3Ddef = false;
        _p2Ddef = false;
        _ref = sRef;
    }

    /// <summary>Resets everything, the solid classifier included.</summary>
    public void ClearAll()
    {
        ClearCurrent();
        _solidClassifier.Clear();
    }

    /// <summary>Resets everything but the solid classifier.</summary>
    public void ClearCurrent()
    {
        _sameDomain = -1;
        _s.Nullify();
        _ref.Nullify();
        _avS.Nullify();
        _mapAvS.Clear();
        _mymre.Clear();
        _mymren = 0;
        _mymredone = false;
        _state = Unknown;
        _edge.Nullify();
        _face.Nullify();
        _p3Ddef = _p2Ddef = false;
    }

    /// <summary>The same-domain mode (1: the shape and the reference are of the same domain).</summary>
    public int SameDomain() => _sameDomain;

    /// <summary>Sets the same-domain mode for the next classification.</summary>
    public void SameDomain(int sam)
    {
        _sameDomain = sam;
    }

    /// <summary>Sets the reference shape.</summary>
    public void SetReference(in Shape sRef)
    {
        _ref = sRef;
        MapRef();
    }

    private void MapRef()
    {
        _mymre.Clear();
        _mymren = 0;
        if (_ref.ShapeType() == ShapeType.Face && _sameDomain == 1)
        {
            Exploration.MapShapes(_ref, ShapeType.Edge, _mymre);
            _mymren = _mymre.Extent();
            if (_mymren == 1)
            {
                Explorer x = new(_ref, ShapeType.Edge);
                Edge e = ShapeCasts.Edge(x.Current());
                Exploration.Vertices(e, out Vertex v1, out Vertex v2);
                if (v1.IsSame(v2))
                {
                    _mymren = 0;
                }
            }
        }
        _mymredone = true;
    }

    /// <summary>The state of <paramref name="s"/> against <paramref name="sRef"/>.</summary>
    public Containment StateShapeShape(in Shape s, in Shape sRef, int samedomain = 0)
    {
        ClearCurrent();
        _sameDomain = samedomain;
        _s = s;
        _avS.Nullify();
        _pAvLS = null;
        _ref = sRef;
        Perform();
        return _state;
    }

    /// <summary>The state of <paramref name="s"/> against <paramref name="sRef"/>, avoiding <paramref name="avS"/>.</summary>
    public Containment StateShapeShape(in Shape s, in Shape avS, in Shape sRef)
    {
        ClearCurrent();
        _s = s;
        _avS = avS;
        _pAvLS = null;
        _ref = sRef;
        Perform();
        return _state;
    }

    /// <summary>The state of <paramref name="s"/> against <paramref name="sRef"/>, avoiding the shapes of <paramref name="avLS"/>.</summary>
    public Containment StateShapeShape(in Shape s, NCollection_List<Shape> avLS, in Shape sRef)
    {
        ClearCurrent();
        _s = s;
        _avS.Nullify();
        _pAvLS = avLS;
        _ref = sRef;
        Perform();
        return _state;
    }

    /// <summary>The state of <paramref name="s"/> against the reference shape, avoiding <paramref name="avS"/>.</summary>
    public Containment StateShapeReference(in Shape s, in Shape avS)
    {
        _s = s;
        _avS = avS;
        _pAvLS = null;
        Perform();
        return _state;
    }

    /// <summary>The state of <paramref name="s"/> against the reference shape, avoiding the shapes of <paramref name="avLS"/>.</summary>
    public Containment StateShapeReference(in Shape s, NCollection_List<Shape> avLS)
    {
        _s = s;
        _avS.Nullify();
        _pAvLS = avLS;
        Perform();
        return _state;
    }

    /// <summary>The solid classifier.</summary>
    public ToolSolidClassifier ChangeSolidClassifier() => _solidClassifier;

    private void FindEdge()
    {
        _edge.Nullify();
        _face.Nullify();

        ShapeType t = _s.ShapeType();
        if (t < ShapeType.Face)
        { // compsolid .. shell
            FindFace(_s);
            FindEdge(_face);
        }
        else
        {
            FindEdge(_s);
        }
    }

    private void FindEdge(in Shape s)
    {
        _edge.Nullify();
        bool isavls = HasAvLS();
        bool isavs = !_avS.IsNull;
        if (s.IsNull)
        {
            return;
        }

        ShapeType tS = s.ShapeType();
        Explorer eex = new();
        if (!_face.IsNull)
        {
            eex.Init(_face, ShapeType.Edge);
        }
        else
        {
            eex.Init(s, ShapeType.Edge);
        }

        for (; eex.More(); eex.Next())
        {
            Edge e = ShapeCasts.Edge(eex.Current());
            bool toavoid = false;
            if (isavls || isavs)
            {
                toavoid = toavoid || _mapAvS.Contains(e);
                if (!_avS.IsNull)
                {
                    toavoid = toavoid || e.IsSame(_avS);
                }
            }
            else if (Tool.Degenerated(e))
            {
                toavoid = tS != ShapeType.Edge;
            }
            if (toavoid)
            {
                continue;
            }
            _edge = e;
            break;
        }
    }

    private void FindFace(in Shape s)
    {
        _face.Nullify();
        bool isavls = HasAvLS();
        bool isavs = !_avS.IsNull;
        Explorer fex = new(s, ShapeType.Face);
        for (; fex.More(); fex.Next())
        {
            Face f = ShapeCasts.Face(fex.Current());
            bool toavoid = false;
            if (isavls || isavs)
            {
                toavoid = toavoid || _mapAvS.Contains(f);
                if (!_avS.IsNull)
                {
                    toavoid = toavoid || f.IsSame(_avS);
                }
            }
            if (toavoid)
            {
                continue;
            }
            _face = f;
            break;
        }
    }

    private void Perform()
    {
        _state = Unknown;
        if (_s.IsNull)
        {
            return;
        }
        if (_ref.IsNull)
        {
            return;
        }

        if (!_mymredone)
        {
            MapRef();
        }

        if (!_avS.IsNull)
        {
            // tAvS = FACE,EDGE --> map(AvS,EDGE)
            // rejection des aretes de myS comme arete de classification
            // (le point de classification doit etre IN S)
            _mapAvS.Clear();
            ShapeType tAvS = _avS.ShapeType();
            if (tAvS == ShapeType.Face)
            {
                _mapAvS.Add(_avS);
                Exploration.MapShapes(_avS, ShapeType.Edge, _mapAvS);
            }
            else if (tAvS == ShapeType.Edge)
            {
                Exploration.MapShapes(_avS, ShapeType.Edge, _mapAvS);
            }
        }
        else if (HasAvLS())
        {
            // tAvS = FACE,EDGE --> map(AvS,EDGE)
            _mapAvS.Clear();
            ShapeType tAvS = _pAvLS!.First().ShapeType();
            if (tAvS == ShapeType.Face)
            {
                NCollection_List<Shape>.Iterator it = new(_pAvLS);
                for (; it.More(); it.Next())
                {
                    Shape s = it.Value();
                    _mapAvS.Add(s);
                    Exploration.MapShapes(s, ShapeType.Edge, _mapAvS);
                }
            }
            else if (tAvS == ShapeType.Edge)
            {
                NCollection_List<Shape>.Iterator it = new(_pAvLS);
                for (; it.More(); it.Next())
                {
                    Shape s = it.Value();
                    Exploration.MapShapes(s, ShapeType.Edge, _mapAvS);
                }
            }
        }
        else
        {
            if (_s.ShapeType() == ShapeType.Face)
            {
                _p3Ddef = SolidExplorer.FindAPointInTheFace(ShapeCasts.Face(_s), ref _p3D);
            }
        }

        ShapeType tS = _s.ShapeType();
        ShapeType tR = _ref.ShapeType();

        if (tS == ShapeType.Vertex)
        {
            if (tR <= ShapeType.Solid)
            {
                Point3D p3D = Tool.Pnt(ShapeCasts.Vertex(_s));
                StateP3DReference(p3D);
            }
        }
        else if (tS == ShapeType.Edge)
        {
            if (tR == ShapeType.Face || tR <= ShapeType.Solid)
            {
                FindEdge();
                StateEdgeReference();
            }
        }
        else if (tS == ShapeType.Wire)
        {
            if (tR == ShapeType.Face || tR <= ShapeType.Solid)
            {
                FindEdge();
                StateEdgeReference();
            }
        }
        else if (tS == ShapeType.Face)
        {
            if (tR == ShapeType.Face)
            {
                FindEdge();
                if (_sameDomain == 1)
                {
                    StateEdgeReference();
                }
                else
                {
                    if (!_p3Ddef)
                    {
                        _p3Ddef = SolidExplorer.FindAPointInTheFace(ShapeCasts.Face(_s), ref _p3D);
                    }
                    if (_p3Ddef)
                    {
                        StateP3DReference(_p3D);
                    }
                    else
                    {
                        _state = Unknown;
                        throw new ProgramErrorException("TopOpeBRepTool_ShapeClassifier !P3Ddef");
                    }
                }
            }
            else if (tR <= ShapeType.Solid)
            {
                FindEdge();
                if (_p3Ddef)
                {
                    StateP3DReference(_p3D);
                }
                else
                {
                    StateEdgeReference();
                }
            }
        }
        else if (tS == ShapeType.Shell)
        {
            if (tR <= ShapeType.Solid)
            {
                FindEdge();
                StateEdgeReference();
            }
        }
        else if (tS == ShapeType.Solid)
        {
            if (tR <= ShapeType.Solid)
            {
                FindEdge();
                StateEdgeReference();
            }
        }
        else
        {
            throw new ProgramErrorException("StateShapeShape : bad operands");
        }

        // take orientation of reference shape in account
        Orientation oriRef = _ref.Orientation();
        if (oriRef == External || oriRef == Internal)
        {
            if (_state == In)
            {
                _state = Out;
            }
        }
    }

    private void StateEdgeReference()
    {
        _state = Unknown;

        if (_edge.IsNull)
        {
            return;
        }
        if (_ref.IsNull)
        {
            return;
        }

        Curve3D? c3D;
        Point3D p3D;
        Curve2D? c2D;
        Point2D p2D;

        ShapeType tR = _ref.ShapeType();

        if (tR == ShapeType.Face)
        {
            Face f = ShapeCasts.Face(_ref);
            if (_sameDomain != 0)
            {
                bool trimCurve = true;
                c2D = Legacy2D.FC2D_CurveOnSurface(_edge, f, out double f2d, out double l2d, out _, trimCurve);

                if (c2D is null)
                {
                    throw new ProgramErrorException("StateShapeShape : no 2d curve");
                }

                double t = 0.127956477;
                double p = (1 - t) * f2d + t * l2d;
                p2D = c2D.Value(p);

                StateP2DReference(p2D);
                return;
            }
            else
            { // myEdge/myRef=face en 3d
                c3D = Tool.Curve(_edge, out double f3d, out double l3d);

                if (c3D is null)
                {
                    throw new ProgramErrorException("StateShapeShape : no 3d curve");
                }

                double t = 0.127956477;
                double p = (1 - t) * f3d + t * l3d;
                p3D = c3D.Value(p);
                StateP3DReference(p3D);
                return;
            }
        }
        else if (tR <= ShapeType.Solid)
        {
            bool degen = Tool.Degenerated(_edge);
            if (degen)
            {
                Vertex v = Exploration.FirstVertex(_edge);
                p3D = Tool.Pnt(v);
                StateP3DReference(p3D);
                return;
            }
            else
            {
                c3D = Tool.Curve(_edge, out double f3d, out double l3d);

                if (c3D is null)
                {
                    throw new ProgramErrorException("StateShapeShape : no 3d curve");
                }

                double t = 0.127956477;
                double p = (1 - t) * f3d + t * l3d;
                p3D = c3D.Value(p);
                StateP3DReference(p3D);
                return;
            }
        }
        else
        {
            throw new ProgramErrorException("StateShapeShape : bad operands");
        }
    }

    /// <summary>Classifies the point <paramref name="p2D"/> in the reference face.</summary>
    public void StateP2DReference(in Point2D p2D)
    {
        _state = Unknown;
        if (_ref.IsNull)
        {
            return;
        }
        ShapeType tR = _ref.ShapeType();

        if (tR == ShapeType.Face)
        {
            if (_mymren == 1)
            {
                Explorer x = new();
                for (x.Init(_ref, ShapeType.Edge); x.More(); x.Next())
                {
                    Orientation o = x.Current().Orientation();
                    if (o == External)
                    {
                        _state = Out;
                    }
                    else if (o == Internal)
                    {
                        _state = In;
                    }
                    else
                    {
                        break;
                    }
                }
            }
            else
            {
                _p2D = p2D;
                _p2Ddef = true;
                Face f = ShapeCasts.Face(_ref);
                f.Orientation(Forward);
                double tolClass = 1e-8;
                FClass2d fClass2d = new(f, tolClass);
                _state = fClass2d.Perform(p2D);
            }
        }
        else
        {
            throw new ProgramErrorException("StateShapeShape : bad operands");
        }
    }

    /// <summary>Classifies the point <paramref name="p3D"/> in the reference solid (or the solids of the reference).</summary>
    public void StateP3DReference(in Point3D p3D)
    {
        _state = Unknown;
        if (_ref.IsNull)
        {
            return;
        }
        ShapeType tR = _ref.ShapeType();

        if (tR == ShapeType.Solid)
        {
            _p3D = p3D;
            _p3Ddef = true;
            Solid so = ShapeCasts.Solid(_ref);
            double tol3d = global::Kerf.Engine.Foundation.Precision.Confusion();
            _solidClassifier.Classify(so, p3D, tol3d);
            _state = _solidClassifier.State();
        }
        else if (tR < ShapeType.Solid)
        {
            _p3D = p3D;
            _p3Ddef = true;
            Explorer ex = new();
            for (ex.Init(_ref, ShapeType.Solid); ex.More(); ex.Next())
            {
                Solid so = ShapeCasts.Solid(ex.Current());
                double tol3d = global::Kerf.Engine.Foundation.Precision.Confusion();
                _solidClassifier.Classify(so, p3D, tol3d);
                _state = _solidClassifier.State();
                if (_state == In || _state == On)
                {
                    break;
                }
            }
        }
        else
        {
            throw new ProgramErrorException("StateShapeShape : bad operands");
        }
    }

    /// <summary>The state found by the last classification.</summary>
    public Containment State() => _state;

    /// <summary>The 3D point used by the last classification.</summary>
    /// <exception cref="ProgramErrorException">There is none.</exception>
    public Point3D P3D()
    {
        if (_p3Ddef)
        {
            return _p3D;
        }
        throw new ProgramErrorException("ShapeClassifier::P3D undefined");
    }

    /// <summary>The 2D point used by the last classification.</summary>
    /// <exception cref="ProgramErrorException">There is none.</exception>
    public Point2D P2D()
    {
        if (_p2Ddef)
        {
            return _p2D;
        }
        throw new ProgramErrorException("ShapeClassifier::P2D undefined");
    }

    private bool HasAvLS()
    {
        bool hasavls = _pAvLS is not null && !_pAvLS.IsEmpty();
        return hasavls;
    }
}
