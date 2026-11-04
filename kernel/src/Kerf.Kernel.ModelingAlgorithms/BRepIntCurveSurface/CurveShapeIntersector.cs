// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepIntCurveSurface/BRepIntCurveSurface_Inter.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepIntCurveSurface/BRepIntCurveSurface_Inter.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Geometry.CurveKind;
using static Kerf.Topology.ShapeType;
using static Kerf.Topology.Containment;

namespace Kerf.Engine.Intersection.CurveSurface;

/// <summary>
/// The intersection of a curve with the faces of a shape: its points IN or ON the faces, face by face, iterated with
/// <see cref="More"/> and <see cref="Next"/>.
/// </summary>
/// <remarks>
/// A working object (decision 21). To intersect several curves with one shape, <see cref="Load"/> it once and
/// <see cref="Init(CurveAdaptor)"/> each curve; the faces' boxes are computed once. It only reads the shape.
/// </remarks>
internal class CurveShapeIntersector
{
    private double _tolerance;
    private CurveAdaptor? _curve;
    private readonly HInter _intcs = new();
    private int _currentindex;
    private int _currentnbpoints;
    private readonly BrepTopolTool _fastClass;
    private Containment _currentstate;
    private double _currentU;
    private double _currentV;
    private Box _curveBox = new();
    private int _indFace;
    private readonly NCollection_Sequence<Shape> _faces = new();
    private NCollection_HArray1<Box>? _faceBoxes;

    /// <summary>An empty intersector.</summary>
    public CurveShapeIntersector()
    {
        _fastClass = new BrepTopolTool();
        Clear();
    }

    /// <summary>Loads the shape and the curve, with the tolerance <paramref name="tol"/> of the classification.</summary>
    public void Init(in Shape shape, CurveAdaptor curve, double tol)
    {
        Load(shape, tol);
        Init(curve);
    }

    /// <summary>Loads the shape and the line, with the tolerance <paramref name="tol"/> of the classification.</summary>
    public void Init(in Shape shape, in Line3D line, double tol)
    {
        var geomline = new LineCurve3D(line);
        var curve = new CurveAdaptor(geomline);
        Load(shape, tol);
        Init(curve);
    }

    /// <summary>Resets the iteration.</summary>
    protected void Clear()
    {
        _currentindex = 0;
        _currentnbpoints = 0;
        _indFace = 0;
        _currentstate = Unknown;
        _currentU = 0;
        _currentV = 0;
    }

    /// <summary>Loads the shape, with the tolerance <paramref name="tol"/> of the classification.</summary>
    public void Load(in Shape shape, double tol)
    {
        Clear();
        _faces.Clear();
        _faceBoxes = null;
        _tolerance = tol;
        var explorer = new Explorer(shape, global::Kerf.Topology.ShapeType.Face);
        for (; explorer.More(); explorer.Next())
        {
            _faces.Append(explorer.Current());
        }
    }

    /// <summary>Intersects the curve with the loaded shape and moves to the first point.</summary>
    /// <remarks>The intersector keeps a copy of the adaptor (OCCT's), sharing its curve.</remarks>
    public void Init(CurveAdaptor curve)
    {
        Clear();
        _curveBox.SetVoid();
        double first = curve.FirstParameter();
        double last = curve.LastParameter();
        _curve = (CurveAdaptor)curve.ShallowCopy();
        if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(first) && !global::Kerf.Engine.Foundation.Precision.IsInfinite(last))
        {
            Add3dCurve.Add(_curve, 0.0, ref _curveBox);
        }

        Find();
    }

    /// <summary>True if there is a current point.</summary>
    public bool More() => (_indFace <= _faces.Length());

    /// <summary>Moves to the next point.</summary>
    public void Next()
    {
        if (_currentnbpoints != 0)
        {
            _currentindex++;
        }

        Find();
    }

    /// <summary>Finds the next point IN or ON a face, from the current one.</summary>
    protected void Find()
    {
        if (_currentnbpoints != 0 && _currentindex <= _currentnbpoints && FindPoint())
        {
            return;
        }

        _currentnbpoints = 0;
        _currentindex = 0;

        int i = _indFace + 1;
        for (; i <= _faces.Length(); i++)
        {
            Shape curface = _faces[i];
            _faceBoxes ??= new NCollection_HArray1<Box>(1, _faces.Length());

            ref Box faceBox = ref _faceBoxes[i];
            if (faceBox.IsVoid)
            {
                Bounds.Add(curface, ref faceBox);
                faceBox.SetGap(_tolerance); // Precision::Confusion());
            }

            bool isOut = (_curve!.GetType() == Line
                              ? faceBox.IsOut(_curve.Line())
                              : (!_curveBox.IsVoid ? faceBox.IsOut(_curveBox) : false));
            if (isOut)
            {
                continue;
            }

            var surfForFastClass = new FaceSurfaceAdaptor(ShapeCasts.Face(curface));
            _intcs.Perform(_curve, surfForFastClass);
            _currentnbpoints = _intcs.PointCount;
            if (_currentnbpoints == 0)
            {
                continue;
            }

            _fastClass.Initialize((SurfaceAdaptorBase)surfForFastClass);
            _indFace = i;
            if (FindPoint())
            {
                return;
            }

            _currentnbpoints = 0;
        }

        if (_currentnbpoints == 0 && i > _faces.Length())
        {
            _indFace = i;
            return;
        }
    }

    /// <summary>Finds the first point from the current index that is IN or ON the current face.</summary>
    protected bool FindPoint()
    {
        int j = (_currentindex == 0 ? 1 : _currentindex);

        for (; j <= _currentnbpoints; j++)
        {
            double u = _intcs.Point(j).U();
            double v = _intcs.Point(j).V();

            Point2D puv = new(u, v);

            _currentstate = _fastClass.Classify(puv, _tolerance);
            if (_currentstate == On || _currentstate == In)
            {
                _currentindex = j;
                _currentU = u;
                _currentV = v;
                return true;
            }
        }

        return false;
    }

    /// <summary>The current point.</summary>
    /// <exception cref="NotDoneException">There is no current point.</exception>
    public CurveSurfaceIntersectionPoint Point()
    {
        if (_currentindex == 0)
        {
            throw new NotDoneException();
        }

        CurveSurfaceIntersectionPoint icps = _intcs.Point(_currentindex);
        return (new CurveSurfaceIntersectionPoint(icps.Pnt(),
                                                      _currentU, // ICPS.U(),
                                                      _currentV, // ICPS.V(),
                                                      icps.W(),
                                                      icps.Transition()));
        //-- return(myIntcs.Point(myCurrentindex));
    }

    /// <summary>The U parameter of the current point on the current face.</summary>
    /// <exception cref="NotDoneException">There is no current point.</exception>
    public double U()
    {
        if (_currentindex == 0)
        {
            throw new NotDoneException();
        }

        //-- return(myIntcs.Point(myCurrentindex).U());
        return (_currentU);
    }

    /// <summary>The V parameter of the current point on the current face.</summary>
    /// <exception cref="NotDoneException">There is no current point.</exception>
    public double V()
    {
        if (_currentindex == 0)
        {
            throw new NotDoneException();
        }

        //-- return(myIntcs.Point(myCurrentindex).V());
        return (_currentV);
    }

    /// <summary>The parameter of the current point on the curve.</summary>
    /// <exception cref="NotDoneException">There is no current point.</exception>
    public double W()
    {
        if (_currentindex == 0)
        {
            throw new NotDoneException();
        }

        return (_intcs.Point(_currentindex).W());
    }

    /// <summary>The state of the current point in the current face: IN or ON.</summary>
    /// <exception cref="NotDoneException">There is no current point.</exception>
    public Containment State()
    {
        if (_currentindex == 0)
        {
            throw new NotDoneException();
        }

        //-- return(classifier.State());
        return (_currentstate);
    }

    /// <summary>The transition of the curve on the surface at the current point (IN, OUT or TANGENT).</summary>
    /// <exception cref="NotDoneException">There is no current point.</exception>
    public TransitionOnCurve Transition()
    {
        if (_currentindex == 0)
        {
            throw new NotDoneException();
        }

        return (_intcs.Point(_currentindex).Transition());
    }

    /// <summary>The current face.</summary>
    public Face Face() => (ShapeCasts.Face(_faces.Value(_indFace)));

    /// <summary>The current point.</summary>
    /// <exception cref="NotDoneException">There is no current point.</exception>
    public Point3D Pnt()
    {
        if (_currentindex == 0)
        {
            throw new NotDoneException();
        }

        return (_intcs.Point(_currentindex).Pnt());
    }
}
