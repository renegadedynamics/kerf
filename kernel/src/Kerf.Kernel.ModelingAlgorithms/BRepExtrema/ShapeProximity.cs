// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepExtrema/BRepExtrema_ShapeProximity.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepExtrema/BRepExtrema_ShapeProximity.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2014-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using ProxPnt_Status = Kerf.Engine.Measurement.Distance.ProximityDistTool.ProxPnt_Status;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Measurement.Distance;

/// <summary>
/// Shape proximity from the shapes' tessellations. With a finite tolerance, finds the faces of two shapes that
/// intersect (tolerance zero) or come within the tolerance of each other. With an infinite tolerance (the default),
/// computes the proximity value of two edges or two faces: the thickness of the thinnest layer holding both.
/// The meshes must exist already; results are approximate and depend on them.
/// </summary>
/// <remarks>
/// A working object (decision 21); it only reads the shapes' meshes. As in OCCT, computing the proximity value sets
/// the tolerance to it, so a later Load switches to the overlap test.
/// </remarks>
internal sealed class ShapeProximity
{
    private double _tolerance;
    private bool _isInitS1;
    private bool _isInitS2;
    private readonly List<Shape> _shapeList1 = new();
    private readonly List<Shape> _shapeList2 = new();
    private TriangleSet _elementSet1;
    private TriangleSet _elementSet2;
    private int _nbSamples1;
    private int _nbSamples2;
    private Point3D _proxPoint1;
    private Point3D _proxPoint2;
    private ProxPnt_Status _proxPntStatus1, _proxPntStatus2;
    private readonly OverlapTool _overlapTool = new();
    private readonly ProximityValueTool _proxValTool = new();

    /// <summary>An empty proximity tool.</summary>
    public ShapeProximity(double tolerance = 2.0e+100) // Precision::Infinite()
    {
        _tolerance = tolerance;
        _elementSet1 = new TriangleSet();
        _elementSet2 = new TriangleSet();
        _nbSamples1 = 0;
        _nbSamples2 = 0;
        // Should be initialized later
        _isInitS1 = _isInitS2 = false;
    }

    /// <summary>A proximity tool for the two shapes.</summary>
    public ShapeProximity(in Shape shape1, in Shape shape2, double tolerance = 2.0e+100)
    {
        _tolerance = tolerance;
        _elementSet1 = new TriangleSet();
        _elementSet2 = new TriangleSet();
        _nbSamples1 = 0;
        _nbSamples2 = 0;
        LoadShape1(shape1);
        LoadShape2(shape2);
    }

    /// <summary>The tolerance of the overlap test (the distance between the shapes).</summary>
    public double Tolerance() => _tolerance;

    /// <summary>Sets the tolerance of the overlap test.</summary>
    public void SetTolerance(double tolerance) => _tolerance = tolerance;

    /// <summary>The proximity value of the whole shapes.</summary>
    public double Proximity() => Tolerance();

    /// <summary>Sets the number of sample points on the first shape (0: every triangulation node).</summary>
    public void SetNbSamples1(int nbSamples) => _nbSamples1 = nbSamples;

    /// <summary>Sets the number of sample points on the second shape (0: every triangulation node).</summary>
    public void SetNbSamples2(int nbSamples) => _nbSamples2 = nbSamples;

    /// <summary>True if the search is completed.</summary>
    public bool IsDone => _overlapTool.IsDone || _proxValTool.IsDone;

    /// <summary>The overlapped faces of the first shape (indices from 0), each with the faces of the second.</summary>
    public Dictionary<int, NCollection_PackedMapOfInteger> OverlapSubShapes1() => _overlapTool.OverlapSubShapes1();

    /// <summary>The overlapped faces of the second shape (indices from 0), each with the faces of the first.</summary>
    public Dictionary<int, NCollection_PackedMapOfInteger> OverlapSubShapes2() => _overlapTool.OverlapSubShapes2();

    /// <summary>The sub-shape of the first shape with the given index (from 0).</summary>
    public Shape GetSubShape1(int id) => _shapeList1[id];

    /// <summary>The sub-shape of the second shape with the given index (from 0).</summary>
    public Shape GetSubShape2(int id) => _shapeList2[id];

    /// <summary>The set of all the face triangles of the first shape.</summary>
    public TriangleSet ElementSet1() => _elementSet1;

    /// <summary>The set of all the face triangles of the second shape.</summary>
    public TriangleSet ElementSet2() => _elementSet2;

    /// <summary>The point of the first shape giving the proximity value.</summary>
    public Point3D ProximityPoint1() => _proxPoint1;

    /// <summary>The point of the second shape giving the proximity value.</summary>
    public Point3D ProximityPoint2() => _proxPoint2;

    /// <summary>The status of the point of the first shape giving the proximity value.</summary>
    public ProxPnt_Status ProxPntStatus1() => _proxPntStatus1;

    /// <summary>The status of the point of the second shape giving the proximity value.</summary>
    public ProxPnt_Status ProxPntStatus2() => _proxPntStatus2;

    // Assign a map of sub-shapes (edges/faces) of a given shape
    private static bool InitSubShapes(in Shape shape,
                                      List<Shape> subshapesList,
                                      ref TriangleSet triangleSet)
    {
        subshapesList.Clear();

        for (var iter = new Explorer(shape, ShapeType.Face); iter.More(); iter.Next())
        {
            subshapesList.Add(iter.Current());
        }

        for (var iter = new Explorer(shape, ShapeType.Edge); iter.More(); iter.Next())
        {
            subshapesList.Add(iter.Current());
        }

        triangleSet ??= new TriangleSet();

        return triangleSet.Init(subshapesList);
    }

    /// <summary>Loads the first shape.</summary>
    public bool LoadShape1(in Shape shape1)
    {
        _isInitS1 = InitSubShapes(shape1, _shapeList1, ref _elementSet1);

        if (_tolerance == global::Kerf.Engine.Foundation.Precision.Infinite())
        {
            _proxValTool.MarkDirty();
        }
        else
        {
            _overlapTool.MarkDirty();
        }

        return _isInitS1;
    }

    /// <summary>Loads the second shape.</summary>
    public bool LoadShape2(in Shape shape2)
    {
        _isInitS2 = InitSubShapes(shape2, _shapeList2, ref _elementSet2);

        if (_tolerance == global::Kerf.Engine.Foundation.Precision.Infinite())
        {
            _proxValTool.MarkDirty();
        }
        else
        {
            _overlapTool.MarkDirty();
        }

        return _isInitS2;
    }

    /// <summary>Searches for overlapped faces, or computes the proximity value when the tolerance is infinite.</summary>
    public void Perform()
    {
        if (_tolerance == global::Kerf.Engine.Foundation.Precision.Infinite())
        {
            if (!_isInitS1 || !_isInitS2 || _proxValTool.IsDone)
            {
                return;
            }

            _proxValTool.LoadTriangleSets(_elementSet1, _elementSet2);
            _proxValTool.LoadShapeLists(_shapeList1, _shapeList2);
            _proxValTool.SetNbSamplePoints(_nbSamples1, _nbSamples2);

            // OCCT: the proximity value is written into myTolerance, so the next Load and Perform test overlaps with it
            _proxValTool.Perform(ref _tolerance);
            _proxValTool.ProximityPoints(out _proxPoint1, out _proxPoint2);
            _proxValTool.ProximityPointsStatus(out _proxPntStatus1, out _proxPntStatus2);
        }
        else
        {
            if (!_isInitS1 || !_isInitS2 || _overlapTool.IsDone)
            {
                return;
            }

            _overlapTool.LoadTriangleSets(_elementSet1, _elementSet2);

            _overlapTool.Perform(_tolerance);
        }
    }
}
