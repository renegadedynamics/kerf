// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepLib/BRepLib_PointCloudShape.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepLib/BRepLib_PointCloudShape.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2021-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation.Std;
using Kerf.Engine.Geometry;
using Kerf.Engine.Measurement;
using Kerf.Engine.Meshing;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;
using static Kerf.Topology.Containment;

namespace Kerf.Engine.Construction.Library;

/// <summary>
/// Generates points on a shape, optionally moved off it along the normal by a random distance up to
/// <see cref="GetDistance"/> (a simulated laser scan): with a density (random points on each face), or at the nodes
/// of the faces' triangulations. A subclass receives each point through <see cref="AddPoint"/>.
/// </summary>
/// <remarks>
/// A working object (decision 21). It only reads the shape: the normals of a triangulation without normals are
/// computed on a copy (<see cref="ToolTriangulatedShape.NormalsComputed"/>), where OCCT writes them into the
/// face's triangulation. The random points come from <c>std::mt19937</c> seeded with 0 for each face, as in OCCT.
/// </remarks>
public abstract class PointCloudShape
{
    protected Shape _shape;
    protected double _dist;
    protected double _tol;
    protected NCollection_DataMap<Shape, double> _faceArea = new(ShapeHasher.Instance);
    protected NCollection_DataMap<Shape, int> _facePoints = new(ShapeHasher.Instance);
    protected int _nbPoints;

    /// <summary>A generator for the shape (null by default) with the tolerance.</summary>
    protected PointCloudShape(in Shape shape, double tol = 1.0e-7)
    {
        _shape = shape;
        _dist = 0.0;
        _tol = tol;
        _nbPoints = 0;
    }

    /// <summary>A generator without a shape.</summary>
    protected PointCloudShape()
        : this(new Shape(), global::Kerf.Engine.Foundation.Precision.Confusion())
    {
    }

    /// <summary>The loaded shape.</summary>
    public Shape Shape() => _shape;

    /// <summary>Sets the shape.</summary>
    public void SetShape(in Shape shape) => _shape = shape;

    /// <summary>The tolerance.</summary>
    public double Tolerance() => _tol;

    /// <summary>Sets the tolerance.</summary>
    public void SetTolerance(double tol) => _tol = tol;

    /// <summary>The distance of the points from the shape along the normal (0 by default).</summary>
    public double GetDistance() => _dist;

    /// <summary>Sets the distance of the points from the shape along the normal.</summary>
    public void SetDistance(double dist) => _dist = dist;

    /// <summary>The number of points for the density (0: computed, 10 points per smallest face area).</summary>
    public int NbPointsByDensity(double density = 0.0)
    {
        Clear();
        double aDensity = (density < global::Kerf.Engine.Foundation.Precision.Confusion() ? ComputeDensity() : density);
        if (aDensity < global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            return 0;
        }

        int nbPoints = 0;
        for (var expF = new Explorer(_shape, ShapeType.Face); expF.More(); expF.Next())
        {
            double area = FaceArea(expF.Current());

            // OCCT divides by theDensity, not aDensity: with the computed density (theDensity 0) the quotient is
            // infinite and the C conversion to int gives INT_MIN, so each face gets 1 point (occt-issues.md).
            int nbPnts = Max(CTruncate(Math.Ceiling(area / density)), 1);
            _facePoints.Bind(expF.Current(), nbPnts);
            nbPoints += nbPnts;
        }
        return nbPoints;
    }

    /// <summary>The number of points of the faces' triangulations.</summary>
    public int NbPointsByTriangulation()
    {
        // at first step find the face with smallest area
        int nbPoints = 0;
        for (var expF = new Explorer(_shape, ShapeType.Face); expF.More(); expF.Next())
        {
            Triangulation? triangulation = Tool.Triangulation(ShapeCasts.Face(expF.Current()), out _);
            if (triangulation is null)
            {
                continue;
            }

            nbPoints += triangulation.NodeCount;
        }
        return nbPoints;
    }

    /// <summary>Generates the points with the density (0: computed). Returns true if any face got points.</summary>
    public bool GeneratePointsByDensity(double density = 0.0)
    {
        if (_facePoints.IsEmpty())
        {
            if (NbPointsByDensity(density) == 0)
            {
                return false;
            }
        }

        int nbAdded = 0;
        for (var expF = new Explorer(_shape, ShapeType.Face); expF.More(); expF.Next())
        {
            if (AddDensityPoints(expF.Current()))
            {
                nbAdded++;
            }
        }
        return (nbAdded > 0);
    }

    /// <summary>Generates the points at the nodes of the faces' triangulations. Returns true if any face had one.</summary>
    public bool GeneratePointsByTriangulation()
    {
        Clear();

        int nbAdded = 0;
        for (var expF = new Explorer(_shape, ShapeType.Face); expF.More(); expF.Next())
        {
            if (AddTriangulationPoints(expF.Current()))
            {
                nbAdded++;
            }
        }
        return (nbAdded > 0);
    }

    /// <summary>The area of the face (cached).</summary>
    protected double FaceArea(in Shape shape)
    {
        if (_faceArea.Find(shape, out double area))
        {
            return area;
        }

        var faceProps = new Properties();
        MassPropertiesBuilder.SurfaceProperties(shape, faceProps);
        area = faceProps.Mass();
        _faceArea.Bind(shape, area);
        return area;
    }

    /// <summary>The default density: a tenth of the smallest face area above the squared tolerance.</summary>
    protected double ComputeDensity()
    {
        // at first step find the face with smallest area
        double areaMin = global::Kerf.Engine.Foundation.Precision.Infinite();
        for (var expF = new Explorer(_shape, ShapeType.Face); expF.More(); expF.Next())
        {
            double area = FaceArea(expF.Current());
            if (area < _tol * _tol)
            {
                continue;
            }

            if (area < areaMin)
            {
                areaMin = area;
            }
        }
        return areaMin * 0.1;
    }

    /// <summary>Adds the face's points for the density, at random inside it.</summary>
    protected bool AddDensityPoints(in Shape face)
    {
        // addition of the points with specified density on the face by random way
        int nbPnts = (_facePoints.IsBound(face) ? _facePoints.Find(face) : 0);
        if (nbPnts == 0)
        {
            return false;
        }

        Face aFace = ShapeCasts.Face(face);
        BrepTools.UVBounds(aFace, out double uMin, out double uMax, out double vMin, out double vMax);
        var classifier = new FClass2d(aFace, global::Kerf.Engine.Foundation.Precision.Confusion());

        Placement loc = face.Location();
        Transform3D trsf = loc.Transformation();
        Surface? surf = Tool.Surface(aFace, out _);
        if (surf is null)
        {
            return false;
        }

        var randomGenerator = new Mt19937(0);
        for (int nbCurPnts = 1; nbCurPnts <= nbPnts;)
        {
            double u = UniformReal(randomGenerator, uMin, uMax);
            double v = UniformReal(randomGenerator, vMin, vMax);
            var uvNode = new Point2D(u, v);
            Containment state = classifier.Perform(uvNode);
            if (state == Out)
            {
                continue;
            }

            nbCurPnts++;

            surf.D1(u, v, out Point3D p1, out Vector3D dU, out Vector3D dV);

            Vector3D norm = dU ^ dV;
            if (aFace.Orientation() == Reversed)
            {
                norm.Reverse();
            }
            double normMod = norm.Magnitude();
            if (normMod > Constants.Resolution())
            {
                norm /= normMod;
            }
            if (_dist > global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                Xyz deflPoint = p1.XYZ() + norm.XYZ() * UniformReal(randomGenerator, 0.0, _dist);
                p1.SetXYZ(deflPoint);
            }
            p1.Transform(trsf);
            if (normMod > Constants.Resolution())
            {
                norm = new Vector3D(new Direction3D(norm).Transformed(trsf));
            }
            AddPoint(p1, norm, uvNode, aFace);
        }
        return true;
    }

    /// <summary>Adds the nodes of the face's triangulation, with their normals.</summary>
    protected bool AddTriangulationPoints(in Shape face)
    {
        Face aFace = ShapeCasts.Face(face);
        Triangulation? triangulation = Tool.Triangulation(aFace, out Placement loc);
        if (triangulation is null)
        {
            return false;
        }

        Transform3D trsf = loc.Transformation();

        // OCCT.NET: frozen. OCCT computes missing normals into the face's triangulation; the port on a copy.
        triangulation = ToolTriangulatedShape.NormalsComputed(aFace, triangulation)!;
        bool hasUVNode = triangulation.HasUVNodes;
        for (int nodeIter = 1; nodeIter <= triangulation.NodeCount; ++nodeIter)
        {
            Point3D p1 = triangulation.Node(nodeIter);
            Direction3D normal = triangulation.Normal(nodeIter);
            if (!loc.IsIdentity)
            {
                p1.Transform(trsf);
                normal.Transform(trsf);
            }

            Point2D uvNode = hasUVNode ? triangulation.UVNode(nodeIter) : new Point2D();
            AddPoint(p1, new Vector3D(normal), uvNode, aFace);
        }
        return true;
    }

    /// <summary>Clears the maps of areas and point counts.</summary>
    protected virtual void Clear()
    {
        _faceArea.Clear();
        _facePoints.Clear();
    }

    /// <summary>Receives a point, the surface normal there, its UV parameters and its face.</summary>
    protected abstract void AddPoint(in Point3D point, in Vector3D norm, in Point2D uv, in Shape face);

    // std::uniform_real_distribution<double>(a, b) on std::mt19937: generate_canonical<double, 53> takes two 32-bit
    // draws, (x0 + x1 * 2^32) / 2^64, then a + u * (b - a) (the same in MSVC's and GCC's libraries).
    private static double UniformReal(Mt19937 generator, double a, double b)
    {
        double sum = 0.0, factor = 1.0;
        for (int k = 0; k < 2; ++k)
        {
            sum += (double)generator.Invoke() * factor;
            factor *= 4294967296.0;
        }
        double canonical = sum / factor;
        return canonical * (b - a) + a;
    }

    // A double to int conversion as x86's cvttsd2si: INT_MIN for NaN and out-of-range values (C++'s undefined case).
    private static int CTruncate(double value)
        => double.IsNaN(value) || value >= 2147483648.0 || value < -2147483648.0 ? int.MinValue : (int)value;
}
