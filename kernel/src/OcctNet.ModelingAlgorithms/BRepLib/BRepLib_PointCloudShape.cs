// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepLib/BRepLib_PointCloudShape.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepLib/BRepLib_PointCloudShape.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2021-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation.Std;
using Kerf.Engine.Geometry;
using Kerf.Engine.Measurement;
using Kerf.Engine.Meshing;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;
using static Kerf.Topology.Containment;

namespace Occt;

/// <summary>
/// Generates points on a shape, optionally moved off it along the normal by a random distance up to
/// <see cref="GetDistance"/> (a simulated laser scan): with a density (random points on each face), or at the nodes
/// of the faces' triangulations. A subclass receives each point through <see cref="addPoint"/>.
/// </summary>
/// <remarks>
/// A working object (decision 21). It only reads the shape: the normals of a triangulation without normals are
/// computed on a copy (<see cref="BRepLib_ToolTriangulatedShape.NormalsComputed"/>), where OCCT writes them into the
/// face's triangulation. The random points come from <c>std::mt19937</c> seeded with 0 for each face, as in OCCT.
/// </remarks>
public abstract class BRepLib_PointCloudShape
{
    protected Shape myShape;
    protected double myDist;
    protected double myTol;
    protected NCollection_DataMap<Shape, double> myFaceArea = new(ShapeHasher.Instance);
    protected NCollection_DataMap<Shape, int> myFacePoints = new(ShapeHasher.Instance);
    protected int myNbPoints;

    /// <summary>A generator for the shape (null by default) with the tolerance.</summary>
    protected BRepLib_PointCloudShape(in Shape theShape, double theTol = 1.0e-7)
    {
        myShape = theShape;
        myDist = 0.0;
        myTol = theTol;
        myNbPoints = 0;
    }

    /// <summary>A generator without a shape.</summary>
    protected BRepLib_PointCloudShape()
        : this(new Shape(), global::Kerf.Engine.Foundation.Precision.Confusion())
    {
    }

    /// <summary>The loaded shape.</summary>
    public Shape Shape() => myShape;

    /// <summary>Sets the shape.</summary>
    public void SetShape(in Shape theShape) => myShape = theShape;

    /// <summary>The tolerance.</summary>
    public double Tolerance() => myTol;

    /// <summary>Sets the tolerance.</summary>
    public void SetTolerance(double theTol) => myTol = theTol;

    /// <summary>The distance of the points from the shape along the normal (0 by default).</summary>
    public double GetDistance() => myDist;

    /// <summary>Sets the distance of the points from the shape along the normal.</summary>
    public void SetDistance(double theDist) => myDist = theDist;

    /// <summary>The number of points for the density (0: computed, 10 points per smallest face area).</summary>
    public int NbPointsByDensity(double theDensity = 0.0)
    {
        clear();
        double aDensity = (theDensity < global::Kerf.Engine.Foundation.Precision.Confusion() ? computeDensity() : theDensity);
        if (aDensity < global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            return 0;
        }

        int aNbPoints = 0;
        for (var aExpF = new Explorer(myShape, ShapeType.Face); aExpF.More(); aExpF.Next())
        {
            double anArea = faceArea(aExpF.Current());

            // OCCT divides by theDensity, not aDensity: with the computed density (theDensity 0) the quotient is
            // infinite and the C conversion to int gives INT_MIN, so each face gets 1 point (occt-issues.md).
            int aNbPnts = Max(cTruncate(Math.Ceiling(anArea / theDensity)), 1);
            myFacePoints.Bind(aExpF.Current(), aNbPnts);
            aNbPoints += aNbPnts;
        }
        return aNbPoints;
    }

    /// <summary>The number of points of the faces' triangulations.</summary>
    public int NbPointsByTriangulation()
    {
        // at first step find the face with smallest area
        int aNbPoints = 0;
        for (var aExpF = new Explorer(myShape, ShapeType.Face); aExpF.More(); aExpF.Next())
        {
            Triangulation? aTriangulation = Tool.Triangulation(ShapeCasts.Face(aExpF.Current()), out _);
            if (aTriangulation is null)
            {
                continue;
            }

            aNbPoints += aTriangulation.NodeCount;
        }
        return aNbPoints;
    }

    /// <summary>Generates the points with the density (0: computed). Returns true if any face got points.</summary>
    public bool GeneratePointsByDensity(double theDensity = 0.0)
    {
        if (myFacePoints.IsEmpty())
        {
            if (NbPointsByDensity(theDensity) == 0)
            {
                return false;
            }
        }

        int aNbAdded = 0;
        for (var aExpF = new Explorer(myShape, ShapeType.Face); aExpF.More(); aExpF.Next())
        {
            if (addDensityPoints(aExpF.Current()))
            {
                aNbAdded++;
            }
        }
        return (aNbAdded > 0);
    }

    /// <summary>Generates the points at the nodes of the faces' triangulations. Returns true if any face had one.</summary>
    public bool GeneratePointsByTriangulation()
    {
        clear();

        int aNbAdded = 0;
        for (var aExpF = new Explorer(myShape, ShapeType.Face); aExpF.More(); aExpF.Next())
        {
            if (addTriangulationPoints(aExpF.Current()))
            {
                aNbAdded++;
            }
        }
        return (aNbAdded > 0);
    }

    /// <summary>The area of the face (cached).</summary>
    protected double faceArea(in Shape theShape)
    {
        if (myFaceArea.Find(theShape, out double anArea))
        {
            return anArea;
        }

        var aFaceProps = new Properties();
        BRepGProp.SurfaceProperties(theShape, aFaceProps);
        anArea = aFaceProps.Mass();
        myFaceArea.Bind(theShape, anArea);
        return anArea;
    }

    /// <summary>The default density: a tenth of the smallest face area above the squared tolerance.</summary>
    protected double computeDensity()
    {
        // at first step find the face with smallest area
        double anAreaMin = global::Kerf.Engine.Foundation.Precision.Infinite();
        for (var aExpF = new Explorer(myShape, ShapeType.Face); aExpF.More(); aExpF.Next())
        {
            double anArea = faceArea(aExpF.Current());
            if (anArea < myTol * myTol)
            {
                continue;
            }

            if (anArea < anAreaMin)
            {
                anAreaMin = anArea;
            }
        }
        return anAreaMin * 0.1;
    }

    /// <summary>Adds the face's points for the density, at random inside it.</summary>
    protected bool addDensityPoints(in Shape theFace)
    {
        // addition of the points with specified density on the face by random way
        int aNbPnts = (myFacePoints.IsBound(theFace) ? myFacePoints.Find(theFace) : 0);
        if (aNbPnts == 0)
        {
            return false;
        }

        Face aFace = ShapeCasts.Face(theFace);
        BrepTools.UVBounds(aFace, out double anUMin, out double anUMax, out double aVMin, out double aVMax);
        var aClassifier = new BRepTopAdaptor_FClass2d(aFace, global::Kerf.Engine.Foundation.Precision.Confusion());

        Placement aLoc = theFace.Location();
        Transform3D aTrsf = aLoc.Transformation();
        Surface? aSurf = Tool.Surface(aFace, out _);
        if (aSurf is null)
        {
            return false;
        }

        var aRandomGenerator = new Mt19937(0);
        for (int nbCurPnts = 1; nbCurPnts <= aNbPnts;)
        {
            double aU = uniformReal(aRandomGenerator, anUMin, anUMax);
            double aV = uniformReal(aRandomGenerator, aVMin, aVMax);
            var aUVNode = new Point2D(aU, aV);
            Containment aState = aClassifier.Perform(aUVNode);
            if (aState == Out)
            {
                continue;
            }

            nbCurPnts++;

            aSurf.D1(aU, aV, out Point3D aP1, out Vector3D dU, out Vector3D dV);

            Vector3D aNorm = dU ^ dV;
            if (aFace.Orientation() == Reversed)
            {
                aNorm.Reverse();
            }
            double aNormMod = aNorm.Magnitude();
            if (aNormMod > Constants.Resolution())
            {
                aNorm /= aNormMod;
            }
            if (myDist > global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                Xyz aDeflPoint = aP1.XYZ() + aNorm.XYZ() * uniformReal(aRandomGenerator, 0.0, myDist);
                aP1.SetXYZ(aDeflPoint);
            }
            aP1.Transform(aTrsf);
            if (aNormMod > Constants.Resolution())
            {
                aNorm = new Vector3D(new Direction3D(aNorm).Transformed(aTrsf));
            }
            addPoint(aP1, aNorm, aUVNode, aFace);
        }
        return true;
    }

    /// <summary>Adds the nodes of the face's triangulation, with their normals.</summary>
    protected bool addTriangulationPoints(in Shape theFace)
    {
        Face aFace = ShapeCasts.Face(theFace);
        Triangulation? aTriangulation = Tool.Triangulation(aFace, out Placement aLoc);
        if (aTriangulation is null)
        {
            return false;
        }

        Transform3D aTrsf = aLoc.Transformation();

        // OCCT.NET: frozen. OCCT computes missing normals into the face's triangulation; the port on a copy.
        aTriangulation = BRepLib_ToolTriangulatedShape.NormalsComputed(aFace, aTriangulation)!;
        bool aHasUVNode = aTriangulation.HasUVNodes;
        for (int aNodeIter = 1; aNodeIter <= aTriangulation.NodeCount; ++aNodeIter)
        {
            Point3D aP1 = aTriangulation.Node(aNodeIter);
            Direction3D aNormal = aTriangulation.Normal(aNodeIter);
            if (!aLoc.IsIdentity)
            {
                aP1.Transform(aTrsf);
                aNormal.Transform(aTrsf);
            }

            Point2D anUVNode = aHasUVNode ? aTriangulation.UVNode(aNodeIter) : new Point2D();
            addPoint(aP1, new Vector3D(aNormal), anUVNode, aFace);
        }
        return true;
    }

    /// <summary>Clears the maps of areas and point counts.</summary>
    protected virtual void clear()
    {
        myFaceArea.Clear();
        myFacePoints.Clear();
    }

    /// <summary>Receives a point, the surface normal there, its UV parameters and its face.</summary>
    protected abstract void addPoint(in Point3D thePoint, in Vector3D theNorm, in Point2D theUV, in Shape theFace);

    // std::uniform_real_distribution<double>(a, b) on std::mt19937: generate_canonical<double, 53> takes two 32-bit
    // draws, (x0 + x1 * 2^32) / 2^64, then a + u * (b - a) (the same in MSVC's and GCC's libraries).
    private static double uniformReal(Mt19937 theGenerator, double theA, double theB)
    {
        double aSum = 0.0, aFactor = 1.0;
        for (int k = 0; k < 2; ++k)
        {
            aSum += (double)theGenerator.Invoke() * aFactor;
            aFactor *= 4294967296.0;
        }
        double aCanonical = aSum / aFactor;
        return aCanonical * (theB - theA) + theA;
    }

    // A double to int conversion as x86's cvttsd2si: INT_MIN for NaN and out-of-range values (C++'s undefined case).
    private static int cTruncate(double theValue)
        => double.IsNaN(theValue) || theValue >= 2147483648.0 || theValue < -2147483648.0 ? int.MinValue : (int)theValue;
}
