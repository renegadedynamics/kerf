// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_PointLine.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_PointLine.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Intersection.Walking;
using Kerf.Geometry;

namespace Kerf.Engine.Intersection.Patches;

/// <summary>
/// An intersection line defined by a set of points (<see cref="LineOn2S"/>), from a walking algorithm
/// (<see cref="WLine"/>) or on a restriction (<see cref="RLine"/>), with its vertices
/// (<see cref="PatchPoint"/>).
/// </summary>
/// <remarks>
/// A working object (decision 609). <see cref="Point"/> and <see cref="Vertex"/> return copies (OCCT's const
/// references); <see cref="ChangeVertex"/> returns a reference to the stored vertex, to change it in place.
/// </remarks>
internal abstract class PointLine : PatchLine
{
    /// <summary>A line whose transitions are in or out.</summary>
    protected PointLine(bool tang, WalkingTypeTrans trans1, WalkingTypeTrans trans2)
        : base(tang, trans1, trans2)
    {
    }

    /// <summary>A line whose transitions are touch.</summary>
    protected PointLine(bool tang, WalkingSituation situ1, WalkingSituation situ2)
        : base(tang, situ1, situ2)
    {
    }

    /// <summary>A member-wise copy of <paramref name="other"/> (OCCT's implicit copy constructor).</summary>
    protected PointLine(PointLine other)
        : base(other)
    {
    }

    /// <summary>A line whose transitions are undecided.</summary>
    protected PointLine(bool tang)
        : base(tang)
    {
    }

    /// <summary>Adds a vertex at the end of the list, or before the first if <paramref name="isPrepend"/>.</summary>
    public abstract void AddVertex(in PatchPoint pnt, bool isPrepend = false);

    /// <summary>The number of intersection points.</summary>
    public abstract int NbPnts();

    /// <summary>The number of vertices.</summary>
    public abstract int VertexCount { get; }

    /// <summary>The intersection point of range <paramref name="index"/>.</summary>
    public abstract PntOn2S Point(int index);

    /// <summary>The vertex of range <paramref name="index"/>.</summary>
    public abstract PatchPoint Vertex(int index);

    /// <summary>The vertex of range <paramref name="index"/>, to change in place.</summary>
    public abstract ref PatchPoint ChangeVertex(int index);

    /// <summary>Removes all the vertices.</summary>
    public abstract void ClearVertexes();

    /// <summary>Removes the vertex of range <paramref name="index"/>.</summary>
    public abstract void RemoveVertex(int index);

    /// <summary>The set of intersection points.</summary>
    public abstract LineOn2S? Curve();

    /// <summary>True if <paramref name="p1"/> is out of the box of the points on the first surface.</summary>
    public abstract bool IsOutSurf1Box(in Point2D p1);

    /// <summary>True if <paramref name="p2"/> is out of the box of the points on the second surface.</summary>
    public abstract bool IsOutSurf2Box(in Point2D p2);

    /// <summary>True if <paramref name="p"/> is out of the box of the 3D points.</summary>
    public abstract bool IsOutBox(in Point3D p);

    /// <summary>
    /// The radius of curvature of the intersection line at a point; negative if it can't be computed.
    /// </summary>
    public static double CurvatureRadiusOfIntersLine(SurfaceAdaptorBase s1, SurfaceAdaptorBase s2,
                                                     in PntOn2S uvPoint)
    {
        double smallValue = 1.0 / global::Kerf.Engine.Foundation.Precision.Infinite();
        double sqSmallValue = smallValue * smallValue;

        uvPoint.Parameters(out double u1, out double v1, out double u2, out double v2);
        Point3D pt;
        Vector3D du1, dv1, duu1, duv1, dvv1;
        Vector3D du2, dv2, duu2, duv2, dvv2;

        s1.D2(u1, v1, out pt, out du1, out dv1, out duu1, out dvv1, out duv1);
        s2.D2(u2, v2, out pt, out du2, out dv2, out duu2, out dvv2, out duv2);

        Vector3D n1 = du1.Crossed(dv1), n2 = du2.Crossed(dv2);
        // Tangent vector to the intersection curve
        Vector3D cTan = n1.Crossed(n2);
        double sqMagnFDer = cTan.SquareMagnitude;

        if (sqMagnFDer < 1.0e-8)
        {
            // Use 1.0e-4 (instead of aSmallValue) to provide
            // stable computation between different platforms.
            // See test bugs modalg_7 bug29807_sc01
            return -1.0;
        }

        double duS1 = 0.0, dvS1 = 0.0, duS2 = 0.0, dvS2 = 1.0;

        {
            // This algorithm is described in NonSingularProcessing() function
            // in ApproxInt_ImpPrmSvSurfaces.gxx file
            double sqNMagn = n1.SquareMagnitude;
            Vector3D tgU = cTan.Crossed(du1), tgV = cTan.Crossed(dv1);
            double deltaU = tgV.SquareMagnitude / sqNMagn;
            double deltaV = tgU.SquareMagnitude / sqNMagn;

            duS1 = Math.CopySign(Math.Sqrt(deltaU), tgV.Dot(n1));
            dvS1 = -Math.CopySign(Math.Sqrt(deltaV), tgU.Dot(n1));

            sqNMagn = n2.SquareMagnitude;
            tgU = tgU.WithXYZ(cTan.Crossed(du2).XYZ());
            tgV = tgV.WithXYZ(cTan.Crossed(dv2).XYZ());
            deltaU = tgV.SquareMagnitude / sqNMagn;
            deltaV = tgU.SquareMagnitude / sqNMagn;

            duS2 = Math.CopySign(Math.Sqrt(deltaU), tgV.Dot(n2));
            dvS2 = -Math.CopySign(Math.Sqrt(deltaV), tgU.Dot(n2));
        }

        // According to "Marching along surface/surface intersection curves
        // with an adaptive step length"
        // by Tz.E.Stoyagov
        //(http://www.sciencedirect.com/science/article/pii/016783969290046R)
        // we obtain the system:
        //             {A*a+B*b=F1
        //             {B*a+C*b=F2
        // where a and b should be found.
        // After that, 2nd derivative of the intersection curve can be computed as
        //             r''(t)=a*aN1+b*aN2.

        double a = n1.Dot(n1), b = n1.Dot(n2), c = n2.Dot(n2);
        double detSyst = b * b - a * c;

        if (Math.Abs(detSyst) < smallValue)
        {
            // Undetermined system solution
            return -1.0;
        }

        double f1 = duS1 * duS1 * duu1.Dot(n1) + 2.0 * duS1 * dvS1 * duv1.Dot(n1)
                     + dvS1 * dvS1 * dvv1.Dot(n1);
        double f2 = duS2 * duS2 * duu2.Dot(n2) + 2.0 * duS2 * dvS2 * duv2.Dot(n2)
                     + dvS2 * dvS2 * dvv2.Dot(n2);

        // Principal normal to the intersection curve
        Vector3D cNorm = (f1 * c - f2 * b) / detSyst * n1 + (a * f2 - f1 * b) / detSyst * n2;
        double sqMagnSDer = cNorm.CrossSquareMagnitude(cTan);

        if (sqMagnSDer < sqSmallValue)
        { // Intersection curve has null curvature in observed point
            return global::Kerf.Engine.Foundation.Precision.Infinite();
        }

        // square of curvature radius
        double factSqRad = sqMagnFDer * sqMagnFDer * sqMagnFDer / sqMagnSDer;

        return Math.Sqrt(factSqRad);
    }
}
