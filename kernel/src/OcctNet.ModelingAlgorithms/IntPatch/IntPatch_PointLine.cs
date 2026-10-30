// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_PointLine.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_PointLine.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// An intersection line defined by a set of points (<see cref="IntSurf_LineOn2S"/>), from a walking algorithm
/// (<see cref="IntPatch_WLine"/>) or on a restriction (<see cref="IntPatch_RLine"/>), with its vertices
/// (<see cref="IntPatch_Point"/>).
/// </summary>
/// <remarks>
/// A working object (decision 609). <see cref="Point"/> and <see cref="Vertex"/> return copies (OCCT's const
/// references); <see cref="ChangeVertex"/> returns a reference to the stored vertex, to change it in place.
/// </remarks>
public abstract class IntPatch_PointLine : IntPatch_Line
{
    /// <summary>A line whose transitions are in or out.</summary>
    protected IntPatch_PointLine(bool Tang, IntSurf_TypeTrans Trans1, IntSurf_TypeTrans Trans2)
        : base(Tang, Trans1, Trans2)
    {
    }

    /// <summary>A line whose transitions are touch.</summary>
    protected IntPatch_PointLine(bool Tang, IntSurf_Situation Situ1, IntSurf_Situation Situ2)
        : base(Tang, Situ1, Situ2)
    {
    }

    /// <summary>A line whose transitions are undecided.</summary>
    protected IntPatch_PointLine(bool Tang)
        : base(Tang)
    {
    }

    /// <summary>Adds a vertex at the end of the list, or before the first if <paramref name="theIsPrepend"/>.</summary>
    public abstract void AddVertex(in IntPatch_Point Pnt, bool theIsPrepend = false);

    /// <summary>The number of intersection points.</summary>
    public abstract int NbPnts();

    /// <summary>The number of vertices.</summary>
    public abstract int NbVertex();

    /// <summary>The intersection point of range <paramref name="Index"/>.</summary>
    public abstract IntSurf_PntOn2S Point(int Index);

    /// <summary>The vertex of range <paramref name="Index"/>.</summary>
    public abstract IntPatch_Point Vertex(int Index);

    /// <summary>The vertex of range <paramref name="Index"/>, to change in place.</summary>
    public abstract ref IntPatch_Point ChangeVertex(int Index);

    /// <summary>Removes all the vertices.</summary>
    public abstract void ClearVertexes();

    /// <summary>Removes the vertex of range <paramref name="theIndex"/>.</summary>
    public abstract void RemoveVertex(int theIndex);

    /// <summary>The set of intersection points.</summary>
    public abstract IntSurf_LineOn2S? Curve();

    /// <summary>True if <paramref name="P1"/> is out of the box of the points on the first surface.</summary>
    public abstract bool IsOutSurf1Box(in gp_Pnt2d P1);

    /// <summary>True if <paramref name="P2"/> is out of the box of the points on the second surface.</summary>
    public abstract bool IsOutSurf2Box(in gp_Pnt2d P2);

    /// <summary>True if <paramref name="P"/> is out of the box of the 3D points.</summary>
    public abstract bool IsOutBox(in gp_Pnt P);

    /// <summary>
    /// The radius of curvature of the intersection line at a point; negative if it can't be computed.
    /// </summary>
    public static double CurvatureRadiusOfIntersLine(Adaptor3d_Surface theS1, Adaptor3d_Surface theS2,
                                                     in IntSurf_PntOn2S theUVPoint)
    {
        double aSmallValue = 1.0 / Precision.Infinite();
        double aSqSmallValue = aSmallValue * aSmallValue;

        theUVPoint.Parameters(out double aU1, out double aV1, out double aU2, out double aV2);
        gp_Pnt aPt;
        gp_Vec aDU1, aDV1, aDUU1, aDUV1, aDVV1;
        gp_Vec aDU2, aDV2, aDUU2, aDUV2, aDVV2;

        theS1.D2(aU1, aV1, out aPt, out aDU1, out aDV1, out aDUU1, out aDVV1, out aDUV1);
        theS2.D2(aU2, aV2, out aPt, out aDU2, out aDV2, out aDUU2, out aDVV2, out aDUV2);

        gp_Vec aN1 = aDU1.Crossed(aDV1), aN2 = aDU2.Crossed(aDV2);
        // Tangent vector to the intersection curve
        gp_Vec aCTan = aN1.Crossed(aN2);
        double aSqMagnFDer = aCTan.SquareMagnitude();

        if (aSqMagnFDer < 1.0e-8)
        {
            // Use 1.0e-4 (instead of aSmallValue) to provide
            // stable computation between different platforms.
            // See test bugs modalg_7 bug29807_sc01
            return -1.0;
        }

        double aDuS1 = 0.0, aDvS1 = 0.0, aDuS2 = 0.0, aDvS2 = 1.0;

        {
            // This algorithm is described in NonSingularProcessing() function
            // in ApproxInt_ImpPrmSvSurfaces.gxx file
            double aSqNMagn = aN1.SquareMagnitude();
            gp_Vec aTgU = aCTan.Crossed(aDU1), aTgV = aCTan.Crossed(aDV1);
            double aDeltaU = aTgV.SquareMagnitude() / aSqNMagn;
            double aDeltaV = aTgU.SquareMagnitude() / aSqNMagn;

            aDuS1 = Math.CopySign(Math.Sqrt(aDeltaU), aTgV.Dot(aN1));
            aDvS1 = -Math.CopySign(Math.Sqrt(aDeltaV), aTgU.Dot(aN1));

            aSqNMagn = aN2.SquareMagnitude();
            aTgU.SetXYZ(aCTan.Crossed(aDU2).XYZ());
            aTgV.SetXYZ(aCTan.Crossed(aDV2).XYZ());
            aDeltaU = aTgV.SquareMagnitude() / aSqNMagn;
            aDeltaV = aTgU.SquareMagnitude() / aSqNMagn;

            aDuS2 = Math.CopySign(Math.Sqrt(aDeltaU), aTgV.Dot(aN2));
            aDvS2 = -Math.CopySign(Math.Sqrt(aDeltaV), aTgU.Dot(aN2));
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

        double aA = aN1.Dot(aN1), aB = aN1.Dot(aN2), aC = aN2.Dot(aN2);
        double aDetSyst = aB * aB - aA * aC;

        if (Math.Abs(aDetSyst) < aSmallValue)
        {
            // Undetermined system solution
            return -1.0;
        }

        double aF1 = aDuS1 * aDuS1 * aDUU1.Dot(aN1) + 2.0 * aDuS1 * aDvS1 * aDUV1.Dot(aN1)
                     + aDvS1 * aDvS1 * aDVV1.Dot(aN1);
        double aF2 = aDuS2 * aDuS2 * aDUU2.Dot(aN2) + 2.0 * aDuS2 * aDvS2 * aDUV2.Dot(aN2)
                     + aDvS2 * aDvS2 * aDVV2.Dot(aN2);

        // Principal normal to the intersection curve
        gp_Vec aCNorm = (aF1 * aC - aF2 * aB) / aDetSyst * aN1 + (aA * aF2 - aF1 * aB) / aDetSyst * aN2;
        double aSqMagnSDer = aCNorm.CrossSquareMagnitude(aCTan);

        if (aSqMagnSDer < aSqSmallValue)
        { // Intersection curve has null curvature in observed point
            return Precision.Infinite();
        }

        // square of curvature radius
        double aFactSqRad = aSqMagnFDer * aSqMagnFDer * aSqMagnFDer / aSqMagnSDer;

        return Math.Sqrt(aFactSqRad);
    }
}
