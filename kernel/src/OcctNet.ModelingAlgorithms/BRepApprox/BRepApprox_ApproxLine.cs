// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepApprox/BRepApprox_ApproxLine.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepApprox/BRepApprox_ApproxLine.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// The line BRepApprox_Approx approximates: the points of an <see cref="IntSurf_LineOn2S"/>, or the poles of a 3D
/// B-spline curve and of two 2D B-spline curves (null curves give the origin).
/// </summary>
/// <remarks>
/// A working object (decision 609), sharing the curves and the line it is given; it is the line of BRepApprox's
/// instantiation of ApproxInt's generics (<see cref="IApproxInt_WLine{TSelf}"/>).
/// </remarks>
public class BRepApprox_ApproxLine : Transient, IApproxInt_WLine<BRepApprox_ApproxLine>
{
    private readonly Geom_BSplineCurve? myCurveXYZ;
    private readonly Geom2d_BSplineCurve? myCurveUV1;
    private readonly Geom2d_BSplineCurve? myCurveUV2;
    private readonly IntSurf_LineOn2S? myLineOn2S;

    public BRepApprox_ApproxLine(Geom_BSplineCurve? CurveXYZ, Geom2d_BSplineCurve? CurveUV1,
                                 Geom2d_BSplineCurve? CurveUV2)
    {
        myCurveXYZ = CurveXYZ;
        myCurveUV1 = CurveUV1;
        myCurveUV2 = CurveUV2;
    }

    /// <summary>The line on the points of <paramref name="lin"/>.</summary>
    /// <remarks><paramref name="theTang"/> is only for compatibility with IntPatch_WLine: it is not used.</remarks>
    public BRepApprox_ApproxLine(IntSurf_LineOn2S lin, bool theTang = false)
    {
        myLineOn2S = lin;
    }

    static BRepApprox_ApproxLine IApproxInt_WLine<BRepApprox_ApproxLine>.Create(IntSurf_LineOn2S lin, bool theTang)
        => new(lin, theTang);

    public int NbPnts()
    {
        if (myCurveXYZ is not null)
        {
            return (myCurveXYZ.NbPoles());
        }
        if (myCurveUV1 is not null)
        {
            return (myCurveUV1.NbPoles());
        }
        if (myCurveUV2 is not null)
        {
            return (myCurveUV2.NbPoles());
        }
        return (myLineOn2S!.NbPoints());
    }

    public IntSurf_PntOn2S Point(int Index)
    {
        if (myLineOn2S is not null)
        {
            if (myLineOn2S.NbPoints() != 0)
            {
                return (myLineOn2S.Value(Index));
            }
        }
        Point2D P1 = new(), P2 = new();
        Point3D P = new();
        if (myCurveXYZ is not null)
        {
            P = myCurveXYZ.Pole(Index);
        }
        if (myCurveUV1 is not null)
        {
            P1 = myCurveUV1.Pole(Index);
        }
        if (myCurveUV2 is not null)
        {
            P2 = myCurveUV2.Pole(Index);
        }

        IntSurf_PntOn2S aPntOn2S = new();
        aPntOn2S.SetValue(P, P1.X(), P1.Y(), P2.X(), P2.Y());

        return aPntOn2S;
    }
}
