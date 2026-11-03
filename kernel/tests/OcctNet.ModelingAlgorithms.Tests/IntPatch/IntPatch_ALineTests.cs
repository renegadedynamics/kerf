// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Intersection;
using Kerf.Geometry;
using static Occt.IntPatch_IType;
using static Occt.IntSurf_TypeTrans;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>Known answers of IntPatch_ALine on the curves of a cylinder and an off-centre sphere (IntAna_IntQuadQuad).</summary>
public class IntPatch_ALineTests
{
    // x^2 + y^2 = 1 and (x - 0.5)^2 + y^2 + z^2 = 4: two closed curves, z = +-sqrt(2.75 + x).
    private static readonly Cylinder theCyl = new(new CoordinateSystem(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 1.0);
    private static readonly Sphere theSph = new(new CoordinateSystem(new Point3D(0.5, 0, 0), new Direction3D(0, 0, 1)), 2.0);

    private static IntPatch_ALine MakeLine()
    {
        IntQuadQuad anInter = new(theCyl, new Quadric(theSph), 1.0e-7);
        Assert.True(anInter.IsDone);
        Assert.True(anInter.CurveCount >= 1);
        return new IntPatch_ALine(anInter.Curve(1), false, IntSurf_In, IntSurf_Out);
    }

    private static Curve2DAdaptorBase Arc() => new CurveAdaptor2D(new LineCurve2D(new Line2D(new Point2D(0, 0), new Direction2D(1, 0))));

    private static IntPatch_Point Vertex(IntPatch_ALine theLine, double theParam, Curve2DAdaptorBase? theArcOnS1 = null)
    {
        IntPatch_Point aPnt = new();
        aPnt.SetValue(theLine.Value(theParam), 1.0e-7, false);
        aPnt.SetParameter(theParam);
        if (theArcOnS1 is not null)
        {
            aPnt.SetArc(true, theArcOnS1, theParam, new IntSurf_Transition(), new IntSurf_Transition());
        }
        return aPnt;
    }

    [Fact]
    public void ValueD1AndFindParameterFollowTheCurve()
    {
        IntPatch_ALine aLine = MakeLine();
        Assert.Equal(IntPatch_Analytic, aLine.ArcType());
        double aFirst = aLine.FirstParameter(out _);
        double aLast = aLine.LastParameter(out _);
        Assert.True(aLast > aFirst);

        for (int i = 1; i < 10; ++i)
        {
            double t = aFirst + (aLast - aFirst) * i / 10.0;
            Point3D aP = aLine.Value(t);
            Assert.Equal(1.0, aP.X() * aP.X() + aP.Y() * aP.Y(), 1.0e-9);
            Assert.Equal(4.0, aP.SquareDistance(new Point3D(0.5, 0, 0)), 1.0e-9);

            Assert.True(aLine.D1(t, out Point3D aPd, out Vector3D aDu));
            Assert.True(aPd.IsEqual(aP, 1.0e-12));
            double h = 1.0e-6;
            Vector3D aFd = new(aLine.Value(t - h), aLine.Value(t + h));
            Assert.True(aDu.IsEqual(aFd.Divided(2.0 * h), 1.0e-5, 1.0e-5));

            NCollection_List<double> aParams = new();
            aLine.FindParameter(aP, aParams);
            Assert.True(aParams.Extent() >= 1);
            bool aFound = false;
            foreach (double aPrm in aParams)
            {
                aFound |= Math.Abs(aPrm - t) < 1.0e-7;
            }
            Assert.True(aFound);
        }

        Curve aCopy = aLine.Curve();
        Assert.Equal(aLine.Value(1.0), aCopy.Value(1.0));
    }

    [Fact]
    public void ComputeVertexParametersMergesAndDropsAsOcct()
    {
        IntPatch_ALine aLine = MakeLine();
        double aFirst = aLine.FirstParameter(out _);
        double aLast = aLine.LastParameter(out _);
        double aMid = 0.5 * (aFirst + aLast);
        Curve2DAdaptorBase anArc1 = Arc(), anArc2 = Arc();

        aLine.AddVertex(Vertex(aLine, aLast - 0.1)); // last, on no restriction: dropped
        aLine.AddVertex(Vertex(aLine, aMid)); // same parameter as the next, on no restriction: killed
        aLine.AddVertex(Vertex(aLine, aMid, anArc1));
        aLine.AddVertex(Vertex(aLine, aFirst + 0.1, anArc2));
        aLine.ComputeVertexParameters(1.0e-7);

        Assert.Equal(2, aLine.NbVertex());
        Assert.Equal(aFirst + 0.1, aLine.Vertex(1).ParameterOnLine());
        Assert.Same(anArc2, aLine.Vertex(1).ArcOnS1());
        Assert.Equal(aMid, aLine.Vertex(2).ParameterOnLine());
        Assert.Same(anArc1, aLine.Vertex(2).ArcOnS1());

        aLine.ChangeVertex(2).SetTolerance(0.25);
        Assert.Equal(0.25, aLine.Vertex(2).Tolerance());
    }

    [Fact]
    public void EqualParametersShareTheirRestrictions()
    {
        IntPatch_ALine aLine = MakeLine();
        double aFirst = aLine.FirstParameter(out _);
        double aLast = aLine.LastParameter(out _);
        double aMid = 0.5 * (aFirst + aLast);
        Curve2DAdaptorBase anArc1 = Arc(), anArc2 = Arc();

        IntPatch_Point aOnS2 = Vertex(aLine, aMid);
        aOnS2.SetArc(false, anArc2, 0.75, new IntSurf_Transition(), new IntSurf_Transition());
        aLine.AddVertex(Vertex(aLine, aMid, anArc1));
        aLine.AddVertex(aOnS2);
        aLine.AddVertex(Vertex(aLine, aFirst + 0.2, Arc()));
        aLine.ComputeVertexParameters(1.0e-7);

        // One on S1 only and one on S2 only at the same parameter: no rule kills either; the last loop gives each the
        // other's arc.
        Assert.Equal(3, aLine.NbVertex());
        IntPatch_Point aV2 = aLine.Vertex(2), aV3 = aLine.Vertex(3);
        Assert.True(aV2.IsOnDomS1() && aV2.IsOnDomS2());
        Assert.True(aV3.IsOnDomS1() && aV3.IsOnDomS2());
        Assert.Same(anArc1, aV3.ArcOnS1());
        Assert.Same(anArc2, aV2.ArcOnS2());
    }
}
