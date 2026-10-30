// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.IntPatch_IType;
using static Occt.IntSurf_TypeTrans;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>Known answers of IntPatch_ALine on the curves of a cylinder and an off-centre sphere (IntAna_IntQuadQuad).</summary>
public class IntPatch_ALineTests
{
    // x^2 + y^2 = 1 and (x - 0.5)^2 + y^2 + z^2 = 4: two closed curves, z = +-sqrt(2.75 + x).
    private static readonly gp_Cylinder theCyl = new(new gp_Ax3(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), 1.0);
    private static readonly gp_Sphere theSph = new(new gp_Ax3(new gp_Pnt(0.5, 0, 0), new gp_Dir(0, 0, 1)), 2.0);

    private static IntPatch_ALine MakeLine()
    {
        IntAna_IntQuadQuad anInter = new(theCyl, new IntAna_Quadric(theSph), 1.0e-7);
        Assert.True(anInter.IsDone());
        Assert.True(anInter.NbCurve() >= 1);
        return new IntPatch_ALine(anInter.Curve(1), false, IntSurf_In, IntSurf_Out);
    }

    private static Adaptor2d_Curve2d Arc() => new Geom2dAdaptor_Curve(new Geom2d_Line(new gp_Lin2d(new gp_Pnt2d(0, 0), new gp_Dir2d(1, 0))));

    private static IntPatch_Point Vertex(IntPatch_ALine theLine, double theParam, Adaptor2d_Curve2d? theArcOnS1 = null)
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
            gp_Pnt aP = aLine.Value(t);
            Assert.Equal(1.0, aP.X() * aP.X() + aP.Y() * aP.Y(), 1.0e-9);
            Assert.Equal(4.0, aP.SquareDistance(new gp_Pnt(0.5, 0, 0)), 1.0e-9);

            Assert.True(aLine.D1(t, out gp_Pnt aPd, out gp_Vec aDu));
            Assert.True(aPd.IsEqual(aP, 1.0e-12));
            double h = 1.0e-6;
            gp_Vec aFd = new(aLine.Value(t - h), aLine.Value(t + h));
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

        IntAna_Curve aCopy = aLine.Curve();
        Assert.Equal(aLine.Value(1.0), aCopy.Value(1.0));
    }

    [Fact]
    public void ComputeVertexParametersMergesAndDropsAsOcct()
    {
        IntPatch_ALine aLine = MakeLine();
        double aFirst = aLine.FirstParameter(out _);
        double aLast = aLine.LastParameter(out _);
        double aMid = 0.5 * (aFirst + aLast);
        Adaptor2d_Curve2d anArc1 = Arc(), anArc2 = Arc();

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
        Adaptor2d_Curve2d anArc1 = Arc(), anArc2 = Arc();

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
