// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using static Occt.IntPatch_IType;
using static Occt.IntSurf_Situation;
using static Occt.IntSurf_TypeTrans;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>Known answers of IntPatch_GLine, following OCCT's C++ (IntPatch_GLine.cxx).</summary>
public class IntPatch_GLineTests
{
    private static readonly Circle3D theCircle = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 1.0);

    private static Curve2DAdaptorBase Arc() => new CurveAdaptor2D(new LineCurve2D(new Line2D(new Point2D(0, 0), new Direction2D(1, 0))));

    private static IntPatch_Point Vertex(double theParam, double theU1 = 0.0, Curve2DAdaptorBase? theArcOnS1 = null)
    {
        IntPatch_Point aPnt = new();
        aPnt.SetValue(global::Kerf.Engine.Geometry.ElCLib.Value(theParam, theCircle), 1.0e-7, false);
        aPnt.SetParameters(theU1, 0.0, 0.0, 0.0);
        aPnt.SetParameter(theParam);
        if (theArcOnS1 is not null)
        {
            aPnt.SetArc(true, theArcOnS1, 0.5, new IntSurf_Transition(), new IntSurf_Transition());
        }
        return aPnt;
    }

    [Fact]
    public void ConicsAndTransitions()
    {
        IntPatch_GLine aCircLine = new(theCircle, false, IntSurf_In, IntSurf_Out);
        Assert.Equal(IntPatch_Circle, aCircLine.ArcType());
        Assert.Equal(IntSurf_In, aCircLine.TransitionOnS1());
        Assert.Equal(IntSurf_Out, aCircLine.TransitionOnS2());
        Assert.Equal(theCircle, aCircLine.Circle());
        Assert.Throws<DomainErrorException>(() => aCircLine.Line());
        Assert.Throws<DomainErrorException>(() => aCircLine.SituationS1());

        Line3D aLin = new(new Point3D(1, 2, 3), new Direction3D(0, 1, 0));
        IntPatch_GLine aLinLine = new(aLin, true, IntSurf_Inside, IntSurf_Outside);
        Assert.Equal(IntPatch_Lin, aLinLine.ArcType());
        Assert.True(aLinLine.IsTangent());
        Assert.Equal(IntSurf_Touch, aLinLine.TransitionOnS1());
        Assert.Equal(IntSurf_Outside, aLinLine.SituationS2());
        Assert.True(aLinLine.Line().Location().IsEqual(aLin.Location(), 0.0));
        Assert.True(aLinLine.Line().Direction().IsEqual(aLin.Direction(), 0.0));

        Ellipse3D anElips = new(new Frame3D(), 3.0, 2.0);
        IntPatch_GLine anElLine = new(anElips, false);
        Assert.Equal(IntSurf_Undecided, anElLine.TransitionOnS1());
        Assert.Equal(3.0, anElLine.Ellipse().MajorRadius());
        Assert.Equal(2.0, anElLine.Ellipse().MinorRadius());

        Assert.Equal(0.5, new IntPatch_GLine(new Parabola3D(new Frame3D(), 0.5), false).Parabola().Focal());
        Assert.Equal(2.0, new IntPatch_GLine(new Hyperbola3D(new Frame3D(), 4.0, 2.0), false).Hyperbola().MinorRadius());
        Assert.False(aCircLine.HasFirstPoint());
        Assert.Throws<DomainErrorException>(() => aCircLine.FirstPoint());
    }

    [Fact]
    public void VerticesOutOfOrderAreSorted()
    {
        IntPatch_GLine aLine = new(theCircle, false, IntSurf_In, IntSurf_Out);
        aLine.AddVertex(Vertex(3.0));
        aLine.AddVertex(Vertex(1.0));
        aLine.AddVertex(Vertex(2.0));
        aLine.ComputeVertexParameters(1.0e-7);

        Assert.Equal(3, aLine.NbVertex());
        Assert.Equal(1.0, aLine.Vertex(1).ParameterOnLine());
        Assert.Equal(2.0, aLine.Vertex(2).ParameterOnLine());
        Assert.Equal(3.0, aLine.Vertex(3).ParameterOnLine());
        // OCCT sets the first and last indices after the sort, without setting the flags
        Assert.False(aLine.HasFirstPoint());
        aLine.SetFirstPoint(1);
        aLine.SetLastPoint(3);
        Assert.Equal(1.0, aLine.FirstPoint().ParameterOnLine());
        Assert.Equal(3.0, aLine.LastPoint().ParameterOnLine());
    }

    [Fact]
    public void OffDomainVertexNearARestrictionVertexIsRemoved()
    {
        IntPatch_GLine aLine = new(theCircle, false, IntSurf_In, IntSurf_Out);
        Curve2DAdaptorBase anArc = Arc();
        aLine.AddVertex(Vertex(1.0, 0.0, anArc));
        aLine.AddVertex(Vertex(1.0 + 1.0e-7));
        aLine.AddVertex(Vertex(2.0));
        aLine.ComputeVertexParameters(1.0e-7);

        Assert.Equal(2, aLine.NbVertex());
        Assert.True(aLine.Vertex(1).IsOnDomS1());
        Assert.Equal(1.0, aLine.Vertex(1).ParameterOnLine());
        Assert.Equal(2.0, aLine.Vertex(2).ParameterOnLine());
    }

    [Fact]
    public void SeamVerticesOfEqualParameterAreSeparatedByAPeriod()
    {
        // Two vertices at 0 on different arcs of S1, whose U1 differ by the period: the one with the larger U1 is moved
        // to 2*PI (OCCT's case 14 of the seam test).
        IntPatch_GLine aLine = new(theCircle, false, IntSurf_In, IntSurf_Out);
        aLine.AddVertex(Vertex(0.0, 0.0, Arc()));
        aLine.AddVertex(Vertex(0.0, 2.0 * Math.PI, Arc()));
        aLine.ComputeVertexParameters(1.0e-7);

        Assert.Equal(2, aLine.NbVertex());
        Assert.Equal(0.0, aLine.Vertex(1).ParameterOnLine());
        aLine.Vertex(1).ParametersOnS1(out double aU1, out _);
        Assert.Equal(0.0, aU1);
        Assert.Equal(2.0 * Math.PI, aLine.Vertex(2).ParameterOnLine());
        aLine.Vertex(2).ParametersOnS1(out aU1, out _);
        Assert.Equal(2.0 * Math.PI, aU1);
    }

    [Fact]
    public void AddVertexOnClosedConicWrapsIntoTheBounds()
    {
        IntPatch_GLine aLine = new(theCircle, false, IntSurf_In, IntSurf_Out);
        aLine.AddVertex(Vertex(0.0));
        aLine.AddVertex(Vertex(Math.PI));
        aLine.SetFirstPoint(1);
        aLine.SetLastPoint(2);

        aLine.AddVertex(Vertex(2.0 * Math.PI + 0.5));
        Assert.Equal(3, aLine.NbVertex());
        Assert.Equal(2.0 * Math.PI + 0.5 - 2.0 * Math.PI, aLine.Vertex(3).ParameterOnLine());

        aLine.AddVertex(Vertex(4.0)); // in neither [0, PI] nor [0, PI] + 2*PI: dropped
        Assert.Equal(3, aLine.NbVertex());

        // Replace appends, as OCCT's does
        aLine.Replace(1, Vertex(1.5));
        Assert.Equal(4, aLine.NbVertex());
        Assert.Equal(0.0, aLine.Vertex(1).ParameterOnLine());
    }

    [Fact]
    public void AddVertexOnOpenConicRejectsOutsideTheBounds()
    {
        Line3D aLin = new(new Point3D(0, 0, 0), new Direction3D(1, 0, 0));
        IntPatch_GLine aLine = new(aLin, false);
        IntPatch_Point aPnt = new();
        aPnt.SetParameter(0.0);
        aLine.AddVertex(aPnt);
        aPnt.SetParameter(10.0);
        aLine.AddVertex(aPnt);
        aLine.SetFirstPoint(1);
        aLine.SetLastPoint(2);
        aPnt.SetParameter(11.0);
        aLine.AddVertex(aPnt);
        Assert.Equal(2, aLine.NbVertex());
        aPnt.SetParameter(5.0);
        aLine.AddVertex(aPnt);
        Assert.Equal(3, aLine.NbVertex());
    }
}
