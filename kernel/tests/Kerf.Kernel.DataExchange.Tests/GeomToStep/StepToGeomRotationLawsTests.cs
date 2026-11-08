// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Exchange.Step.Basic;
using Kerf.Engine.Exchange.Step.Data;
using Kerf.Engine.Exchange.Step.Geometry;
using Kerf.Engine.Exchange.Step.Kinematics;
using Kerf.Engine.Exchange.Step.Representation;
using Converter = Kerf.Engine.Exchange.Step.Import.Geometry.StepToGeom;

namespace Kerf.Kernel.DataExchange.Tests;

/// <summary>
/// A rotation about a direction as yaw, pitch and roll (unit 12Q, decision 1753): composed as ISO 10303-105's
/// ypr_rotation, Rx(roll)·Ry(pitch)·Rz(yaw), the three angles give back the rotation about the axis, in the context's
/// plane angle unit, whichever way the axis points.
/// </summary>
public class StepToGeomRotationLawsTests
{
    private static TCollection_HAsciiString Name() => new("");

    private static SiUnitAndPlaneAngleUnit Radian(bool theMilli = false)
    {
        var aUnit = new SiUnitAndPlaneAngleUnit();
        aUnit.Init(theMilli, SiPrefix.SpMilli, SiUnitName.SunRadian);
        return aUnit;
    }

    private static ConversionBasedUnitAndPlaneAngleUnit Degree()
    {
        var aRadian = new Unit();
        aRadian.SetValue(Radian());
        var aFactor = new MeasureWithUnit();
        aFactor.SetValueComponent(Math.PI / 180);
        aFactor.SetUnitComponent(aRadian);
        var aDimensions = new DimensionalExponents();
        aDimensions.Init(0, 0, 0, 0, 0, 0, 0);
        var aDegree = new ConversionBasedUnitAndPlaneAngleUnit();
        aDegree.Init(aDimensions, new TCollection_HAsciiString("DEGREE"), aFactor);
        return aDegree;
    }

    private static GlobalUnitAssignedContext Context(params NamedUnit[] theUnits)
    {
        var aUnits = new NCollection_HArray1<NamedUnit>(1, theUnits.Length);
        for (int i = 0; i < theUnits.Length; i++)
        {
            aUnits.SetValue(i + 1, theUnits[i]);
        }
        var aContext = new GlobalUnitAssignedContext();
        aContext.Init(Name(), Name(), aUnits);
        return aContext;
    }

    private static SpatialRotation About(double x, double y, double z, double theAngle)
    {
        var aRatios = new NCollection_HArray1<double>(1, 3);
        aRatios.SetValue(1, x);
        aRatios.SetValue(2, y);
        aRatios.SetValue(3, z);
        var aDirection = new StepDirection();
        aDirection.Init(Name(), aRatios);
        var aRotation = new RotationAboutDirection();
        aRotation.Init(Name(), aDirection, theAngle);
        var aSpatial = new SpatialRotation();
        aSpatial.SetValue(aRotation);
        return aSpatial;
    }

    private static double[,] Product(double[,] a, double[,] b)
    {
        var c = new double[3, 3];
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                for (int k = 0; k < 3; k++)
                {
                    c[i, j] += a[i, k] * b[k, j];
                }
            }
        }
        return c;
    }

    // The rotation by a about the unit axis (x, y, z), by Rodrigues' formula
    private static double[,] AboutAxis(double x, double y, double z, double a)
    {
        double n = Math.Sqrt(x * x + y * y + z * z);
        x /= n;
        y /= n;
        z /= n;
        double c = Math.Cos(a), s = Math.Sin(a), m = 1 - c;
        return new[,]
        {
            { x * x * m + c, x * y * m - z * s, x * z * m + y * s },
            { x * y * m + z * s, y * y * m + c, y * z * m - x * s },
            { x * z * m - y * s, y * z * m + x * s, z * z * m + c },
        };
    }

    private static double[,] Ypr(double theYaw, double thePitch, double theRoll)
    {
        double cy = Math.Cos(theYaw), sy = Math.Sin(theYaw);
        double cp = Math.Cos(thePitch), sp = Math.Sin(thePitch);
        double cr = Math.Cos(theRoll), sr = Math.Sin(theRoll);
        double[,] aRz = { { cy, -sy, 0 }, { sy, cy, 0 }, { 0, 0, 1 } };
        double[,] aRy = { { cp, 0, sp }, { 0, 1, 0 }, { -sp, 0, cp } };
        double[,] aRx = { { 1, 0, 0 }, { 0, cr, -sr }, { 0, sr, cr } };
        return Product(aRx, Product(aRy, aRz));
    }

    private static void AssertSame(double[,] theExpected, double[,] theActual)
    {
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                Assert.Equal(theExpected[i, j], theActual[i, j], 9);
            }
        }
    }

    [Theory]
    [InlineData(1, 0, 0, 0.5)]
    [InlineData(-1, 0, 0, 0.5)]
    [InlineData(1, 0, 0, Math.PI)]
    [InlineData(0, 0, 1, 2.0)]
    [InlineData(0, 0, -1, 2.0)]
    [InlineData(0, 0, 1, 5.0)] // past a half turn, about the same axis
    [InlineData(0, 1, 0, 0.5)]
    [InlineData(0, 1, 0, 2.5)] // past a quarter turn: yaw and roll a half turn each
    [InlineData(0, -1, 0, 2.5)]
    [InlineData(1, 2, 3, 0.7)]
    [InlineData(1, 2, 3, 2.8)]
    [InlineData(-3, 1, -2, -1.9)]
    [InlineData(1, 0, 1, Math.PI)] // pitch a right angle: only yaw and roll together are fixed, and roll is 0
    [InlineData(-1, 0, 1, Math.PI)]
    public void Yaw_pitch_and_roll_compose_to_the_rotation_about_the_axis(double x, double y, double z, double theAngle)
    {
        NCollection_HArray1<double>? aYpr = Converter.MakeYprRotation(About(x, y, z, theAngle), Context(Radian()));
        Assert.NotNull(aYpr);
        Assert.Equal(3, aYpr.Length());
        AssertSame(AboutAxis(x, y, z, theAngle), Ypr(aYpr.Value(1), aYpr.Value(2), aYpr.Value(3)));
    }

    [Theory]
    [InlineData(1, 2, 3, 40.0)]
    [InlineData(0, 1, 0, 120.0)]
    [InlineData(0, 0, 1, -30.0)]
    public void In_degrees_the_angles_are_in_degrees(double x, double y, double z, double theDegrees)
    {
        NCollection_HArray1<double>? aYpr = Converter.MakeYprRotation(About(x, y, z, theDegrees), Context(Degree()));
        Assert.NotNull(aYpr);
        double k = Math.PI / 180;
        AssertSame(AboutAxis(x, y, z, theDegrees * k), Ypr(aYpr.Value(1) * k, aYpr.Value(2) * k, aYpr.Value(3) * k));
    }

    [Fact]
    public void In_milliradians_the_angles_are_in_milliradians()
    {
        NCollection_HArray1<double>? aYpr = Converter.MakeYprRotation(About(2, -1, 1, 600), Context(Radian(theMilli: true)));
        Assert.NotNull(aYpr);
        AssertSame(AboutAxis(2, -1, 1, 0.6), Ypr(aYpr.Value(1) / 1000, aYpr.Value(2) / 1000, aYpr.Value(3) / 1000));
    }

    [Fact]
    public void A_rotation_given_as_yaw_pitch_and_roll_is_itself_and_no_rotation_is_none()
    {
        var aGiven = new NCollection_HArray1<double>(1, 3);
        aGiven.SetValue(1, 0.1);
        aGiven.SetValue(2, 0.2);
        aGiven.SetValue(3, 0.3);
        var aSpatial = new SpatialRotation();
        aSpatial.SetValue(aGiven);
        Assert.Same(aGiven, Converter.MakeYprRotation(aSpatial, null));

        NCollection_HArray1<double>? aNone = Converter.MakeYprRotation(About(1, 2, 3, 0.0), Context(Radian()));
        Assert.NotNull(aNone);
        Assert.Equal([0.0, 0.0, 0.0], new[] { aNone.Value(1), aNone.Value(2), aNone.Value(3) });
    }

    [Fact]
    public void Without_one_plane_angle_unit_in_radians_there_is_no_rotation()
    {
        Assert.Null(Converter.MakeYprRotation(About(1, 2, 3, 0.5), null));
        Assert.Null(Converter.MakeYprRotation(About(1, 2, 3, 0.5), Context()));
        Assert.Null(Converter.MakeYprRotation(About(1, 2, 3, 0.5), Context(Radian(), Degree())));
        var aSteradian = new SiUnitAndPlaneAngleUnit();
        aSteradian.Init(false, SiPrefix.SpMilli, SiUnitName.SunSteradian);
        Assert.Null(Converter.MakeYprRotation(About(1, 2, 3, 0.5), Context(aSteradian)));
    }
}
