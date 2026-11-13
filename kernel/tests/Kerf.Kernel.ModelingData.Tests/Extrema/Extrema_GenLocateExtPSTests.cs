// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Extrema;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;

namespace Kerf.Kernel.ModelingData.Tests;

/// <summary>Upstream ec0167b409's Extrema_GenLocateExtPS_Test (decision 2409).</summary>
public class Extrema_GenLocateExtPSTests
{
    // ClosedSplineNormalizesStartingParameter: a cylinder made a closed, not periodic, B-spline, a point on it, and a
    // start one or two turns away from the point's parameter: the projection is found, inside the surface's range
    [Fact]
    public void A_start_beyond_a_closed_surfaces_range_is_brought_round_its_seam()
    {
        foreach (bool isExchanged in new[] { false, true })
        {
            BSplineSurface aSurface = global::Kerf.Engine.Geometry.Conversion.GeomConvert.SurfaceToBSplineSurface(
                new TrimmedSurface(new CylindricalSurface(new CoordinateSystem(Constants.XOY()), 2.0), 0.0, 2.0 * Math.PI,
                                   -1.0, 1.0));
            aSurface.SetUNotPeriodic();
            if (isExchanged)
            {
                aSurface.ExchangeUV();
            }

            var anAdaptor = new SurfaceAdaptor(aSurface);
            Assert.True(isExchanged ? anAdaptor.IsVClosed : anAdaptor.IsUClosed);
            Assert.False(isExchanged ? anAdaptor.IsVPeriodic : anAdaptor.IsUPeriodic);
            foreach (double aParameter in new[] { 0.2, 2.0 * Math.PI - 0.2 })
            {
                foreach (int aShift in new[] { -2, -1, 0, 1, 2 })
                {
                    string aCase = $"exchanged {isExchanged}, parameter {aParameter}, shift {aShift}";
                    double aStart = aParameter + aShift * 2.0 * Math.PI;
                    double aU = isExchanged ? 0.3 : aParameter;
                    double aV = isExchanged ? aParameter : 0.3;
                    Point3D aPoint = aSurface.Value(aU, aV);
                    var aProjection = new GenLocateExtPS(anAdaptor, global::Kerf.Engine.Foundation.Precision.PConfusion(),
                                                         global::Kerf.Engine.Foundation.Precision.PConfusion());
                    aProjection.Perform(aPoint, isExchanged ? aU : aStart, isExchanged ? aStart : aV);
                    Assert.True(aProjection.IsDone, aCase);
                    Assert.True(aProjection.SquareDistance() <= global::Kerf.Engine.Foundation.Precision.SquareConfusion(),
                                $"{aCase}: {Math.Sqrt(aProjection.SquareDistance())}");
                    aProjection.Point().Parameter(out double aResultU, out double aResultV);
                    Assert.InRange(aResultU, anAdaptor.FirstUParameter(), anAdaptor.LastUParameter());
                    Assert.InRange(aResultV, anAdaptor.FirstVParameter(), anAdaptor.LastVParameter());
                }
            }
        }
    }
}
