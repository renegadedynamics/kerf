// Port of: src/ModelingData/TKGeomBase/GTests/Geom2dConvert_CompCurveToBSplineCurve_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace OcctNet.ModelingData.Tests;

public class Geom2dConvert_CompCurveToBSplineCurveTest
{
    [Fact]
    public void OCC30747_ClosedContourFromCircleArcs()
    {
        // OCC30747: 2d Curves concatenator must properly handle closed contours.
        // Split a full circle into 10 arcs and assemble them into a closed BSpline.
        Geom2d_Circle aCirc = new GC_MakeCircle2d(new Point2D(0, 0), 50).Value();
        Assert.NotNull(aCirc);

        double aF = aCirc.FirstParameter();
        double aL = aCirc.LastParameter();
        const int aNb = 10;
        double aDelta = (aF + aL) / aNb;

        Geom2d_TrimmedCurve aFTrim = new(aCirc, aF, aDelta);
        Geom2dConvert_CompCurveToBSplineCurve aRes = new(aFTrim);

        for (int anId = 1; anId < aNb; anId++)
        {
            Geom2d_TrimmedCurve aLTrim;
            if (anId == (aNb - 1))
            {
                aLTrim = new Geom2d_TrimmedCurve(aCirc, anId * aDelta, aF);
            }
            else
            {
                aLTrim = new Geom2d_TrimmedCurve(aCirc, anId * aDelta, (anId + 1) * aDelta);
            }

            aRes.Add(aLTrim, global::Kerf.Engine.Foundation.Precision.PConfusion());
        }

        Geom2d_BSplineCurve? aBSpline = aRes.BSplineCurve();
        Assert.NotNull(aBSpline);
        Assert.True(aBSpline.IsClosed(), "Assembled BSpline curve from closed circle arcs must be closed");
    }
}
