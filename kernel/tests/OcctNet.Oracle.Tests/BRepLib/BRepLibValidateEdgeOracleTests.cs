#if HAS_ORACLE
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Xunit.Abstractions;
using N = Build123d.Occt;
using static Occt.TopAbs_Orientation;

namespace OcctNet.Oracle.Tests;

/// <summary>
/// BRepLib_ValidateEdge against native OCCT. BRepLib_ValidateEdge and BRepLib_CheckCurveOnSurface aren't bound
/// natively; BRepCheck_Analyzer with geometric controls calls ValidateEdge for each pcurve of each edge
/// (BRepCheck_Edge::InContext), so the comparison goes through it: twin triangular faces on a plane whose pcurves
/// are moved and turned by random amounts around the edge tolerance (the vertices' tolerance is large, so only the
/// edges can fail), checked natively with <c>BRepCheck_Analyzer(F, true, false, exact)</c> and by the port with
/// ValidateEdge set up as BRepCheck_Edge sets it up (exit at the tolerance, CheckTolerance, the approximate and the
/// exact methods).
/// </summary>
public class BRepLibValidateEdgeOracleTests(ITestOutputHelper output)
{
    private const int Scenes = 300;
    private const double VertexTol = 1.0;

    [Fact]
    public void Pcurves_are_validated_as_native_OCCT_validates_them()
    {
        var anOracle = new Oracle(output);
        int aValidFaces = 0, anInvalidEdges = 0, anEdges = 0;
        for (int aScene = 0; aScene < Scenes; aScene++)
        {
            bool isExact = aScene % 2 == 1;
            double anEdgeTol = Math.Pow(10, anOracle.Real(-6, -2));

            // a counter-clockwise triangle on z = 0
            var aP = new gp_Pnt[3];
            do
            {
                for (int i = 0; i < 3; i++)
                {
                    aP[i] = new gp_Pnt(anOracle.Real(-10, 10), anOracle.Real(-10, 10), 0);
                }
            }
            while (Math.Abs(new gp_Vec(aP[0], aP[1]).Crossed(new gp_Vec(aP[0], aP[2])).Z()) < 10);

            if (new gp_Vec(aP[0], aP[1]).Crossed(new gp_Vec(aP[0], aP[2])).Z() < 0)
            {
                (aP[1], aP[2]) = (aP[2], aP[1]);
            }

            var aNB = new N.BRep_Builder();
            var aB = new BRep_Builder();
            using var aNO = new N.gp_Pnt(0, 0, 0);
            using var aNZ = new N.gp_Dir(0, 0, 1);
            using var aNX = new N.gp_Dir(1, 0, 0);
            using var aNAx3 = new N.gp_Ax3(aNO, aNZ, aNX);
            var aNPlane = new N.Geom_Plane(aNAx3);
            var aPlane = new Geom_Plane(new gp_Ax3(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1), new gp_Dir(1, 0, 0)));
            var aNF = new N.TopoDS_Face();
            aNB.MakeFace(aNF, aNPlane, anEdgeTol);
            TopoDS_Face aF = new();
            aB.MakeFace(ref aF, aPlane, anEdgeTol);

            var aNV = new N.TopoDS_Vertex[3];
            var aV = new TopoDS_Vertex[3];
            for (int i = 0; i < 3; i++)
            {
                aNV[i] = new N.TopoDS_Vertex();
                using var aNP = Oracle.ToNative(aP[i]);
                aNB.MakeVertex(aNV[i], aNP, VertexTol);
                aV[i] = new TopoDS_Vertex();
                aB.MakeVertex(ref aV[i], aP[i], VertexTol);
            }

            var aNW = new N.TopoDS_Wire();
            aNB.MakeWire(aNW);
            TopoDS_Wire aW = new();
            aB.MakeWire(ref aW);
            bool isPortValid = true;
            for (int i = 0; i < 3; i++)
            {
                gp_Pnt aP1 = aP[i], aP2 = aP[(i + 1) % 3];
                double dx = aP2.X() - aP1.X(), dy = aP2.Y() - aP1.Y();
                double aLength = aP1.Distance(aP2);

                // the pcurve: the edge's image moved by about the tolerance and turned slightly (or exact)
                double anOffset = anOracle.Int(0, 2) != 0 ? 0.0 : anEdgeTol * anOracle.Real(0.2, 2.0);
                double anAngle = anOracle.Bool() ? 0.0 : anEdgeTol * anOracle.Real(-0.6, 0.6) / aLength;
                double aDirAngle = anOracle.Angle();
                double u0 = aP1.X() + anOffset * Math.Cos(aDirAngle), v0 = aP1.Y() + anOffset * Math.Sin(aDirAngle);
                double aCos = Math.Cos(anAngle), aSin = Math.Sin(anAngle);
                double du = dx * aCos - dy * aSin, dv = dx * aSin + dy * aCos;

                var aNE = new N.TopoDS_Edge();
                using (var aNLP = Oracle.ToNative(aP1))
                using (var aNLD = new N.gp_Dir(dx, dy, 0))
                {
                    aNB.MakeEdge(aNE, new N.Geom_Line(aNLP, aNLD), anEdgeTol);
                }

                aNB.Add(aNE, aNV[i].Oriented(N.TopAbs_Orientation.TopAbs_FORWARD));
                aNB.Add(aNE, aNV[(i + 1) % 3].Oriented(N.TopAbs_Orientation.TopAbs_REVERSED));
                aNB.Range(aNE, 0.0, aLength);
                using (var aNP2 = new N.gp_Pnt2d(u0, v0))
                using (var aND2 = new N.gp_Dir2d(du, dv))
                {
                    aNB.UpdateEdge(aNE, new N.Geom2d_Line(aNP2, aND2), aNF, anEdgeTol);
                }

                aNB.Add(aNW, aNE);

                var aLine = new Geom_Line(aP1, new gp_Dir(dx, dy, 0));
                var aPCurve = new Geom2d_Line(new gp_Pnt2d(u0, v0), new gp_Dir2d(du, dv));
                TopoDS_Edge anE = new();
                aB.MakeEdge(ref anE, aLine, anEdgeTol);
                aB.Add(anE, aV[i].Oriented(TopAbs_FORWARD));
                aB.Add(anE, aV[(i + 1) % 3].Oriented(TopAbs_REVERSED));
                aB.Range(anE, 0.0, aLength);
                aB.UpdateEdge(anE, aPCurve, aF, anEdgeTol);
                aB.Add(aW, anE);

                // ValidateEdge as BRepCheck_Edge::InContext runs it
                var aRef = new GeomAdaptor_Curve(aLine, 0.0, aLength);
                var anACS = new Adaptor3d_CurveOnSurface(new Geom2dAdaptor_Curve(aPCurve, 0.0, aLength),
                                                         new GeomAdaptor_Surface(aPlane));
                var aValidate = new BRepLib_ValidateEdge(aRef, anACS, true);
                aValidate.SetExitIfToleranceExceeded(anEdgeTol);
                aValidate.SetExactMethod(isExact);
                aValidate.Process();
                bool isEdgeValid = aValidate.IsDone() && aValidate.CheckTolerance(anEdgeTol);
                anEdges++;
                if (!isEdgeValid)
                {
                    anInvalidEdges++;
                    isPortValid = false;
                }
            }

            aNW.Closed(true);
            aNB.Add(aNF, aNW);
            aW.Closed(true);
            aB.Add(aF, aW);
            aF.TShape()!.FreezeGraph();

            bool isNativeValid = new N.BRepCheck_Analyzer(aNF, true, false, isExact).IsValid();
            anOracle.Same(isNativeValid ? 1 : 0, isPortValid ? 1 : 0, $"scene {aScene} (exact {isExact}): IsValid");
            if (isPortValid)
            {
                aValidFaces++;
            }
        }

        output.WriteLine($"{Scenes} faces, {aValidFaces} valid; {anEdges} edges, {anInvalidEdges} with an invalid pcurve");
        anOracle.Report("BRepLib_ValidateEdge through BRepCheck_Analyzer.IsValid");
    }
}
#endif
