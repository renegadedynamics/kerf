// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Tools;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Api;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Intersection.Tools;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.ShapeType;

namespace Kerf.Kernel.ModelingAlgorithms.Tests.BOPTools;

/// <summary>
/// The per-thread IntTools_Context of BOPTools_Parallel (decision 580) with real context solvers: the BOPTools and
/// IntTools queries a pave filler's loops make, run serially and in parallel at several degrees, give the serial
/// answers bit for bit.
/// </summary>
public class BOPTools_ParallelContextTests
{
    /// <summary>One face's queries, as a BOPAlgo solver would make them through its context.</summary>
    private sealed class FaceSolver(Face theFace, Solid theSolid, Point3D[] thePoints)
        : BooleanParallel.IContextSolver<IntersectionContext>
    {
        private IntersectionContext? myContext;

        public Face Face => theFace;
        public Solid Solid => theSolid;
        public Point3D[] Points => thePoints;

        public FaceSolver Fresh() => new(theFace, theSolid, thePoints);

        public List<long> Result { get; } = [];

        public void SetContext(IntersectionContext theContext) => myContext = theContext;

        public void Perform()
        {
            IntersectionContext aCtx = myContext!;
            Point3D aP = new();
            Point2D aP2D = new();
            int iErr = AlgoTools3D.PointInFace(theFace, ref aP, ref aP2D, aCtx);
            Result.Add(iErr);
            Result.Add(BitConverter.DoubleToInt64Bits(aP.X));
            Result.Add(BitConverter.DoubleToInt64Bits(aP2D.Y));
            Result.Add((long)aCtx.StatePointFace(theFace, aP2D));
            foreach (Point3D aPnt in thePoints)
            {
                Result.Add(aCtx.IsValidPointForFace(aPnt, theFace, 1.0e-3) ? 1 : 0);
                Result.Add((long)AlgoTools.ComputeState(aPnt, theSolid, 1.0e-7, aCtx));
                ProjectPointOnSurface aProj = aCtx.ProjPS(theFace);
                aProj.Perform(aPnt);
                Result.Add(aProj.IsDone ? BitConverter.DoubleToInt64Bits(aProj.LowerDistance()) : -1);
            }
        }
    }

    private static List<FaceSolver> MakeSolvers(Shape theShape, int theSeed)
    {
        Solid aSolid = ShapeCasts.Solid(new Explorer(theShape, ShapeType.Solid).Current());
        Box aBox = new();
        Bounds.Add(theShape, ref aBox);
        aBox.Get(out double x0, out double y0, out double z0, out double x1, out double y1, out double z1);
        Random aRnd = new(theSeed);
        List<FaceSolver> aSolvers = [];
        foreach (Face aF in BOPToolsTestShapes.Faces(theShape))
        {
            Point3D[] aPnts = new Point3D[3];
            for (int i = 0; i < aPnts.Length; i++)
            {
                aPnts[i] = new Point3D(x0 + (x1 - x0) * aRnd.NextDouble(), y0 + (y1 - y0) * aRnd.NextDouble(),
                                      z0 + (z1 - z0) * aRnd.NextDouble());
            }
            aSolvers.Add(new FaceSolver(aF, aSolid, aPnts));
        }
        // Each face twice, shuffled, so that a context is reused across faces in different orders.
        return [.. aSolvers.Concat(aSolvers.Select(s => s.Fresh())).OrderBy(_ => aRnd.Next())];
    }

    private static List<List<long>> Run(List<FaceSolver> theTemplate, bool theParallel)
    {
        List<FaceSolver> aSolvers = [.. theTemplate.Select(s => s.Fresh())];
        BooleanParallel.Perform(theParallel, aSolvers, new IntersectionContext());
        return [.. aSolvers.Select(s => s.Result)];
    }

    [Theory]
    [InlineData("box")]
    [InlineData("cylinder")]
    [InlineData("bottle.brep")]
    public void ContextSolvers_ParallelGivesTheSerialResult(string theName)
    {
        Shape aShape = BOPToolsTestShapes.Primitives().Concat(theName.EndsWith(".brep", StringComparison.Ordinal)
                                                                         ? [(theName, BRepMeshModelKit.ReadDataOcc(theName))]
                                                                         : [])
                                                .First(p => p.Name == theName).Shape;
        List<FaceSolver> aTemplate = MakeSolvers(aShape, 6401);
        List<List<long>> aSerial;
        using (OsdParallel.ForceSerialScope())
        {
            aSerial = Run(aTemplate, true);
        }
        Assert.Equal(aSerial, Run(aTemplate, false));
        foreach (int aDegree in new[] { 2, Environment.ProcessorCount })
        {
            using (OsdParallel.MaxDegreeOfParallelismScope(aDegree))
            {
                for (int aRepeat = 0; aRepeat < 1; aRepeat++)
                {
                    Assert.Equal(aSerial, Run(aTemplate, true));
                }
            }
        }
    }
}
