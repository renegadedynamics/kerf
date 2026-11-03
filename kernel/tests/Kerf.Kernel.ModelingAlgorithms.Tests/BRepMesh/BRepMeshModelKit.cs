// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Meshing;
using Kerf.Engine.Meshing.Model;
using Kerf.Engine.Meshing.Tools;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using System.Globalization;
using System.Reflection;
using System.Text;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// Unit 4C's model tools run as BRepMesh_Context chains them (IMeshTools_MeshBuilder: build, discretize the edges,
/// heal, pre-process), a stub face mesher, and a bit-exact dump of a discrete model.
/// </summary>
internal static class BRepMeshModelKit
{
    /// <summary>How far <see cref="Model"/> goes.</summary>
    public enum Stage
    {
        Built = 1,
        EdgesDiscretized = 2,
        Healed = 3,
        PreProcessed = 4,
    }

    public static string DataOcc
        => Path.Combine(typeof(BRepMeshModelKit).Assembly.GetCustomAttributes<AssemblyMetadataAttribute>()
                                                     .Single(a => a.Key == "OcctSourceDir").Value!,
                        "data", "occ");

    public static Shape ReadDataOcc(string theName)
    {
        Assert.True(BrepTools.Read(out Shape aShape, Path.Combine(DataOcc, theName)), theName);
        return aShape;
    }

    /// <summary>The parameters as BRepMesh_IncrementalMesh completes them (initParameters).</summary>
    public static ToolParameters Params(double theDeflection, double theAngle = 0.5, bool isRelative = false,
                                               bool isInParallel = false)
    {
        var aParams = new ToolParameters
        {
            Deflection = theDeflection,
            Angle = theAngle,
            Relative = isRelative,
            InParallel = isInParallel,
        };
        aParams.DeflectionInterior = theDeflection;
        aParams.MinSize = Math.Max(ToolParameters.RelMinSize() * theDeflection, global::Kerf.Engine.Foundation.Precision.Confusion());
        aParams.AngleInterior = 2.0 * theAngle;
        return aParams;
    }

    /// <summary>A context with unit 4C's tools (and the given face mesher), as BRepMesh_Context sets them.</summary>
    public static ToolContext Context(in Shape theShape, in ToolParameters theParameters,
                                             ModelAlgo? theFaceDiscret = null)
    {
        var aContext = new ToolContext();
        aContext.SetModelBuilder(new MeshModelBuilder());
        aContext.SetEdgeDiscret(new EdgeDiscret());
        aContext.SetModelHealer(new ModelHealer());
        aContext.SetPreProcessor(new ModelPreProcessor());
        aContext.SetFaceDiscret(theFaceDiscret);
        aContext.SetPostProcessor(new ModelPostProcessor());
        aContext.SetShape(theShape);
        aContext.ChangeParameters() = theParameters;
        return aContext;
    }

    /// <summary>The discrete model of the shape, up to the stage.</summary>
    public static ModelModel Model(in Shape theShape, in ToolParameters theParameters,
                                        Stage theStage = Stage.PreProcessed)
    {
        ToolContext aContext = Context(theShape, theParameters);
        Assert.True(aContext.BuildModel());
        if (theStage >= Stage.EdgesDiscretized)
        {
            Assert.True(aContext.DiscretizeEdges());
        }
        if (theStage >= Stage.Healed)
        {
            Assert.True(aContext.HealModel());
        }
        if (theStage >= Stage.PreProcessed)
        {
            Assert.True(aContext.PreProcessModel());
        }
        return aContext.GetModel()!;
    }

    /// <summary>
    /// Meshes each face that is neither reused nor failed with a fan over its pcurves' points (one node per point,
    /// numbered in wire order), registering the nodes in the pcurves and attaching the triangulation through
    /// BRepMesh_ShapeTool, as unit 4D's algorithms do.
    /// </summary>
    public sealed class FanFaceDiscret : ModelAlgo
    {
        public int Meshed;

        protected override bool PerformInternal(ModelModel? theModel, in ToolParameters theParameters,
                                                global::Kerf.Engine.Foundation.ProgressRange theRange)
        {
            for (int i = 0; i < theModel!.FacesNb(); i++)
            {
                FaceModel aDFace = theModel.GetFace(i);
                if (aDFace.IsSet(StatusModel.Reused) || aDFace.IsSet(StatusModel.Failure))
                {
                    continue;
                }

                var aNodes = new List<Point3D>();
                var aUV = new List<Point2D>();
                for (int w = 0; w < aDFace.WiresNb(); w++)
                {
                    WireModel aDWire = aDFace.GetWire(w);
                    for (int e = 0; e < aDWire.EdgesNb(); e++)
                    {
                        EdgeModel aDEdge = aDWire.GetEdge(e);
                        PCurveModel aPCurve = aDEdge.GetPCurve(aDFace, aDWire.GetEdgeOrientation(e));
                        for (int k = 0; k < aPCurve.ParametersNb(); k++)
                        {
                            aNodes.Add(aDEdge.GetCurve().GetPoint(k));
                            aUV.Add(aPCurve.GetPoint(k));
                            aPCurve.GetIndex(k) = aNodes.Count;
                        }
                    }
                }

                if (aNodes.Count < 3)
                {
                    continue;
                }

                var aTri = new Triangulation(aNodes.Count, aNodes.Count - 2, true);
                for (int k = 0; k < aNodes.Count; k++)
                {
                    aTri.SetNode(k + 1, aNodes[k]);
                    aTri.SetUVNode(k + 1, aUV[k]);
                }
                for (int k = 1; k <= aNodes.Count - 2; k++)
                {
                    aTri.SetTriangle(k, new Triangle1(1, k + 1, k + 2));
                }
                ShapeTool.AddInFace(aDFace.GetFace(), aTri);
                Meshed++;
            }
            return true;
        }
    }

    /// <summary>
    /// Every value of the model, doubles as their bits: statuses, deflections, wires (edges by model index, with their
    /// orientations), each edge's flags, 3D points and parameters, and each pcurve's face, orientation, points,
    /// parameters and indices.
    /// </summary>
    public static string Dump(ModelModel theModel)
    {
        var aText = new StringBuilder();
        var anEdgeIndex = new Dictionary<EdgeModel, int>(ReferenceEqualityComparer.Instance);
        var aFaceIndex = new Dictionary<FaceModel, int>(ReferenceEqualityComparer.Instance);
        for (int i = 0; i < theModel.EdgesNb(); i++)
        {
            anEdgeIndex.Add(theModel.GetEdge(i), i);
        }
        for (int i = 0; i < theModel.FacesNb(); i++)
        {
            aFaceIndex.Add(theModel.GetFace(i), i);
        }

        aText.Append(CultureInfo.InvariantCulture, $"maxsize {B(theModel.GetMaxSize())}\n");
        for (int i = 0; i < theModel.FacesNb(); i++)
        {
            FaceModel aF = theModel.GetFace(i);
            aText.Append(CultureInfo.InvariantCulture, $"F{i} s{aF.GetStatusMask():X} d{B(aF.GetDeflection())}\n");
            for (int w = 0; w < aF.WiresNb(); w++)
            {
                WireModel aW = aF.GetWire(w);
                aText.Append(CultureInfo.InvariantCulture, $" W s{aW.GetStatusMask():X} d{B(aW.GetDeflection())}:");
                for (int e = 0; e < aW.EdgesNb(); e++)
                {
                    aText.Append(CultureInfo.InvariantCulture, $" {anEdgeIndex[aW.GetEdge(e)]}{aW.GetEdgeOrientation(e)}");
                }
                aText.Append('\n');
            }
        }

        for (int i = 0; i < theModel.EdgesNb(); i++)
        {
            EdgeModel anE = theModel.GetEdge(i);
            aText.Append(CultureInfo.InvariantCulture,
                         $"E{i} s{anE.GetStatusMask():X} d{B(anE.GetDeflection())} a{B(anE.GetAngularDeflection())} "
                         + $"sp{anE.GetSameParam()} sr{anE.GetSameRange()} dg{anE.GetDegenerated()}\n C");
            CurveModel aC = anE.GetCurve();
            for (int k = 0; k < aC.ParametersNb(); k++)
            {
                Point3D aP = aC.GetPoint(k);
                aText.Append(CultureInfo.InvariantCulture, $" {B(aC.GetParameter(k))}:{B(aP.X())},{B(aP.Y())},{B(aP.Z())}");
            }
            aText.Append('\n');
            for (int p = 0; p < anE.PCurvesNb(); p++)
            {
                PCurveModel aPC = anE.GetPCurve(p);
                aText.Append(CultureInfo.InvariantCulture, $" P F{aFaceIndex[aPC.GetFace()]} {aPC.GetOrientation()}");
                for (int k = 0; k < aPC.ParametersNb(); k++)
                {
                    Point2D aP = aPC.GetPoint(k);
                    aText.Append(CultureInfo.InvariantCulture, $" {B(aPC.GetParameter(k))}:{B(aP.X())},{B(aP.Y())}#{aPC.GetIndex(k)}");
                }
                aText.Append('\n');
            }
        }
        return aText.ToString();
    }

    private static string B(double theValue) => BitConverter.DoubleToInt64Bits(theValue).ToString("X16", CultureInfo.InvariantCulture);
}
