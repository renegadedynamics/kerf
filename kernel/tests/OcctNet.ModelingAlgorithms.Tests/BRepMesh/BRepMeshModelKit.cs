// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;
using System.Reflection;
using System.Text;

namespace OcctNet.ModelingAlgorithms.Tests;

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

    public static TopoDS_Shape ReadDataOcc(string theName)
    {
        Assert.True(BRepTools.Read(out TopoDS_Shape aShape, Path.Combine(DataOcc, theName)), theName);
        return aShape;
    }

    /// <summary>The parameters as BRepMesh_IncrementalMesh completes them (initParameters).</summary>
    public static IMeshTools_Parameters Params(double theDeflection, double theAngle = 0.5, bool isRelative = false,
                                               bool isInParallel = false)
    {
        var aParams = new IMeshTools_Parameters
        {
            Deflection = theDeflection,
            Angle = theAngle,
            Relative = isRelative,
            InParallel = isInParallel,
        };
        aParams.DeflectionInterior = theDeflection;
        aParams.MinSize = Math.Max(IMeshTools_Parameters.RelMinSize() * theDeflection, Precision.Confusion());
        aParams.AngleInterior = 2.0 * theAngle;
        return aParams;
    }

    /// <summary>A context with unit 4C's tools (and the given face mesher), as BRepMesh_Context sets them.</summary>
    public static IMeshTools_Context Context(in TopoDS_Shape theShape, in IMeshTools_Parameters theParameters,
                                             IMeshTools_ModelAlgo? theFaceDiscret = null)
    {
        var aContext = new IMeshTools_Context();
        aContext.SetModelBuilder(new BRepMesh_ModelBuilder());
        aContext.SetEdgeDiscret(new BRepMesh_EdgeDiscret());
        aContext.SetModelHealer(new BRepMesh_ModelHealer());
        aContext.SetPreProcessor(new BRepMesh_ModelPreProcessor());
        aContext.SetFaceDiscret(theFaceDiscret);
        aContext.SetPostProcessor(new BRepMesh_ModelPostProcessor());
        aContext.SetShape(theShape);
        aContext.ChangeParameters() = theParameters;
        return aContext;
    }

    /// <summary>The discrete model of the shape, up to the stage.</summary>
    public static IMeshData_Model Model(in TopoDS_Shape theShape, in IMeshTools_Parameters theParameters,
                                        Stage theStage = Stage.PreProcessed)
    {
        IMeshTools_Context aContext = Context(theShape, theParameters);
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
    public sealed class FanFaceDiscret : IMeshTools_ModelAlgo
    {
        public int Meshed;

        protected override bool performInternal(IMeshData_Model? theModel, in IMeshTools_Parameters theParameters,
                                                Message_ProgressRange theRange)
        {
            for (int i = 0; i < theModel!.FacesNb(); i++)
            {
                IMeshData_Face aDFace = theModel.GetFace(i);
                if (aDFace.IsSet(IMeshData_Status.IMeshData_Reused) || aDFace.IsSet(IMeshData_Status.IMeshData_Failure))
                {
                    continue;
                }

                var aNodes = new List<gp_Pnt>();
                var aUV = new List<gp_Pnt2d>();
                for (int w = 0; w < aDFace.WiresNb(); w++)
                {
                    IMeshData_Wire aDWire = aDFace.GetWire(w);
                    for (int e = 0; e < aDWire.EdgesNb(); e++)
                    {
                        IMeshData_Edge aDEdge = aDWire.GetEdge(e);
                        IMeshData_PCurve aPCurve = aDEdge.GetPCurve(aDFace, aDWire.GetEdgeOrientation(e));
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

                var aTri = new Poly_Triangulation(aNodes.Count, aNodes.Count - 2, true);
                for (int k = 0; k < aNodes.Count; k++)
                {
                    aTri.SetNode(k + 1, aNodes[k]);
                    aTri.SetUVNode(k + 1, aUV[k]);
                }
                for (int k = 1; k <= aNodes.Count - 2; k++)
                {
                    aTri.SetTriangle(k, new Poly_Triangle(1, k + 1, k + 2));
                }
                BRepMesh_ShapeTool.AddInFace(aDFace.GetFace(), aTri);
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
    public static string Dump(IMeshData_Model theModel)
    {
        var aText = new StringBuilder();
        var anEdgeIndex = new Dictionary<IMeshData_Edge, int>(ReferenceEqualityComparer.Instance);
        var aFaceIndex = new Dictionary<IMeshData_Face, int>(ReferenceEqualityComparer.Instance);
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
            IMeshData_Face aF = theModel.GetFace(i);
            aText.Append(CultureInfo.InvariantCulture, $"F{i} s{aF.GetStatusMask():X} d{B(aF.GetDeflection())}\n");
            for (int w = 0; w < aF.WiresNb(); w++)
            {
                IMeshData_Wire aW = aF.GetWire(w);
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
            IMeshData_Edge anE = theModel.GetEdge(i);
            aText.Append(CultureInfo.InvariantCulture,
                         $"E{i} s{anE.GetStatusMask():X} d{B(anE.GetDeflection())} a{B(anE.GetAngularDeflection())} "
                         + $"sp{anE.GetSameParam()} sr{anE.GetSameRange()} dg{anE.GetDegenerated()}\n C");
            IMeshData_Curve aC = anE.GetCurve();
            for (int k = 0; k < aC.ParametersNb(); k++)
            {
                gp_Pnt aP = aC.GetPoint(k);
                aText.Append(CultureInfo.InvariantCulture, $" {B(aC.GetParameter(k))}:{B(aP.X())},{B(aP.Y())},{B(aP.Z())}");
            }
            aText.Append('\n');
            for (int p = 0; p < anE.PCurvesNb(); p++)
            {
                IMeshData_PCurve aPC = anE.GetPCurve(p);
                aText.Append(CultureInfo.InvariantCulture, $" P F{aFaceIndex[aPC.GetFace()]} {aPC.GetOrientation()}");
                for (int k = 0; k < aPC.ParametersNb(); k++)
                {
                    gp_Pnt2d aP = aPC.GetPoint(k);
                    aText.Append(CultureInfo.InvariantCulture, $" {B(aPC.GetParameter(k))}:{B(aP.X())},{B(aP.Y())}#{aPC.GetIndex(k)}");
                }
                aText.Append('\n');
            }
        }
        return aText.ToString();
    }

    private static string B(double theValue) => BitConverter.DoubleToInt64Bits(theValue).ToString("X16", CultureInfo.InvariantCulture);
}
