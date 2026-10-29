// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_IncrementalMesh.hxx
// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_IncrementalMesh.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2013-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.IMeshData_Status;

namespace Occt;

/// <summary>
/// Meshes a shape, keeping the meshes of its faces that are already fine enough ("incremental").
/// </summary>
/// <remarks>
/// <para>
/// <b>Writing into the shape</b> (decision 489). OCCT writes the triangulations into the faces of the shape it is
/// given, and the polygons into its edges.
/// <list type="bullet">
/// <item>On a shape whose graph holds a frozen TShape (any shape a caller holds), the port meshes
/// <c>BRepTools.ExactCopyForWrite</c> of it; then <c>BRepTools.KeepUnchanged</c> gives back the input's own sub-shapes
/// wherever meshing changed nothing (decisions 416, 468), and the result is frozen. <see cref="BRepMesh_DiscretRoot.Shape"/>
/// then returns the meshed result and <see cref="History"/> maps the input's sub-shapes to the result's. The input is
/// never written; meshing a shape that is already meshed finely enough gives back the input itself.</item>
/// <item>On an unfrozen shape (ported code's), it meshes in place, as OCCT does; <see cref="History"/> is empty.</item>
/// </list>
/// So build123d.NET's <c>new BRepMesh_IncrementalMesh(copy, tol, rel, ang, true)</c> becomes
/// <c>new BRepMesh_IncrementalMesh(shape, tol, rel, ang, true).Shape()</c>. The functional twin is <see cref="Meshed(in TopoDS_Shape, in IMeshTools_Parameters)"/>.
/// </para>
/// <para>
/// <b>Unit 4A</b> ports this class's shell: the default <see cref="Perform(Message_ProgressRange)"/> uses a context
/// without tools (which meshes nothing) until unit 4D's BRepMesh_Context replaces it; <see cref="Perform(IMeshTools_Context, Message_ProgressRange)"/>
/// is OCCT's and runs any context.
/// </para>
/// </remarks>
public class BRepMesh_IncrementalMesh : BRepMesh_DiscretRoot
{
    // Default flag to control parallelization for BRepMesh_IncrementalMesh tool returned for Mesh Factory
    // (process-wide, with an internal setter: decisions 415, 492).
    private static volatile bool IS_IN_PARALLEL = false;

    protected IMeshTools_Parameters myParameters = new();
    protected bool myModified;
    protected int myStatus;

    // OCCT.NET: the copy for write (decision 489)
    private BRepTools_History myHistory = new();
    private bool myToCopyAlways;

    /// <summary>An algorithm without a shape.</summary>
    public BRepMesh_IncrementalMesh()
    {
        myModified = false;
        myStatus = (int)IMeshData_NoError;
    }

    /// <summary>
    /// Meshes the shape with the linear deflection (relative to each edge's size if <paramref name="isRelative"/>; the
    /// faces then take their edges' maximum) and the angular deflection, in parallel if <paramref name="isInParallel"/>.
    /// </summary>
    /// <exception cref="Standard_NumericError">The deflection or the angle is too small.</exception>
    public BRepMesh_IncrementalMesh(in TopoDS_Shape theShape,
                                    double theLinDeflection,
                                    bool isRelative = false,
                                    double theAngDeflection = 0.5,
                                    bool isInParallel = false)
    {
        myModified = false;
        myStatus = (int)IMeshData_NoError;
        myParameters.Deflection = theLinDeflection;
        myParameters.Angle = theAngDeflection;
        myParameters.Relative = isRelative;
        myParameters.InParallel = isInParallel;

        myShape = theShape;
        Perform();
    }

    /// <summary>Meshes the shape with the parameters.</summary>
    /// <exception cref="Standard_NumericError">The deflection or the angle is too small.</exception>
    public BRepMesh_IncrementalMesh(in TopoDS_Shape theShape,
                                    in IMeshTools_Parameters theParameters,
                                    Message_ProgressRange theRange = default)
    {
        myParameters = theParameters;
        myShape = theShape;
        Perform(theRange);
    }

    /// <summary>Meshes the shape.</summary>
    public override void Perform(Message_ProgressRange theRange = default)
    {
        IMeshTools_Context aContext = newDefaultContext();
        Perform(aContext, theRange);
    }

    /// <summary>Meshes the shape with a custom context.</summary>
    public void Perform(IMeshTools_Context theContext, Message_ProgressRange theRange = default)
    {
        initParameters();

        // OCCT.NET: a shape holding frozen TShapes is meshed on an exact copy (decision 489)
        TopoDS_Shape anInput = Shape();
        bool isCopy = !anInput.IsNull() && (myToCopyAlways || BRepTools.HoldsFrozen(anInput));
        TopoDS_Shape aTarget = isCopy ? BRepTools.ExactCopyForWrite(anInput, out _) : anInput;
        myHistory = new BRepTools_History();

        theContext.SetShape(aTarget);
        theContext.ChangeParameters() = myParameters;
        theContext.ChangeParameters().CleanModel = false;

        using var aPS = new Message_ProgressScope(theRange, "Perform incmesh", 10);
        IMeshTools_MeshBuilder aIncMesh = new(theContext);
        aIncMesh.Perform(aPS.Next(9));
        if (!aPS.More())
        {
            myStatus = (int)IMeshData_UserBreak;
            return;
        }
        myStatus = (int)IMeshData_NoError;
        IMeshData_Model? aModel = theContext.GetModel();
        if (aModel is not null)
        {
            for (int aFaceIt = 0; aFaceIt < aModel.FacesNb(); ++aFaceIt)
            {
                IMeshData_Face aDFace = aModel.GetFace(aFaceIt);
                myStatus |= aDFace.GetStatusMask();

                for (int aWireIt = 0; aWireIt < aDFace.WiresNb(); ++aWireIt)
                {
                    IMeshData_Wire aDWire = aDFace.GetWire(aWireIt);
                    myStatus |= aDWire.GetStatusMask();
                }
            }
        }

        // OCCT.NET: publish the meshed copy, sharing what meshing left unchanged
        if (isCopy)
        {
            myShape = BRepTools.KeepUnchanged(anInput, aTarget, out myHistory);
            myShape.TShape()?.FreezeGraph();
        }

        aPS.Next(1);
        setDone();
    }

    /// <summary>
    /// The functional form: the shape meshed with the parameters, frozen, sharing the input's sub-shapes wherever
    /// meshing changed nothing. The input is never written, frozen or not (decision 489).
    /// </summary>
    /// <exception cref="Standard_NumericError">The deflection or the angle is too small.</exception>
    public static TopoDS_Shape Meshed(in TopoDS_Shape theShape, in IMeshTools_Parameters theParameters)
        => Meshed(theShape, theParameters, out _);

    /// <summary>
    /// <see cref="Meshed(in TopoDS_Shape, in IMeshTools_Parameters)"/>, with the history from the input's vertices,
    /// edges, faces and solids to the result's.
    /// </summary>
    public static TopoDS_Shape Meshed(in TopoDS_Shape theShape, in IMeshTools_Parameters theParameters,
                                      out BRepTools_History theHistory)
    {
        BRepMesh_IncrementalMesh anAlgo = new();
        anAlgo.myParameters = theParameters;
        anAlgo.myToCopyAlways = true;
        anAlgo.SetShape(theShape);
        anAlgo.Perform();
        theHistory = anAlgo.History();
        return anAlgo.Shape();
    }

    // accessing to parameters

    /// <summary>The meshing parameters.</summary>
    public IMeshTools_Parameters Parameters() => myParameters;

    /// <summary>The meshing parameters, for writing.</summary>
    public ref IMeshTools_Parameters ChangeParameters() => ref myParameters;

    /// <summary>The modified flag (never set, as in OCCT).</summary>
    public bool IsModified() => myModified;

    /// <summary>The status flags (<see cref="IMeshData_Status"/>) met while meshing.</summary>
    public int GetStatusFlags() => myStatus;

    /// <summary>
    /// The history of the meshed copy (OCCT.NET, decision 489): the input's vertices, edges, faces and solids modified
    /// into the result's. Empty when the shape was meshed in place, or left as it was.
    /// </summary>
    public BRepTools_History History() => myHistory;

    // Initializes specific parameters
    private void initParameters()
    {
        if (myParameters.Deflection < Precision.Confusion())
        {
            throw new Standard_NumericError(
                "BRepMesh_IncrementalMesh::initParameters : invalid parameter value");
        }

        if (myParameters.DeflectionInterior < Precision.Confusion())
        {
            myParameters.DeflectionInterior = myParameters.Deflection;
        }

        if (myParameters.MinSize < Precision.Confusion())
        {
            myParameters.MinSize =
                Max(IMeshTools_Parameters.RelMinSize()
                        * Min(myParameters.Deflection, myParameters.DeflectionInterior),
                    Precision.Confusion());
        }

        if (myParameters.Angle < Precision.Angular())
        {
            throw new Standard_NumericError(
                "BRepMesh_IncrementalMesh::initParameters : invalid parameter value");
        }

        if (myParameters.AngleInterior < Precision.Angular())
        {
            myParameters.AngleInterior = 2.0 * myParameters.Angle;
        }
    }

    // The context of Perform(): OCCT's `new BRepMesh_Context(myParameters.MeshAlgo)`. Unit 4D replaces this stub,
    // a context without tools, which meshes nothing (IMeshTools_MeshBuilder sets Message_Fail1).
    private IMeshTools_Context newDefaultContext()
    {
        return new IMeshTools_Context();
    }

    // plugin API

    /// <summary>
    /// The plugin interface of the mesh factories: an algorithm for the shape with the deflections, in parallel if
    /// <see cref="IsParallelDefault"/>, not yet performed. Returns 0 (no error).
    /// </summary>
    public static int Discret(in TopoDS_Shape theShape,
                              double theDeflection,
                              double theAngle,
                              out BRepMesh_DiscretRoot theAlgo)
    {
        BRepMesh_IncrementalMesh anAlgo = new();
        anAlgo.ChangeParameters().Deflection = theDeflection;
        anAlgo.ChangeParameters().Angle = theAngle;
        anAlgo.ChangeParameters().InParallel = IS_IN_PARALLEL;
        anAlgo.SetShape(theShape);
        theAlgo = anAlgo;
        return 0; // no error
    }

    /// <summary>The parallel flag that <see cref="Discret"/> sets (for the mesh factories).</summary>
    public static bool IsParallelDefault() => IS_IN_PARALLEL;

    /// <summary>
    /// Sets the parallel flag of <see cref="Discret"/>. Internal: a process-wide setting that only Draw-lite's
    /// <c>mpparallel</c> changes (decision 492).
    /// </summary>
    internal static void SetParallelDefault(bool theInParallel) => IS_IN_PARALLEL = theInParallel;
}
