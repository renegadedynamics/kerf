// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_Solid.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_Solid.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1998-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using static Occt.ShapeExtend_Status;
using static Kerf.Topology.ShapeType;
using static Kerf.Topology.Containment;

namespace Occt;

/// <summary>
/// Fixes a solid: its shells (by ShapeFix_Shell), and the orientation of its shells (a solid whose shells are badly
/// oriented, or are several solids, is rebuilt); also makes a solid from a shell.
/// </summary>
/// <remarks>
/// A working object (decision 21). <see cref="Init"/> only records the solid. <see cref="Perform"/> edits the shapes it
/// works on in place, as OCCT's: a solid holding frozen shapes (any solid a caller holds) is copied first
/// (<see cref="CopyAtEntry"/>, decision 583), the caller's solid never changes, and at the end the copies left
/// unchanged are the caller's own again, in the results and in the context. <see cref="Solid"/>, <see cref="Shape"/> and
/// <see cref="SolidFromShell"/> freeze what they return; ported code reads the internal forms.
/// </remarks>
public class ShapeFix_Solid : ShapeFix_Root
{
    /// <summary>The solid fixed.</summary>
    protected Shape   mySolid;
    /// <summary>The shell fixing tool.</summary>
    protected ShapeFix_Shell myFixShell;
    /// <summary>The status.</summary>
    protected int            myStatus;
    /// <summary>The mode of the shell fixes.</summary>
    protected int            myFixShellMode;
    /// <summary>The mode of the orientation fix of the shells.</summary>
    protected int            myFixShellOrientationMode;
    /// <summary>Whether a solid is made from an open shell.</summary>
    protected bool           myCreateOpenSolidMode;

    /// <summary>Empty constructor.</summary>
    public ShapeFix_Solid()
    {
        mySolid                   = new Shape();
        myStatus                  = ShapeExtend.EncodeStatus(ShapeExtend_OK);
        myFixShellMode            = -1;
        myFixShellOrientationMode = -1;
        myFixShell                = new ShapeFix_Shell();
        myCreateOpenSolidMode     = false;
    }

    /// <summary>Initializes by a solid.</summary>
    public ShapeFix_Solid(in Solid solid)
    {
        mySolid                   = new Shape();
        myStatus                  = ShapeExtend.EncodeStatus(ShapeExtend_OK);
        myFixShellMode            = -1;
        myFixShellOrientationMode = -1;
        myFixShell                = new ShapeFix_Shell();
        myCreateOpenSolidMode     = false;
        Init(solid);
    }

    /// <summary>Initializes by a solid.</summary>
    public virtual void Init(in Solid solid)
    {
        mySolid = solid;
        // mySolid = TopoDS::Solid(shape.EmptyCopied());
        // BRep_Builder B;
        // for( TopoDS_Iterator iter(solid); iter.More(); iter.Next())
        //  B.Add(mySolid,TopoDS::Shell(iter.Value()));
        myShape = solid;
    }

    //=================================================================================================

    private static void CollectSolids(NCollection_Sequence<Shape> aSeqShells,
                                      NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> anIndexedMapShellHoles,
                                      NCollection_DataMap<Shape, int> theMapStatus)
    {
        var aMapHoles      = new NCollection_Map<Shape>(ShapeHasher.Instance);
        var aMapShellHoles = new NCollection_DataMap<Shape, NCollection_List<Shape>>(ShapeHasher.Instance);
        for (int i1 = 1; i1 <= aSeqShells.Length(); i1++)
        {
            Shell aShell1 = ShapeCasts.Shell(aSeqShells.Value(i1));
            var          lshells = new NCollection_List<Shape>();
            aMapShellHoles.Bind(aShell1, lshells);
        }
        // Finds roots shells and hole shells.
        for (int i = 1; i <= aSeqShells.Length(); i++)
        {
            Shell aShell1   = ShapeCasts.Shell(aSeqShells.Value(i));
            var          aExpEdges = new Explorer(aShell1, ShapeType.Edge);
            if (!Tool.IsClosed(aShell1) || !aExpEdges.More())
            {
                continue;
            }
            var solid = new Solid();
            var B     = new BrepBuilder();
            B.MakeSolid(ref solid);
            B.Add(solid, aShell1);
            try
            {
                Containment infinstatus = Unknown;
                var          bsc3d       = new BRepClass3d_SolidClassifier(solid);
                int          st          = 0;
                if (!theMapStatus.IsBound(aShell1))
                {
                    bsc3d.PerformInfinitePoint(Kerf.Engine.Foundation.Precision.Confusion());
                    infinstatus = bsc3d.State();
                    if (infinstatus != Unknown && infinstatus != On)
                    {
                        st = (infinstatus == In ? 1 : 2);
                    }
                    theMapStatus.Bind(aShell1, st);
                }
                else
                {
                    st = theMapStatus.Find(aShell1);
                    if (st != 0)
                    {
                        infinstatus = (theMapStatus.Find(aShell1) == 1 ? In : Out);
                    }
                }
                if (st == 0)
                {
                    continue;
                }
                for (int j = 1; j <= aSeqShells.Length(); j++)
                {
                    if (i == j)
                    {
                        continue;
                    }
                    Shape aShell2 = aSeqShells.Value(j);
                    if (!Tool.IsClosed(aShell2))
                    {
                        continue;
                    }
                    if (aMapHoles.Contains(aShell2))
                    {
                        continue;
                    }
                    if (aMapShellHoles.IsBound(aShell2))
                    {
                        bool                           isAnalysis = false;
                        NCollection_List<Shape> ls         = aMapShellHoles.Find(aShell2);
                        for (var li = new NCollection_List<Shape>.Iterator(ls); li.More() && !isAnalysis; li.Next())
                        {
                            isAnalysis = li.Value().IsSame(aShell1);
                        }
                        if (isAnalysis)
                        {
                            continue;
                        }
                    }
                    Containment pointstatus = Unknown;
                    int          numon       = 0;
                    var          amapVert    = new NCollection_IndexedMap<Shape>(ShapeHasher.Instance);
                    for (var aExpVert = new Explorer(aShell2, ShapeType.Vertex); aExpVert.More() && amapVert.Extent() < 10;
                         aExpVert.Next())
                    {
                        amapVert.Add(aExpVert.Current());
                    }
                    for (int k = 1;
                         k <= amapVert.Extent() && (pointstatus == Unknown || (pointstatus == On && numon < 3));
                         k++)
                    {
                        Vertex aV  = ShapeCasts.Vertex(amapVert.FindKey(k));
                        Point3D        aPf = Tool.Pnt(aV);
                        bsc3d.Perform(aPf, Kerf.Engine.Foundation.Precision.Confusion());
                        pointstatus = bsc3d.State();
                        if (pointstatus == On)
                        {
                            numon++;
                        }
                    }
                    if (numon == 3 && pointstatus == On)
                    {
                        pointstatus = /*(bsc3d.State() == TopAbs_IN ? TopAbs_IN :*/ Out;
                    }
                    if (pointstatus != infinstatus)
                    {
                        aMapShellHoles.ChangeFind(aShell1).Append(aShell2);
                        if (aMapHoles.Contains(aShell2))
                        {
                            aMapHoles.Remove(aShell2);
                        }
                        else
                        {
                            aMapHoles.Add(aShell2);
                        }
                    }
                }
            }
            catch (FailureException)
            {
                continue;
            }
        }
        foreach (KeyValuePair<Shape, NCollection_List<Shape>> aItShellHoles in aMapShellHoles)
        {
            if (aMapHoles.Contains(aItShellHoles.Key))
            {
                continue;
            }
            NCollection_List<Shape> lHoles = aItShellHoles.Value;
            if (lHoles.IsEmpty())
            {
                continue;
            }
            for (var lItHoles = new NCollection_List<Shape>.Iterator(lHoles); lItHoles.More(); lItHoles.Next())
            {
                if (aMapHoles.Contains(lItHoles.Value()))
                {
                    NCollection_List<Shape> lUnHoles = aMapShellHoles.Find(lItHoles.Value());
                    for (var lItUnHoles = new NCollection_List<Shape>.Iterator(lUnHoles); lItUnHoles.More(); lItUnHoles.Next())
                    {
                        aMapHoles.Remove(lItUnHoles.Value());
                    }
                }
            }
        }
        foreach (Shape aHole in aMapHoles)
        {
            aMapShellHoles.UnBind(aHole);
        }
        for (int i = 1; i <= aSeqShells.Length(); ++i)
        {
            Shape aShell1 = aSeqShells.Value(i);
            if (aMapShellHoles.IsBound(aShell1))
            {
                NCollection_List<Shape> ls = aMapShellHoles.Find(aShell1);
                anIndexedMapShellHoles.Add(aShell1, ls);
            }
        }
    }

    //=================================================================================================

    private static bool CreateSolids(in Shape theShape, NCollection_IndexedMap<Shape> aMapSolids)
    {
        var  aSeqShells = new NCollection_Sequence<Shape>();
        bool isDone     = false;
        for (var aExpShell = new Explorer(theShape, ShapeType.Shell); aExpShell.More(); aExpShell.Next())
        {
            aSeqShells.Append(aExpShell.Current());
        }
        var aMapShellHoles = new NCollection_IndexedDataMap<Shape, NCollection_List<Shape>>(ShapeHasher.Instance);
        var aMapStatus     = new NCollection_DataMap<Shape, int>(ShapeHasher.Instance);
        CollectSolids(aSeqShells, aMapShellHoles, aMapStatus);
        var ShellSolid = new NCollection_IndexedDataMap<Shape, Shape>(ShapeHasher.Instance);
        // Defines correct orientation of shells
        for (int i = 1; i <= aMapShellHoles.Extent(); ++i)
        {
            Shell aShell    = ShapeCasts.Shell(aMapShellHoles.FindKey(i));
            var          aExpEdges = new Explorer(aShell, ShapeType.Edge);
            if (!Tool.IsClosed(aShell) || !aExpEdges.More())
            {
                ShellSolid.Add(aShell, aShell);
                isDone = true;
                continue;
            }
            var          B           = new BrepBuilder();
            Containment infinstatus = Unknown;
            var          aSolid      = new Solid();
            B.MakeSolid(ref aSolid);
            B.Add(aSolid, aShell);
            if (aMapStatus.IsBound(aShell))
            {
                int st = aMapStatus.Find(aShell);
                if (st != 0)
                {
                    infinstatus = (aMapStatus.Find(aShell) == 1 ? In : Out);
                }
            }
            else
            {
                try
                {
                    var bsc3d = new BRepClass3d_SolidClassifier(aSolid);
                    bsc3d.PerformInfinitePoint(Kerf.Engine.Foundation.Precision.Confusion());
                    infinstatus = bsc3d.State();
                }
                catch (FailureException)
                {
                    ShellSolid.Add(aShell, aSolid);
                    continue;
                }
            }
            if (infinstatus == In)
            {
                isDone = true;
                aShell.Reverse();
                var aTmpSolid = new Solid();
                B.MakeSolid(ref aTmpSolid);
                B.Add(aTmpSolid, aShell);
                aSolid = aTmpSolid;
            }
            NCollection_List<Shape> lHoles = aMapShellHoles[i];
            for (var lItHoles = new NCollection_List<Shape>.Iterator(lHoles); lItHoles.More(); lItHoles.Next())
            {
                Shell aShellHole = ShapeCasts.Shell(lItHoles.Value());
                if (aMapStatus.IsBound(aShellHole))
                {
                    infinstatus = (aMapStatus.Find(aShellHole) == 1 ? In : Out);
                }
                else
                {
                    var solid = new Solid();
                    B.MakeSolid(ref solid);
                    B.Add(solid, aShellHole);
                    var bsc3dHol = new BRepClass3d_SolidClassifier(solid);
                    bsc3dHol.PerformInfinitePoint(Kerf.Engine.Foundation.Precision.Confusion());
                    infinstatus = bsc3dHol.State();
                }
                if (infinstatus == Out)
                {
                    aShellHole.Reverse();
                    isDone = true;
                }
                B.Add(aSolid, aShellHole);
            }
            ShellSolid.Add(aShell, aSolid);
        }
        // Creation of compsolid from shells containing shared faces.
        var aMapFaceShells = new NCollection_IndexedDataMap<Shape, NCollection_List<Shape>>(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(theShape, ShapeType.Face, ShapeType.Shell, aMapFaceShells);
        for (int i = 1; i <= aMapFaceShells.Extent(); i++)
        {
            NCollection_List<Shape> lshells = aMapFaceShells.FindFromIndex(i);
            if (lshells.Extent() < 2)
            {
                continue;
            }
            var aCompSolid = new CompSolid();
            var aB         = new BrepBuilder();
            aB.MakeCompSolid(ref aCompSolid);
            isDone = (theShape.ShapeType() != ShapeType.CompSolid || isDone);
            int nbSol = 0;
            for (var lItSh = new NCollection_List<Shape>.Iterator(lshells); lItSh.More(); lItSh.Next())
            {
                if (ShellSolid.Contains(lItSh.Value()))
                {
                    Shape aShape  = ShellSolid.FindFromKey(lItSh.Value());
                    var          aExpSol = new Explorer(aShape, global::Kerf.Topology.ShapeType.Solid);
                    for (; aExpSol.More(); aExpSol.Next())
                    {
                        aB.Add(aCompSolid, aExpSol.Current());
                        nbSol++;
                    }
                }
            }
            if (nbSol > 1)
            {
                for (var lItSh1 = new NCollection_List<Shape>.Iterator(lshells); lItSh1.More(); lItSh1.Next())
                {
                    if (ShellSolid.Contains(lItSh1.Value()))
                    {
                        ShellSolid.ChangeFromKey(lItSh1.Value()) = aCompSolid;
                    }
                }
            }
        }
        for (int kk = 1; kk <= ShellSolid.Extent(); kk++)
        {
            if (!aMapSolids.Contains(ShellSolid.FindFromIndex(kk)))
            {
                aMapSolids.Add(ShellSolid.FindFromIndex(kk));
            }
        }
        isDone = (aMapSolids.Extent() > 1 || isDone);
        return isDone;
    }

    //=================================================================================================

    /// <summary>
    /// Fixes the shells of the solid (if the shell mode allows) and the orientation of the shells (if the orientation
    /// mode allows): a solid is made from a closed shell (or from an open one in <see cref="CreateOpenSolidMode"/>),
    /// several solids or a compsolid from shells that are several solids. Returns true if something was done.
    /// </summary>
    public virtual bool Perform(ProgressRange theProgress = default)
    {
        if (Context() is null)
        {
            SetContext(new ShapeBuild_ReShape());
        }
        var anEntry = new CopyAtEntry(myShape, Context());
        myShape = anEntry.Shape();
        mySolid = anEntry.Copied(mySolid);
        bool status = perform(theProgress);
        if (anEntry.IsCopied)
        {
            myShape = anEntry.Finish(myShape, [mySolid]);
            mySolid = anEntry.Final(mySolid);
        }
        else
        {
            anEntry.Finish(myShape);
        }
        return status;
    }

    // ShapeFix_Solid::Perform on the working solid
    private bool perform(ProgressRange theProgress)
    {
        bool status = false;
        if (Context() is null)
        {
            SetContext(new ShapeBuild_ReShape());
        }
        ShapeBuild_ReShape aContext = Context()!;
        myFixShell.SetContext(aContext);
        int          NbShells = 0;
        Shape S        = aContext.InternalApply(myShape);

        // Calculate number of underlying shells
        int aNbShells = 0;
        for (var aExpSh = new Explorer(S, ShapeType.Shell); aExpSh.More(); aExpSh.Next())
        {
            aNbShells++;
        }

        // Start progress scope (no need to check if progress exists -- it is safe)
        using var aPS = new ProgressScope(theProgress, "Fixing solid stage", 2);

        if (NeedFix(myFixShellMode))
        {
            // Start progress scope (no need to check if progress exists -- it is safe)
            using var aPSFixShell = new ProgressScope(aPS.Next(), "Fixing shell", aNbShells);

            // Fix shell by shell using ShapeFix_Shell tool
            for (var aExpSh = new Explorer(S, ShapeType.Shell); aExpSh.More() && aPSFixShell.More(); aExpSh.Next())
            {
                Shape sh = aExpSh.Current();

                myFixShell.Init(ShapeCasts.Shell(sh));
                if (myFixShell.Perform(aPSFixShell.Next()))
                {
                    status = true;
                    myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE1);
                }
                NbShells += myFixShell.NbShells();
            }

            // Halt algorithm in case of user's abort
            if (!aPSFixShell.More())
            {
                return false;
            }
        }
        else
        {
            NbShells = aNbShells;
        }

        if (!NeedFix(myFixShellOrientationMode))
        {
            myShape = aContext.InternalApply(myShape);
            return status;
        }

        if (NbShells == 1)
        {
            Shape tmpShape = aContext.InternalApply(myShape);
            var          aExp     = new Explorer(tmpShape, ShapeType.Shell);
            bool         isClosed = false;
            if (aExp.More())
            {
                Shell    aShtmp  = ShapeCasts.Shell(aExp.Current());
                var             sfb     = new ShapeAnalysis_FreeBounds(aShtmp);
                Compound aC1     = sfb.InternalGetClosedWires();
                Compound aC2     = sfb.InternalGetOpenWires();
                int             numedge = 0;
                var             aExp1   = new Explorer(aC1, ShapeType.Edge);
                for (; aExp1.More(); aExp1.Next())
                {
                    numedge++;
                }
                for (aExp1.Init(aC2, ShapeType.Edge); aExp1.More(); aExp1.Next())
                {
                    numedge++;
                }
                isClosed = (numedge == 0);
                aShtmp.Closed(isClosed);
            }

            if (isClosed || myCreateOpenSolidMode)
            {
                var          itersh = new Iterator(tmpShape);
                Shell aShell = new();
                if (itersh.More() && itersh.Value().ShapeType() == ShapeType.Shell)
                {
                    aShell = ShapeCasts.Shell(itersh.Value());
                }
                if (!aShell.IsNull)
                {
                    Solid aSol = InternalSolidFromShell(aShell);
                    if (ShapeExtend.DecodeStatus(myStatus, ShapeExtend_DONE2))
                    {
                        SendWarning(new Msg("FixAdvSolid.FixOrientation.MSG20")); // Orientation of shell was corrected.
                        aContext.Replace(tmpShape, aSol);
                        tmpShape = aSol;
                    }
                }
                mySolid = ShapeCasts.Solid(tmpShape);
            }
            else
            {
                status = true;
                myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE3);
                var aIt = new Iterator(tmpShape, false);
                aContext.Replace(tmpShape, aIt.Value());
                SendFail(new Msg("FixAdvSolid.FixShell.MSG10")); // Solid can not be created from open shell.
            }
        }
        else
        {
            Shape aResShape = aContext.InternalApply(myShape);
            var          aMapSolids = new NCollection_IndexedMap<Shape>(ShapeHasher.Instance);
            if (CreateSolids(aResShape, aMapSolids))
            {
                SendWarning(new Msg("FixAdvSolid.FixOrientation.MSG20")); // Orientation of shell was corrected..
                if (aMapSolids.Extent() == 1)
                {
                    Shape aResSol = aMapSolids.FindKey(1);
                    if (aResShape.ShapeType() == ShapeType.Shell && myCreateOpenSolidMode)
                    {
                        var solid = new Solid();
                        var B     = new BrepBuilder();
                        B.MakeSolid(ref solid);
                        B.Add(solid, aResSol);
                        mySolid = solid;
                    }
                    else
                    {
                        mySolid = aResSol;
                        if (aResSol.ShapeType() == ShapeType.Shell)
                        {
                            SendFail(new Msg("FixAdvSolid.FixShell.MSG10")); // Solid can not be created from open shell.
                        }
                    }
                    aContext.Replace(aResShape, mySolid);
                }
                else if (aMapSolids.Extent() > 1)
                {
                    SendWarning(new Msg("FixAdvSolid.FixOrientation.MSG30")); // Bad connected solid a few solids were created.
                    var aB    = new BrepBuilder();
                    var aComp = new Compound();
                    aB.MakeCompound(ref aComp);
                    using var aPSCreatingSolid = new ProgressScope(aPS.Next(), "Creating solid", aMapSolids.Extent());
                    for (int i = 1; (i <= aMapSolids.Extent()) && (aPSCreatingSolid.More()); i++, aPSCreatingSolid.Next())
                    {
                        Shape aResSh = aMapSolids.FindKey(i);
                        if (aResShape.ShapeType() == ShapeType.Shell && myCreateOpenSolidMode)
                        {
                            aResSh.Closed(false);
                            var solid = new Solid();
                            var B     = new BrepBuilder();
                            B.MakeSolid(ref solid);
                            B.Add(solid, aResSh);
                            aResSh = solid;
                        }
                        else if (aResShape.ShapeType() == ShapeType.Shell)
                        {
                            SendFail(new Msg("FixAdvSolid.FixShell.MSG10")); // Solid can not be created from open shell.
                        }
                        aB.Add(aComp, aResSh);
                    }
                    if (!aPSCreatingSolid.More())
                    {
                        return false; // aborted execution
                    }
                    aContext.Replace(aResShape, aComp);
                }
            }
        }
        myShape = aContext.InternalApply(myShape);
        return status;
    }

    //=================================================================================================

    /// <summary>Returns the resulting shape, frozen.</summary>
    public Shape Shape() => Published(myShape);

    /// <summary>Returns the resulting shape, unfrozen, for ported callers working on shapes they own.</summary>
    internal Shape InternalShape() => myShape;

    /// <summary>
    /// Makes a solid from a shell, reversing the shell when the solid it makes would hold the infinite point (status
    /// DONE2). The shell is shared (a frozen shell is the caller's own); the solid is frozen.
    /// </summary>
    public Solid SolidFromShell(in Shell shell) => ShapeCasts.Solid(Published(InternalSolidFromShell(shell)));

    /// <summary>
    /// <see cref="SolidFromShell"/> without freezing the solid, for ported callers working on shapes they own (OCCT
    /// sets the Free flag of an unfrozen shell).
    /// </summary>
    internal Solid InternalSolidFromShell(in Shell shell)
    {
        Shell sh = shell;
        if (!sh.Free() && !sh.TShape()!.IsFrozen) // OCCT.NET: frozen: a frozen shell keeps its Free flag
        {
            sh.Free(true);
        }
        var solid = new Solid();
        var B     = new BrepBuilder();
        B.MakeSolid(ref solid);
        B.Add(solid, sh);
        //   Pas encore fini : il faut une bonne orientation
        try
        {
            var    bsc3d = new BRepClass3d_SolidClassifier(solid);
            double t     = Kerf.Engine.Foundation.Precision.Confusion(); // tolerance moyenne
            bsc3d.PerformInfinitePoint(t);

            if (bsc3d.State() == In)
            {
                //         Ensuite, inverser C-A-D REPRENDRE LES SHELLS
                //         (l inversion du solide n est pas bien prise en compte)
                sh = shell;
                if (!sh.Free() && !sh.TShape()!.IsFrozen) // OCCT.NET: frozen
                {
                    sh.Free(true);
                }
                var soli2 = new Solid();
                B.MakeSolid(ref soli2); // on recommence
                sh.Reverse();
                B.Add(soli2, sh);
                solid = soli2;
                myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE2);
            }
        }
        catch (FailureException)
        {
            return solid;
        }
        return solid;
    }

    /// <summary>Returns the status (the bits of the fixes).</summary>
    public bool Status(ShapeExtend_Status theStatus) => ShapeExtend.DecodeStatus(myStatus, theStatus);

    /// <summary>Returns the fixed solid, frozen.</summary>
    public Shape Solid() => Published(mySolid);

    /// <summary>Returns the fixed solid, unfrozen, for ported callers working on shapes they own.</summary>
    internal Shape InternalSolid() => mySolid;

    /// <summary>Returns the shell fixing tool.</summary>
    public ShapeFix_Shell FixShellTool() => myFixShell;

    /// <summary>Sets the message registrator, for this tool and the shell tool.</summary>
    public override void SetMsgRegistrator(ShapeExtend_BasicMsgRegistrator? msgreg)
    {
        base.SetMsgRegistrator(msgreg);
        myFixShell.SetMsgRegistrator(msgreg);
    }

    /// <summary>Sets the basic precision, for this tool and the shell tool.</summary>
    public override void SetPrecision(double preci)
    {
        base.SetPrecision(preci);
        myFixShell.SetPrecision(preci);
    }

    /// <summary>Sets the minimal tolerance, for this tool and the shell tool.</summary>
    public override void SetMinTolerance(double mintol)
    {
        base.SetMinTolerance(mintol);
        myFixShell.SetMinTolerance(mintol);
    }

    /// <summary>Sets the maximal tolerance, for this tool and the shell tool.</summary>
    public override void SetMaxTolerance(double maxtol)
    {
        base.SetMaxTolerance(maxtol);
        myFixShell.SetMaxTolerance(maxtol);
    }

    /// <summary>The mode of the shell fixes (-1 default, 0 off, 1 on).</summary>
    public ref int FixShellMode() => ref myFixShellMode;

    /// <summary>The mode of the orientation fix of the shells (-1 default, 0 off, 1 on).</summary>
    public ref int FixShellOrientationMode() => ref myFixShellOrientationMode;

    /// <summary>Whether a solid is made from an open shell (false by default).</summary>
    public ref bool CreateOpenSolidMode() => ref myCreateOpenSolidMode;

    private static Shape Published(in Shape theShape)
    {
        theShape.TShape()?.FreezeGraph();
        return theShape;
    }
}
