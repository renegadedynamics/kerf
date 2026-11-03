// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_Solid.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_Solid.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1998-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Classification;
using Kerf.Engine.Foundation;
using Kerf.Engine.Healing.Analysis;
using Kerf.Engine.Healing.Build;
using Kerf.Engine.Healing.Extend;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Healing.Extend.ExtendStatus;
using static Kerf.Topology.ShapeType;
using static Kerf.Topology.Containment;

namespace Kerf.Engine.Healing.Fixes;

/// <summary>
/// Fixes a solid: its shells (by ShapeFix_Shell), and the orientation of its shells (a solid whose shells are badly
/// oriented, or are several solids, is rebuilt); also makes a solid from a shell.
/// </summary>
/// <remarks>
/// A working object (decision 21). <see cref="Init"/> only records the solid. <see cref="Perform"/> edits the shapes it
/// works on in place, as OCCT's, in a working overlay (Kerf unit 10H4) that takes the writes into frozen shapes (any
/// solid a caller holds), so the caller's solid never changes; at the end the results hold new TShapes for what changed
/// and the caller's own for the rest, and the context answers for the caller's shapes. Inside another operation's
/// overlay (ShapeFix_Shape's) it runs in place in that one. <see cref="Solid"/>, <see cref="Shape"/> and
/// <see cref="SolidFromShell"/> freeze what they return; ported code reads the internal forms.
/// </remarks>
public class SolidFixer : RootFixer
{
    /// <summary>The solid fixed.</summary>
    protected Shape   _solid;
    /// <summary>The shell fixing tool.</summary>
    protected ShellFixer _fixShell;
    /// <summary>The status.</summary>
    protected int            _status;
    /// <summary>The mode of the shell fixes.</summary>
    protected int            _fixShellMode;
    /// <summary>The mode of the orientation fix of the shells.</summary>
    protected int            _fixShellOrientationMode;
    /// <summary>Whether a solid is made from an open shell.</summary>
    protected bool           _createOpenSolidMode;

    /// <summary>Empty constructor.</summary>
    public SolidFixer()
    {
        _solid                   = new Shape();
        _status                  = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.OK);
        _fixShellMode            = -1;
        _fixShellOrientationMode = -1;
        _fixShell                = new ShellFixer();
        _createOpenSolidMode     = false;
    }

    /// <summary>Initializes by a solid.</summary>
    public SolidFixer(in Solid solid)
    {
        _solid                   = new Shape();
        _status                  = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.OK);
        _fixShellMode            = -1;
        _fixShellOrientationMode = -1;
        _fixShell                = new ShellFixer();
        _createOpenSolidMode     = false;
        Init(solid);
    }

    /// <summary>Initializes by a solid.</summary>
    public virtual void Init(in Solid solid)
    {
        _solid = solid;
        // mySolid = TopoDS::Solid(shape.EmptyCopied());
        // BRep_Builder B;
        // for( TopoDS_Iterator iter(solid); iter.More(); iter.Next())
        //  B.Add(mySolid,TopoDS::Shell(iter.Value()));
        _shape = solid;
    }

    //=================================================================================================

    private static void CollectSolids(NCollection_Sequence<Shape> aSeqShells,
                                      NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> anIndexedMapShellHoles,
                                      NCollection_DataMap<Shape, int> mapStatus)
    {
        var mapHoles      = new HashSet<Shape>(ShapeHasher.Instance);
        var mapShellHoles = new NCollection_DataMap<Shape, NCollection_List<Shape>>(ShapeHasher.Instance);
        for (int i1 = 1; i1 <= aSeqShells.Length(); i1++)
        {
            Shell shell1 = ShapeCasts.Shell(aSeqShells.Value(i1));
            var          lshells = new NCollection_List<Shape>();
            mapShellHoles.Bind(shell1, lshells);
        }
        // Finds roots shells and hole shells.
        for (int i = 1; i <= aSeqShells.Length(); i++)
        {
            Shell shell1   = ShapeCasts.Shell(aSeqShells.Value(i));
            var          expEdges = new Explorer(shell1, ShapeType.Edge);
            if (!Tool.IsClosed(shell1) || !expEdges.More())
            {
                continue;
            }
            var solid = new Solid();
            var b     = new BrepBuilder();
            b.MakeSolid(ref solid);
            b.Add(solid, shell1);
            try
            {
                Containment infinstatus = Unknown;
                var          bsc3d       = new SolidClassifier(solid);
                int          st          = 0;
                if (!mapStatus.IsBound(shell1))
                {
                    bsc3d.PerformInfinitePoint(Kerf.Engine.Foundation.Precision.Confusion());
                    infinstatus = bsc3d.State();
                    if (infinstatus != Unknown && infinstatus != On)
                    {
                        st = (infinstatus == In ? 1 : 2);
                    }
                    mapStatus.Bind(shell1, st);
                }
                else
                {
                    st = mapStatus.Find(shell1);
                    if (st != 0)
                    {
                        infinstatus = (mapStatus.Find(shell1) == 1 ? In : Out);
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
                    Shape shell2 = aSeqShells.Value(j);
                    if (!Tool.IsClosed(shell2))
                    {
                        continue;
                    }
                    if (mapHoles.Contains(shell2))
                    {
                        continue;
                    }
                    if (mapShellHoles.IsBound(shell2))
                    {
                        bool                           isAnalysis = false;
                        NCollection_List<Shape> ls         = mapShellHoles.Find(shell2);
                        for (var li = new NCollection_List<Shape>.Iterator(ls); li.More() && !isAnalysis; li.Next())
                        {
                            isAnalysis = li.Value().IsSame(shell1);
                        }
                        if (isAnalysis)
                        {
                            continue;
                        }
                    }
                    Containment pointstatus = Unknown;
                    int          numon       = 0;
                    var          amapVert    = new NCollection_IndexedMap<Shape>(ShapeHasher.Instance);
                    for (var expVert = new Explorer(shell2, ShapeType.Vertex); expVert.More() && amapVert.Extent() < 10;
                         expVert.Next())
                    {
                        amapVert.Add(expVert.Current());
                    }
                    for (int k = 1;
                         k <= amapVert.Extent() && (pointstatus == Unknown || (pointstatus == On && numon < 3));
                         k++)
                    {
                        Vertex v  = ShapeCasts.Vertex(amapVert.FindKey(k));
                        Point3D        pf = Tool.Pnt(v);
                        bsc3d.Perform(pf, Kerf.Engine.Foundation.Precision.Confusion());
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
                        mapShellHoles.ChangeFind(shell1).Append(shell2);
                        if (mapHoles.Contains(shell2))
                        {
                            mapHoles.Remove(shell2);
                        }
                        else
                        {
                            mapHoles.Add(shell2);
                        }
                    }
                }
            }
            catch (FailureException)
            {
                continue;
            }
        }
        foreach (KeyValuePair<Shape, NCollection_List<Shape>> itShellHoles in mapShellHoles)
        {
            if (mapHoles.Contains(itShellHoles.Key))
            {
                continue;
            }
            NCollection_List<Shape> lHoles = itShellHoles.Value;
            if (lHoles.IsEmpty())
            {
                continue;
            }
            for (var lItHoles = new NCollection_List<Shape>.Iterator(lHoles); lItHoles.More(); lItHoles.Next())
            {
                if (mapHoles.Contains(lItHoles.Value()))
                {
                    NCollection_List<Shape> lUnHoles = mapShellHoles.Find(lItHoles.Value());
                    for (var lItUnHoles = new NCollection_List<Shape>.Iterator(lUnHoles); lItUnHoles.More(); lItUnHoles.Next())
                    {
                        mapHoles.Remove(lItUnHoles.Value());
                    }
                }
            }
        }
        foreach (Shape hole in mapHoles)
        {
            mapShellHoles.UnBind(hole);
        }
        for (int i = 1; i <= aSeqShells.Length(); ++i)
        {
            Shape shell1 = aSeqShells.Value(i);
            if (mapShellHoles.IsBound(shell1))
            {
                NCollection_List<Shape> ls = mapShellHoles.Find(shell1);
                anIndexedMapShellHoles.Add(shell1, ls);
            }
        }
    }

    //=================================================================================================

    private static bool CreateSolids(in Shape shape, NCollection_IndexedMap<Shape> aMapSolids)
    {
        var  seqShells = new NCollection_Sequence<Shape>();
        bool isDone     = false;
        for (var expShell = new Explorer(shape, ShapeType.Shell); expShell.More(); expShell.Next())
        {
            seqShells.Append(expShell.Current());
        }
        var mapShellHoles = new NCollection_IndexedDataMap<Shape, NCollection_List<Shape>>(ShapeHasher.Instance);
        var mapStatus     = new NCollection_DataMap<Shape, int>(ShapeHasher.Instance);
        CollectSolids(seqShells, mapShellHoles, mapStatus);
        var shellSolid = new NCollection_IndexedDataMap<Shape, Shape>(ShapeHasher.Instance);
        // Defines correct orientation of shells
        for (int i = 1; i <= mapShellHoles.Extent(); ++i)
        {
            Shell shell    = ShapeCasts.Shell(mapShellHoles.FindKey(i));
            var          expEdges = new Explorer(shell, ShapeType.Edge);
            if (!Tool.IsClosed(shell) || !expEdges.More())
            {
                shellSolid.Add(shell, shell);
                isDone = true;
                continue;
            }
            var          B           = new BrepBuilder();
            Containment infinstatus = Unknown;
            var          aSolid      = new Solid();
            B.MakeSolid(ref aSolid);
            B.Add(aSolid, shell);
            if (mapStatus.IsBound(shell))
            {
                int st = mapStatus.Find(shell);
                if (st != 0)
                {
                    infinstatus = (mapStatus.Find(shell) == 1 ? In : Out);
                }
            }
            else
            {
                try
                {
                    var bsc3d = new SolidClassifier(aSolid);
                    bsc3d.PerformInfinitePoint(Kerf.Engine.Foundation.Precision.Confusion());
                    infinstatus = bsc3d.State();
                }
                catch (FailureException)
                {
                    shellSolid.Add(shell, aSolid);
                    continue;
                }
            }
            if (infinstatus == In)
            {
                isDone = true;
                shell.Reverse();
                var tmpSolid = new Solid();
                B.MakeSolid(ref tmpSolid);
                B.Add(tmpSolid, shell);
                aSolid = tmpSolid;
            }
            NCollection_List<Shape> lHoles = mapShellHoles[i];
            for (var lItHoles = new NCollection_List<Shape>.Iterator(lHoles); lItHoles.More(); lItHoles.Next())
            {
                Shell shellHole = ShapeCasts.Shell(lItHoles.Value());
                if (mapStatus.IsBound(shellHole))
                {
                    infinstatus = (mapStatus.Find(shellHole) == 1 ? In : Out);
                }
                else
                {
                    var solid = new Solid();
                    B.MakeSolid(ref solid);
                    B.Add(solid, shellHole);
                    var bsc3dHol = new SolidClassifier(solid);
                    bsc3dHol.PerformInfinitePoint(Kerf.Engine.Foundation.Precision.Confusion());
                    infinstatus = bsc3dHol.State();
                }
                if (infinstatus == Out)
                {
                    shellHole.Reverse();
                    isDone = true;
                }
                B.Add(aSolid, shellHole);
            }
            shellSolid.Add(shell, aSolid);
        }
        // Creation of compsolid from shells containing shared faces.
        var mapFaceShells = new NCollection_IndexedDataMap<Shape, NCollection_List<Shape>>(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(shape, ShapeType.Face, ShapeType.Shell, mapFaceShells);
        for (int i = 1; i <= mapFaceShells.Extent(); i++)
        {
            NCollection_List<Shape> lshells = mapFaceShells.FindFromIndex(i);
            if (lshells.Extent() < 2)
            {
                continue;
            }
            var compSolid = new CompSolid();
            var b         = new BrepBuilder();
            b.MakeCompSolid(ref compSolid);
            isDone = (shape.ShapeType() != ShapeType.CompSolid || isDone);
            int nbSol = 0;
            for (var lItSh = new NCollection_List<Shape>.Iterator(lshells); lItSh.More(); lItSh.Next())
            {
                if (shellSolid.Contains(lItSh.Value()))
                {
                    Shape aShape  = shellSolid.FindFromKey(lItSh.Value());
                    var          expSol = new Explorer(aShape, global::Kerf.Topology.ShapeType.Solid);
                    for (; expSol.More(); expSol.Next())
                    {
                        b.Add(compSolid, expSol.Current());
                        nbSol++;
                    }
                }
            }
            if (nbSol > 1)
            {
                for (var lItSh1 = new NCollection_List<Shape>.Iterator(lshells); lItSh1.More(); lItSh1.Next())
                {
                    if (shellSolid.Contains(lItSh1.Value()))
                    {
                        shellSolid.ChangeFromKey(lItSh1.Value()) = compSolid;
                    }
                }
            }
        }
        for (int kk = 1; kk <= shellSolid.Extent(); kk++)
        {
            if (!aMapSolids.Contains(shellSolid.FindFromIndex(kk)))
            {
                aMapSolids.Add(shellSolid.FindFromIndex(kk));
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
    public virtual bool Perform(ProgressRange progress = default)
    {
        if (Context() is null)
        {
            SetContext(new ReShapeBuilder());
        }
        using WorkingOverlay? overlay = WorkingOverlay.BeginUnlessWriting(_shape, Context());
        bool status = PerformCore(progress);
        if (overlay is not null)
        {
            _shape = overlay.Finish(_shape, [_solid]);
            _solid = overlay.Final(_solid);
        }
        return status;
    }

    // ShapeFix_Solid::Perform on the working solid
    private bool PerformCore(ProgressRange progress)
    {
        bool status = false;
        if (Context() is null)
        {
            SetContext(new ReShapeBuilder());
        }
        ReShapeBuilder context = Context()!;
        _fixShell.SetContext(context);
        int          NbShells = 0;
        Shape s        = context.InternalApply(_shape);

        // Calculate number of underlying shells
        int nbShells = 0;
        for (var expSh = new Explorer(s, ShapeType.Shell); expSh.More(); expSh.Next())
        {
            nbShells++;
        }

        // Start progress scope (no need to check if progress exists -- it is safe)
        using var ps = new ProgressScope(progress, "Fixing solid stage", 2);

        if (NeedFix(_fixShellMode))
        {
            // Start progress scope (no need to check if progress exists -- it is safe)
            using var psFixShell = new ProgressScope(ps.Next(), "Fixing shell", nbShells);

            // Fix shell by shell using ShapeFix_Shell tool
            for (var expSh = new Explorer(s, ShapeType.Shell); expSh.More() && psFixShell.More(); expSh.Next())
            {
                Shape sh = expSh.Current();

                _fixShell.Init(ShapeCasts.Shell(sh));
                if (_fixShell.Perform(psFixShell.Next()))
                {
                    status = true;
                    _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE1);
                }
                NbShells += _fixShell.ShellCount;
            }

            // Halt algorithm in case of user's abort
            if (!psFixShell.More())
            {
                return false;
            }
        }
        else
        {
            NbShells = nbShells;
        }

        if (!NeedFix(_fixShellOrientationMode))
        {
            _shape = context.InternalApply(_shape);
            return status;
        }

        if (NbShells == 1)
        {
            Shape tmpShape = context.InternalApply(_shape);
            var          exp     = new Explorer(tmpShape, ShapeType.Shell);
            bool         isClosed = false;
            if (exp.More())
            {
                Shell    shtmp  = ShapeCasts.Shell(exp.Current());
                var             sfb     = new FreeBounds(shtmp);
                Compound c1     = sfb.InternalGetClosedWires();
                Compound c2     = sfb.InternalGetOpenWires();
                int             numedge = 0;
                var             exp1   = new Explorer(c1, ShapeType.Edge);
                for (; exp1.More(); exp1.Next())
                {
                    numedge++;
                }
                for (exp1.Init(c2, ShapeType.Edge); exp1.More(); exp1.Next())
                {
                    numedge++;
                }
                isClosed = (numedge == 0);
                shtmp.Closed(isClosed);
            }

            if (isClosed || _createOpenSolidMode)
            {
                var          itersh = new Iterator(tmpShape);
                Shell shell = new();
                if (itersh.More() && itersh.Value().ShapeType() == ShapeType.Shell)
                {
                    shell = ShapeCasts.Shell(itersh.Value());
                }
                if (!shell.IsNull)
                {
                    Solid sol = InternalSolidFromShell(shell);
                    if (global::Kerf.Engine.Healing.Extend.ShapeExtend.DecodeStatus(_status, ExtendStatus.DONE2))
                    {
                        SendWarning(new Msg("FixAdvSolid.FixOrientation.MSG20")); // Orientation of shell was corrected.
                        context.Replace(tmpShape, sol);
                        tmpShape = sol;
                    }
                }
                _solid = ShapeCasts.Solid(tmpShape);
            }
            else
            {
                status = true;
                _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE3);
                var it = new Iterator(tmpShape, false);
                context.Replace(tmpShape, it.Value());
                SendFail(new Msg("FixAdvSolid.FixShell.MSG10")); // Solid can not be created from open shell.
            }
        }
        else
        {
            Shape resShape = context.InternalApply(_shape);
            var          mapSolids = new NCollection_IndexedMap<Shape>(ShapeHasher.Instance);
            if (CreateSolids(resShape, mapSolids))
            {
                SendWarning(new Msg("FixAdvSolid.FixOrientation.MSG20")); // Orientation of shell was corrected..
                if (mapSolids.Extent() == 1)
                {
                    Shape resSol = mapSolids.FindKey(1);
                    if (resShape.ShapeType() == ShapeType.Shell && _createOpenSolidMode)
                    {
                        var solid = new Solid();
                        var B     = new BrepBuilder();
                        B.MakeSolid(ref solid);
                        B.Add(solid, resSol);
                        _solid = solid;
                    }
                    else
                    {
                        _solid = resSol;
                        if (resSol.ShapeType() == ShapeType.Shell)
                        {
                            SendFail(new Msg("FixAdvSolid.FixShell.MSG10")); // Solid can not be created from open shell.
                        }
                    }
                    context.Replace(resShape, _solid);
                }
                else if (mapSolids.Extent() > 1)
                {
                    SendWarning(new Msg("FixAdvSolid.FixOrientation.MSG30")); // Bad connected solid a few solids were created.
                    var aB    = new BrepBuilder();
                    var comp = new Compound();
                    aB.MakeCompound(ref comp);
                    using var psCreatingSolid = new ProgressScope(ps.Next(), "Creating solid", mapSolids.Extent());
                    for (int i = 1; (i <= mapSolids.Extent()) && (psCreatingSolid.More()); i++, psCreatingSolid.Next())
                    {
                        Shape resSh = mapSolids.FindKey(i);
                        if (resShape.ShapeType() == ShapeType.Shell && _createOpenSolidMode)
                        {
                            resSh.Closed(false);
                            var solid = new Solid();
                            var b     = new BrepBuilder();
                            b.MakeSolid(ref solid);
                            b.Add(solid, resSh);
                            resSh = solid;
                        }
                        else if (resShape.ShapeType() == ShapeType.Shell)
                        {
                            SendFail(new Msg("FixAdvSolid.FixShell.MSG10")); // Solid can not be created from open shell.
                        }
                        aB.Add(comp, resSh);
                    }
                    if (!psCreatingSolid.More())
                    {
                        return false; // aborted execution
                    }
                    context.Replace(resShape, comp);
                }
            }
        }
        _shape = context.InternalApply(_shape);
        return status;
    }

    //=================================================================================================

    /// <summary>Returns the resulting shape, frozen.</summary>
    public Shape Shape() => Published(_shape);

    /// <summary>Returns the resulting shape, unfrozen, for ported callers working on shapes they own.</summary>
    internal Shape InternalShape() => _shape;

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
        if (!sh.Free() && !WorkingOverlay.IsReadOnly(sh.TShape()!)) // OCCT.NET: frozen: a frozen shell keeps its Free flag outside an overlay
        {
            sh.Free(true);
        }
        var solid = new Solid();
        var b     = new BrepBuilder();
        b.MakeSolid(ref solid);
        b.Add(solid, sh);
        //   Pas encore fini : il faut une bonne orientation
        try
        {
            var    bsc3d = new SolidClassifier(solid);
            double t     = Kerf.Engine.Foundation.Precision.Confusion(); // tolerance moyenne
            bsc3d.PerformInfinitePoint(t);

            if (bsc3d.State() == In)
            {
                //         Ensuite, inverser C-A-D REPRENDRE LES SHELLS
                //         (l inversion du solide n est pas bien prise en compte)
                sh = shell;
                if (!sh.Free() && !WorkingOverlay.IsReadOnly(sh.TShape()!)) // OCCT.NET: frozen
                {
                    sh.Free(true);
                }
                var soli2 = new Solid();
                b.MakeSolid(ref soli2); // on recommence
                sh.Reverse();
                b.Add(soli2, sh);
                solid = soli2;
                _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE2);
            }
        }
        catch (FailureException)
        {
            return solid;
        }
        return solid;
    }

    /// <summary>Returns the status (the bits of the fixes).</summary>
    public bool Status(ExtendStatus status) => global::Kerf.Engine.Healing.Extend.ShapeExtend.DecodeStatus(_status, status);

    /// <summary>Returns the fixed solid, frozen.</summary>
    public Shape Solid() => Published(_solid);

    /// <summary>Returns the fixed solid, unfrozen, for ported callers working on shapes they own.</summary>
    internal Shape InternalSolid() => _solid;

    /// <summary>Returns the shell fixing tool.</summary>
    public ShellFixer FixShellTool() => _fixShell;

    /// <summary>Sets the message registrator, for this tool and the shell tool.</summary>
    public override void SetMsgRegistrator(BasicMsgRegistrator? msgreg)
    {
        base.SetMsgRegistrator(msgreg);
        _fixShell.SetMsgRegistrator(msgreg);
    }

    /// <summary>Sets the basic precision, for this tool and the shell tool.</summary>
    public override void SetPrecision(double preci)
    {
        base.SetPrecision(preci);
        _fixShell.SetPrecision(preci);
    }

    /// <summary>Sets the minimal tolerance, for this tool and the shell tool.</summary>
    public override void SetMinTolerance(double mintol)
    {
        base.SetMinTolerance(mintol);
        _fixShell.SetMinTolerance(mintol);
    }

    /// <summary>Sets the maximal tolerance, for this tool and the shell tool.</summary>
    public override void SetMaxTolerance(double maxtol)
    {
        base.SetMaxTolerance(maxtol);
        _fixShell.SetMaxTolerance(maxtol);
    }

    /// <summary>The mode of the shell fixes (-1 default, 0 off, 1 on).</summary>
    public ref int FixShellMode() => ref _fixShellMode;

    /// <summary>The mode of the orientation fix of the shells (-1 default, 0 off, 1 on).</summary>
    public ref int FixShellOrientationMode() => ref _fixShellOrientationMode;

    /// <summary>Whether a solid is made from an open shell (false by default).</summary>
    public ref bool CreateOpenSolidMode() => ref _createOpenSolidMode;

    private static Shape Published(in Shape shape)
    {
        shape.TShape()?.FreezeGraph();
        return shape;
    }
}
