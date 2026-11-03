// Port of: src/DataExchange/TKDESTEP/StepToTopoDS/StepToTopoDS_TranslateShell.hxx
// Port of: src/DataExchange/TKDESTEP/StepToTopoDS/StepToTopoDS_TranslateShell.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Exchange.Framework.Transfer;
using Kerf.Engine.Exchange.Step.Data;
using Kerf.Engine.Exchange.Step.Shapes;
using Kerf.Engine.Exchange.Step.Visual;
using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Exchange.Step.Import.TranslateShellError;

namespace Kerf.Engine.Exchange.Step.Import;

/// <summary>Translates a connected_face_set (an open or closed shell) or a tessellated_shell into a TopoDS_Shell.</summary>
/// <remarks><see cref="Value"/> freezes the result; the translators use <c>InternalValue</c> (decision 1247).</remarks>
public sealed class TranslateShell : ImportRoot
{
    private TranslateShellError _error;
    private Shape _result = new Shape();

    public TranslateShell()
    {
        _error = TranslateShellError.TranslateShellOther;
        _done = false;
    }

    /// <summary>Translates a connected_face_set.</summary>
    public void Init(StepConnectedFaceSet cfs, ImportTool aTool, NMTool nmTool, in Factors localFactors,
                     ProgressRange progress = default)
    {
        // bug15697
        if (cfs is null)
        {
            return;
        }

        if (!aTool.IsBound(cfs))
        {
            var b = new BrepBuilder();
            TransientProcess tp = aTool.TransientProcess();

            int nbFc = cfs.NbCfsFaces();
            Shell sh = new Shell();
            b.MakeShell(ref sh);
            Face f;
            Shape s;
            StepFace stepFace;

            var myTranFace = new TranslateFace();
            myTranFace.SetPrecision(Precision()); // gka
            myTranFace.SetMaxTol(MaxTol());

            using var ps = new ProgressScope(progress, "Face", nbFc);
            for (int i = 1; i <= nbFc && ps.More(); i++, ps.Next())
            {
                stepFace = cfs.CfsFacesValue(i);
                StepFaceSurface theFS = stepFace as StepFaceSurface;
                if (theFS is not null)
                {
                    myTranFace.Init(theFS, aTool, nmTool, localFactors);
                    if (myTranFace.IsDone)
                    {
                        s = myTranFace.InternalValue();
                        f = ShapeCasts.Face(s);
                        b.Add(sh, f);
                    }
                    else
                    { // Warning only + add FaceSurface file Identifier
                        tp.AddWarning(theFS, " a Face from Shell not mapped to TopoDS");
                    }
                }
                else
                { // Warning : add identifier
                    tp.AddWarning(stepFace, " Face is not of FaceSurface Type; not mapped to TopoDS");
                }
            }

            sh.Closed(Tool.IsClosed(sh));
            _result = sh;
            aTool.Bind(cfs, _result);
            _error = TranslateShellError.TranslateShellDone;
            _done = true;
        }
        else
        {
            _result = ShapeCasts.Shell(aTool.Find(cfs));
            _error = TranslateShellError.TranslateShellDone;
            _done = true;
        }
    }

    public void Init(StepConnectedFaceSet cfs, ImportTool aTool, NMTool nmTool)
        => Init(cfs, aTool, nmTool, new Factors());

    /// <summary>Translates a tessellated_shell.</summary>
    /// <remarks>
    /// With a topological link to a shell already translated, the faces' triangulations are added to that shell's
    /// faces in place (as OCCT does: they must be the reader's working shapes, unfrozen).
    /// </remarks>
    public void Init(TessellatedShell tSh, ImportTool tool, NMTool nmTool,
                     bool readTessellatedWhenNoBRepOnly, ref bool hasGeom, in Factors localFactors,
                     ProgressRange progress = default)
    {
        if (tSh is null)
        {
            return;
        }

        var b = new BrepBuilder();
        Shell sh = new Shell();

        int nb = tSh.NbItems();
        using var ps = new ProgressScope(progress, "Face", tSh.HasTopologicalLink ? nb + 1 : nb);

        TransientProcess tp = tool.TransientProcess();

        if (tSh.HasTopologicalLink)
        {
            ShapeBinder binder = tp.Find(tSh.TopologicalLink()) as ShapeBinder;
            if (binder is not null)
            {
                sh = binder.Shell();
                hasGeom = true;
            }
        }

        bool newShell = false;
        if (sh.IsNull)
        {
            b.MakeShell(ref sh);
            newShell = true;
            hasGeom = false;
        }

        var tranTF = new TranslateFace();
        tranTF.SetPrecision(Precision());
        tranTF.SetMaxTol(MaxTol());

        for (int i = 1; i <= nb && ps.More(); i++, ps.Next())
        {
            TessellatedStructuredItem item = tSh.ItemsValue(i);
            if (item is TessellatedFace)
            {
                TessellatedFace tFace = item as TessellatedFace;
                bool hasFaceGeom = false;
                tranTF.Init(tFace, tool, nmTool, readTessellatedWhenNoBRepOnly, ref hasFaceGeom, localFactors);
                if (tranTF.IsDone)
                {
                    if (newShell)
                    {
                        b.Add(sh, ShapeCasts.Face(tranTF.InternalValue()));
                    }

                    hasGeom &= hasFaceGeom;
                }
                else
                {
                    tp.AddWarning(item, " Triangulated face if not mapped to TopoDS");
                }
            }
            else
            {
                tp.AddWarning(item, " Face is not of TriangulatedFace Type; not mapped to TopoDS");
            }
        }

        sh.Closed(Tool.IsClosed(sh));

        _result = sh;
        _error = TranslateShellError.TranslateShellDone;
        _done = true;
    }

    public void Init(TessellatedShell tSh, ImportTool tool, NMTool nmTool,
                     bool readTessellatedWhenNoBRepOnly, ref bool hasGeom)
        => Init(tSh, tool, nmTool, readTessellatedWhenNoBRepOnly, ref hasGeom, new Factors());

    /// <summary>The shell, frozen.</summary>
    /// <exception cref="NotDoneException">The translation wasn't done.</exception>
    public Shape Value()
    {
        Shape shape = InternalValue();
        shape.TShape()?.FreezeGraph();
        return shape;
    }

    /// <summary>As <see cref="Value"/>, without freezing: the reader's working shape.</summary>
    internal Shape InternalValue()
    {
        StdFail_NotDone_Raise_if(!_done, "StepToTopoDS_TranslateShell::Value() - no result");
        return _result;
    }

    public TranslateShellError Error() => _error;
}
