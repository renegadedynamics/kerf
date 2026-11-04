// Port of: src/DataExchange/TKDESTEP/TopoDSToStep/TopoDSToStep_Builder.hxx
// Port of: src/DataExchange/TKDESTEP/TopoDSToStep/TopoDSToStep_Builder.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Exchange.Framework.Transfer;
using Kerf.Engine.Exchange.Step.Data;
using Kerf.Engine.Exchange.Step.Shapes;
using Kerf.Engine.Exchange.Step.Visual;
using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.ShapeType;
using static Kerf.Engine.Exchange.Step.Export.ExportBuilderError;

namespace Kerf.Engine.Exchange.Step.Export;

/// <summary>Builds the STEP topology of a shell (an open or closed shell) or of a face, and optionally its tessellation.</summary>
internal sealed class ExportBuilder : ExportRoot
{
    private StepTopologicalRepresentationItem _result;
    private TessellatedItem _tessellatedResult;
    private ExportBuilderError _error;

    public ExportBuilder()
    {
        _error = ExportBuilderError.BuilderOther;
        _done = false;
    }

    public ExportBuilder(in Shape s, ExportTool t, FinderProcess fp, int tessellatedGeomParam,
                                in Factors localFactors, ProgressRange progress = default)
    {
        _done = false;
        Init(s, t, fp, tessellatedGeomParam, localFactors, progress);
    }

    public ExportBuilder(in Shape s, ExportTool t, FinderProcess fp, int tessellatedGeomParam)
        : this(s, t, fp, tessellatedGeomParam, new Factors())
    {
    }

    public void Init(in Shape aShape, ExportTool myTool, FinderProcess fp, int tessellatedGeomParam,
                     in Factors localFactors, ProgressRange progress = default)
    {
        if (myTool.IsBound(aShape))
        {
            _error = ExportBuilderError.BuilderDone;
            _done = true;
            _result = myTool.Find(aShape);
            return;
        }

        switch (aShape.Type)
        {
            case ShapeType.Shell:
            {
                Shell myShell = ShapeCasts.Shell(aShape);
                myTool.SetCurrentShell(myShell);

                StepFaceSurface fs;
                StepTopologicalRepresentationItem fpms;
                var mySeq = new NCollection_Sequence<Transient>();

                //  CKY  9-DEC-1997 (PRO9824 et consorts)
                //   Pour passer les orientations : ELLES SONT DONNEES EN RELATIF
                //   Donc, pour SHELL, on doit l ecrire en direct en STEP (pas le choix)
                //   -> il faut repercuter en dessous, donc explorer le Shell TEL QUEL
                //   Pour FACE WIRE, d une part on ECRIT SON ORIENTATION relative au contenant
                //     (puisqu on peut), d autre part on EXPLORE EN FORWARD : ainsi les
                //     orientations des sous-shapes sont relatives a leur contenant immediat
                //     et la recombinaison en lecture est sans malice
                //  Il reste ici et la du code relatif a "en Faceted on combine differemment"
                //  -> reste encore du menage a faire

                var exp = new Explorer();

                var mkFace = new MakeStepFace();

                using var ps = new ProgressScope(progress, null, (tessellatedGeomParam != 0) ? 2 : 1);

                int nbshapes = 0;
                for (exp.Init(myShell, ShapeType.Face); exp.More(); exp.Next())
                {
                    nbshapes++;
                }

                using var ps1 = new ProgressScope(ps.Next(), null, nbshapes);
                for (exp.Init(myShell, ShapeType.Face); exp.More() && ps1.More(); exp.Next(), ps1.Next())
                {
                    Face face = ShapeCasts.Face(exp.Current());

                    mkFace.Init(face, myTool, fp, localFactors);

                    if (mkFace.IsDone)
                    {
                        fs = mkFace.Value() as StepFaceSurface;
                        fpms = fs;
                        mySeq.Append(fpms);
                    }
                    else
                    {
                        // MakeFace Error Handling : warning only
                    }
                }

                if (!ps1.More())
                {
                    return;
                }

                int nbFaces = mySeq.Length();
                if (nbFaces >= 1)
                {
                    var set = new NCollection_HArray1<StepFace>(1, nbFaces);
                    for (int i = 1; i <= nbFaces; i++)
                    {
                        set.SetValue(i, mySeq.Value(i) as StepFace);
                    }

                    StepConnectedFaceSet cfSpms;
                    if (myShell.IsClosed)
                    {
                        cfSpms = new StepClosedShell();
                    }
                    else
                    {
                        cfSpms = new StepOpenShell();
                    }

                    var name = new TCollection_HAsciiString("");
                    cfSpms.Init(name, set);

                    // --------------------------------------------------------------
                    // To add later : if not facetted context & shell is reversed
                    //                then shall create an oriented_shell with
                    //                orientation flag to false.
                    // --------------------------------------------------------------

                    myTool.Bind(aShape, cfSpms);
                    _result = cfSpms;
                    _done = true;
                }
                else
                {
                    // Builder Error handling;
                    _error = ExportBuilderError.NoFaceMapped;
                    _done = false;
                }

                if (tessellatedGeomParam == 1 || (tessellatedGeomParam == 2 && _result is null))
                {
                    var mkTessShell = new MakeTessellatedItem(myShell, myTool, fp, localFactors, ps.Next());
                    if (mkTessShell.IsDone)
                    {
                        _tessellatedResult = mkTessShell.Value();
                        _error = ExportBuilderError.BuilderDone;
                        _done = true;
                    }
                }

                break;
            }

            case ShapeType.Face:
            {
                Face face = ShapeCasts.Face(aShape);

                StepFaceSurface fs;
                StepTopologicalRepresentationItem fpms;

                var mkFace = new MakeStepFace(face, myTool, fp, localFactors);

                var mkTessFace = new MakeTessellatedItem();
                if (tessellatedGeomParam == 1 || (tessellatedGeomParam == 2 && !mkFace.IsDone))
                {
                    using var ps = new ProgressScope(progress, null, 1);
                    // fourth parameter is true in order to create a tessellated_surface_set entity
                    // or put false to create a triangulated_face instead
                    mkTessFace.Init(face, myTool, fp, true, localFactors, ps.Next());
                }

                if (mkFace.IsDone || mkTessFace.IsDone)
                {
                    if (mkFace.IsDone)
                    {
                        fs = mkFace.Value() as StepFaceSurface;
                        fpms = fs;
                        _result = fpms;
                    }

                    if (mkTessFace.IsDone)
                    {
                        _tessellatedResult = mkTessFace.Value();
                    }

                    _error = ExportBuilderError.BuilderDone;
                    _done = true;
                }
                else
                {
                    // MakeFace Error Handling : Face not Mapped
                    _error = ExportBuilderError.BuilderOther;
                    _done = false;
                }

                break;
            }

            default:
                break;
        }
    }

    public void Init(in Shape aShape, ExportTool myTool, FinderProcess fp, int tessellatedGeomParam)
        => Init(aShape, myTool, fp, tessellatedGeomParam, new Factors());

    public ExportBuilderError Error() => _error;

    /// <summary>The topological representation item made.</summary>
    /// <exception cref="NotDoneException">The build wasn't done.</exception>
    public StepTopologicalRepresentationItem Value()
    {
        StdFail_NotDone_Raise_if(!_done, "TopoDSToStep_Builder::Value() - no result");
        return _result;
    }

    /// <summary>The tessellated item made, the optional result.</summary>
    /// <exception cref="NotDoneException">The build wasn't done.</exception>
    public TessellatedItem TessellatedValue()
    {
        StdFail_NotDone_Raise_if(!_done, "TopoDSToStep_Builder::TessellatedValue() - no result");
        return _tessellatedResult;
    }
}
