// Port of: src/DataExchange/TKDESTEP/TopoDSToStep/TopoDSToStep_Builder.hxx
// Port of: src/DataExchange/TKDESTEP/TopoDSToStep/TopoDSToStep_Builder.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Topology;
using static Kerf.Topology.ShapeType;
using static Occt.TopoDSToStep_BuilderError;

namespace Occt;

/// <summary>Builds the STEP topology of a shell (an open or closed shell) or of a face, and optionally its tessellation.</summary>
public sealed class TopoDSToStep_Builder : TopoDSToStep_Root
{
    private StepShape_TopologicalRepresentationItem myResult;
    private StepVisual_TessellatedItem myTessellatedResult;
    private TopoDSToStep_BuilderError myError;

    public TopoDSToStep_Builder()
    {
        myError = TopoDSToStep_BuilderError.TopoDSToStep_BuilderOther;
        done = false;
    }

    public TopoDSToStep_Builder(in Shape S, TopoDSToStep_Tool T, Transfer_FinderProcess FP, int theTessellatedGeomParam,
                                in StepData_Factors theLocalFactors, ProgressRange theProgress = default)
    {
        done = false;
        Init(S, T, FP, theTessellatedGeomParam, theLocalFactors, theProgress);
    }

    public TopoDSToStep_Builder(in Shape S, TopoDSToStep_Tool T, Transfer_FinderProcess FP, int theTessellatedGeomParam)
        : this(S, T, FP, theTessellatedGeomParam, new StepData_Factors())
    {
    }

    public void Init(in Shape aShape, TopoDSToStep_Tool myTool, Transfer_FinderProcess FP, int theTessellatedGeomParam,
                     in StepData_Factors theLocalFactors, ProgressRange theProgress = default)
    {
        if (myTool.IsBound(aShape))
        {
            myError = TopoDSToStep_BuilderError.TopoDSToStep_BuilderDone;
            done = true;
            myResult = myTool.Find(aShape);
            return;
        }

        switch (aShape.ShapeType())
        {
            case ShapeType.Shell:
            {
                Shell myShell = ShapeCasts.Shell(aShape);
                myTool.SetCurrentShell(myShell);

                StepShape_FaceSurface FS;
                StepShape_TopologicalRepresentationItem Fpms;
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

                var anExp = new Explorer();

                var MkFace = new TopoDSToStep_MakeStepFace();

                using var aPS = new ProgressScope(theProgress, null, (theTessellatedGeomParam != 0) ? 2 : 1);

                int nbshapes = 0;
                for (anExp.Init(myShell, ShapeType.Face); anExp.More(); anExp.Next())
                {
                    nbshapes++;
                }

                using var aPS1 = new ProgressScope(aPS.Next(), null, nbshapes);
                for (anExp.Init(myShell, ShapeType.Face); anExp.More() && aPS1.More(); anExp.Next(), aPS1.Next())
                {
                    Face Face = ShapeCasts.Face(anExp.Current());

                    MkFace.Init(Face, myTool, FP, theLocalFactors);

                    if (MkFace.IsDone())
                    {
                        FS = MkFace.Value() as StepShape_FaceSurface;
                        Fpms = FS;
                        mySeq.Append(Fpms);
                    }
                    else
                    {
                        // MakeFace Error Handling : warning only
                    }
                }

                if (!aPS1.More())
                {
                    return;
                }

                int nbFaces = mySeq.Length();
                if (nbFaces >= 1)
                {
                    var aSet = new NCollection_HArray1<StepShape_Face>(1, nbFaces);
                    for (int i = 1; i <= nbFaces; i++)
                    {
                        aSet.SetValue(i, mySeq.Value(i) as StepShape_Face);
                    }

                    StepShape_ConnectedFaceSet CFSpms;
                    if (myShell.Closed())
                    {
                        CFSpms = new StepShape_ClosedShell();
                    }
                    else
                    {
                        CFSpms = new StepShape_OpenShell();
                    }

                    var aName = new TCollection_HAsciiString("");
                    CFSpms.Init(aName, aSet);

                    // --------------------------------------------------------------
                    // To add later : if not facetted context & shell is reversed
                    //                then shall create an oriented_shell with
                    //                orientation flag to false.
                    // --------------------------------------------------------------

                    myTool.Bind(aShape, CFSpms);
                    myResult = CFSpms;
                    done = true;
                }
                else
                {
                    // Builder Error handling;
                    myError = TopoDSToStep_BuilderError.TopoDSToStep_NoFaceMapped;
                    done = false;
                }

                if (theTessellatedGeomParam == 1 || (theTessellatedGeomParam == 2 && myResult is null))
                {
                    var MkTessShell = new TopoDSToStep_MakeTessellatedItem(myShell, myTool, FP, theLocalFactors, aPS.Next());
                    if (MkTessShell.IsDone())
                    {
                        myTessellatedResult = MkTessShell.Value();
                        myError = TopoDSToStep_BuilderError.TopoDSToStep_BuilderDone;
                        done = true;
                    }
                }

                break;
            }

            case ShapeType.Face:
            {
                Face Face = ShapeCasts.Face(aShape);

                StepShape_FaceSurface FS;
                StepShape_TopologicalRepresentationItem Fpms;

                var MkFace = new TopoDSToStep_MakeStepFace(Face, myTool, FP, theLocalFactors);

                var MkTessFace = new TopoDSToStep_MakeTessellatedItem();
                if (theTessellatedGeomParam == 1 || (theTessellatedGeomParam == 2 && !MkFace.IsDone()))
                {
                    using var aPS = new ProgressScope(theProgress, null, 1);
                    // fourth parameter is true in order to create a tessellated_surface_set entity
                    // or put false to create a triangulated_face instead
                    MkTessFace.Init(Face, myTool, FP, true, theLocalFactors, aPS.Next());
                }

                if (MkFace.IsDone() || MkTessFace.IsDone())
                {
                    if (MkFace.IsDone())
                    {
                        FS = MkFace.Value() as StepShape_FaceSurface;
                        Fpms = FS;
                        myResult = Fpms;
                    }

                    if (MkTessFace.IsDone())
                    {
                        myTessellatedResult = MkTessFace.Value();
                    }

                    myError = TopoDSToStep_BuilderError.TopoDSToStep_BuilderDone;
                    done = true;
                }
                else
                {
                    // MakeFace Error Handling : Face not Mapped
                    myError = TopoDSToStep_BuilderError.TopoDSToStep_BuilderOther;
                    done = false;
                }

                break;
            }

            default:
                break;
        }
    }

    public void Init(in Shape aShape, TopoDSToStep_Tool myTool, Transfer_FinderProcess FP, int theTessellatedGeomParam)
        => Init(aShape, myTool, FP, theTessellatedGeomParam, new StepData_Factors());

    public TopoDSToStep_BuilderError Error() => myError;

    /// <summary>The topological representation item made.</summary>
    /// <exception cref="NotDoneException">The build wasn't done.</exception>
    public StepShape_TopologicalRepresentationItem Value()
    {
        StdFail_NotDone_Raise_if(!done, "TopoDSToStep_Builder::Value() - no result");
        return myResult;
    }

    /// <summary>The tessellated item made, the optional result.</summary>
    /// <exception cref="NotDoneException">The build wasn't done.</exception>
    public StepVisual_TessellatedItem TessellatedValue()
    {
        StdFail_NotDone_Raise_if(!done, "TopoDSToStep_Builder::TessellatedValue() - no result");
        return myTessellatedResult;
    }
}
