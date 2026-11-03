// Port of: src/DataExchange/TKDESTEP/TopoDSToStep/TopoDSToStep_MakeStepFace.hxx
// Port of: src/DataExchange/TKDESTEP/TopoDSToStep/TopoDSToStep_MakeStepFace.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Conversion;
using Kerf.Engine.Healing;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Geometry.Continuity;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;
using static Occt.TopoDSToStep_MakeFaceError;

namespace Occt;

/// <summary>
/// Maps a TopoDS_Face (with its relative orientation in the shell) to a STEP advanced_face: its surface, its bounds and
/// the pcurves of its edges.
/// </summary>
public sealed class TopoDSToStep_MakeStepFace : TopoDSToStep_Root
{
    private StepShape_TopologicalRepresentationItem myResult;
    private TopoDSToStep_MakeFaceError myError;

    public TopoDSToStep_MakeStepFace()
    {
        myError = TopoDSToStep_MakeFaceError.TopoDSToStep_FaceOther;
        done = false;
    }

    public TopoDSToStep_MakeStepFace(in Face F, TopoDSToStep_Tool T, Transfer_FinderProcess FP, in StepData_Factors theLocalFactors)
    {
        done = false;
        Init(F, T, FP, theLocalFactors);
    }

    public TopoDSToStep_MakeStepFace(in Face F, TopoDSToStep_Tool T, Transfer_FinderProcess FP)
        : this(F, T, FP, new StepData_Factors())
    {
    }

    /// <remarks>Needs <see cref="ShapeAlgo.Init"/> (degenerate tori are rewritten through its container).</remarks>
    public void Init(in Face aFace, TopoDSToStep_Tool aTool, Transfer_FinderProcess FP, in StepData_Factors theLocalFactors)
    {
        // --------------------------------------------------------------
        // the face is given with its relative orientation (in the Shell)
        // --------------------------------------------------------------

        // szv#4:S4163:12Mar99 SGI warns
        Shape sh = aFace.Oriented(Forward);
        Face ForwardFace = ShapeCasts.Face(sh);
        aTool.SetCurrentFace(ForwardFace);
        var errShape = new TransferBRep_ShapeMapper(aFace); // on ne sait jamais

        // [BEGIN] Processing non-manifold topology (another approach) (ssv; 10.11.2010)
        if ((FP.Model() as StepData_StepModel).InternalParameters.WriteNonmanifold)
        {
            StepShape_AdvancedFace anAF = null;
            TransferBRep_ShapeMapper aSTEPMapper = TransferBRep.ShapeMapper(FP, aFace);
            if (FP.FindTypedTransient(aSTEPMapper, typeof(StepShape_AdvancedFace), ref anAF))
            {
                // Non-manifold topology detected
                var aLinkingAF = new StepShape_AdvancedFace();
                aLinkingAF.Init(anAF.Name(), anAF.Bounds(), anAF.FaceGeometry(), !anAF.SameSense());

                myError = TopoDSToStep_MakeFaceError.TopoDSToStep_FaceDone;
                myResult = aLinkingAF;
                done = true;
                return;
            }
        }
        // [END] Processing non-manifold topology (ssv; 10.11.2010)

        if (aTool.IsBound(aFace))
        {
            myError = TopoDSToStep_MakeFaceError.TopoDSToStep_FaceDone;
            done = true;
            myResult = aTool.Find(aFace);
            return;
        }

        if (aFace.Orientation() == Internal || aFace.Orientation() == External)
        {
            FP.AddWarning(errShape, " Face from Non Manifold Topology");
            myError = TopoDSToStep_MakeFaceError.TopoDSToStep_NonManifoldFace;
            done = false;
            return;
        }

        // ------------------
        // Get the Outer Wire
        // ------------------

        Wire theOuterWire = BrepTools.OuterWire(ForwardFace);

        if (theOuterWire.IsNull)
        {
            FP.AddWarning(errShape, " Face without wire not mapped");
            myError = TopoDSToStep_MakeFaceError.TopoDSToStep_InfiniteFace;
            done = false;
            return;
        }

        try
        {
            // -----------------
            // Translate Surface
            // -----------------

            Surface Su = Tool.Surface(ForwardFace);
            if (Su is null)
            {
                FP.AddWarning(errShape, " Face without geometry not mapped");
                myError = TopoDSToStep_MakeFaceError.TopoDSToStep_FaceOther;
                done = false;
                return;
            }

            //  CKY  23 SEP 1996 : une FACE de Step n a pas droit a RECTANGULAR_TRIMMED...
            //  Il faut donc d abord "demonter" la RectangularTrimmedSurface pour
            //  passer la Surface de base
            TrimmedSurface aRTS = Su as TrimmedSurface;
            if (aRTS is not null)
            {
                Su = aRTS.BasisSurface();
            }

            // Surfaces with indirect Axes are already reversed
            aTool.SetSurfaceReversed(false);

            var MkSurface = new GeomToStep_MakeSurface(Su, theLocalFactors);
            StepGeom_Surface Spms = MkSurface.Value();

            //%pdn 30 Nov 98: TestRally 9 issue on r1001_ec.stp:
            // toruses with major_radius < minor are re-coded as degenerate
            // rln 19.01.99: uncomment %30 pdn for integration into K4L
            {
                // If the surface is Offset it is necessary to check the base surface
                bool aSurfaceIsOffset = false;
                OffsetSurface anOffsetSu = null;
                if (Su is OffsetSurface)
                {
                    aSurfaceIsOffset = true;
                    anOffsetSu = Su as OffsetSurface;
                }

                if ((Spms is StepGeom_ToroidalSurface) || ((aSurfaceIsOffset) && anOffsetSu.BasisSurface() is ToroidalSurface))
                {
                    ToroidalSurface TS;
                    if (aSurfaceIsOffset)
                    {
                        TS = anOffsetSu.BasisSurface() as ToroidalSurface;
                    }
                    else
                    {
                        TS = Su as ToroidalSurface;
                    }

                    double R = TS.MajorRadius();
                    double r = TS.MinorRadius();
                    if (R < r) // if torus is degenerate or base surface is degenerate, make revolution instead
                    {
                        CoordinateSystem Ax3 = TS.Position();
                        Point3D pos = Ax3.Location();
                        Direction3D dir = Ax3.Direction();
                        Direction3D X = Ax3.XDirection();

                        // create basis curve
                        global::Kerf.Engine.Healing.ShapeAlgo.AlgoContainer().GetFaceUVBounds(aFace, out double UF, out double UL, out double VF, out double VL);
                        Frame3D Ax2 = new Frame3D(new Point3D(pos.XYZ() + X.XYZ() * TS.MajorRadius()), X ^ dir, X);
                        Curve3D BasisCurve = new CircleCurve3D(Ax2, TS.MinorRadius());

                        // convert basis curve to bspline in order to avoid self-intersecting
                        // surface of revolution (necessary e.g. for CATIA)
                        if (VL - VF - 2 * M_PI < -global::Kerf.Engine.Foundation.Precision.PConfusion())
                        {
                            BasisCurve = global::Kerf.Engine.Healing.ShapeAlgo.AlgoContainer().ConvertCurveToBSpline(BasisCurve, VF, VL, global::Kerf.Engine.Foundation.Precision.Approximation(),
                                                                                        C1, 100, 9);
                        }

                        // create surface of revolution
                        Axis3D Axis = Ax3.Axis();
                        if (!Ax3.Direct())
                        {
                            Axis.Reverse();
                        }

                        var Rev = new RevolutionSurface(BasisCurve, Axis);

                        // and translate it
                        if (aSurfaceIsOffset)
                        {
                            // OCCT.NET: OCCT changes the face's own offset surface; the port changes a copy, so the
                            // shape written never changes (decision 1252)
                            anOffsetSu = (OffsetSurface)anOffsetSu.Copy();
                            anOffsetSu.SetBasisSurface(Rev);
                            var MkRev = new GeomToStep_MakeSurface(anOffsetSu, theLocalFactors);
                            Spms = MkRev.Value();
                        }
                        else
                        {
                            var MkRev = new GeomToStep_MakeSurface(Rev, theLocalFactors);
                            Spms = MkRev.Value();
                        }
                    }
                }
            }

            // ----------------
            // Translates Wires
            // ----------------

            StepShape_Loop Loop = null;

            var MkWire = new TopoDSToStep_MakeStepWire();
            var mySeq = new NCollection_Sequence<Transient>();

            // Initialize the Wire Explorer with the forward face

            var WireExp = new Explorer();
            for (WireExp.Init(ForwardFace, ShapeType.Wire); WireExp.More(); WireExp.Next())
            {
                Wire CurrentWire = ShapeCasts.Wire(WireExp.Current());
                if (!CurrentWire.IsNull)
                {
                    MkWire.Init(CurrentWire, aTool, FP, theLocalFactors);
                    if (MkWire.IsDone())
                    {
                        Loop = MkWire.Value() as StepShape_Loop;
                    }
                    else
                    {
                        FP.AddWarning(errShape, " a Wire not mapped");
                        continue;
                    }
                }

                var FaceBound = new StepShape_FaceBound();

                // ----------------------------------------------------
                // When the geometric normal of a Surface is reversed :
                //    - the wire topological orientation is reversed
                // ----------------------------------------------------
                // CAS.CADE face orientation :
                // when a face is reversed in a shell, the orientation of the underlying
                // topology is implicitly reversed. This is not the case in Step.
                // If face orientation is Reversed => the underlying (Step mapped) wire
                // are explicitly reversed

                var aName = new TCollection_HAsciiString("");
                // Ajoute le 30 Juin pour TEST
                // Il convient de reprendre a la base ce probleme d'orientation
                // et notamment la politique d`exploration du Shape (on explore
                // toujours les sous-shapes d'un shape de maniere FORWARD !
                // la modif (on ajoute : si context faceted ... sinon) est a verifier
                // aupres des autres editeurs de CFAO de la Round Table.
                if (!aTool.Faceted() && aFace.Orientation() == Reversed)
                {
                    FaceBound.Init(aName, Loop, (CurrentWire.Orientation() == Reversed));
                }
                else
                {
                    FaceBound.Init(aName, Loop, (CurrentWire.Orientation() == Forward));
                }

                mySeq.Append(FaceBound);
            }

            // ----------------------------------------
            // Translate the Edge 2D Geometry (pcurves)
            // ----------------------------------------

            if (!aTool.Faceted() && aTool.PCurveMode() != 0)
            {
                var Ex = new Explorer(ForwardFace, ShapeType.Edge);

                // ------------------------------------------------
                // Exploration of all the Edges in the current face
                // ------------------------------------------------

                for (; Ex.More(); Ex.Next())
                {
                    Edge E = ShapeCasts.Edge(Ex.Current());
                    Curve2D C2d = Tool.CurveOnSurface(E, ForwardFace, out double cf, out double cl);

                    if (Tool.Degenerated(E) || C2d is null)
                    {
                        // The edge 2D Geometry degenerates in 3D
                        // The edge 2D geometry is not mapped onto any Step entity
                        // (ProStep agreement)
                        continue;
                    }

                    // Copy the Curve2d which might be changed
                    C2d = (Curve2D)C2d.Copy();

                    // for writing VERTEX_LOOP
                    if (!aTool.IsBound(E))
                    {
                        continue;
                    }

                    StepGeom_Curve Cpms = (aTool.Find(E) as StepShape_EdgeCurve).EdgeGeometry();
                    if (Cpms is null)
                    {
                        continue;
                    }

                    if (aTool.IsBound(E))
                    {
                        if (C2d is HyperbolaCurve2D || C2d is ParabolaCurve2D)
                        {
                            if (Su is SphericalSurface || Su is CylindricalSurface || Su is ConicalSurface
                                || Su is ToroidalSurface)
                            {
                                C2d = new TrimmedCurve2D(C2d, cf, cl, true);
                            }
                        }

                        if ((C2d is TrimmedCurve2D) || (C2d is BezierCurve2D))
                        {
                            C2d = Geom2dConvert2D.CurveToBSplineCurve(C2d);
                        }

                        // if the Surface is a RectangularTrimmedSurface,
                        // use the BasisSurface.
                        //   CKY  23 SEP 1996 : on reste en Radian car on code des Radians
                        //    sauf que ca ne marche pas bien ...
                        Curve2D C2dMapped;
                        if (Su is TrimmedSurface)
                        {
                            TrimmedSurface alocalRTS = Su as TrimmedSurface;
                            C2dMapped = Units.RadianToDegree(C2d, alocalRTS.BasisSurface(), theLocalFactors.LengthFactor(),
                                                                         theLocalFactors.FactorRadianDegree());
                        }
                        else
                        {
                            C2dMapped = Units.RadianToDegree(C2d, Su, theLocalFactors.LengthFactor(),
                                                                         theLocalFactors.FactorRadianDegree());
                        }

                        //
                        //	C2dMapped = C2d;  // cky : en remplacement de ce qui precede
                        var MkCurve = new GeomToStep_MakeCurve(C2dMapped, theLocalFactors);

                        // --------------------
                        // Translate the Pcurve
                        // --------------------

                        var Pc = new StepGeom_Pcurve();
                        var DRI = new StepRepr_DefinitionalRepresentation();
                        var aItems = new NCollection_HArray1<StepRepr_RepresentationItem>(1, 1);
                        aItems.SetValue(1, MkCurve.Value());
                        var aContext = new StepGeom_GeometricRepresentationContextAndParametricRepresentationContext();
                        var aContextIdentifier = new TCollection_HAsciiString("2D SPACE");
                        var aContextType = new TCollection_HAsciiString("");
                        int aCoordSpaceDim = 2;
                        aContext.Init(aContextIdentifier, aContextType, aCoordSpaceDim);

                        var aName = new TCollection_HAsciiString("");

                        DRI.Init(aName, aItems, aContext);
                        Pc.Init(aName, Spms, DRI);
                        StepGeom_SurfaceCurve C1pms = Cpms as StepGeom_SurfaceCurve;
                        NCollection_HArray1<StepGeom_PcurveOrSurface> aGeom = C1pms.AssociatedGeometry();
                        if (aGeom is null)
                        {
                            // OCCT's array default-constructs its two SELECT values (empty)
                            aGeom = new NCollection_HArray1<StepGeom_PcurveOrSurface>(1, 2);
                            aGeom.SetValue(1, new StepGeom_PcurveOrSurface());
                            aGeom.SetValue(2, new StepGeom_PcurveOrSurface());
                        }

                        var PcOrSur = new StepGeom_PcurveOrSurface();
                        PcOrSur.SetValue(Pc);
                        if ((aGeom.Value(1)).IsNull())
                        {
                            aGeom.SetValue(1, new StepGeom_PcurveOrSurface(PcOrSur)); // C++ stores a copy
                            if (C1pms is StepGeom_SeamCurve)
                            {
                                aGeom.SetValue(2, new StepGeom_PcurveOrSurface(PcOrSur)); // c est au moins ca
                            }
                        }
                        else if (aGeom.Value(2).IsNull() || //) {
                                 C1pms is StepGeom_SeamCurve)
                        { //: a8 abv 13 Feb 98: allow seam to have two different pcurves
                            aGeom.SetValue(2, new StepGeom_PcurveOrSurface(PcOrSur));
                        }

                        C1pms.SetAssociatedGeometry(aGeom);
                    }
                }
            }

            // ------------------
            // Translate the Face
            // ------------------

            int nbWires = mySeq.Length();
            if (nbWires != 0)
            {
                var aBounds = new NCollection_HArray1<StepShape_FaceBound>(1, nbWires);
                for (int i = 1; i <= nbWires; i++)
                {
                    aBounds.SetValue(i, mySeq.Value(i) as StepShape_FaceBound);
                }

                var Fpms = new StepShape_AdvancedFace();
                // The underlying surface has always a direct axis (see above)
                var aName = new TCollection_HAsciiString("");

                Fpms.Init(aName, aBounds, Spms, aFace.Orientation() == Forward);

                aTool.Bind(aFace, Fpms);
                myError = TopoDSToStep_MakeFaceError.TopoDSToStep_FaceDone;
                myResult = Fpms;
                done = true;
            }
            else
            {
                // ----------------------------
                // MakeFace Face Error Handling
                // ----------------------------
                FP.AddWarning(errShape, " No Wires of this Face were mapped");
                myError = TopoDSToStep_MakeFaceError.TopoDSToStep_NoWireMapped;
                done = false;
            }
        }
        catch (FailureException theFailure)
        {
            FP.AddFail(errShape, Transfer_ProcessForTransient.What(theFailure));
            myError = TopoDSToStep_MakeFaceError.TopoDSToStep_FaceOther;
            done = false;
        }
    }

    public void Init(in Face aFace, TopoDSToStep_Tool aTool, Transfer_FinderProcess FP)
        => Init(aFace, aTool, FP, new StepData_Factors());

    /// <exception cref="NotDoneException">The mapping wasn't done.</exception>
    public StepShape_TopologicalRepresentationItem Value()
    {
        StdFail_NotDone_Raise_if(!done, "TopoDSToStep_MakeStepFace::Value() - no result");
        return myResult;
    }

    public TopoDSToStep_MakeFaceError Error() => myError;
}
