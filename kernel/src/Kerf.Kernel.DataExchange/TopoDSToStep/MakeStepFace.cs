// Port of: src/DataExchange/TKDESTEP/TopoDSToStep/TopoDSToStep_MakeStepFace.hxx
// Port of: src/DataExchange/TKDESTEP/TopoDSToStep/TopoDSToStep_MakeStepFace.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Exchange.Framework.Transfer;
using Kerf.Engine.Exchange.Step.Data;
using Kerf.Engine.Exchange.Step.Export.Geometry;
using Kerf.Engine.Exchange.Step.Geometry;
using Kerf.Engine.Exchange.Step.Representation;
using Kerf.Engine.Exchange.Step.Shapes;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Conversion;
using Kerf.Engine.Healing;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Geometry.Continuity;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;
using static Kerf.Engine.Exchange.Step.Export.MakeFaceError;

namespace Kerf.Engine.Exchange.Step.Export;

/// <summary>
/// Maps a TopoDS_Face (with its relative orientation in the shell) to a STEP advanced_face: its surface, its bounds and
/// the pcurves of its edges.
/// </summary>
internal sealed class MakeStepFace : ExportRoot
{
    private StepTopologicalRepresentationItem _result;
    private MakeFaceError _error;

    public MakeStepFace()
    {
        _error = MakeFaceError.FaceOther;
        _done = false;
    }

    public MakeStepFace(in Face f, ExportTool t, FinderProcess fp, in Factors localFactors)
    {
        _done = false;
        Init(f, t, fp, localFactors);
    }

    public MakeStepFace(in Face f, ExportTool t, FinderProcess fp)
        : this(f, t, fp, new Factors())
    {
    }

    /// <remarks>Needs <see cref="ShapeAlgo.Init"/> (degenerate tori are rewritten through its container).</remarks>
    public void Init(in Face aFace, ExportTool aTool, FinderProcess fp, in Factors localFactors)
    {
        // --------------------------------------------------------------
        // the face is given with its relative orientation (in the Shell)
        // --------------------------------------------------------------

        // szv#4:S4163:12Mar99 SGI warns
        Shape sh = aFace.Oriented(Forward);
        Face forwardFace = ShapeCasts.Face(sh);
        aTool.SetCurrentFace(forwardFace);
        var errShape = new ShapeMapper(aFace); // on ne sait jamais

        // [BEGIN] Processing non-manifold topology (another approach) (ssv; 10.11.2010)
        if ((fp.Model() as StepModel).InternalParameters.WriteNonmanifold)
        {
            StepAdvancedFace af = null;
            ShapeMapper stepMapper = global::Kerf.Engine.Exchange.Framework.Transfer.TransferBRep.ShapeMapper(fp, aFace);
            if (fp.FindTypedTransient(stepMapper, typeof(StepAdvancedFace), ref af))
            {
                // Non-manifold topology detected
                var linkingAF = new StepAdvancedFace();
                linkingAF.Init(af.Name(), af.Bounds(), af.FaceGeometry(), !af.SameSense());

                _error = MakeFaceError.FaceDone;
                _result = linkingAF;
                _done = true;
                return;
            }
        }
        // [END] Processing non-manifold topology (ssv; 10.11.2010)

        if (aTool.IsBound(aFace))
        {
            _error = MakeFaceError.FaceDone;
            _done = true;
            _result = aTool.Find(aFace);
            return;
        }

        if (aFace.Orientation == Internal || aFace.Orientation == External)
        {
            fp.AddWarning(errShape, " Face from Non Manifold Topology");
            _error = MakeFaceError.NonManifoldFace;
            _done = false;
            return;
        }

        // ------------------
        // Get the Outer Wire
        // ------------------

        Wire theOuterWire = BrepTools.OuterWire(forwardFace);

        if (theOuterWire.IsNull)
        {
            fp.AddWarning(errShape, " Face without wire not mapped");
            _error = MakeFaceError.InfiniteFace;
            _done = false;
            return;
        }

        try
        {
            // -----------------
            // Translate Surface
            // -----------------

            Surface su = Tool.Surface(forwardFace);
            if (su is null)
            {
                fp.AddWarning(errShape, " Face without geometry not mapped");
                _error = MakeFaceError.FaceOther;
                _done = false;
                return;
            }

            //  CKY  23 SEP 1996 : une FACE de Step n a pas droit a RECTANGULAR_TRIMMED...
            //  Il faut donc d abord "demonter" la RectangularTrimmedSurface pour
            //  passer la Surface de base
            TrimmedSurface rts = su as TrimmedSurface;
            if (rts is not null)
            {
                su = rts.BasisSurface;
            }

            // Surfaces with indirect Axes are already reversed
            aTool.SetSurfaceReversed(false);

            var mkSurface = new MakeSurface(su, localFactors);
            StepSurface spms = mkSurface.Value();

            //%pdn 30 Nov 98: TestRally 9 issue on r1001_ec.stp:
            // toruses with major_radius < minor are re-coded as degenerate
            // rln 19.01.99: uncomment %30 pdn for integration into K4L
            {
                // If the surface is Offset it is necessary to check the base surface
                bool surfaceIsOffset = false;
                OffsetSurface offsetSu = null;
                if (su is OffsetSurface)
                {
                    surfaceIsOffset = true;
                    offsetSu = su as OffsetSurface;
                }

                if ((spms is StepToroidalSurface) || ((surfaceIsOffset) && offsetSu.BasisSurface is ToroidalSurface))
                {
                    ToroidalSurface ts;
                    if (surfaceIsOffset)
                    {
                        ts = offsetSu.BasisSurface as ToroidalSurface;
                    }
                    else
                    {
                        ts = su as ToroidalSurface;
                    }

                    double R = ts.MajorRadius;
                    double r = ts.MinorRadius;
                    if (R < r) // if torus is degenerate or base surface is degenerate, make revolution instead
                    {
                        CoordinateSystem ax3 = ts.Position;
                        Point3D pos = ax3.Location;
                        Direction3D dir = ax3.Direction;
                        Direction3D x = ax3.XDirection;

                        // create basis curve
                        global::Kerf.Engine.Healing.ShapeAlgo.AlgoContainer().GetFaceUVBounds(aFace, out double uf, out double ul, out double vf, out double vl);
                        Frame3D ax2 = new Frame3D(new Point3D(pos.XYZ() + x.XYZ() * ts.MajorRadius), x ^ dir, x);
                        Curve3D basisCurve = new CircleCurve3D(ax2, ts.MinorRadius);

                        // convert basis curve to bspline in order to avoid self-intersecting
                        // surface of revolution (necessary e.g. for CATIA)
                        if (vl - vf - 2 * M_PI < -global::Kerf.Engine.Foundation.Precision.PConfusion())
                        {
                            basisCurve = global::Kerf.Engine.Healing.ShapeAlgo.AlgoContainer().ConvertCurveToBSpline(basisCurve, vf, vl, global::Kerf.Engine.Foundation.Precision.Approximation(),
                                                                                        C1, 100, 9);
                        }

                        // create surface of revolution
                        Axis3D axis = ax3.Axis;
                        if (!ax3.IsDirect)
                        {
                            axis = axis.Reversed();
                        }

                        var rev = new RevolutionSurface(basisCurve, axis);

                        // and translate it
                        if (surfaceIsOffset)
                        {
                            // OCCT.NET: OCCT changes the face's own offset surface; the port changes a copy, so the
                            // shape written never changes (decision 1252)
                            offsetSu = (OffsetSurface)offsetSu.Copy();
                            offsetSu.SetBasisSurface(rev);
                            var mkRev = new MakeSurface(offsetSu, localFactors);
                            spms = mkRev.Value();
                        }
                        else
                        {
                            var mkRev = new MakeSurface(rev, localFactors);
                            spms = mkRev.Value();
                        }
                    }
                }
            }

            // ----------------
            // Translates Wires
            // ----------------

            StepLoop loop = null;

            var mkWire = new MakeStepWire();
            var mySeq = new NCollection_Sequence<Transient>();

            // Initialize the Wire Explorer with the forward face

            var wireExp = new Explorer();
            for (wireExp.Init(forwardFace, ShapeType.Wire); wireExp.More(); wireExp.Next())
            {
                Wire currentWire = ShapeCasts.Wire(wireExp.Current());
                if (!currentWire.IsNull)
                {
                    mkWire.Init(currentWire, aTool, fp, localFactors);
                    if (mkWire.IsDone)
                    {
                        loop = mkWire.Value() as StepLoop;
                    }
                    else
                    {
                        fp.AddWarning(errShape, " a Wire not mapped");
                        continue;
                    }
                }

                var faceBound = new StepFaceBound();

                // ----------------------------------------------------
                // When the geometric normal of a Surface is reversed :
                //    - the wire topological orientation is reversed
                // ----------------------------------------------------
                // CAS.CADE face orientation :
                // when a face is reversed in a shell, the orientation of the underlying
                // topology is implicitly reversed. This is not the case in Step.
                // If face orientation is Reversed => the underlying (Step mapped) wire
                // are explicitly reversed

                var name = new TCollection_HAsciiString("");
                // Ajoute le 30 Juin pour TEST
                // Il convient de reprendre a la base ce probleme d'orientation
                // et notamment la politique d`exploration du Shape (on explore
                // toujours les sous-shapes d'un shape de maniere FORWARD !
                // la modif (on ajoute : si context faceted ... sinon) est a verifier
                // aupres des autres editeurs de CFAO de la Round Table.
                if (!aTool.Faceted() && aFace.Orientation == Reversed)
                {
                    faceBound.Init(name, loop, (currentWire.Orientation == Reversed));
                }
                else
                {
                    faceBound.Init(name, loop, (currentWire.Orientation == Forward));
                }

                mySeq.Append(faceBound);
            }

            // ----------------------------------------
            // Translate the Edge 2D Geometry (pcurves)
            // ----------------------------------------

            if (!aTool.Faceted() && aTool.PCurveMode() != 0)
            {
                var ex = new Explorer(forwardFace, ShapeType.Edge);

                // ------------------------------------------------
                // Exploration of all the Edges in the current face
                // ------------------------------------------------

                for (; ex.More(); ex.Next())
                {
                    Edge e = ShapeCasts.Edge(ex.Current());
                    Curve2D c2d = Tool.CurveOnSurface(e, forwardFace, out double cf, out double cl);

                    if (Tool.Degenerated(e) || c2d is null)
                    {
                        // The edge 2D Geometry degenerates in 3D
                        // The edge 2D geometry is not mapped onto any Step entity
                        // (ProStep agreement)
                        continue;
                    }

                    // Copy the Curve2d which might be changed
                    c2d = (Curve2D)c2d.Copy();

                    // for writing VERTEX_LOOP
                    if (!aTool.IsBound(e))
                    {
                        continue;
                    }

                    StepCurve cpms = (aTool.Find(e) as StepEdgeCurve).EdgeGeometry();
                    if (cpms is null)
                    {
                        continue;
                    }

                    if (aTool.IsBound(e))
                    {
                        if (c2d is HyperbolaCurve2D || c2d is ParabolaCurve2D)
                        {
                            if (su is SphericalSurface || su is CylindricalSurface || su is ConicalSurface
                                || su is ToroidalSurface)
                            {
                                c2d = new TrimmedCurve2D(c2d, cf, cl, true);
                            }
                        }

                        if ((c2d is TrimmedCurve2D) || (c2d is BezierCurve2D))
                        {
                            c2d = Geom2dConvert2D.CurveToBSplineCurve(c2d);
                        }

                        // if the Surface is a RectangularTrimmedSurface,
                        // use the BasisSurface.
                        //   CKY  23 SEP 1996 : on reste en Radian car on code des Radians
                        //    sauf que ca ne marche pas bien ...
                        Curve2D c2dMapped;
                        if (su is TrimmedSurface)
                        {
                            TrimmedSurface alocalRTS = su as TrimmedSurface;
                            c2dMapped = Units.RadianToDegree(c2d, alocalRTS.BasisSurface, localFactors.LengthFactor(),
                                                                         localFactors.FactorRadianDegree());
                        }
                        else
                        {
                            c2dMapped = Units.RadianToDegree(c2d, su, localFactors.LengthFactor(),
                                                                         localFactors.FactorRadianDegree());
                        }

                        //
                        //	C2dMapped = C2d;  // cky : en remplacement de ce qui precede
                        var mkCurve = new MakeCurve(c2dMapped, localFactors);

                        // --------------------
                        // Translate the Pcurve
                        // --------------------

                        var pc = new StepPcurve();
                        var dri = new DefinitionalRepresentation();
                        var items = new NCollection_HArray1<RepresentationItem>(1, 1);
                        items.SetValue(1, mkCurve.Value());
                        var context = new StepGeometricRepresentationContextAndParametricRepresentationContext();
                        var contextIdentifier = new TCollection_HAsciiString("2D SPACE");
                        var contextType = new TCollection_HAsciiString("");
                        int coordSpaceDim = 2;
                        context.Init(contextIdentifier, contextType, coordSpaceDim);

                        var name = new TCollection_HAsciiString("");

                        dri.Init(name, items, context);
                        pc.Init(name, spms, dri);
                        StepSurfaceCurve c1pms = cpms as StepSurfaceCurve;
                        NCollection_HArray1<StepPcurveOrSurface> geom = c1pms.AssociatedGeometry();
                        if (geom is null)
                        {
                            // OCCT's array default-constructs its two SELECT values (empty)
                            geom = new NCollection_HArray1<StepPcurveOrSurface>(1, 2);
                            geom.SetValue(1, new StepPcurveOrSurface());
                            geom.SetValue(2, new StepPcurveOrSurface());
                        }

                        var pcOrSur = new StepPcurveOrSurface();
                        pcOrSur.SetValue(pc);
                        if ((geom.Value(1)).IsNull)
                        {
                            geom.SetValue(1, new StepPcurveOrSurface(pcOrSur)); // C++ stores a copy
                            if (c1pms is StepSeamCurve)
                            {
                                geom.SetValue(2, new StepPcurveOrSurface(pcOrSur)); // c est au moins ca
                            }
                        }
                        else if (geom.Value(2).IsNull || //) {
                                 c1pms is StepSeamCurve)
                        { //: a8 abv 13 Feb 98: allow seam to have two different pcurves
                            geom.SetValue(2, new StepPcurveOrSurface(pcOrSur));
                        }

                        c1pms.SetAssociatedGeometry(geom);
                    }
                }
            }

            // ------------------
            // Translate the Face
            // ------------------

            int nbWires = mySeq.Length();
            if (nbWires != 0)
            {
                var bounds = new NCollection_HArray1<StepFaceBound>(1, nbWires);
                for (int i = 1; i <= nbWires; i++)
                {
                    bounds.SetValue(i, mySeq.Value(i) as StepFaceBound);
                }

                var fpms = new StepAdvancedFace();
                // The underlying surface has always a direct axis (see above)
                var name = new TCollection_HAsciiString("");

                fpms.Init(name, bounds, spms, aFace.Orientation == Forward);

                aTool.Bind(aFace, fpms);
                _error = MakeFaceError.FaceDone;
                _result = fpms;
                _done = true;
            }
            else
            {
                // ----------------------------
                // MakeFace Face Error Handling
                // ----------------------------
                fp.AddWarning(errShape, " No Wires of this Face were mapped");
                _error = MakeFaceError.NoWireMapped;
                _done = false;
            }
        }
        catch (Exception theFailure) when (Refusal.IsFailure(theFailure))
        {
            fp.AddFail(errShape, ProcessForTransient.What(theFailure));
            _error = MakeFaceError.FaceOther;
            _done = false;
        }
    }

    public void Init(in Face aFace, ExportTool aTool, FinderProcess fp)
        => Init(aFace, aTool, fp, new Factors());

    /// <exception cref="NotDoneException">The mapping wasn't done.</exception>
    public StepTopologicalRepresentationItem Value()
    {
        StdFail_NotDone_Raise_if(!_done, "TopoDSToStep_MakeStepFace::Value() - no result");
        return _result;
    }

    public MakeFaceError Error() => _error;
}
