// Port of: src/DataExchange/TKDESTEP/TopoDSToStep/TopoDSToStep_WireframeBuilder.hxx
// Port of: src/DataExchange/TKDESTEP/TopoDSToStep/TopoDSToStep_WireframeBuilder.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using static Occt.StepGeom_TrimmingPreference;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;
using static Occt.TopoDSToStep_BuilderError;

namespace Occt;

/// <summary>Builds the trimmed curves of the edges of a shape (an edge, wire, face, shell, solid or compound).</summary>
public sealed class TopoDSToStep_WireframeBuilder : TopoDSToStep_Root
{
    private const int Nbpt = 23;

    private NCollection_HSequence<Transient> myResult;
    private TopoDSToStep_BuilderError myError;

    public TopoDSToStep_WireframeBuilder()
    {
        myError = TopoDSToStep_BuilderOther;
        done = false;
    }

    public TopoDSToStep_WireframeBuilder(in TopoDS_Shape aShape, TopoDSToStep_Tool aTool, in StepData_Factors theLocalFactors)
    {
        done = false;
        Init(aShape, aTool, theLocalFactors);
    }

    public TopoDSToStep_WireframeBuilder(in TopoDS_Shape aShape, TopoDSToStep_Tool aTool) : this(aShape, aTool, new StepData_Factors())
    {
    }

    public void Init(in TopoDS_Shape aShape, TopoDSToStep_Tool T, in StepData_Factors theLocalFactors)
    {
        _ = T;
        var itemList = new NCollection_HSequence<Transient>();
        var aPmsMap = new NCollection_DataMap<TopoDS_Shape, Transient>(TopTools_ShapeMapHasher.Instance);
        done = GetTrimmedCurveFromShape(aShape, aPmsMap, ref itemList, theLocalFactors);
        myResult = itemList;
    }

    public void Init(in TopoDS_Shape aShape, TopoDSToStep_Tool T) => Init(aShape, T, new StepData_Factors());

    public TopoDSToStep_BuilderError Error() => myError;

    /// <summary>The curves made.</summary>
    /// <exception cref="NotDoneException">The build wasn't done.</exception>
    public NCollection_HSequence<Transient> Value()
    {
        StdFail_NotDone_Raise_if(!done, "TopoDSToStep_WireframeBuilder::Value() - no result");
        return myResult;
    }

    //: S4134: abv 10 Mar 99: the methods below moved from package TopoDSToGBWire

    private static StepGeom_TrimmedCurve MakeTrimmedCurve(StepGeom_Curve C, StepGeom_CartesianPoint P1, StepGeom_CartesianPoint P2,
                                                          double trim1, double trim2, bool sense)
    {
        var aSTS1 = new NCollection_HArray1<StepGeom_TrimmingSelect>(1, 2);
        var tSel = new StepGeom_TrimmingSelect();
        tSel.SetValue(P1);
        aSTS1.SetValue(1, new StepGeom_TrimmingSelect(tSel)); // C++ stores a copy
        tSel.SetParameterValue(trim1);
        aSTS1.SetValue(2, new StepGeom_TrimmingSelect(tSel));

        var aSTS2 = new NCollection_HArray1<StepGeom_TrimmingSelect>(1, 2);
        tSel.SetValue(P2);
        aSTS2.SetValue(1, new StepGeom_TrimmingSelect(tSel));
        tSel.SetParameterValue(trim2);
        aSTS2.SetValue(2, new StepGeom_TrimmingSelect(tSel));

        var empty = new TCollection_HAsciiString("");
        var pmsTC = new StepGeom_TrimmedCurve();
        pmsTC.Init(empty, C, aSTS1, aSTS2, sense, StepGeom_tpParameter);
        return pmsTC;
    }

    /// <summary>Extracts the trimmed curve of an edge (on a face, which may be null), into the list.</summary>
    public bool GetTrimmedCurveFromEdge(in TopoDS_Edge theEdge, in TopoDS_Face theFace, NCollection_DataMap<TopoDS_Shape, Transient> theMap,
                                        ref NCollection_HSequence<Transient> theCurveList, in StepData_Factors theLocalFactors)
    {
        if (theEdge.Orientation() == TopAbs_INTERNAL || theEdge.Orientation() == TopAbs_EXTERNAL)
        {
            // OCCT_DEBUG output left out
            return false;
        }

        // szv#4:S4163:12Mar99 SGI warns
        TopoDS_Shape aSh = theEdge.Oriented(TopAbs_FORWARD);
        TopoDS_Edge anEdge = TopoDS.Edge(aSh);

        // resulting curve
        StepGeom_Curve aSGC = null;
        if (theMap.Find(anEdge, out Transient aTransient))
        {
            aSGC = aTransient as StepGeom_Curve; // OCCT: found, then made again
        }

        var aCA = new BRepAdaptor_Curve();
        try
        {
            aCA.Initialize(anEdge);
        }
        catch (NullObjectException)
        {
            return false;
        }

        TopoDS_Vertex aVFirst = new TopoDS_Vertex(), aVLast = new TopoDS_Vertex();
        StepGeom_CartesianPoint aSGCP1 = null, aSGCP2 = null;
        for (var anExp = new TopExp_Explorer(anEdge, TopAbs_VERTEX); anExp.More(); anExp.Next())
        {
            TopoDS_Vertex aVertex = TopoDS.Vertex(anExp.Value());
            Point3D aGpP = BRep_Tool.Pnt(aVertex);
            if (aVertex.Orientation() == TopAbs_FORWARD)
            {
                aVFirst = aVertex;
                // 1.point for trimming
                var aGTSMCP = new GeomToStep_MakeCartesianPoint(aGpP, theLocalFactors.LengthFactor());
                aSGCP1 = aGTSMCP.Value();
            }

            if (aVertex.Orientation() == TopAbs_REVERSED)
            {
                aVLast = aVertex;
                // 2.point for trimming
                var aGTSMCP = new GeomToStep_MakeCartesianPoint(aGpP, theLocalFactors.LengthFactor());
                aSGCP2 = aGTSMCP.Value();
            }
        }

        Geom_Curve aC = BRep_Tool.Curve(anEdge, out double aFirst, out double aLast);

        if (aC is not null)
        {
            if (aC is Geom_TrimmedCurve aTrimmed)
            {
                aC = aTrimmed.BasisCurve();
            }

            var aGTSMC = new GeomToStep_MakeCurve(aC, theLocalFactors);
            if (!aGTSMC.IsDone())
            {
                return false;
            }

            StepGeom_Curve aPMSC = aGTSMC.Value();

            // trim the curve
            double aTrim1 = aCA.FirstParameter();
            double aTrim2 = aCA.LastParameter();

            if (aVFirst.IsNull() && aVLast.IsNull() && global::Kerf.Engine.Foundation.Precision.IsInfinite(aFirst) && global::Kerf.Engine.Foundation.Precision.IsInfinite(aLast))
            {
                var aCurveMaker = new GeomToStep_MakeCurve(aC, theLocalFactors);
                if (aCurveMaker.IsDone())
                {
                    aSGC = aCurveMaker.Value();
                    theCurveList.Append(aSGC);
                    return true;
                }

                return false;
            }

            if (aVFirst.IsNull())
            {
                var aGTSMCP = new GeomToStep_MakeCartesianPoint(aCA.Value(aFirst), theLocalFactors.LengthFactor());
                aSGCP1 = aGTSMCP.Value();
            }

            if (aVLast.IsNull())
            {
                var aGTSMCP = new GeomToStep_MakeCartesianPoint(aCA.Value(aLast), theLocalFactors.LengthFactor());
                aSGCP2 = aGTSMCP.Value();
            }

            aSGC = MakeTrimmedCurve(aPMSC, aSGCP1, aSGCP2, aTrim1, aTrim2, true);
        }
        else
        {
            // -------------------------
            // a 3D Curve is constructed
            // -------------------------
            bool aIPlan = false;
            if (!theFace.IsNull())
            {
                Geom2d_Curve aC2d = BRep_Tool.CurveOnSurface(anEdge, theFace, out double aCF, out double aCL);
                Geom_Surface aS = BRep_Tool.Surface(theFace);
                if (aS is Geom_Plane && aC2d is Geom2d_Line)
                {
                    aIPlan = true;
                }
            }

            // to be modified : cf and cl are the topological trimming parameter
            // these are computed after ! (U1 and U2) -> cf and cl instead
            if (aIPlan)
            {
                Point3D aPnt1 = aCA.Value(aCA.FirstParameter()), aPnt2 = aCA.Value(aCA.LastParameter());
                Vector3D aV = new Vector3D(aPnt1, aPnt2);
                double aLength = aV.Magnitude();
                if (aLength >= global::Kerf.Engine.Foundation.Precision.Confusion())
                {
                    var aL = new Geom_Line(aPnt1, new Direction3D(aV));
                    var aGTSML = new GeomToStep_MakeLine(aL, theLocalFactors);
                    aSGC = aGTSML.Value();
                    aSGC = MakeTrimmedCurve(aGTSML.Value(), aSGCP1, aSGCP2, 0, aLength, true);
                }
                // OCCT_DEBUG output left out
            }
            else
            {
                var aPoints = new NCollection_Array1<Point3D>(1, Nbpt);
                var aKnots = new NCollection_Array1<double>(1, Nbpt);
                var aMult = new NCollection_Array1<int>(1, Nbpt);
                double aU1 = aCA.FirstParameter();
                double aU2 = aCA.LastParameter();
                for (int i = 1; i <= Nbpt; i++)
                {
                    double aU = aU1 + (i - 1) * (aU2 - aU1) / (Nbpt - 1);
                    Point3D aP = aCA.Value(aU);
                    aPoints.SetValue(i, aP);
                    aKnots.SetValue(i, aU);
                    aMult.SetValue(i, 1);
                }

                aPoints.SetValue(1, BRep_Tool.Pnt(aVFirst));
                aPoints.SetValue(Nbpt, BRep_Tool.Pnt(aVLast));
                aMult.SetValue(1, 2);
                aMult.SetValue(Nbpt, 2);
                Geom_Curve aBSCurve = new Geom_BSplineCurve(aPoints, aKnots, aMult, 1);
                var aGTSMC = new GeomToStep_MakeCurve(aBSCurve, theLocalFactors);
                aSGC = aGTSMC.Value();
            }
        }

        if (aSGC is null)
        {
            return false;
        }

        theMap.Bind(anEdge, aSGC);
        theCurveList.Append(aSGC);
        return true;
    }

    /// <summary>Extracts the trimmed curves of a face's edges, into the list.</summary>
    public bool GetTrimmedCurveFromFace(in TopoDS_Face aFace, NCollection_DataMap<TopoDS_Shape, Transient> aMap,
                                        ref NCollection_HSequence<Transient> aCurveList, in StepData_Factors theLocalFactors)
    {
        TopoDS_Shape curShape;
        TopoDS_Edge curEdge;
        var exp = new TopExp_Explorer();
        bool result = false; // szv#4:S4163:12Mar99 `done` hid one from this, initialisation needed

        for (exp.Init(aFace, TopAbs_EDGE); exp.More(); exp.Next())
        {
            curShape = exp.Current();
            curEdge = TopoDS.Edge(curShape);
            if (GetTrimmedCurveFromEdge(curEdge, aFace, aMap, ref aCurveList, theLocalFactors))
            {
                result = true;
            }
        }

        return result;
    }

    /// <summary>Extracts the trimmed curves of a shape's edges, into the list.</summary>
    public bool GetTrimmedCurveFromShape(in TopoDS_Shape aShape, NCollection_DataMap<TopoDS_Shape, Transient> aMap,
                                         ref NCollection_HSequence<Transient> aCurveList, in StepData_Factors theLocalFactors)
    {
        var It = new TopoDS_Iterator();
        bool result = false; // szv#4:S4163:12Mar99 `done` hid one from this, initialisation needed

        // szv#4:S4163:12Mar99 optimized
        switch (aShape.ShapeType())
        {
            case TopAbs_EDGE:
            {
                TopoDS_Edge curEdge = TopoDS.Edge(aShape);
                TopoDS_Face nulFace = new TopoDS_Face();
                result = GetTrimmedCurveFromEdge(curEdge, nulFace, aMap, ref aCurveList, theLocalFactors);
                break;
            }
            case TopAbs_WIRE:
            {
                TopoDS_Face nulFace = new TopoDS_Face();
                TopoDS_Shape curShape;
                TopoDS_Edge curEdge;
                var exp = new TopExp_Explorer();

                for (exp.Init(aShape, TopAbs_EDGE); exp.More(); exp.Next())
                {
                    curShape = exp.Current();
                    curEdge = TopoDS.Edge(curShape);
                    if (GetTrimmedCurveFromEdge(curEdge, nulFace, aMap, ref aCurveList, theLocalFactors))
                    {
                        result = true;
                    }
                }

                break;
            }
            case TopAbs_FACE:
            {
                TopoDS_Face curFace = TopoDS.Face(aShape);
                result = GetTrimmedCurveFromFace(curFace, aMap, ref aCurveList, theLocalFactors);
                break;
            }
            case TopAbs_SHELL:
            {
                TopoDS_Shell Sh = TopoDS.Shell(aShape);
                It.Initialize(Sh);
                for (; It.More(); It.Next())
                {
                    TopoDS_Face curFace = TopoDS.Face(It.Value());
                    if (GetTrimmedCurveFromFace(curFace, aMap, ref aCurveList, theLocalFactors))
                    {
                        result = true;
                    }
                }

                break;
            }

            case TopAbs_SOLID:
            {
                It.Initialize(aShape);
                for (; It.More(); It.Next())
                {
                    if (It.Value().ShapeType() == TopAbs_SHELL)
                    {
                        if (GetTrimmedCurveFromShape(It.Value(), aMap, ref aCurveList, theLocalFactors))
                        {
                            result = true;
                        }
                    }
                }

                break;
            }

            case TopAbs_COMPOUND:
            {
                It.Initialize(aShape);
                for (; It.More(); It.Next())
                {
                    if (GetTrimmedCurveFromShape(It.Value(), aMap, ref aCurveList, theLocalFactors))
                    {
                        result = true;
                    }
                }

                break;
            }
            default:
                break;
        }

        return result;
    }
}
