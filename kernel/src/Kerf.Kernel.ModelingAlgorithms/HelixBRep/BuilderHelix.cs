// Port of: src/ModelingAlgorithms/TKHelix/HelixBRep/HelixBRep_BuilderHelix.hxx
// Port of: src/ModelingAlgorithms/TKHelix/HelixBRep/HelixBRep_BuilderHelix.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Construction.Helix.Geometry;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Engine;
using static Kerf.Topology.Orientation;

namespace Kerf.Engine.Construction.Helix;

/// <summary>
/// Builds a helix wire: a composite helix of several parts, each with its own height, pitch (or number of turns)
/// and start and end diameters, about an axis.
/// </summary>
/// <remarks>
/// <para>Error status: 0 OK; 1 just initialized (no parameters set); 2 the approximation failed; 3 an edge could not
/// be made; 10 the start point is too close to the axis; 11 the pitch is below the tolerance; 12 the height is below
/// the tolerance; 13 the taper angle is above π/2 - 1e-7 (OCCT documents negative angles as errors too, but doesn't
/// check them: occt-issues.md, unit 7D). Warning status: 0 OK; 1 the tolerance reached by the
/// approximation is above the requested one.</para>
/// <para>A working object (decision 21). <see cref="Shape"/> is a publication point: it freezes the wire.</para>
/// </remarks>
internal class BuilderHelix
{
    protected CoordinateSystem _axis3 = new();
    protected NCollection_HArray1<double>? _diams;
    protected NCollection_HArray1<double>? _heights;
    protected NCollection_HArray1<double>? _pitches;
    protected NCollection_HArray1<bool>? _isPitches;
    protected double _tolerance;
    protected double _tolReached;
    protected Continuity _continuity;
    protected int _maxDegree;
    protected int _maxSegments;
    protected int _errorStatus;
    protected int _warningStatus;
    protected Shape _shape = new();
    private int _nParts;

    /// <summary>A builder with no parameters (error status 1), about the global Z axis.</summary>
    public BuilderHelix()
    {
        Point3D p0 = new(0.0, 0.0, 0);
        // Initialize coordinate system and data arrays
        _axis3 = _axis3.WithDirection(Constants.DZ());
        _axis3 = _axis3.WithLocation(p0);
        _diams = null;
        _heights = null;
        _pitches = null;
        _isPitches = null;
        _nParts = 1;
        _shape.Nullify();

        // Initialize approximation parameters
        _tolerance = 0.0001;
        _continuity = Continuity.C1;
        _maxDegree = 8;
        _maxSegments = 1000;
        // Initialize status variables
        _tolReached = 99.0;
        // Set initial error state
        _errorStatus = 1;
        _warningStatus = 1;
    }

    /// <summary>
    /// Sets the parameters of a general composite helix: np + 1 diameters, and np heights, pitches and flags (true:
    /// the pitch is a pitch; false: it is a number of turns).
    /// </summary>
    /// <exception cref="ConstructionErrorException">The arrays' lengths don't match.</exception>
    public void SetParameters(in CoordinateSystem axis,
                              NCollection_Array1<double> diams,
                              NCollection_Array1<double> heights,
                              NCollection_Array1<double> pitches,
                              NCollection_Array1<bool> bIsPitches)
    {
        _nParts = diams.Length() - 1;

        _axis3 = axis;

        _diams = null;
        _heights = null;
        _pitches = null;
        _shape.Nullify();

        _errorStatus = 1;
        _warningStatus = 1;

        if (_nParts != heights.Length() || _nParts != pitches.Length()
            || _nParts != bIsPitches.Length())
        {
            throw new ConstructionErrorException(
                "HelixBRep_BuilderHelix::SetParameters: wrong array dimension");
        }

        _diams = new NCollection_HArray1<double>(1, _nParts + 1);
        _heights = new NCollection_HArray1<double>(1, _nParts);
        _pitches = new NCollection_HArray1<double>(1, _nParts);
        _isPitches = new NCollection_HArray1<bool>(1, _nParts);

        _diams.ChangeArray1().Assign(diams);
        _heights.ChangeArray1().Assign(heights);
        _pitches.ChangeArray1().Assign(pitches);
        _isPitches.ChangeArray1().Assign(bIsPitches);

        _errorStatus = 0;
        _warningStatus = 0;
    }

    /// <summary>Sets the parameters of a pure (cylindrical) helix of diameter <paramref name="diam"/>.</summary>
    /// <exception cref="ConstructionErrorException">The arrays' lengths don't match.</exception>
    public void SetParameters(in CoordinateSystem axis,
                              double diam,
                              NCollection_Array1<double> heights,
                              NCollection_Array1<double> pitches,
                              NCollection_Array1<bool> bIsPitches)
    {
        int nbParts = heights.Length();
        NCollection_Array1<double> diams = new(1, nbParts + 1);
        diams.Init(diam);
        SetParameters(axis, diams, heights, pitches, bIsPitches);
    }

    /// <summary>
    /// Sets the parameters of a pure spiral: the diameter changes linearly with the height from
    /// <paramref name="diam1"/> to <paramref name="diam2"/>.
    /// </summary>
    /// <exception cref="ConstructionErrorException">The arrays' lengths don't match.</exception>
    public void SetParameters(in CoordinateSystem axis,
                              double diam1,
                              double diam2,
                              NCollection_Array1<double> heights,
                              NCollection_Array1<double> pitches,
                              NCollection_Array1<bool> bIsPitches)
    {
        int nbParts = heights.Length();
        NCollection_Array1<double> diams = new(1, nbParts + 1);

        int i, j;
        double h = 0.0;
        for (i = heights.Lower(); i <= heights.Upper(); ++i)
        {
            h += heights[i];
        }
        double k = (diam2 - diam1) / h;
        diams[1] = diam1;
        diams[nbParts + 1] = diam2;

        // OCCT: theHeights(1), not theHeights(Lower()): an array not starting at 1 is read wrongly or throws.
        h = heights[1];
        for (i = heights.Lower() + 1, j = 2; i <= heights.Upper(); ++i, ++j)
        {
            diams[j] = diam1 + k * h;
            h += heights[i];
        }

        SetParameters(axis, diams, heights, pitches, bIsPitches);
    }

    /// <summary>
    /// Sets the parameters of a general composite helix by pitches and numbers of turns: np + 1 diameters, np pitches
    /// and np numbers of turns.
    /// </summary>
    /// <exception cref="ConstructionErrorException">The arrays' lengths don't match.</exception>
    public void SetParameters(in CoordinateSystem axis,
                              NCollection_Array1<double> diams,
                              NCollection_Array1<double> pitches,
                              NCollection_Array1<double> nbTurns)
    {
        int nbParts = diams.Length() - 1;

        if (nbParts != pitches.Length() || nbParts != nbTurns.Length())
        {
            throw new ConstructionErrorException(
                "HelixBRep_BuilderHelix::SetParameters: wrong array dimension");
        }

        NCollection_Array1<double> heights = new(1, nbParts);
        NCollection_Array1<bool> bIsPitches = new(1, nbParts);
        bIsPitches.Init(true);
        int i, ip, @in;
        for (i = 1, ip = pitches.Lower(), @in = nbTurns.Lower(); i <= pitches.Length();
             ++i, ip++, @in++)
        {
            heights[i] = pitches[ip] * nbTurns[@in];
        }

        SetParameters(axis, diams, heights, pitches, bIsPitches);
    }

    /// <summary>Sets the parameters of a pure helix by pitches and numbers of turns.</summary>
    /// <exception cref="ConstructionErrorException">The arrays' lengths don't match.</exception>
    public void SetParameters(in CoordinateSystem axis,
                              double diam,
                              NCollection_Array1<double> pitches,
                              NCollection_Array1<double> nbTurns)
    {
        int nbParts = pitches.Length();

        if (nbParts != nbTurns.Length())
        {
            throw new ConstructionErrorException(
                "HelixBRep_BuilderHelix::SetParameters: wrong array dimension");
        }

        NCollection_Array1<double> heights = new(1, nbParts);
        NCollection_Array1<bool> bIsPitches = new(1, nbParts);
        bIsPitches.Init(true);
        int i, ip, @in;
        for (i = 1, ip = pitches.Lower(), @in = nbTurns.Lower(); i <= pitches.Length();
             ++i, ip++, @in++)
        {
            heights[i] = pitches[ip] * nbTurns[@in];
        }

        SetParameters(axis, diam, heights, pitches, bIsPitches);
    }

    /// <summary>Sets the parameters of a pure spiral by pitches and numbers of turns.</summary>
    /// <exception cref="ConstructionErrorException">The arrays' lengths don't match.</exception>
    public void SetParameters(in CoordinateSystem axis,
                              double diam1,
                              double diam2,
                              NCollection_Array1<double> pitches,
                              NCollection_Array1<double> nbTurns)
    {
        int nbParts = pitches.Length();

        if (nbParts != nbTurns.Length())
        {
            throw new ConstructionErrorException(
                "HelixBRep_BuilderHelix::SetParameters: wrong array dimension");
        }

        NCollection_Array1<double> heights = new(1, nbParts);
        NCollection_Array1<bool> bIsPitches = new(1, nbParts);
        bIsPitches.Init(true);
        int i, ip, @in;
        for (i = 1, ip = pitches.Lower(), @in = nbTurns.Lower(); i <= pitches.Length();
             ++i, ip++, @in++)
        {
            heights[i] = pitches[ip] * nbTurns[@in];
        }

        SetParameters(axis, diam1, diam2, heights, pitches, bIsPitches);
    }

    /// <summary>Sets the approximation's tolerance, maximum degree and continuity.</summary>
    public void SetApproxParameters(double aTolerance, int aMaxDegree, Continuity aCont)
    {
        _tolerance = aTolerance;
        _maxDegree = aMaxDegree;
        _continuity = aCont;
    }

    /// <summary>The tolerance reached by the approximation.</summary>
    public double ToleranceReached() => _tolReached;

    /// <summary>Builds the helix wire.</summary>
    public void Perform()
    {
        if (_errorStatus != 0)
        {
            return;
        }

        int i;

        _tolReached = 0.0;
        _shape.Nullify();
        BrepBuilder bb = new();
        // aBB.MakeCompound(TopoDS::Compound(myShape));

        NCollection_List<Shape> lst = new();

        Axis3D axis = _axis3.Axis;
        Point3D pStart = _axis3.Location;
        pStart = pStart.Translated(.5 * _diams!.Value(1) * new Vector3D(_axis3.XDirection));
        bool bIsClockwise = _axis3.IsDirect;

        double height;
        double pitch;
        double taperAngle;

        for (i = 1; i <= _nParts; ++i)
        {
            height = _heights!.Value(i);

            if (_isPitches!.Value(i))
            {
                pitch = _pitches!.Value(i);
            }
            else
            {
                pitch = height / _pitches!.Value(i);
            }

            taperAngle = Math.Atan(.5 * (_diams.Value(i + 1) - _diams.Value(i)) / height);

            Wire part = new();
            bb.MakeWire(ref part);

            BuildPart(axis, pStart, height, pitch, taperAngle, bIsClockwise, ref part);
            if (_errorStatus != 0)
            {
                return;
            }

            Exploration.Vertices(part, out Vertex v1, out Vertex v2);

            pStart = Tool.Pnt(v2);

            axis = axis.WithLocation(axis.Location.Translated(height * new Vector3D(axis.Direction)));

            lst.Append(part);
            // aBB.Add(myShape, aPart);
        }

        Smoothing(lst);

        NCollection_List<Shape>.Iterator it = new(lst);
        MakeWire mkWire = new(ShapeCasts.Wire(lst.First()));
        it.Next();
        for (; it.More(); it.Next())
        {
            mkWire.Add(ShapeCasts.Wire(it.Value()));
        }

        _shape = mkWire.InternalShape();
    }

    private void BuildPart(in Axis3D axis,
                           in Point3D pStart,
                           double height,
                           double pitch,
                           double taperAngle,
                           bool bIsClockwise,
                           ref Wire part)
    {
        if (_errorStatus != 0)
        {
            return;
        }

        _errorStatus = 0;
        _warningStatus = 0;

        // 1. check & prepare data
        double tolPrec, dist, dm, twoPI, c1, t2, t1, t0;
        // Initialize tolerance and angular precision
        tolPrec = _tolerance;
        double tolAng = 1.0e-7;
        // Validate input parameters
        if (taperAngle > M_PI / 2.0 - tolAng)
        {
            _errorStatus = 13; // invalid TaperAngle value
            return;
        }
        if (height < tolPrec)
        {
            _errorStatus = 12; // invalid Height value
            return;
        }
        if (pitch < tolPrec)
        {
            _errorStatus = 11; // invalid Pitch value
            return;
        }
        //
        Line3D lin = new(axis);
        dist = lin.DistanceTo(pStart);
        if (dist < tolPrec)
        {
            _errorStatus = 10; // myPStart belongs to the myAxis
            return;
        }

        tolAng = tolPrec / dist;
        double angleStart = 0.0;

        // const bool bIsOutWard = theTaperAngle > 0.0;
        Direction3D dir = axis.Direction;
        Vector3D vec1 = new(dir);
        Point3D m0 = axis.Location;
        Vector3D vec = new(m0, pStart);
        dm = vec1.Dot(vec);
        Point3D m1 = m0.Translated(dm * vec1);
        Vector3D vecX = new(m1, pStart);
        Direction3D dirX = new(vecX);
        Frame3D ax2 = new(m1, dir, dirX);
        //
        twoPI = 2.0 * M_PI;
        c1 = pitch / twoPI;
        t0 = 0.0;
        t1 = angleStart;

        t2 = height / c1;
        //
        // 2. compute
        bool bIsDone;
        int iErr, nbC, i;
        GeometryBuilderHelix bh = new();
        Point3D p1, p2;
        BrepBuilder bb = new();
        MakeEdge bme = new();
        Vertex v1 = new(), v2 = new();
        // TopoDS_Wire aW;
        Edge e;
        //
        bh.SetPosition(ax2);
        bh.SetCurveParameters(t0, t2, pitch, dist, taperAngle, bIsClockwise);
        bh.SetTolerance(_tolerance);
        bh.SetApproxParameters(_continuity, _maxDegree, _maxSegments);
        //
        bh.Perform();
        iErr = bh.ErrorStatus();
        if (iErr != 0)
        {
            _errorStatus = 2;
            return;
        }
        //
        bb.MakeWire(ref part);
        //
        _tolReached = Math.Max(_tolReached, bh.ToleranceReached());
        NCollection_Sequence<Curve3D> sc = new();
        sc.Assign(bh.InternalCurves());
        if (t1 < 0.0)
        {
            GeometryBuilderHelix bh1 = new();
            bh1.SetPosition(ax2);
            bh1.SetCurveParameters(t1, t0, pitch, dist, taperAngle, bIsClockwise);
            bh1.SetTolerance(_tolerance);
            bh1.SetApproxParameters(_continuity, _maxDegree, _maxSegments);
            //
            bh1.Perform();
            iErr = bh1.ErrorStatus();
            if (iErr != 0)
            {
                _errorStatus = 2;
                return;
            }

            _tolReached = Math.Max(_tolReached, bh1.ToleranceReached());
            NCollection_Sequence<Curve3D> sc1 = bh1.InternalCurves();
            int nbc = sc1.Length();
            for (i = nbc; i >= 1; i--)
            {
                sc.Prepend(sc1.Value(i));
            }
        }

        nbC = sc.Length();
        for (i = 1; i <= nbC; ++i)
        {
            Curve3D c = sc[i];
            //
            if (i == 1)
            {
                if (t1 > 0.0)
                {
                    t2 = c.LastParameter;
                    TrimmedCurve3D ct = new(c, t1, t2);
                    c = ct;
                }
                t1 = c.FirstParameter;
                c.D0(t1, out p1);
                bb.MakeVertex(ref v1, p1, _tolReached);
                v1.Orient(Forward);
            }
            //
            t2 = c.LastParameter;
            c.D0(t2, out p2);
            bb.MakeVertex(ref v2, p2, _tolReached);
            v2.Orient(Reversed);
            //
            bme.Init(c, v1, v2);
            bIsDone = bme.IsDone;
            if (!bIsDone)
            {
                _errorStatus = 3;
                return;
            }
            e = bme.InternalEdge();
            bb.UpdateEdge(e, _tolReached);
            bb.Add(part, e);

            //
            v1 = v2;
            v1.Orient(Forward);
        }
        //
        if (_tolReached > _tolerance)
        {
            _warningStatus = 1;
        }
    }

    private void Smoothing(NCollection_List<Shape> parts)
    {
        if (parts.Extent() == 1)
        {
            return;
        }

        WireExplorer expl = new();

        Wire prevWire = ShapeCasts.Wire(parts.First());
        expl.Init(prevWire);
        Edge prevEdge = new();
        for (; expl.More(); expl.Next())
        {
            prevEdge = expl.Current();
        }

        NCollection_List<Shape>.Iterator iter = new(parts);
        iter.Next();
        for (; iter.More(); iter.Next())
        {
            Wire nextWire = ShapeCasts.Wire(iter.Value());
            expl.Clear();
            expl.Init(nextWire);
            Edge nextEdge = expl.Current();

            // Smoothing curves
            SmoothingEdges(prevEdge, nextEdge);

            for (; expl.More(); expl.Next())
            {
                prevEdge = expl.Current();
            }
        }
    }

    private void SmoothingEdges(in Edge prev, in Edge next)
    {
        const double epsAng = 1.0e-7;
        BrepBuilder bb = new();

        double f1, l1, f2, l2;
        BSplineCurve3D cPrev = (Tool.Curve(prev, out f1, out l1) as BSplineCurve3D)!;

        BSplineCurve3D cNext = (Tool.Curve(next, out f2, out l2) as BSplineCurve3D)!;

        Point3D p1, p2;
        Vector3D v1, v2;

        cPrev.D1(l1, out p1, out v1);
        cNext.D1(f2, out p2, out v2);

        if (v1.AngleTo(v2) < epsAng)
        {
            return;
        }

        v1 = 0.5 * (v1 + v2);
        v2 = v1;

        // OCCT changes the edges' own curves in place, which BRep_Builder froze when it attached them: the port
        // changes copies and puts them on the edges (decision 994). OCCT's curve also keeps the degrees it was
        // raised to when no degree lets the tangent move; the copy is put on the edge in that case too.
        cPrev = (BSplineCurve3D)cPrev.Copy();
        cNext = (BSplineCurve3D)cNext.Copy();

        int errorStatus = 1;
        int degMax = BSplineCurve3D.MaxDegree;
        int deg = cPrev.Degree;
        int i;
        bool bPrevOK = false;
        bool bNextOK = false;

        cPrev.MovePointAndTangent(l1, p1, v1, _tolerance, 1, -1, out errorStatus);
        if (errorStatus != 0)
        {
            for (i = deg + 1; i <= degMax; ++i)
            {
                cPrev.IncreaseDegree(i);
                cPrev.MovePointAndTangent(l1, p1, v1, _tolerance, 1, -1, out errorStatus);
                if (errorStatus == 0)
                {
                    bPrevOK = true;
                    break;
                }
            }
        }
        else
        {
            bPrevOK = true;
        }

        if (bPrevOK || cPrev.Degree != deg)
        {
            bb.UpdateEdge(prev, cPrev, Tool.Tolerance(prev));
        }

        deg = cNext.Degree;
        cNext.MovePointAndTangent(f2, p2, v2, _tolerance, -1, 1, out errorStatus);
        if (errorStatus != 0)
        {
            for (i = deg + 1; i <= degMax; ++i)
            {
                cNext.IncreaseDegree(i);
                cNext.MovePointAndTangent(f2, p2, v2, _tolerance, -1, 1, out errorStatus);
                if (errorStatus == 0)
                {
                    bNextOK = true;
                    break;
                }
            }
        }
        else
        {
            bNextOK = true;
        }

        if (bNextOK || cNext.Degree != deg)
        {
            bb.UpdateEdge(next, cNext, Tool.Tolerance(next));
        }
    }

    /// <summary>The error status (see the class remarks).</summary>
    public int ErrorStatus() => _errorStatus;

    /// <summary>The warning status (see the class remarks).</summary>
    public int WarningStatus() => _warningStatus;

    /// <summary>The helix wire (frozen), or a null shape if it wasn't built.</summary>
    public Shape Shape()
    {
        _shape.TShape()?.FreezeGraph();
        return _shape;
    }

    /// <summary>OCCT's <c>Shape()</c>, without freezing, for ported code.</summary>
    internal Shape InternalShape() => _shape;
}
