using Kerf.Geometry;
using Kerf.Modeling;
using Kerf.Topology;
using static Kerf.Modeling.Operations;

namespace Renegade.BladeBumper;

/// <summary>Figures the tests and the README check: clearances, walls and positions.</summary>
public sealed record Layout
{
    public double XCoil { get; init; }
    public double ZCoilTop { get; init; }
    public double ZChamberTop { get; init; }
    public double XChamberEnd { get; init; }
    public (double X0, double X1) XHousing { get; init; }
    public double XSleeve { get; init; }
    public double DoorL { get; init; }
    public List<(double Y, double Z)> DoorScrews { get; init; } = [];
    public double BootAirTop { get; init; }
    public double BootAirSide { get; init; }
    public double BootAirBottom { get; init; }
    public double PocketLowZ { get; init; }
    public (double X, double Y, double Z) GrooveProbe { get; init; }
    public double SealInnerLand { get; init; }
    public double SealOuterLand { get; init; }
    public double ScrewToGroove { get; init; }
    public double DoorBossWall { get; init; }
    public double WireChannel { get; init; }
    public double BootEndBottom { get; init; }
    public double SocketShaftGap { get; init; }
    public double InsertTopZ { get; init; }
    public double InsertToBore { get; init; }
    public double InsertToDoorScrew { get; init; }
    public double InsertToPocket { get; init; }
    public double LowScrewToSealed { get; init; }
    public double UsbTipX { get; init; }
    public double UsbPlugRoom { get; init; }
    public double WallPocketSocket { get; init; }
    public double WallSocketBore { get; init; }
    public double WallPocketOuter { get; init; }
    public double WallDoor { get; init; }
}

/// <summary>How far the sensor stands off the shaft.</summary>
public sealed record Protrusion(double Stern, double Under, double WidthOverBumpers);

public static partial class Bumper
{
    // ---------------------------------------------------------------- bought parts and context

    /// <summary>Across a sled, between its rails: (u0, u1).</summary>
    static (double U0, double U1) Channel() => (P.SLED_U.Lo + P.SLED_RAIL_W, P.SLED_U.Hi - P.SLED_RAIL_W);

    static double SledTop() => P.N_FLOOR + P.SLED_CLR + P.SLED_T;

    /// <summary>Lengthwise on its sled: 22.1 along the door, on pads at its ends; header rows at those ends.</summary>
    static Part Shuttle(int s, double x0, double x1)
    {
        var (c0, c1) = Channel();
        var uc = (c0 + c1) / 2;
        double u0 = uc - P.SHUTTLE_W / 2, u1 = uc + P.SHUTTLE_W / 2;
        var n0 = SledTop() + P.SHUTTLE_LIFT + P.BOND;
        var outPart = FBox(s, x0, x1, u0, u1, n0, n0 + P.SHUTTLE_T);
        var xc = (x0 + x1) / 2;
        outPart += FBox(s, xc - 1.5, xc + 1.5, uc - 1.5, uc + 1.5, n0 - P.SHUTTLE_CHIP, n0);
        foreach (var k in new[] { -1, 1 })
        {
            var xr = xc + k * P.SHUTTLE_ROW / 2;
            outPart += FBox(s, xr - 1.3, xr + 1.3, u0 + 1, u1 - 1, n0 + P.SHUTTLE_T, n0 + P.SHUTTLE_T + P.SHUTTLE_PINS);
        }
        return outPart;
    }

    /// <summary>A cap screw pointing up, its head's top face at zSeat.</summary>
    static Part VBolt(double x, double y, double zSeat, double headD, double headH, double d, double length) =>
        new Pos(x, y, zSeat - headH) * new Cylinder(headD / 2, headH, align: MIN) +
        new Pos(x, y, zSeat) * new Cylinder(d / 2 - 0.05, length, align: MIN);

    static readonly PerSize<IReadOnlyDictionary<string, Part>> components = new(() =>
    {
        var lay = Sleds();
        var xs = lay.Xiao.Boards.Concat(lay.Bmm.Boards).ToDictionary();
        var n0 = P.N_FLOOR + P.TAPE;
        var o = new Dictionary<string, Part>();
        var pocketEnd = P.BODY_L - P.FRONT_WALL;
        var pu = P.POCKET_U.Lo;
        o["battery"] = FBox(-1, lay.BatX0, pocketEnd, pu + 0.3, pu + 0.3 + P.BAT_W, P.N_FLOOR + P.BAT_LIFT, P.N_FLOOR + P.BAT_LIFT + P.BAT_H);
        o["ldc"] = FBox(1, pocketEnd - P.LDC_L, pocketEnd, pu + 0.5, pu + 0.5 + P.LDC_W, n0, n0 + P.LDC_H);

        var (c0, c1) = Channel();
        var nb = SledTop() + P.BOND;
        var (x0, x1) = xs["xiao"];
        var u0 = (c0 + c1) / 2 - P.XIAO_W / 2;
        var xiao = FBox(1, x0, x1, u0, u0 + P.XIAO_W, nb, nb + P.XIAO_T);
        xiao += FBox(1, x0 + 2, x1 - 6, u0 + 1, u0 + P.XIAO_W - 1, nb + P.XIAO_T, nb + P.XIAO_T + P.XIAO_TOP);
        var uc = u0 + P.XIAO_W / 2;
        xiao += FBox(1, x1 - P.USB_IN, P.USB_TIP_X, uc - P.USB_W / 2, uc + P.USB_W / 2, nb + P.XIAO_T, nb + P.XIAO_T + P.USB_H);
        o["xiao"] = xiao;
        o["bmi330"] = Shuttle(1, xs["bmi330"].X0, xs["bmi330"].X1);
        o["bmm350"] = Shuttle(-1, xs["bmm350"].X0, xs["bmm350"].X1);
        (x0, x1) = xs["flash"];
        u0 = (c0 + c1) / 2 - P.FLASH_W / 2;
        o["flash"] = FBox(-1, x0, x1, u0, u0 + P.FLASH_W, nb, nb + P.FLASH_H);
        var zr = P.Z_TOP_FLOOR + P.TAPE;
        o["regulator"] = new Pos(P.REG_X - P.REG_L / 2, 0, zr + P.REG_H / 2) * new Box(P.REG_L, P.REG_W, P.REG_H);
        x1 = xs["xiao"].X1 - 1.5;
        o["antenna"] = new Pos(x1 - P.ANT_L / 2, 0, P.Z_ROOF - 0.05 - P.ANT_T / 2) * new Box(P.ANT_L, P.ANT_W, P.ANT_T);

        double za = ZArm(), xt = XTip();
        Part tube = new Cylinder(P.ARM_D / 2, xt - P.RIM_T, align: MIN) - new Cylinder(P.ARM_ID / 2, xt - P.RIM_T, align: MIN);
        o["arm"] = new Pos(P.RIM_T, 0, za) * new Rotation(0, 90, 0) * tube;
        double zc = ZCoilBottom(), xc = XCoil();
        Part coil = new Pos(xc, 0, zc) * new Cylinder(P.COIL_HEAD_D / 2, P.COIL_T, align: MIN);
        coil += new Pos(xc - P.COIL_TAIL / 2, 0, zc + P.COIL_T / 2) * new Box(P.COIL_TAIL, P.COIL_TAIL_W, P.COIL_T);
        o["coil"] = coil;
        o["plate"] = Plate();
        var xr0 = P.BODY_L + P.RUBBER_FRONT - P.RUBBER_L;
        o["shaft"] = Cone(XSleeve() - 60, XHousing().X1 + 60);
        o["rubber"] = Ex(Circle2(BodyBoreR()), xr0, xr0 + P.RUBBER_L) - Cone(xr0 - 1, xr0 + P.RUBBER_L + 1);
        o["sleeve"] = new Pos(XSleeve() - 60, 0, 0) * new Rotation(0, 90, 0) * new Cylinder(P.SLEEVE_R, 60.0, align: MIN) - Cone(XSleeve() - 61, XSleeve() + 1);
        foreach (var (i, (y, z)) in DoorScrews().Index())
        {
            Part s = new Pos(P.BODY_L, y, z) * new Rotation(0, 90, 0) * new Cylinder(P.M3_HEAD_D / 2, P.M3_HEAD_H, align: MIN);
            s += new Pos(P.BODY_L - P.M3_L, y, z) * new Rotation(0, 90, 0) * new Cylinder(1.45, P.M3_L, align: MIN);
            o[$"door_screw{i}"] = s;
        }
        foreach (var (i, (x, y)) in ClampBolts().Index())
            o[$"clamp_bolt{i}"] = VBolt(x, y, P.CLAMP_SEAT_Z, P.M3_HEAD_D, P.M3_HEAD_H, 3.0, P.CLAMP_BOLT_L);
        foreach (var (i, (x, y)) in HousingBolts().Index())
        {
            o[$"housing_bolt{i}"] = VBolt(x, y, P.HOUSING_SEAT_Z, P.M3_HEAD_D, P.M3_HEAD_H, 3.0, P.HOUSING_BOLT_L);
            var nut = ExZ(Hexagon(P.M3_NUT_AF), P.M3_NUT_H) - new Cylinder(1.5, 20);
            o[$"housing_nut{i}"] = new Pos(x, y, P.HOUSING_NUT_Z + 0.05) * nut;
            o[$"housing_plug{i}"] = HousingPlug(i);
        }
        return o;
    });

    /// <summary>The bought parts, the shaft and its rubber, and the four glued plugs, in assembly position.</summary>
    public static IReadOnlyDictionary<string, Part> Components => components.Value;

    static readonly PerSize<Layout> layout = new(() =>
    {
        var (w, z0, z1) = BootInner();
        var za = ZArm();
        var pocketTop = Yz(P.N_FLOOR, P.POCKET_U.Hi);
        var lowZ = Yz(P.N_FLOOR, P.POCKET_U.Lo).Z;
        var insertTop = -P.SPLIT_Z + P.M3_INSERT_HOLE;
        var pocketLow = Yz(P.N_FLOOR, P.POCKET_U.Lo);
        var usbMid = SledTop() + P.BOND + P.XIAO_T + P.USB_H / 2;
        var flange = FlangeOutline().Faces()[0].OuterWire();
        var groove = GrooveFace().Faces()[0];
        return new Layout
        {
            XCoil = XCoil(),
            ZCoilTop = ZCoilBottom() + P.COIL_T,
            ZChamberTop = ZChamberTop(),
            XChamberEnd = XChamberEnd(),
            XHousing = XHousing(),
            XSleeve = XSleeve(),
            DoorL = DoorLen(),
            DoorScrews = DoorScrews(),
            BootAirTop = z1 - za - P.ARM_D / 2,
            BootAirSide = (w - P.ARM_D) / 2,
            BootAirBottom = za - P.ARM_D / 2 - z0,
            PocketLowZ = lowZ,
            GrooveProbe = (0.0, 0.0, P.Z_TOP_FLOOR - P.SEAL_LAND - P.GROOVE_W / 2),
            SealInnerLand = (P.N_FLOOR - P.SEAL_LAND - P.GROOVE_W) - BodyBoreR(),
            SealOuterLand = (P.N_ROOF + P.WALL + P.FLANGE_RAISE) - (P.N_ROOF + P.SEAL_LAND + P.GROOVE_W),
            ScrewToGroove = DoorScrews().Min(s => groove.DistanceTo(new Point3D(s.Y, s.Z, 0))) - P.M3_INSERT_D / 2,
            DoorBossWall = DoorScrews().Min(s => flange.DistanceTo(new Point3D(s.Y, s.Z, 0))) - P.DOOR_BOSS_R,
            WireChannel = P.TOP_SCREW.Z - P.SCREW_CLEAR - P.Z_TOP_FLOOR,
            BootEndBottom = z0 - P.BOOT_END_FLOOR,
            SocketShaftGap = SocketArcR() - ShaftR(XHousing().X0),
            InsertTopZ = insertTop,
            InsertToBore = P.CLAMP_Y - P.M3_INSERT_D / 2 - BodyBoreR(),
            InsertToDoorScrew = InsertToLowScrew(),
            InsertToPocket = Math.Sqrt(Math.Pow(pocketLow.Y - P.CLAMP_Y, 2) + Math.Pow(pocketLow.Z - insertTop, 2)) - P.M3_INSERT_D / 2,
            LowScrewToSealed = SealedFace().Faces()[0].DistanceTo(new Point3D(P.LOW_SCREW.Y, P.LOW_SCREW.Z, 0)),
            UsbTipX = P.USB_TIP_X,
            UsbPlugRoom = 2 * Math.Min(P.N_ROOF - usbMid, usbMid - P.N_FLOOR),
            WallPocketSocket = Math.Sqrt(pocketTop.Y * pocketTop.Y + Math.Pow(pocketTop.Z - za, 2)) - P.SOCKET_D / 2,
            WallSocketBore = za - P.SOCKET_D / 2 - BodyBoreR(),
            WallPocketOuter = P.N_ROOF + P.WALL - P.POCKET_N1,
            WallDoor = P.WALL,
        };
    });

    public static Layout Layout => layout.Value;

    public static Protrusion Protrusion() => new(
        Stern: BodyTopZ() + P.BUMPER_T + P.BUMPER_CLR - P.SHAFT_D / 2,
        Under: P.CLAMP_BOTTOM - P.SHAFT_D / 2,
        WidthOverBumpers: Off(ArchHull(), P.WALL + P.BUMPER_T + P.BUMPER_CLR).Bounds().Size.X);

    // ---------------------------------------------------------------- outputs

    /// <summary>A part rotated into its print orientation, sitting on z = 0.</summary>
    public static Part PrintPose(string name, Part part)
    {
        if (name is "body" or "body_bumper" or "door_bumper" or "adapter")
            part = new Rotation(0, -90, 0) * part;   // low x end down
        else if (name is "door" or "housing" or "boot" or "boot_tower")
            part = new Rotation(0, 90, 0) * part;    // high x end down: the door's mouth, the housing's far end
        else if (name == "saddle")
            part = new Rotation(180, 0, 0) * part;   // plate face down
        // the boot and its tower print together: both placed by the tower, which surrounds the boot
        var bb = (name is "boot" or "boot_tower" ? new Rotation(0, 90, 0) * BootTower() : part).Bounds();
        return new Pos(-bb.Center().X, -bb.Center().Y, -bb.Min.Z) * part;
    }

    public static readonly string[] TpuParts = ["door_bumper", "body_bumper", "boot", "housing", "housing_plug"];

    /// <summary>Grams at 100 % infill: ABS 1.05, TPU 1.21 g/cm^3.</summary>
    public static Dictionary<string, double> Masses() =>
        Parts.ToDictionary(kv => kv.Key, kv => kv.Value.Volume / 1000 * (TpuParts.Contains(kv.Key) ? 1.21 : 1.05));

    /// <summary>Plastic between a clamp insert's hole (upright, under the body) and a lower door screw's hole (along x).</summary>
    static double InsertToLowScrew()
    {
        var (y, z) = P.LOW_SCREW;
        double z0 = -P.SPLIT_Z, z1 = -P.SPLIT_Z + P.M3_INSERT_HOLE;
        return Math.Sqrt(Math.Pow(y - P.CLAMP_Y, 2) + Math.Pow(z - Math.Clamp(z, z0, z1), 2)) - P.M3_HOLE / 2 - P.M3_INSERT_D / 2;
    }
}
